"""COPC -> official DALES SPT inference -> wall-line fitting -> final vectors.

This is an application wrapper. It deliberately does not modify src/ or configs/.
Model inference follows notebooks/superpoint_transformer_tutorial.ipynb.
"""
from __future__ import annotations

import argparse
import ast
from collections import Counter
import gc
import hashlib
import json
import logging
import math
import os
from pathlib import Path
import random
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
LOG = logging.getLogger("copc_buildings")
WRAPPER_VERSION = 1
# Compatibility for the exact pre-integration wrapper and dependency tree.
# Export-only changes must not force a repeat of successful SPT predictions.
LEGACY_CACHE_SOURCE = "ea3a07ee635151f40a2dfd6b87e5267d8e1d365a4d869053b40c2d3ba6e5425f"
LEGACY_CACHE_DEPENDENCIES = "109647fd8bc8e21b34774cec61a86796a8a670ba6d360d7ef8b8face0153a3cf"


def write_json(path, value):
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(tmp, path)


def file_hash(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def dependency_hash():
    """Fingerprint all original model, preprocessing and configuration code."""
    digest = hashlib.sha256()
    paths = sorted(ROOT.joinpath("src").rglob("*.py"))
    paths += sorted(ROOT.joinpath("configs").rglob("*.yaml"))
    for path in paths:
        digest.update(path.relative_to(ROOT).as_posix().encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def source_hash():
    """Hash inference code, excluding vector export and fitting settings."""
    digest = hashlib.sha256(dependency_hash().encode())
    names = {"OfficialPredictor", "predict_tile", "make_grid", "horizontal_crs", "grid_tiles",
             "tile_bounds", "cache_path", "load_cache", "save_cache"}
    tree = ast.parse(Path(__file__).read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in names:
            digest.update(ast.dump(node, include_attributes=False).encode())
    return digest.hexdigest()


def compatible_cache(previous, current):
    if previous == current:
        return True
    old, new = dict(previous), dict(current)
    old_source = old.pop("source_sha256", None)
    new.pop("source_sha256", None)
    return (old == new and old_source == LEGACY_CACHE_SOURCE and
            dependency_hash() == LEGACY_CACHE_DEPENDENCIES)


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=ROOT / "testdata/pointcloud/AHN4_C_155000_463000.COPC.LAZ")
    parser.add_argument("--checkpoint", type=Path, default=ROOT / "spt-2_dales.ckpt")
    parser.add_argument("--output", type=Path, default=ROOT / "testdata/testresult")
    parser.add_argument("--run-name", help="Output basename; choose a new name when changing inference settings.")
    parser.add_argument("--tile-size", type=float, default=50.0, help="Core square size in source CRS metres.")
    parser.add_argument("--halo", type=float, default=10.0, help="Extra context on every side, discarded after prediction.")
    parser.add_argument("--resolution", type=float, default=0.5, help="Vectorization grid; SPT voxel remains 0.1 m.")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--device", default="cuda:0", help="CUDA device used by the original GPU transforms.")
    parser.add_argument("--epsg", type=int, help="Explicit horizontal CRS if LAS metadata has none.")
    parser.add_argument("--max-tiles", type=int, default=0, help="Predict at most this many NEW tiles, then stop without exporting partial vectors.")
    parser.add_argument("--vector-only", action="store_true", help="Fit and export final vectors from a COMPLETE tile cache; no input vector required.")
    parser.add_argument("--fit-tolerance", type=float, default=2.0, help="Outer/inner ring fitting: maximum boundary offset in metres.")
    parser.add_argument("--fit-max-area-change", type=float, default=0.20, help="Maximum relative area/coverage change during wall fitting.")
    parser.add_argument("--fit-merge-angle", type=float, default=35.0, help="Angle in degrees for merging adjacent fitted wall chains.")
    parser.add_argument("--simplify", type=float, default=0.0, help="Optional simplification before automatic wall-line fitting, in metres.")
    parser.add_argument("--close-radius", type=float, default=0.0, help="Optional raster closing radius in metres; 0 disables it.")
    parser.add_argument("--fill-holes-area", type=float, default=100.0, help="Remove unoccupied holes up to this area in m2; fit remaining larger inner rings. 0 disables hole filling.")
    parser.add_argument("--min-area", type=float, default=20.0, help="Remove building components with net area below this m2 threshold BEFORE hole filling/fitting; 0 disables removal.")
    parser.add_argument("--max-raster-cells", type=int, default=100_000_000, help="Refuse larger whole-area vectorization rasters rather than exhausting RAM.")
    args = parser.parse_args()
    for key in ("input", "checkpoint", "output"):
        setattr(args, key, getattr(args, key).resolve())
    for key in ("tile_size", "resolution"):
        if not math.isfinite(getattr(args, key)) or getattr(args, key) <= 0:
            parser.error(f"--{key.replace('_', '-')} must be positive and finite")
    for key in ("halo", "simplify", "close_radius", "fill_holes_area", "min_area"):
        if not math.isfinite(getattr(args, key)) or getattr(args, key) < 0:
            parser.error(f"--{key.replace('_', '-')} must be nonnegative and finite")
    if args.max_tiles < 0 or args.max_raster_cells <= 0:
        parser.error("Invalid tile/raster limit")
    if not math.isfinite(args.fit_tolerance) or args.fit_tolerance <= 0:
        parser.error("--fit-tolerance must be positive and finite")
    if not math.isfinite(args.fit_max_area_change) or not 0 < args.fit_max_area_change < 1:
        parser.error("--fit-max-area-change must be between 0 and 1")
    if not math.isfinite(args.fit_merge_angle) or not 0 < args.fit_merge_angle < 90:
        parser.error("--fit-merge-angle must be between 0 and 90")
    ratio = args.tile_size / args.resolution
    if not math.isclose(ratio, round(ratio), abs_tol=1e-8):
        parser.error("--tile-size must be an integer multiple of --resolution")
    if not args.device.startswith("cuda"):
        parser.error("This wrapper preserves the original CUDA preprocessing; use --device cuda:0 (or another CUDA index)")
    if not args.input.is_file() or not args.checkpoint.is_file():
        parser.error("Input COPC and checkpoint files must both exist")
    args.run_name = args.run_name or args.input.stem.replace(".", "_") + "_spt_dales"
    if Path(args.run_name).name != args.run_name or args.run_name in (".", ".."):
        parser.error("--run-name must be a basename, without directories")
    return args


def setup_logging(work):
    work.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s",
                        handlers=[logging.StreamHandler(sys.stdout), logging.FileHandler(work / "prediction.log", encoding="utf-8")])


def make_grid(header, resolution, max_cells):
    x0 = math.floor(float(header.mins[0]) / resolution) * resolution
    ytop = math.ceil(float(header.maxs[1]) / resolution) * resolution
    # Include points exactly at source minima/maxima as well.
    width = int(math.floor((float(header.maxs[0]) - x0) / resolution)) + 1
    height = int(math.floor((ytop - float(header.mins[1])) / resolution)) + 1
    if width * height > max_cells:
        raise ValueError(f"Raster has {width * height:,} cells, above --max-raster-cells={max_cells:,}. Split the survey into separate runs or explicitly raise this RAM safeguard.")
    return {"x0": x0, "ytop": ytop, "width": width, "height": height, "resolution": resolution}


def horizontal_crs(header, epsg):
    from pyproj import CRS
    crs = CRS.from_epsg(epsg) if epsg else header.parse_crs()
    if crs is None:
        raise ValueError("Input lacks CRS metadata. Specify --epsg for the actual horizontal CRS.")
    if crs.is_compound:
        crs = next((c for c in crs.sub_crs_list if c.is_projected), None)
    if crs is None or not crs.is_projected:
        raise ValueError("Use a projected horizontal CRS in metres, not longitude/latitude.")
    if any(not math.isclose(axis.unit_conversion_factor, 1.0, abs_tol=1e-8) for axis in crs.axis_info[:2]):
        raise ValueError("Source horizontal coordinates must be in metres; reproject the point cloud first.")
    return crs


def grid_tiles(grid, tile_size):
    size = int(round(tile_size / grid["resolution"]))
    tid = 0
    for row in range(0, grid["height"], size):
        for col in range(0, grid["width"], size):
            h = min(size, grid["height"] - row)
            w = min(size, grid["width"] - col)
            yield {"id": tid, "row": row, "col": col, "height": h, "width": w}
            tid += 1


def tile_bounds(grid, tile):
    r = grid["resolution"]
    xmin = grid["x0"] + tile["col"] * r
    ymax = grid["ytop"] - tile["row"] * r
    return xmin, ymax - tile["height"] * r, xmin + tile["width"] * r, ymax


def cache_path(work, tile):
    return work / "tiles" / f"tile_{tile['id']:06d}.npz"


def load_cache(path, tile):
    import numpy as np
    with np.load(path, allow_pickle=False) as data:
        mask = data["mask"]
        if mask.shape != (tile["height"], tile["width"]) or mask.dtype != np.uint8:
            raise ValueError(f"Invalid tile cache: {path}")
        if not np.isin(mask, [0, 1, 2]).all():
            raise ValueError(f"Invalid mask values: {path}")
        metadata = json.loads(str(data["metadata"].item()))
        if metadata["tile"] != tile or not (0 <= metadata["building_points"] <= metadata["core_points"] <= metadata["context_points"]):
            raise ValueError(f"Invalid tile metadata: {path}")
        return mask.copy(), metadata


def save_cache(path, mask, metadata):
    import numpy as np
    tmp = path.with_suffix(".tmp")
    with tmp.open("wb") as stream:
        np.savez_compressed(stream, mask=mask, metadata=json.dumps(metadata))
    os.replace(tmp, path)


def check_vector_dependencies(args):
    """Fail before lengthy inference if final export dependencies are absent."""
    import rasterio
    import fiona
    from shapely.validation import make_valid
    from building_line_fitting import FitOptions, fit_polygons
    for driver in ("GPKG", "ESRI Shapefile"):
        if "w" not in fiona.supported_drivers.get(driver, ""):
            raise RuntimeError(f"Fiona lacks write support for {driver}")
    if args.close_radius:
        from scipy import ndimage


class OfficialPredictor:
    """Use the author's custom-cloud tutorial without changing network/transforms."""

    def __init__(self, args, work):
        import torch
        import hydra
        from src.utils import init_config
        from src.transforms import instantiate_datamodule_transforms
        self.torch = torch
        self.args = args
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA is unavailable. Activate the laboratory SPT CUDA environment first.")
        self.device = torch.device(args.device)
        torch.cuda.set_device(self.device)
        # DataTo('cuda') in the unchanged pre_transform uses the selected device.
        self.cfg = init_config(config_name="eval.yaml", overrides=["experiment=semantic/dales"])
        from omegaconf import OmegaConf
        # Keep unrelated Hydra runtime paths unresolved (compose has no Trainer
        # run directory); model/transform values resolve at instantiation time.
        (work / "official_config.yaml").write_text(OmegaConf.to_yaml(self.cfg, resolve=False), encoding="utf-8")
        self.transforms = instantiate_datamodule_transforms(self.cfg.datamodule)
        for key in ("pre_transform", "on_device_test_transform"):
            if key not in self.transforms:
                raise RuntimeError(f"Official transform was not instantiated: {key}")
        model = hydra.utils.instantiate(self.cfg.model)
        # Official compatibility hooks handle old published SPT checkpoints.
        # Strict loading rejects missing/unexpected learned parameters.
        self.model = model._load_from_checkpoint(str(args.checkpoint), map_location="cpu", strict=True)
        if self.model.num_classes != 8:
            raise ValueError("Expected the 8-class DALES standard SPT checkpoint")
        self.model = self.model.eval().to(self.device)
        from src.datasets.dales_config import CLASS_NAMES
        self.building_id = CLASS_NAMES.index("Buildings")
        if self.building_id != 7:
            raise ValueError("Unexpected DALES training class mapping")
        LOG.info("Strict checkpoint load complete. Device=%s; building training ID=%d (not LAS class 6).", torch.cuda.get_device_name(self.device), self.building_id)

    def predict(self, xyz, intensity, tile_id):
        import numpy as np
        from src.data import Data
        from src.transforms import NAGRemoveKeys
        torch = self.torch
        seed = (self.args.seed + tile_id) % (2 ** 32)
        random.seed(seed)
        np.random.seed(seed)
        torch.manual_seed(seed)
        torch.cuda.manual_seed_all(seed)
        # Translate in float64 before converting local positions to float32.
        # No scaling of XYZ and no height/intensity fitting to the new region.
        local_xyz = (xyz - xyz[0]).astype(np.float32)
        data = Data(pos=torch.from_numpy(local_xyz),
                    intensity=torch.from_numpy(np.clip(intensity.astype(np.float32), 0, 60000) / 60000))
        # Existing LAS classifications are deliberately NOT model inputs/targets.
        nag = self.transforms["pre_transform"](data)
        # The original SaveNodeIndex + GridSampling3D retain the raw->voxel map.
        raw_to_voxel = nag[0].sub.to_super_index().cpu().long()
        if raw_to_voxel.numel() != len(xyz):
            raise RuntimeError("Official raw-to-voxel mapping does not cover the complete input tile")
        nag = NAGRemoveKeys(level=0, keys=[k for k in nag[0].keys if k not in self.cfg.datamodule.point_load_keys])(nag)
        nag = NAGRemoveKeys(level="1+", keys=[k for k in nag[1].keys if k not in self.cfg.datamodule.segment_load_keys])(nag)
        if self.transforms.get("test_transform") is not None:
            nag = self.transforms["test_transform"](nag)
        nag = self.transforms["on_device_test_transform"](nag.to(self.device))
        with torch.no_grad():
            output = self.model(nag)
            # Same argmax + label broadcast as the author's inference helpers.
            voxel_labels = output.voxel_semantic_pred(super_index=nag[0].super_index).cpu()
            raw_labels = voxel_labels[raw_to_voxel].numpy().astype(np.uint8)
        return raw_labels


def predict_tile(reader, predictor, grid, tile, args):
    import numpy as np
    from laspy.copc import Bounds
    xmin, ymin, xmax, ymax = tile_bounds(grid, tile)
    lo = np.array([xmin - args.halo, ymin - args.halo, reader.header.mins[2]], dtype=np.float64)
    hi = np.array([xmax + args.halo, ymax + args.halo, reader.header.maxs[2]], dtype=np.float64)
    LOG.info("Tile %06d: full-resolution COPC query, core=[%.2f %.2f %.2f %.2f]", tile["id"], xmin, ymin, xmax, ymax)
    # No resolution/level filter: never silently predict on a coarse COPC LOD.
    points = reader.query(bounds=Bounds(lo, hi))
    xyz = np.column_stack((points.x, points.y, points.z))
    intensity = np.asarray(points.intensity).copy()
    del points
    if len(xyz) and not np.isfinite(xyz).all():
        raise ValueError(f"Non-finite input coordinates in tile {tile['id']}")
    r = grid["resolution"]
    rows = np.floor((grid["ytop"] - xyz[:, 1]) / r).astype(np.int64)
    cols = np.floor((xyz[:, 0] - grid["x0"]) / r).astype(np.int64)
    core = ((rows >= tile["row"]) & (rows < tile["row"] + tile["height"]) &
            (cols >= tile["col"]) & (cols < tile["col"] + tile["width"]))
    mask = np.zeros((tile["height"], tile["width"]), dtype=np.uint8)
    metadata = {"tile": tile, "context_points": len(xyz), "core_points": int(core.sum()), "building_points": 0}
    if not core.any():
        return mask, metadata
    LOG.info("Tile %06d: %s context points -> original preprocessing and prediction", tile["id"], f"{len(xyz):,}")
    labels = predictor.predict(xyz, intensity, tile["id"])
    local_row, local_col = rows[core] - tile["row"], cols[core] - tile["col"]
    mask[local_row, local_col] = 1  # observed, but no predicted building point yet
    building = labels[core] == predictor.building_id
    mask[local_row[building], local_col[building]] = 2
    metadata["building_points"] = int(building.sum())
    return mask, metadata


def write_raster(path, array, grid, crs, nodata=None):
    import rasterio
    from rasterio.transform import from_origin
    tmp = path.with_name(path.stem + ".tmp.tif")
    with rasterio.open(tmp, "w", driver="GTiff", width=grid["width"], height=grid["height"],
                       count=1, dtype=array.dtype, crs=crs.to_wkt(),
                       transform=from_origin(grid["x0"], grid["ytop"], grid["resolution"], grid["resolution"]),
                       compress="deflate", tiled=True, BIGTIFF="IF_SAFER", nodata=nodata) as dst:
        dst.write(array, 1)
    os.replace(tmp, path)


def export_vectors(args, work, tiles, grid, crs):
    import numpy as np
    import rasterio.features
    from rasterio.transform import from_origin
    import fiona
    from shapely.geometry import shape, mapping
    from shapely.validation import make_valid
    LOG.info("All tile caches are present. Building the whole-area raster before polygonization.")
    # Disk-backed global mosaic avoids retaining point sets/graphs for the full area.
    mosaic = np.memmap(work / "coverage.uint8", mode="w+", dtype=np.uint8,
                       shape=(grid["height"], grid["width"]))
    mosaic[:] = 0
    core_points = building_points = 0
    for tile in tiles:
        mask, meta = load_cache(cache_path(work, tile), tile)
        rr, cc = tile["row"], tile["col"]
        mosaic[rr:rr + tile["height"], cc:cc + tile["width"]] = mask
        core_points += meta["core_points"]
        building_points += meta["building_points"]
    expected_count = json.loads((work / "run.json").read_text(encoding="utf-8"))["signature"]["point_count"]
    if core_points != expected_count:
        raise RuntimeError(f"Core-point coverage mismatch: cached {core_points:,}, source {expected_count:,}. Refusing incomplete/duplicated full-area vectors.")
    mosaic.flush()
    prefix = args.output / args.run_name
    raw = np.asarray(mosaic) == 2
    final = raw.copy() if args.close_radius else raw
    if args.close_radius:
        from scipy import ndimage
        if args.close_radius:
            radius = int(math.ceil(args.close_radius / grid["resolution"]))
            yy, xx = np.ogrid[-radius:radius + 1, -radius:radius + 1]
            footprint = (xx * xx + yy * yy) * grid["resolution"] ** 2 <= args.close_radius ** 2 + 1e-10
            pad = radius + 1
            padded = np.pad(final, pad, mode="edge")
            final = ndimage.binary_closing(padded, structure=footprint)[pad:-pad, pad:-pad]
    transform = from_origin(grid["x0"], grid["ytop"], grid["resolution"], grid["resolution"])
    schema = {"geometry": "Polygon", "properties": {"bldg_id": "int", "area_m2": "float", "holes": "int", "src_edge": "int", "grid_m": "float",
              "fit_done": "int", "fit_tol": "float", "v_old": "int", "v_new": "int", "holes_rm": "int", "hole_m2": "float", "holes_fit": "int"}}
    gpkg = prefix.with_name(prefix.name + "_buildings.gpkg")
    shp = prefix.with_name(prefix.name + "_buildings.shp")
    # Stage final vector files together; interrupted exports leave caches intact.
    stage = work / "vector_export"
    stage.mkdir(exist_ok=True)
    staged_gpkg, staged_shp = stage / gpkg.name, stage / shp.name
    for old in stage.iterdir():
        if old.is_file() and (old.stem == gpkg.stem or old.name.startswith(gpkg.name + "-")):
            old.unlink()
    total_area = 0.0
    extent = (grid["x0"], grid["ytop"] - grid["height"] * grid["resolution"],
              grid["x0"] + grid["width"] * grid["resolution"], grid["ytop"])
    def polygons(geom):
        if geom.geom_type == "Polygon":
            yield geom
        elif hasattr(geom, "geoms"):
            for child in geom.geoms:
                yield from polygons(child)
    # Keep temporary outlines in memory. Never require/read an input GPKG or
    # write unfitted vector files before fitting the predicted coverage.
    outlines, source_edges = [], []
    small_buildings_removed, small_buildings_area = 0, 0.0
    for geom_json, value in rasterio.features.shapes(final.astype(np.uint8), mask=final, connectivity=4, transform=transform):
        geom = shape(geom_json)
        if not geom.is_valid:
            geom = make_valid(geom)
        for raw_polygon in polygons(geom):
            if raw_polygon.area < args.min_area:
                small_buildings_removed += 1
                small_buildings_area += raw_polygon.area
                continue
            polygon = raw_polygon
            if args.simplify:
                candidate = polygon.simplify(args.simplify, preserve_topology=True)
                if candidate.is_valid and not candidate.is_empty and len(candidate.interiors) == len(polygon.interiors):
                    polygon = candidate
            bounds = raw_polygon.bounds
            source_edges.append(int(any(abs(bounds[k] - extent[k]) < grid["resolution"] * 0.01 for k in range(4))))
            outlines.append(polygon)
    del mosaic, raw, final
    gc.collect()
    from building_line_fitting import FitOptions, fit_polygons, vertex_count
    LOG.info("Prediction complete. Aggressive outer/inner ring fitting v3 on %d outlines; tolerance=%.3fm, fill holes<=%.3fm2, remove buildings<%.3fm2.", len(outlines), args.fit_tolerance, args.fill_holes_area, args.min_area)
    def fit_progress(done, total, report):
        if done % 100 == 0 or done == total:
            LOG.info("Wall-line fitting %d/%d", done, total)
    fit_options = FitOptions(args.fit_tolerance, args.fit_max_area_change, args.fit_merge_angle,
                             fill_holes_area=args.fill_holes_area, min_building_area=args.min_area)
    fitted, fit_reports = fit_polygons(outlines, fit_options, fit_progress)
    retained = {r["source_index"] for r in fit_reports}
    small_buildings_removed += len(outlines) - len(retained)
    small_buildings_area += sum(p.area for i, p in enumerate(outlines) if i not in retained)
    count = len(fitted)
    with fiona.open(staged_gpkg, "w", driver="GPKG", layer="buildings", schema=schema, crs_wkt=crs.to_wkt()) as gdst, \
         fiona.open(staged_shp, "w", driver="ESRI Shapefile", schema=schema, crs_wkt=crs.to_wkt(), encoding="UTF-8") as sdst:
        for index, (polygon, report) in enumerate(zip(fitted, fit_reports), 1):
            edge = source_edges[report["source_index"]]
            total_area += polygon.area
            record = {"geometry": mapping(polygon), "properties": {"bldg_id": index, "area_m2": polygon.area,
                      "holes": len(polygon.interiors), "src_edge": edge, "grid_m": grid["resolution"],
                      "fit_done": int(report["changed"]), "fit_tol": report["tolerance"],
                      "v_old": report["vertices_old"], "v_new": report["vertices_new"],
                      "holes_rm": report["holes_removed"], "hole_m2": report["hole_fill_area"],
                      "holes_fit": report["holes_fitted"]}}
            gdst.write(record)
            sdst.write(record)
    for file in stage.iterdir():
        if file.is_file() and file.stem == gpkg.stem:
            os.replace(file, args.output / file.name)
    summary = {"complete": True, "inference_tiles": len(tiles), "core_points": core_points, "predicted_building_points": building_points,
               "polygons": count, "area_m2": total_area, "horizontal_crs": crs.to_string(), "grid": grid,
               "postprocessing": {"simplify_m": args.simplify, "close_radius_m": args.close_radius,
                                  "fill_holes_area_m2": args.fill_holes_area, "min_area_m2": args.min_area,
                                  "line_fitting": vars(fit_options)},
               "line_fitting": {"changed": sum(r["changed"] for r in fit_reports),
                                "algorithm": "tls_local_v3_outer_inner_cleanup",
                                "small_buildings_removed": small_buildings_removed,
                                "small_buildings_removed_area_m2": small_buildings_area,
                                "small_holes_removed": sum(r["holes_removed"] for r in fit_reports),
                                "retained_holes_fitted": sum(r["holes_fitted"] for r in fit_reports),
                                "hole_fill_area_m2": sum(r["hole_fill_area"] for r in fit_reports),
                                "reasons": dict(Counter(r["reason"] for r in fit_reports)),
                                "attempt_reasons": dict(Counter(reason for r in fit_reports for reason in r["attempt_reasons"])),
                                "vertices_before": sum(map(vertex_count, outlines)),
                                "vertices_after": sum(map(vertex_count, fitted))},
               "outputs": [gpkg.name, shp.name],
               "meaning": "Predicted semantic building coverage with automatic constrained wall-line fitting. Final fitted vectors only; no building-instance or cadastral inference."}
    write_json(work / "vector_summary.json", summary)
    LOG.info("Cleanup complete: small buildings removed=%d; holes filled=%d; retained inner rings fitted=%d.", small_buildings_removed, summary["line_fitting"]["small_holes_removed"], summary["line_fitting"]["retained_holes_fitted"])
    LOG.info("Prediction, wall-line fitting and vector export complete: %s polygons. %s and %s", f"{count:,}", gpkg, shp)


def main():
    args = arguments()
    # init_config uses the repository root as its discovery starting point.
    os.chdir(ROOT)
    args.output.mkdir(parents=True, exist_ok=True)
    work = args.output / (args.run_name + "_work")
    setup_logging(work)
    # Prevent concurrent runs writing the same cache/mosaic.
    lock = work / "running.lock"
    try:
        descriptor = os.open(lock, os.O_WRONLY | os.O_CREAT | os.O_EXCL)
    except FileExistsError:
        raise RuntimeError(f"Run lock exists: {lock}. If a previous process crashed, confirm it is no longer running and remove only this lock file.")
    try:
        with os.fdopen(descriptor, "w") as stream:
            stream.write(str(os.getpid()))
        import numpy as np
        import laspy
        check_vector_dependencies(args)
        # COPC reader uses hierarchy queries, not a sequential ordinary LAZ read.
        with laspy.CopcReader.open(args.input) as reader:
            crs = horizontal_crs(reader.header, args.epsg)
            grid = make_grid(reader.header, args.resolution, args.max_raster_cells)
            tiles = list(grid_tiles(grid, args.tile_size))
            (work / "tiles").mkdir(exist_ok=True)
            stat = args.input.stat()
            signature = {"wrapper_version": WRAPPER_VERSION, "input_name": args.input.name,
                         "input_bytes": stat.st_size, "input_mtime_ns": stat.st_mtime_ns,
                         "checkpoint_sha256": file_hash(args.checkpoint), "source_sha256": source_hash(),
                         "tile_size": args.tile_size, "halo": args.halo, "seed": args.seed,
                         "grid": grid, "crs_wkt": crs.to_wkt(), "point_count": int(reader.header.point_count)}
            state_file = work / "run.json"
            if state_file.exists():
                if not compatible_cache(json.loads(state_file.read_text(encoding="utf-8"))["signature"], signature):
                    raise ValueError("Cache does not match the input, checkpoint, source, or inference settings. Use a NEW --run-name; old results are preserved.")
            else:
                write_json(state_file, {"signature": signature, "complete": False, "total_tiles": len(tiles), "completed_tiles": 0})
            done = 0
            missing = []
            for tile in tiles:
                path = cache_path(work, tile)
                if path.exists():
                    load_cache(path, tile)
                    done += 1
                else:
                    missing.append(tile)
            LOG.info("Source=%s, %s points. Cached tiles=%d/%d", args.input, f"{reader.header.point_count:,}", done, len(tiles))
            if args.vector_only and missing:
                raise ValueError(f"Refusing partial vector export: {len(missing)} tiles have no prediction cache.")
            write_json(state_file, {"signature": signature, "complete": False, "prediction_complete": not missing,
                                    "total_tiles": len(tiles), "completed_tiles": done})
            if missing and not args.vector_only:
                predictor = OfficialPredictor(args, work)
                new_count = 0
                for tile in missing:
                    if args.max_tiles and new_count >= args.max_tiles:
                        LOG.info("Stopped at --max-tiles. Prediction is INCOMPLETE; no final vectors exported. Repeat without that limit to continue.")
                        return
                    started = time.monotonic()
                    try:
                        mask, metadata = predict_tile(reader, predictor, grid, tile, args)
                    except Exception as exc:
                        # No missing/failed tile is replaced with an empty prediction.
                        LOG.exception("Tile %06d FAILED. Existing successful tiles are retained.", tile["id"])
                        if "out of memory" in str(exc).lower():
                            LOG.error("CUDA OOM: use a smaller --tile-size together with a NEW --run-name, or use a GPU with more VRAM. Model features/voxel size were not automatically changed.")
                        raise
                    save_cache(cache_path(work, tile), mask, metadata)
                    done += 1
                    new_count += 1
                    write_json(state_file, {"signature": signature, "complete": False, "prediction_complete": done == len(tiles), "total_tiles": len(tiles), "completed_tiles": done})
                    del mask
                    gc.collect()
                    predictor.torch.cuda.empty_cache()
                    LOG.info("Saved tile %06d (%d/%d), %.1fs", tile["id"], done, len(tiles), time.monotonic() - started)
                del predictor
                gc.collect()
            # Export only after the entire prediction has succeeded.
            export_vectors(args, work, tiles, grid, crs)
            write_json(state_file, {"signature": signature, "complete": True, "total_tiles": len(tiles), "completed_tiles": len(tiles)})
    finally:
        lock.unlink(missing_ok=True)


if __name__ == "__main__":
    try:
        main()
    except Exception:
        LOG.exception("Stopped; the full-area prediction/vector export is not confirmed complete.")
        sys.exit(1)
