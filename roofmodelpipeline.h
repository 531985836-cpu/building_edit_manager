#pragma once

#include "roofmodelsettings.h"

namespace RoofModelPipeline
{
  QVector<BuildingRoof::RoofSample> collectSamples( const QgsGeometry &geometry, const QString &pointCloudLayerId );
  BuildingRoof::MeshResult build( const QgsGeometry &geometry, const RoofModelSettings::Selection &selection );
}
