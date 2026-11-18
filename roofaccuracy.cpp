#include "roofaccuracy.h"
#include "buildingeditpreviewbus.h"
#include "roofmodelpipeline.h"
#include "ui_roofedit.h"

#include <qgisinterface.h>
#include <qgs3dmapcanvas.h>
#include <qgs3dmapsettings.h>
#include <qgsapplication.h>
#include <qgscoordinatetransform.h>
#include <qgsexception.h>
#include <qgsgeometry.h>
#include <qgslayertree.h>
#include <qgslayertreelayer.h>
#include <qgsmarkersymbol.h>
#include <qgsphongmaterialsettings.h>
#include <qgspoint3dsymbol.h>
#include <qgspointcloudlayer.h>
#include <qgsproject.h>
#include <qgsrulebased3drenderer.h>
#include <qgsrulebasedrenderer.h>
#include <qgstaskmanager.h>
#include <qgsunittypes.h>
#include <qgsvectordataprovider.h>
#include <qgsvectorlayer.h>

#include <QApplication>
#include <QCoreApplication>
#include <QMessageBox>
#include <QPointer>
#include <QSet>
#include <QSignalBlocker>
#include <QWidget>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace
{
using Vec = std::array<double, 3>;
Vec
sub ( const Vec &a, const Vec &b )
{
  return { a[0] - b[0], a[1] - b[1], a[2] - b[2] };
}
Vec
add ( const Vec &a, const Vec &b )
{
  return { a[0] + b[0], a[1] + b[1], a[2] + b[2] };
}
Vec
mul ( const Vec &a, double s )
{
  return { a[0] * s, a[1] * s, a[2] * s };
}
double
dot ( const Vec &a, const Vec &b )
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
bool
finitePoint ( const QgsPoint &p )
{
  return std::isfinite ( p.x () ) && std::isfinite ( p.y () ) && std::isfinite ( p.z () );
}
double
boundary ( double tolerance )
{
  return tolerance + 1e-9;
}

struct Triangle
{
  Vec a, b, c;
};
double
triangleDistanceSquared ( const Vec &p, const Triangle &t )
{
  const Vec ab = sub ( t.b, t.a ), ac = sub ( t.c, t.a ), ap = sub ( p, t.a );
  const double d1 = dot ( ab, ap ), d2 = dot ( ac, ap );
  if ( d1 <= 0 && d2 <= 0 )
    return dot ( ap, ap );
  const Vec bp = sub ( p, t.b );
  const double d3 = dot ( ab, bp ), d4 = dot ( ac, bp );
  if ( d3 >= 0 && d4 <= d3 )
    return dot ( bp, bp );
  const double vc = d1 * d4 - d3 * d2;
  if ( vc <= 0 && d1 >= 0 && d3 <= 0 )
  {
    const Vec delta = sub ( ap, mul ( ab, d1 / ( d1 - d3 ) ) );
    return dot ( delta, delta );
  }
  const Vec cp = sub ( p, t.c );
  const double d5 = dot ( ab, cp ), d6 = dot ( ac, cp );
  if ( d6 >= 0 && d5 <= d6 )
    return dot ( cp, cp );
  const double vb = d5 * d2 - d1 * d6;
  if ( vb <= 0 && d2 >= 0 && d6 <= 0 )
  {
    const Vec delta = sub ( ap, mul ( ac, d2 / ( d2 - d6 ) ) );
    return dot ( delta, delta );
  }
  const double va = d3 * d6 - d5 * d4;
  if ( va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0 )
  {
    const Vec delta = sub ( bp, mul ( sub ( t.c, t.b ), ( d4 - d3 ) / ( d4 - d3 + d5 - d6 ) ) );
    return dot ( delta, delta );
  }
  const double inverse = 1.0 / ( va + vb + vc );
  const Vec delta = sub ( ap, add ( mul ( ab, vb * inverse ), mul ( ac, vc * inverse ) ) );
  return dot ( delta, delta );
}

// Exact finite-triangle distances accelerated by a bounding-volume tree.
class SurfaceIndex
{
  struct Node
  {
    Vec low, high;
    int begin = 0, end = 0, left = -1, right = -1;
  };
  std::vector<int> order;
  std::vector<Node> nodes;
  int
  buildNode ( int begin, int end )
  {
    Node n;
    n.begin = begin;
    n.end = end;
    n.low.fill ( std::numeric_limits<double>::infinity () );
    n.high.fill ( -std::numeric_limits<double>::infinity () );
    for ( int i = begin; i < end; ++i )
    {
      const auto &t = triangles[order[i]];
      for ( int a = 0; a < 3; ++a )
      {
        n.low[a] = std::min ( { n.low[a], t.a[a], t.b[a], t.c[a] } );
        n.high[a] = std::max ( { n.high[a], t.a[a], t.b[a], t.c[a] } );
      }
    }
    const int id = static_cast<int> ( nodes.size () );
    nodes.push_back ( n );
    if ( end - begin > 8 )
    {
      int axis = 0;
      for ( int a = 1; a < 3; ++a )
        if ( n.high[a] - n.low[a] > n.high[axis] - n.low[axis] )
          axis = a;
      const int mid = begin + ( end - begin ) / 2;
      std::nth_element ( order.begin () + begin, order.begin () + mid, order.begin () + end,
                         [this, axis] ( int a, int b )
                         {
                           const auto &x = triangles[a], &y = triangles[b];
                           return x.a[axis] + x.b[axis] + x.c[axis] < y.a[axis] + y.b[axis] + y.c[axis];
                         } );
      const int left = buildNode ( begin, mid ), right = buildNode ( mid, end );
      nodes[id].left = left;
      nodes[id].right = right;
    }
    return id;
  }
  double
  boundsDistance ( const Node &n, const Vec &p ) const
  {
    double sum = 0;
    for ( int a = 0; a < 3; ++a )
    {
      const double d = std::max ( { n.low[a] - p[a], 0.0, p[a] - n.high[a] } );
      sum += d * d;
    }
    return sum;
  }
  void
  visit ( int id, const Vec &p, double &best ) const
  {
    const Node &n = nodes[id];
    if ( boundsDistance ( n, p ) > best )
      return;
    if ( n.left < 0 )
    {
      for ( int i = n.begin; i < n.end; ++i )
        best = std::min ( best, triangleDistanceSquared ( p, triangles[order[i]] ) );
      return;
    }
    int first = n.left, second = n.right;
    if ( boundsDistance ( nodes[first], p ) > boundsDistance ( nodes[second], p ) )
      std::swap ( first, second );
    visit ( first, p, best );
    visit ( second, p, best );
  }

public:
  std::vector<Triangle> triangles;
  void
  build ()
  {
    order.resize ( triangles.size () );
    std::iota ( order.begin (), order.end (), 0 );
    nodes.reserve ( triangles.size () * 2 );
    buildNode ( 0, static_cast<int> ( triangles.size () ) );
  }
  double
  distanceSquared ( const Vec &p ) const
  {
    double best = std::numeric_limits<double>::infinity ();
    visit ( 0, p, best );
    return best;
  }
};

class AccuracyTask final : public QgsTask
{
public:
  AccuracyTask ( QgsGeometry footprint, BuildingRoof::Mesh mesh, QVector<BuildingRoof::RoofSample> samples,
                 QVector<QgsPoint> display )
      : QgsTask ( QObject::tr ( "屋顶精度评价" ), QgsTask::CanCancel ), footprint ( std::move ( footprint ) ),
        mesh ( std::move ( mesh ) ),
        samples ( std::move ( samples ) ), display ( std::move ( display ) )
  {
  }
  RoofAccuracy::Result result;

protected:
  bool
  run () override
  {
    result = RoofAccuracy::evaluate (
        footprint, mesh, samples, display, [this] () { return isCanceled (); }, [this] ( double p ) { setProgress ( p ); } );
    return !result.canceled && result.error.isEmpty () && !result.points.isEmpty ();
  }

private:
  QgsGeometry footprint;
  BuildingRoof::Mesh mesh;
  QVector<BuildingRoof::RoofSample> samples;
  QVector<QgsPoint> display;
};

QgsMarkerSymbol *
marker ( const QColor &color, double size )
{
  return QgsMarkerSymbol::createSimple ( { { QStringLiteral ( "name" ), QStringLiteral ( "square" ) },
                                           { QStringLiteral ( "color" ), color.name () },
                                           { QStringLiteral ( "outline_style" ), QStringLiteral ( "no" ) },
                                           { QStringLiteral ( "size" ), QString::number ( size ) } } )
      .release ();
}
QgsPoint3DSymbol *
symbol3D ( const QColor &color )
{
  auto *symbol = new QgsPoint3DSymbol ();
  symbol->setAltitudeClamping ( Qgis::AltitudeClamping::Absolute );
  symbol->setShape ( Qgis::Point3DShape::Billboard );
  symbol->setBillboardSymbol ( marker ( color, 3.2 ) );
  auto *material = new QgsPhongMaterialSettings ();
  material->setAmbient ( color );
  material->setDiffuse ( color );
  symbol->setMaterialSettings ( material );
  return symbol;
}
void
styleLayer ( QgsVectorLayer *layer, const RoofAccuracy::Result &result, double tolerance )
{
  const std::array<QColor, 4> colors
      = { QColor ( 35, 180, 80 ), QColor ( 175, 205, 35 ), QColor ( 255, 155, 20 ), QColor ( 225, 45, 45 ) };
  const std::array<double, 3> limits = { tolerance * .5, tolerance, tolerance * 2 };
  std::array<int, 4> counts = { 0, 0, 0, 0 };
  for ( const auto &point : result.points )
    ++counts[point.distance <= boundary ( limits[0] )   ? 0
             : point.distance <= boundary ( limits[1] ) ? 1
             : point.distance <= boundary ( limits[2] ) ? 2
                                                        : 3];
  auto *root2D = new QgsRuleBasedRenderer::Rule ( nullptr );
  auto *root3D = new QgsRuleBased3DRenderer::Rule ( nullptr );
  for ( int i = 0; i < 4; ++i )
  {
    const QString lo
        = i ? QStringLiteral ( "\"distance_m\" > %1" ).arg ( boundary ( limits[i - 1] ), 0, 'g', 17 ) : QString ();
    const QString hi
        = i < 3 ? QStringLiteral ( "\"distance_m\" <= %1" ).arg ( boundary ( limits[i] ), 0, 'g', 17 ) : QString ();
    const QString filter = lo.isEmpty () ? hi : hi.isEmpty () ? lo : lo + QStringLiteral ( " AND " ) + hi;
    const QString range = i == 0 ? QObject::tr ( "≤ %1 m" ).arg ( limits[0], 0, 'f', 3 )
                          : i == 3
                              ? QObject::tr ( "> %1 m" ).arg ( limits[2], 0, 'f', 3 )
                              : QObject::tr ( "%1–%2 m" ).arg ( limits[i - 1], 0, 'f', 3 ).arg ( limits[i], 0, 'f', 3 );
    const QString label = QObject::tr ( "%1（%2 点）" ).arg ( range ).arg ( counts[i] );
    root2D->appendChild ( new QgsRuleBasedRenderer::Rule ( marker ( colors[i], 1.6 ), 0, 0, filter, label ) );
    root3D->appendChild ( new QgsRuleBased3DRenderer::Rule ( symbol3D ( colors[i] ), filter, label ) );
  }
  layer->setRenderer ( new QgsRuleBasedRenderer ( root2D ) );
  auto *renderer = new QgsRuleBased3DRenderer ( root3D );
  renderer->setLayer ( layer );
  layer->setRenderer3D ( renderer );
  layer->triggerRepaint ();
}
}

double
RoofAccuracy::Result::inlierRate ( double tolerance ) const
{
  if ( points.isEmpty () )
    return 0;
  const double limit = boundary ( tolerance );
  qsizetype count = 0;
  for ( const auto &p : points )
    if ( p.distance <= limit )
      ++count;
  return 100.0 * count / points.size ();
}

RoofAccuracy::Result
RoofAccuracy::evaluate ( const QgsGeometry &footprint, const BuildingRoof::Mesh &mesh,
                         const QVector<BuildingRoof::RoofSample> &samples,
                         const QVector<QgsPoint> &displayPoints, const std::function<bool ()> &canceled,
                         const std::function<void ( double )> &progress )
{
  Result result;
  auto stop = [&] ()
  {
    if ( canceled && canceled () )
    {
      result.canceled = true;
      return true;
    }
    return false;
  };
  try
  {
    if ( footprint.isEmpty () || mesh.isEmpty () || samples.isEmpty ()
         || ( !displayPoints.isEmpty () && displayPoints.size () != samples.size () ) )
    {
      result.error = QObject::tr ( "缺少有效的屋顶模型或评价点云。" );
      return result;
    }
    if ( stop () )
      return result;

    BuildingRoof::RoofSegmentationOptions segmentationOptions;
    segmentationOptions.allowVerticalPlanes = true;
    segmentationOptions.useRoofHeightBand = false;
    const auto segmentation = BuildingRoof::segmentRoofCloud ( footprint, samples, segmentationOptions );
    if ( stop () )
      return result;

    struct Candidate
    {
      const QVector<QgsPoint> *points;
      QgsGeometry projection;
      double minZ, maxZ;
    };
    QVector<Candidate> candidates;
    for ( const auto &segment : segmentation.segments )
    {
      // Use the same near-vertical cutoff as the roof modeling pipeline.
      if ( segment.points.isEmpty () || std::fabs ( segment.normalZ ) < 0.20 )
        continue;
      QgsMultiPointXY projectedPoints;
      projectedPoints.reserve ( segment.points.size () );
      double minZ = segment.points.first ().z (), maxZ = minZ;
      for ( const QgsPoint &point : segment.points )
      {
        projectedPoints.append ( QgsPointXY ( point.x (), point.y () ) );
        minZ = std::min ( minZ, point.z () );
        maxZ = std::max ( maxZ, point.z () );
      }
      candidates.append ( { &segment.points, QgsGeometry::fromMultiPointXY ( projectedPoints ).convexHull (), minZ, maxZ } );
    }

    auto key = [] ( const QgsPoint &point ) { return std::make_tuple ( point.x (), point.y (), point.z () ); };
    std::set<std::tuple<double, double, double> > roofPoints;
    for ( const Candidate &candidate : std::as_const ( candidates ) )
    {
      if ( stop () )
        return result;
      // Reject underlying surfaces only when clearly higher candidates cover most of their projection.
      // This also preserves a lower roof adjoining a higher roof.
      QVector<QgsGeometry> above;
      const QgsRectangle bounds = candidate.projection.boundingBox ();
      for ( const Candidate &other : std::as_const ( candidates ) )
        if ( other.minZ > candidate.maxZ + 0.5 && bounds.intersects ( other.projection.boundingBox () ) )
          above.append ( other.projection );
      if ( !above.isEmpty ()
           && candidate.projection.intersection ( QgsGeometry::unaryUnion ( above ) ).area () > 0.5 * candidate.projection.area () )
        continue;
      for ( const QgsPoint &point : *candidate.points )
        roofPoints.insert ( key ( point ) );
    }
    if ( roofPoints.empty () )
    {
      result.error = QObject::tr ( "点云分割未识别到可评价的屋面点。" );
      return result;
    }

    const QgsPoint origin = mesh.vertices.first ();
    auto local = [&] ( const QgsPoint &p ) -> Vec
    { return { p.x () - origin.x (), p.y () - origin.y (), p.z () - origin.z () }; };
    SurfaceIndex surface;
    for ( int i = 0; i + 2 < mesh.indices.size (); i += 3 )
    {
      if ( ( i % 3072 ) == 0 && stop () )
        return result;
      const int a = mesh.indices[i], b = mesh.indices[i + 1], c = mesh.indices[i + 2];
      if ( a < 0 || b < 0 || c < 0 || a >= mesh.vertices.size () || b >= mesh.vertices.size ()
           || c >= mesh.vertices.size () )
        continue;
      const QgsPoint &pa = mesh.vertices[a], &pb = mesh.vertices[b], &pc = mesh.vertices[c];
      if ( !finitePoint ( pa ) || !finitePoint ( pb ) || !finitePoint ( pc ) )
        continue;
      const Vec ab = sub ( local ( pb ), local ( pa ) ), ac = sub ( local ( pc ), local ( pa ) );
      // Existing roof builders use Z=0 for the bottom, and vertical walls have zero XY area.
      if ( std::fabs ( ab[0] * ac[1] - ab[1] * ac[0] ) <= 1e-10 || std::max ( { pa.z (), pb.z (), pc.z () } ) <= 1e-8 )
        continue;
      surface.triangles.push_back ( { local ( pa ), local ( pb ), local ( pc ) } );
    }
    if ( surface.triangles.empty () )
    {
      result.error = QObject::tr ( "模型中没有可评价的屋面三角形。" );
      return result;
    }
    surface.build ();
    result.points.reserve ( static_cast<qsizetype> ( roofPoints.size () ) );
    long double sum = 0, squared = 0;
    for ( qsizetype i = 0; i < samples.size (); ++i )
    {
      if ( ( i & 127 ) == 0 )
      {
        if ( stop () )
          return result;
        if ( progress )
          progress ( 100.0 * i / samples.size () );
      }
      const QgsPoint &point = samples.at ( i ).point;
      if ( !finitePoint ( point ) || roofPoints.find ( key ( point ) ) == roofPoints.end () )
        continue;
      const double d2 = surface.distanceSquared ( local ( point ) );
      if ( !std::isfinite ( d2 ) )
      {
        result.error = QObject::tr ( "点到屋面距离计算失败。" );
        return result;
      }
      const double d = std::sqrt ( std::max ( 0.0, d2 ) );
      result.points.append ( { displayPoints.isEmpty () ? point : displayPoints.at ( i ), d } );
      sum += d;
      squared += d2;
    }
    const qsizetype count = result.points.size ();
    if ( count == 0 )
    {
      result.error = QObject::tr ( "点云分割未识别到可评价的屋面点。" );
      return result;
    }
    result.mae = static_cast<double> ( sum / count );
    result.rmse = std::sqrt ( static_cast<double> ( squared / count ) );
    if ( progress )
      progress ( 100 );
    stop ();
  }
  catch ( const std::exception &e )
  {
    result.error = QObject::tr ( "精度计算失败：%1" ).arg ( QString::fromUtf8 ( e.what () ) );
  }
  return result;
}

struct RoofAccuracy::Controller::Private
{
  Controller *owner;
  QPointer<QWidget> panel;
  Ui::roofedit &ui;
  QPointer<QgisInterface> iface;
  QPointer<QgsVectorLayer> layer, displayLayer;
  QgsFeatureId fid = FID_NULL;
  RoofModelSettings::Selection selection;
  Result result;
  QPointer<AccuracyTask> task;
  QList<QPointer<AccuracyTask> > tasks;
  QList<QMetaObject::Connection> inputConnections;
  QSet<QString> confirmedLocalMeters;
  QString coordinateNote;

  Private ( Controller *owner, QWidget *panel, Ui::roofedit &ui, QgisInterface *iface )
      : owner ( owner ), panel ( panel ), ui ( ui ), iface ( iface )
  {
  }

  void
  update ()
  {
    if ( !panel )
      return;
    const bool valid = !result.points.isEmpty () && result.error.isEmpty () && !result.canceled;
    const QString empty = QStringLiteral ( "—" );
    ui.accuracyPointCountValueLabel->setText ( valid ? QString::number ( result.points.size () ) : empty );
    ui.accuracyMaeValueLabel->setText ( valid ? QString::number ( result.mae, 'f', 3 ) : empty );
    ui.accuracyRmseValueLabel->setText ( valid ? QString::number ( result.rmse, 'f', 3 ) : empty );
    ui.accuracyInlierRateValueLabel->setText (
        valid ? QString::number ( result.inlierRate ( ui.accuracyToleranceSpinBox->value () ), 'f', 2 ) : empty );
    auto *cloud
        = qobject_cast<QgsPointCloudLayer *> ( QgsProject::instance ()->mapLayer ( selection.pointCloudLayerId ) );
    ui.calculateAccuracyButton->setEnabled ( task
                                             || ( layer && fid != FID_NULL
                                                  && selection.type != RoofModelSettings::Type::None && cloud
                                                  && cloud->isValid () ) );
    if ( !task )
      ui.calculateAccuracyButton->setText ( QObject::tr ( "计算精度" ) );
    ui.showAccuracyIn3DCheckBox->setEnabled ( valid && !task );
    ui.accuracyGroupBox->setToolTip ( valid ? QObject::tr ( "%1\n逐点评价点云分割识别的 %2 个屋面点。误差为点到屋面的最短三维距离。" )
                                                  .arg ( coordinateNote )
                                                  .arg ( result.points.size () )
                                            : QObject::tr ( "计算当前建筑物屋顶与所选点云的拟合误差。" ) );
  }

  void
  clearDisplay ()
  {
    if ( !displayLayer )
      return;
    for ( auto *canvas : iface ? iface->mapCanvases3D () : QList<Qgs3DMapCanvas *> () )
    {
      if ( !canvas || !canvas->mapSettings () )
        continue;
      auto layers = canvas->mapSettings ()->layers ();
      if ( layers.removeAll ( displayLayer ) )
        canvas->mapSettings ()->setLayers ( layers );
    }
    const QString id = displayLayer->id ();
    displayLayer = nullptr;
    QgsProject::instance ()->removeMapLayer ( id );
  }
  void
  clear ()
  {
    if ( task )
    {
      QObject::disconnect ( task, nullptr, owner, nullptr );
      task->cancel ();
      task = nullptr;
    }
    result = Result ();
    if ( panel )
    {
      const QSignalBlocker blocker ( ui.showAccuracyIn3DCheckBox );
      ui.showAccuracyIn3DCheckBox->setChecked ( false );
    }
    clearDisplay ();
    update ();
  }

  void
  watchInputs ()
  {
    for ( const auto &connection : inputConnections )
      QObject::disconnect ( connection );
    inputConnections.clear ();
    for ( QgsMapLayer *input : { static_cast<QgsMapLayer *> ( layer.data () ),
                                 QgsProject::instance ()->mapLayer ( selection.pointCloudLayerId ) } )
    {
      if ( !input )
        continue;
      inputConnections.append ( QObject::connect ( input, &QgsMapLayer::dataChanged, owner, [this] () { clear (); } ) );
      inputConnections.append (
          QObject::connect ( input, &QgsMapLayer::dataSourceChanged, owner, [this] () { clear (); } ) );
      inputConnections.append ( QObject::connect ( input, &QgsMapLayer::crsChanged, owner, [this] () { clear (); } ) );
      inputConnections.append ( QObject::connect ( input, &QObject::destroyed, owner, [this] () { clear (); } ) );
    }
    if ( layer )
    {
      inputConnections.append ( QObject::connect ( layer, &QgsVectorLayer::geometryChanged, owner,
                                                   [this] ( QgsFeatureId changed, const QgsGeometry & )
                                                   {
                                                     if ( changed == fid )
                                                       clear ();
                                                   } ) );
      inputConnections.append ( QObject::connect ( layer, &QgsVectorLayer::featuresDeleted, owner,
                                                   [this] ( const QgsFeatureIds &deleted )
                                                   {
                                                     if ( deleted.contains ( fid ) )
                                                     {
                                                       fid = FID_NULL;
                                                       clear ();
                                                     }
                                                   } ) );
    }
    if ( auto *cloud
         = qobject_cast<QgsPointCloudLayer *> ( QgsProject::instance ()->mapLayer ( selection.pointCloudLayerId ) ) )
      inputConnections.append (
          QObject::connect ( cloud, &QgsPointCloudLayer::subsetStringChanged, owner, [this] () { clear (); } ) );
  }

  bool
  metricInputs ( QgsPointCloudLayer *cloud, QgsGeometry &footprint, BuildingRoof::Mesh &mesh,
                 QVector<BuildingRoof::RoofSample> &samples, QVector<QgsPoint> &display )
  {
    const auto vectorCrs = layer->crs (), cloudCrs = cloud->crs ();
    const bool shared = vectorCrs.isValid () && cloudCrs.isValid () && vectorCrs == cloudCrs;
    const auto units = shared ? vectorCrs.mapUnits () : Qgis::DistanceUnit::Unknown;
    double xyFactor = 1, zFactor = 1;
    QgsCoordinateTransform toMetric;
    bool transform = false;
    if ( shared && !vectorCrs.isGeographic () && units != Qgis::DistanceUnit::Unknown )
    {
      xyFactor = QgsUnitTypes::fromUnitToUnitFactor ( units, Qgis::DistanceUnit::Meters );
      zFactor = xyFactor;
      const auto vertical = cloudCrs.verticalCrs ();
      if ( vertical.isValid () && vertical.mapUnits () != Qgis::DistanceUnit::Unknown )
        zFactor = QgsUnitTypes::fromUnitToUnitFactor ( vertical.mapUnits (), Qgis::DistanceUnit::Meters );
      coordinateNote = QObject::tr ( "按图层坐标单位换算为米" );
    }
    else
    {
      // A shared CRS label is not a reliable unit declaration for synthetic/local data.
      // Do not reject it just because the label says geographic or is missing.
      const QString key = cloud->id () + cloud->source () + cloudCrs.toWkt () + vectorCrs.toWkt ();
      bool localMeters = confirmedLocalMeters.contains ( key );
      const QgsRectangle cloudExtent = cloud->extent ();
      const bool plausibleGeographic = shared && vectorCrs.isGeographic () && cloudExtent.xMinimum () >= -180
                                       && cloudExtent.xMaximum () <= 180 && cloudExtent.yMinimum () >= -90
                                       && cloudExtent.yMaximum () <= 90;
      if ( !localMeters )
      {
        QMessageBox choice ( panel );
        choice.setWindowTitle ( QObject::tr ( "评价坐标单位" ) );
        choice.setText (
            QObject::tr ( "当前坐标系标签不足以确定距离单位。\n建筑图层：%1\n点云图层：%"
                          "2\n如果建筑与点云的原始坐标已对齐，且 X/Y/Z 已按米缩放，请选择“按本地米制计算”。" )
                .arg ( vectorCrs.isValid () ? vectorCrs.userFriendlyIdentifier () : QObject::tr ( "未定义" ) )
                .arg ( cloudCrs.isValid () ? cloudCrs.userFriendlyIdentifier () : QObject::tr ( "未定义" ) ) );
        auto *localButton = choice.addButton ( QObject::tr ( "按本地米制计算" ), QMessageBox::AcceptRole );
        QAbstractButton *geographicButton
            = plausibleGeographic ? choice.addButton ( QObject::tr ( "按经纬度换算" ), QMessageBox::ActionRole )
                                  : nullptr;
        choice.addButton ( QMessageBox::Cancel );
        choice.exec ();
        if ( choice.clickedButton () == localButton )
        {
          localMeters = true;
          confirmedLocalMeters.insert ( key );
        }
        else if ( !geographicButton || choice.clickedButton () != geographicButton )
          return false;
      }
      if ( localMeters )
        coordinateNote = QObject::tr ( "本地坐标：X/Y/Z 按米计算，使用原始坐标对齐" );
      else
      {
        const QgsPointXY center = footprint.boundingBox ().center ();
        const QgsCoordinateTransform toWgs ( vectorCrs, QgsCoordinateReferenceSystem::fromEpsgId ( 4326 ),
                                             QgsProject::instance ()->transformContext () );
        const QgsPointXY anchor = toWgs.transform ( center );
        QgsCoordinateReferenceSystem metric;
        metric.createFromProj ( QStringLiteral ( "+proj=aeqd +lat_0=%1 +lon_0=%2 +datum=WGS84 +units=m +no_defs" )
                                    .arg ( anchor.y (), 0, 'g', 17 )
                                    .arg ( anchor.x (), 0, 'g', 17 ) );
        if ( !metric.isValid () )
          throw std::runtime_error ( "Cannot create a local metric coordinate system" );
        toMetric = QgsCoordinateTransform ( vectorCrs, metric, QgsProject::instance ()->transformContext () );
        transform = true;
        coordinateNote = QObject::tr ( "经纬度在内部转换为局部米制坐标，Z 按米计算" );
      }
    }
    if ( !transform && xyFactor == 1 && zFactor == 1 )
      return true;

    auto convert = [&] ( const QgsPoint &point )
    {
      const QgsPointXY xy = transform ? toMetric.transform ( QgsPointXY ( point.x (), point.y () ) )
                                      : QgsPointXY ( point.x (), point.y () );
      return QgsPoint ( xy.x () * xyFactor, xy.y () * xyFactor, point.z () * zFactor );
    };
    if ( transform || xyFactor != 1 )
      footprint.transformVertices ( convert );
    for ( auto &point : mesh.vertices )
      point = convert ( point );
    display.reserve ( samples.size () );
    for ( auto &sample : samples )
    {
      display.append ( sample.point );
      sample.point = convert ( sample.point );
    }
    return true;
  }

  void
  calculate ()
  {
    if ( task )
    {
      clear ();
      return;
    }
    if ( !layer || fid == FID_NULL )
      return;
    auto *cloud
        = qobject_cast<QgsPointCloudLayer *> ( QgsProject::instance ()->mapLayer ( selection.pointCloudLayerId ) );
    const QgsFeature feature = layer->getFeature ( fid );
    if ( !cloud || !cloud->dataProvider () || !feature.isValid () )
    {
      clear ();
      return;
    }
    clear ();
    BuildingRoof::MeshResult model;
    QVector<BuildingRoof::RoofSample> samples;
    bool complete = false;
    QApplication::setOverrideCursor ( Qt::WaitCursor );
    try
    {
      model = RoofModelPipeline::build ( feature.geometry (), selection );
      if ( model.success )
        samples = RoofModelPipeline::collectSamples ( feature.geometry (), selection.pointCloudLayerId, &complete );
    }
    catch ( const QgsException &e )
    {
      model.success = false;
      model.error = e.what ();
    }
    catch ( const std::exception &e )
    {
      model.success = false;
      model.error = QString::fromUtf8 ( e.what () );
    }
    QApplication::restoreOverrideCursor ();
    if ( !model.success || !complete || samples.isEmpty () )
    {
      QMessageBox::warning ( panel, QObject::tr ( "精度计算" ),
                             !model.success ? model.error
                             : !complete    ? QObject::tr ( "点云索引未就绪或读取不完整，请稍后重试。" )
                                            : QObject::tr ( "建筑轮廓内未读到点云，请检查实际坐标范围是否重叠。" ) );
      return;
    }
    QVector<QgsPoint> display;
    QgsGeometry footprint = feature.geometry ();
    try
    {
      if ( !metricInputs ( cloud, footprint, model.mesh, samples, display ) )
        return;
    }
    catch ( const QgsException &e )
    {
      QMessageBox::warning ( panel, QObject::tr ( "坐标换算" ), e.what () );
      return;
    }
    catch ( const std::exception &e )
    {
      QMessageBox::warning ( panel, QObject::tr ( "坐标换算" ), QString::fromUtf8 ( e.what () ) );
      return;
    }
    auto *next = new AccuracyTask ( std::move ( footprint ), std::move ( model.mesh ), std::move ( samples ), std::move ( display ) );
    next->setDependentLayers ( { layer.data (), cloud } );
    tasks.erase (
        std::remove_if ( tasks.begin (), tasks.end (), [] ( const QPointer<AccuracyTask> &t ) { return t.isNull (); } ),
        tasks.end () );
    tasks.append ( next );
    task = next;
    QObject::connect ( next, &QgsTask::progressChanged, owner,
                       [this, next] ( double p )
                       {
                         if ( task == next && panel )
                           ui.calculateAccuracyButton->setText (
                               QObject::tr ( "取消计算（%1%）" ).arg ( qRound ( p ) ) );
                       } );
    QObject::connect ( next, &QgsTask::taskCompleted, owner,
                       [this, next] ()
                       {
                         if ( task != next )
                           return;
                         result = next->result;
                         task = nullptr;
                         update ();
                       } );
    QObject::connect ( next, &QgsTask::taskTerminated, owner,
                       [this, next] ()
                       {
                         if ( task != next )
                           return;
                         const auto error = next->result.error;
                         task = nullptr;
                         update ();
                         if ( !error.isEmpty () )
                           QMessageBox::warning ( panel, QObject::tr ( "精度计算" ), error );
                       } );
    ui.calculateAccuracyButton->setText ( QObject::tr ( "取消计算" ) );
    update ();
    QgsApplication::taskManager ()->addTask ( next );
  }

  void
  show3D ( bool enabled )
  {
    if ( !enabled )
    {
      clearDisplay ();
      return;
    }
    if ( result.points.isEmpty () || !layer || !iface )
      return;
    auto points = std::make_unique<QgsVectorLayer> (
        QStringLiteral ( "PointZ?field=building_fid:long&field=distance_m:double" ), QObject::tr ( "屋顶精度误差点" ),
        QStringLiteral ( "memory" ) );
    points->setCrs ( layer->crs () );
    points->setCustomProperty ( QStringLiteral ( "building_edit_manager/accuracy_points" ), true );
    QgsFeatureList features;
    features.reserve ( result.points.size () );
    for ( const auto &p : result.points )
    {
      QgsFeature f ( points->fields () );
      f.setGeometry ( QgsGeometry::fromPoint ( p.point ) );
      f.setAttributes ( QgsAttributes () << fid << p.distance );
      features.append ( f );
    }
    if ( !points->isValid () || !points->dataProvider ()->addFeatures ( features ) )
    {
      const QSignalBlocker blocker ( ui.showAccuracyIn3DCheckBox );
      ui.showAccuracyIn3DCheckBox->setChecked ( false );
      QMessageBox::warning ( panel, QObject::tr ( "精度显示" ), QObject::tr ( "无法生成误差点图层。" ) );
      return;
    }
    points->updateExtents ();
    styleLayer ( points.get (), result, ui.accuracyToleranceSpinBox->value () );
    displayLayer = points.release ();
    QObject::connect ( displayLayer, &QObject::destroyed, owner,
                       [this] ()
                       {
                         displayLayer = nullptr;
                         if ( panel )
                         {
                           const QSignalBlocker blocker ( ui.showAccuracyIn3DCheckBox );
                           ui.showAccuracyIn3DCheckBox->setChecked ( false );
                         }
                       } );
    QgsProject::instance ()->addMapLayer ( displayLayer );
    auto canvases = iface->mapCanvases3D ();
    if ( canvases.isEmpty () )
    {
      if ( auto *canvas = iface->createNewMapCanvas3D ( QObject::tr ( "屋顶精度评价" ) ) )
        canvases.append ( canvas );
    }
    bool shown = false;
    for ( auto *canvas : canvases )
    {
      if ( !canvas || !canvas->mapSettings () )
        continue;
      auto layers = canvas->mapSettings ()->layers ();
      if ( !layers.contains ( displayLayer ) )
      {
        layers.append ( displayLayer );
        canvas->mapSettings ()->setLayers ( layers );
      }
      if ( auto *dock = qobject_cast<QWidget *> ( canvas->parent () ) )
        dock->show ();
      shown = true;
    }
    if ( !shown )
    {
      clearDisplay ();
      QMessageBox::warning ( panel, QObject::tr ( "精度显示" ), QObject::tr ( "请先打开三维视图后重试。" ) );
    }
  }
};

RoofAccuracy::Controller::Controller ( QWidget *panel, Ui::roofedit &ui, QgisInterface *iface, QObject *parent )
    : QObject ( parent ), d ( std::make_unique<Private> ( this, panel, ui, iface ) )
{
  connect ( ui.calculateAccuracyButton, &QPushButton::clicked, this, [this] () { d->calculate (); } );
  connect ( ui.showAccuracyIn3DCheckBox, &QCheckBox::toggled, this,
            [this] ( bool enabled ) { d->show3D ( enabled ); } );
  connect ( ui.accuracyToleranceSpinBox, qOverload<double> ( &QDoubleSpinBox::valueChanged ), this,
            [this] ( double tolerance )
            {
              d->update ();
              if ( d->displayLayer )
                styleLayer ( d->displayLayer, d->result, tolerance );
            } );
  connect ( BuildingEditPreviewBus::instance (), &BuildingEditPreviewBus::roofModelChanged, this,
            [this] ( QgsVectorLayer *layer, QgsFeatureId fid )
            {
              if ( layer == d->layer && fid == d->fid )
                d->clear ();
            } );
  connect ( QgsProject::instance (), &QgsProject::layersRemoved, this,
            [this] ()
            {
              if ( !QgsProject::instance ()->mapLayer ( d->selection.pointCloudLayerId ) )
                d->clear ();
            } );
  ui.accuracyPointCountLabel->setToolTip (
      tr ( "统计点云分割识别的屋面点，逐点计算到屋顶模型的距离。" ) );
  ui.showAccuracyIn3DCheckBox->setToolTip (
      tr ( "绿：≤0.5倍允许误差；黄绿：≤1倍；橙：≤2倍；红：>2倍。模型遮挡点时可开启建筑物三角网。" ) );
  d->update ();
}

RoofAccuracy::Controller::~Controller ()
{
  d->clear ();
  const auto tasks = d->tasks;
  for ( const auto &task : tasks )
  {
    if ( !task )
      continue;
    task->cancel ();
    task->waitForFinished ( 0 );
    if ( task )
      QCoreApplication::sendPostedEvents ( task, QEvent::DeferredDelete );
  }
}

void
RoofAccuracy::Controller::setContext ( QgsVectorLayer *layer, QgsFeatureId fid,
                                       const RoofModelSettings::Selection &selection )
{
  if ( d->layer != layer || d->fid != fid || d->selection.type != selection.type
       || d->selection.pointCloudLayerId != selection.pointCloudLayerId )
    d->clear ();
  d->layer = layer;
  d->fid = fid;
  d->selection = selection;
  d->watchInputs ();
  d->update ();
}

void
RoofAccuracy::Controller::clear ()
{
  d->clear ();
}
