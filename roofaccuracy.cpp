#include "roofaccuracy.h"
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
#include <qgspointclouddataprovider.h>
#include <qgsproject.h>
#include <qgsrulebased3drenderer.h>
#include <qgsrulebasedrenderer.h>
#include <qgstaskmanager.h>
#include <qgsunittypes.h>
#include <qgsvectordataprovider.h>
#include <qgsvectorlayer.h>

#include <QHash>
#include <QMessageBox>
#include <QPointer>
#include <QScopeGuard>
#include <QSignalBlocker>
#include <QTimer>
#include <QWidget>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>
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

struct EvaluationPoints
{
  QVector<QPair<QgsPoint, QgsPoint>> points;
  QVector<qsizetype> displayIndices;
  QString error;
};

QVector<qsizetype>
sampleDisplayPoints ( const QVector<QPair<QgsPoint, QgsPoint>> &points )
{
  constexpr int minimumDisplayBudget = 100;
  constexpr int maximumDisplayBudget = 2200;
  QVector<qsizetype> indices;
  if ( points.size () <= minimumDisplayBudget )
  {
    indices.resize ( points.size () );
    std::iota ( indices.begin (), indices.end (), qsizetype ( 0 ) );
    return indices;
  }

  const int budget = std::clamp<qsizetype> ( points.size () / 3, minimumDisplayBudget, maximumDisplayBudget );
  double minX = points.first ().first.x (), maxX = minX;
  double minY = points.first ().first.y (), maxY = minY;
  for ( const auto &entry : points )
  {
    minX = std::min ( minX, entry.first.x () );
    maxX = std::max ( maxX, entry.first.x () );
    minY = std::min ( minY, entry.first.y () );
    maxY = std::max ( maxY, entry.first.y () );
  }
  const double width = maxX - minX, height = maxY - minY;
  const double aspect = height > 0.0 ? std::clamp ( width / height, 1.0 / budget, double ( budget ) ) : budget;
  const int columns = std::clamp ( int ( std::lround ( std::sqrt ( budget * aspect ) ) ), 1, budget );
  const int rows = budget / columns;
  const double xScale = width > 0.0 ? columns / width : 0.0;
  const double yScale = height > 0.0 ? rows / height : 0.0;

  struct Cell
  {
    qsizetype index = -1;
    double centerDistanceSquared = std::numeric_limits<double>::infinity ();
  };
  QVector<Cell> cells ( columns * rows );
  // Keep one point per occupied cell, independent of its error and of the edited mesh.
  for ( qsizetype i = 0; i < points.size (); ++i )
  {
    const QgsPoint &point = points[i].first;
    const double x = ( point.x () - minX ) * xScale, y = ( point.y () - minY ) * yScale;
    const int column = std::min ( int ( x ), columns - 1 ), row = std::min ( int ( y ), rows - 1 );
    const double dx = x - column - 0.5, dy = y - row - 0.5;
    const double distanceSquared = dx * dx + dy * dy;
    Cell &cell = cells[row * columns + column];
    if ( distanceSquared < cell.centerDistanceSquared )
      cell = { i, distanceSquared };
  }
  indices.reserve ( cells.size () );
  for ( const Cell &cell : std::as_const ( cells ) )
    if ( cell.index >= 0 )
      indices.append ( cell.index );
  return indices;
}

EvaluationPoints preparePoints ( const QgsGeometry &footprint, const QVector<BuildingRoof::RoofSample> &samples,
                                  const QVector<QgsPoint> &display, const std::function<bool ()> &canceled );
RoofAccuracy::Result measure ( const BuildingRoof::Mesh &mesh, const EvaluationPoints &points,
                               const std::function<bool ()> &canceled, const std::function<void ( double )> &progress );

class AccuracyTask final : public QgsTask
{
public:
  using QgsTask::isCanceled;

  AccuracyTask ( QgsGeometry footprint, BuildingRoof::Mesh mesh, QVector<BuildingRoof::RoofSample> samples,
                 QVector<QgsPoint> display, std::shared_ptr<const EvaluationPoints> prepared )
      : QgsTask ( QObject::tr ( "屋顶精度评价" ), QgsTask::CanCancel | QgsTask::Hidden ),
        prepared ( std::move ( prepared ) ), footprint ( std::move ( footprint ) ),
        mesh ( std::move ( mesh ) ),
        samples ( std::move ( samples ) ), display ( std::move ( display ) )
  {
  }
  std::shared_ptr<const EvaluationPoints> prepared;
  RoofAccuracy::Result result;

protected:
  bool
  run () override
  {
    const auto canceled = [this] () { return isCanceled (); };
    if ( !prepared )
    {
      EvaluationPoints points = preparePoints ( footprint, samples, display, canceled );
      if ( !canceled () )
        points.displayIndices = sampleDisplayPoints ( points.points );
      prepared = std::make_shared<const EvaluationPoints> ( std::move ( points ) );
    }
    result = measure ( mesh, *prepared, canceled, [this] ( double p ) { setProgress ( p ); } );
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
styleLayer ( QgsVectorLayer *layer, const RoofAccuracy::Result &result, const QVector<qsizetype> &displayIndices,
             double tolerance )
{
  const std::array<QColor, 4> colors
      = { QColor ( 35, 180, 80 ), QColor ( 175, 205, 35 ), QColor ( 255, 155, 20 ), QColor ( 225, 45, 45 ) };
  const std::array<double, 3> limits = { tolerance * .5, tolerance, tolerance * 2 };
  std::array<int, 4> counts = { 0, 0, 0, 0 };
  for ( const qsizetype index : displayIndices )
  {
    const double distance = result.points[index].distance;
    ++counts[distance <= boundary ( limits[0] )   ? 0
             : distance <= boundary ( limits[1] ) ? 1
             : distance <= boundary ( limits[2] ) ? 2
                                                  : 3];
  }
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

namespace
{
EvaluationPoints
preparePoints ( const QgsGeometry &footprint, const QVector<BuildingRoof::RoofSample> &samples,
                const QVector<QgsPoint> &display, const std::function<bool ()> &canceled )
{
  EvaluationPoints result;
  try
  {
    if ( canceled && canceled () )
      return result;
    BuildingRoof::RoofSegmentationOptions segmentationOptions;
    segmentationOptions.allowVerticalPlanes = true;
    segmentationOptions.useRoofHeightBand = false;
    const auto segmentation = BuildingRoof::segmentRoofCloud ( footprint, samples, segmentationOptions );
    if ( canceled && canceled () )
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
      if ( canceled && canceled () )
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

    result.points.reserve ( static_cast<qsizetype> ( roofPoints.size () ) );
    for ( qsizetype i = 0; i < samples.size (); ++i )
    {
      if ( ( i & 127 ) == 0 && canceled && canceled () )
        return {};
      const QgsPoint &point = samples.at ( i ).point;
      if ( finitePoint ( point ) && roofPoints.find ( key ( point ) ) != roofPoints.end () )
        result.points.append ( { point, display.isEmpty () ? point : display.at ( i ) } );
    }
    if ( result.points.isEmpty () )
      result.error = QObject::tr ( "点云分割未识别到可评价的屋面点。" );
  }
  catch ( const std::exception &e )
  {
    result.error = QObject::tr ( "精度计算失败：%1" ).arg ( QString::fromUtf8 ( e.what () ) );
  }
  return result;
}

RoofAccuracy::Result
measure ( const BuildingRoof::Mesh &mesh, const EvaluationPoints &points, const std::function<bool ()> &canceled,
          const std::function<void ( double )> &progress )
{
  RoofAccuracy::Result result;
  auto stop = [&] ()
  {
    result.canceled = canceled && canceled ();
    return result.canceled;
  };
  if ( stop () )
    return result;
  if ( !points.error.isEmpty () )
  {
    result.error = points.error;
    return result;
  }
  if ( mesh.isEmpty () || points.points.isEmpty () )
  {
    result.error = QObject::tr ( "缺少有效的屋顶模型或评价点云。" );
    return result;
  }
  try
  {
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
    const qsizetype count = points.points.size ();
    result.points.reserve ( count );
    long double sum = 0, squared = 0;
    for ( qsizetype i = 0; i < count; ++i )
    {
      if ( ( i & 127 ) == 0 )
      {
        if ( stop () )
          return result;
        if ( progress )
          progress ( 100.0 * i / count );
      }
      const auto &point = points.points.at ( i );
      const double d2 = surface.distanceSquared ( local ( point.first ) );
      if ( !std::isfinite ( d2 ) )
      {
        result.error = QObject::tr ( "点到屋面距离计算失败。" );
        return result;
      }
      const double d = std::sqrt ( std::max ( 0.0, d2 ) );
      result.points.append ( { point.second, d } );
      sum += d;
      squared += d2;
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
}

RoofAccuracy::Result
RoofAccuracy::evaluate ( const QgsGeometry &footprint, const BuildingRoof::Mesh &mesh,
                         const QVector<BuildingRoof::RoofSample> &samples,
                         const QVector<QgsPoint> &displayPoints, const std::function<bool ()> &canceled,
                         const std::function<void ( double )> &progress )
{
  if ( footprint.isEmpty () || mesh.isEmpty () || samples.isEmpty ()
       || ( !displayPoints.isEmpty () && displayPoints.size () != samples.size () ) )
  {
    Result result;
    result.error = QObject::tr ( "缺少有效的屋顶模型或评价点云。" );
    return result;
  }
  return measure ( mesh, preparePoints ( footprint, samples, displayPoints, canceled ), canceled, progress );
}

struct RoofAccuracy::Controller::Private
{
  struct Inputs
  {
    QgsGeometry footprint;
    QVector<BuildingRoof::RoofSample> samples;
    QVector<QgsPoint> display;
    std::function<QgsPoint ( const QgsPoint & )> toMetric;
  };
  enum class CoordinateMode { LocalMeters, Geographic };

  Controller *owner;
  QPointer<QWidget> panel;
  Ui::roofedit &ui;
  QPointer<QgisInterface> iface;
  QPointer<QgsVectorLayer> layer, displayLayer;
  QgsFeatureId fid = FID_NULL;
  RoofModelSettings::Selection selection;
  BuildingRoof::Mesh previewMesh;
  Result result;
  QPointer<AccuracyTask> task;
  QTimer timer;
  bool pending = false;
  bool preparing = false;
  bool needsCoordinates = false;
  quint64 inputRevision = 0, modelRevision = 0;
  std::optional<Inputs> inputs;
  std::shared_ptr<const EvaluationPoints> evaluationPoints;
  QList<QMetaObject::Connection> inputConnections;
  QHash<QString, CoordinateMode> coordinateModes;
  QString coordinateNote, status;
  QVector<QgsFeatureId> displayOrder;
  QMetaObject::Connection displayDestroyed;

  Private ( Controller *owner, QWidget *panel, Ui::roofedit &ui, QgisInterface *iface )
      : owner ( owner ), panel ( panel ), ui ( ui ), iface ( iface )
  {
    timer.setInterval ( 120 );
    timer.setSingleShot ( true );
    QObject::connect ( &timer, &QTimer::timeout, owner, [this] () { calculate ( false ); } );
  }

  QgsPointCloudLayer *cloud () const
  {
    return qobject_cast<QgsPointCloudLayer *> ( QgsProject::instance ()->mapLayer ( selection.pointCloudLayerId ) );
  }

  bool canCalculate () const
  {
    const auto *source = cloud ();
    return layer && fid != FID_NULL && selection.hasRoof () && source && source->isValid () && source->dataProvider ();
  }

  void update ()
  {
    if ( !panel )
      return;
    const bool valid = !result.points.isEmpty ();
    const QString empty = QStringLiteral ( "—" );
    ui.accuracyPointCountValueLabel->setText ( valid ? QString::number ( result.points.size () ) : empty );
    ui.accuracyMaeValueLabel->setText ( valid ? QString::number ( result.mae, 'f', 3 ) : empty );
    ui.accuracyRmseValueLabel->setText ( valid ? QString::number ( result.rmse, 'f', 3 ) : empty );
    ui.accuracyInlierRateValueLabel->setText (
        valid ? QString::number ( result.inlierRate ( ui.accuracyToleranceSpinBox->value () ), 'f', 2 ) : empty );
    const bool busy = pending || ( task && !task->isCanceled () );
    ui.calculateAccuracyButton->setEnabled ( canCalculate () && !preparing );
    ui.calculateAccuracyButton->setText ( busy ? QObject::tr ( "取消计算" )
                                          : needsCoordinates ? QObject::tr ( "确认坐标并计算" ) : QObject::tr ( "计算精度" ) );
    ui.showAccuracyIn3DCheckBox->setEnabled ( valid || ui.showAccuracyIn3DCheckBox->isChecked () );
    QString note = valid ? QObject::tr ( "%1\n误差为点到屋面的最短三维距离。" ).arg ( coordinateNote )
                         : QObject::tr ( "选定屋顶和点云后自动计算精度。" );
    if ( busy && valid )
      note += QObject::tr ( "\n正在更新精度，当前显示最近一次完成的结果。" );
    if ( !status.isEmpty () )
      note += QLatin1Char ( '\n' ) + status;
    ui.accuracyGroupBox->setToolTip ( note );
    ui.calculateAccuracyButton->setToolTip ( note );
  }

  void clearDisplay ()
  {
    if ( !displayLayer )
      return;
    QObject::disconnect ( displayDestroyed );
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
    displayOrder.clear ();
    QgsProject::instance ()->removeMapLayer ( id );
  }

  void resetInputs ()
  {
    ++inputRevision;
    timer.stop ();
    pending = false;
    if ( task )
      task->cancel ();
    inputs.reset ();
    evaluationPoints.reset ();
    result = Result ();
    coordinateNote.clear ();
    status.clear ();
    needsCoordinates = false;
    clearDisplay ();
  }

  void schedule ()
  {
    if ( canCalculate () && !needsCoordinates )
    {
      const bool alreadyScheduled = pending && ( task || timer.isActive () );
      pending = true;
      if ( !task && !timer.isActive () && !preparing )
        timer.start ();
      if ( alreadyScheduled )
        return;
    }
    update ();
  }

  void watchInputs ()
  {
    for ( const auto &connection : inputConnections )
      QObject::disconnect ( connection );
    inputConnections.clear ();
    const auto refresh = [this] ()
    {
      previewMesh = {};
      resetInputs ();
      schedule ();
    };
    const auto removed = [this] ()
    {
      resetInputs ();
      update ();
    };
    for ( QgsMapLayer *input : { static_cast<QgsMapLayer *> ( layer.data () ), static_cast<QgsMapLayer *> ( cloud () ) } )
    {
      if ( !input )
        continue;
      inputConnections.append ( QObject::connect ( input, &QgsMapLayer::dataChanged, owner, refresh ) );
      inputConnections.append ( QObject::connect ( input, &QgsMapLayer::dataSourceChanged, owner, [this, refresh] ()
      {
        watchInputs ();
        refresh ();
      } ) );
      inputConnections.append ( QObject::connect ( input, &QgsMapLayer::crsChanged, owner, refresh ) );
      inputConnections.append ( QObject::connect ( input, &QObject::destroyed, owner, removed ) );
    }
    if ( layer )
    {
      inputConnections.append ( QObject::connect ( layer, &QgsVectorLayer::geometryChanged, owner,
                                                   [this, refresh] ( QgsFeatureId changed, const QgsGeometry & )
                                                   {
                                                     if ( changed == fid )
                                                       refresh ();
                                                   } ) );
      inputConnections.append ( QObject::connect ( layer, &QgsVectorLayer::featuresDeleted, owner,
                                                   [this, removed] ( const QgsFeatureIds &deleted )
                                                   {
                                                     if ( deleted.contains ( fid ) )
                                                     {
                                                       fid = FID_NULL;
                                                       removed ();
                                                     }
                                                   } ) );
    }
    if ( auto *source = cloud () )
    {
      inputConnections.append ( QObject::connect ( source, &QgsPointCloudLayer::subsetStringChanged, owner, refresh ) );
      if ( source->dataProvider () )
        inputConnections.append ( QObject::connect ( source->dataProvider (), &QgsPointCloudDataProvider::indexGenerationStateChanged,
                                                     owner, refresh ) );
    }
  }

  bool
  metricInputs ( QgsPointCloudLayer *cloud, Inputs &input, bool manual )
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
      auto mode = coordinateModes.constFind ( key );
      const QgsRectangle cloudExtent = cloud->extent ();
      const bool plausibleGeographic = shared && vectorCrs.isGeographic () && cloudExtent.xMinimum () >= -180
                                       && cloudExtent.xMaximum () <= 180 && cloudExtent.yMinimum () >= -90
                                       && cloudExtent.yMaximum () <= 90;
      if ( mode == coordinateModes.cend () )
      {
        needsCoordinates = true;
        if ( !manual )
          return false;
        const quint64 revision = inputRevision;
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
        if ( revision != inputRevision )
          return false;
        if ( choice.clickedButton () == localButton )
          mode = coordinateModes.insert ( key, CoordinateMode::LocalMeters );
        else if ( geographicButton && choice.clickedButton () == geographicButton )
          mode = coordinateModes.insert ( key, CoordinateMode::Geographic );
        else
          return false;
        needsCoordinates = false;
      }
      if ( mode.value () == CoordinateMode::LocalMeters )
        coordinateNote = QObject::tr ( "本地坐标：X/Y/Z 按米计算，使用原始坐标对齐" );
      else
      {
        const QgsPointXY center = input.footprint.boundingBox ().center ();
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

    input.toMetric = [transform, toMetric, xyFactor, zFactor] ( const QgsPoint &point )
    {
      const QgsPointXY xy = transform ? toMetric.transform ( QgsPointXY ( point.x (), point.y () ) )
                                      : QgsPointXY ( point.x (), point.y () );
      return QgsPoint ( xy.x () * xyFactor, xy.y () * xyFactor, point.z () * zFactor );
    };
    if ( transform || xyFactor != 1 )
      input.footprint.transformVertices ( input.toMetric );
    return true;
  }

  void reportError ( const QString &error, bool manual )
  {
    status = error;
    result = {};
    clearDisplay ();
    update ();
    if ( manual && panel && !error.isEmpty () )
      QMessageBox::warning ( panel, QObject::tr ( "精度计算" ), error );
  }

  void calculate ( bool manual )
  {
    if ( preparing )
      return;
    if ( task || ( manual && pending ) )
    {
      if ( manual )
      {
        pending = false;
        timer.stop ();
        if ( task )
          task->cancel ();
        update ();
      }
      return;
    }
    timer.stop ();
    pending = false;
    if ( !canCalculate () )
    {
      update ();
      return;
    }

    preparing = true;
    const auto donePreparing = qScopeGuard ( [this] ()
    {
      preparing = false;
      if ( pending )
        schedule ();
      else
        update ();
    } );
    const quint64 revision = inputRevision;
    status.clear ();
    try
    {
      const QgsFeature feature = layer->getFeature ( fid );
      if ( !feature.isValid () )
      {
        reportError ( QObject::tr ( "当前建筑不可用。" ), manual );
        return;
      }
      if ( !inputs )
      {
        Inputs input;
        input.footprint = feature.geometry ();
        if ( !metricInputs ( cloud (), input, manual ) )
          return;
        bool complete = false;
        input.samples = RoofModelPipeline::collectSamples ( feature.geometry (), selection.pointCloudLayerId, &complete );
        if ( revision != inputRevision )
          return;
        if ( !complete || input.samples.isEmpty () )
        {
          reportError ( !complete ? QObject::tr ( "点云索引未就绪或读取不完整。" )
                                 : QObject::tr ( "建筑轮廓内未读到点云。" ), manual );
          return;
        }
        if ( input.toMetric )
        {
          input.display.reserve ( input.samples.size () );
          for ( auto &sample : input.samples )
          {
            input.display.append ( sample.point );
            sample.point = input.toMetric ( sample.point );
          }
        }
        inputs = std::move ( input );
      }
      BuildingRoof::Mesh mesh = previewMesh;
      if ( mesh.isEmpty () )
      {
        auto model = RoofModelPipeline::build ( feature.geometry (), selection );
        if ( revision != inputRevision )
          return;
        if ( !model.success )
        {
          reportError ( model.error, manual );
          return;
        }
        mesh = std::move ( model.mesh );
      }
      if ( inputs->toMetric )
      {
        for ( auto &point : mesh.vertices )
          point = inputs->toMetric ( point );
      }
      auto *next = new AccuracyTask ( inputs->footprint, std::move ( mesh ), inputs->samples, inputs->display, evaluationPoints );
      next->setDependentLayers ( { layer.data (), cloud () } );
      task = next;
      const quint64 evaluatedModel = modelRevision;
      const auto finished = [this, next, revision, evaluatedModel, manual] ()
      {
        task = nullptr;
        if ( revision == inputRevision && !next->isCanceled () )
        {
          evaluationPoints = next->prepared;
          if ( !next->result.points.isEmpty () && next->result.error.isEmpty () )
          {
            result = std::move ( next->result );
            if ( panel && ui.showAccuracyIn3DCheckBox->isChecked () )
              show3D ( true );
          }
          else if ( evaluatedModel == modelRevision )
            reportError ( next->result.error, manual );
        }
        if ( pending )
          schedule ();
        else
          update ();
      };
      QObject::connect ( next, &QgsTask::taskCompleted, owner, finished );
      QObject::connect ( next, &QgsTask::taskTerminated, owner, finished );
      QgsApplication::taskManager ()->addTask ( next );
    }
    catch ( const QgsException &e )
    {
      reportError ( e.what (), manual );
    }
    catch ( const std::exception &e )
    {
      reportError ( QString::fromUtf8 ( e.what () ), manual );
    }
  }

  void
  show3D ( bool enabled, bool manual = false )
  {
    if ( !enabled )
    {
      clearDisplay ();
      return;
    }
    if ( result.points.isEmpty () || !evaluationPoints || !layer || !iface )
      return;
    const auto &displayIndices = evaluationPoints->displayIndices;
    const auto displayError = [this, manual] ( const QString &error )
    {
      const QSignalBlocker blocker ( ui.showAccuracyIn3DCheckBox );
      ui.showAccuracyIn3DCheckBox->setChecked ( false );
      status = error;
      update ();
      if ( manual )
        QMessageBox::warning ( panel, QObject::tr ( "精度显示" ), error );
    };
    if ( displayLayer && displayLayer->dataProvider () && displayOrder.size () == displayIndices.size () )
    {
      QgsChangedAttributesMap changes;
      const int field = displayLayer->fields ().lookupField ( QStringLiteral ( "distance_m" ) );
      for ( qsizetype i = 0; i < displayOrder.size (); ++i )
        changes[displayOrder[i]].insert ( field, result.points[displayIndices[i]].distance );
      if ( field >= 0 && displayLayer->dataProvider ()->changeAttributeValues ( changes ) )
      {
        styleLayer ( displayLayer, result, displayIndices, ui.accuracyToleranceSpinBox->value () );
        return;
      }
    }
    clearDisplay ();
    auto points = std::make_unique<QgsVectorLayer> (
        QStringLiteral ( "PointZ?field=building_fid:long&field=distance_m:double" ), QObject::tr ( "屋顶精度误差点" ),
        QStringLiteral ( "memory" ) );
    points->setCrs ( layer->crs () );
    points->setCustomProperty ( QStringLiteral ( "building_edit_manager/accuracy_points" ), true );
    QgsFeatureList features;
    features.reserve ( displayIndices.size () );
    for ( const qsizetype index : displayIndices )
    {
      const auto &p = result.points[index];
      QgsFeature f ( points->fields () );
      f.setGeometry ( QgsGeometry::fromPoint ( p.point ) );
      f.setAttributes ( QgsAttributes () << fid << p.distance );
      features.append ( f );
    }
    if ( !points->isValid () || !points->dataProvider ()->addFeatures ( features ) )
    {
      displayError ( QObject::tr ( "无法生成误差点图层。" ) );
      return;
    }
    points->updateExtents ();
    styleLayer ( points.get (), result, displayIndices, ui.accuracyToleranceSpinBox->value () );
    displayOrder.reserve ( features.size () );
    for ( const QgsFeature &feature : features )
      displayOrder.append ( feature.id () );
    displayLayer = points.release ();
    displayDestroyed = QObject::connect ( displayLayer, &QObject::destroyed, owner,
                       [this] ()
                       {
                         displayLayer = nullptr;
                         displayOrder.clear ();
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
      displayError ( QObject::tr ( "请先打开三维视图后重试。" ) );
    }
  }
};

RoofAccuracy::Controller::Controller ( QWidget *panel, Ui::roofedit &ui, QgisInterface *iface, QObject *parent )
    : QObject ( parent ), d ( std::make_unique<Private> ( this, panel, ui, iface ) )
{
  connect ( ui.calculateAccuracyButton, &QPushButton::clicked, this, [this] () { d->calculate ( true ); } );
  connect ( ui.showAccuracyIn3DCheckBox, &QCheckBox::toggled, this,
            [this] ( bool enabled ) { d->show3D ( enabled, true ); } );
  connect ( ui.accuracyToleranceSpinBox, qOverload<double> ( &QDoubleSpinBox::valueChanged ), this,
            [this] ( double tolerance )
            {
              d->update ();
              if ( d->displayLayer && d->evaluationPoints )
                styleLayer ( d->displayLayer, d->result, d->evaluationPoints->displayIndices, tolerance );
            } );
  connect ( QgsProject::instance (), &QgsProject::layersRemoved, this,
            [this] ()
            {
              if ( !d->selection.pointCloudLayerId.isEmpty () && !d->cloud () )
              {
                d->resetInputs ();
                d->update ();
              }
            } );
  ui.accuracyPointCountLabel->setToolTip (
      tr ( "统计点云分割识别的屋面点，逐点计算到屋顶模型的距离。" ) );
  ui.showAccuracyIn3DCheckBox->setToolTip (
      tr ( "绿：≤0.5倍允许误差；黄绿：≤1倍；橙：≤2倍；红：>2倍。模型遮挡点时可开启建筑物三角网。" ) );
  d->update ();
}

RoofAccuracy::Controller::~Controller ()
{
  if ( const auto task = d->task )
    QObject::disconnect ( task, nullptr, this, nullptr );
  clear ();
  if ( d->task )
    d->task->waitForFinished ( 0 );
}

void
RoofAccuracy::Controller::setContext ( QgsVectorLayer *layer, QgsFeatureId fid,
                                       const RoofModelSettings::Selection &selection, const BuildingRoof::Mesh *previewMesh )
{
  const bool inputChanged = d->layer != layer || d->fid != fid || d->selection.pointCloudLayerId != selection.pointCloudLayerId;
  if ( !inputChanged && d->selection == selection )
  {
    if ( previewMesh && d->previewMesh.isEmpty () )
      d->previewMesh = *previewMesh;
    return;
  }

  if ( inputChanged )
    d->resetInputs ();
  d->layer = layer;
  d->fid = fid;
  d->selection = selection;
  ++d->modelRevision;
  d->previewMesh = previewMesh ? *previewMesh : BuildingRoof::Mesh{};
  if ( inputChanged )
    d->watchInputs ();
  if ( !d->canCalculate () && !inputChanged )
    d->resetInputs ();
  d->schedule ();
}

void
RoofAccuracy::Controller::clear ()
{
  d->resetInputs ();
  d->layer = nullptr;
  d->fid = FID_NULL;
  d->selection = {};
  d->previewMesh = {};
  d->watchInputs ();
  if ( d->panel )
  {
    const QSignalBlocker blocker ( d->ui.showAccuracyIn3DCheckBox );
    d->ui.showAccuracyIn3DCheckBox->setChecked ( false );
  }
  d->update ();
}
