#include "threedviewtool.h"
#include "buildingeditpreviewbus.h"
#include "buildingroof.h"
#include <qgsvectorlayer.h>
#include <qgspointcloudlayer.h>
#include <qgspointcloudattribute.h>
#include <qgspointcloudblock.h>
#include <qgspointcloudindex.h>
#include <qgspointcloudrequest.h>
#include <qgsrasterlayer.h>
#include <qgsproject.h>
#include <qgsmapmouseevent.h>
#include <qgsfeatureiterator.h>
#include <qgsfield.h>
#include <qgsgeometry.h>
#include <qgsmapcanvas.h>
#include <qgsrectangle.h>
#include <qgscoordinatetransform.h>
#include <qgsprovidermetadata.h>
#include <qgsproviderregistry.h>
#include <QMessageBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QKeyEvent>
#include <QCryptographicHash>
#include <cmath>

// 基础几何
#include <qgspolygon.h>
#include <qgslinestring.h>
#include <qgspoint.h>

// 3D 渲染模块
#include <qgspolygon3dsymbol.h>
#include <qgspointcloudlayer3drenderer.h>
#include <qgspointcloud3dsymbol.h>
#include <qgspointcloudlayerelevationproperties.h>
#include <qgscolorrampshader.h>
#include <qgsline3dsymbol.h>
#include <qgsvectorlayer3drenderer.h>
#include <qgsphongmaterialsettings.h>
#include <qgs3dtypes.h>

#include <qgsgeometrycollection.h>
#include <qgisinterface.h>
#include <qgs3dmapcanvas.h>
#include <qgs3dmapsettings.h>
#include <qgs3dmapscene.h>
#include <qgsvector3d.h>
#include <qgsnullsymbolrenderer.h>
#include <qgslayertreelayer.h>
#include <qgslayertree.h>
#include <qgslayertreeview.h>
#include <QMap>
#include <QHash>
#include <QSet>
#include <QVector3D>
#include <QtMath>
#include <cstring>
#include <limits>
#include <Qt3DCore/QEntity>
#include <Qt3DRender/QAttribute>
#include <Qt3DRender/QBuffer>
#include <Qt3DRender/QGeometry>
#include <Qt3DRender/QGeometryRenderer>
#include <Qt3DExtras/QPhongMaterial>

// ==================== 构造与析构 ====================
namespace
{
  QString buildingShapeSignature( const QgsGeometry &geometry )
  {
    if ( geometry.isNull() || geometry.isEmpty() )
      return QString();

    QgsPolygonXY polygon = geometry.asPolygon();
    if ( polygon.isEmpty() )
    {
      const QgsMultiPolygonXY multiPolygon = geometry.asMultiPolygon();
      if ( !multiPolygon.isEmpty() )
        polygon = multiPolygon.first();
    }
    if ( polygon.isEmpty() || polygon.first().size() < 3 )
      return QString();

    const QgsRectangle bounds = geometry.boundingBox();
    QByteArray bytes;
    bytes.reserve( polygon.first().size() * 32 );
    bytes.append( QByteArray::number( qRound64( bounds.width() * 10000.0 ) ) );
    bytes.append( ':' );
    bytes.append( QByteArray::number( qRound64( bounds.height() * 10000.0 ) ) );
    bytes.append( ';' );

    for ( const QgsPointXY &point : polygon.first() )
    {
      bytes.append( QByteArray::number( qRound64( ( point.x() - bounds.xMinimum() ) * 10000.0 ) ) );
      bytes.append( ',' );
      bytes.append( QByteArray::number( qRound64( ( point.y() - bounds.yMinimum() ) * 10000.0 ) ) );
      bytes.append( ';' );
    }

    return QString::fromLatin1( QCryptographicHash::hash( bytes, QCryptographicHash::Md5 ).toHex() );
  }

  QVector<QgsPoint> wireframeFootprintRing( const QgsGeometry &geometry )
  {
    QVector<QgsPoint> ring;
    if ( geometry.isNull() || geometry.isEmpty() )
      return ring;

    QgsPolygonXY polygon = geometry.asPolygon();
    if ( polygon.isEmpty() )
    {
      const QgsMultiPolygonXY multiPolygon = geometry.asMultiPolygon();
      if ( !multiPolygon.isEmpty() )
        polygon = multiPolygon.first();
    }
    if ( polygon.isEmpty() || polygon.first().size() < 3 )
      return ring;

    const QVector<QgsPointXY> exterior = polygon.first();
    const int count = exterior.size() > 1 && exterior.first() == exterior.last() ? exterior.size() - 1 : exterior.size();
    ring.reserve( count );
    for ( int i = 0; i < count; ++i )
      ring.append( QgsPoint( exterior.at( i ).x(), exterior.at( i ).y(), 0.0 ) );
    return ring;
  }

  struct WireframeEdgeRecord
  {
      QgsPoint a;
      QgsPoint b;
      QVector3D normal;
      int count = 0;
      double minNormalDot = 1.0;
  };

  QString wireframeVertexKey( const QgsPoint &point )
  {
    return QStringLiteral( "%1,%2,%3" )
      .arg( qRound64( point.x() * 1000.0 ) )
      .arg( qRound64( point.y() * 1000.0 ) )
      .arg( qRound64( point.z() * 1000.0 ) );
  }

  QString wireframeEdgeKey( const QgsPoint &a, const QgsPoint &b )
  {
    const QString ka = wireframeVertexKey( a );
    const QString kb = wireframeVertexKey( b );
    return ka < kb ? ka + QLatin1Char( '|' ) + kb : kb + QLatin1Char( '|' ) + ka;
  }

  QVector3D triangleNormal( const QgsPoint &a, const QgsPoint &b, const QgsPoint &c )
  {
    QVector3D u( b.x() - a.x(), b.y() - a.y(), b.z() - a.z() );
    QVector3D v( c.x() - a.x(), c.y() - a.y(), c.z() - a.z() );
    QVector3D n = QVector3D::crossProduct( u, v );
    if ( n.lengthSquared() > 1e-12f )
      n.normalize();
    return n;
  }

  double wireframeCross2D( const QgsPoint &o, const QgsPoint &a, const QgsPoint &b )
  {
    return ( a.x() - o.x() ) * ( b.y() - o.y() ) - ( a.y() - o.y() ) * ( b.x() - o.x() );
  }

  double distancePointToSegment2D( double px, double py, const QgsPoint &a, const QgsPoint &b )
  {
    const double vx = b.x() - a.x();
    const double vy = b.y() - a.y();
    const double wx = px - a.x();
    const double wy = py - a.y();
    const double len2 = vx * vx + vy * vy;
    if ( len2 < 1e-12 )
      return std::hypot( px - a.x(), py - a.y() );

    const double t = std::max( 0.0, std::min( 1.0, ( wx * vx + wy * vy ) / len2 ) );
    const double projX = a.x() + t * vx;
    const double projY = a.y() + t * vy;
    return std::hypot( px - projX, py - projY );
  }

  double distancePointToHull2D( double px, double py, const QVector<QgsPoint> &hull )
  {
    if ( hull.size() < 2 )
      return std::numeric_limits<double>::max();

    double best = std::numeric_limits<double>::max();
    for ( int i = 0; i < hull.size(); ++i )
      best = std::min( best, distancePointToSegment2D( px, py, hull.at( i ), hull.at( ( i + 1 ) % hull.size() ) ) );
    return best;
  }

  QVector<QgsPoint> convexHullByXY( QVector<QgsPoint> points )
  {
    if ( points.size() < 3 )
      return points;

    std::sort( points.begin(), points.end(), []( const QgsPoint &a, const QgsPoint &b ) {
      if ( !qgsDoubleNear( a.x(), b.x(), 1e-9 ) )
        return a.x() < b.x();
      return a.y() < b.y();
    } );

    QVector<QgsPoint> uniquePoints;
    uniquePoints.reserve( points.size() );
    for ( const QgsPoint &point : std::as_const( points ) )
    {
      if ( uniquePoints.isEmpty() || !qgsDoubleNear( uniquePoints.last().x(), point.x(), 1e-6 ) || !qgsDoubleNear( uniquePoints.last().y(), point.y(), 1e-6 ) )
        uniquePoints.append( point );
    }

    if ( uniquePoints.size() < 3 )
      return uniquePoints;

    QVector<QgsPoint> lower;
    for ( const QgsPoint &point : std::as_const( uniquePoints ) )
    {
      while ( lower.size() >= 2 && wireframeCross2D( lower.at( lower.size() - 2 ), lower.last(), point ) <= 1e-9 )
        lower.removeLast();
      lower.append( point );
    }

    QVector<QgsPoint> upper;
    for ( int i = uniquePoints.size() - 1; i >= 0; --i )
    {
      const QgsPoint &point = uniquePoints.at( i );
      while ( upper.size() >= 2 && wireframeCross2D( upper.at( upper.size() - 2 ), upper.last(), point ) <= 1e-9 )
        upper.removeLast();
      upper.append( point );
    }

    lower.removeLast();
    upper.removeLast();
    lower += upper;
    return lower;
  }

  void appendWireframeLine( QgsFeatureList &lines, QSet<QString> &emittedKeys, QgsFeatureId fid, const QgsPoint &a, const QgsPoint &b )
  {
    const double dx = a.x() - b.x();
    const double dy = a.y() - b.y();
    const double dz = a.z() - b.z();
    if ( dx * dx + dy * dy + dz * dz < 1e-12 )
      return;

    const QString key = wireframeEdgeKey( a, b );
    if ( emittedKeys.contains( key ) )
      return;

    emittedKeys.insert( key );
    QgsLineString *line = new QgsLineString();
    line->setPoints( QgsPointSequence() << a << b );

    QgsFeature feature;
    feature.setGeometry( QgsGeometry( line ) );
    feature.setAttributes( QgsAttributes() << fid );
    lines.append( feature );
  }

  void collectPointCloudNodesForRoof( const QgsPointCloudIndex &index, const QgsPointCloudNodeId &nodeId, QList<QgsPointCloudNodeId> &nodes )
  {
    if ( !nodeId.isValid() )
      return;

    nodes.append( nodeId );
    if ( nodeId.d() > 20 )
      return;

    for ( int i = 0; i < 8; ++i )
    {
      QgsPointCloudNodeId childId(
        nodeId.d() + 1,
        ( nodeId.x() << 1 ) + ( i & 1 ),
        ( nodeId.y() << 1 ) + ( ( i >> 1 ) & 1 ),
        ( nodeId.z() << 1 ) + ( ( i >> 2 ) & 1 )
      );
      if ( index.hasNode( childId ) )
        collectPointCloudNodesForRoof( index, childId, nodes );
    }
  }

  QVector<BuildingRoof::RoofSample> collectRoofSamplesFromFirstPointCloud( const QgsGeometry &geometry, int maxSamples = 20000 )
  {
    QVector<BuildingRoof::RoofSample> samples;
    if ( geometry.isNull() || geometry.isEmpty() )
      return samples;

    QgsPointCloudLayer *pointCloudLayer = nullptr;
    const auto layers = QgsProject::instance()->mapLayers().values();
    for ( QgsMapLayer *layer : layers )
    {
      pointCloudLayer = qobject_cast<QgsPointCloudLayer *>( layer );
      if ( pointCloudLayer && pointCloudLayer->dataProvider() )
        break;
      pointCloudLayer = nullptr;
    }
    if ( !pointCloudLayer )
      return samples;

    QgsPointCloudIndex index = pointCloudLayer->dataProvider()->index();
    const QgsRectangle extent = geometry.boundingBox();
    QList<QgsPointCloudNodeId> nodeIds;
    collectPointCloudNodesForRoof( index, index.root(), nodeIds );
    nodeIds = nodeIds.toSet().toList();

    QgsPointCloudRequest request;
    request.setFilterRect( extent );
    const QgsPointCloudAttributeCollection attributes = index.attributes();
    request.setAttributes( attributes );
    const int recordSize = attributes.pointRecordSize();
    const double xScale = index.scale().x();
    const double yScale = index.scale().y();
    const double zScale = index.scale().z();
    const double xOffset = index.offset().x();
    const double yOffset = index.offset().y();
    const double zOffset = index.offset().z();

    QVector<BuildingRoof::RoofSample> allSamples;
    for ( const QgsPointCloudNodeId &nodeId : nodeIds )
    {
      std::unique_ptr<QgsPointCloudBlock> block( index.nodeData( nodeId, request ) );
      if ( !block )
        continue;

      const char *data = block->data();
      for ( int i = 0; i < block->pointCount(); ++i )
      {
        const char *ptr = data + i * recordSize;
        int32_t ix = 0;
        int32_t iy = 0;
        int32_t iz = 0;
        std::memcpy( &ix, ptr, 4 );
        std::memcpy( &iy, ptr + 4, 4 );
        std::memcpy( &iz, ptr + 8, 4 );

        const double x = ix * xScale + xOffset;
        const double y = iy * yScale + yOffset;
        const QgsPointXY xy( x, y );
        if ( !geometry.contains( QgsGeometry::fromPointXY( xy ) ) )
          continue;

        allSamples.append( BuildingRoof::RoofSample{ QgsPoint( x, y, iz * zScale + zOffset ) } );
      }
    }

    if ( maxSamples <= 0 || allSamples.size() <= maxSamples )
      return allSamples;

    const int stride = std::max( 1, allSamples.size() / maxSamples );
    for ( int i = 0; i < allSamples.size(); i += stride )
    {
      samples.append( allSamples.at( i ) );
      if ( samples.size() >= maxSamples )
        break;
    }
    return samples;
  }
}

ThreeDViewTool::ThreeDViewTool( QgsMapCanvas *canvas, QgisInterface *iface )
  : QgsMapTool( canvas )
  , mCanvas( canvas )
  , mIface( iface )
{
  setCursor( Qt::CrossCursor );

  mFeatureUpdateTimer = new QTimer( this );
  mFeatureUpdateTimer->setSingleShot( true );
  mFeatureUpdateTimer->setInterval( 33 );
  connect( mFeatureUpdateTimer, &QTimer::timeout, this, &ThreeDViewTool::flushPendingFeatureUpdates );
  connect( BuildingEditPreviewBus::instance(), &BuildingEditPreviewBus::heightPreviewChanged, this, &ThreeDViewTool::onHeightPreviewChanged );
  connect( BuildingEditPreviewBus::instance(), &BuildingEditPreviewBus::heightPreviewFinished, this, &ThreeDViewTool::onHeightPreviewFinished );
  connect( BuildingEditPreviewBus::instance(), &BuildingEditPreviewBus::roofModelChanged, this, &ThreeDViewTool::onRoofModelChanged );
  connect( BuildingEditPreviewBus::instance(), &BuildingEditPreviewBus::buildingTriangleMeshModeChanged, this, &ThreeDViewTool::onBuildingTriangleMeshModeChanged );

  auto layers = QgsProject::instance()->layers<QgsVectorLayer *>();
  if ( !layers.isEmpty() )
    mActiveLayer = layers.first();
}

ThreeDViewTool::~ThreeDViewTool()
{
  cleanup3DState();

  if ( mWidget )
  {
    mWidget->removeEventFilter( this );
    mWidget->deleteLater();
    mWidget = nullptr;
  }
}

void ThreeDViewTool::cleanup3DState()
{
  clearPreviewEntity();
  clearWireframeLayer();

  if ( mActiveLayer )
    disconnect( mActiveLayer, nullptr, this, nullptr );

  if ( mTempLayer )
  {
    for ( Qgs3DMapCanvas *canvas3D : mIface ? mIface->mapCanvases3D() : QList<Qgs3DMapCanvas *>() )
    {
      if ( !canvas3D || !canvas3D->mapSettings() )
        continue;

      QList<QgsMapLayer *> layers = canvas3D->mapSettings()->layers();
      if ( layers.removeAll( mTempLayer ) > 0 )
        canvas3D->mapSettings()->setLayers( layers );
    }

    const QString tempLayerId = mTempLayer->id();
    mTempLayer = nullptr;
    QgsProject::instance()->removeMapLayer( tempLayerId );
  }

  refresh3DCanvases();
}

void ThreeDViewTool::refresh3DCanvases()
{
  if ( !mIface )
    return;

  for ( Qgs3DMapCanvas *canvas3D : mIface->mapCanvases3D() )
  {
    if ( canvas3D && canvas3D->mapSettings() )
    {
      const QList<QgsMapLayer *> layers = canvas3D->mapSettings()->layers();
      canvas3D->mapSettings()->setLayers( layers );
    }
  }
}

void ThreeDViewTool::applyBuildingTriangleMeshMode()
{
  if ( !mTempLayer )
    return;

  QgsPolygon3DSymbol *symbol = new QgsPolygon3DSymbol();
  symbol->setAltitudeClamping( Qgis::AltitudeClamping::Absolute );
  symbol->setEdgesEnabled( false );

  QgsPhongMaterialSettings *material = new QgsPhongMaterialSettings();
  if ( mBuildingTriangleMeshMode )
  {
    material->setAmbient( QColor( 210, 210, 210 ) );
    material->setDiffuse( QColor( 225, 225, 225 ) );
    material->setSpecular( QColor( 255, 255, 255 ) );
    material->setOpacity( 0.08 );
  }
  else
  {
    material->setAmbient( QColor( 80, 80, 80 ) );
    material->setDiffuse( QColor( 120, 120, 120 ) );
    material->setSpecular( QColor( 190, 190, 190 ) );
    material->setOpacity( 1.0 );
  }
  symbol->setMaterialSettings( material );

  QgsVectorLayer3DRenderer *renderer = new QgsVectorLayer3DRenderer();
  renderer->setLayer( mTempLayer );
  renderer->setSymbol( symbol );
  mTempLayer->setRenderer3D( renderer );
  mTempLayer->triggerRepaint();
  refresh3DCanvases();
}

void ThreeDViewTool::ensureWireframeLayer()
{
  if ( mWireframeLayer )
    return;

  const QString crs = mActiveLayer && mActiveLayer->crs().isValid() ? mActiveLayer->crs().authid() : QStringLiteral( "EPSG:4326" );
  mWireframeLayer = new QgsVectorLayer(
    QStringLiteral( "LineStringZ?crs=%1&field=original_fid:long" ).arg( crs ),
    QStringLiteral( "Building_Wireframe_Current" ),
    QStringLiteral( "memory" )
  );

  if ( !mWireframeLayer || !mWireframeLayer->isValid() )
    return;

  mWireframeLayer->setRenderer( new QgsNullSymbolRenderer() );
  mWireframeLayer->setOpacity( 0.0 );
  mWireframeLayer->setFlags( mWireframeLayer->flags() & ~QgsMapLayer::Identifiable );
  mWireframeLayer->setFlags( mWireframeLayer->flags() & ~QgsMapLayer::Searchable );

  QgsLine3DSymbol *symbol = new QgsLine3DSymbol();
  symbol->setAltitudeClamping( Qgis::AltitudeClamping::Absolute );
  symbol->setAltitudeBinding( Qgis::AltitudeBinding::Vertex );
  symbol->setRenderAsSimpleLines( true );
  symbol->setWidth( 1.0f );

  QgsPhongMaterialSettings *material = new QgsPhongMaterialSettings();
  material->setAmbient( QColor( 20, 20, 20 ) );
  material->setDiffuse( QColor( 30, 30, 30 ) );
  material->setSpecular( QColor( 80, 80, 80 ) );
  material->setOpacity( 1.0 );
  symbol->setMaterialSettings( material );

  QgsVectorLayer3DRenderer *renderer = new QgsVectorLayer3DRenderer();
  renderer->setLayer( mWireframeLayer );
  renderer->setSymbol( symbol );
  mWireframeLayer->setRenderer3D( renderer );

  QgsProject::instance()->addMapLayer( mWireframeLayer, false );
  QgsLayerTreeLayer *treeLayer = QgsProject::instance()->layerTreeRoot()->addLayer( mWireframeLayer );
  if ( treeLayer )
    treeLayer->setCustomProperty( "nodeHidden", true );

  ensureLayerIn3DView( mWireframeLayer );
}

void ThreeDViewTool::clearWireframeLayer()
{
  if ( !mWireframeLayer )
    return;

  for ( Qgs3DMapCanvas *canvas3D : mIface ? mIface->mapCanvases3D() : QList<Qgs3DMapCanvas *>() )
  {
    if ( !canvas3D || !canvas3D->mapSettings() )
      continue;

    QList<QgsMapLayer *> layers = canvas3D->mapSettings()->layers();
    if ( layers.removeAll( mWireframeLayer ) > 0 )
      canvas3D->mapSettings()->setLayers( layers );
  }

  const QString layerId = mWireframeLayer->id();
  mWireframeLayer = nullptr;
  mWireframeFid = FID_NULL;
  QgsProject::instance()->removeMapLayer( layerId );
}

QgsFeatureList ThreeDViewTool::buildSimplifiedWireframeFromMesh( const MeshData &mesh, QgsFeatureId fid, bool flatTopWireframe, bool curvedWireframe, bool eaveWireframe ) const
{
  QgsFeatureList lines;
  if ( mesh.vertices.isEmpty() || mesh.indices.size() < 3 )
    return lines;

  QSet<QString> emittedLineKeys;
  QHash<QString, int> edgeIndex;
  QVector<WireframeEdgeRecord> edges;
  double minZ = std::numeric_limits<double>::max();
  double maxZ = std::numeric_limits<double>::lowest();
  double minX = std::numeric_limits<double>::max();
  double minY = std::numeric_limits<double>::max();
  double maxX = std::numeric_limits<double>::lowest();
  double maxY = std::numeric_limits<double>::lowest();
  QVector<QgsPoint> footprintCandidates;
  QSet<QString> footprintKeys;

  for ( const QgsPoint &point : mesh.vertices )
  {
    minZ = std::min( minZ, point.z() );
    maxZ = std::max( maxZ, point.z() );
    minX = std::min( minX, point.x() );
    minY = std::min( minY, point.y() );
    maxX = std::max( maxX, point.x() );
    maxY = std::max( maxY, point.y() );

    const QString xyKey = QStringLiteral( "%1,%2" )
      .arg( qRound64( point.x() * 1000.0 ) )
      .arg( qRound64( point.y() * 1000.0 ) );
    if ( !footprintKeys.contains( xyKey ) )
    {
      footprintKeys.insert( xyKey );
      footprintCandidates.append( point );
    }
  }

  const QVector<QgsPoint> footprintHull = mesh.footprintRing.size() >= 3 ? mesh.footprintRing : convexHullByXY( footprintCandidates );
  const double footprintDiagonal = std::hypot( maxX - minX, maxY - minY );
  const double footprintBoundaryTolerance = std::max( 0.05, footprintDiagonal * 0.015 );

  auto addEdge = [&edgeIndex, &edges]( const QgsPoint &a, const QgsPoint &b, const QVector3D &normal ) {
    const QString key = wireframeEdgeKey( a, b );
    const int existingIndex = edgeIndex.value( key, -1 );
    if ( existingIndex < 0 )
    {
      WireframeEdgeRecord record;
      record.a = a;
      record.b = b;
      record.normal = normal;
      record.count = 1;
      edges.append( record );
      edgeIndex.insert( key, edges.size() - 1 );
      return;
    }

    WireframeEdgeRecord &record = edges[existingIndex];
    record.count++;
    if ( record.normal.lengthSquared() > 1e-12f && normal.lengthSquared() > 1e-12f )
    {
      const double dot = std::fabs( QVector3D::dotProduct( record.normal, normal ) );
      record.minNormalDot = std::min( record.minNormalDot, dot );
    }
  };

  for ( int i = 0; i + 2 < mesh.indices.size(); i += 3 )
  {
    const int ia = mesh.indices.at( i );
    const int ib = mesh.indices.at( i + 1 );
    const int ic = mesh.indices.at( i + 2 );
    if ( ia < 0 || ib < 0 || ic < 0 || ia >= mesh.vertices.size() || ib >= mesh.vertices.size() || ic >= mesh.vertices.size() )
      continue;

    const QgsPoint &a = mesh.vertices.at( ia );
    const QgsPoint &b = mesh.vertices.at( ib );
    const QgsPoint &c = mesh.vertices.at( ic );
    const QVector3D normal = triangleNormal( a, b, c );
    addEdge( a, b, normal );
    addEdge( b, c, normal );
    addEdge( c, a, normal );
  }

  const double zRange = maxZ - minZ;
  const double topTolerance = std::max( 0.02, zRange * 0.02 );
  QVector<QgsPoint> topPoints;
  QSet<QString> topPointKeys;
  bool hasFlatTopFace = false;
  if ( flatTopWireframe )
  {
    for ( int i = 0; i + 2 < mesh.indices.size(); i += 3 )
    {
      const int ia = mesh.indices.at( i );
      const int ib = mesh.indices.at( i + 1 );
      const int ic = mesh.indices.at( i + 2 );
      if ( ia < 0 || ib < 0 || ic < 0 || ia >= mesh.vertices.size() || ib >= mesh.vertices.size() || ic >= mesh.vertices.size() )
        continue;

      const QgsPoint &a = mesh.vertices.at( ia );
      const QgsPoint &b = mesh.vertices.at( ib );
      const QgsPoint &c = mesh.vertices.at( ic );
      if ( maxZ - a.z() <= topTolerance && maxZ - b.z() <= topTolerance && maxZ - c.z() <= topTolerance )
      {
        const QVector3D normal = triangleNormal( a, b, c );
        if ( std::fabs( normal.z() ) > 0.94 )
        {
          hasFlatTopFace = true;
          break;
        }
      }
    }
  }

  for ( const QgsPoint &point : mesh.vertices )
  {
    if ( maxZ - point.z() > topTolerance )
      continue;

    const QString key = wireframeVertexKey( point );
    if ( topPointKeys.contains( key ) )
      continue;

    topPointKeys.insert( key );
    topPoints.append( point );
  }

  QVector<const WireframeEdgeRecord *> keptEdges;
  keptEdges.reserve( edges.size() );
  const double sharpEdgeDotThreshold = 0.86;
  for ( const WireframeEdgeRecord &edge : edges )
  {
    if ( edge.count == 1 || edge.minNormalDot < sharpEdgeDotThreshold )
      keptEdges.append( &edge );
  }

  const int maxWireframeEdges = 650;
  const int stride = keptEdges.size() > maxWireframeEdges ? qCeil( static_cast<double>( keptEdges.size() ) / maxWireframeEdges ) : 1;
  for ( int i = 0; i < keptEdges.size(); ++i )
  {
    if ( stride > 1 && i % stride != 0 )
      continue;

    const WireframeEdgeRecord *edge = keptEdges.at( i );
    if ( edge->count == 1 )
    {
      const double midX = ( edge->a.x() + edge->b.x() ) * 0.5;
      const double midY = ( edge->a.y() + edge->b.y() ) * 0.5;
      const bool nearFootprintBoundary = distancePointToHull2D( midX, midY, footprintHull ) <= footprintBoundaryTolerance;
      const bool nearFlatTop = flatTopWireframe && hasFlatTopFace && maxZ - edge->a.z() <= topTolerance && maxZ - edge->b.z() <= topTolerance;
      if ( curvedWireframe && !nearFootprintBoundary && !nearFlatTop )
        continue;
    }
    appendWireframeLine( lines, emittedLineKeys, fid, edge->a, edge->b );
  }

  if ( eaveWireframe && footprintHull.size() >= 3 )
  {
    QVector<QgsPoint> eaveHull;
    eaveHull.reserve( footprintHull.size() );
    const double xyTolerance = std::max( 0.02, footprintDiagonal * 0.003 );
    const double wallTopTolerance = std::max( 0.02, zRange * 0.02 );

    for ( const QgsPoint &hullPoint : footprintHull )
    {
      bool found = false;
      QgsPoint eavePoint = hullPoint;
      double bestZ = std::numeric_limits<double>::max();
      for ( const QgsPoint &candidate : mesh.vertices )
      {
        if ( std::hypot( candidate.x() - hullPoint.x(), candidate.y() - hullPoint.y() ) > xyTolerance )
          continue;
        if ( candidate.z() <= minZ + wallTopTolerance )
          continue;
        if ( candidate.z() < bestZ )
        {
          found = true;
          bestZ = candidate.z();
          eavePoint = candidate;
        }
      }

      if ( found )
        eaveHull.append( eavePoint );
    }

    if ( eaveHull.size() >= 3 )
    {
      for ( int i = 0; i < eaveHull.size(); ++i )
        appendWireframeLine( lines, emittedLineKeys, fid, eaveHull.at( i ), eaveHull.at( ( i + 1 ) % eaveHull.size() ) );
    }

    const QVector<QgsPoint> verticalSource = eaveHull.size() >= 3 ? eaveHull : footprintHull;
    if ( verticalSource.size() >= 3 )
    {
      const int maxVerticalEdges = 12;
      const int verticalStride = verticalSource.size() > maxVerticalEdges ? qCeil( static_cast<double>( verticalSource.size() ) / maxVerticalEdges ) : 1;
      for ( int i = 0; i < verticalSource.size(); i += verticalStride )
      {
        const QgsPoint &topPoint = verticalSource.at( i );
        appendWireframeLine( lines, emittedLineKeys, fid, QgsPoint( topPoint.x(), topPoint.y(), minZ ), topPoint );
      }
    }
  }

  if ( flatTopWireframe && hasFlatTopFace && topPoints.size() >= 3 )
  {
    const QVector<QgsPoint> topHull = convexHullByXY( topPoints );
    for ( int i = 0; i < topHull.size(); ++i )
      appendWireframeLine( lines, emittedLineKeys, fid, topHull.at( i ), topHull.at( ( i + 1 ) % topHull.size() ) );

    if ( footprintHull.size() >= 3 && topHull.size() >= 3 )
    {
      for ( const QgsPoint &outerPoint2d : footprintHull )
      {
        QgsPoint outerPoint( outerPoint2d.x(), outerPoint2d.y(), maxZ );
        double bestOuterDistance2 = std::numeric_limits<double>::max();
        for ( const QgsPoint &candidate : mesh.vertices )
        {
          const double dx = candidate.x() - outerPoint2d.x();
          const double dy = candidate.y() - outerPoint2d.y();
          const double distance2 = dx * dx + dy * dy;
          if ( candidate.z() > minZ + topTolerance && distance2 < bestOuterDistance2 )
          {
            bestOuterDistance2 = distance2;
            outerPoint = candidate;
          }
        }

        double bestDistance2 = std::numeric_limits<double>::max();
        QgsPoint bestTopPoint;
        for ( const QgsPoint &topPoint : topHull )
        {
          const double dx = topPoint.x() - outerPoint2d.x();
          const double dy = topPoint.y() - outerPoint2d.y();
          const double distance2 = dx * dx + dy * dy;
          if ( distance2 < bestDistance2 )
          {
            bestDistance2 = distance2;
            bestTopPoint = topPoint;
          }
        }

        if ( std::isfinite( bestDistance2 ) )
          appendWireframeLine( lines, emittedLineKeys, fid, outerPoint, bestTopPoint );
      }
    }
  }

  if ( topPoints.size() == 1 )
  {
    QVector<const WireframeEdgeRecord *> spokeEdges;
    for ( const WireframeEdgeRecord &edge : edges )
    {
      const bool aIsTop = maxZ - edge.a.z() <= topTolerance;
      const bool bIsTop = maxZ - edge.b.z() <= topTolerance;
      if ( aIsTop == bIsTop )
        continue;
      if ( std::fabs( edge.a.z() - edge.b.z() ) < std::max( 0.05, zRange * 0.08 ) )
        continue;
      spokeEdges.append( &edge );
    }

    const int maxSpokeEdges = 2;
    const int spokeStride = spokeEdges.size() > maxSpokeEdges ? qCeil( static_cast<double>( spokeEdges.size() ) / maxSpokeEdges ) : 1;
    for ( int i = 0; i < spokeEdges.size(); ++i )
    {
      if ( spokeStride > 1 && i % spokeStride != 0 )
        continue;

      const WireframeEdgeRecord *edge = spokeEdges.at( i );
      appendWireframeLine( lines, emittedLineKeys, fid, edge->a, edge->b );
    }
  }

  return lines;
}

void ThreeDViewTool::updateWireframeLayerFromMesh( const MeshData &mesh, QgsFeatureId fid )
{
  if ( !mBuildingTriangleMeshMode || fid == FID_NULL )
    return;

  ensureWireframeLayer();
  if ( !mWireframeLayer )
    return;

  QgsFeatureList lines = buildSimplifiedWireframeFromMesh( mesh, fid, mesh.flatTopWireframe, mesh.curvedWireframe, mesh.eaveWireframe );
  mWireframeLayer->startEditing();
  mWireframeLayer->deleteFeatures( mWireframeLayer->allFeatureIds() );
  mWireframeLayer->addFeatures( lines );
  mWireframeLayer->commitChanges();
  mWireframeLayer->triggerRepaint();
  ensureLayerIn3DView( mWireframeLayer );
}

void ThreeDViewTool::updateWireframeLayer( QgsVectorLayer *layer, QgsFeatureId fid )
{
  if ( !mBuildingTriangleMeshMode || !layer || layer != mActiveLayer || fid == FID_NULL )
  {
    clearWireframeLayer();
    return;
  }

  QgsFeature feature;
  if ( !layer->getFeatures( QgsFeatureRequest( fid ) ).nextFeature( feature ) )
  {
    clearWireframeLayer();
    return;
  }

  double h = feature.attribute( mSelectedHeightField ).toDouble();
  if ( h <= 0 )
    h = 5.0;

  const MeshData mesh = buildMeshForFeature( feature, h );
  updateWireframeLayerFromMesh( mesh, fid );
}

void ThreeDViewTool::removeTempFeatures( const QgsFeatureIds &fids )
{
  if ( !mTempLayer || fids.isEmpty() )
    return;

  mTempLayer->startEditing();
  for ( QgsFeatureId fid : fids )
  {
    QgsFeatureRequest request;
    request.setFilterExpression( QString( "original_fid = %1" ).arg( fid ) );
    QgsFeatureIterator it = mTempLayer->getFeatures( request );
    QgsFeature f;
    QgsFeatureIds toDelete;
    while ( it.nextFeature( f ) )
      toDelete << f.id();
    mTempLayer->deleteFeatures( toDelete );
    if ( fid == mWireframeFid )
      clearWireframeLayer();
  }
  mTempLayer->commitChanges();
  mTempLayer->triggerRepaint();
}

QList<BuildingRoof::RoofPoint> ThreeDViewTool::roofPointsForFeature( const QgsFeature &sourceFeature, bool *fromSavedLayer )
{
  struct SavedRoofPointUpdate
  {
      QgsFeatureId fid = FID_NULL;
      QgsPoint point;
      bool updateGeometry = false;
      bool updateShape = false;
      bool updateCenter = false;
      bool updateRelative = false;
      QString shape;
      double centerX = 0.0;
      double centerY = 0.0;
      double relX = 0.0;
      double relY = 0.0;
  };

  auto collectPoints = [&sourceFeature]( const QString &layerName, bool useRelativePosition ) {
    QList<BuildingRoof::RoofPoint> points;
    const QList<QgsMapLayer *> layers = QgsProject::instance()->mapLayersByName( layerName );
    if ( layers.isEmpty() )
      return points;

    QgsVectorLayer *pointLayer = qobject_cast<QgsVectorLayer *>( layers.first() );
    if ( !pointLayer )
      return points;

    QgsFeatureRequest request;
    request.setFilterExpression( QStringLiteral( "building_fid = %1" ).arg( sourceFeature.id() ) );
    QgsFeatureIterator it = pointLayer->getFeatures( request );
    QgsFeature feature;
    const QgsRectangle bounds = sourceFeature.geometry().boundingBox();
    const int relXIndex = pointLayer->fields().indexOf( QStringLiteral( "rel_x" ) );
    const int relYIndex = pointLayer->fields().indexOf( QStringLiteral( "rel_y" ) );
    const int buildingShapeIndex = pointLayer->fields().indexOf( QStringLiteral( "building_shape" ) );
    const int buildingCenterXIndex = pointLayer->fields().indexOf( QStringLiteral( "building_center_x" ) );
    const int buildingCenterYIndex = pointLayer->fields().indexOf( QStringLiteral( "building_center_y" ) );
    const QString currentShape = buildingShapeSignature( sourceFeature.geometry() );
    QVector<SavedRoofPointUpdate> savedPointUpdates;
    while ( it.nextFeature( feature ) )
    {
      if ( !feature.hasGeometry() )
        continue;

      const QgsPoint *point = qgsgeometry_cast<const QgsPoint *>( feature.geometry().constGet() );
      if ( !point )
        continue;

      QgsPoint roofPoint = *point;
      bool updateSavedGeometry = false;
      bool updateSavedShape = false;
      bool updateSavedCenter = false;
      bool updateSavedRelative = false;
      if ( useRelativePosition )
      {
        bool hasSavedShape = false;
        if ( buildingShapeIndex >= 0 && buildingCenterXIndex >= 0 && buildingCenterYIndex >= 0 )
        {
          const QString savedShape = feature.attribute( buildingShapeIndex ).toString();
          bool okCenterX = false;
          bool okCenterY = false;
          const double savedCenterX = feature.attribute( buildingCenterXIndex ).toDouble( &okCenterX );
          const double savedCenterY = feature.attribute( buildingCenterYIndex ).toDouble( &okCenterY );
          if ( !savedShape.isEmpty() && okCenterX && okCenterY )
          {
            hasSavedShape = true;
            if ( savedShape == currentShape )
            {
              const double dx = bounds.center().x() - savedCenterX;
              const double dy = bounds.center().y() - savedCenterY;
              if ( std::fabs( dx ) > 1e-9 || std::fabs( dy ) > 1e-9 )
              {
                roofPoint = QgsPoint( point->x() + dx, point->y() + dy, roofPoint.z() );
                updateSavedGeometry = true;
                updateSavedCenter = true;
                updateSavedRelative = true;
              }
            }
            else
            {
              updateSavedShape = true;
              updateSavedCenter = true;
              updateSavedRelative = true;
            }
          }
        }

        if ( !hasSavedShape && relXIndex >= 0 && relYIndex >= 0 && bounds.width() > 1e-12 && bounds.height() > 1e-12 )
        {
          bool okX = false;
          bool okY = false;
          const double relX = feature.attribute( relXIndex ).toDouble( &okX );
          const double relY = feature.attribute( relYIndex ).toDouble( &okY );
          if ( okX && okY )
          {
            roofPoint = QgsPoint( bounds.xMinimum() + relX * bounds.width(), bounds.yMinimum() + relY * bounds.height(), roofPoint.z() );
            updateSavedGeometry = true;
            updateSavedShape = true;
            updateSavedCenter = true;
            updateSavedRelative = true;
          }
        }
      }

      bool ok = false;
      const double z = feature.attribute( QStringLiteral( "z" ) ).toDouble( &ok );
      if ( ok )
        roofPoint.setZ( z );

      if ( useRelativePosition && ( updateSavedGeometry || updateSavedShape || updateSavedCenter || updateSavedRelative ) )
      {
        SavedRoofPointUpdate update;
        update.fid = feature.id();
        update.point = roofPoint;
        update.updateGeometry = updateSavedGeometry;
        update.updateShape = updateSavedShape;
        update.updateCenter = updateSavedCenter;
        update.updateRelative = updateSavedRelative;
        update.shape = currentShape;
        update.centerX = bounds.center().x();
        update.centerY = bounds.center().y();
        if ( bounds.width() > 1e-12 && bounds.height() > 1e-12 )
        {
          update.relX = ( roofPoint.x() - bounds.xMinimum() ) / bounds.width();
          update.relY = ( roofPoint.y() - bounds.yMinimum() ) / bounds.height();
        }
        else
        {
          update.relX = 0.5;
          update.relY = 0.5;
        }
        savedPointUpdates.append( update );
      }

      points.append( BuildingRoof::RoofPoint{ roofPoint, feature.attribute( QStringLiteral( "type" ) ).toString() } );
    }

    if ( useRelativePosition && !savedPointUpdates.isEmpty() )
    {
      const bool wasEditable = pointLayer->isEditable();
      if ( !wasEditable )
        pointLayer->startEditing();

      for ( const SavedRoofPointUpdate &update : std::as_const( savedPointUpdates ) )
      {
        if ( update.updateGeometry )
        {
          QgsGeometry updatedGeometry( new QgsPoint( update.point ) );
          pointLayer->changeGeometry( update.fid, updatedGeometry );
        }
        if ( update.updateShape && buildingShapeIndex >= 0 )
          pointLayer->changeAttributeValue( update.fid, buildingShapeIndex, update.shape );
        if ( update.updateCenter && buildingCenterXIndex >= 0 )
          pointLayer->changeAttributeValue( update.fid, buildingCenterXIndex, update.centerX );
        if ( update.updateCenter && buildingCenterYIndex >= 0 )
          pointLayer->changeAttributeValue( update.fid, buildingCenterYIndex, update.centerY );
        if ( update.updateRelative && relXIndex >= 0 )
          pointLayer->changeAttributeValue( update.fid, relXIndex, update.relX );
        if ( update.updateRelative && relYIndex >= 0 )
          pointLayer->changeAttributeValue( update.fid, relYIndex, update.relY );
      }

      if ( !wasEditable )
        pointLayer->commitChanges();
      pointLayer->triggerRepaint();
    }

    return points;
  };

  if ( fromSavedLayer )
    *fromSavedLayer = false;

  QList<BuildingRoof::RoofPoint> points = collectPoints( QStringLiteral( "Roof_Edit_Points" ), false );
  if ( points.isEmpty() )
  {
    points = collectPoints( QStringLiteral( "Roof_Saved_Points" ), true );
    if ( fromSavedLayer && !points.isEmpty() )
      *fromSavedLayer = true;
  }
  return points;
}

double ThreeDViewTool::savedRoofBaseHeight( QgsFeatureId fid, double currentHeight )
{
  const QList<QgsMapLayer *> layers = QgsProject::instance()->mapLayersByName( QStringLiteral( "Roof_Saved_Points" ) );
  if ( layers.isEmpty() )
    return currentHeight;

  QgsVectorLayer *savedLayer = qobject_cast<QgsVectorLayer *>( layers.first() );
  if ( !savedLayer )
    return currentHeight;

  int baseIndex = savedLayer->fields().indexOf( QStringLiteral( "base_height" ) );
  if ( baseIndex < 0 )
  {
    savedLayer->dataProvider()->addAttributes( QList<QgsField>() << QgsField( QStringLiteral( "base_height" ), QVariant::Double ) );
    savedLayer->updateFields();
    baseIndex = savedLayer->fields().indexOf( QStringLiteral( "base_height" ) );
  }

  QgsFeatureRequest request;
  request.setFilterExpression( QStringLiteral( "building_fid = %1" ).arg( fid ) );
  QgsFeatureIterator it = savedLayer->getFeatures( request );
  QgsFeature feature;
  QgsFeatureIds ids;
  double baseHeight = std::numeric_limits<double>::quiet_NaN();
  while ( it.nextFeature( feature ) )
  {
    ids.insert( feature.id() );
    bool ok = false;
    const double value = feature.attribute( QStringLiteral( "base_height" ) ).toDouble( &ok );
    if ( ok )
      baseHeight = value;
  }

  if ( std::isfinite( baseHeight ) )
    return baseHeight;

  if ( baseIndex >= 0 && !ids.isEmpty() )
  {
    if ( !savedLayer->isEditable() )
      savedLayer->startEditing();
    for ( QgsFeatureId savedFid : ids )
      savedLayer->changeAttributeValue( savedFid, baseIndex, currentHeight );
    savedLayer->commitChanges();
  }

  return currentHeight;
}

MeshData ThreeDViewTool::buildMeshForFeature( const QgsFeature &feature, double height )
{
  bool fromSavedLayer = false;
  QList<BuildingRoof::RoofPoint> points = roofPointsForFeature( feature, &fromSavedLayer );
  const QVector<QgsPoint> footprintRing = wireframeFootprintRing( feature.geometry() );
  auto isGroundPoint = []( const QString &type ) {
    return type.contains( QStringLiteral( "??" ) )
           || type.contains( QStringLiteral( "ground" ), Qt::CaseInsensitive );
  };

  if ( fromSavedLayer && !points.isEmpty() )
  {
    const double delta = height - savedRoofBaseHeight( feature.id(), height );
    if ( std::fabs( delta ) > 1e-9 )
    {
      for ( BuildingRoof::RoofPoint &point : points )
      {
        if ( !isGroundPoint( point.type ) )
          point.point.setZ( point.point.z() + delta );
      }
    }
  }

  if ( !points.isEmpty() )
  {
    const QVector<BuildingRoof::RoofSample> pointCloudSamples = collectRoofSamplesFromFirstPointCloud( feature.geometry() );
    const BuildingRoof::MeshResult flatReliefRoof = BuildingRoof::buildFlatReliefPrismMesh( feature.geometry(), height, points, pointCloudSamples );
    if ( flatReliefRoof.success )
    {
      MeshData mesh{ flatReliefRoof.mesh.vertices, flatReliefRoof.mesh.indices };
      mesh.footprintRing = footprintRing;
      return mesh;
    }

    const BuildingRoof::MeshResult clusteredFlatTopHippedRoof = BuildingRoof::buildClusteredFlatTopHippedRoofPrismMesh( feature.geometry(), height, points, pointCloudSamples );
    if ( clusteredFlatTopHippedRoof.success )
    {
      MeshData mesh{ clusteredFlatTopHippedRoof.mesh.vertices, clusteredFlatTopHippedRoof.mesh.indices, true };
      mesh.footprintRing = footprintRing;
      return mesh;
    }

    const BuildingRoof::MeshResult curvedRoof = BuildingRoof::buildCurvedRoofPrismMesh( feature.geometry(), height, points );
    if ( curvedRoof.success )
    {
      MeshData mesh{ curvedRoof.mesh.vertices, curvedRoof.mesh.indices, false, true, true };
      mesh.footprintRing = footprintRing;
      return mesh;
    }

    const BuildingRoof::MeshResult apexRoof = BuildingRoof::buildApexRoofPrismMesh( feature.geometry(), height, points );
    if ( apexRoof.success )
    {
      MeshData mesh{ apexRoof.mesh.vertices, apexRoof.mesh.indices, false, false, true };
      mesh.footprintRing = footprintRing;
      return mesh;
    }

    const BuildingRoof::MeshResult gabledRoof = BuildingRoof::buildGabledRoofPrismMesh( feature.geometry(), height, points, pointCloudSamples );
    if ( gabledRoof.success )
    {
      MeshData mesh{ gabledRoof.mesh.vertices, gabledRoof.mesh.indices };
      mesh.footprintRing = footprintRing;
      return mesh;
    }

    const BuildingRoof::MeshResult hippedRoof = BuildingRoof::buildHippedRoofPrismMesh( feature.geometry(), height, points );
    if ( hippedRoof.success )
    {
      MeshData mesh{ hippedRoof.mesh.vertices, hippedRoof.mesh.indices };
      mesh.footprintRing = footprintRing;
      return mesh;
    }

    const BuildingRoof::MeshResult multiRidgeRoof = BuildingRoof::buildMultiRidgePrismMesh( feature.geometry(), height, points );
    if ( multiRidgeRoof.success )
    {
      MeshData mesh{ multiRidgeRoof.mesh.vertices, multiRidgeRoof.mesh.indices };
      mesh.footprintRing = footprintRing;
      return mesh;
    }
  }

  MeshData mesh = BuildMesh::build( feature.geometry(), height );
  mesh.eaveWireframe = true;
  mesh.footprintRing = footprintRing;
  return mesh;
}

void ThreeDViewTool::ensurePreviewEntity()
{
  if ( mPreviewEntity )
    return;
  if ( !mIface )
    return;

  Qgs3DMapCanvas *activeCanvas3D = mIface->mapCanvases3D().isEmpty()
                                     ? mIface->createNewMapCanvas3D( tr( "3D Preview" ) )
                                     : mIface->mapCanvases3D().first();
  if ( !activeCanvas3D || !activeCanvas3D->scene() )
    return;

  Qgs3DMapScene *scene = activeCanvas3D->scene();
  connect( scene, &QObject::destroyed, this, [this]() {
    mPreviewEntity = nullptr;
    mPreviewRenderer = nullptr;
    mPreviewGeometry = nullptr;
    mPreviewVertexBuffer = nullptr;
    mPreviewPositionAttribute = nullptr;
    mPreviewNormalAttribute = nullptr;
    mPreviewMaterial = nullptr;
    mPreviewFids.clear();
  }, Qt::UniqueConnection );

  mPreviewEntity = new Qt3DCore::QEntity( scene );
  mPreviewRenderer = new Qt3DRender::QGeometryRenderer( mPreviewEntity );
  mPreviewGeometry = new Qt3DRender::QGeometry( mPreviewRenderer );
  mPreviewVertexBuffer = new Qt3DRender::QBuffer( mPreviewGeometry );
  mPreviewPositionAttribute = new Qt3DRender::QAttribute( mPreviewGeometry );
  mPreviewNormalAttribute = new Qt3DRender::QAttribute( mPreviewGeometry );

  constexpr int stride = 6 * sizeof( float );

  mPreviewPositionAttribute->setName( Qt3DRender::QAttribute::defaultPositionAttributeName() );
  mPreviewPositionAttribute->setVertexBaseType( Qt3DRender::QAttribute::Float );
  mPreviewPositionAttribute->setVertexSize( 3 );
  mPreviewPositionAttribute->setAttributeType( Qt3DRender::QAttribute::VertexAttribute );
  mPreviewPositionAttribute->setBuffer( mPreviewVertexBuffer );
  mPreviewPositionAttribute->setByteStride( stride );
  mPreviewPositionAttribute->setByteOffset( 0 );

  mPreviewNormalAttribute->setName( Qt3DRender::QAttribute::defaultNormalAttributeName() );
  mPreviewNormalAttribute->setVertexBaseType( Qt3DRender::QAttribute::Float );
  mPreviewNormalAttribute->setVertexSize( 3 );
  mPreviewNormalAttribute->setAttributeType( Qt3DRender::QAttribute::VertexAttribute );
  mPreviewNormalAttribute->setBuffer( mPreviewVertexBuffer );
  mPreviewNormalAttribute->setByteStride( stride );
  mPreviewNormalAttribute->setByteOffset( 3 * sizeof( float ) );

  mPreviewGeometry->addAttribute( mPreviewPositionAttribute );
  mPreviewGeometry->addAttribute( mPreviewNormalAttribute );

  mPreviewRenderer->setGeometry( mPreviewGeometry );
  mPreviewRenderer->setPrimitiveType( Qt3DRender::QGeometryRenderer::Triangles );

  mPreviewMaterial = new Qt3DExtras::QPhongMaterial( mPreviewEntity );
  mPreviewMaterial->setDiffuse( QColor( 255, 180, 60, 230 ) );
  mPreviewMaterial->setAmbient( QColor( 90, 70, 35 ) );
  mPreviewMaterial->setSpecular( QColor( 80, 80, 80 ) );
  mPreviewMaterial->setShininess( 18.0f );

  mPreviewEntity->addComponent( mPreviewRenderer );
  mPreviewEntity->addComponent( mPreviewMaterial );
}

void ThreeDViewTool::updatePreviewEntity( QgsVectorLayer *layer, const QgsFeatureIds &fids, const QString &heightFieldName, double height )
{
  if ( !layer || !mIface || fids.isEmpty() )
    return;

  Qgs3DMapCanvas *activeCanvas3D = mIface->mapCanvases3D().isEmpty()
                                     ? mIface->createNewMapCanvas3D( tr( "3D Preview" ) )
                                     : mIface->mapCanvases3D().first();
  if ( !activeCanvas3D || !activeCanvas3D->mapSettings() )
    return;

  ensurePreviewEntity();
  if ( !mPreviewEntity || !mPreviewRenderer || !mPreviewVertexBuffer || !mPreviewPositionAttribute || !mPreviewNormalAttribute )
    return;

  if ( mPreviewFids != fids )
  {
    if ( !mPreviewFids.isEmpty() )
      refreshMemoryData();
    mPreviewFids = fids;
    removeTempFeatures( fids );
  }

  Qgs3DMapSettings *settings = activeCanvas3D->mapSettings();
  QgsCoordinateTransform layerToSceneTransform;
  const bool transformLayerCoordinates = layer->crs().isValid() && settings->crs().isValid() && layer->crs() != settings->crs();
  if ( transformLayerCoordinates )
    layerToSceneTransform = QgsCoordinateTransform( layer->crs(), settings->crs(), settings->transformContext() );

  QByteArray vertexBufferData;
  QVector<float> raw;

  for ( QgsFeatureId fid : fids )
  {
    QgsFeature feat;
    if ( !layer->getFeatures( QgsFeatureRequest( fid ) ).nextFeature( feat ) )
      continue;

    const MeshData mesh = buildMeshForFeature( feat, height );

    auto appendMeshToPreview = [&]( const MeshData &mesh )
    {
      if ( mesh.isEmpty() )
        return;

      for ( int i = 0; i + 2 < mesh.indices.size(); i += 3 )
      {
        QVector3D tri[3];
        bool skipTriangle = false;
        for ( int j = 0; j < 3; ++j )
        {
          const QgsPoint &p = mesh.vertices[mesh.indices[i + j]];
          QgsPointXY mapPoint( p.x(), p.y() );
          if ( transformLayerCoordinates )
          {
            try
            {
              mapPoint = layerToSceneTransform.transform( mapPoint );
            }
            catch ( ... )
            {
              skipTriangle = true;
              break;
            }
          }
          QgsVector3D world = settings->mapToWorldCoordinates( QgsVector3D( mapPoint.x(), mapPoint.y(), p.z() ) );
          tri[j] = QVector3D( static_cast<float>( world.x() ), static_cast<float>( world.y() ), static_cast<float>( world.z() ) );
        }
        if ( skipTriangle )
          continue;

        QVector3D normal = QVector3D::crossProduct( tri[1] - tri[0], tri[2] - tri[0] ).normalized();
        if ( normal.isNull() )
          normal = QVector3D( 0.0f, 0.0f, 1.0f );

        for ( int j = 0; j < 3; ++j )
        {
          raw << tri[j].x() << tri[j].y() << tri[j].z()
              << normal.x() << normal.y() << normal.z();
        }
      }
    };

    appendMeshToPreview( mesh );
  }

  vertexBufferData.resize( raw.size() * static_cast<int>( sizeof( float ) ) );
  memcpy( vertexBufferData.data(), raw.constData(), vertexBufferData.size() );
  const int vertexCount = raw.size() / 6;

  if ( !mPreviewEntity || !mPreviewRenderer || !mPreviewVertexBuffer || !mPreviewPositionAttribute || !mPreviewNormalAttribute )
    return;

  mPreviewVertexBuffer->setData( vertexBufferData );
  mPreviewPositionAttribute->setCount( vertexCount );
  mPreviewNormalAttribute->setCount( vertexCount );
  mPreviewRenderer->setVertexCount( vertexCount );
  mPreviewEntity->setEnabled( vertexCount > 0 );
}

void ThreeDViewTool::clearPreviewEntity()
{
  QPointer<Qt3DCore::QEntity> entityToDelete = mPreviewEntity;

  mPreviewRenderer = nullptr;
  mPreviewGeometry = nullptr;
  mPreviewVertexBuffer = nullptr;
  mPreviewPositionAttribute = nullptr;
  mPreviewNormalAttribute = nullptr;
  mPreviewMaterial = nullptr;
  mPreviewEntity = nullptr;
  mPreviewFids.clear();

  if ( entityToDelete )
  {
    entityToDelete->setEnabled( false );
    entityToDelete->setParent( static_cast<Qt3DCore::QNode *>( nullptr ) );
    entityToDelete->deleteLater();
  }
}

// ==================== 激活与停用 ====================
void ThreeDViewTool::activate()
{
  QgsMapTool::activate();
  setupUI();
}

void ThreeDViewTool::deactivate()
{
  if ( mWidget )
    mWidget->hide();
  QgsMapTool::deactivate();
}

// ==================== UI 交互 ====================
// 显示字段选择弹窗（旧版）
void ThreeDViewTool::showFieldSelectUI()
{
  if ( !mWidget )
  {
    mWidget = new QWidget();
    mUI.setupUi( mWidget );
    mWidget->setWindowTitle( tr( "设置选项" ) );
    mWidget->installEventFilter( this );
  }

  mUI.comboBox->clear();
  const QgsFields &fields = mActiveLayer->fields();
  for ( const QgsField &f : fields )
  {
    if ( f.isNumeric() )
      mUI.comboBox->addItem( f.name() );
  }

  mWidget->show();
}

// 确认选择：创建内存图层、配置 3D 渲染、绑定信号
void ThreeDViewTool::confirmSelection()
{
  if ( !mActiveLayer || !mWidget || !mIface )
    return;

  mSelectedHeightField = mUI.comboBox->currentText();
  mWidget->hide();

  if ( !mActiveLayer->isEditable() )
    mActiveLayer->startEditing();

  if ( !mTempLayer )
  {
    QString uri = QString( "PolygonZ?crs=%1&field=original_fid:long&field=part:string" )
                    .arg( mActiveLayer->crs().authid() );
    QString tempLayerName = QString( "%1_3Dbuilding" ).arg( mActiveLayer->name() );
    mTempLayer = new QgsVectorLayer( uri, tempLayerName, "memory" );

    QgsProject::instance()->addMapLayer( mTempLayer, false );

    QgsLayerTree *root = QgsProject::instance()->layerTreeRoot();
    QgsLayerTreeLayer *activeNode = root->findLayer( mActiveLayer->id() );

    if ( activeNode )
    {
      QgsLayerTreeGroup *parentGroup = qobject_cast<QgsLayerTreeGroup *>( activeNode->parent() );
      if ( parentGroup )
      {
        int index = parentGroup->children().indexOf( activeNode );
        parentGroup->insertLayer( index + 1, mTempLayer );
      }
      else
      {
        root->addLayer( mTempLayer );
      }
    }
    else
    {
      root->addLayer( mTempLayer );
    }

    QgsLayerTreeLayer *treeLayer = root->findLayer( mTempLayer->id() );
    if ( treeLayer )
      treeLayer->setCustomProperty( "nodeHidden", true );

    // 禁止 2D 交互与渲染
    mTempLayer->setFlags( mTempLayer->flags() & ~QgsMapLayer::Identifiable );
    mTempLayer->setFlags( mTempLayer->flags() & ~QgsMapLayer::Searchable );
    mTempLayer->setRenderer( new QgsNullSymbolRenderer() );
    mTempLayer->setOpacity( 0.0 );

    // 配置 3D 渲染器
    applyBuildingTriangleMeshMode();

    // 绑定同步信号
    connect( mActiveLayer, &QgsVectorLayer::geometryChanged, this, &ThreeDViewTool::onFeatureUpdated, Qt::UniqueConnection );
    connect( mActiveLayer, &QgsVectorLayer::attributeValueChanged, this, [this]( QgsFeatureId fid, int idx, const QVariant &value ) {
                    Q_UNUSED(idx)
                    Q_UNUSED(value)
                    onFeatureUpdated(fid); }, Qt::UniqueConnection );
    connect( mActiveLayer, &QgsVectorLayer::featuresDeleted, this, &ThreeDViewTool::onFeaturesDeleted, Qt::UniqueConnection );
    connect( mActiveLayer, &QgsVectorLayer::featureAdded, this, &ThreeDViewTool::onFeatureAdded, Qt::UniqueConnection );
  }

  // 获取或创建 3D 视图
  Qgs3DMapCanvas *activeCanvas3D = mIface->mapCanvases3D().isEmpty()
                                     ? mIface->createNewMapCanvas3D( tr( "3D Preview" ) )
                                     : mIface->mapCanvases3D().first();

  if ( activeCanvas3D )
  {
    if ( QWidget *dock = qobject_cast<QWidget *>( activeCanvas3D->parent() ) )
    {
      dock->show();
      dock->raise();
    }

    Qgs3DMapSettings *settings = activeCanvas3D->mapSettings();
    QList<QgsMapLayer *> current3DLayers = settings->layers();
    if ( !current3DLayers.contains( mTempLayer ) )
    {
      current3DLayers.append( mTempLayer );
      settings->setLayers( current3DLayers );
    }

    if ( QWidget *cv = qobject_cast<QWidget *>( activeCanvas3D ) )
      cv->update();
  }

  addLoadedPointCloudLayersTo3DView();
  refreshMemoryData();
  mIface->setActiveLayer( mActiveLayer );
}

// 取消选择并提示
void ThreeDViewTool::cancelSelection()
{
  if ( mWidget )
  {
    QMessageBox::warning( mWidget, tr( "提示" ), tr( "已取消设置并退出。" ) );
    mWidget->hide();
  }
}

// 事件过滤器：回车确认，ESC取消
bool ThreeDViewTool::eventFilter( QObject *obj, QEvent *event )
{
  if ( obj == mWidget && event->type() == QEvent::KeyPress )
  {
    QKeyEvent *ke = static_cast<QKeyEvent *>( event );
    if ( ke->key() == Qt::Key_Return || ke->key() == Qt::Key_Enter )
    {
      confirmSelection();
      return true;
    }
    if ( ke->key() == Qt::Key_Escape )
    {
      cancelSelection();
      return true;
    }
  }
  return QgsMapTool::eventFilter( obj, event );
}

// ==================== 新版 UI（图层 + 字段选择） ====================
// 初始化设置界面
void ThreeDViewTool::setupUI()
{
  if ( !mWidget )
  {
    mWidget = new QWidget();
    mUI.setupUi( mWidget );
    connect( mUI.layercomboBox, QOverload<int>::of( &QComboBox::currentIndexChanged ), this, &ThreeDViewTool::onLayerChanged );
    connect( mUI.addvector, &QPushButton::clicked, this, &ThreeDViewTool::addVectorData );
    connect( mUI.addpointwould, &QPushButton::clicked, this, &ThreeDViewTool::addPointCloudData );
    connect( mUI.addraster, &QPushButton::clicked, this, &ThreeDViewTool::addRasterData );
    mWidget->installEventFilter( this );
  }

  refreshLayerList();
  mWidget->show();
}

// 刷新图层列表（仅显示面图层）
void ThreeDViewTool::refreshLayerList()
{
  mUI.layercomboBox->blockSignals( true );
  mUI.layercomboBox->clear();
  mUI.layercomboBox->addItem( "请选择矢量图层...", QVariant() );

  auto layers = QgsProject::instance()->mapLayers().values();
  for ( QgsMapLayer *layer : layers )
  {
    if ( layer->type() == Qgis::LayerType::Vector )
    {
      QgsVectorLayer *vlyr = qobject_cast<QgsVectorLayer *>( layer );
      if ( vlyr && vlyr->geometryType() == Qgis::GeometryType::Polygon )
        mUI.layercomboBox->addItem( vlyr->name(), vlyr->id() );
    }
  }
  mUI.layercomboBox->blockSignals( false );
}

// 图层切换时刷新字段列表
void ThreeDViewTool::onLayerChanged( int index )
{
  if ( index <= 0 )
  {
    mActiveLayer = nullptr;
    mUI.comboBox->clear();
    return;
  }

  QString layerId = mUI.layercomboBox->currentData().toString();
  mActiveLayer = qobject_cast<QgsVectorLayer *>( QgsProject::instance()->mapLayer( layerId ) );
  updateFieldsCombo();
}

// 更新高度字段下拉（仅数值字段）
void ThreeDViewTool::updateFieldsCombo()
{
  mUI.comboBox->clear();
  if ( !mActiveLayer )
    return;

  mUI.comboBox->addItem( "请选择高度字段..." );
  const QgsFields &fields = mActiveLayer->fields();
  for ( const QgsField &field : fields )
  {
    if ( field.isNumeric() )
      mUI.comboBox->addItem( field.name() );
  }
}

// ==================== 几何辅助函数 ====================
void ThreeDViewTool::addVectorData()
{
  if ( !mIface )
    return;

  const QStringList paths = QFileDialog::getOpenFileNames(
    mWidget,
    tr( "Add Vector Data" ),
    QString(),
    tr( "Vector data (*.shp *.gpkg *.geojson *.json *.kml *.dxf);;All files (*.*)" )
  );

  if ( paths.isEmpty() )
    return;

  QgsVectorLayer *lastPolygonLayer = nullptr;
  for ( const QString &path : paths )
  {
    QgsVectorLayer *layer = mIface->addVectorLayer( path, QFileInfo( path ).baseName(), QStringLiteral( "ogr" ) );
    if ( !layer || !layer->isValid() )
    {
      QMessageBox::warning( mWidget, tr( "Add Vector Data" ), tr( "Failed to load vector layer:\n%1" ).arg( path ) );
      continue;
    }

    if ( layer->geometryType() == Qgis::GeometryType::Polygon )
      lastPolygonLayer = layer;
  }

  refreshLayerList();
  if ( lastPolygonLayer )
  {
    const int index = mUI.layercomboBox->findData( lastPolygonLayer->id() );
    if ( index >= 0 )
      mUI.layercomboBox->setCurrentIndex( index );
  }
}

void ThreeDViewTool::addPointCloudData()
{
  if ( !mIface )
    return;

  const QStringList paths = QFileDialog::getOpenFileNames(
    mWidget,
    tr( "Add Point Cloud Data" ),
    QString(),
    tr( "Point cloud (*.las *.laz *.copc.laz *.ept.json *.vpc);;All files (*.*)" )
  );

  if ( paths.isEmpty() )
    return;

  for ( const QString &path : paths )
  {
    const QList<QgsProviderRegistry::ProviderCandidateDetails> preferredProviders =
      QgsProviderRegistry::instance()->preferredProvidersForUri( path );
    const QString providerKey = preferredProviders.empty()
                                  ? QStringLiteral( "pdal" )
                                  : preferredProviders.first().metadata()->key();

    QgsPointCloudLayer *layer = mIface->addPointCloudLayer( path, QFileInfo( path ).baseName(), providerKey );
    if ( !layer || !layer->isValid() )
    {
      QMessageBox::warning( mWidget, tr( "Add Point Cloud Data" ), tr( "Failed to load point cloud layer:\n%1" ).arg( path ) );
      continue;
    }

    configurePointCloud3DRenderer( layer );
    QPointer<QgsPointCloudLayer> safeLayer = layer;
    connect( layer, &QgsPointCloudLayer::statisticsCalculationStateChanged, this, [this, safeLayer]( QgsPointCloudLayer::PointCloudStatisticsCalculationState state ) {
      if ( state != QgsPointCloudLayer::PointCloudStatisticsCalculationState::Calculated )
        return;
      if ( !safeLayer || !mIface )
        return;

      configurePointCloud3DRenderer( safeLayer );
      refresh3DCanvases();
    } );
  }
}

void ThreeDViewTool::addRasterData()
{
  if ( !mIface )
    return;

  const QStringList paths = QFileDialog::getOpenFileNames(
    mWidget,
    tr( "Add Raster Data" ),
    QString(),
    tr( "Raster data (*.tif *.tiff *.img *.vrt *.jp2 *.png *.jpg *.jpeg);;GeoTIFF (*.tif *.tiff);;All files (*.*)" )
  );

  if ( paths.isEmpty() )
    return;

  for ( const QString &path : paths )
  {
    QgsRasterLayer *layer = mIface->addRasterLayer( path, QFileInfo( path ).baseName(), QStringLiteral( "gdal" ) );
    if ( !layer || !layer->isValid() )
      QMessageBox::warning( mWidget, tr( "Add Raster Data" ), tr( "Failed to load raster layer:\n%1" ).arg( path ) );
  }
}

void ThreeDViewTool::configurePointCloud3DRenderer( QgsPointCloudLayer *layer )
{
  if ( !layer )
    return;

  if ( QgsPointCloudLayerElevationProperties *elevation =
         qobject_cast<QgsPointCloudLayerElevationProperties *>( layer->elevationProperties() ) )
  {
    elevation->setZScale( 1.0 );
    elevation->setZOffset( 0.0 );
  }

  const QString zAttribute = QStringLiteral( "Z" );
  double zMin = layer->statistics().minimum( zAttribute );
  double zMax = layer->statistics().maximum( zAttribute );
  if ( !std::isfinite( zMin ) )
    zMin = -50.0;
  if ( !std::isfinite( zMax ) )
    zMax = 50.0;
  if ( zMax <= zMin + 1e-9 )
  {
    zMin -= 1.0;
    zMax += 1.0;
  }

  QgsColorRampShader shader( zMin, zMax, nullptr, Qgis::ShaderInterpolationMethod::Discrete, Qgis::ShaderClassificationMethod::Continuous );
  const double range = zMax - zMin;
  QList<QgsColorRampShader::ColorRampItem> items;
  items << QgsColorRampShader::ColorRampItem( zMin, QColor( 49, 54, 149 ), QString::number( zMin, 'f', 2 ) )
        << QgsColorRampShader::ColorRampItem( zMin + range * 0.14, QColor( 69, 117, 180 ) )
        << QgsColorRampShader::ColorRampItem( zMin + range * 0.28, QColor( 116, 173, 209 ) )
        << QgsColorRampShader::ColorRampItem( zMin + range * 0.42, QColor( 171, 217, 233 ) )
        << QgsColorRampShader::ColorRampItem( zMin + range * 0.56, QColor( 253, 174, 97 ) )
        << QgsColorRampShader::ColorRampItem( zMin + range * 0.70, QColor( 244, 109, 67 ) )
        << QgsColorRampShader::ColorRampItem( zMin + range * 0.84, QColor( 215, 48, 39 ) )
        << QgsColorRampShader::ColorRampItem( zMax, QColor( 211, 47, 47 ), QString::number( zMax, 'f', 2 ) );
  shader.setColorRampType( Qgis::ShaderInterpolationMethod::Discrete );
  shader.setClassificationMode( Qgis::ShaderClassificationMethod::Continuous );
  shader.setColorRampItemList( items );

  QgsColorRampPointCloud3DSymbol *symbol = new QgsColorRampPointCloud3DSymbol();
  symbol->setAttribute( zAttribute );
  symbol->setColorRampShader( shader );
  symbol->setColorRampShaderMinMax( zMin, zMax );
  symbol->setPointSize( 2.0f );

  QgsPointCloudLayer3DRenderer *renderer = new QgsPointCloudLayer3DRenderer();
  renderer->setLayer( layer );
  renderer->setSymbol( symbol );
  renderer->setMaximumScreenError( 1.0 );
  renderer->setPointRenderingBudget( 5000000 );
  layer->setRenderer3D( renderer );
}

void ThreeDViewTool::ensureLayerIn3DView( QgsMapLayer *layer )
{
  if ( !mIface || !layer )
    return;

  Qgs3DMapCanvas *activeCanvas3D = mIface->mapCanvases3D().isEmpty()
                                     ? mIface->createNewMapCanvas3D( tr( "3D Preview" ) )
                                     : mIface->mapCanvases3D().first();

  if ( !activeCanvas3D || !activeCanvas3D->mapSettings() )
    return;

  if ( QWidget *dock = qobject_cast<QWidget *>( activeCanvas3D->parent() ) )
  {
    dock->show();
    dock->raise();
  }

  Qgs3DMapSettings *settings = activeCanvas3D->mapSettings();
  QList<QgsMapLayer *> current3DLayers = settings->layers();
  if ( !current3DLayers.contains( layer ) )
    current3DLayers.append( layer );

  settings->setLayers( current3DLayers );

  if ( QWidget *cv = qobject_cast<QWidget *>( activeCanvas3D ) )
    cv->update();
}

void ThreeDViewTool::addLoadedPointCloudLayersTo3DView()
{
  for ( QgsMapLayer *projectLayer : QgsProject::instance()->mapLayers().values() )
  {
    QgsPointCloudLayer *pointCloudLayer = qobject_cast<QgsPointCloudLayer *>( projectLayer );
    if ( !pointCloudLayer || !pointCloudLayer->isValid() )
      continue;

    configurePointCloud3DRenderer( pointCloudLayer );
    ensureLayerIn3DView( pointCloudLayer );
  }
}

static double cross2D( const QgsPointXY &a, const QgsPointXY &b, const QgsPointXY &c )
{
  return ( b.x() - a.x() ) * ( c.y() - a.y() ) - ( b.y() - a.y() ) * ( c.x() - a.x() );
}

static bool pointInTriangle( const QgsPointXY &p, const QgsPointXY &a, const QgsPointXY &b, const QgsPointXY &c )
{
  double c1 = cross2D( a, b, p );
  double c2 = cross2D( b, c, p );
  double c3 = cross2D( c, a, p );
  return ( c1 > 0 && c2 > 0 && c3 > 0 );
}

static bool isConvex( const QgsPointXY &prev, const QgsPointXY &curr, const QgsPointXY &next )
{
  return cross2D( prev, curr, next ) > 0;
}

// 耳切法三角剖分
static QVector<int> earClipping( const QVector<QgsPointXY> &pts )
{
  QVector<int> result;
  int n = pts.size();
  if ( n < 3 )
    return result;

  QVector<int> V;
  V.reserve( n );
  for ( int i = 0; i < n; ++i )
    V.push_back( i );

  while ( V.size() > 3 )
  {
    bool earFound = false;
    for ( int i = 0; i < V.size(); ++i )
    {
      int prev = V[( i - 1 + V.size() ) % V.size()];
      int curr = V[i];
      int next = V[( i + 1 ) % V.size()];

      if ( !isConvex( pts[prev], pts[curr], pts[next] ) )
        continue;

      bool hasInside = false;
      for ( int j = 0; j < V.size(); ++j )
      {
        int vi = V[j];
        if ( vi == prev || vi == curr || vi == next )
          continue;
        if ( pointInTriangle( pts[vi], pts[prev], pts[curr], pts[next] ) )
        {
          hasInside = true;
          break;
        }
      }
      if ( hasInside )
        continue;

      result << prev << curr << next;
      V.removeAt( i );
      earFound = true;
      break;
    }
    if ( !earFound )
      break;
  }

  if ( V.size() == 3 )
    result << V[0] << V[1] << V[2];

  return result;
}

// ==================== 3D 模型构建 ====================
// 将网格数据转换为 QgsFeature 列表
QgsFeatureList ThreeDViewTool::buildBuildingFromMesh( const MeshData &mesh, const QMatrix4x4 &mat )
{
  QgsFeatureList features;
  int triCount = mesh.indices.size() / 3;

  for ( int i = 0; i < triCount; i++ )
  {
    QgsPoint p0 = mesh.vertices[mesh.indices[i * 3]];
    QgsPoint p1 = mesh.vertices[mesh.indices[i * 3 + 1]];
    QgsPoint p2 = mesh.vertices[mesh.indices[i * 3 + 2]];

    std::unique_ptr<QgsPolygon> poly( new QgsPolygon() );
    std::unique_ptr<QgsLineString> ring( new QgsLineString() );

    ring->addVertex( p0 );
    ring->addVertex( p1 );
    ring->addVertex( p2 );
    ring->addVertex( p0 ); // 闭合

    poly->setExteriorRing( ring.release() );
    QgsFeature feat;
    feat.setGeometry( QgsGeometry( std::move( poly ) ) );
    features.append( feat );
  }
  return features;
}

// 备用三角测试函数（使用 QVector3D）
static bool isPointInTriangle( const QVector3D &a, const QVector3D &b, const QVector3D &c, const QVector3D &p )
{
  float v0x = c.x() - a.x(), v0y = c.y() - a.y();
  float v1x = b.x() - a.x(), v1y = b.y() - a.y();
  float v2x = p.x() - a.x(), v2y = p.y() - a.y();
  float dot00 = v0x * v0x + v0y * v0y;
  float dot01 = v0x * v1x + v0y * v1y;
  float dot02 = v0x * v2x + v0y * v2y;
  float dot11 = v1x * v1x + v1y * v1y;
  float dot12 = v1x * v2x + v1y * v2y;
  float invDenom = 1.0 / ( dot00 * dot11 - dot01 * dot01 );
  float u = ( dot11 * dot02 - dot01 * dot12 ) * invDenom;
  float v = ( dot00 * dot12 - dot01 * dot02 ) * invDenom;
  return ( u >= 0 ) && ( v >= 0 ) && ( u + v < 1 );
}

// ==================== BuildMesh::build ====================
// 根据多边形构建带高度的立体网格
MeshData BuildMesh::build( const QgsGeometry &geom, double height )
{
  MeshData mesh;
  if ( geom.isEmpty() || !geom.isGeosValid() )
    return mesh;

  double h = ( height <= 0 ? 3.0 : height );

  QgsGeometry inputGeom = geom.convertToType( Qgis::GeometryType::Polygon );
  const QgsPolygon *poly = qgsgeometry_cast<const QgsPolygon *>( inputGeom.constGet() );
  if ( !poly || !poly->exteriorRing() )
    return mesh;

  const QgsCurve *ring = poly->exteriorRing();
  QVector<QgsPointXY> polyPts;

  for ( int i = 0; i < ring->numPoints() - 1; ++i )
  {
    QgsPoint p;
    Qgis::VertexType vt;
    ring->pointAt( i, p, vt );
    polyPts.push_back( QgsPointXY( p.x(), p.y() ) );
  }

  // 保证逆时针顺序
  double area = 0;
  for ( int i = 0; i < polyPts.size(); ++i )
  {
    const QgsPointXY &a = polyPts[i];
    const QgsPointXY &b = polyPts[( i + 1 ) % polyPts.size()];
    area += ( a.x() * b.y() - b.x() * a.y() );
  }
  if ( area < 0 )
    std::reverse( polyPts.begin(), polyPts.end() );

  int base = polyPts.size();

  // 顶点生成（底面 0 + 顶面 h）
  for ( const QgsPointXY &p : polyPts )
  {
    mesh.vertices.append( QgsPoint( p.x(), p.y(), 0.0 ) );
    mesh.vertices.append( QgsPoint( p.x(), p.y(), h ) );
  }

  // 侧面索引
  for ( int i = 0; i < base; ++i )
  {
    int next = ( i + 1 ) % base;
    mesh.indices << 2 * i << 2 * next << 2 * i + 1;
    mesh.indices << 2 * i + 1 << 2 * next << 2 * next + 1;
  }

  // 耳切法剖分顶面和底面
  auto pointInTri = [&]( const QgsPointXY &p, const QgsPointXY &a, const QgsPointXY &b, const QgsPointXY &c ) {
    double c1 = ( b.x() - a.x() ) * ( p.y() - a.y() ) - ( b.y() - a.y() ) * ( p.x() - a.x() );
    double c2 = ( c.x() - b.x() ) * ( p.y() - b.y() ) - ( c.y() - b.y() ) * ( p.x() - b.x() );
    double c3 = ( a.x() - c.x() ) * ( p.y() - c.y() ) - ( a.y() - c.y() ) * ( p.x() - c.x() );
    return ( c1 >= -1e-10 && c2 >= -1e-10 && c3 >= -1e-10 );
  };

  QVector<int> V;
  for ( int i = 0; i < base; ++i )
    V.push_back( i );

  while ( V.size() > 3 )
  {
    bool earFound = false;
    for ( int i = 0; i < V.size(); ++i )
    {
      int p_idx = V[( i - 1 + V.size() ) % V.size()];
      int c_idx = V[i];
      int n_idx = V[( i + 1 ) % V.size()];
      const QgsPointXY &p1 = polyPts[p_idx], &p2 = polyPts[c_idx], &p3 = polyPts[n_idx];

      double cp = ( p2.x() - p1.x() ) * ( p3.y() - p1.y() ) - ( p2.y() - p1.y() ) * ( p3.x() - p1.x() );
      if ( cp <= 0 )
        continue;

      bool hasInside = false;
      for ( int j = 0; j < V.size(); ++j )
      {
        if ( V[j] == p_idx || V[j] == c_idx || V[j] == n_idx )
          continue;
        if ( pointInTri( polyPts[V[j]], p1, p2, p3 ) )
        {
          hasInside = true;
          break;
        }
      }

      if ( !hasInside )
      {
        mesh.indices << ( 2 * p_idx + 1 ) << ( 2 * c_idx + 1 ) << ( 2 * n_idx + 1 ); // 顶
        mesh.indices << ( 2 * p_idx ) << ( 2 * n_idx ) << ( 2 * c_idx );             // 底
        V.removeAt( i );
        earFound = true;
        break;
      }
    }
    if ( !earFound )
      break;
  }

  if ( V.size() == 3 )
  {
    mesh.indices << ( 2 * V[0] + 1 ) << ( 2 * V[1] + 1 ) << ( 2 * V[2] + 1 );
    mesh.indices << ( 2 * V[0] ) << ( 2 * V[2] ) << ( 2 * V[1] );
  }

  return mesh;
}

// ==================== 数据同步 ====================
// 根据原始要素更新其对应的 3D 模型
void ThreeDViewTool::updateFeature3D( const QgsFeature &originFeat )
{
  if ( !mTempLayer )
    return;

  double h = originFeat.attribute( mSelectedHeightField ).toDouble();
  if ( h <= 0 )
    h = 5.0;

  const MeshData mesh = buildMeshForFeature( originFeat, h );
  if ( mesh.isEmpty() )
    return;

  QgsFeatureList newTriangles = buildBuildingFromMesh( mesh, QMatrix4x4() );
  for ( QgsFeature &tri : newTriangles )
    tri.setAttributes( QgsAttributes() << originFeat.id() << QStringLiteral( "full" ) );

  mTempLayer->startEditing();

  // 删除该要素旧三角形
  QgsFeatureRequest request;
  request.setFilterExpression( QString( "original_fid = %1" ).arg( originFeat.id() ) );
  QgsFeatureIterator it = mTempLayer->getFeatures( request );
  QgsFeature f;
  QgsFeatureIds toDelete;
  while ( it.nextFeature( f ) )
    toDelete << f.id();
  mTempLayer->deleteFeatures( toDelete );

  mTempLayer->addFeatures( newTriangles );
  mTempLayer->commitChanges();
  mTempLayer->triggerRepaint();

  if ( mBuildingTriangleMeshMode && originFeat.id() == mWireframeFid )
    updateWireframeLayerFromMesh( mesh, originFeat.id() );
}

// 响应要素几何/属性变更
void ThreeDViewTool::onFeatureUpdated( QgsFeatureId fid )
{
  if ( !mFeatureUpdateTimer )
    return;

  mPendingFeatureUpdates.insert( fid );
  if ( !mFeatureUpdateTimer->isActive() )
    mFeatureUpdateTimer->start();
}

void ThreeDViewTool::applyFeature3DUpdate( QgsFeatureId fid )
{
  if ( !mActiveLayer )
    return;

  QgsFeature originFeat;
  if ( mActiveLayer->getFeatures( QgsFeatureRequest( fid ) ).nextFeature( originFeat ) )
    updateFeature3D( originFeat );
}

void ThreeDViewTool::flushPendingFeatureUpdates()
{
  const QgsFeatureIds pendingUpdates = mPendingFeatureUpdates;
  mPendingFeatureUpdates.clear();

  for ( QgsFeatureId fid : pendingUpdates )
    applyFeature3DUpdate( fid );

  refresh3DCanvases();
}

// 响应要素删除
void ThreeDViewTool::onFeaturesDeleted( const QgsFeatureIds &fids )
{
  if ( !mTempLayer )
    return;

  mTempLayer->startEditing();
  for ( QgsFeatureId fid : fids )
  {
    QgsFeatureRequest request;
    request.setFilterExpression( QString( "original_fid = %1" ).arg( fid ) );
    QgsFeatureIterator it = mTempLayer->getFeatures( request );
    QgsFeature f;
    QgsFeatureIds toDelete;
    while ( it.nextFeature( f ) )
      toDelete << f.id();
    mTempLayer->deleteFeatures( toDelete );
    if ( fid == mWireframeFid )
      clearWireframeLayer();
  }
  mTempLayer->commitChanges();
  mTempLayer->triggerRepaint();
}

// 响应新增要素
void ThreeDViewTool::onFeatureAdded( QgsFeatureId fid )
{
  if ( !mActiveLayer )
    return;

  QgsFeature feat;
  if ( mActiveLayer->getFeatures( QgsFeatureRequest( fid ) ).nextFeature( feat ) )
    updateFeature3D( feat );
}

// 完全重建内存图层数据
void ThreeDViewTool::onHeightPreviewChanged( QgsVectorLayer *layer, const QgsFeatureIds &fids, const QString &heightFieldName, double height )
{
  if ( !layer || layer != mActiveLayer || !mTempLayer )
    return;

  Q_UNUSED( heightFieldName )
  updatePreviewEntity( layer, fids, heightFieldName, height );
}

void ThreeDViewTool::onHeightPreviewFinished( QgsVectorLayer *layer, const QgsFeatureIds &fids, const QString &heightFieldName, double height )
{
  if ( !layer || layer != mActiveLayer || !mTempLayer )
    return;

  Q_UNUSED( heightFieldName )
  Q_UNUSED( height )

  clearPreviewEntity();
  mPendingFeatureUpdates.subtract( fids );

  for ( QgsFeatureId fid : fids )
    applyFeature3DUpdate( fid );

  refresh3DCanvases();
}

void ThreeDViewTool::onRoofModelChanged( QgsVectorLayer *layer, QgsFeatureId fid )
{
  if ( !layer || layer != mActiveLayer || !mTempLayer || fid == FID_NULL )
    return;

  applyFeature3DUpdate( fid );
  if ( mBuildingTriangleMeshMode && fid == mWireframeFid )
    updateWireframeLayer( layer, fid );
  refresh3DCanvases();
}

void ThreeDViewTool::onBuildingTriangleMeshModeChanged( QgsVectorLayer *layer, QgsFeatureId fid, bool enabled )
{
  mBuildingTriangleMeshMode = enabled && layer && layer == mActiveLayer && fid != FID_NULL;
  mWireframeFid = mBuildingTriangleMeshMode ? fid : FID_NULL;
  applyBuildingTriangleMeshMode();
  if ( mBuildingTriangleMeshMode )
    updateWireframeLayer( layer, fid );
  else
    clearWireframeLayer();
}

void ThreeDViewTool::refreshMemoryData()
{
  if ( !mTempLayer || !mActiveLayer || mSelectedHeightField.isEmpty() )
    return;

  mTempLayer->startEditing();
  mTempLayer->deleteFeatures( mTempLayer->allFeatureIds() );

  QgsFeatureIterator it = mActiveLayer->getFeatures();
  QgsFeature f;
  QgsFeatureList allTriangles;

  while ( it.nextFeature( f ) )
  {
    double h = f.attribute( mSelectedHeightField ).toDouble();
    if ( h <= 0 )
      h = 10.0;

    const MeshData mesh = buildMeshForFeature( f, h );
    QgsFeatureList triangles = buildBuildingFromMesh( mesh, QMatrix4x4() );

    for ( QgsFeature &tri : triangles )
    {
      tri.setAttributes( QgsAttributes() << f.id() << QStringLiteral( "full" ) );
      allTriangles.append( tri );
    }
  }

  mTempLayer->addFeatures( allTriangles );
  mTempLayer->commitChanges();
  mTempLayer->triggerRepaint();

  // 强制刷新 3D 视图
  refresh3DCanvases();
}
