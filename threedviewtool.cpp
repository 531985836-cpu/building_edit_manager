#include "threedviewtool.h"
#include "buildingeditpreviewbus.h"
#include "buildingroof.h"
#include "roofmodelsettings.h"
#include "roofmodelpipeline.h"
#include <qgsvectorlayer.h>
#include <qgspointcloudlayer.h>
#include <qgspointcloudattributebyramprenderer.h>
#include <qgspoint3dsymbol.h>
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
#include <qgsspatialindex.h>
#include <qgsunittypes.h>
#include <QMessageBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QKeyEvent>
#include <algorithm>
#include <array>
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
#include <qgsmarkersymbol.h>
#include <qgscategorizedsymbolrenderer.h>
#include <qgsrulebased3drenderer.h>
#include <qgslinesymbol.h>
#include <qgssinglesymbolrenderer.h>
#include <qgslayertreelayer.h>
#include <qgslayertree.h>
#include <qgslayertreeview.h>
#include <QMap>
#include <QVector3D>
#include <limits>
#include <map>
#include <optional>
#include <tuple>
#include <Qt3DCore/QEntity>
#include <Qt3DRender/QAttribute>
#include <Qt3DRender/QBuffer>
#include <Qt3DRender/QGeometry>
#include <Qt3DRender/QGeometryRenderer>
#include <Qt3DExtras/QPhongMaterial>

// ==================== 构造与析构 ====================
namespace
{
  QColor roofSegmentColor( int segmentId, bool accepted )
  {
    if ( !accepted )
      return QColor( 255, 125, 25 );

    const QList<QColor> colors = {
      QColor( 230, 45, 45 ),
      QColor( 20, 175, 80 ),
      QColor( 35, 105, 235 ),
      QColor( 210, 55, 205 ),
      QColor( 255, 205, 20 ),
      QColor( 0, 190, 205 ),
      QColor( 150, 75, 225 ),
      QColor( 235, 90, 150 )
    };
    return colors.at( std::max( 0, segmentId - 1 ) % colors.size() );
  }

  QgsMarkerSymbol *createRoofSegmentMarkerSymbol( const QColor &color, double size )
  {
    return QgsMarkerSymbol::createSimple(
      { { QStringLiteral( "name" ), QStringLiteral( "square" ) },
        { QStringLiteral( "color" ), QStringLiteral( "%1,%2,%3,235" ).arg( color.red() ).arg( color.green() ).arg( color.blue() ) },
        { QStringLiteral( "outline_style" ), QStringLiteral( "no" ) },
        { QStringLiteral( "size" ), QString::number( size, 'f', 1 ) } } ).release();
  }

  QgsPoint3DSymbol *createRoofSegmentPoint3DSymbol( const QColor &color )
  {
    QgsPoint3DSymbol *symbol = new QgsPoint3DSymbol();
    symbol->setAltitudeClamping( Qgis::AltitudeClamping::Absolute );
    symbol->setShape( Qgis::Point3DShape::Billboard );
    symbol->setBillboardSymbol( createRoofSegmentMarkerSymbol( color, 3.8 ) );

    QgsPhongMaterialSettings *material = new QgsPhongMaterialSettings();
    material->setAmbient( color.darker( 120 ) );
    material->setDiffuse( color );
    symbol->setMaterialSettings( material );
    return symbol;
  }

  QList<QgsColorRampShader::ColorRampItem> pointCloudElevationRampItems( double zMin, double zMax )
  {
    const QList<QColor> colors = {
      QColor( 35, 42, 165 ),
      QColor( 28, 90, 210 ),
      QColor( 0, 145, 255 ),
      QColor( 0, 205, 255 ),
      QColor( 0, 230, 190 ),
      QColor( 0, 190, 90 ),
      QColor( 55, 210, 60 ),
      QColor( 145, 220, 0 ),
      QColor( 225, 235, 0 ),
      QColor( 255, 210, 0 ),
      QColor( 255, 165, 0 ),
      QColor( 255, 105, 0 ),
      QColor( 245, 55, 0 ),
      QColor( 220, 0, 35 ),
      QColor( 190, 0, 120 ),
      QColor( 135, 0, 180 )
    };

    const double range = zMax - zMin;
    QList<QgsColorRampShader::ColorRampItem> items;
    items.reserve( colors.size() );
    for ( int i = 0; i < colors.size(); ++i )
    {
      const double t = colors.size() <= 1 ? 0.0 : static_cast<double>( i ) / static_cast<double>( colors.size() - 1 );
      const double value = i == colors.size() - 1 ? zMax : zMin + range * t;
      items << QgsColorRampShader::ColorRampItem( value, colors.at( i ), QString::number( value, 'f', 2 ) );
    }
    return items;
  }

  QgsColorRampShader pointCloudElevationShader( double zMin, double zMax )
  {
    QgsColorRampShader shader( zMin, zMax, nullptr, Qgis::ShaderInterpolationMethod::Discrete, Qgis::ShaderClassificationMethod::Continuous );
    shader.setColorRampType( Qgis::ShaderInterpolationMethod::Discrete );
    shader.setClassificationMode( Qgis::ShaderClassificationMethod::Continuous );
    shader.setColorRampItemList( pointCloudElevationRampItems( zMin, zMax ) );
    return shader;
  }

  QString nonNegativePointCloudSubset( const QString &currentSubset )
  {
    const QString zFilter = QStringLiteral( "Z >= 0" );
    if ( currentSubset.trimmed().isEmpty() )
      return zFilter;
    if ( currentSubset.contains( zFilter, Qt::CaseInsensitive ) )
      return currentSubset;
    return QStringLiteral( "(%1) AND (%2)" ).arg( currentSubset, zFilter );
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
  connect( BuildingEditPreviewBus::instance(), &BuildingEditPreviewBus::roofPreviewChanged, this, &ThreeDViewTool::onRoofPreviewChanged );
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
  for ( const QMetaObject::Connection &connection : mDisplayedMeshConnections )
    disconnect( connection );
  mDisplayedMeshConnections.clear();
  mDisplayedMeshLayer = nullptr;
  clearDisplayedMeshCache();
  restoreLayerVisibilityAfterTriangleMesh();
  clearPreviewEntity();
  clearWireframeLayer();
  clearRoofSegmentationLayer();

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

void ThreeDViewTool::hideActiveLayerForTriangleMesh()
{
  if ( !mActiveLayer )
    return;

  QgsLayerTree *root = QgsProject::instance()->layerTreeRoot();
  if ( !root )
    return;

  if ( mTriangleMeshHasHiddenLayerState )
  {
    if ( mTriangleMeshHiddenLayerId == mActiveLayer->id() )
      return;
    restoreLayerVisibilityAfterTriangleMesh();
  }

  QgsLayerTreeLayer *activeNode = root->findLayer( mActiveLayer->id() );
  if ( !activeNode )
    return;

  mTriangleMeshHiddenLayerId = mActiveLayer->id();
  mTriangleMeshHiddenLayerWasVisible = activeNode->itemVisibilityChecked();
  mTriangleMeshHasHiddenLayerState = true;
  activeNode->setItemVisibilityChecked( false );

  if ( mCanvas )
    mCanvas->refresh();
}

void ThreeDViewTool::restoreLayerVisibilityAfterTriangleMesh()
{
  if ( !mTriangleMeshHasHiddenLayerState )
    return;

  QgsLayerTree *root = QgsProject::instance()->layerTreeRoot();
  if ( root )
  {
    QgsLayerTreeLayer *hiddenNode = root->findLayer( mTriangleMeshHiddenLayerId );
    if ( hiddenNode )
      hiddenNode->setItemVisibilityChecked( mTriangleMeshHiddenLayerWasVisible );
  }

  mTriangleMeshHiddenLayerId.clear();
  mTriangleMeshHiddenLayerWasVisible = false;
  mTriangleMeshHasHiddenLayerState = false;

  if ( mCanvas )
    mCanvas->refresh();
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

  const QString crs = mActiveLayer && mActiveLayer->crs().isValid() ? mActiveLayer->crs().authid() : QString();
  mWireframeLayer = new QgsVectorLayer(
    QStringLiteral( "LineStringZ?crs=%1&field=original_fid:long" ).arg( crs ),
    QStringLiteral( "Building_Wireframe_Current" ),
    QStringLiteral( "memory" )
  );

  if ( !mWireframeLayer || !mWireframeLayer->isValid() )
    return;

  std::unique_ptr<QgsLineSymbol> wireframe2DSymbol = QgsLineSymbol::createSimple(
    { { QStringLiteral( "line_color" ), QStringLiteral( "0,0,0,230" ) },
      { QStringLiteral( "line_width" ), QStringLiteral( "0.65" ) },
      { QStringLiteral( "line_joinstyle" ), QStringLiteral( "miter" ) },
      { QStringLiteral( "line_capstyle" ), QStringLiteral( "square" ) } }
  );
  mWireframeLayer->setRenderer( new QgsSingleSymbolRenderer( wireframe2DSymbol.release() ) );
  mWireframeLayer->setOpacity( 1.0 );
  mWireframeLayer->setFlags( mWireframeLayer->flags() & ~QgsMapLayer::Identifiable );
  mWireframeLayer->setFlags( mWireframeLayer->flags() & ~QgsMapLayer::Searchable );

  QgsLine3DSymbol *symbol = new QgsLine3DSymbol();
  symbol->setAltitudeClamping( Qgis::AltitudeClamping::Absolute );
  symbol->setAltitudeBinding( Qgis::AltitudeBinding::Vertex );
  symbol->setRenderAsSimpleLines( true );
  symbol->setWidth( 1.8f );

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
    treeLayer->setItemVisibilityChecked( true );

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

void ThreeDViewTool::ensureRoofSegmentationLayer()
{
  if ( mRoofSegmentationLayer )
    return;

  const QString crs = mActiveLayer && mActiveLayer->crs().isValid() ? mActiveLayer->crs().authid() : QString();
  mRoofSegmentationLayer = new QgsVectorLayer(
    QStringLiteral( "PointZ?crs=%1&field=original_fid:long&field=class_id:int&field=segment_id:int&field=status:string&field=area_ratio:double" ).arg( crs ),
    QStringLiteral( "Roof_RANSAC_Segmentation_Debug" ),
    QStringLiteral( "memory" )
  );
  if ( !mRoofSegmentationLayer || !mRoofSegmentationLayer->isValid() )
    return;

  mRoofSegmentationLayer->setFlags( mRoofSegmentationLayer->flags() & ~QgsMapLayer::Identifiable );
  mRoofSegmentationLayer->setFlags( mRoofSegmentationLayer->flags() & ~QgsMapLayer::Searchable );
  QgsProject::instance()->addMapLayer( mRoofSegmentationLayer, false );
  QgsLayerTreeLayer *treeLayer = QgsProject::instance()->layerTreeRoot()->addLayer( mRoofSegmentationLayer );
  if ( treeLayer )
    treeLayer->setItemVisibilityChecked( true );
  ensureLayerIn3DView( mRoofSegmentationLayer );
}

void ThreeDViewTool::clearRoofSegmentationLayer()
{
  if ( !mRoofSegmentationLayer )
    return;

  for ( Qgs3DMapCanvas *canvas3D : mIface ? mIface->mapCanvases3D() : QList<Qgs3DMapCanvas *>() )
  {
    if ( !canvas3D || !canvas3D->mapSettings() )
      continue;
    QList<QgsMapLayer *> layers = canvas3D->mapSettings()->layers();
    if ( layers.removeAll( mRoofSegmentationLayer ) > 0 )
      canvas3D->mapSettings()->setLayers( layers );
  }

  const QString layerId = mRoofSegmentationLayer->id();
  mRoofSegmentationLayer = nullptr;
  QgsProject::instance()->removeMapLayer( layerId );
}

void ThreeDViewTool::updateRoofSegmentationLayer( QgsVectorLayer *layer, QgsFeatureId fid )
{
  if ( !mShowRoofSegmentationDebug || !mBuildingTriangleMeshMode || !layer || layer != mActiveLayer || fid == FID_NULL )
  {
    clearRoofSegmentationLayer();
    return;
  }

  QgsFeature feature;
  if ( !layer->getFeatures( QgsFeatureRequest( fid ) ).nextFeature( feature ) )
  {
    clearRoofSegmentationLayer();
    return;
  }

  const RoofModelSettings::Selection selection = RoofModelSettings::read( layer, feature.id() );
  const QVector<BuildingRoof::RoofSample> samples = RoofModelPipeline::collectSamples( feature.geometry(), selection.pointCloudLayerId );
  const QList<BuildingRoof::RoofPoint> roofPoints = RoofModelSettings::builderMarkers( selection.type );
  int ridgePointCount = 0;
  int boundaryPointCount = 0;
  for ( const BuildingRoof::RoofPoint &roofPoint : roofPoints )
  {
    if ( roofPoint.type.contains( QStringLiteral( "屋脊" ) )
         || roofPoint.type.contains( QStringLiteral( "灞嬭剨" ) )
         || roofPoint.type.contains( QStringLiteral( "ridge" ), Qt::CaseInsensitive ) )
    {
      ++ridgePointCount;
      continue;
    }
    if ( !roofPoint.type.contains( QStringLiteral( "地面" ) )
         && !roofPoint.type.contains( QStringLiteral( "鍦伴潰" ) )
         && !roofPoint.type.contains( QStringLiteral( "ground" ), Qt::CaseInsensitive )
         && !roofPoint.type.contains( QStringLiteral( "曲面点" ) )
         && !roofPoint.type.contains( QStringLiteral( "surface" ), Qt::CaseInsensitive )
         && !roofPoint.type.contains( QStringLiteral( "顶点" ) )
         && !roofPoint.type.contains( QStringLiteral( "vertex" ), Qt::CaseInsensitive ) )
      ++boundaryPointCount;
  }
  const bool flatTopRoof = boundaryPointCount == 2 && ridgePointCount == 1;
  const BuildingRoof::RoofPlaneSegmentation segmentation = flatTopRoof
                                                           ? BuildingRoof::segmentFlatTopRoofCloud( feature.geometry(), samples )
                                                           : BuildingRoof::segmentRoofPlanesForDebug( feature.geometry(), samples, ridgePointCount >= 2 );
  ensureRoofSegmentationLayer();
  if ( !mRoofSegmentationLayer )
    return;

  QgsFeatureList features;
  QgsCategoryList categories;
  QgsRuleBased3DRenderer::Rule *rootRule = new QgsRuleBased3DRenderer::Rule( nullptr );
  auto addClass = [&]( int classId, int segmentId, bool accepted, double areaRatio, const QVector<QgsPoint> &points ) {
    const QColor color = classId == -1 ? QColor( 45, 45, 45 )
                         : segmentId == 0 ? QColor( 125, 125, 125 ) : roofSegmentColor( segmentId, accepted );
    const QString status = classId == -1 ? tr( "候选范围外" )
                           : segmentId == 0 ? tr( "未归类候选点" ) : ( accepted ? tr( "主要候选面" ) : tr( "小面积或非主连通候选面" ) );
    const QString label = classId == -1
                            ? tr( "候选范围外（%1 点）" ).arg( points.size() )
                            : segmentId == 0
                              ? ( flatTopRoof ? tr( "未分入平面的点（%1 点）" ) : tr( "未归类候选点（含高度带外，%1 点）" ) ).arg( points.size() )
                              : tr( "平面 %1 - %2（投影占比 %3%，%4 点）" ).arg( segmentId ).arg( status ).arg( areaRatio * 100.0, 0, 'f', 1 ).arg( points.size() );
    categories << QgsRendererCategory( classId, createRoofSegmentMarkerSymbol( color, segmentId <= 0 ? 2.2 : 3.2 ), label );
    rootRule->appendChild( new QgsRuleBased3DRenderer::Rule(
      createRoofSegmentPoint3DSymbol( color ),
      QStringLiteral( "\"class_id\" = %1" ).arg( classId ),
      label ) );

    for ( const QgsPoint &point : points )
    {
      QgsFeature debugPoint( mRoofSegmentationLayer->fields() );
      debugPoint.setGeometry( QgsGeometry::fromPoint( point ) );
      debugPoint.setAttributes( QgsAttributes() << fid << classId << segmentId << status << areaRatio );
      features.append( debugPoint );
    }
  };

  struct DebugClass
  {
    int segmentId = 0;
    bool accepted = false;
    double areaRatio = 0.0;
    QVector<QgsPoint> points;
  };
  QMap<int, DebugClass> classes;
  using PointKey = std::tuple<double, double, double>;
  auto pointKey = []( const QgsPoint &point ) -> PointKey {
    return std::make_tuple( point.x(), point.y(), point.z() );
  };
  std::map<PointKey, int> classByPoint;
  for ( const BuildingRoof::RoofPlaneSegment &segment : segmentation.segments )
  {
    const int classId = segment.meetsAreaReference ? segment.id : 100 + segment.id;
    DebugClass &debugClass = classes[classId];
    debugClass.segmentId = segment.id;
    debugClass.accepted = segment.meetsAreaReference;
    debugClass.areaRatio = segment.projectedAreaRatio;
    for ( const QgsPoint &point : segment.points )
      classByPoint[pointKey( point )] = classId;
  }
  for ( const QgsPoint &point : segmentation.unclassifiedPoints )
    classByPoint.emplace( pointKey( point ), 0 );

  for ( const BuildingRoof::RoofSample &sample : samples )
  {
    const auto found = classByPoint.find( pointKey( sample.point ) );
    const int classId = found == classByPoint.end() ? -1 : found->second;
    classes[classId].points.append( sample.point );
  }
  for ( auto it = classes.cbegin(); it != classes.cend(); ++it )
    addClass( it.key(), it.value().segmentId, it.value().accepted, it.value().areaRatio, it.value().points );

  mRoofSegmentationLayer->startEditing();
  mRoofSegmentationLayer->deleteFeatures( mRoofSegmentationLayer->allFeatureIds() );
  mRoofSegmentationLayer->addFeatures( features );
  mRoofSegmentationLayer->commitChanges();
  mRoofSegmentationLayer->setRenderer( new QgsCategorizedSymbolRenderer( QStringLiteral( "class_id" ), categories ) );

  QgsRuleBased3DRenderer *renderer3D = new QgsRuleBased3DRenderer( rootRule );
  renderer3D->setLayer( mRoofSegmentationLayer );
  mRoofSegmentationLayer->setRenderer3D( renderer3D );
  mRoofSegmentationLayer->triggerRepaint();
  ensureLayerIn3DView( mRoofSegmentationLayer );
}

QgsFeatureList ThreeDViewTool::buildStructuralWireframe( const MeshData &mesh, QgsFeatureId fid ) const
{
  struct Edge
  {
    QgsPoint start;
    QgsPoint end;
    QgsVector3D normal;
    bool interior = false;
    bool crease = false;
  };

  const auto finite = []( const QgsPoint &point ) {
    return std::isfinite( point.x() ) && std::isfinite( point.y() ) && std::isfinite( point.z() );
  };

  // Compare face angles in compatible horizontal and vertical units.
  const double xyScale = mActiveLayer && mActiveLayer->crs().isValid()
                         ? QgsUnitTypes::fromUnitToUnitFactor( mActiveLayer->crs().mapUnits(), Qgis::DistanceUnit::Meters ) : 1.0;
  constexpr double tolerance = 1e-7;
  using Vertex = std::tuple<double, double, double>;
  const auto vertexKey = [xyScale]( const QgsPoint &point ) {
    // Facets clipped independently may produce slightly different coordinates for the same corner.
    return Vertex( std::round( point.x() * xyScale / tolerance ), std::round( point.y() * xyScale / tolerance ),
                   std::round( point.z() / tolerance ) );
  };
  const auto offset = [xyScale]( const QgsPoint &from, const QgsPoint &to ) {
    return QgsVector3D( ( to.x() - from.x() ) * xyScale, ( to.y() - from.y() ) * xyScale, to.z() - from.z() );
  };
  // Smooth roofs also suppress small dihedral angles caused by tessellating a curved surface.
  const double normalThreshold = mesh.smoothRoof ? 0.9659258263 : 0.9999984769; // 15 degrees or 0.1 degrees
  const auto continuousSurface = [normalThreshold]( const QgsVector3D &first, const QgsVector3D &second, double dot ) {
    constexpr double verticalTolerance = 1e-8;
    return std::abs( dot ) >= normalThreshold
           && ( std::abs( first.z() ) <= verticalTolerance ) == ( std::abs( second.z() ) <= verticalTolerance );
  };

  std::map<std::pair<Vertex, Vertex>, Edge> edges;
  for ( int i = 0; i + 2 < mesh.indices.size(); i += 3 )
  {
    const int a = mesh.indices[i], b = mesh.indices[i + 1], c = mesh.indices[i + 2];
    if ( a < 0 || b < 0 || c < 0 || a >= mesh.vertices.size() || b >= mesh.vertices.size() || c >= mesh.vertices.size() )
      continue;

    const std::array<const QgsPoint *, 3> points{ &mesh.vertices[a], &mesh.vertices[b], &mesh.vertices[c] };
    if ( !finite( *points[0] ) || !finite( *points[1] ) || !finite( *points[2] ) )
      continue;
    const QgsVector3D normal = QgsVector3D::crossProduct( offset( *points[0], *points[1] ), offset( *points[0], *points[2] ) );
    const double area = normal.length();
    if ( area <= 0.0 || !std::isfinite( area ) )
      continue;

    for ( int side = 0; side < 3; ++side )
    {
      const QgsPoint &a = *points[side], &b = *points[( side + 1 ) % 3];
      const Vertex first = vertexKey( a ), last = vertexKey( b );
      if ( first == last )
        continue;
      const bool forward = first < last;
      // Orient the normal relative to the canonical edge direction. Adjacent coplanar triangles then face opposite ways.
      const QgsVector3D orientedNormal = normal / ( forward ? area : -area );
      const auto [it, inserted] = edges.try_emplace( forward ? std::make_pair( first, last ) : std::make_pair( last, first ),
                                                   Edge{ forward ? a : b, forward ? b : a, orientedNormal } );
      if ( inserted )
        continue;
      Edge &edge = it->second;
      const double dot = QgsVector3D::dotProduct( edge.normal, orientedNormal );
      if ( continuousSurface( edge.normal, orientedNormal, dot ) )
        edge.interior |= dot < 0.0;
      else
        edge.crease = true;
    }
  }

  QgsFeatureList lines;
  const auto appendLine = [&lines, fid]( const QgsPoint &start, const QgsPoint &end ) {
    QgsFeature line;
    line.setGeometry( QgsGeometry( std::make_unique<QgsLineString>( start, end ) ) );
    line.setAttributes( QgsAttributes() << fid );
    lines.append( line );
  };

  // Independently triangulated regions can meet along one long edge on one side and several short edges on the other.
  // Match the remaining collinear spans in 3D so those coplanar joins are also hidden.
  QVector<const Edge *> openEdges;
  QgsSpatialIndex index;
  const auto bounds = []( const Edge &edge ) { return QgsRectangle( edge.start.x(), edge.start.y(), edge.end.x(), edge.end.y() ); };
  for ( const auto &entry : edges )
  {
    const Edge &edge = entry.second;
    if ( edge.crease )
      appendLine( edge.start, edge.end );
    else if ( !edge.interior )
    {
      index.addFeature( openEdges.size(), bounds( edge ) );
      openEdges.append( &edge );
    }
  }

  for ( int i = 0; i < openEdges.size(); ++i )
  {
    const Edge &edge = *openEdges[i];
    const QgsVector3D axis = offset( edge.start, edge.end );
    const double length = axis.length();
    if ( length <= tolerance )
      continue;
    const QgsVector3D direction = axis / length;
    QgsRectangle nearby = bounds( edge );
    nearby.grow( tolerance / xyScale );
    QVector<std::pair<double, double>> covered;
    for ( QgsFeatureId id : index.intersects( nearby ) )
    {
      if ( id == i )
        continue;
      const Edge &other = *openEdges[id];
      const QgsVector3D first = offset( edge.start, other.start ), last = offset( edge.start, other.end );
      const double from = QgsVector3D::dotProduct( direction, first ), to = QgsVector3D::dotProduct( direction, last );
      const double overlapStart = std::max( 0.0, std::min( from, to ) ), overlapEnd = std::min( length, std::max( from, to ) );
      if ( overlapEnd - overlapStart <= tolerance )
        continue;
      const double dot = QgsVector3D::dotProduct( edge.normal, from < to ? other.normal : -other.normal );
      if ( dot >= 0.0 || !continuousSurface( edge.normal, other.normal, dot )
           || QgsVector3D::crossProduct( direction, first ).length() > tolerance
           || QgsVector3D::crossProduct( direction, last ).length() > tolerance )
        continue;
      covered.append( { overlapStart, overlapEnd } );
    }
    std::sort( covered.begin(), covered.end() );
    const auto pointAt = [&edge, length]( double distance ) {
      const double t = distance / length;
      return QgsPoint( edge.start.x() + t * ( edge.end.x() - edge.start.x() ),
                       edge.start.y() + t * ( edge.end.y() - edge.start.y() ),
                       edge.start.z() + t * ( edge.end.z() - edge.start.z() ) );
    };
    double start = 0.0;
    for ( const auto &[from, to] : covered )
    {
      if ( from - start > tolerance )
        appendLine( pointAt( start ), pointAt( from ) );
      start = std::max( start, to );
    }
    if ( length - start > tolerance )
      appendLine( pointAt( start ), edge.end );
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

  QgsFeatureList lines = buildStructuralWireframe( mesh, fid );
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
    request.setFlags( Qgis::FeatureRequestFlag::NoGeometry );
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

MeshData ThreeDViewTool::buildMeshForFeature( const QgsFeature &feature, double height )
{
  const RoofModelSettings::Selection selection = RoofModelSettings::read( mActiveLayer, feature.id() );
  if ( selection.hasRoof() )
  {
    const BuildingRoof::MeshResult result = RoofModelPipeline::build( feature.geometry(), selection );
    if ( result.success )
    {
      const bool smoothRoof = selection.regions.isEmpty() && ( selection.type == RoofModelSettings::Type::Barrel
                                                              || selection.type == RoofModelSettings::Type::Dome );
      return { result.mesh.vertices, result.mesh.indices, smoothRoof };
    }
  }

  return BuildMesh::build( feature.geometry(), height );
}

void ThreeDViewTool::ensurePreviewEntity( Qgs3DMapScene *scene )
{
  if ( mPreviewEntity && mPreviewEntity->parent() == scene )
    return;
  clearPreviewEntity();
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

QVector<float> ThreeDViewTool::previewVertexData( QgsVectorLayer *layer, const Qgs3DMapSettings &settings,
                                                   const QVector<MeshData> &meshes, bool wireframe ) const
{
  QgsCoordinateTransform layerToScene;
  if ( layer->crs().isValid() && settings.crs().isValid() && layer->crs() != settings.crs() )
    layerToScene = QgsCoordinateTransform( layer->crs(), settings.crs(), settings.transformContext() );

  const auto worldPoint = [&]( const QgsPoint &point ) -> std::optional<QVector3D> {
    QgsPointXY xy( point );
    try
    {
      if ( layerToScene.isValid() )
        xy = layerToScene.transform( xy );
    }
    catch ( const QgsCsException & )
    {
      return std::nullopt;
    }
    const auto world = settings.mapToWorldCoordinates( QgsVector3D( xy.x(), xy.y(), point.z() ) );
    if ( !std::isfinite( world.x() ) || !std::isfinite( world.y() ) || !std::isfinite( world.z() ) )
      return std::nullopt;
    return QVector3D( world.x(), world.y(), world.z() );
  };

  QVector<float> raw;
  const QVector3D up( 0.0f, 0.0f, 1.0f );
  const auto append = [&]( const QVector3D &point, const QVector3D &normal ) {
    raw << point.x() << point.y() << point.z() << normal.x() << normal.y() << normal.z();
  };
  for ( const auto &mesh : meshes )
  {
    if ( wireframe )
    {
      const auto lines = buildStructuralWireframe( mesh, FID_NULL );
      for ( const QgsFeature &line : lines )
      {
        const QgsGeometry geometry = line.geometry();
        const auto a = worldPoint( geometry.vertexAt( 0 ) ), b = worldPoint( geometry.vertexAt( 1 ) );
        if ( a && b )
        {
          append( *a, up );
          append( *b, up );
        }
      }
      continue;
    }

    for ( int i = 0; i + 2 < mesh.indices.size(); i += 3 )
    {
      const int a = mesh.indices[i], b = mesh.indices[i + 1], c = mesh.indices[i + 2];
      if ( a < 0 || b < 0 || c < 0 || a >= mesh.vertices.size() || b >= mesh.vertices.size() || c >= mesh.vertices.size() )
        continue;
      const auto p = worldPoint( mesh.vertices[a] ), q = worldPoint( mesh.vertices[b] ), r = worldPoint( mesh.vertices[c] );
      if ( !p || !q || !r )
        continue;
      QVector3D normal = QVector3D::normal( *q - *p, *r - *p );
      if ( normal.isNull() )
        normal = up;
      append( *p, normal );
      append( *q, normal );
      append( *r, normal );
    }
  }
  return raw;
}

bool ThreeDViewTool::renderPreview( QgsVectorLayer *layer, const QgsFeatureIds &fids, const QVector<MeshData> &meshes, QgsFeatureId roofFid )
{
  if ( !layer || !mIface || fids.isEmpty() )
    return false;

  Qgs3DMapCanvas *activeCanvas3D = mIface->mapCanvases3D().isEmpty()
                                     ? mIface->createNewMapCanvas3D( tr( "3D Preview" ) )
                                     : mIface->mapCanvases3D().first();
  if ( !activeCanvas3D || !activeCanvas3D->mapSettings() || !activeCanvas3D->scene() )
    return false;

  const bool wireframe = roofFid != FID_NULL && mBuildingTriangleMeshMode && mWireframeFid == roofFid;
  const QVector<float> raw = previewVertexData( layer, *activeCanvas3D->mapSettings(), meshes, wireframe );
  if ( raw.isEmpty() )
    return false;

  const QgsFeatureIds previous = mPreviewFids;
  ensurePreviewEntity( activeCanvas3D->scene() );
  if ( !mPreviewEntity || !mPreviewRenderer || !mPreviewVertexBuffer || !mPreviewPositionAttribute || !mPreviewNormalAttribute || !mPreviewMaterial )
    return false;

  mPreviewFids = fids;
  mRoofPreviewFid = roofFid;
  if ( previous != fids )
  {
    removeTempFeatures( fids - previous );
    for ( QgsFeatureId fid : previous - fids )
      applyFeature3DUpdate( fid );
  }

  mPreviewRenderer->setPrimitiveType( wireframe ? Qt3DRender::QGeometryRenderer::Lines : Qt3DRender::QGeometryRenderer::Triangles );
  mPreviewMaterial->setAmbient( wireframe ? QColor( 20, 20, 20 ) : QColor( 90, 70, 35 ) );
  mPreviewMaterial->setDiffuse( wireframe ? QColor( 25, 25, 25 ) : QColor( 255, 180, 60, 230 ) );
  mPreviewMaterial->setSpecular( wireframe ? Qt::black : QColor( 80, 80, 80 ) );

  const QByteArray vertices( reinterpret_cast<const char *>( raw.constData() ), raw.size() * sizeof( float ) );
  const int vertexCount = raw.size() / 6;
  mPreviewVertexBuffer->setData( vertices );
  mPreviewPositionAttribute->setCount( vertexCount );
  mPreviewNormalAttribute->setCount( vertexCount );
  mPreviewRenderer->setVertexCount( vertexCount );
  mPreviewEntity->setEnabled( true );
  return true;
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
  mRoofPreviewFid = FID_NULL;

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
  if ( mBuildingTriangleMeshMode || mTriangleMeshHasHiddenLayerState )
  {
    restoreLayerVisibilityAfterTriangleMesh();
    mBuildingTriangleMeshMode = false;
    mWireframeFid = FID_NULL;
    clearWireframeLayer();
    clearRoofSegmentationLayer();
    applyBuildingTriangleMeshMode();
  }

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

    layer->setSubsetString( nonNegativePointCloudSubset( layer->subsetString() ) );
    configurePointCloudRenderers( layer );
    QPointer<QgsPointCloudLayer> safeLayer = layer;
    connect( layer, &QgsPointCloudLayer::statisticsCalculationStateChanged, this, [this, safeLayer]( QgsPointCloudLayer::PointCloudStatisticsCalculationState state ) {
      if ( state != QgsPointCloudLayer::PointCloudStatisticsCalculationState::Calculated )
        return;
      if ( !safeLayer || !mIface )
        return;

      configurePointCloudRenderers( safeLayer );
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

void ThreeDViewTool::configurePointCloudRenderers( QgsPointCloudLayer *layer )
{
  if ( !layer )
    return;

  layer->setSubsetString( nonNegativePointCloudSubset( layer->subsetString() ) );

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
    zMin = 0.0;
  if ( !std::isfinite( zMax ) )
    zMax = 50.0;
  zMin = std::max( 0.0, zMin );
  if ( zMax < 0.0 )
    zMax = 1.0;
  if ( zMax <= zMin + 1e-9 )
  {
    zMin = std::max( 0.0, zMin - 1.0 );
    zMax += 1.0;
  }

  const QgsColorRampShader shader = pointCloudElevationShader( zMin, zMax );

  QgsPointCloudAttributeByRampRenderer *renderer2D = new QgsPointCloudAttributeByRampRenderer();
  renderer2D->setAttribute( zAttribute );
  renderer2D->setMinimum( zMin );
  renderer2D->setMaximum( zMax );
  renderer2D->setColorRampShader( shader );
  renderer2D->setPointSize( 1.2 );
  renderer2D->setMaximumScreenError( 0.5 );
  layer->setRenderer( renderer2D );

  QgsColorRampPointCloud3DSymbol *symbol = new QgsColorRampPointCloud3DSymbol();
  symbol->setAttribute( zAttribute );
  symbol->setColorRampShader( shader );
  symbol->setColorRampShaderMinMax( zMin, zMax );
  symbol->setPointSize( 5.0f );

  QgsPointCloudLayer3DRenderer *renderer = new QgsPointCloudLayer3DRenderer();
  renderer->setLayer( layer );
  renderer->setSymbol( symbol );
  renderer->setMaximumScreenError( 1.0 );
  renderer->setPointRenderingBudget( 5000000 );
  layer->setRenderer3D( renderer );
  layer->triggerRepaint();
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
  for ( int i = current3DLayers.size() - 1; i >= 0; --i )
  {
    if ( current3DLayers[i] && RoofModelSettings::isLegacyPointLayer( current3DLayers[i]->name() ) )
      current3DLayers.removeAt( i );
  }
  if ( !RoofModelSettings::isLegacyPointLayer( layer->name() ) && !current3DLayers.contains( layer ) )
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

    configurePointCloudRenderers( pointCloudLayer );
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
void ThreeDViewTool::clearDisplayedMeshCache()
{
  if ( mDisplayedMeshFid == FID_NULL )
    return;
  mDisplayedMeshFid = FID_NULL;
  mDisplayedMesh = MeshData();
}

// 根据原始要素更新其对应的 3D 模型
void ThreeDViewTool::updateFeature3D( const QgsFeature &originFeat )
{
  if ( !mTempLayer || mPreviewFids.contains( originFeat.id() ) )
    return;

  if ( mDisplayedMeshLayer != mTempLayer )
  {
    for ( const QMetaObject::Connection &connection : mDisplayedMeshConnections )
      disconnect( connection );
    mDisplayedMeshConnections.clear();
    mDisplayedMeshLayer = mTempLayer;
    clearDisplayedMeshCache();
    mDisplayedMeshConnections.append( connect( mTempLayer.data(), &QgsMapLayer::dataChanged,
                                               this, &ThreeDViewTool::clearDisplayedMeshCache ) );
    mDisplayedMeshConnections.append( connect( mTempLayer.data(), &QgsVectorLayer::layerModified,
                                               this, &ThreeDViewTool::clearDisplayedMeshCache ) );
    mDisplayedMeshConnections.append( connect( mTempLayer.data(), &QObject::destroyed,
                                               this, &ThreeDViewTool::clearDisplayedMeshCache ) );
  }

  double h = originFeat.attribute( mSelectedHeightField ).toDouble();
  if ( h <= 0 )
    h = 5.0;

  const MeshData mesh = buildMeshForFeature( originFeat, h );
  if ( mesh.isEmpty() )
    return;

  // Shared buffers identify the exact mesh last committed to this feature, without approximate geometry comparisons.
  const bool alreadyDisplayed = !mTempLayer->isEditable() && mDisplayedMeshFid == originFeat.id()
                                && mDisplayedMesh.vertices.size() == mesh.vertices.size()
                                && mDisplayedMesh.indices.size() == mesh.indices.size()
                                && mDisplayedMesh.vertices.constData() == mesh.vertices.constData()
                                && mDisplayedMesh.indices.constData() == mesh.indices.constData();
  if ( !alreadyDisplayed )
  {
    QgsFeatureList newTriangles = buildBuildingFromMesh( mesh, QMatrix4x4() );
    for ( QgsFeature &tri : newTriangles )
      tri.setAttributes( QgsAttributes() << originFeat.id() << QStringLiteral( "full" ) );

    mTempLayer->startEditing();
    QgsFeatureRequest request;
    request.setFilterExpression( QString( "original_fid = %1" ).arg( originFeat.id() ) );
    request.setFlags( Qgis::FeatureRequestFlag::NoGeometry );
    QgsFeatureIterator it = mTempLayer->getFeatures( request );
    QgsFeature f;
    QgsFeatureIds toDelete;
    while ( it.nextFeature( f ) )
      toDelete << f.id();
    const bool removed = mTempLayer->deleteFeatures( toDelete );
    const bool added = mTempLayer->addFeatures( newTriangles );
    const bool committed = mTempLayer->commitChanges();
    if ( removed && added && committed )
    {
      mDisplayedMeshFid = originFeat.id();
      mDisplayedMesh = mesh;
    }
    mTempLayer->triggerRepaint();
  }

  if ( mBuildingTriangleMeshMode && originFeat.id() == mWireframeFid )
  {
    updateWireframeLayerFromMesh( mesh, originFeat.id() );
    updateRoofSegmentationLayer( mActiveLayer, originFeat.id() );
  }
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
  if ( fids.contains( mRoofPreviewFid ) )
    clearPreviewEntity();
  if ( !mTempLayer )
    return;

  mTempLayer->startEditing();
  for ( QgsFeatureId fid : fids )
  {
    QgsFeatureRequest request;
    request.setFilterExpression( QString( "original_fid = %1" ).arg( fid ) );
    request.setFlags( Qgis::FeatureRequestFlag::NoGeometry );
    QgsFeatureIterator it = mTempLayer->getFeatures( request );
    QgsFeature f;
    QgsFeatureIds toDelete;
    while ( it.nextFeature( f ) )
      toDelete << f.id();
    mTempLayer->deleteFeatures( toDelete );
    if ( fid == mWireframeFid )
    {
      clearWireframeLayer();
      clearRoofSegmentationLayer();
    }
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
  QVector<MeshData> meshes;
  meshes.reserve( fids.size() );
  for ( QgsFeatureId fid : fids )
  {
    QgsFeature feature;
    if ( layer->getFeatures( QgsFeatureRequest( fid ) ).nextFeature( feature ) )
      meshes.append( buildMeshForFeature( feature, height ) );
  }
  renderPreview( layer, fids, meshes );
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

void ThreeDViewTool::onRoofPreviewChanged( QgsVectorLayer *layer, QgsFeatureId fid, const BuildingRoof::Mesh &mesh, bool dragging )
{
  if ( !layer || layer != mActiveLayer || !mTempLayer || fid == FID_NULL )
    return;

  if ( !renderPreview( layer, { fid }, { MeshData{ mesh.vertices, mesh.indices } }, fid ) && !dragging )
    onRoofModelChanged( layer, fid );
}

void ThreeDViewTool::onRoofModelChanged( QgsVectorLayer *layer, QgsFeatureId fid )
{
  if ( !layer || layer != mActiveLayer || !mTempLayer || fid == FID_NULL )
    return;

  if ( mRoofPreviewFid == fid )
    clearPreviewEntity();
  mPendingFeatureUpdates.remove( fid );
  if ( mFeatureUpdateTimer && mPendingFeatureUpdates.isEmpty() )
    mFeatureUpdateTimer->stop();
  applyFeature3DUpdate( fid );
  refresh3DCanvases();
}

void ThreeDViewTool::onBuildingTriangleMeshModeChanged( QgsVectorLayer *layer, QgsFeatureId fid, bool enabled )
{
  const bool nextMode = enabled && layer && layer == mActiveLayer && fid != FID_NULL;
  const QgsFeatureId nextFid = nextMode ? fid : FID_NULL;
  if ( mBuildingTriangleMeshMode == nextMode && mWireframeFid == nextFid )
    return;
  mBuildingTriangleMeshMode = nextMode;
  mWireframeFid = nextFid;
  applyBuildingTriangleMeshMode();
  if ( mBuildingTriangleMeshMode )
  {
    hideActiveLayerForTriangleMesh();
    if ( mRoofPreviewFid != fid )
    {
      updateWireframeLayer( layer, fid );
      updateRoofSegmentationLayer( layer, fid );
    }
  }
  else
  {
    restoreLayerVisibilityAfterTriangleMesh();
    clearWireframeLayer();
    clearRoofSegmentationLayer();
  }
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
    if ( mPreviewFids.contains( f.id() ) )
      continue;
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
