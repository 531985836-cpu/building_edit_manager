#include "threedviewtool.h"
#include "buildingeditpreviewbus.h"
#include "buildingroof.h"
#include <qgsvectorlayer.h>
#include <qgspointcloudlayer.h>
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
#include <QVector3D>
#include <cstring>
#include <limits>
#include <Qt3DCore/QEntity>
#include <Qt3DRender/QAttribute>
#include <Qt3DRender/QBuffer>
#include <Qt3DRender/QGeometry>
#include <Qt3DRender/QGeometryRenderer>
#include <Qt3DExtras/QPhongMaterial>

// ==================== 构造与析构 ====================
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
  }
  mTempLayer->commitChanges();
  mTempLayer->triggerRepaint();
}

QList<BuildingRoof::RoofPoint> ThreeDViewTool::roofPointsForFeature( const QgsFeature &sourceFeature, bool *fromSavedLayer )
{
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
    while ( it.nextFeature( feature ) )
    {
      if ( !feature.hasGeometry() )
        continue;

      const QgsPoint *point = qgsgeometry_cast<const QgsPoint *>( feature.geometry().constGet() );
      if ( !point )
        continue;

      QgsPoint roofPoint = *point;
      if ( useRelativePosition && relXIndex >= 0 && relYIndex >= 0 && bounds.width() > 1e-12 && bounds.height() > 1e-12 )
      {
        bool okX = false;
        bool okY = false;
        const double relX = feature.attribute( relXIndex ).toDouble( &okX );
        const double relY = feature.attribute( relYIndex ).toDouble( &okY );
        if ( okX && okY )
          roofPoint = QgsPoint( bounds.xMinimum() + relX * bounds.width(), bounds.yMinimum() + relY * bounds.height(), roofPoint.z() );
      }

      bool ok = false;
      const double z = feature.attribute( QStringLiteral( "z" ) ).toDouble( &ok );
      if ( ok )
        roofPoint.setZ( z );

      points.append( BuildingRoof::RoofPoint{ roofPoint, feature.attribute( QStringLiteral( "type" ) ).toString() } );
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
  auto isGroundPoint = []( const QString &type ) {
    return type.contains( QStringLiteral( "地面" ) )
           || type.contains( QStringLiteral( "鍦伴潰" ) )
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
    const BuildingRoof::MeshResult roof = BuildingRoof::buildSingleSlopePrismMesh( feature.geometry(), height, points );
    if ( roof.success )
    {
      return MeshData{ roof.mesh.vertices, roof.mesh.indices };
    }

    const BuildingRoof::MeshResult gabledRoof = BuildingRoof::buildGabledRoofPrismMesh( feature.geometry(), height, points );
    if ( gabledRoof.success )
    {
      return MeshData{ gabledRoof.mesh.vertices, gabledRoof.mesh.indices };
    }

    const BuildingRoof::MeshResult hippedRoof = BuildingRoof::buildHippedRoofPrismMesh( feature.geometry(), height, points );
    if ( hippedRoof.success )
    {
      return MeshData{ hippedRoof.mesh.vertices, hippedRoof.mesh.indices };
    }

    const BuildingRoof::MeshResult multiRidgeRoof = BuildingRoof::buildMultiRidgePrismMesh( feature.geometry(), height, points );
    if ( multiRidgeRoof.success )
    {
      return MeshData{ multiRidgeRoof.mesh.vertices, multiRidgeRoof.mesh.indices };
    }
  }

  return BuildMesh::build( feature.geometry(), height );
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
    QgsPolygon3DSymbol *symbol = new QgsPolygon3DSymbol();
    symbol->setAltitudeClamping( Qgis::AltitudeClamping::Absolute );
    QgsVectorLayer3DRenderer *renderer = new QgsVectorLayer3DRenderer();
    renderer->setLayer( mTempLayer );
    renderer->setSymbol( symbol );
    mTempLayer->setRenderer3D( renderer );

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
  double zMax = layer->statistics().maximum( zAttribute );
  if ( !std::isfinite( zMax ) || zMax <= -50.0 )
    zMax = 50.0;

  QgsColorRampShader shader( -50.0, zMax, nullptr, Qgis::ShaderInterpolationMethod::Linear, Qgis::ShaderClassificationMethod::Continuous );
  const double range = zMax + 50.0;
  QList<QgsColorRampShader::ColorRampItem> items;
  items << QgsColorRampShader::ColorRampItem( -50.0, QColor( 36, 57, 175 ), QStringLiteral( "-50" ) )
        << QgsColorRampShader::ColorRampItem( -50.0 + range * 0.25, QColor( 0, 188, 212 ) )
        << QgsColorRampShader::ColorRampItem( -50.0 + range * 0.50, QColor( 76, 175, 80 ) )
        << QgsColorRampShader::ColorRampItem( -50.0 + range * 0.75, QColor( 255, 235, 59 ) )
        << QgsColorRampShader::ColorRampItem( zMax, QColor( 211, 47, 47 ), QString::number( zMax, 'f', 2 ) );
  shader.setColorRampType( Qgis::ShaderInterpolationMethod::Linear );
  shader.setClassificationMode( Qgis::ShaderClassificationMethod::Continuous );
  shader.setColorRampItemList( items );

  QgsColorRampPointCloud3DSymbol *symbol = new QgsColorRampPointCloud3DSymbol();
  symbol->setAttribute( zAttribute );
  symbol->setColorRampShader( shader );
  symbol->setColorRampShaderMinMax( -50.0, zMax );
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
  refresh3DCanvases();
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
