#include "roofedit.h"

#include "buildingeditpreviewbus.h"
#include "roofmodelpipeline.h"

#include <qgisinterface.h>
#include <qgs3dmapcanvas.h>
#include <qgs3dmapsettings.h>
#include <qgsfeatureiterator.h>
#include <qgsgeometry.h>
#include <qgslayertree.h>
#include <qgslayertreelayer.h>
#include <qgsmapcanvas.h>
#include <qgsmapmouseevent.h>
#include <qgspointcloudlayer.h>
#include <qgsproject.h>
#include <qgsvectorlayer.h>

#include <QCheckBox>
#include <QComboBox>
#include <QEvent>
#include <QMessageBox>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTimer>
#include <limits>

RoofEditTool::RoofEditTool( QgsMapCanvas *canvas, QgisInterface *iface )
  : QgsMapTool( canvas )
  , mIface( iface )
  , mCanvas( canvas )
{
  setCursor( Qt::CrossCursor );
  mRoofPreviewTimer = new QTimer( this );
  mRoofPreviewTimer->setSingleShot( true );
  mRoofPreviewTimer->setInterval( 180 );
  connect( mRoofPreviewTimer, &QTimer::timeout, this, &RoofEditTool::previewRoofSelection );
}

RoofEditTool::~RoofEditTool()
{
  setTriangleMeshModeEnabled( false );
  clearRoofPreview();
  if ( mWidget )
  {
    mWidget->removeEventFilter( this );
    mWidget->deleteLater();
  }
}

void RoofEditTool::activate()
{
  QgsMapTool::activate();
  refreshLayerRefs();
  hideLegacyPointLayers();
}

void RoofEditTool::deactivate()
{
  setTriangleMeshModeEnabled( false );
  clearRoofPreview();
  if ( mWidget )
    mWidget->hide();
  QgsMapTool::deactivate();
}

bool RoofEditTool::eventFilter( QObject *object, QEvent *event )
{
  if ( object == mWidget && event && ( event->type() == QEvent::Hide || event->type() == QEvent::Close ) )
  {
    setTriangleMeshModeEnabled( false );
    clearRoofPreview();
  }
  return QgsMapTool::eventFilter( object, event );
}

void RoofEditTool::setupUi()
{
  if ( mWidget )
    return;

  mWidget = new QWidget();
  mUI.setupUi( mWidget );
  mWidget->installEventFilter( this );

  using Type = RoofModelSettings::Type;
  mUI.roofTypeComboBox->addItem( tr( "双坡" ), RoofModelSettings::typeId( Type::Gabled ) );
  mUI.roofTypeComboBox->addItem( tr( "四面" ), RoofModelSettings::typeId( Type::Hipped ) );
  mUI.roofTypeComboBox->addItem( tr( "折线双坡" ), RoofModelSettings::typeId( Type::BrokenGabled ) );
  mUI.roofTypeComboBox->addItem( tr( "圆锥/棱锥" ), RoofModelSettings::typeId( Type::Apex ) );
  mUI.roofTypeComboBox->addItem( tr( "圆柱" ), RoofModelSettings::typeId( Type::Barrel ) );
  mUI.roofTypeComboBox->addItem( tr( "穹顶" ), RoofModelSettings::typeId( Type::Dome ) );
  mUI.roofTypeComboBox->addItem( tr( "棱台" ), RoofModelSettings::typeId( Type::Frustum ) );
  mUI.roofTypeComboBox->addItem( tr( "凹凸" ), RoofModelSettings::typeId( Type::Relief ) );
  mUI.roofTypeComboBox->setCurrentIndex( -1 );

  connect( mUI.roofTypeComboBox, qOverload<int>( &QComboBox::currentIndexChanged ),
           this, [this]( int ) { scheduleRoofPreview(); } );
  connect( mUI.pointCloudComboBox, qOverload<int>( &QComboBox::currentIndexChanged ),
           this, [this]( int ) { scheduleRoofPreview(); } );
  connect( mUI.deleteRoofButton, &QPushButton::clicked, this, &RoofEditTool::deleteRoofModel );
  connect( mUI.saveRoofButton, &QPushButton::clicked, this, &RoofEditTool::saveRoofModel );
  connect( mUI.triangleMeshCheckBox, &QCheckBox::toggled, this, [this]( bool checked ) {
    if ( checked )
    {
      previewRoofSelection();
      updateTriangleMeshModeForCurrentBuilding();
    }
    else
      emit BuildingEditPreviewBus::instance()->buildingTriangleMeshModeChanged( mActiveLayer, FID_NULL, false );
  } );

  refreshPointCloudCombo();
}

void RoofEditTool::refreshLayerRefs()
{
  QgsVectorLayer *nextLayer = qobject_cast<QgsVectorLayer *>( mIface ? mIface->activeLayer() : nullptr );
  if ( !isCandidateBuildingLayer( nextLayer ) )
    nextLayer = isCandidateBuildingLayer( mActiveLayer ) ? mActiveLayer.data() : nullptr;
  if ( !nextLayer )
  {
    const auto layers = QgsProject::instance()->layers<QgsVectorLayer *>();
    for ( QgsVectorLayer *layer : layers )
    {
      if ( isCandidateBuildingLayer( layer ) && layer->isEditable() )
      {
        nextLayer = layer;
        break;
      }
    }
    if ( !nextLayer )
    {
      for ( QgsVectorLayer *layer : layers )
      {
        if ( isCandidateBuildingLayer( layer ) )
        {
          nextLayer = layer;
          break;
        }
      }
    }
  }

  if ( mActiveLayer != nextLayer )
  {
    setTriangleMeshModeEnabled( false );
    clearRoofPreview();
    mCurrentBuilding = QgsFeature();
    mActiveLayer = nextLayer;
  }
  if ( mWidget )
    refreshPointCloudCombo();
}

bool RoofEditTool::isCandidateBuildingLayer( QgsVectorLayer *layer ) const
{
  if ( !layer || layer->geometryType() != Qgis::GeometryType::Polygon )
    return false;
  const QString name = layer->name();
  if ( RoofModelSettings::isLegacyPointLayer( name )
       || name == QLatin1String( "Roof_Edit_Roof_Preview" ) || name == QLatin1String( "Roof_Edit_Roofs" )
       || name.endsWith( QLatin1String( "_3Dbuilding" ) ) )
    return false;
  return layer->providerType() != QLatin1String( "memory" )
         || layer->fields().indexOf( QStringLiteral( "original_fid" ) ) < 0;
}

void RoofEditTool::refreshPointCloudCombo()
{
  if ( !mWidget )
    return;
  const QString currentId = mUI.pointCloudComboBox->currentData().toString();
  const QSignalBlocker blocker( mUI.pointCloudComboBox );
  mUI.pointCloudComboBox->clear();
  mUI.pointCloudComboBox->addItem( tr( "请选择点云图层..." ), QString() );
  for ( QgsMapLayer *layer : QgsProject::instance()->mapLayers().values() )
  {
    if ( qobject_cast<QgsPointCloudLayer *>( layer ) )
      mUI.pointCloudComboBox->addItem( layer->name(), layer->id() );
  }
  int index = mUI.pointCloudComboBox->findData( currentId );
  if ( index <= 0 && mUI.pointCloudComboBox->count() == 2 )
    index = 1;
  mUI.pointCloudComboBox->setCurrentIndex( index < 0 ? 0 : index );
}

void RoofEditTool::hideLegacyPointLayers()
{
  for ( QgsMapLayer *layer : QgsProject::instance()->mapLayers().values() )
  {
    if ( !RoofModelSettings::isLegacyPointLayer( layer->name() ) )
      continue;
    if ( QgsLayerTreeLayer *node = QgsProject::instance()->layerTreeRoot()->findLayer( layer->id() ) )
      node->setItemVisibilityChecked( false );
    if ( !mIface )
      continue;
    for ( Qgs3DMapCanvas *canvas : mIface->mapCanvases3D() )
    {
      if ( !canvas || !canvas->mapSettings() )
        continue;
      QList<QgsMapLayer *> layers = canvas->mapSettings()->layers();
      if ( layers.removeAll( layer ) )
        canvas->mapSettings()->setLayers( layers );
    }
  }
}

void RoofEditTool::canvasPressEvent( QgsMapMouseEvent *event )
{
  if ( !event || event->button() != Qt::LeftButton )
    return;
  refreshLayerRefs();
  if ( !selectBuildingAt( event->mapPoint() ) )
    return;
  setupUi();
  loadRoofSelection();
  mWidget->show();
  mWidget->raise();
  mWidget->activateWindow();
}

bool RoofEditTool::selectBuildingAt( const QgsPointXY &point )
{
  if ( !mActiveLayer )
    return false;
  const double tolerance = mCanvas ? mCanvas->mapUnitsPerPixel() * 8.0 : 1.0;
  const QgsRectangle bounds( point.x() - tolerance, point.y() - tolerance, point.x() + tolerance, point.y() + tolerance );
  QgsFeatureIterator it = mActiveLayer->getFeatures( QgsFeatureRequest( bounds ) );
  QgsFeature feature, bestFeature;
  double bestDistance = std::numeric_limits<double>::max();
  const QgsGeometry clickGeometry = QgsGeometry::fromPointXY( point );
  while ( it.nextFeature( feature ) )
  {
    if ( !feature.hasGeometry() )
      continue;
    const QgsGeometry geometry = feature.geometry();
    if ( geometry.contains( clickGeometry ) )
    {
      bestFeature = feature;
      break;
    }
    const double distance = geometry.distance( clickGeometry );
    if ( distance < bestDistance )
    {
      bestDistance = distance;
      bestFeature = feature;
    }
  }
  if ( !bestFeature.isValid() )
    return false;

  if ( mCurrentBuilding.isValid() && mCurrentBuilding.id() == bestFeature.id()
       && mWidget && mWidget->isVisible() )
    return false;

  setTriangleMeshModeEnabled( false );
  clearRoofPreview();
  mCurrentBuilding = bestFeature;
  mActiveLayer->selectByIds( QgsFeatureIds() << bestFeature.id() );
  return true;
}

void RoofEditTool::loadRoofSelection()
{
  const RoofModelSettings::Selection saved = RoofModelSettings::read( mActiveLayer, mCurrentBuilding.id(), false );
  const QSignalBlocker typeBlocker( mUI.roofTypeComboBox );
  const QSignalBlocker cloudBlocker( mUI.pointCloudComboBox );
  const int typeIndex = mUI.roofTypeComboBox->findData( RoofModelSettings::typeId( saved.type ) );
  mUI.roofTypeComboBox->setCurrentIndex( typeIndex );
  if ( !saved.pointCloudLayerId.isEmpty() )
  {
    const int cloudIndex = mUI.pointCloudComboBox->findData( saved.pointCloudLayerId );
    mUI.pointCloudComboBox->setCurrentIndex( cloudIndex < 0 ? 0 : cloudIndex );
  }
}

RoofModelSettings::Selection RoofEditTool::currentSelection() const
{
  if ( !mWidget )
    return {};
  return { RoofModelSettings::typeFromId( mUI.roofTypeComboBox->currentData().toString() ),
           mUI.pointCloudComboBox->currentData().toString() };
}

void RoofEditTool::scheduleRoofPreview()
{
  if ( mWidget && mWidget->isVisible() && mActiveLayer && mCurrentBuilding.isValid() )
    mRoofPreviewTimer->start();
}

void RoofEditTool::previewRoofSelection()
{
  mRoofPreviewTimer->stop();
  if ( !mWidget || !mWidget->isVisible() || !mActiveLayer || !mCurrentBuilding.isValid() )
    return;
  const RoofModelSettings::Selection selection = currentSelection();
  if ( selection.type == RoofModelSettings::Type::None || selection.pointCloudLayerId.isEmpty() )
  {
    clearRoofPreview();
    return;
  }
  const RoofModelSettings::Selection previous = RoofModelSettings::read( mActiveLayer, mCurrentBuilding.id() );
  if ( previous.type == selection.type && previous.pointCloudLayerId == selection.pointCloudLayerId )
    return;
  RoofModelSettings::setPreview( mActiveLayer, mCurrentBuilding.id(), selection );
  notifyRoofModelChanged();
}

void RoofEditTool::clearRoofPreview()
{
  mRoofPreviewTimer->stop();
  if ( RoofModelSettings::clearPreview( mActiveLayer, mCurrentBuilding.id() ) )
    notifyRoofModelChanged();
}

void RoofEditTool::notifyRoofModelChanged()
{
  if ( mActiveLayer && mCurrentBuilding.isValid() )
    emit BuildingEditPreviewBus::instance()->roofModelChanged( mActiveLayer, mCurrentBuilding.id() );
}

void RoofEditTool::setTriangleMeshModeEnabled( bool enabled )
{
  if ( mWidget )
  {
    const QSignalBlocker blocker( mUI.triangleMeshCheckBox );
    mUI.triangleMeshCheckBox->setChecked( enabled );
  }
  emit BuildingEditPreviewBus::instance()->buildingTriangleMeshModeChanged(
    mActiveLayer, enabled && mCurrentBuilding.isValid() ? mCurrentBuilding.id() : FID_NULL,
    enabled && mCurrentBuilding.isValid() );
}

void RoofEditTool::updateTriangleMeshModeForCurrentBuilding()
{
  if ( mWidget && mUI.triangleMeshCheckBox->isChecked() )
    setTriangleMeshModeEnabled( true );
}

void RoofEditTool::deleteRoofModel()
{
  mRoofPreviewTimer->stop();
  if ( !mActiveLayer || !mCurrentBuilding.isValid() )
  {
    QMessageBox::warning( mWidget, tr( "提示" ), tr( "请先选择建筑物。" ) );
    return;
  }
  setTriangleMeshModeEnabled( false );
  RoofModelSettings::save( mActiveLayer, mCurrentBuilding.id(), {} );
  RoofModelSettings::clearPreview( mActiveLayer, mCurrentBuilding.id() );
  {
    const QSignalBlocker blocker( mUI.roofTypeComboBox );
    mUI.roofTypeComboBox->setCurrentIndex( -1 );
  }
  notifyRoofModelChanged();
  QMessageBox::information( mWidget, tr( "提示" ), tr( "已删除当前建筑物的屋顶模型。" ) );
}

void RoofEditTool::saveRoofModel()
{
  mRoofPreviewTimer->stop();
  if ( !mActiveLayer || !mCurrentBuilding.isValid() )
  {
    QMessageBox::warning( mWidget, tr( "提示" ), tr( "请先选择建筑物。" ) );
    return;
  }
  const RoofModelSettings::Selection selection = currentSelection();
  if ( selection.type == RoofModelSettings::Type::None )
  {
    QMessageBox::warning( mWidget, tr( "提示" ), tr( "请先选择屋顶类型。" ) );
    return;
  }
  QgsPointCloudLayer *cloud = qobject_cast<QgsPointCloudLayer *>(
    QgsProject::instance()->mapLayer( selection.pointCloudLayerId ) );
  if ( !cloud || !cloud->dataProvider() )
  {
    QMessageBox::warning( mWidget, tr( "提示" ), tr( "请先选择点云图层。" ) );
    return;
  }

  mCurrentBuilding = mActiveLayer->getFeature( mCurrentBuilding.id() );
  if ( !mCurrentBuilding.isValid() )
    return;
  const BuildingRoof::MeshResult result = RoofModelPipeline::build( mCurrentBuilding.geometry(), selection );
  if ( !result.success )
  {
    clearRoofPreview();
    QMessageBox::warning( mWidget, tr( "错误" ), result.error );
    return;
  }

  RoofModelSettings::save( mActiveLayer, mCurrentBuilding.id(), selection );
  RoofModelSettings::clearPreview( mActiveLayer, mCurrentBuilding.id() );
  setTriangleMeshModeEnabled( false );
  notifyRoofModelChanged();
  QMessageBox::information( mWidget, tr( "提示" ), tr( "屋顶模型已保存。" ) );
  mActiveLayer->removeSelection();
  mWidget->hide();
  mCurrentBuilding = QgsFeature();
}

