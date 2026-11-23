#include "roofedit.h"

#include "buildingeditpreviewbus.h"
#include "roofmodelpipeline.h"
#include "roofaccuracy.h"
#include "roofinteraction.h"

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
#include <QKeySequence>
#include <QMessageBox>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTimer>
#include <QWheelEvent>
#include <QKeyEvent>
#include <limits>

RoofEditTool::RoofEditTool( QgsMapCanvas *canvas, QgisInterface *iface )
  : QgsMapTool( canvas )
  , mIface( iface )
  , mCanvas( canvas )
{
  setCursor( Qt::CrossCursor );
  mRoofPreviewTimer = new QTimer( this );
  mRoofPreviewTimer->setSingleShot( true );
  mRoofPreviewTimer->setTimerType( Qt::PreciseTimer );
  mRoofPreviewTimer->setInterval( 180 );
  connect( mRoofPreviewTimer, &QTimer::timeout, this, &RoofEditTool::previewRoofSelection );
}

RoofEditTool::~RoofEditTool()
{
  delete mAccuracy;
  mAccuracy = nullptr;
  mInteraction.reset();
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
  if ( mCanvas )
  {
    mCanvas->installEventFilter( this );
    mCanvas->viewport()->installEventFilter( this );
  }
  hideLegacyPointLayers();
  if ( mWidget && mInteraction && mInteraction->hasBuilding() )
  {
    refreshPointCloudCombo();
    mWidget->show();
    syncAccuracyContext();
    scheduleRoofPreview();
  }
}

void RoofEditTool::deactivate()
{
  if ( mCanvas )
  {
    mCanvas->removeEventFilter( this );
    mCanvas->viewport()->removeEventFilter( this );
  }
  if ( mAccuracy ) mAccuracy->clear();
  if ( mInteraction ) mInteraction->cancelDrag();
  setTriangleMeshModeEnabled( false );
  clearRoofPreview();
  if ( mWidget )
    mWidget->hide();
  QgsMapTool::deactivate();
}

bool RoofEditTool::eventFilter( QObject *object, QEvent *event )
{
  const bool onCanvas = mCanvas && ( object == mCanvas || object == mCanvas->viewport() );
  if ( mInteraction && event )
  {
    if ( event->type() == QEvent::ShortcutOverride || event->type() == QEvent::KeyPress )
    {
      auto *key = static_cast<QKeyEvent *>( event );
      const bool undo = mInteraction->hasBuilding() && key->matches( QKeySequence::Undo );
      const bool deselectRegion = key->key() == Qt::Key_Escape && mInteraction->hasSelectedRegion();
      if ( event->type() == QEvent::ShortcutOverride && ( undo || deselectRegion ) )
      {
        event->accept();
        return true;
      }
      if ( event->type() == QEvent::KeyPress && ( onCanvas || undo || deselectRegion ) && mInteraction->keyPress( key ) )
        return true;
    }
    if ( onCanvas && event->type() == QEvent::FocusOut )
      mInteraction->cancelDrag();
    if ( mCanvas && object == mCanvas->viewport() )
      mInteraction->canvasEvent( event );
    if ( object == mWidget && event->type() == QEvent::Close )
      endBuildingEdit();
  }
  return QgsMapTool::eventFilter( object, event );
}

void RoofEditTool::endBuildingEdit()
{
  if ( mAccuracy ) mAccuracy->clear();
  if ( mInteraction ) mInteraction->cancelDrag();
  setTriangleMeshModeEnabled( false );
  clearRoofPreview();
  if ( mInteraction ) mInteraction->reset();
  mCurrentBuilding = QgsFeature();
}

void RoofEditTool::setupUi()
{
  if ( mWidget )
    return;

  mWidget = new QWidget();
  mUI.setupUi( mWidget );
  mWidget->installEventFilter( this );
  for ( QWidget *control : mWidget->findChildren<QWidget *>() )
    control->installEventFilter( this );
  mInteraction = std::make_unique<RoofInteraction>( mCanvas, mIface, mUI, mWidget,
                 [this]() { scheduleRoofPreview(); }, [this]() { scheduleDragPreview(); } );

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
      updateTriangleMeshModeForCurrentBuilding();
    else
      emit BuildingEditPreviewBus::instance()->buildingTriangleMeshModeChanged( mActiveLayer, FID_NULL, false );
    previewRoofSelection();
  } );

  mAccuracy = new RoofAccuracy::Controller( mWidget, mUI, mIface, this );
}

QgsVectorLayer *RoofEditTool::buildingLayerForSearch() const
{
  QgsVectorLayer *nextLayer = qobject_cast<QgsVectorLayer *>( mIface ? mIface->activeLayer() : nullptr );
  if ( isCandidateBuildingLayer( nextLayer ) )
    return nextLayer;
  if ( isCandidateBuildingLayer( mActiveLayer ) )
    return mActiveLayer;

  QgsVectorLayer *fallback = nullptr;
  const auto layers = QgsProject::instance()->layers<QgsVectorLayer *>();
  for ( QgsVectorLayer *layer : layers )
  {
    if ( !isCandidateBuildingLayer( layer ) )
      continue;
    if ( layer->isEditable() )
      return layer;
    if ( !fallback )
      fallback = layer;
  }
  return fallback;
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
  if ( !event )
    return;
  if ( mInteraction && mInteraction->press( event ) )
    return;
  if ( event->button() != Qt::LeftButton )
    return;
  if ( !selectBuildingAt( event->mapPoint() ) )
    return;
  setupUi();
  refreshPointCloudCombo();
  loadRoofSelection();
  mWidget->show();
  syncAccuracyContext();
  mWidget->raise();
  mWidget->activateWindow();
}

void RoofEditTool::canvasMoveEvent( QgsMapMouseEvent *event )
{
  if ( mInteraction ) mInteraction->move( event );
}

void RoofEditTool::canvasDoubleClickEvent( QgsMapMouseEvent *event )
{
  if ( mInteraction && mInteraction->doubleClick( event ) )
    return;
  QgsMapTool::canvasDoubleClickEvent( event );
}

void RoofEditTool::canvasReleaseEvent( QgsMapMouseEvent *event )
{
  if ( mInteraction && mInteraction->release( event ) )
    return;
  QgsMapTool::canvasReleaseEvent( event );
}

void RoofEditTool::wheelEvent( QWheelEvent *event )
{
  if ( mInteraction && mInteraction->wheel( event ) )
    event->accept();
  else
    QgsMapTool::wheelEvent( event );
}

void RoofEditTool::keyPressEvent( QKeyEvent *event )
{
  if ( mInteraction && mInteraction->keyPress( event ) )
    event->ignore(); // QgsMapCanvas treats ignored map-tool keys as consumed.
  else
    QgsMapTool::keyPressEvent( event );
}

bool RoofEditTool::selectBuildingAt( const QgsPointXY &point )
{
  QgsVectorLayer *layer = buildingLayerForSearch();
  if ( !layer )
    return false;
  const double tolerance = mCanvas ? mCanvas->mapUnitsPerPixel() * 8.0 : 1.0;
  const QgsRectangle mapBounds( point.x() - tolerance, point.y() - tolerance, point.x() + tolerance, point.y() + tolerance );
  const QgsRectangle bounds = toLayerCoordinates( layer, mapBounds );
  QgsFeatureIterator it = layer->getFeatures( QgsFeatureRequest( bounds ) );
  QgsFeature feature, bestFeature;
  double bestDistance = std::numeric_limits<double>::max();
  const QgsGeometry clickGeometry = QgsGeometry::fromPointXY( toLayerCoordinates( layer, point ) );
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

  if ( layer == mActiveLayer && mCurrentBuilding.isValid() && mCurrentBuilding.id() == bestFeature.id()
       && mInteraction && mInteraction->hasBuilding() )
    return false;

  if ( mAccuracy ) mAccuracy->clear();
  setTriangleMeshModeEnabled( false );
  clearRoofPreview();
  mActiveLayer = layer;
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
  mInteraction->load( mActiveLayer, mCurrentBuilding, saved );
}

RoofModelSettings::Selection RoofEditTool::currentSelection() const
{
  if ( !mWidget )
    return {};
  RoofModelSettings::Selection selection{ RoofModelSettings::typeFromId( mUI.roofTypeComboBox->currentData().toString() ),
                                          mUI.pointCloudComboBox->currentData().toString() };
  if ( mInteraction ) mInteraction->addToSelection( selection );
  return selection;
}

void RoofEditTool::scheduleRoofPreview()
{
  if ( mInteraction && mInteraction->hasRegions() )
  {
    scheduleDragPreview();
    return;
  }
  if ( mWidget && mWidget->isVisible() && mActiveLayer && mCurrentBuilding.isValid() )
    mRoofPreviewTimer->start( 180 );
}

void RoofEditTool::scheduleDragPreview()
{
  // Render the latest position once per frame, even while input events keep arriving.
  constexpr int frameInterval = 16;
  if ( mWidget && mWidget->isVisible() && mActiveLayer && mCurrentBuilding.isValid()
       && ( !mRoofPreviewTimer->isActive() || mRoofPreviewTimer->remainingTime() > frameInterval ) )
    mRoofPreviewTimer->start( frameInterval );
}

void RoofEditTool::previewRoofSelection()
{
  mRoofPreviewTimer->stop();
  if ( !mWidget || !mWidget->isVisible() || !mActiveLayer || !mCurrentBuilding.isValid() )
    return;
  const RoofModelSettings::Selection selection = currentSelection();
  const bool removedRegions = mInteraction && mInteraction->hasRemovedSavedRegions();
  if ( !removedRegions && ( !selection.hasRoof() || ( selection.regions.isEmpty() && selection.pointCloudLayerId.isEmpty() ) ) )
  {
    syncAccuracyContext();
    clearRoofPreview();
    return;
  }
  const bool changed = !( RoofModelSettings::read( mActiveLayer, mCurrentBuilding.id() ) == selection );
  if ( changed )
    RoofModelSettings::setPreview( mActiveLayer, mCurrentBuilding.id(), selection );
  if ( !selection.regions.isEmpty() )
  {
    auto visibleSelection = selection;
    const bool dragging = mInteraction && mInteraction->addDragToSelection( visibleSelection );
    const auto result = RoofModelPipeline::build( mCurrentBuilding.geometry(), visibleSelection );
    if ( result.success )
    {
      if ( mAccuracy )
        mAccuracy->setContext( mActiveLayer, mCurrentBuilding.id(), visibleSelection, &result.mesh );
      mLiveRoofPreview = true;
      emit BuildingEditPreviewBus::instance()->roofPreviewChanged( mActiveLayer, mCurrentBuilding.id(), result.mesh, dragging );
      return;
    }
    if ( dragging )
      return; // Leave the last valid frame visible while a vertex crosses an invalid position.
  }
  syncAccuracyContext();
  if ( changed || mLiveRoofPreview )
    notifyRoofModelChanged();
}

void RoofEditTool::clearRoofPreview()
{
  mRoofPreviewTimer->stop();
  if ( RoofModelSettings::clearPreview( mActiveLayer, mCurrentBuilding.id() ) || mLiveRoofPreview )
    notifyRoofModelChanged();
}

void RoofEditTool::notifyRoofModelChanged()
{
  mLiveRoofPreview = false;
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
  if ( mInteraction ) mInteraction->clearRegions();
  mRoofPreviewTimer->stop();
  RoofModelSettings::save( mActiveLayer, mCurrentBuilding.id(), {} );
  RoofModelSettings::clearPreview( mActiveLayer, mCurrentBuilding.id() );
  {
    const QSignalBlocker blocker( mUI.roofTypeComboBox );
    mUI.roofTypeComboBox->setCurrentIndex( -1 );
  }
  syncAccuracyContext();
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
  if ( mInteraction && mInteraction->hasUnfinishedEdit() )
  {
    QMessageBox::warning( mWidget, tr( "提示" ), tr( "区域草图按 Enter 完成，屋脊草图用鼠标右键确认。拖动时请先松开鼠标，或按 Esc 取消。" ) );
    return;
  }
  const RoofModelSettings::Selection selection = currentSelection();
  const bool removedRoof = !selection.hasRoof() && mInteraction && mInteraction->hasRemovedSavedRegions();
  if ( !selection.hasRoof() && !removedRoof )
  {
    QMessageBox::warning( mWidget, tr( "提示" ), tr( "请先绘制区域，或选择自动建模的屋顶类型。" ) );
    return;
  }
  QgsPointCloudLayer *cloud = qobject_cast<QgsPointCloudLayer *>(
    QgsProject::instance()->mapLayer( selection.pointCloudLayerId ) );
  if ( !removedRoof && selection.regions.isEmpty() && ( !cloud || !cloud->dataProvider() ) )
  {
    QMessageBox::warning( mWidget, tr( "提示" ), tr( "请先选择点云图层。" ) );
    return;
  }

  mCurrentBuilding = mActiveLayer->getFeature( mCurrentBuilding.id() );
  if ( !mCurrentBuilding.isValid() )
    return;
  if ( !removedRoof )
  {
    const BuildingRoof::MeshResult result = RoofModelPipeline::build( mCurrentBuilding.geometry(), selection );
    if ( !result.success )
    {
      clearRoofPreview();
      QMessageBox::warning( mWidget, tr( "错误" ), result.error );
      return;
    }
  }

  RoofModelSettings::save( mActiveLayer, mCurrentBuilding.id(), selection );
  RoofModelSettings::clearPreview( mActiveLayer, mCurrentBuilding.id() );
  setTriangleMeshModeEnabled( false );
  notifyRoofModelChanged();
  QMessageBox::information( mWidget, tr( "提示" ), removedRoof ? tr( "已保存区域删除。" ) : tr( "屋顶模型已保存。" ) );
  mActiveLayer->removeSelection();
  mWidget->hide();
  endBuildingEdit();
}

void RoofEditTool::syncAccuracyContext()
{
  if ( mAccuracy )
    mAccuracy->setContext( mActiveLayer, mCurrentBuilding.isValid() ? mCurrentBuilding.id() : FID_NULL, currentSelection() );
}

