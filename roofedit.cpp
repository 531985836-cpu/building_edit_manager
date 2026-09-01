#include "roofedit.h"

#include "buildingeditpreviewbus.h"

#include <qgisinterface.h>
#include <qgs3dmapcanvas.h>
#include <qgs3dmapsettings.h>
#include <qgs3dmapscene.h>
#include <qgsfeatureiterator.h>
#include <qgsfield.h>
#include <qgsgeometry.h>
#include <qgsmapcanvas.h>
#include <qgsmapmouseevent.h>
#include <qgspoint.h>
#include <qgspoint3dsymbol.h>
#include <qgspointcloudattribute.h>
#include <qgspointcloudblock.h>
#include <qgspointcloudindex.h>
#include <qgspointcloudlayer.h>
#include <qgspointcloudrequest.h>
#include <qgsproject.h>
#include <qgsmarkersymbol.h>
#include <qgscategorizedsymbolrenderer.h>
#include <qgsfillsymbol.h>
#include <qgsnullsymbolrenderer.h>
#include <qgsphongmaterialsettings.h>
#include <qgspolygon3dsymbol.h>
#include <qgsrulebased3drenderer.h>
#include <qgsrubberband.h>
#include <qgssinglesymbolrenderer.h>
#include <qgsvectorlayer.h>
#include <qgsvectorlayer3drenderer.h>
#include <qgslayertree.h>
#include <qgslayertreelayer.h>

#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DExtras/QSphereMesh>
#include <QBrush>
#include <QComboBox>
#include <QCryptographicHash>
#include <QEvent>
#include <QHeaderView>
#include <QKeyEvent>
#include <QMessageBox>
#include <QRadioButton>
#include <QShortcut>
#include <QTableWidgetItem>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>

namespace
{
  const QString ROOF_POINT_BOUNDARY = QStringLiteral( "边界点" );
  const QString ROOF_POINT_RIDGE = QStringLiteral( "屋脊点" );
  const QString ROOF_POINT_GROUND = QStringLiteral( "地面点" );
  const QString ROOF_POINT_VERTEX = QStringLiteral( "顶点" );
  const QString ROOF_POINT_SURFACE = QStringLiteral( "曲面点" );

  struct EditableRidgePoint
  {
    QgsFeatureId fid = FID_NULL;
    QgsPoint point;
    double s = 0.0;
  };

  bool isRoofRidgePointType( const QString &type )
  {
    return type.contains( ROOF_POINT_RIDGE )
           || type.contains( QStringLiteral( "灞嬭剨" ) )
           || type.contains( QStringLiteral( "ridge" ), Qt::CaseInsensitive );
  }

  bool isRoofGroundPointType( const QString &type )
  {
    return type.contains( ROOF_POINT_GROUND )
           || type.contains( QStringLiteral( "鍦伴潰" ) )
           || type.contains( QStringLiteral( "ground" ), Qt::CaseInsensitive );
  }

  bool isRoofVertexPointType( const QString &type )
  {
    return type.contains( ROOF_POINT_VERTEX )
           || type.contains( QStringLiteral( "vertex" ), Qt::CaseInsensitive );
  }

  bool isRoofSurfacePointType( const QString &type )
  {
    return type.contains( ROOF_POINT_SURFACE )
           || type.contains( QStringLiteral( "surface" ), Qt::CaseInsensitive );
  }

  bool isRoofBoundaryPointType( const QString &type )
  {
    return !isRoofRidgePointType( type )
           && !isRoofGroundPointType( type )
           && !isRoofVertexPointType( type )
           && !isRoofSurfacePointType( type );
  }

  double percentileValue( const QVector<double> &sortedValues, double percentile )
  {
    if ( sortedValues.isEmpty() )
      return 0.0;

    const double clamped = std::max( 0.0, std::min( 1.0, percentile ) );
    const int index = std::max( 0, std::min( sortedValues.size() - 1, static_cast<int>( std::round( clamped * ( sortedValues.size() - 1 ) ) ) ) );
    return sortedValues.at( index );
  }

  double heightPercentileForPointType( const QString &pointType )
  {
    if ( isRoofVertexPointType( pointType ) )
      return 0.98;
    if ( isRoofRidgePointType( pointType ) )
      return 0.95;
    if ( isRoofSurfacePointType( pointType ) )
      return 0.60;
    return 0.80;
  }

  QVector<QgsPointXY> featureExteriorRing( const QgsFeature &feature )
  {
    QVector<QgsPointXY> ring;
    if ( !feature.hasGeometry() )
      return ring;

    QgsPolygonXY polygon = feature.geometry().asPolygon();
    if ( polygon.isEmpty() )
    {
      const QgsMultiPolygonXY multiPolygon = feature.geometry().asMultiPolygon();
      if ( !multiPolygon.isEmpty() )
        polygon = multiPolygon.first();
    }

    if ( polygon.isEmpty() )
      return ring;

    for ( const QgsPointXY &point : polygon.first() )
    {
      if ( ring.isEmpty() || point != ring.last() )
        ring.append( point );
    }
    if ( ring.size() > 1 && ring.first() == ring.last() )
      ring.removeLast();
    return ring;
  }

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

  double pointSegmentDistance2ForRoofEdit( const QgsPointXY &point, const QgsPointXY &a, const QgsPointXY &b )
  {
    const double dx = b.x() - a.x();
    const double dy = b.y() - a.y();
    const double len2 = dx * dx + dy * dy;
    if ( len2 <= 1e-12 )
      return std::pow( point.x() - a.x(), 2.0 ) + std::pow( point.y() - a.y(), 2.0 );

    const double t = std::max( 0.0, std::min( 1.0, ( ( point.x() - a.x() ) * dx + ( point.y() - a.y() ) * dy ) / len2 ) );
    const double px = a.x() + t * dx;
    const double py = a.y() + t * dy;
    return std::pow( point.x() - px, 2.0 ) + std::pow( point.y() - py, 2.0 );
  }

  bool nearestRoofEditEdgeDirection( const QVector<QgsPointXY> &ring, const QgsPoint &boundaryPoint, double &dirX, double &dirY )
  {
    if ( ring.size() < 2 )
      return false;

    const QgsPointXY query( boundaryPoint.x(), boundaryPoint.y() );
    double bestDistance = std::numeric_limits<double>::max();
    int bestIndex = -1;
    for ( int i = 0; i < ring.size(); ++i )
    {
      const double distance = pointSegmentDistance2ForRoofEdit( query, ring[i], ring[( i + 1 ) % ring.size()] );
      if ( distance < bestDistance )
      {
        bestDistance = distance;
        bestIndex = i;
      }
    }

    if ( bestIndex < 0 )
      return false;

    const QgsPointXY &a = ring[bestIndex];
    const QgsPointXY &b = ring[( bestIndex + 1 ) % ring.size()];
    dirX = b.x() - a.x();
    dirY = b.y() - a.y();
    const double length = std::hypot( dirX, dirY );
    if ( length <= 1e-12 )
      return false;

    dirX /= length;
    dirY /= length;
    return true;
  }

  double roofEditProfileDistance( const QgsPoint &point, double normalX, double normalY )
  {
    return point.x() * normalX + point.y() * normalY;
  }

  QgsMarkerSymbol *createRoofMarkerSymbol( const QString &shape, const QColor &color, double size )
  {
    return QgsMarkerSymbol::createSimple(
      { { QStringLiteral( "name" ), shape },
        { QStringLiteral( "color" ), QStringLiteral( "%1,%2,%3" ).arg( color.red() ).arg( color.green() ).arg( color.blue() ) },
        { QStringLiteral( "outline_color" ), QStringLiteral( "255,255,255" ) },
        { QStringLiteral( "outline_width" ), QStringLiteral( "0.35" ) },
        { QStringLiteral( "size" ), QString::number( size, 'f', 1 ) } } ).release();
  }

  QgsPoint3DSymbol *createRoofPoint3DSymbol( const QString &shape, const QColor &color )
  {
    QgsPoint3DSymbol *symbol = new QgsPoint3DSymbol();
    symbol->setAltitudeClamping( Qgis::AltitudeClamping::Absolute );
    symbol->setShape( Qgis::Point3DShape::Billboard );
    symbol->setBillboardSymbol( createRoofMarkerSymbol( shape, color, 4.8 ) );

    QgsPhongMaterialSettings *material = new QgsPhongMaterialSettings();
    material->setAmbient( color.darker( 120 ) );
    material->setDiffuse( color );
    symbol->setMaterialSettings( material );
    return symbol;
  }
}

RoofEditTool::RoofEditTool( QgsMapCanvas *canvas, QgisInterface *iface )
  : QgsMapTool( canvas )
  , mIface( iface )
  , mCanvas( canvas )
{
  setCursor( Qt::CrossCursor );
}

RoofEditTool::~RoofEditTool()
{
  clearPointPreviewDisplay();
  clearPositionPreview();
  if ( mPreviewRootEntity )
  {
    mPreviewRootEntity->setEnabled( false );
    mPreviewRootEntity->deleteLater();
    mPreviewRootEntity = nullptr;
  }
  if ( mWidget )
  {
    mWidget->removeEventFilter( this );
    mWidget->deleteLater();
    mWidget = nullptr;
  }
}

void RoofEditTool::activate()
{
  QgsMapTool::activate();
  refreshLayerRefs();
}

void RoofEditTool::deactivate()
{
  cancelPointEditPreview();
  clearPositionPreview();
  setTriangleMeshModeEnabled( false );
  if ( mWidget )
    mWidget->hide();
  QgsMapTool::deactivate();
}

bool RoofEditTool::eventFilter( QObject *obj, QEvent *event )
{
  if ( obj == mWidget && event && ( event->type() == QEvent::Hide || event->type() == QEvent::Close ) )
    setTriangleMeshModeEnabled( false );

  return QgsMapTool::eventFilter( obj, event );
}

void RoofEditTool::setupUi()
{
  if ( mWidget )
    return;

  mWidget = new QWidget();
  mUI.setupUi( mWidget );
  mWidget->installEventFilter( this );
  mWidget->setWindowTitle( tr( "屋顶编辑" ) );

  if ( mUI.pointTableWidget )
  {
    mUI.pointTableWidget->horizontalHeader()->setSectionResizeMode( QHeaderView::Stretch );
    connect( mUI.pointTableWidget, &QTableWidget::itemSelectionChanged, this, &RoofEditTool::onPointSelectionChanged );
  }

  connect( mUI.clearPointsButton, &QPushButton::clicked, this, &RoofEditTool::clearCurrentBuildingPoints );
  connect( mUI.previewRoofButton, &QPushButton::clicked, this, &RoofEditTool::deleteRoofModel );
  connect( mUI.saveRoofButton, &QPushButton::clicked, this, &RoofEditTool::saveRoofModel );
  if ( mUI.pointradio )
  {
    mUI.pointradio->setAutoExclusive( false );
    connect( mUI.pointradio, &QRadioButton::toggled, this, [this]( bool checked ) {
      if ( checked )
        updateTriangleMeshModeForCurrentBuilding();
      else
        emit BuildingEditPreviewBus::instance()->buildingTriangleMeshModeChanged( mActiveLayer, FID_NULL, false );
    } );
  }

  QShortcut *returnShortcut = new QShortcut( QKeySequence( Qt::Key_Return ), mWidget );
  returnShortcut->setContext( Qt::WidgetWithChildrenShortcut );
  connect( returnShortcut, &QShortcut::activated, this, [this] {
    if ( mHasPointEditPreview )
      confirmPointEditPreview();
  } );

  QShortcut *enterShortcut = new QShortcut( QKeySequence( Qt::Key_Enter ), mWidget );
  enterShortcut->setContext( Qt::WidgetWithChildrenShortcut );
  connect( enterShortcut, &QShortcut::activated, this, [this] {
    if ( mHasPointEditPreview )
      confirmPointEditPreview();
  } );

  refreshPointCloudCombo();
  updatePointEditTip();
}

void RoofEditTool::refreshLayerRefs()
{
  QgsMapLayer *currentLayer = mIface ? mIface->activeLayer() : nullptr;
  mActiveLayer = qobject_cast<QgsVectorLayer *>( currentLayer );

  if ( !isCandidateBuildingLayer( mActiveLayer ) )
  {
    mActiveLayer = nullptr;
    const auto vectorLayers = QgsProject::instance()->layers<QgsVectorLayer *>();
    for ( QgsVectorLayer *layer : vectorLayers )
    {
      if ( isCandidateBuildingLayer( layer ) && layer->isEditable() )
      {
        mActiveLayer = layer;
        break;
      }
    }
    if ( !mActiveLayer )
    {
      for ( QgsVectorLayer *layer : vectorLayers )
      {
        if ( isCandidateBuildingLayer( layer ) )
        {
          mActiveLayer = layer;
          break;
        }
      }
    }
  }

  if ( mWidget )
    refreshPointCloudCombo();
}

bool RoofEditTool::isCandidateBuildingLayer( QgsVectorLayer *layer ) const
{
  if ( !layer || layer->geometryType() != Qgis::GeometryType::Polygon )
    return false;

  if ( layer == mPointLayer )
    return false;

  const QString name = layer->name();
  if ( name == QLatin1String( "Roof_Edit_Points" ) || name == QLatin1String( "Roof_Saved_Points" ) || name == QLatin1String( "Roof_Edit_Roof_Preview" ) || name == QLatin1String( "Roof_Edit_Roofs" ) || name == QLatin1String( "Roof_Edit_3D_Preview_Point" ) || name.endsWith( QLatin1String( "_3Dbuilding" ) ) )
    return false;

  const QgsFields fields = layer->fields();
  if ( layer->providerType() == QLatin1String( "memory" ) && fields.indexOf( QStringLiteral( "original_fid" ) ) >= 0 )
    return false;

  return true;
}

void RoofEditTool::refreshPointCloudCombo()
{
  if ( !mWidget || !mUI.pointCloudComboBox )
    return;

  const QString currentId = mUI.pointCloudComboBox->currentData().toString();
  mUI.pointCloudComboBox->blockSignals( true );
  mUI.pointCloudComboBox->clear();
  mUI.pointCloudComboBox->addItem( tr( "请选择点云图层..." ), QVariant() );

  const auto layers = QgsProject::instance()->mapLayers().values();
  int selectedIndex = 0;
  for ( QgsMapLayer *layer : layers )
  {
    if ( layer && layer->type() == Qgis::LayerType::PointCloud )
    {
      mUI.pointCloudComboBox->addItem( layer->name(), layer->id() );
      if ( layer->id() == currentId )
        selectedIndex = mUI.pointCloudComboBox->count() - 1;
    }
  }

  mUI.pointCloudComboBox->setCurrentIndex( selectedIndex );
  mUI.pointCloudComboBox->blockSignals( false );
}

void RoofEditTool::canvasPressEvent( QgsMapMouseEvent *e )
{
  if ( !e )
    return;

  refreshLayerRefs();

  if ( e->button() == Qt::RightButton && isPositionEditEnabled() && mHasPointEditPreview )
  {
    cancelPointEditPreview();
    e->accept();
    return;
  }

  if ( e->button() != Qt::LeftButton )
    return;

  const QgsPointXY mapPoint = e->mapPoint();

  if ( mCurrentBuilding.isValid() && mWidget && mWidget->isVisible() )
  {
    if ( handlePositionEditClick( mapPoint ) )
      return;
    addRoofPointAt( mapPoint );
    return;
  }

  if ( selectBuildingAt( mapPoint ) )
  {
    setupUi();
    ensurePointLayer();
    ensurePointLayerIn3DView();
    refreshPointTable();
    mWidget->show();
    mWidget->raise();
    mWidget->activateWindow();
  }
}

void RoofEditTool::canvasMoveEvent( QgsMapMouseEvent *e )
{
  if ( !e )
    return;

  if ( mMovingPointFid != FID_NULL && isPositionEditEnabled() )
  {
    if ( !mMoveThrottleTimer.isValid() )
      mMoveThrottleTimer.start();
    else if ( mMoveThrottleTimer.elapsed() < 16 )
      return;

    mMoveThrottleTimer.restart();
    updatePositionPreview( e->mapPoint() );
  }
}

void RoofEditTool::canvasDoubleClickEvent( QgsMapMouseEvent *e )
{
  if ( !e || e->button() != Qt::LeftButton )
    return;

  if ( !mWidget || !mWidget->isVisible() || !mCurrentBuilding.isValid() )
    return;

  const QgsFeatureId fid = findRoofPointAt( e->mapPoint() );
  if ( fid != FID_NULL )
  {
    deleteRoofPoint( fid );
    e->accept();
  }
}

void RoofEditTool::wheelEvent( QWheelEvent *e )
{
  if ( adjustSelectedPointHeight( e ) )
    return;

  QgsMapTool::wheelEvent( e );
}

void RoofEditTool::keyPressEvent( QKeyEvent *e )
{
  if ( e && ( e->key() == Qt::Key_Return || e->key() == Qt::Key_Enter ) && mHasPointEditPreview )
  {
    confirmPointEditPreview();
    e->accept();
    return;
  }

  QgsMapTool::keyPressEvent( e );
}

bool RoofEditTool::selectBuildingAt( const QgsPointXY &mapPoint )
{
  if ( !mActiveLayer )
    return false;

  const double tolerance = mCanvas ? mCanvas->mapUnitsPerPixel() * 8.0 : 1.0;
  QgsRectangle searchRect( mapPoint.x() - tolerance, mapPoint.y() - tolerance, mapPoint.x() + tolerance, mapPoint.y() + tolerance );
  QgsFeatureIterator it = mActiveLayer->getFeatures( QgsFeatureRequest( searchRect ) );
  QgsFeature feature;
  QgsFeature bestFeature;
  double bestDistance = std::numeric_limits<double>::max();
  QgsGeometry clickGeom = QgsGeometry::fromPointXY( mapPoint );

  while ( it.nextFeature( feature ) )
  {
    if ( !feature.hasGeometry() )
      continue;

    const QgsGeometry geom = feature.geometry();
    if ( geom.contains( clickGeom ) )
    {
      bestFeature = feature;
      bestDistance = 0.0;
      break;
    }

    const double distance = geom.distance( clickGeom );
    if ( distance < bestDistance )
    {
      bestDistance = distance;
      bestFeature = feature;
    }
  }

  if ( !bestFeature.isValid() )
    return false;

  const QgsFeatureId previousBuildingFid = mCurrentBuilding.isValid() ? mCurrentBuilding.id() : FID_NULL;
  const bool switchingBuilding = previousBuildingFid != FID_NULL && previousBuildingFid != bestFeature.id();
  if ( switchingBuilding && hasSavedRoofPoints( previousBuildingFid ) )
  {
    clearDraftPointsForBuilding( previousBuildingFid, false );
    notifyRoofModelChanged( previousBuildingFid );
  }

  cancelPointEditPreview();
  mSelectedPointFid = FID_NULL;
  mCurrentBuilding = bestFeature;
  mActiveLayer->selectByIds( QgsFeatureIds() << bestFeature.id() );
  updateTriangleMeshModeForCurrentBuilding();
  return true;
}

QString RoofEditTool::currentPointType() const
{
  if ( !mUI.pointTypeComboBox )
    return ROOF_POINT_BOUNDARY;
  return mUI.pointTypeComboBox->currentText();
}

QColor RoofEditTool::colorForPointType( const QString &type ) const
{
  if ( type.contains( ROOF_POINT_RIDGE ) || type.contains( QStringLiteral( "屋脊" ) ) )
    return QColor( 220, 40, 40 );
  if ( type.contains( ROOF_POINT_VERTEX ) || type.contains( QStringLiteral( "顶点" ) ) )
    return QColor( 245, 205, 35 );
  if ( type.contains( ROOF_POINT_SURFACE ) || type.contains( QStringLiteral( "曲面" ) ) )
    return QColor( 170, 60, 210 );
  if ( type.contains( ROOF_POINT_GROUND ) || type.contains( QStringLiteral( "地面" ) ) )
    return QColor( 240, 190, 35 );
  return QColor( 20, 170, 80 );
}

void RoofEditTool::addRoofPointAt( const QgsPointXY &mapPoint )
{
  if ( !mCurrentBuilding.isValid() )
    return;

  ensurePointLayer();
  if ( !mPointLayer )
    return;

  const QString pcId = mUI.pointCloudComboBox ? mUI.pointCloudComboBox->currentData().toString() : QString();
  mPointCloudLayer = qobject_cast<QgsPointCloudLayer *>( QgsProject::instance()->mapLayer( pcId ) );
  const QString pointType = currentPointType();
  const double z = estimateHeightFromPointCloud( mapPoint, pointType );

  if ( !mPointLayer->isEditable() )
    mPointLayer->startEditing();

  QgsFeature feature( mPointLayer->fields() );
  feature.setGeometry( QgsGeometry( new QgsPoint( mapPoint.x(), mapPoint.y(), z ) ) );
  feature.setAttribute( QStringLiteral( "building_fid" ), mCurrentBuilding.id() );
  feature.setAttribute( QStringLiteral( "type" ), pointType );
  feature.setAttribute( QStringLiteral( "z" ), z );
  feature.setAttribute( QStringLiteral( "source" ), mPointCloudLayer ? mPointCloudLayer->name() : QString() );

  mPointLayer->addFeature( feature );
  mPointLayer->commitChanges();
  mPointLayer->triggerRepaint();

  normalizeCurrentRidgeHeights( false );
  refreshPointTable();
  ensurePointLayerIn3DView();
  notifyRoofModelChanged();
  if ( mCanvas )
    mCanvas->refresh();
}

QgsFeatureId RoofEditTool::findRoofPointAt( const QgsPointXY &mapPoint ) const
{
  if ( !mPointLayer || !mCurrentBuilding.isValid() || !mCanvas )
    return FID_NULL;

  const double tolerance = mCanvas->mapUnitsPerPixel() * 10.0;
  QgsRectangle searchRect( mapPoint.x() - tolerance, mapPoint.y() - tolerance, mapPoint.x() + tolerance, mapPoint.y() + tolerance );
  QgsFeatureRequest request( searchRect );
  request.setFilterExpression( QStringLiteral( "building_fid = %1" ).arg( mCurrentBuilding.id() ) );

  QgsFeatureIterator it = mPointLayer->getFeatures( request );
  QgsFeature feature;
  QgsFeatureId bestFid = FID_NULL;
  double bestDistance = tolerance;
  const QgsGeometry clickGeom = QgsGeometry::fromPointXY( mapPoint );

  while ( it.nextFeature( feature ) )
  {
    if ( !feature.hasGeometry() )
      continue;

    const double distance = feature.geometry().distance( clickGeom );
    if ( distance < bestDistance )
    {
      bestDistance = distance;
      bestFid = feature.id();
    }
  }

  return bestFid;
}

void RoofEditTool::deleteRoofPoint( QgsFeatureId fid )
{
  if ( !mPointLayer || fid == FID_NULL )
    return;

  if ( fid == mMovingPointFid )
    clearPositionPreview();
  if ( fid == mSelectedPointFid )
    mSelectedPointFid = FID_NULL;
  if ( fid == mPreviewPointFid )
  {
    mHasPointEditPreview = false;
    mPreviewPointFid = FID_NULL;
    clearPointPreviewDisplay();
    updatePointEditTip();
  }

  mPointLayer->startEditing();
  mPointLayer->deleteFeature( fid );
  mPointLayer->commitChanges();
  mPointLayer->triggerRepaint();
  refreshPointTable();
  ensurePointLayerIn3DView();
  notifyRoofModelChanged();
  if ( mCanvas )
    mCanvas->refresh();
}

bool RoofEditTool::selectRoofPoint( QgsFeatureId fid )
{
  if ( !mPointLayer || fid == FID_NULL )
    return false;

  QgsFeature feature;
  if ( !mPointLayer->getFeatures( QgsFeatureRequest( fid ) ).nextFeature( feature ) || !feature.isValid() )
    return false;

  mSelectedPointFid = fid;
  mPointLayer->removeSelection();

  if ( mUI.pointTableWidget )
  {
    mUI.pointTableWidget->blockSignals( true );
    mUI.pointTableWidget->clearSelection();
    for ( int row = 0; row < mUI.pointTableWidget->rowCount(); ++row )
    {
      QTableWidgetItem *item = mUI.pointTableWidget->item( row, 0 );
      if ( item && item->data( Qt::UserRole ).toLongLong() == fid )
      {
        mUI.pointTableWidget->selectRow( row );
        break;
      }
    }
    mUI.pointTableWidget->blockSignals( false );
  }

  if ( mCanvas )
    mCanvas->refresh();
  return true;
}

bool RoofEditTool::selectRoofPointAt( const QgsPointXY &mapPoint )
{
  const QgsFeatureId fid = findRoofPointAt( mapPoint );
  return selectRoofPoint( fid );
}

bool RoofEditTool::updateRoofPoint( QgsFeatureId fid, const QgsPoint &point )
{
  return applyRoofPointToLayer( fid, point, true, true );
}

bool RoofEditTool::applyRoofPointToLayer( QgsFeatureId fid, const QgsPoint &point, bool commitNow, bool refreshTable )
{
  if ( !mPointLayer || fid == FID_NULL )
    return false;

  QgsFeature feature;
  if ( !mPointLayer->getFeatures( QgsFeatureRequest( fid ) ).nextFeature( feature ) || !feature.isValid() )
    return false;

  const int zIndex = mPointLayer->fields().indexOf( QStringLiteral( "z" ) );
  if ( !mPointLayer->isEditable() )
    mPointLayer->startEditing();

  QgsGeometry geometry( new QgsPoint( point ) );
  mPointLayer->changeGeometry( fid, geometry );
  if ( zIndex >= 0 )
    mPointLayer->changeAttributeValue( fid, zIndex, point.z() );
  if ( commitNow )
    mPointLayer->commitChanges();
  mPointLayer->triggerRepaint();

  if ( refreshTable )
    refreshPointTable();
  mSelectedPointFid = fid;
  ensurePointLayerIn3DView();
  if ( mCanvas )
    mCanvas->refresh();
  return true;
}

bool RoofEditTool::beginPointEditPreview( QgsFeatureId fid )
{
  if ( !mPointLayer || fid == FID_NULL )
    return false;

  if ( mHasPointEditPreview && mPreviewPointFid == fid )
    return true;

  if ( mHasPointEditPreview )
    cancelPointEditPreview();

  QgsFeature feature;
  if ( !mPointLayer->getFeatures( QgsFeatureRequest( fid ) ).nextFeature( feature ) || !feature.hasGeometry() )
    return false;

  const QgsPoint *point = qgsgeometry_cast<const QgsPoint *>( feature.geometry().constGet() );
  if ( !point )
    return false;

  mPreviewPointFid = fid;
  mPreviewOriginalPoint = *point;
  mPreviewCurrentPoint = *point;
  mHasPointEditPreview = true;
  updatePointEditTip( tr( "点编辑预览中，按 Enter 保存结果，右键取消并回退。" ) );
  return true;
}

bool RoofEditTool::previewRoofPoint( QgsFeatureId fid, const QgsPoint &point )
{
  if ( !beginPointEditPreview( fid ) )
    return false;

  mPreviewCurrentPoint = point;
  updatePointPreviewDisplay( point );
  updateSelectedPointTableRow( point );
  updatePointEditTip( tr( "点编辑预览中，按 Enter 保存结果，右键取消并回退。" ) );
  if ( mCanvas )
    mCanvas->refresh();
  return true;
}

void RoofEditTool::confirmPointEditPreview()
{
  if ( !mHasPointEditPreview )
    return;

  const QgsFeatureId fid = mPreviewPointFid;
  const QgsPoint point = mPreviewCurrentPoint;
  mHasPointEditPreview = false;
  mPreviewPointFid = FID_NULL;
  updateRoofPoint( fid, point );
  normalizeCurrentRidgeHeights( false );
  refreshPointTable();
  clearPointSelection();
  clearPointPreviewDisplay();
  ensurePointLayerIn3DView();
  notifyRoofModelChanged();
  updatePointEditTip( tr( "点编辑结果已确定。" ) );
  if ( mWidget )
    QMessageBox::information( mWidget, tr( "点编辑" ), tr( "点编辑结果已保存。" ) );
  if ( mCanvas )
    mCanvas->refresh();
}

void RoofEditTool::cancelPointEditPreview()
{
  const bool hadPreview = mHasPointEditPreview;

  mHasPointEditPreview = false;
  mPreviewPointFid = FID_NULL;
  clearPositionPreview();
  clearPointPreviewDisplay();

  if ( hadPreview )
    refreshPointTable();
  updatePointEditTip();
  if ( mCanvas )
    mCanvas->refresh();
}

void RoofEditTool::updatePointEditTip( const QString &text )
{
  if ( !mWidget || !mUI.labelTip )
    return;

  if ( text.isEmpty() )
  {
    mUI.labelTip->setText( tr( "提示：添加点时默认使用所选点云图层估算高度；编辑点时先选中点，移动鼠标预览位置，使用鼠标滚轮调整高度，按 Enter 保存，右键取消。" ) );
    return;
  }

  mUI.labelTip->setText( text );
}

void RoofEditTool::clearPointSelection()
{
  mSelectedPointFid = FID_NULL;
  mMovingPointFid = FID_NULL;

  if ( mPointLayer )
    mPointLayer->removeSelection();

  if ( mUI.pointTableWidget )
  {
    mUI.pointTableWidget->blockSignals( true );
    mUI.pointTableWidget->clearSelection();
    mUI.pointTableWidget->blockSignals( false );
  }

  clearPositionPreview();
}

void RoofEditTool::updateSelectedPointTableRow( const QgsPoint &point )
{
  if ( !mUI.pointTableWidget || mPreviewPointFid == FID_NULL )
    return;

  QTableWidget *table = mUI.pointTableWidget;
  table->blockSignals( true );
  for ( int row = 0; row < table->rowCount(); ++row )
  {
    QTableWidgetItem *fidItem = table->item( row, 0 );
    if ( !fidItem || fidItem->data( Qt::UserRole ).toLongLong() != mPreviewPointFid )
      continue;

    if ( QTableWidgetItem *item = table->item( row, 1 ) )
      item->setText( QString::number( point.x(), 'f', 3 ) );
    if ( QTableWidgetItem *item = table->item( row, 2 ) )
      item->setText( QString::number( point.y(), 'f', 3 ) );
    if ( QTableWidgetItem *item = table->item( row, 3 ) )
      item->setText( QString::number( point.z(), 'f', 2 ) );
    break;
  }
  table->blockSignals( false );
}

void RoofEditTool::updatePointPreviewDisplay( const QgsPoint &point )
{
  if ( mCanvas )
  {
    if ( !mPointPreviewRubberBand )
    {
      mPointPreviewRubberBand = new QgsRubberBand( mCanvas, Qgis::GeometryType::Point );
      mPointPreviewRubberBand->setColor( QColor( 205, 65, 220, 230 ) );
      mPointPreviewRubberBand->setSecondaryStrokeColor( QColor( 255, 255, 255, 230 ) );
      mPointPreviewRubberBand->setIcon( QgsRubberBand::ICON_FULL_DIAMOND );
      mPointPreviewRubberBand->setIconSize( 10 );
      mPointPreviewRubberBand->setWidth( 2 );
    }
    mPointPreviewRubberBand->reset( Qgis::GeometryType::Point );
    mPointPreviewRubberBand->addPoint( QgsPointXY( point.x(), point.y() ) );
    mPointPreviewRubberBand->show();
  }

  update3DPreviewPoint( point, mMovingPointFid == FID_NULL );
}

void RoofEditTool::clearPointPreviewDisplay()
{
  if ( mPointPreviewRubberBand )
  {
    mPointPreviewRubberBand->hide();
    mPointPreviewRubberBand->reset( Qgis::GeometryType::Point );
  }

  clear3DPreviewPoint();
}

Qgs3DMapCanvas *RoofEditTool::active3DCanvas() const
{
  if ( !mIface )
    return nullptr;

  Qgs3DMapCanvas *canvas3D = mIface->mapCanvases3D().isEmpty()
                               ? mIface->createNewMapCanvas3D( tr( "3D Preview" ) )
                               : mIface->mapCanvases3D().first();
  if ( canvas3D )
  {
    if ( QWidget *dock = qobject_cast<QWidget *>( canvas3D->parent() ) )
    {
      dock->show();
      dock->raise();
    }
  }
  return canvas3D;
}

void RoofEditTool::ensure3DPreviewEntity()
{
  Qgs3DMapCanvas *canvas3D = active3DCanvas();
  if ( !canvas3D || !canvas3D->scene() || !canvas3D->mapSettings() )
    return;

  if ( mPreviewRootEntity && mPreviewCanvas3D == canvas3D )
    return;

  if ( mPreviewRootEntity )
  {
    mPreviewRootEntity->setEnabled( false );
    mPreviewRootEntity->deleteLater();
  }

  mPreviewCanvas3D = canvas3D;
  mPreviewRootEntity = new Qt3DCore::QEntity( canvas3D->scene() );
  mPreviewRootEntity->setObjectName( QStringLiteral( "RoofEditPreviewRoot" ) );
  mPreviewRootEntity->setEnabled( false );

  mPreviewPointEntity = new Qt3DCore::QEntity( mPreviewRootEntity );
  mPreviewPointEntity->setObjectName( QStringLiteral( "RoofEditPreviewPoint" ) );
  Qt3DExtras::QSphereMesh *pointMesh = new Qt3DExtras::QSphereMesh( mPreviewPointEntity );
  pointMesh->setRadius( 0.2f );
  pointMesh->setRings( 12 );
  pointMesh->setSlices( 16 );
  Qt3DExtras::QPhongMaterial *pointMaterial = new Qt3DExtras::QPhongMaterial( mPreviewPointEntity );
  pointMaterial->setDiffuse( QColor( 210, 45, 230 ) );
  pointMaterial->setAmbient( QColor( 105, 25, 120 ) );
  pointMaterial->setSpecular( QColor( 255, 255, 255 ) );
  pointMaterial->setShininess( 45.0f );
  mPreviewPointTransform = new Qt3DCore::QTransform( mPreviewPointEntity );
  mPreviewPointEntity->addComponent( pointMesh );
  mPreviewPointEntity->addComponent( pointMaterial );
  mPreviewPointEntity->addComponent( mPreviewPointTransform );
}

void RoofEditTool::update3DPreviewPoint( const QgsPoint &point, bool force )
{
  if ( !force )
  {
    if ( !m3DPreviewThrottleTimer.isValid() )
      m3DPreviewThrottleTimer.start();
    else if ( m3DPreviewThrottleTimer.elapsed() < 16 )
      return;
    m3DPreviewThrottleTimer.restart();
  }

  ensure3DPreviewEntity();
  if ( !mPreviewCanvas3D || !mPreviewCanvas3D->mapSettings() || !mPreviewRootEntity || !mPreviewPointTransform )
    return;

  const QgsVector3D world = mPreviewCanvas3D->mapSettings()->mapToWorldCoordinates( QgsVector3D( point.x(), point.y(), point.z() ) );
  mPreviewPointTransform->setTranslation( world.toVector3D() );
  mPreviewRootEntity->setEnabled( true );
}

void RoofEditTool::clear3DPreviewPoint()
{
  if ( mPreviewRootEntity )
    mPreviewRootEntity->setEnabled( false );
}

bool RoofEditTool::adjustSelectedPointHeight( QWheelEvent *event )
{
  if ( !event || !mWidget || !mWidget->isVisible() || !isHeightEditEnabled() || !mPointLayer || mSelectedPointFid == FID_NULL )
    return false;

  const int wheelSteps = event->angleDelta().y() / 120;
  if ( wheelSteps == 0 )
    return false;

  QgsFeature feature;
  if ( !mPointLayer->getFeatures( QgsFeatureRequest( mSelectedPointFid ) ).nextFeature( feature ) || !feature.hasGeometry() )
    return false;

  const QgsPoint *currentPoint = qgsgeometry_cast<const QgsPoint *>( feature.geometry().constGet() );
  if ( !currentPoint )
    return false;

  const QgsPoint basePoint = ( mHasPointEditPreview && mPreviewPointFid == mSelectedPointFid ) ? mPreviewCurrentPoint : *currentPoint;
  const double nextZ = std::max( 0.0, basePoint.z() + wheelSteps * wheelStep() );
  previewRoofPoint( mSelectedPointFid, QgsPoint( basePoint.x(), basePoint.y(), nextZ ) );
  event->accept();
  return true;
}

double RoofEditTool::wheelStep() const
{
  if ( !mUI.wheelStepComboBox )
    return 1.0;

  bool ok = false;
  const double value = mUI.wheelStepComboBox->currentText().toDouble( &ok );
  return ok && value > 0.0 ? value : 1.0;
}

bool RoofEditTool::isHeightEditEnabled() const
{
  if ( !mWidget || !mWidget->isVisible() )
    return false;
  return mCurrentBuilding.isValid();
}

bool RoofEditTool::isPositionEditEnabled() const
{
  if ( !mWidget || !mWidget->isVisible() )
    return false;
  return mCurrentBuilding.isValid();
}

bool RoofEditTool::handlePositionEditClick( const QgsPointXY &mapPoint )
{
  if ( !mPointLayer || !mCurrentBuilding.isValid() )
    return false;

  if ( mMovingPointFid != FID_NULL )
  {
    previewRoofPoint( mMovingPointFid, QgsPoint( mapPoint.x(), mapPoint.y(), currentMovingPreviewZ() ) );
    clearPositionPreview();
    return true;
  }

  const QgsFeatureId fid = findRoofPointAt( mapPoint );
  if ( fid == FID_NULL )
    return false;

  if ( selectRoofPoint( fid ) )
  {
    startPositionPreview( fid );
    updatePositionPreview( mapPoint );
    return true;
  }

  return false;
}

double RoofEditTool::currentMovingPreviewZ() const
{
  if ( mMovingPointFid != FID_NULL && mHasPointEditPreview && mPreviewPointFid == mMovingPointFid )
    return mPreviewCurrentPoint.z();

  return mMovingOriginalPoint.z();
}

void RoofEditTool::startPositionPreview( QgsFeatureId fid )
{
  if ( !mPointLayer || fid == FID_NULL )
    return;

  QgsFeature feature;
  if ( !mPointLayer->getFeatures( QgsFeatureRequest( fid ) ).nextFeature( feature ) || !feature.hasGeometry() )
    return;

  const QgsPoint *point = qgsgeometry_cast<const QgsPoint *>( feature.geometry().constGet() );
  if ( !point )
    return;

  if ( !beginPointEditPreview( fid ) )
    return;

  mMovingPointFid = fid;
  mMovingOriginalPoint = *point;
}

void RoofEditTool::updatePositionPreview( const QgsPointXY &mapPoint )
{
  if ( mMovingPointFid == FID_NULL )
    return;

  const QgsPoint previewPoint( mapPoint.x(), mapPoint.y(), currentMovingPreviewZ() );
  previewRoofPoint( mMovingPointFid, previewPoint );

  if ( mCanvas )
    mCanvas->refresh();
}

void RoofEditTool::clearPositionPreview()
{
  mMovingPointFid = FID_NULL;

  if ( mCanvas )
    mCanvas->refresh();
}

void RoofEditTool::ensurePointLayer()
{
  if ( mPointLayer )
    return;

  const QString crs = mActiveLayer && mActiveLayer->crs().isValid()
                        ? mActiveLayer->crs().authid()
                        : ( mCanvas ? mCanvas->mapSettings().destinationCrs().authid() : QStringLiteral( "EPSG:4326" ) );
  const QString uri = QStringLiteral( "PointZ?crs=%1&field=building_fid:long&field=type:string&field=z:double&field=source:string&field=base_height:double" ).arg( crs );
  mPointLayer = new QgsVectorLayer( uri, tr( "Roof_Edit_Points" ), QStringLiteral( "memory" ) );

  setupPointLayerRenderer();
  QgsProject::instance()->addMapLayer( mPointLayer );
}

void RoofEditTool::ensureSavedPointLayer()
{
  auto ensureSavedFields = [this]() {
    if ( !mSavedPointLayer )
      return;

    QList<QgsField> missingFields;
    if ( mSavedPointLayer->fields().indexOf( QStringLiteral( "base_height" ) ) < 0 )
      missingFields << QgsField( QStringLiteral( "base_height" ), QVariant::Double );
    if ( mSavedPointLayer->fields().indexOf( QStringLiteral( "rel_x" ) ) < 0 )
      missingFields << QgsField( QStringLiteral( "rel_x" ), QVariant::Double );
    if ( mSavedPointLayer->fields().indexOf( QStringLiteral( "rel_y" ) ) < 0 )
      missingFields << QgsField( QStringLiteral( "rel_y" ), QVariant::Double );
    if ( mSavedPointLayer->fields().indexOf( QStringLiteral( "building_shape" ) ) < 0 )
      missingFields << QgsField( QStringLiteral( "building_shape" ), QVariant::String );
    if ( mSavedPointLayer->fields().indexOf( QStringLiteral( "building_center_x" ) ) < 0 )
      missingFields << QgsField( QStringLiteral( "building_center_x" ), QVariant::Double );
    if ( mSavedPointLayer->fields().indexOf( QStringLiteral( "building_center_y" ) ) < 0 )
      missingFields << QgsField( QStringLiteral( "building_center_y" ), QVariant::Double );

    if ( !missingFields.isEmpty() )
    {
      mSavedPointLayer->dataProvider()->addAttributes( missingFields );
      mSavedPointLayer->updateFields();
    }
  };

  if ( mSavedPointLayer )
  {
    ensureSavedFields();
    return;
  }

  const QList<QgsMapLayer *> existingLayers = QgsProject::instance()->mapLayersByName( QStringLiteral( "Roof_Saved_Points" ) );
  if ( !existingLayers.isEmpty() )
  {
    mSavedPointLayer = qobject_cast<QgsVectorLayer *>( existingLayers.first() );
    if ( mSavedPointLayer )
    {
      ensureSavedFields();
      return;
    }
  }

  const QString crs = mActiveLayer && mActiveLayer->crs().isValid()
                        ? mActiveLayer->crs().authid()
                        : ( mCanvas ? mCanvas->mapSettings().destinationCrs().authid() : QStringLiteral( "EPSG:4326" ) );
  const QString uri = QStringLiteral( "PointZ?crs=%1&field=building_fid:long&field=type:string&field=z:double&field=source:string&field=base_height:double&field=rel_x:double&field=rel_y:double&field=building_shape:string&field=building_center_x:double&field=building_center_y:double" ).arg( crs );
  mSavedPointLayer = new QgsVectorLayer( uri, tr( "Roof_Saved_Points" ), QStringLiteral( "memory" ) );
  QgsProject::instance()->addMapLayer( mSavedPointLayer, false );

  QgsLayerTree *root = QgsProject::instance()->layerTreeRoot();
  if ( root )
  {
    root->addLayer( mSavedPointLayer );
    if ( QgsLayerTreeLayer *treeLayer = root->findLayer( mSavedPointLayer->id() ) )
      treeLayer->setItemVisibilityChecked( false );
  }
}

void RoofEditTool::setupPointLayerRenderer()
{
  if ( !mPointLayer )
    return;

  const QColor boundaryColor( 20, 170, 80 );
  const QColor ridgeColor( 220, 40, 40 );
  const QColor vertexColor( 245, 205, 35 );
  const QColor surfaceColor( 170, 60, 210 );

  QgsCategoryList categories;
  categories << QgsRendererCategory( ROOF_POINT_BOUNDARY, createRoofMarkerSymbol( QStringLiteral( "circle" ), boundaryColor, 5.0 ), ROOF_POINT_BOUNDARY )
             << QgsRendererCategory( ROOF_POINT_RIDGE, createRoofMarkerSymbol( QStringLiteral( "triangle" ), ridgeColor, 5.0 ), ROOF_POINT_RIDGE )
             << QgsRendererCategory( ROOF_POINT_VERTEX, createRoofMarkerSymbol( QStringLiteral( "diamond" ), vertexColor, 5.0 ), ROOF_POINT_VERTEX )
             << QgsRendererCategory( ROOF_POINT_SURFACE, createRoofMarkerSymbol( QStringLiteral( "square" ), surfaceColor, 5.0 ), ROOF_POINT_SURFACE );
  mPointLayer->setRenderer( new QgsCategorizedSymbolRenderer( QStringLiteral( "type" ), categories ) );

  QgsRuleBased3DRenderer::Rule *rootRule = new QgsRuleBased3DRenderer::Rule( nullptr );
  rootRule->appendChild( new QgsRuleBased3DRenderer::Rule(
    createRoofPoint3DSymbol( QStringLiteral( "circle" ), boundaryColor ),
    QStringLiteral( "\"type\" = '边界点'" ),
    ROOF_POINT_BOUNDARY ) );
  rootRule->appendChild( new QgsRuleBased3DRenderer::Rule(
    createRoofPoint3DSymbol( QStringLiteral( "triangle" ), ridgeColor ),
    QStringLiteral( "\"type\" = '屋脊点'" ),
    ROOF_POINT_RIDGE ) );
  rootRule->appendChild( new QgsRuleBased3DRenderer::Rule(
    createRoofPoint3DSymbol( QStringLiteral( "diamond" ), vertexColor ),
    QStringLiteral( "\"type\" = '顶点'" ),
    ROOF_POINT_VERTEX ) );
  rootRule->appendChild( new QgsRuleBased3DRenderer::Rule(
    createRoofPoint3DSymbol( QStringLiteral( "square" ), surfaceColor ),
    QStringLiteral( "\"type\" = '曲面点'" ),
    ROOF_POINT_SURFACE ) );

  QgsRuleBased3DRenderer *renderer3D = new QgsRuleBased3DRenderer( rootRule );
  renderer3D->setLayer( mPointLayer );
  mPointLayer->setRenderer3D( renderer3D );
}

void RoofEditTool::ensurePointLayerIn3DView()
{
  if ( !mIface || !mPointLayer )
    return;

  Qgs3DMapCanvas *canvas3D = mIface->mapCanvases3D().isEmpty()
                               ? mIface->createNewMapCanvas3D( tr( "3D Preview" ) )
                               : mIface->mapCanvases3D().first();
  if ( !canvas3D || !canvas3D->mapSettings() )
    return;

  QList<QgsMapLayer *> layers = canvas3D->mapSettings()->layers();
  if ( !layers.contains( mPointLayer ) )
  {
    layers.append( mPointLayer );
    canvas3D->mapSettings()->setLayers( layers );
  }

  if ( QWidget *dock = qobject_cast<QWidget *>( canvas3D->parent() ) )
  {
    dock->show();
    dock->raise();
  }
}

void RoofEditTool::ensureRoofPreviewLayer()
{
  if ( mRoofPreviewLayer )
    return;

  const QString crs = mActiveLayer && mActiveLayer->crs().isValid()
                        ? mActiveLayer->crs().authid()
                        : ( mCanvas ? mCanvas->mapSettings().destinationCrs().authid() : QStringLiteral( "EPSG:4326" ) );
  const QString uri = QStringLiteral( "PolygonZ?crs=%1&field=building_fid:long&field=roof_type:string&field=preview:int" ).arg( crs );
  mRoofPreviewLayer = new QgsVectorLayer( uri, tr( "Roof_Edit_Roof_Preview" ), QStringLiteral( "memory" ) );

  setupRoofLayerRenderer( mRoofPreviewLayer, true );
  QgsProject::instance()->addMapLayer( mRoofPreviewLayer );
}

void RoofEditTool::ensureRoofModelLayer()
{
  if ( mRoofModelLayer )
    return;

  const QString crs = mActiveLayer && mActiveLayer->crs().isValid()
                        ? mActiveLayer->crs().authid()
                        : ( mCanvas ? mCanvas->mapSettings().destinationCrs().authid() : QStringLiteral( "EPSG:4326" ) );
  const QString uri = QStringLiteral( "PolygonZ?crs=%1&field=building_fid:long&field=roof_type:string&field=preview:int" ).arg( crs );
  mRoofModelLayer = new QgsVectorLayer( uri, tr( "Roof_Edit_Roofs" ), QStringLiteral( "memory" ) );

  setupRoofLayerRenderer( mRoofModelLayer, false );
  QgsProject::instance()->addMapLayer( mRoofModelLayer );
}

void RoofEditTool::setupRoofLayerRenderer( QgsVectorLayer *layer, bool preview )
{
  if ( !layer )
    return;

  const QString fillColor = preview ? QStringLiteral( "0,170,255,70" ) : QStringLiteral( "30,150,210,120" );
  const QString outlineColor = preview ? QStringLiteral( "0,115,210,210" ) : QStringLiteral( "20,80,150,230" );
  std::unique_ptr<QgsFillSymbol> symbol = QgsFillSymbol::createSimple(
    { { QStringLiteral( "color" ), fillColor },
      { QStringLiteral( "outline_color" ), outlineColor },
      { QStringLiteral( "outline_width" ), QStringLiteral( "0.45" ) } } );
  layer->setRenderer( new QgsSingleSymbolRenderer( symbol.release() ) );

  QgsPolygon3DSymbol *symbol3D = new QgsPolygon3DSymbol();
  symbol3D->setAltitudeClamping( Qgis::AltitudeClamping::Absolute );
  QgsPhongMaterialSettings *material = new QgsPhongMaterialSettings();
  const QColor color = preview ? QColor( 0, 170, 255, 150 ) : QColor( 30, 150, 210, 185 );
  material->setAmbient( color.darker( 130 ) );
  material->setDiffuse( color );
  symbol3D->setMaterialSettings( material );

  QgsVectorLayer3DRenderer *renderer3D = new QgsVectorLayer3DRenderer();
  renderer3D->setLayer( layer );
  renderer3D->setSymbol( symbol3D );
  layer->setRenderer3D( renderer3D );
}

void RoofEditTool::ensureLayerIn3DView( QgsVectorLayer *layer )
{
  if ( !mIface || !layer )
    return;

  Qgs3DMapCanvas *canvas3D = mIface->mapCanvases3D().isEmpty()
                               ? mIface->createNewMapCanvas3D( tr( "3D Preview" ) )
                               : mIface->mapCanvases3D().first();
  if ( !canvas3D || !canvas3D->mapSettings() )
    return;

  QList<QgsMapLayer *> layers = canvas3D->mapSettings()->layers();
  if ( !layers.contains( layer ) )
  {
    layers.append( layer );
    canvas3D->mapSettings()->setLayers( layers );
  }

  if ( QWidget *dock = qobject_cast<QWidget *>( canvas3D->parent() ) )
  {
    dock->show();
    dock->raise();
  }

  if ( QWidget *canvasWidget = qobject_cast<QWidget *>( canvas3D ) )
    canvasWidget->update();
}

QList<BuildingRoof::RoofPoint> RoofEditTool::currentRoofPoints() const
{
  QList<BuildingRoof::RoofPoint> points;
  if ( !mPointLayer || !mCurrentBuilding.isValid() )
    return points;

  QgsFeatureRequest request;
  request.setFilterExpression( QStringLiteral( "building_fid = %1" ).arg( mCurrentBuilding.id() ) );
  QgsFeatureIterator it = mPointLayer->getFeatures( request );
  QgsFeature feature;

  while ( it.nextFeature( feature ) )
  {
    if ( !feature.hasGeometry() )
      continue;

    const QgsPoint *point = qgsgeometry_cast<const QgsPoint *>( feature.geometry().constGet() );
    if ( !point )
      continue;

    QgsPoint roofPoint = *point;
    bool ok = false;
    const double z = feature.attribute( QStringLiteral( "z" ) ).toDouble( &ok );
    if ( ok )
      roofPoint.setZ( z );

    points.append( BuildingRoof::RoofPoint{ roofPoint, feature.attribute( QStringLiteral( "type" ) ).toString() } );
  }

  return points;
}

bool RoofEditTool::normalizeCurrentRidgeHeights( bool refreshUi )
{
  if ( !mPointLayer || !mCurrentBuilding.isValid() )
    return false;

  QVector<QPair<QgsFeatureId, QgsPoint>> boundaries;
  QVector<EditableRidgePoint> ridges;

  QgsFeatureRequest request;
  request.setFilterExpression( QStringLiteral( "building_fid = %1" ).arg( mCurrentBuilding.id() ) );
  QgsFeatureIterator it = mPointLayer->getFeatures( request );
  QgsFeature feature;
  while ( it.nextFeature( feature ) )
  {
    const QgsPoint *point = feature.hasGeometry() ? qgsgeometry_cast<const QgsPoint *>( feature.geometry().constGet() ) : nullptr;
    if ( !point )
      continue;

    QgsPoint roofPoint = *point;
    bool ok = false;
    const double z = feature.attribute( QStringLiteral( "z" ) ).toDouble( &ok );
    if ( ok )
      roofPoint.setZ( z );

    const QString type = feature.attribute( QStringLiteral( "type" ) ).toString();
    if ( isRoofRidgePointType( type ) )
      ridges.append( EditableRidgePoint{ feature.id(), roofPoint, 0.0 } );
    else if ( isRoofBoundaryPointType( type ) )
      boundaries.append( qMakePair( feature.id(), roofPoint ) );
  }

  if ( boundaries.isEmpty() || ridges.size() < 2 )
    return false;

  QVector<QPair<QgsFeatureId, double>> updates;
  auto appendAverageUpdate = [&updates]( const EditableRidgePoint &first, const EditableRidgePoint &second ) {
    if ( std::fabs( first.point.z() - second.point.z() ) > BuildingRoof::RIDGE_HEIGHT_AVERAGE_THRESHOLD )
      return;

    const double averageZ = 0.5 * ( first.point.z() + second.point.z() );
    updates.append( qMakePair( first.fid, averageZ ) );
    updates.append( qMakePair( second.fid, averageZ ) );
  };

  if ( boundaries.size() == 1 && ridges.size() == 2 )
  {
    appendAverageUpdate( ridges[0], ridges[1] );
  }
  else
  {
    const QVector<QgsPointXY> ring = featureExteriorRing( mCurrentBuilding );
    double dirX = 0.0;
    double dirY = 0.0;
    if ( !nearestRoofEditEdgeDirection( ring, boundaries.first().second, dirX, dirY ) )
      return false;

    const double normalX = -dirY;
    const double normalY = dirX;
    for ( EditableRidgePoint &ridge : ridges )
      ridge.s = roofEditProfileDistance( ridge.point, normalX, normalY );

    std::sort( ridges.begin(), ridges.end(), []( const EditableRidgePoint &lhs, const EditableRidgePoint &rhs ) {
      return lhs.s < rhs.s;
    } );

    for ( int left = 0, right = ridges.size() - 1; left < right; ++left, --right )
      appendAverageUpdate( ridges[left], ridges[right] );
  }

  if ( updates.isEmpty() )
    return false;

  const int zIndex = mPointLayer->fields().indexOf( QStringLiteral( "z" ) );
  if ( !mPointLayer->isEditable() )
    mPointLayer->startEditing();

  for ( const QPair<QgsFeatureId, double> &update : updates )
  {
    QgsFeature pointFeature;
    if ( !mPointLayer->getFeatures( QgsFeatureRequest( update.first ) ).nextFeature( pointFeature ) || !pointFeature.hasGeometry() )
      continue;

    const QgsPoint *oldPoint = qgsgeometry_cast<const QgsPoint *>( pointFeature.geometry().constGet() );
    if ( !oldPoint )
      continue;

    QgsPoint newPoint = *oldPoint;
    newPoint.setZ( update.second );
    QgsGeometry newGeometry( new QgsPoint( newPoint ) );
    mPointLayer->changeGeometry( update.first, newGeometry );
    if ( zIndex >= 0 )
      mPointLayer->changeAttributeValue( update.first, zIndex, update.second );
  }

  mPointLayer->commitChanges();
  mPointLayer->triggerRepaint();

  if ( refreshUi )
    refreshPointTable();
  ensurePointLayerIn3DView();
  if ( mCanvas )
    mCanvas->refresh();
  return true;
}

void RoofEditTool::saveCurrentRoofPoints()
{
  if ( !mCurrentBuilding.isValid() || !mPointLayer )
    return;

  ensureSavedPointLayer();
  if ( !mSavedPointLayer )
    return;

  QgsFeatureIds oldIds;
  QgsFeatureRequest savedRequest;
  savedRequest.setFilterExpression( QStringLiteral( "building_fid = %1" ).arg( mCurrentBuilding.id() ) );
  QgsFeatureIterator savedIt = mSavedPointLayer->getFeatures( savedRequest );
  QgsFeature savedFeature;
  while ( savedIt.nextFeature( savedFeature ) )
    oldIds.insert( savedFeature.id() );

  QgsFeatureList newFeatures;
  const QgsRectangle buildingBounds = mCurrentBuilding.geometry().boundingBox();
  const double width = buildingBounds.width();
  const double height = buildingBounds.height();
  const QString buildingShape = buildingShapeSignature( mCurrentBuilding.geometry() );
  const double buildingCenterX = buildingBounds.center().x();
  const double buildingCenterY = buildingBounds.center().y();
  QgsFeatureRequest draftRequest;
  draftRequest.setFilterExpression( QStringLiteral( "building_fid = %1" ).arg( mCurrentBuilding.id() ) );
  QgsFeatureIterator draftIt = mPointLayer->getFeatures( draftRequest );
  QgsFeature draftFeature;
  while ( draftIt.nextFeature( draftFeature ) )
  {
    const QgsPoint *draftPoint = draftFeature.hasGeometry() ? qgsgeometry_cast<const QgsPoint *>( draftFeature.geometry().constGet() ) : nullptr;
    if ( !draftPoint )
      continue;

    const double relX = width > 1e-12 ? ( draftPoint->x() - buildingBounds.xMinimum() ) / width : 0.5;
    const double relY = height > 1e-12 ? ( draftPoint->y() - buildingBounds.yMinimum() ) / height : 0.5;

    QgsFeature feature( mSavedPointLayer->fields() );
    feature.setGeometry( draftFeature.geometry() );
    feature.setAttribute( QStringLiteral( "building_fid" ), mCurrentBuilding.id() );
    feature.setAttribute( QStringLiteral( "type" ), draftFeature.attribute( QStringLiteral( "type" ) ) );
    feature.setAttribute( QStringLiteral( "z" ), draftFeature.attribute( QStringLiteral( "z" ) ) );
    feature.setAttribute( QStringLiteral( "source" ), draftFeature.attribute( QStringLiteral( "source" ) ) );
    feature.setAttribute( QStringLiteral( "base_height" ), QVariant() );
    feature.setAttribute( QStringLiteral( "rel_x" ), relX );
    feature.setAttribute( QStringLiteral( "rel_y" ), relY );
    feature.setAttribute( QStringLiteral( "building_shape" ), buildingShape );
    feature.setAttribute( QStringLiteral( "building_center_x" ), buildingCenterX );
    feature.setAttribute( QStringLiteral( "building_center_y" ), buildingCenterY );
    newFeatures.append( feature );
  }

  if ( !mSavedPointLayer->isEditable() )
    mSavedPointLayer->startEditing();
  if ( !oldIds.isEmpty() )
    mSavedPointLayer->deleteFeatures( oldIds );
  if ( !newFeatures.isEmpty() )
    mSavedPointLayer->addFeatures( newFeatures );
  mSavedPointLayer->commitChanges();
  mSavedPointLayer->triggerRepaint();
}

bool RoofEditTool::hasSavedRoofPoints( QgsFeatureId buildingFid ) const
{
  if ( buildingFid == FID_NULL )
    return false;

  const QList<QgsMapLayer *> layers = QgsProject::instance()->mapLayersByName( QStringLiteral( "Roof_Saved_Points" ) );
  if ( layers.isEmpty() )
    return false;

  QgsVectorLayer *savedLayer = qobject_cast<QgsVectorLayer *>( layers.first() );
  if ( !savedLayer )
    return false;

  QgsFeatureRequest request;
  request.setFilterExpression( QStringLiteral( "building_fid = %1" ).arg( buildingFid ) );
  QgsFeature feature;
  return savedLayer->getFeatures( request ).nextFeature( feature );
}

void RoofEditTool::clearDraftPointsForBuilding( QgsFeatureId buildingFid, bool refreshUi )
{
  if ( !mPointLayer || buildingFid == FID_NULL )
    return;

  QgsFeatureRequest request;
  request.setFilterExpression( QStringLiteral( "building_fid = %1" ).arg( buildingFid ) );
  QgsFeatureIterator it = mPointLayer->getFeatures( request );
  QgsFeature feature;
  QgsFeatureIds ids;
  while ( it.nextFeature( feature ) )
    ids.insert( feature.id() );

  if ( ids.isEmpty() )
    return;

  if ( ids.contains( mMovingPointFid ) )
    clearPositionPreview();
  if ( ids.contains( mSelectedPointFid ) )
    mSelectedPointFid = FID_NULL;
  if ( ids.contains( mPreviewPointFid ) )
  {
    mHasPointEditPreview = false;
    mPreviewPointFid = FID_NULL;
    clearPointPreviewDisplay();
    updatePointEditTip();
  }

  if ( !mPointLayer->isEditable() )
    mPointLayer->startEditing();
  mPointLayer->deleteFeatures( ids );
  mPointLayer->commitChanges();
  mPointLayer->triggerRepaint();

  if ( refreshUi )
    refreshPointTable();
  if ( mCanvas )
    mCanvas->refresh();
}

void RoofEditTool::notifyRoofModelChanged()
{
  if ( mCurrentBuilding.isValid() )
    notifyRoofModelChanged( mCurrentBuilding.id() );
}

void RoofEditTool::notifyRoofModelChanged( QgsFeatureId buildingFid )
{
  if ( mActiveLayer && buildingFid != FID_NULL )
    emit BuildingEditPreviewBus::instance()->roofModelChanged( mActiveLayer, buildingFid );
}

void RoofEditTool::setTriangleMeshModeEnabled( bool enabled )
{
  if ( mWidget && mUI.pointradio && mUI.pointradio->isChecked() != enabled )
  {
    mUI.pointradio->setChecked( enabled );
    return;
  }

  emit BuildingEditPreviewBus::instance()->buildingTriangleMeshModeChanged(
    mActiveLayer,
    enabled && mCurrentBuilding.isValid() ? mCurrentBuilding.id() : FID_NULL,
    enabled && mCurrentBuilding.isValid()
  );
}

void RoofEditTool::updateTriangleMeshModeForCurrentBuilding()
{
  if ( !mWidget || !mUI.pointradio || !mUI.pointradio->isChecked() )
    return;

  emit BuildingEditPreviewBus::instance()->buildingTriangleMeshModeChanged(
    mActiveLayer,
    mCurrentBuilding.isValid() ? mCurrentBuilding.id() : FID_NULL,
    mCurrentBuilding.isValid()
  );
}

bool RoofEditTool::updateRoofLayerFeature( QgsVectorLayer *layer, const QgsGeometry &geometry, bool preview )
{
  if ( !layer || !mCurrentBuilding.isValid() || geometry.isNull() || geometry.isEmpty() )
    return false;

  QgsFeatureIds oldIds;
  QgsFeatureRequest request;
  request.setFilterExpression( QStringLiteral( "building_fid = %1" ).arg( mCurrentBuilding.id() ) );
  QgsFeatureIterator it = layer->getFeatures( request );
  QgsFeature oldFeature;
  while ( it.nextFeature( oldFeature ) )
    oldIds.insert( oldFeature.id() );

  if ( !layer->isEditable() )
    layer->startEditing();

  if ( !oldIds.isEmpty() )
    layer->deleteFeatures( oldIds );

  QgsFeature feature( layer->fields() );
  feature.setGeometry( geometry );
  feature.setAttribute( QStringLiteral( "building_fid" ), mCurrentBuilding.id() );
  feature.setAttribute( QStringLiteral( "roof_type" ), tr( "Roof model" ) );
  feature.setAttribute( QStringLiteral( "preview" ), preview ? 1 : 0 );
  layer->addFeature( feature );
  layer->commitChanges();
  layer->triggerRepaint();

  if ( mCanvas )
    mCanvas->refresh();
  return true;
}

void RoofEditTool::previewRoofModel()
{
  if ( !mCurrentBuilding.isValid() )
  {
    QMessageBox::warning( mWidget, tr( "Roof preview" ), tr( "Please select a building feature first." ) );
    return;
  }

  if ( mHasPointEditPreview )
  {
    QMessageBox::warning( mWidget, tr( "Roof preview" ), tr( "Please press Enter to save the current key point edit first." ) );
    return;
  }

  normalizeCurrentRidgeHeights( true );
  const QList<BuildingRoof::RoofPoint> roofPoints = currentRoofPoints();
  const QString pointCloudLayerId = mUI.pointCloudComboBox ? mUI.pointCloudComboBox->currentData().toString() : QString();
  mPointCloudLayer = qobject_cast<QgsPointCloudLayer *>( QgsProject::instance()->mapLayer( pointCloudLayerId ) );
  const QVector<BuildingRoof::RoofSample> pointCloudSamples = collectPointCloudSamplesForGeometry( mCurrentBuilding.geometry() );
  BuildingRoof::MeshResult roofMesh = BuildingRoof::buildFlatReliefPrismMesh( mCurrentBuilding.geometry(), 1.0, roofPoints, pointCloudSamples );
  if ( !roofMesh.success )
    roofMesh = BuildingRoof::buildClusteredFlatTopHippedRoofPrismMesh( mCurrentBuilding.geometry(), 1.0, roofPoints, pointCloudSamples );
  if ( !roofMesh.success )
    roofMesh = BuildingRoof::buildCurvedRoofPrismMesh( mCurrentBuilding.geometry(), 1.0, roofPoints );
  if ( !roofMesh.success )
    roofMesh = BuildingRoof::buildApexRoofPrismMesh( mCurrentBuilding.geometry(), 1.0, roofPoints );
  if ( !roofMesh.success )
    roofMesh = BuildingRoof::buildGabledRoofPrismMesh( mCurrentBuilding.geometry(), 1.0, roofPoints, pointCloudSamples );
  if ( !roofMesh.success )
    roofMesh = BuildingRoof::buildHippedRoofPrismMesh( mCurrentBuilding.geometry(), 1.0, roofPoints );
  if ( !roofMesh.success )
    roofMesh = BuildingRoof::buildMultiRidgePrismMesh( mCurrentBuilding.geometry(), 1.0, roofPoints );
  if ( !roofMesh.success )
  {
    QMessageBox::warning( mWidget, tr( "Roof preview" ), roofMesh.error );
    return;
  }

  notifyRoofModelChanged();
  return;

  if ( !mCurrentBuilding.isValid() )
  {
    QMessageBox::warning( mWidget, tr( "屋顶预览" ), tr( "请先选择一个建筑物面要素。" ) );
    return;
  }

  if ( mHasPointEditPreview )
  {
    QMessageBox::warning( mWidget, tr( "屋顶预览" ), tr( "请先按 Enter 保存当前关键点编辑结果。" ) );
    return;
  }

  const BuildingRoof::Result roof = BuildingRoof::buildSingleSlopeRoof( mCurrentBuilding.geometry(), currentRoofPoints() );
  if ( !roof.success )
  {
    QMessageBox::warning( mWidget, tr( "屋顶预览" ), roof.error );
    return;
  }

  ensureRoofPreviewLayer();
  if ( updateRoofLayerFeature( mRoofPreviewLayer, roof.geometry, true ) )
    ensureLayerIn3DView( mRoofPreviewLayer );
}

void RoofEditTool::deleteRoofModel()
{
  if ( !mCurrentBuilding.isValid() )
  {
    QMessageBox::warning( mWidget, tr( "Delete roof" ), tr( "Please select a building feature first." ) );
    return;
  }

  const QgsFeatureId buildingFid = mCurrentBuilding.id();
  auto deleteLayerFeaturesForBuilding = [buildingFid]( QgsVectorLayer *layer ) {
    if ( !layer )
      return;

    QgsFeatureIds ids;
    QgsFeatureRequest request;
    request.setFilterExpression( QStringLiteral( "building_fid = %1" ).arg( buildingFid ) );
    QgsFeatureIterator it = layer->getFeatures( request );
    QgsFeature feature;
    while ( it.nextFeature( feature ) )
      ids.insert( feature.id() );

    if ( ids.isEmpty() )
      return;

    if ( !layer->isEditable() )
      layer->startEditing();
    layer->deleteFeatures( ids );
    layer->commitChanges();
    layer->triggerRepaint();
  };

  cancelPointEditPreview();
  clearPointSelection();
  setTriangleMeshModeEnabled( false );

  deleteLayerFeaturesForBuilding( mPointLayer );
  ensureSavedPointLayer();
  deleteLayerFeaturesForBuilding( mSavedPointLayer );
  deleteLayerFeaturesForBuilding( mRoofPreviewLayer );
  deleteLayerFeaturesForBuilding( mRoofModelLayer );

  refreshPointTable();
  clearPointPreviewDisplay();
  notifyRoofModelChanged( buildingFid );
  updatePointEditTip( tr( "Roof model deleted. The building has returned to the original flat roof." ) );

  if ( mCanvas )
    mCanvas->refresh();
}

void RoofEditTool::saveRoofModel()
{
  if ( !mCurrentBuilding.isValid() )
  {
    QMessageBox::warning( mWidget, tr( "Save roof" ), tr( "Please select a building feature first." ) );
    return;
  }

  if ( mHasPointEditPreview )
  {
    QMessageBox::warning( mWidget, tr( "Save roof" ), tr( "Please press Enter to save the current key point edit first." ) );
    return;
  }

  normalizeCurrentRidgeHeights( true );
  const QList<BuildingRoof::RoofPoint> roofPoints = currentRoofPoints();
  const QString pointCloudLayerId = mUI.pointCloudComboBox ? mUI.pointCloudComboBox->currentData().toString() : QString();
  mPointCloudLayer = qobject_cast<QgsPointCloudLayer *>( QgsProject::instance()->mapLayer( pointCloudLayerId ) );
  const QVector<BuildingRoof::RoofSample> pointCloudSamples = collectPointCloudSamplesForGeometry( mCurrentBuilding.geometry() );
  BuildingRoof::MeshResult roofMesh = BuildingRoof::buildFlatReliefPrismMesh( mCurrentBuilding.geometry(), 1.0, roofPoints, pointCloudSamples );
  if ( !roofMesh.success )
    roofMesh = BuildingRoof::buildClusteredFlatTopHippedRoofPrismMesh( mCurrentBuilding.geometry(), 1.0, roofPoints, pointCloudSamples );
  if ( !roofMesh.success )
    roofMesh = BuildingRoof::buildCurvedRoofPrismMesh( mCurrentBuilding.geometry(), 1.0, roofPoints );
  if ( !roofMesh.success )
    roofMesh = BuildingRoof::buildApexRoofPrismMesh( mCurrentBuilding.geometry(), 1.0, roofPoints );
  if ( !roofMesh.success )
    roofMesh = BuildingRoof::buildGabledRoofPrismMesh( mCurrentBuilding.geometry(), 1.0, roofPoints, pointCloudSamples );
  if ( !roofMesh.success )
    roofMesh = BuildingRoof::buildHippedRoofPrismMesh( mCurrentBuilding.geometry(), 1.0, roofPoints );
  if ( !roofMesh.success )
    roofMesh = BuildingRoof::buildMultiRidgePrismMesh( mCurrentBuilding.geometry(), 1.0, roofPoints );
  if ( !roofMesh.success )
  {
    QMessageBox::warning( mWidget, tr( "Save roof" ), roofMesh.error );
    return;
  }

  const QgsFeatureId savedBuildingFid = mCurrentBuilding.id();
  saveCurrentRoofPoints();
  clearDraftPointsForBuilding( savedBuildingFid, true );
  notifyRoofModelChanged( savedBuildingFid );
  QMessageBox::information( mWidget, tr( "Save roof" ), tr( "Roof key points saved. The 3D building mesh has been rebuilt." ) );

  cancelPointEditPreview();
  clearPointSelection();
  setTriangleMeshModeEnabled( false );
  if ( mActiveLayer )
    mActiveLayer->removeSelection();
  mCurrentBuilding = QgsFeature();
  if ( mUI.pointTableWidget )
    mUI.pointTableWidget->setRowCount( 0 );
  if ( mWidget )
    mWidget->hide();
  if ( mCanvas )
    mCanvas->refresh();
  return;

  if ( !mCurrentBuilding.isValid() )
  {
    QMessageBox::warning( mWidget, tr( "保存屋顶" ), tr( "请先选择一个建筑物面要素。" ) );
    return;
  }

  if ( mHasPointEditPreview )
  {
    QMessageBox::warning( mWidget, tr( "保存屋顶" ), tr( "请先按 Enter 保存当前关键点编辑结果。" ) );
    return;
  }

  const BuildingRoof::Result roof = BuildingRoof::buildSingleSlopeRoof( mCurrentBuilding.geometry(), currentRoofPoints() );
  if ( !roof.success )
  {
    QMessageBox::warning( mWidget, tr( "保存屋顶" ), roof.error );
    return;
  }

  ensureRoofModelLayer();
  if ( updateRoofLayerFeature( mRoofModelLayer, roof.geometry, false ) )
  {
    ensureLayerIn3DView( mRoofModelLayer );
    QMessageBox::information( mWidget, tr( "保存屋顶" ), tr( "屋顶模型已保存。" ) );
  }
}

void RoofEditTool::refreshPointTable()
{
  if ( !mWidget || !mPointLayer || !mCurrentBuilding.isValid() )
    return;

  QTableWidget *table = mUI.pointTableWidget;
  if ( !table )
    return;

  table->blockSignals( true );
  table->setRowCount( 0 );

  QgsFeatureRequest request;
  request.setFilterExpression( QStringLiteral( "building_fid = %1" ).arg( mCurrentBuilding.id() ) );
  QgsFeatureIterator it = mPointLayer->getFeatures( request );
  QgsFeature feature;
  int row = 0;

  while ( it.nextFeature( feature ) )
  {
    const QgsPoint *point = qgsgeometry_cast<const QgsPoint *>( feature.geometry().constGet() );
    if ( !point )
      continue;

    table->insertRow( row );
    const QString type = feature.attribute( QStringLiteral( "type" ) ).toString();
    const double z = feature.attribute( QStringLiteral( "z" ) ).toDouble();
    const QColor color = colorForPointType( type );

    QList<QTableWidgetItem *> items;
    items << new QTableWidgetItem( type )
          << new QTableWidgetItem( QString::number( point->x(), 'f', 3 ) )
          << new QTableWidgetItem( QString::number( point->y(), 'f', 3 ) )
          << new QTableWidgetItem( QString::number( z, 'f', 2 ) );

    for ( QTableWidgetItem *item : items )
    {
      item->setData( Qt::UserRole, feature.id() );
      item->setBackground( QBrush( color.lighter( 170 ) ) );
      table->setItem( row, items.indexOf( item ), item );
    }
    ++row;
  }

  table->blockSignals( false );
}

void RoofEditTool::deleteSelectedPoint()
{
  if ( !mPointLayer || !mUI.pointTableWidget )
    return;

  const QList<QTableWidgetItem *> selected = mUI.pointTableWidget->selectedItems();
  if ( selected.isEmpty() )
    return;

  const QgsFeatureId fid = selected.first()->data( Qt::UserRole ).toLongLong();
  deleteRoofPoint( fid );
}

void RoofEditTool::clearCurrentBuildingPoints()
{
  if ( !mPointLayer || !mCurrentBuilding.isValid() )
    return;

  QgsFeatureRequest request;
  request.setFilterExpression( QStringLiteral( "building_fid = %1" ).arg( mCurrentBuilding.id() ) );
  QgsFeatureIterator it = mPointLayer->getFeatures( request );
  QgsFeature feature;
  QgsFeatureIds ids;
  while ( it.nextFeature( feature ) )
    ids.insert( feature.id() );

  if ( ids.isEmpty() )
    return;

  if ( ids.contains( mMovingPointFid ) )
    clearPositionPreview();
  if ( ids.contains( mSelectedPointFid ) )
    mSelectedPointFid = FID_NULL;
  if ( ids.contains( mPreviewPointFid ) )
  {
    mHasPointEditPreview = false;
    mPreviewPointFid = FID_NULL;
    clearPointPreviewDisplay();
    updatePointEditTip();
  }

  mPointLayer->startEditing();
  mPointLayer->deleteFeatures( ids );
  mPointLayer->commitChanges();
  mPointLayer->triggerRepaint();
  refreshPointTable();
  notifyRoofModelChanged();
  if ( mCanvas )
    mCanvas->refresh();
}

void RoofEditTool::onPointSelectionChanged()
{
  if ( !mUI.pointTableWidget )
    return;

  const QList<QTableWidgetItem *> selected = mUI.pointTableWidget->selectedItems();
  if ( selected.isEmpty() )
  {
    mSelectedPointFid = FID_NULL;
    if ( mPointLayer )
      mPointLayer->removeSelection();
    return;
  }

  const QgsFeatureId fid = selected.first()->data( Qt::UserRole ).toLongLong();
  if ( mHasPointEditPreview && fid != mPreviewPointFid )
    cancelPointEditPreview();

  mSelectedPointFid = fid;
  if ( mPointLayer )
    mPointLayer->removeSelection();
}

double RoofEditTool::estimateHeightFromPointCloud( const QgsPointXY &mapPoint, const QString &pointType ) const
{
  if ( !mPointCloudLayer || !mPointCloudLayer->dataProvider() )
    return 0.0;

  QgsPointCloudIndex index = mPointCloudLayer->dataProvider()->index();
  const double radius = mCanvas ? std::max( mCanvas->mapUnitsPerPixel() * 4.0, 0.5 ) : 0.5;
  QgsRectangle extent( mapPoint.x() - radius, mapPoint.y() - radius, mapPoint.x() + radius, mapPoint.y() + radius );

  QList<QgsPointCloudNodeId> nodeIds;
  collectNodes( index, index.root(), extent, nodeIds );
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

  struct HeightCandidate
  {
    double distance2 = 0.0;
    double z = 0.0;
  };
  QVector<HeightCandidate> candidates;
  const double radius2 = radius * radius;

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
      const double dx = x - mapPoint.x();
      const double dy = y - mapPoint.y();
      const double distance2 = dx * dx + dy * dy;
      if ( distance2 <= radius2 )
        candidates.append( HeightCandidate{ distance2, iz * zScale + zOffset } );
    }
  }

  if ( candidates.isEmpty() )
    return 0.0;

  std::sort( candidates.begin(), candidates.end(), []( const HeightCandidate &lhs, const HeightCandidate &rhs ) {
    return lhs.distance2 < rhs.distance2;
  } );

  QVector<double> values;
  const int nearestCount = std::min( candidates.size(), 20 );
  for ( int i = 0; i < nearestCount; ++i )
    values.append( candidates.at( i ).z );

  std::sort( values.begin(), values.end() );
  return percentileValue( values, heightPercentileForPointType( pointType ) );
}

QVector<BuildingRoof::RoofSample> RoofEditTool::collectPointCloudSamplesForGeometry( const QgsGeometry &geometry, int maxSamples ) const
{
  QVector<BuildingRoof::RoofSample> samples;
  if ( geometry.isNull() || geometry.isEmpty() )
    return samples;

  QgsPointCloudLayer *pointCloudLayer = mPointCloudLayer;
  if ( !pointCloudLayer && mUI.pointCloudComboBox )
    pointCloudLayer = qobject_cast<QgsPointCloudLayer *>( QgsProject::instance()->mapLayer( mUI.pointCloudComboBox->currentData().toString() ) );
  if ( !pointCloudLayer || !pointCloudLayer->dataProvider() )
    return samples;

  QgsPointCloudIndex index = pointCloudLayer->dataProvider()->index();
  const QgsRectangle extent = geometry.boundingBox();
  QList<QgsPointCloudNodeId> nodeIds;
  collectNodes( index, index.root(), extent, nodeIds );
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
  const QgsGeometry preparedGeometry = geometry;
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
      if ( !preparedGeometry.contains( QgsGeometry::fromPointXY( xy ) ) )
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

void RoofEditTool::collectNodes( const QgsPointCloudIndex &index, const QgsPointCloudNodeId &nodeId, const QgsRectangle &extent, QList<QgsPointCloudNodeId> &nodes ) const
{
  Q_UNUSED( extent )
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
      collectNodes( index, childId, extent, nodes );
  }
}
