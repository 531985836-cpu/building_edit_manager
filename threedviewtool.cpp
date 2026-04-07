#include "threedviewtool.h"
#include <qgsvectorlayer.h>
#include <qgsproject.h>
#include <qgsmapmouseevent.h>
#include <qgsfeatureiterator.h>
#include <qgsgeometry.h>
#include <qgsmapcanvas.h>
#include <QMessageBox>
#include <qgswkbtypes.h>

#include <Qt3DCore/QEntity>
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DRender/QCamera>
#include <Qt3DExtras/QOrbitCameraController>
#include <Qt3DRender/QSceneLoader>

#include <QVector>
#include <QVector3D>
#include <QColor>
#include <QFile>
#include <QTextStream>
#include <QDir>

ThreeDViewTool::ThreeDViewTool( QgsMapCanvas *canvas )
  : QgsMapTool( canvas )
{
  setCursor( Qt::CrossCursor );
  auto layers = QgsProject::instance()->layers<QgsVectorLayer *>();
  if ( !layers.isEmpty() )
    mActiveLayer = layers.first();

  createRubberBand();
}

ThreeDViewTool::~ThreeDViewTool()
{
  clearRubberBand();
}

// ==================== 鼠标事件 ====================
void ThreeDViewTool::canvasPressEvent( QgsMapMouseEvent *e )
{
  if ( !mActiveLayer || e->button() != Qt::LeftButton )
    return;

  mStartPoint = toMapCoordinates( e->pos() );
  mStartScreenPoint = e->pos();
  mDragging = true;
  mIsBoxSelecting = false;
}

void ThreeDViewTool::canvasMoveEvent( QgsMapMouseEvent *e )
{
  if ( !mDragging || !mRubberBand )
    return;

  int dx = e->pos().x() - mStartScreenPoint.x();
  int dy = e->pos().y() - mStartScreenPoint.y();
  if ( dx * dx + dy * dy < 25 )
    return;

  mIsBoxSelecting = true;
  QgsPointXY current = toMapCoordinates( e->pos() );

  mRubberBand->reset();
  mRubberBand->addPoint( mStartPoint, false );
  mRubberBand->addPoint( QgsPointXY( current.x(), mStartPoint.y() ), false );
  mRubberBand->addPoint( current, false );
  mRubberBand->addPoint( QgsPointXY( mStartPoint.x(), current.y() ), false );
  mRubberBand->closePoints();
  mRubberBand->show();
}

void ThreeDViewTool::canvasReleaseEvent( QgsMapMouseEvent *e )
{
  if ( !mDragging || !mActiveLayer )
    return;

  QgsPointXY endPoint = toMapCoordinates( e->pos() );

  if ( mIsBoxSelecting )
    selectByRectangle( QgsRectangle( mStartPoint, endPoint ) );
  else
    selectAtPoint( endPoint );

  clearRubberBand();
  mDragging = false;
  mIsBoxSelecting = false;

  if ( !mActiveLayer->selectedFeatureIds().isEmpty() )
  {
    QMessageBox::information( nullptr, tr( "三维视图" ), tr( "已选择要素，请继续选择高度字段。" ) );
    showFieldSelectUI();
  }
}

// ==================== 选择逻辑 ====================
void ThreeDViewTool::selectAtPoint( const QgsPointXY &point )
{
  double r = searchRadiusMU( canvas() );
  QgsRectangle rect( point.x() - r, point.y() - r, point.x() + r, point.y() + r );

  QgsFeatureIterator it = mActiveLayer->getFeatures( QgsFeatureRequest( rect ) );
  QgsFeature feat;
  QgsFeatureId hit = -1;
  double best = r;

  while ( it.nextFeature( feat ) )
  {
    if ( !feat.hasGeometry() )
      continue;

    double d = feat.geometry().distance( QgsGeometry::fromPointXY( point ) );
    if ( d < best )
    {
      best = d;
      hit = feat.id();
    }
  }

  if ( hit != -1 )
    mActiveLayer->selectByIds( { hit } );

  canvas()->refresh();
}

void ThreeDViewTool::selectByRectangle( const QgsRectangle &rect )
{
  QgsFeatureIds ids;
  QgsFeatureIterator it = mActiveLayer->getFeatures( QgsFeatureRequest( rect ) );
  QgsFeature feat;

  while ( it.nextFeature( feat ) )
    ids.insert( feat.id() );

  mActiveLayer->selectByIds( ids );
  canvas()->refresh();
}

// ==================== UI 相关 ====================
void ThreeDViewTool::showFieldSelectUI()
{
  if ( !mActiveLayer )
    return;

  if ( !mWidget )
  {
    mWidget = new QWidget();
    mUI.setupUi( mWidget );
    mWidget->setWindowTitle( tr( "选择高度字段" ) );
    mWidget->installEventFilter( this );
    connect( mWidget, &QWidget::destroyed, this, &ThreeDViewTool::cancelSelection );
  }

  mUI.comboBox->clear();
  const QgsFields &fields = mActiveLayer->fields();
  for ( const QgsField &f : fields )
  {
    if ( f.isNumeric() )
      mUI.comboBox->addItem( f.name() );
  }

  if ( mUI.comboBox->count() == 0 )
  {
    QMessageBox::warning( nullptr, tr( "三维视图" ), tr( "当前图层没有可用的数值字段" ) );
    return;
  }

  mUI.comboBox->setCurrentIndex( 0 );
  mWidget->show();
  mWidget->raise();
  mWidget->activateWindow();
}

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

// ==================== 确认选择 ====================
void ThreeDViewTool::confirmSelection()
{
  mSelectedField = mUI.comboBox->currentText();
  if ( !mActiveLayer )
    return;

  if ( mWidget )
    mWidget->hide();

  if ( !m3DWindow )
  {
    m3DWindow = new ThreeDWindow();
    m3DWindow->resize( 800, 600 );
  }

  m3DWindow->buildScene( mActiveLayer, mActiveLayer->selectedFeatureIds(), mSelectedField );
  m3DWindow->show();
}

void ThreeDViewTool::cancelSelection()
{
  if ( mWidget )
    mWidget->hide();
  if ( canvas() )
    canvas()->unsetMapTool( this );
}

// ==================== 橡皮筋 ====================
void ThreeDViewTool::createRubberBand()
{
  if ( !mRubberBand )
  {
    mRubberBand = new QgsRubberBand( canvas() );
    mRubberBand->setColor( Qt::blue );
    mRubberBand->setWidth( 1 );
    mRubberBand->hide();
  }
}

void ThreeDViewTool::clearRubberBand()
{
  if ( mRubberBand )
  {
    mRubberBand->hide();
    mRubberBand->reset();
  }
}

void ThreeDViewTool::deactivate()
{
  clearRubberBand();
  QgsMapTool::deactivate();
}

// ==================== Qt3DWindow 实现 ====================
ThreeDViewTool::ThreeDWindow::ThreeDWindow( QScreen *screen )
  : Qt3DExtras::Qt3DWindow( screen )
{
  mRootEntity = new Qt3DCore::QEntity();
  mBuildingsEntity = new Qt3DCore::QEntity( mRootEntity );

  auto cam = this->camera();
  cam->lens()->setPerspectiveProjection( 45.0f, float( width() ) / float( height() ), 0.1f, 1000.0f );
  cam->setPosition( QVector3D( 0, 0, 50 ) );
  cam->setViewCenter( QVector3D( 0, 0, 0 ) );

  auto camController = new Qt3DExtras::QOrbitCameraController( mRootEntity );
  camController->setCamera( cam );

  this->setRootEntity( mRootEntity );
}

// ==================== 构建建筑物 ====================
void ThreeDViewTool::ThreeDWindow::buildScene( QgsVectorLayer *layer, const QgsFeatureIds &ids, const QString &field )
{
  delete mBuildingsEntity;
  mBuildingsEntity = new Qt3DCore::QEntity( mRootEntity );

  if ( !layer || ids.isEmpty() )
    return;

  // 临时 OBJ 保存路径
  QString objFilePath = "D:/cjg/build/Temp/building.obj";

  QFile objFile( objFilePath );
  if ( !objFile.open( QIODevice::WriteOnly | QIODevice::Text ) )
  {
    qWarning() << "无法创建 OBJ 文件:" << objFilePath;
    return;
  }
  QTextStream out( &objFile );

  double xmin = 1e10, xmax = -1e10, ymin = 1e10, ymax = -1e10, maxHeight = 0;
  int vertexOffset = 1; // OBJ 顶点索引从 1 开始

  for ( auto fid : ids )
  {
    QgsFeature feat;
    QgsFeatureRequest req( fid );
    QgsFeatureIterator it = layer->getFeatures( req );
    if ( !it.nextFeature( feat ) || !feat.hasGeometry() )
      continue;

    auto geom = feat.geometry();
    double height = feat.attribute( field ).toDouble();
    maxHeight = std::max( maxHeight, height );

    auto writePolygon = [&]( const QVector<QgsPointXY> &poly ) {
      if ( poly.size() < 3 )
        return;

      // 更新整体 bounding box
      for ( auto &p : poly )
      {
        xmin = std::min( xmin, p.x() );
        xmax = std::max( xmax, p.x() );
        ymin = std::min( ymin, p.y() );
        ymax = std::max( ymax, p.y() );
      }

      // 顶面和底面顶点
      for ( const auto &p : poly )
        out << "v " << p.x() << " " << p.y() << " 0\n";
      for ( const auto &p : poly )
        out << "v " << p.x() << " " << p.y() << " " << height << "\n";

      int n = poly.size();
      // 底面三角面
      for ( int i = 1; i < n - 1; ++i )
        out << "f " << vertexOffset << " " << vertexOffset + i << " " << vertexOffset + i + 1 << "\n";

      // 顶面三角面
      for ( int i = 1; i < n - 1; ++i )
        out << "f " << vertexOffset + n << " " << vertexOffset + n + i + 1 << " " << vertexOffset + n + i << "\n";

      // 侧面四边形拆三角
      for ( int i = 0; i < n; ++i )
      {
        int next = ( i + 1 ) % n;
        int b0 = vertexOffset + i;
        int b1 = vertexOffset + next;
        int t0 = vertexOffset + i + n;
        int t1 = vertexOffset + next + n;
        out << "f " << b0 << " " << b1 << " " << t1 << "\n";
        out << "f " << b0 << " " << t1 << " " << t0 << "\n";
      }

      vertexOffset += 2 * n;
    };

    if ( geom.isMultipart() )
    {
      for ( auto &poly : geom.asMultiPolygon() )
        if ( !poly.isEmpty() )
          writePolygon( poly[0] );
    }
    else
    {
      auto poly = geom.asPolygon();
      if ( !poly.isEmpty() )
        writePolygon( poly[0] );
    }
  }

  objFile.close();

  // ==================== 加载 OBJ ====================
  auto loader = new Qt3DRender::QSceneLoader( mBuildingsEntity );
  loader->setSource( QUrl::fromLocalFile( objFilePath ) );
  mBuildingsEntity->addComponent( loader );

  // ==================== 摄像机调整 ====================
  QVector3D center( ( xmin + xmax ) / 2.0, ( ymin + ymax ) / 2.0, maxHeight / 2.0 );
  auto cam = this->camera();
  cam->setViewCenter( center );
  cam->setPosition( center + QVector3D( 0, 0, maxHeight * 3 ) ); // 拉远三倍高度
  cam->setUpVector( QVector3D( 0, 1, 0 ) );
}
