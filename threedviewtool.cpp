#include "threedviewtool.h"
#include <qgsvectorlayer.h>
#include <qgsproject.h>
#include <qgsmapmouseevent.h>
#include <qgsfeatureiterator.h>
#include <qgsgeometry.h>
#include <qgsmapcanvas.h>
#include <QMessageBox>
#include <QKeyEvent>

// 基础几何
#include <qgspolygon.h>      // 解决“QgsPolygon 不完整类型”
#include <qgslinestring.h>   // 解决“QgsLineString 不完整类型”
#include <qgspoint.h>
#include <qgsfeature.h>

// 3D 渲染模块 (解决渲染器、符号、材质未定义)
#include <qgspolygon3dsymbol.h>
#include <qgsvectorlayer3drenderer.h>
#include <qgsphongmaterialsettings.h>
#include <qgs3dtypes.h>

// Qt 相关
#include <QMatrix4x4>
#include <QVector3D>

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
  if ( mWidget )
  {
    mWidget->deleteLater();
  }
}

// ==================== 鼠标选择逻辑 (保留) ====================
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
  if ( ( e->pos() - mStartScreenPoint ).manhattanLength() < 5 )
    return;

  mIsBoxSelecting = true;
  QgsPointXY current = toMapCoordinates( e->pos() );

  mRubberBand->reset( Qgis::GeometryType::Polygon ); // 修复：使用新的枚举
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

  if ( mIsBoxSelecting )
    selectByRectangle( QgsRectangle( mStartPoint, toMapCoordinates( e->pos() ) ) );
  else
    selectAtPoint( toMapCoordinates( e->pos() ) );

  clearRubberBand();
  mDragging = false;
  mIsBoxSelecting = false;

  if ( !mActiveLayer->selectedFeatureIds().isEmpty() )
    showFieldSelectUI();
}

// ==================== 选择实现 (保留) ====================
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

// ==================== UI 弹窗 (修复错误) ====================
void ThreeDViewTool::showFieldSelectUI()
{
  if ( !mWidget )
  {
    mWidget = new QWidget();
    mUI.setupUi( mWidget );
    mWidget->setWindowTitle( tr( "设置选项" ) );
    mWidget->installEventFilter( this );

    // 修复：如果 UI 里没有 buttonBox，请检查你的 UI 对象名。
    // 如果你用的是 QPushButton，请改为 connect(mUI.yourButtonName, ...)
    // 这里暂时注释掉或改为通用的按钮逻辑
    /*
        connect( mUI.buttonBox, &QDialogButtonBox::accepted, this, &ThreeDViewTool::confirmSelection );
        connect( mUI.buttonBox, &QDialogButtonBox::rejected, this, &ThreeDViewTool::cancelSelection );
        */
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

void ThreeDViewTool::confirmSelection()
{
  if ( !mActiveLayer || !mWidget )
    return;

  QString selectedField = mUI.comboBox->currentText();
  mWidget->hide();

  // 1. 初始化变换矩阵 (参考代码中用于处理位置平移和旋转的 QMatrix4x4)
  QMatrix4x4 mat;
  mat.setToIdentity();

  QgsFeatureList allTriangles;
  QgsFeatureIterator it = mActiveLayer->getSelectedFeatures();
  QgsFeature f;

  while ( it.nextFeature( f ) )
  {
    // 获取用户在 UI 中选择的高度属性值
    double h = f.attribute( selectedField ).toDouble();
    if ( h <= 0 )
      h = 10.0;

    // 这里应当根据你的业务逻辑获取 MeshData
    // 假设你有一个方法能根据拉伸逻辑生成 mesh (类似原代码中的 BuildMesh::build)
    MeshData mesh;
    // ... (此处为生成 mesh 的逻辑，如填充 mesh.vertices 和 mesh.indices)

    // 调用刚才定义的构建函数
    allTriangles.append( buildBuildingFromMesh( mesh, mat ) );
  }

  // 2. 创建内存图层并加载数据
  QString uri = QString( "PolygonZ?crs=%1" ).arg( mActiveLayer->crs().authid() );
  QgsVectorLayer *memLayer = new QgsVectorLayer( uri, tr( "3D_Building_Layer" ), "memory" );
  memLayer->dataProvider()->addFeatures( allTriangles );

  // 3. 核心 3D 渲染器设置 (完全复用参考代码中的渲染参数)
  QgsPolygon3DSymbol *symbol = new QgsPolygon3DSymbol();
  symbol->setAltitudeClamping( Qgis::AltitudeClamping::Absolute ); // 绝对高度
  symbol->setAltitudeBinding( Qgis::AltitudeBinding::Vertex );     // 顶点绑定
  symbol->setCullingMode( Qgs3DTypes::NoCulling );                 // 禁用剔除，保证内外可见

// 1. 使用指针方式创建材质设置
  QgsPhongMaterialSettings *matSettings = new QgsPhongMaterialSettings();
  matSettings->setDiffuse( Qt::cyan );
  matSettings->setAmbient( Qt::darkCyan ); // 建议增加环境光效果更好

  // 2. 传入指针，QGIS 会接管该指针的生命周期（自动管理内存）
  symbol->setMaterialSettings( matSettings );

  QgsVectorLayer3DRenderer *renderer = new QgsVectorLayer3DRenderer();
  renderer->setSymbol( symbol );
  memLayer->setRenderer3D( renderer );

  // 4. 添加到项目
  QgsProject::instance()->addMapLayer( memLayer );
}

void ThreeDViewTool::cancelSelection()
{
  if ( mWidget )
  {
    // 弹出退出提示
    QMessageBox::warning( mWidget, tr( "提示" ), tr( "已取消设置并退出。" ) );

    mWidget->hide();
  }
}

bool ThreeDViewTool::eventFilter( QObject *obj, QEvent *event )
{
  if ( obj == mWidget && event->type() == QEvent::KeyPress )
  {
    QKeyEvent *ke = static_cast<QKeyEvent *>( event );

    // 回车键 -> 保存并提示
    if ( ke->key() == Qt::Key_Return || ke->key() == Qt::Key_Enter )
    {
      confirmSelection();
      return true;
    }

    // ESC 键 -> 退出并提示 (对应你要求的退出功能)
    if ( ke->key() == Qt::Key_Escape )
    {
      cancelSelection();
      return true;
    }
  }
  return QgsMapTool::eventFilter( obj, event );
}

void ThreeDViewTool::createRubberBand()
{
  if ( !mRubberBand )
  {
    mRubberBand = new QgsRubberBand( canvas(), Qgis::GeometryType::Polygon );
    mRubberBand->setColor( QColor( 0, 0, 255, 64 ) );
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
  if ( mWidget )
    mWidget->hide();
  QgsMapTool::deactivate();
}


// ==================== 核心 ====================
QgsFeatureList ThreeDViewTool::buildBuildingFromMesh( const MeshData &mesh, const QMatrix4x4 &mat )
{
  QgsFeatureList features;
  if ( mesh.isEmpty() )
    return features;

  // 每一个三角形面作为一个 Feature
  int triCount = mesh.indices.size() / 3;
  for ( int i = 0; i < triCount; i++ )
  {
    // 1. 获取三角形的三个顶点索引，并应用位姿变换矩阵 (参考代码中的 mat.map 逻辑)
    QVector3D v0 = mat.map( mesh.vertices[mesh.indices[i * 3]] );
    QVector3D v1 = mat.map( mesh.vertices[mesh.indices[i * 3 + 1]] );
    QVector3D v2 = mat.map( mesh.vertices[mesh.indices[i * 3 + 2]] );

    // 2. 构建 QgsPolygon (使用 std::unique_ptr 确保内存安全，适配 QGIS 3)
    std::unique_ptr<QgsPolygon> poly( new QgsPolygon() );
    std::unique_ptr<QgsLineString> ring( new QgsLineString() );

    // 3. 设置三角形的点（必须首尾相接闭合，共4个点）
    ring->setPoints( QgsPointSequence() << QgsPoint( v0.x(), v0.y(), v0.z() ) << QgsPoint( v1.x(), v1.y(), v1.z() ) << QgsPoint( v2.x(), v2.y(), v2.z() ) << QgsPoint( v0.x(), v0.y(), v0.z() ) );

    poly->setExteriorRing( ring.release() );

    // 4. 封装为要素并添加到列表
    QgsFeature feat;
    feat.setGeometry( QgsGeometry( std::move( poly ) ) );
    features.append( feat );
  }

  return features;
}
