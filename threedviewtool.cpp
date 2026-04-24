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
#include <qgspolygon.h>    // 解决“QgsPolygon 不完整类型”
#include <qgslinestring.h> // 解决“QgsLineString 不完整类型”
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

#include <qgsgeometrycollection.h>
#include <qgisinterface.h>    // 解决“不完整类型 QgisInterface”
#include <qgs3dmapcanvas.h>   // 解决“未定义标识符 Qgs3DMapCanvas”
#include <qgs3dmapsettings.h>

// 1. 修改参数列表：增加 QgisInterface *iface
ThreeDViewTool::ThreeDViewTool( QgsMapCanvas *canvas, QgisInterface *iface )
  : QgsMapTool( canvas ) // 初始化父类
  , mIface( iface )      // 核心修改：在这里将传入的 iface 赋值给成员变量 mIface
{
  // 以下逻辑保持不变
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
  QTimer::singleShot( 0, canvas(), [this]() { canvas()->refresh(); } );
}

void ThreeDViewTool::selectByRectangle( const QgsRectangle &rect )
{
  QgsFeatureIds ids;
  QgsFeatureIterator it = mActiveLayer->getFeatures( QgsFeatureRequest( rect ) );
  QgsFeature feat;
  while ( it.nextFeature( feat ) )
    ids.insert( feat.id() );
  mActiveLayer->selectByIds( ids );
  QTimer::singleShot( 0, canvas(), [this]() { canvas()->refresh(); } );
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
  if ( !mActiveLayer || !mWidget || !mIface )
    return;

  mSelectedHeightField = mUI.comboBox->currentText();
  mWidget->hide();

  // ⭐ 只新增这个开关（你自己改 true / false）
  bool showTempLayer = true; // true=显示  false=隐藏

  // 1. 创建内存图层
  if ( !mTempLayer )
  {
    QString uri = QString( "PolygonZ?crs=%1&field=original_fid:long" )
                    .arg( mActiveLayer->crs().authid() );

    mTempLayer = new QgsVectorLayer( uri, tr( "Internal_Memory_3D" ), "memory" );

    // ⭐⭐⭐ 关键新增：控制是否加入项目 ⭐⭐⭐
    if ( showTempLayer )
    {
      QgsProject::instance()->addMapLayer( mTempLayer );
    }

    // ===== 以下全部保持你原来的 =====
    QgsPolygon3DSymbol *symbol = new QgsPolygon3DSymbol();
    symbol->setAltitudeClamping( Qgis::AltitudeClamping::Absolute );

    QgsVectorLayer3DRenderer *renderer = new QgsVectorLayer3DRenderer();
    renderer->setSymbol( symbol );
    mTempLayer->setRenderer3D( renderer );

    connect( mActiveLayer, &QgsVectorLayer::geometryChanged, this, &ThreeDViewTool::onFeatureUpdated );

    connect( mActiveLayer, &QgsVectorLayer::attributeValueChanged, this, [this]( QgsFeatureId fid, int idx, const QVariant &value ) {
      onFeatureUpdated( fid );
    } );
  }

  // ⭐⭐⭐ 关键新增：如果图层已存在，也要根据开关控制显示/隐藏 ⭐⭐⭐
  if ( mTempLayer )
  {
    if ( showTempLayer )
    {
      // 如果没加进项目，就加进去
      if ( !QgsProject::instance()->mapLayers().values().contains( mTempLayer ) )
      {
        QgsProject::instance()->addMapLayer( mTempLayer );
      }
    }
    else
    {
      // 如果已经在项目里，就移除
      if ( QgsProject::instance()->mapLayers().values().contains( mTempLayer ) )
      {
        QgsProject::instance()->removeMapLayer( mTempLayer->id() );
      }
    }
  }

  // 2. 3D窗口逻辑（完全不动）
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

    // 保持你原逻辑：始终注入3D（不动）
    Qgs3DMapSettings *settings = activeCanvas3D->mapSettings();
    QList<QgsMapLayer *> currentLayers = settings->layers();
    if ( !currentLayers.contains( mTempLayer ) )
    {
      currentLayers.append( mTempLayer );
      settings->setLayers( currentLayers );
    }
  }

  // 3. 刷新数据
  refreshMemoryData();
}

// 辅助函数：彻底刷新内存数据
void ThreeDViewTool::refreshMemoryData()
{
  if ( !mTempLayer || !mActiveLayer || mSelectedHeightField.isEmpty() )
    return;

  mTempLayer->startEditing();

  // 1. 清空旧数据
  mTempLayer->deleteFeatures( mTempLayer->allFeatureIds() );

  // 2. 修改点：获取所有要素，而不仅仅是选中的
  // 如果只想看一部分，可以保留 getSelectedFeatures，但必须确保用户操作后重新选中
  QgsFeatureIterator it = mActiveLayer->getFeatures();

  QgsFeature f;
  QgsFeatureList allTriangles;

  while ( it.nextFeature( f ) )
  {
    double h = f.attribute( mSelectedHeightField ).toDouble();
    if ( h <= 0 )
      h = 10.0;

    MeshData mesh = BuildMesh::build( f.geometry(), h );
    QgsFeatureList triangles = buildBuildingFromMesh( mesh, QMatrix4x4() );

    for ( QgsFeature &tri : triangles )
    {
      tri.setAttributes( QgsAttributes() << f.id() );
      allTriangles.append( tri );
    }
  }

  mTempLayer->addFeatures( allTriangles );
  mTempLayer->commitChanges();
  mTempLayer->triggerRepaint();
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

// ====================  ====================
static double cross2D( const QgsPointXY &a, const QgsPointXY &b, const QgsPointXY &c )
{
  return ( b.x() - a.x() ) * ( c.y() - a.y() ) - ( b.y() - a.y() ) * ( c.x() - a.x() );
}

static bool pointInTriangle(const QgsPointXY &p,const QgsPointXY &a,const QgsPointXY &b,const QgsPointXY &c)
{
  double c1 = cross2D( a, b, p );
  double c2 = cross2D( b, c, p );
  double c3 = cross2D( c, a, p );

  // ⚠️ 必须严格 > 0
  return ( c1 > 0 && c2 > 0 && c3 > 0 );
}

static bool isConvex( const QgsPointXY &prev, const QgsPointXY &curr, const QgsPointXY &next )
{
  return cross2D( prev, curr, next ) > 0;
}

// ⭐ 核心：增强版耳切法
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

      // 1. 必须是凸点
      if ( !isConvex( pts[prev], pts[curr], pts[next] ) )
        continue;

      // 2. 检查是否有点在三角形内
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

      // ✔ 找到耳朵
      result << prev << curr << next;
      V.removeAt( i );
      earFound = true;
      break;
    }

    if ( !earFound )
      break; // 防死循环
  }

  if ( V.size() == 3 )
    result << V[0] << V[1] << V[2];

  return result;
}

// ==================== 核心 ====================
QgsFeatureList ThreeDViewTool::buildBuildingFromMesh( const MeshData &mesh, const QMatrix4x4 &mat )
{
  QgsFeatureList features;
  int triCount = mesh.indices.size() / 3;

  for ( int i = 0; i < triCount; i++ )
  {
    // 从 mesh 中获取双精度点
    QgsPoint p0 = mesh.vertices[mesh.indices[i * 3]];
    QgsPoint p1 = mesh.vertices[mesh.indices[i * 3 + 1]];
    QgsPoint p2 = mesh.vertices[mesh.indices[i * 3 + 2]];

    // 如果需要应用矩阵变换 (注意：QMatrix4x4 内部也是 float)
    // 如果只是平移/旋转地理坐标，建议手动处理 double 变换
    // 这里演示直接构建，保持最高精度
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

MeshData BuildMesh::build( const QgsGeometry &geom, double height )
{
  MeshData mesh;
  if ( geom.isEmpty() || !geom.isGeosValid() )
    return mesh;

  double h = ( height <= 0 ? 3.0 : height );

  // 1. 获取并清理几何体
  QgsGeometry inputGeom = geom.convertToType( Qgis::GeometryType::Polygon );
  const QgsPolygon *poly = qgsgeometry_cast<const QgsPolygon *>( inputGeom.constGet() );
  if ( !poly || !poly->exteriorRing() )
    return mesh;

  const QgsCurve *ring = poly->exteriorRing();
  QVector<QgsPointXY> polyPts;

  qDebug() << "--------------------------------------------------";
  qDebug() << "[VS Debug] --- 开启双精度构建模式 ---";

  // 2. 提取点：直接使用 double 存储
  for ( int i = 0; i < ring->numPoints() - 1; ++i )
  {
    QgsPoint p;
    Qgis::VertexType vt;
    ring->pointAt( i, p, vt );
    polyPts.push_back( QgsPointXY( p.x(), p.y() ) );
    // VS 输出：确认此时坐标未丢失精度
    qDebug() << QString( "  [Point %1] Source: X=%2, Y=%3" )
                  .arg( i )
                  .arg( p.x(), 0, 'f', 6 )
                  .arg( p.y(), 0, 'f', 6 );
  }

  // 3. 保证 CCW
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

  // 4. 顶点生成：使用 QgsPoint(x, y, z) 保持 double 精度
  // 这样就不会像 QVector3D (float) 那样产生微小偏移
  for ( const QgsPointXY &p : polyPts )
  {
    mesh.vertices.append( QgsPoint( p.x(), p.y(), 0.0 ) ); // 底点
    mesh.vertices.append( QgsPoint( p.x(), p.y(), h ) );   // 顶点
  }

  // 5. 侧面索引生成
  for ( int i = 0; i < base; ++i )
  {
    int next = ( i + 1 ) % base;
    mesh.indices << 2 * i << 2 * next << 2 * i + 1;
    mesh.indices << 2 * i + 1 << 2 * next << 2 * next + 1;
  }

  // 6. Ear Clipping
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
      int p_idx = V[( i - 1 + V.size() ) % V.size()], c_idx = V[i], n_idx = V[( i + 1 ) % V.size()];
      const QgsPointXY &p1 = polyPts[p_idx], &p2 = polyPts[c_idx], &p3 = polyPts[n_idx];

      double cp = ( p2.x() - p1.x() ) * ( p3.y() - p1.y() ) - ( p2.y() - p1.y() ) * ( p3.x() - p1.x() );
      if ( cp <= 0 )
        continue; // 凹点跳过

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
        qDebug() << QString( "  [VS Success] Clipping Ear: %1-%2-%3" ).arg( p_idx ).arg( c_idx ).arg( n_idx );
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

  qDebug() << "[VS Debug] --- 构建结束 ---";
  return mesh;
}

void ThreeDViewTool::updateFeature3D( const QgsFeature &originFeat )
{
  if ( !mTempLayer )
    return;

  // 计算高度
  double h = originFeat.attribute( mSelectedHeightField ).toDouble();
  if ( h <= 0 )
    h = 5.0; // 默认高度

  // 生成网格
  MeshData mesh = BuildMesh::build( originFeat.geometry(), h );
  if ( mesh.isEmpty() )
    return;

  // 将网格转为 3D 三角形 Feature 列表
  QgsFeatureList newTriangles = buildBuildingFromMesh( mesh, QMatrix4x4() );

  // 标记归属 ID
  for ( QgsFeature &tri : newTriangles )
  {
    tri.setAttributes( QgsAttributes() << originFeat.id() );
  }

  // 更新内存图层 (参考代码增删逻辑)
  mTempLayer->startEditing();

  // 根据 original_fid 删除该要素旧的 3D 模型
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

  // 重要：确保 3D 视图意识到数据变了
  mTempLayer->triggerRepaint();
}


void ThreeDViewTool::onFeatureUpdated( QgsFeatureId fid )
{
  // 获取最新的原图层要素
  QgsFeature originFeat;
  if ( mActiveLayer->getFeatures( QgsFeatureRequest( fid ) ).nextFeature( originFeat ) )
  {
    updateFeature3D( originFeat );
  }
}


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


