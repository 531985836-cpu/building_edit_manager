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
<<<<<<< HEAD
#include <qgspolygon.h>
#include <qgslinestring.h>
=======
<<<<<<< HEAD
#include <qgspolygon.h>    // 解决“QgsPolygon 不完整类型”
#include <qgslinestring.h> // 解决“QgsLineString 不完整类型”
=======
#include <qgspolygon.h>      // 解决“QgsPolygon 不完整类型”
#include <qgslinestring.h>   // 解决“QgsLineString 不完整类型”
>>>>>>> f644786332709f1cc37fdca583a1742748c8ba08
>>>>>>> 63dd1b9f77ae99b3223420186805982d8863bfd5
#include <qgspoint.h>

// 3D 渲染模块
#include <qgspolygon3dsymbol.h>
#include <qgsvectorlayer3drenderer.h>
#include <qgsphongmaterialsettings.h>
#include <qgs3dtypes.h>

#include <qgsgeometrycollection.h>
#include <qgisinterface.h>
#include <qgs3dmapcanvas.h>
#include <qgs3dmapsettings.h>
#include <qgsnullsymbolrenderer.h>
#include <qgslayertreelayer.h>
#include <qgslayertree.h>
#include <qgslayertreeview.h>
#include <QMap>

// ==================== 构造与析构 ====================
ThreeDViewTool::ThreeDViewTool( QgsMapCanvas *canvas, QgisInterface *iface )
  : QgsMapTool( canvas )
  , mIface( iface )
{
  setCursor( Qt::CrossCursor );
  auto layers = QgsProject::instance()->layers<QgsVectorLayer *>();
  if ( !layers.isEmpty() )
    mActiveLayer = layers.first();
}

ThreeDViewTool::~ThreeDViewTool()
{
  if ( mWidget )
    mWidget->deleteLater();
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

<<<<<<< HEAD
// ==================== UI 交互 ====================
// 显示字段选择弹窗（旧版）
=======
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
<<<<<<< HEAD
  QTimer::singleShot( 0, canvas(), [this]() { canvas()->refresh(); } );
=======
  canvas()->refresh();
>>>>>>> f644786332709f1cc37fdca583a1742748c8ba08
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
>>>>>>> 63dd1b9f77ae99b3223420186805982d8863bfd5
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
<<<<<<< HEAD
  if ( !mActiveLayer || !mWidget || !mIface )
    return;

  mSelectedHeightField = mUI.comboBox->currentText();
  mWidget->hide();

  if ( !mActiveLayer->isEditable() )
    mActiveLayer->startEditing();

  if ( !mTempLayer )
  {
    QString uri = QString( "PolygonZ?crs=%1&field=original_fid:long" )
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

  MeshData mesh = BuildMesh::build( originFeat.geometry(), h );
  if ( mesh.isEmpty() )
    return;

  QgsFeatureList newTriangles = buildBuildingFromMesh( mesh, QMatrix4x4() );
  for ( QgsFeature &tri : newTriangles )
    tri.setAttributes( QgsAttributes() << originFeat.id() );

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
  QgsFeature originFeat;
  if ( mActiveLayer->getFeatures( QgsFeatureRequest( fid ) ).nextFeature( originFeat ) )
    updateFeature3D( originFeat );
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
<<<<<<< HEAD
=======
=======
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
>>>>>>> f644786332709f1cc37fdca583a1742748c8ba08
}
>>>>>>> 63dd1b9f77ae99b3223420186805982d8863bfd5

  // 强制刷新 3D 视图
  for ( Qgs3DMapCanvas *canvas3D : mIface->mapCanvases3D() )
  {
    if ( canvas3D && canvas3D->mapSettings() )
    {
      QList<QgsMapLayer *> layers = canvas3D->mapSettings()->layers();
      canvas3D->mapSettings()->setLayers( layers );
    }
  }
}
<<<<<<< HEAD
=======

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

<<<<<<< HEAD
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
=======

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
>>>>>>> f644786332709f1cc37fdca583a1742748c8ba08
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


>>>>>>> 63dd1b9f77ae99b3223420186805982d8863bfd5
