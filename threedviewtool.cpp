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
#include <qgsmaptool.h>
#include <QMap>
#include <qgsnullsymbolrenderer.h>
#include <qgslayertreelayer.h>
#include <qgslayertree.h>
#include <qgslayertreeview.h>
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

}

ThreeDViewTool::~ThreeDViewTool()
{
  if ( mWidget )
  {
    mWidget->deleteLater();
  }
}

void ThreeDViewTool::activate()
{
  QgsMapTool::activate();
  setupUI();
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

  // =========================
  // 自动开启原始图层编辑
  // =========================
  if ( !mActiveLayer->isEditable() )
  {
    mActiveLayer->startEditing();
  }

  // =========================
  // 创建并初始化内存图层
  // =========================
  if ( !mTempLayer )
  {
    QString uri = QString(
                    "PolygonZ?crs=%1&field=original_fid:long"
    )
                    .arg(
                      mActiveLayer->crs().authid()
                    );

    mTempLayer = new QgsVectorLayer(
      uri,
      tr( "Internal_Memory_3D" ),
      "memory"
    );

    // =========================
    // 注册到工程（但不自动加入图层树）
    // =========================
    QgsProject::instance()->addMapLayer(
      mTempLayer,
      false
    );

    // =========================
    // 插入到原图层下面
    // =========================
    QgsLayerTree *root = QgsProject::instance()->layerTreeRoot();

    QgsLayerTreeLayer *activeNode = root->findLayer( mActiveLayer->id() );

    if ( activeNode )
    {
      QgsLayerTreeGroup *parentGroup = qobject_cast<QgsLayerTreeGroup *>(activeNode->parent());

      if ( parentGroup )
      {
        int index = parentGroup->children().indexOf(
          activeNode
        );

        parentGroup->insertLayer(
          index + 1,
          mTempLayer
        );
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

    // =========================
    // 隐藏图层树节点
    // =========================
    QgsLayerTreeLayer *treeLayer = root->findLayer( mTempLayer->id() );

    if ( treeLayer )
    {
      treeLayer->setItemVisibilityChecked( false );

      treeLayer->setCustomProperty(
        "nodeHidden",
        true
      );
    }

    // =========================
    // 禁止2D交互
    // =========================
    mTempLayer->setFlags(
      mTempLayer->flags()
      & ~QgsMapLayer::Identifiable
    );

    mTempLayer->setFlags(
      mTempLayer->flags()
      & ~QgsMapLayer::Searchable
    );

    // =========================
    // 禁止2D渲染
    // =========================
    mTempLayer->setRenderer(
      new QgsNullSymbolRenderer()
    );

    // 完全透明
    mTempLayer->setOpacity( 0.0 );

    // =========================
    // 3D Renderer
    // =========================
    QgsPolygon3DSymbol *symbol = new QgsPolygon3DSymbol();

    symbol->setAltitudeClamping(
      Qgis::AltitudeClamping::Absolute
    );

    QgsVectorLayer3DRenderer *renderer = new QgsVectorLayer3DRenderer();

    renderer->setSymbol( symbol );

    mTempLayer->setRenderer3D( renderer );

    // =========================
    // 信号绑定
    // =========================

    // 几何更新
    connect(
      mActiveLayer,
      &QgsVectorLayer::geometryChanged,
      this,
      &ThreeDViewTool::onFeatureUpdated,
      Qt::UniqueConnection
    );

    // 属性更新
    connect(
      mActiveLayer,
      &QgsVectorLayer::attributeValueChanged,
      this,
      [this](
        QgsFeatureId fid,
        int idx,
        const QVariant &value
      ) {
        Q_UNUSED( idx )
        Q_UNUSED( value )

        onFeatureUpdated( fid );
      },
      Qt::UniqueConnection
    );

    // 删除同步
    connect(
      mActiveLayer,
      &QgsVectorLayer::featuresDeleted,
      this,
      &ThreeDViewTool::onFeaturesDeleted,
      Qt::UniqueConnection
    );
  }

  // =========================
  // 获取或创建3D视图
  // =========================
  Qgs3DMapCanvas *activeCanvas3D = mIface->mapCanvases3D().isEmpty()
                                     ? mIface->createNewMapCanvas3D(
                                         tr( "3D Preview" )
                                       )
                                     : mIface->mapCanvases3D().first();

  if ( activeCanvas3D )
  {
    // 显示窗口
    if ( QWidget *dock = qobject_cast<QWidget *>(
           activeCanvas3D->parent()
         ) )
    {
      dock->show();
      dock->raise();
    }

    // 获取当前3D图层
    Qgs3DMapSettings *settings = activeCanvas3D->mapSettings();

    QList<QgsMapLayer *> current3DLayers = settings->layers();

    // 注入临时图层
    if ( !current3DLayers.contains( mTempLayer ) )
    {
      current3DLayers.append( mTempLayer );

      settings->setLayers(
        current3DLayers
      );
    }

    // 刷新3D
    if ( QWidget *cv = qobject_cast<QWidget *>( activeCanvas3D ) )
    {
      cv->update();
    }
  }

  // =========================
  // 刷新三维数据
  // =========================
  refreshMemoryData();

  // =========================
  // 恢复原始图层为当前图层
  // =========================
  mIface->setActiveLayer( mActiveLayer );
}
// 辅助函数：彻底刷新内存数据
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
      tri.setAttributes( QgsAttributes() << f.id() ); // 绑定原始 ID
      allTriangles.append( tri );
    }
  }

  mTempLayer->addFeatures( allTriangles );
  mTempLayer->commitChanges();

  // --- 核心：强制刷新 3D 渲染 ---
  mTempLayer->triggerRepaint(); // 触发信号

  // 某些版本下 triggerRepaint 对 3D 引擎响应较慢，通过重设图层列表强制刷新
  for ( Qgs3DMapCanvas *canvas3D : mIface->mapCanvases3D() )
  {
    if ( canvas3D && canvas3D->mapSettings() )
    {
      QList<QgsMapLayer *> layers = canvas3D->mapSettings()->layers();
      canvas3D->mapSettings()->setLayers( layers );
    }
  }
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

void ThreeDViewTool::deactivate()
{
  if ( mWidget )
    mWidget->hide();
  QgsMapTool::deactivate();
}

// ====================  ====================
void ThreeDViewTool::setupUI()
{
  if ( !mWidget )
  {
    mWidget = new QWidget();
    mUI.setupUi( mWidget );

    // 信号槽连接：切换图层时自动刷新字段
    connect( mUI.layercomboBox, QOverload<int>::of( &QComboBox::currentIndexChanged ), this, &ThreeDViewTool::onLayerChanged );

    // 事件过滤器：捕获回车键执行后续计算
    mWidget->installEventFilter( this );
  }

  // 填充图层列表
  refreshLayerList();
  mWidget->show();
}

void ThreeDViewTool::refreshLayerList()
{
  mUI.layercomboBox->blockSignals( true );
  mUI.layercomboBox->clear();
  mUI.layercomboBox->addItem( "请选择矢量图层...", QVariant() );

  // 遍历项目图层，只添加面图层
  auto layers = QgsProject::instance()->mapLayers().values();
  for ( QgsMapLayer *layer : layers )
  {
    if ( layer->type() == Qgis::LayerType::Vector )
    {
      QgsVectorLayer *vlyr = qobject_cast<QgsVectorLayer *>( layer );
      if ( vlyr && vlyr->geometryType() == Qgis::GeometryType::Polygon )
      {
        mUI.layercomboBox->addItem( vlyr->name(), vlyr->id() );
      }
    }
  }
  mUI.layercomboBox->blockSignals( false );
}

void ThreeDViewTool::onLayerChanged( int index )
{
  if ( index <= 0 )
  {
    mActiveLayer = nullptr;
    mUI.comboBox->clear();
    return;
  }

  // 获取选中的图层 ID
  QString layerId = mUI.layercomboBox->currentData().toString();
  mActiveLayer = qobject_cast<QgsVectorLayer *>( QgsProject::instance()->mapLayer( layerId ) );

  // 刷新字段 ComboBox
  updateFieldsCombo();
}

void ThreeDViewTool::updateFieldsCombo()
{
  mUI.comboBox->clear();
  if ( !mActiveLayer )
    return;

  mUI.comboBox->addItem( "请选择高度字段..." );

  // 获取图层的所有字段
  const QgsFields &fields = mActiveLayer->fields();
  for ( const QgsField &field : fields )
  {
    // 只添加数值类型的字段作为高度源
    if ( field.isNumeric() )
    {
      mUI.comboBox->addItem( field.name() );
    }
  }
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


  // 2. 提取点：直接使用 double 存储
  for ( int i = 0; i < ring->numPoints() - 1; ++i )
  {
    QgsPoint p;
    Qgis::VertexType vt;
    ring->pointAt( i, p, vt );
    polyPts.push_back( QgsPointXY( p.x(), p.y() ) );
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


