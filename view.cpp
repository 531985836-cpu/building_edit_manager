#include "view.h"

#include <qgsmapcanvas.h>
#include <QVBoxLayout>
#include <QWidget>
#include <qgsproject.h>
#include <qgsvectorlayer.h>
#include <qgsmapmouseevent.h>
#include <qgsfeatureiterator.h>
#include <qgsgeometry.h>
#include <qgspolygon.h>
#include <qgslinestring.h>
#include <qgsrectangle.h>
#include <qgsfillsymbol.h>
#include <qgssinglesymbolrenderer.h>
#include <qgslayertreeview.h>
#include <cmath>

#include <memory>

// ======================== 构造函数 ========================

View::View( QgsMapCanvas *canvas, QgisInterface *iface )
  : QgsMapTool( canvas )
  , mIface( iface )
{
  setCursor( Qt::CrossCursor );

  auto layers = QgsProject::instance()->layers<QgsVectorLayer *>();

  for ( QgsVectorLayer *layer : layers )
  {
    if ( layer && layer->geometryType() == Qgis::GeometryType::Polygon )
    {
      mActiveLayer = layer;
      break;
    }
  }
}

// ======================== 析构 ========================

View::~View()
{
}

// ======================== 点击事件 ========================

void View::canvasReleaseEvent( QgsMapMouseEvent *e )
{
  // =========================
  // 右键释放
  // =========================

  if ( e->button() == Qt::RightButton )
  {
    mRightButtonPressed = false;
    return;
  }

  // =========================
  // 左键选择建筑
  // =========================

  if ( !mActiveLayer )
    return;

  QgsPointXY pt = e->mapPoint();

  QgsRectangle rect(
    pt.x() - 1.0,
    pt.y() - 1.0,
    pt.x() + 1.0,
    pt.y() + 1.0
  );

  QgsFeatureRequest request;

  request.setFilterRect( rect );

  QgsFeatureIterator it = mActiveLayer->getFeatures(
    request
  );

  QgsFeature feat;

  if ( it.nextFeature( feat ) )
  {
    mCurrentFeature = feat;

    showSideView( feat );
  }
}

void View::canvasPressEvent( QgsMapMouseEvent *e )
{
  // 右键按下
  if ( e->button() == Qt::RightButton )
  {
    mRightButtonPressed = true;

    mLastMousePos = e->pos();
  }
}

void View::canvasMoveEvent( QgsMapMouseEvent *e )
{
  if ( !mRightButtonPressed )
    return;

  int dx = e->pos().x() - mLastMousePos.x();

  // 鼠标移动控制旋转
  mViewAngle += dx * 0.5;

  // 限制到 0~360
  while ( mViewAngle < 0 )
    mViewAngle += 360.0;

  while ( mViewAngle >= 360.0 )
    mViewAngle -= 360.0;

  mLastMousePos = e->pos();

  // 实时刷新
  if ( mCurrentFeature.isValid() )
  {
    showSideView(
      mCurrentFeature
    );
  }
}
// ======================== 显示侧视图 ========================

void View::showSideView( const QgsFeature &feat )
{
  if ( !mSideLayer )
  {
    QString uri = QString(
                    "Polygon?crs=%1&field=original_fid:long"
    )
                    .arg( mActiveLayer->crs().authid() );

    mSideLayer = new QgsVectorLayer(
      uri,
      "Building_Side_View",
      "memory"
    );

    QgsProject::instance()->addMapLayer( mSideLayer );

    // 设置符号
    std::unique_ptr<QgsFillSymbol> symbol = QgsFillSymbol::createSimple(
      { { "color", "220,220,220" },
        { "outline_color", "0,0,0" },
        { "outline_width", "0.5" }
      }
    );

    mSideLayer->setRenderer(
      new QgsSingleSymbolRenderer(
        symbol.release()
      )
    );
  }

  double h = feat.attribute(
                   mHeightField
  )
               .toDouble();

  if ( h <= 0 )
    h = 10.0;

  QgsGeometry sideGeom = buildSideGeometry(
    feat.geometry(),
    h,
    mViewAngle
  );

  if ( sideGeom.isEmpty() )
    return;

  QgsFeature sideFeat;

  sideFeat.setGeometry(
    sideGeom
  );

  sideFeat.setAttributes(
    QgsAttributes()
    << feat.id()
  );

  mSideLayer->startEditing();

  mSideLayer->deleteFeatures(
    mSideLayer->allFeatureIds()
  );

  mSideLayer->addFeature(
    sideFeat
  );

  mSideLayer->commitChanges();

  mSideLayer->triggerRepaint();

  if ( mIface )
  {
    mIface->layerTreeView()
      ->refreshLayerSymbology(
        mSideLayer->id()
      );
  }

  // ===============================
  // 创建独立侧视图窗口
  // ===============================

  if ( !mSideCanvas )
  {
    QWidget *window = new QWidget();

    window->setWindowTitle(
      "Building Side View"
    );

    window->resize( 800, 600 );

    QVBoxLayout *layout = new QVBoxLayout( window );

    mSideCanvas = new QgsMapCanvas();

    layout->addWidget(
      mSideCanvas
    );

    window->setLayout(
      layout
    );

    window->show();
  }

  // 设置显示图层
  mSideCanvas->setLayers(
    QList<QgsMapLayer *>()
    << mSideLayer
  );

  // 缩放范围
  QgsRectangle extent = sideGeom.boundingBox();

  extent.scale( 1.2 );

  mSideCanvas->setExtent(
    extent
  );

  mSideCanvas->refresh();
}

// ======================== 构建侧视图几何 ========================

QgsGeometry View::buildSideGeometry(
  const QgsGeometry &geom,
  double height,
  double angleDeg
)
{
  if ( geom.isEmpty() )
    return QgsGeometry();

  QgsGeometry inputGeom = geom.convertToType(
    Qgis::GeometryType::Polygon
  );

  const QgsPolygon *poly = qgsgeometry_cast<const QgsPolygon *>(
    inputGeom.constGet()
  );

  if ( !poly || !poly->exteriorRing() )
    return QgsGeometry();

  const QgsCurve *ring = poly->exteriorRing();

  double minProj = 1e20;
  double maxProj = -1e20;

  double rad = angleDeg * M_PI / 180.0;

  double dirX = cos( rad );
  double dirY = sin( rad );

  for ( int i = 0; i < ring->numPoints(); ++i )
  {
    QgsPoint p;
    Qgis::VertexType vt;

    ring->pointAt(
      i,
      p,
      vt
    );

    double proj = p.x() * dirX + p.y() * dirY;

    minProj = std::min(
      minProj,
      proj
    );

    maxProj = std::max(
      maxProj,
      proj
    );
  }

  std::unique_ptr<QgsPolygon> sidePoly(
    new QgsPolygon()
  );

  std::unique_ptr<QgsLineString> sideRing(
    new QgsLineString()
  );

  // 底部
  sideRing->addVertex(
    QgsPoint(
      minProj,
      0
    )
  );

  sideRing->addVertex(
    QgsPoint(
      maxProj,
      0
    )
  );

  // 顶部
  sideRing->addVertex(
    QgsPoint(
      maxProj,
      height
    )
  );

  sideRing->addVertex(
    QgsPoint(
      minProj,
      height
    )
  );

  // 闭合
  sideRing->addVertex(
    QgsPoint(
      minProj,
      0
    )
  );

  sidePoly->setExteriorRing(
    sideRing.release()
  );

  return QgsGeometry(
    std::move(
      sidePoly
    )
  );
}
