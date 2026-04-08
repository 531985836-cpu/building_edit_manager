#include "pointedit.h"
#include <qgsmapcanvas.h>
#include <qgsproject.h>
#include <qgsmapmouseevent.h>
#include <qgsfeatureiterator.h>
#include <qgsgeometry.h>
#include <qgsvectorlayer.h>
#include <QMessageBox>
#include <qgsvertexid.h>
#include <qgspoint.h>
#include <qgsabstractgeometry.h>

// =======================初始化与基础管理=====================================

PointEdit::PointEdit( QgsMapCanvas *canvas )
  : QgsMapTool( canvas )
{
  setCursor( Qt::CrossCursor );
  createRubberBand();

  auto layers = QgsProject::instance()->layers<QgsVectorLayer *>();
  if ( !layers.isEmpty() )
    mActiveLayer = layers.first();

  if ( mActiveLayer )
    startEditingLayer( mActiveLayer );
}

PointEdit::~PointEdit() {}

// 确保图层处于编辑状态
void PointEdit::startEditingLayer( QgsVectorLayer *layer )
{
  if ( layer && !layer->isEditable() )
    layer->startEditing();
}

// 初始化各种预览辅助线
void PointEdit::createRubberBand()
{
  if ( !mRubberBand )
  {
    mRubberBand = new QgsRubberBand( canvas() );
    mRubberBand->setWidth( 1 );
    mRubberBand->setColor( Qt::blue );
    mRubberBand->hide();
  }
  if ( !mTempRubber )
  {
    mTempRubber = new QgsRubberBand( canvas() );
    mTempRubber->setWidth( 1 );
    mTempRubber->setColor( Qt::blue );
    mTempRubber->setLineStyle( Qt::DashLine );
    mTempRubber->hide();
  }
  if ( !mDigitizeRubber )
  {
    mDigitizeRubber = new QgsRubberBand( canvas(), Qgis::GeometryType::Polygon );
    mDigitizeRubber->setColor( QColor( 0, 255, 0, 60 ) );
    mDigitizeRubber->setStrokeColor( Qt::green );
    mDigitizeRubber->setWidth( 2 );
    mDigitizeRubber->hide();
  }
}

// 重置辅助线
void PointEdit::clearRubberBand()
{
  if ( mRubberBand )
    mRubberBand->reset();
  if ( mTempRubber )
    mTempRubber->reset();
  if ( mDigitizeRubber )
    mDigitizeRubber->reset();
}

// 工具卸载时取消新建操作
void PointEdit::deactivate()
{
  cancelNewFace();
  QgsMapTool::deactivate();
}

// ========================事件分发逻辑====================================

// 鼠标按下
void PointEdit::canvasPressEvent( QgsMapMouseEvent *e )
{
  if ( !mActiveLayer )
    return;

  QgsPointXY pos = toMapCoordinates( e->pos() );
  double tol = 10.0 * canvas()->mapUnitsPerPixel();
  QgsFeatureIds selectedIds = mActiveLayer->selectedFeatureIds();

  // --- 处理右键逻辑 ---
  if ( e->button() == Qt::RightButton )
  {
    if ( mCurrentMode == DigitizeMode )
    {
      finishNewFace();
      return;
    }
  }
  if ( e->button() != Qt::LeftButton )
    return;

  // --- 处理正在进行的模式 ---
  if ( mCurrentMode == DigitizeMode )
  {
    addPointToNewFace( pos );
    return;
  }
  if ( mCurrentMode == VertexMode )
  {
    finishEditVertex( pos );
    mCurrentMode = NoneMode;
    mTempRubber->hide();
    return;
  }
  if ( mCurrentMode == EdgeMode )
  {
    finishEditEdge( pos );
    mCurrentMode = NoneMode;
    mTempRubber->hide();
    return;
  }
  if ( mCurrentMode == FaceMode )
  {
    finishFaceMove( pos );
    mCurrentMode = NoneMode;
    mTempRubber->hide();
    return;
  }

  // --- 核心改动：仅针对选中要素进行探测 ---
  if ( !selectedIds.isEmpty() )
  {
    // 1. 探测选中要素的节点
    if ( findClosestVertex( pos, mDraggingFeatureId, mDraggingVertexIndex, tol, selectedIds ) )
    {
      mCurrentMode = VertexMode;
      return;
    }
    // 2. 探测选中要素的边
    if ( findClosestEdge( pos, mEditingFeatureId, mEditingEdgeStartIndex, tol, selectedIds ) )
    {
      mCurrentMode = EdgeMode;
      return;
    }
    // 3. 探测选中要素的面（平移）
    QgsFeature feat;
    // 简单起见取第一个选中要素，或遍历 selectedIds 探测 pointInFeature
    mActiveLayer->getFeatures( QgsFeatureRequest( *selectedIds.begin() ) ).nextFeature( feat );
    if ( feat.geometry().contains( QgsGeometry::fromPointXY( pos ) ) )
    {
      mMovingFeatureId = feat.id();
      mInitialClickPoint = pos;
      mCurrentMode = FaceMode;
      getGeometryPoints( feat.geometry(), mOriginalFacePts );
      // ... (此处省略 rubberband 初始化代码)
      return;
    }
  }

  // --- 如果没点中任何选中要素的组件，则处理图层交互 ---
  QgsFeatureId foundId = pointInFeature( pos );
  if ( foundId == FID_NULL )
  {
    if ( selectedIds.isEmpty() )
    {
      mCurrentMode = DigitizeMode;
      addPointToNewFace( pos );
    }
    else
    {
      mActiveLayer->removeSelection();
      canvas()->refresh();
    }
  }
  else
  {
    // 如果点中了某个要素，但它之前没被选中，则选中它（这样下次点击就能编辑它了）
    selectAtPoint( pos, mShiftPressed );
  }
}

// 鼠标移动
void PointEdit::canvasMoveEvent( QgsMapMouseEvent *e )
{
  QgsPointXY mousePos = toMapCoordinates( e->pos() );

  if ( mCurrentMode == VertexMode && mDraggingFeatureId != -1 )
  {
    QgsFeature feat;
    mActiveLayer->getFeatures( QgsFeatureRequest( mDraggingFeatureId ) ).nextFeature( feat );
    QgsPointXY prev, next;
    if ( getAdjacentPoints( feat.geometry(), mDraggingVertexIndex, prev, next ) )
    {
      mTempRubber->reset();
      mTempRubber->addPoint( prev );
      mTempRubber->addPoint( mousePos );
      mTempRubber->addPoint( next );
      mTempRubber->show();
    }
  }
  else if ( mCurrentMode == EdgeMode && mEditingFeatureId != -1 )
  {
    QList<QgsPointXY> pts;
    QgsFeature feat;
    mActiveLayer->getFeatures( QgsFeatureRequest( mEditingFeatureId ) ).nextFeature( feat );
    getGeometryPoints( feat.geometry(), pts );
    int n = pts.size();
    QgsPointXY p0 = pts[mEditingEdgeStartIndex % n], p1 = pts[( mEditingEdgeStartIndex + 1 ) % n];
    QgsPointXY mid( ( p0.x() + p1.x() ) / 2, ( p0.y() + p1.y() ) / 2 );
    double dx = mousePos.x() - mid.x(), dy = mousePos.y() - mid.y();

    mTempRubber->reset();
    for ( int i = 0; i < n; ++i )
    {
      if ( i == ( mEditingEdgeStartIndex % n ) || i == ( ( mEditingEdgeStartIndex + 1 ) % n ) )
        mTempRubber->addPoint( QgsPointXY( pts[i].x() + dx, pts[i].y() + dy ) );
      else
        mTempRubber->addPoint( pts[i] );
    }
    if ( mTempRubber->numberOfVertices() > 0 )
      mTempRubber->addPoint( *( mTempRubber->getPoint( 0, 0 ) ) );
    mTempRubber->show();
  }
  else if ( mCurrentMode == FaceMode && mMovingFeatureId != -1 )
  {
    double dx = mousePos.x() - mInitialClickPoint.x(), dy = mousePos.y() - mInitialClickPoint.y();
    mTempRubber->reset();
    for ( const auto &pt : mOriginalFacePts )
      mTempRubber->addPoint( QgsPointXY( pt.x() + dx, pt.y() + dy ) );
    if ( mTempRubber->numberOfVertices() > 0 )
      mTempRubber->addPoint( *( mTempRubber->getPoint( 0, 0 ) ) );
    mTempRubber->show();
  }
  else if ( mCurrentMode == DigitizeMode && !mNewFacePoints.isEmpty() )
  {
    mDigitizeRubber->removeLastPoint();
    mDigitizeRubber->addPoint( mousePos );
  }
}

// 双击事件
void PointEdit::canvasDoubleClickEvent( QgsMapMouseEvent *e )
{
  mCurrentMode = NoneMode;
  mTempRubber->hide();
  QgsPointXY pos = toMapCoordinates( e->pos() );

  if ( deleteVertexAt( pos ) )
    return;                   // 优先尝试删点
  addVertexToSelected( pos ); // 尝试加点
  if ( deleteFaceAt( pos ) )
    return; // 最后尝试删面
}

// 键盘事件
void PointEdit::keyPressEvent( QKeyEvent *e )
{
  // 1. Esc 键取消当前预览/新建状态
  if ( e->key() == Qt::Key_Escape )
  {
    cancelNewFace();
    mCurrentMode = NoneMode;
    mTempRubber->hide();
  }
  // 2. 回车键执行全局保存逻辑 (不再处理 DigitizeMode 的完成)
  else if ( e->key() == Qt::Key_Return || e->key() == Qt::Key_Enter )
  {
    if ( mActiveLayer && mActiveLayer->isEditable() )
    {
      saveAllEdits();
    }
  }
  // 3. Undo 撤销
  else if ( e->matches( QKeySequence::Undo ) && mActiveLayer->isEditable() )
  {
    mActiveLayer->undoStack()->undo();
  }

  QgsMapTool::keyPressEvent( e );
}

void PointEdit::canvasReleaseEvent( QgsMapMouseEvent *e ) { Q_UNUSED( e ); }

// =========================核心功能实现===================================

// 向新建要素列表中添加点
void PointEdit::addPointToNewFace( const QgsPointXY &pt )
{
  mNewFacePoints.append( pt );
  mDigitizeRubber->reset( Qgis::GeometryType::Polygon );
  for ( const auto &p : mNewFacePoints )
    mDigitizeRubber->addPoint( p );
  mDigitizeRubber->addPoint( pt );
  mDigitizeRubber->show();
}

// 提交新建的面要素到图层
void PointEdit::finishNewFace()
{
  if ( mNewFacePoints.size() >= 3 )
  {
    mActiveLayer->beginEditCommand( "Create Building" );
    QgsFeature f( mActiveLayer->fields() );
    QgsPolygonXY poly;
    poly.append( QVector<QgsPointXY>::fromList( mNewFacePoints ) );
    f.setGeometry( QgsGeometry::fromPolygonXY( poly ) );
    mActiveLayer->addFeature( f );
    mActiveLayer->endEditCommand();
    mActiveLayer->triggerRepaint();
  }
  cancelNewFace();
}

// 取消当前新建面操作
void PointEdit::cancelNewFace()
{
  mNewFacePoints.clear();
  mDigitizeRubber->reset();
  mDigitizeRubber->hide();
  mCurrentMode = NoneMode;
}

// 提交节点移动结果
void PointEdit::finishEditVertex( const QgsPointXY &newPos )
{
  QgsFeature feat;
  mActiveLayer->getFeatures( QgsFeatureRequest( mDraggingFeatureId ) ).nextFeature( feat );
  mActiveLayer->beginEditCommand( tr( "Move vertex" ) );
  QList<QgsPointXY> pts;
  getGeometryPoints( feat.geometry(), pts );
  if ( mDraggingVertexIndex == 0 || mDraggingVertexIndex == pts.size() )
    pts[0] = newPos;
  else
    pts[mDraggingVertexIndex] = newPos;
  updateFeatureGeometry( feat, pts );
  mActiveLayer->updateFeature( feat );
  mActiveLayer->endEditCommand();
  mActiveLayer->triggerRepaint();
}

// 提交边线平移结果
void PointEdit::finishEditEdge( const QgsPointXY &newPos )
{
  QgsFeature feat;
  mActiveLayer->getFeatures( QgsFeatureRequest( mEditingFeatureId ) ).nextFeature( feat );
  QList<QgsPointXY> pts;
  getGeometryPoints( feat.geometry(), pts );
  int n = pts.size();
  QgsPointXY p0 = pts[mEditingEdgeStartIndex % n], p1 = pts[( mEditingEdgeStartIndex + 1 ) % n];
  QgsPointXY mid( ( p0.x() + p1.x() ) / 2, ( p0.y() + p1.y() ) / 2 );
  double dx = newPos.x() - mid.x(), dy = newPos.y() - mid.y();
  pts[mEditingEdgeStartIndex % n] = QgsPointXY( p0.x() + dx, p0.y() + dy );
  pts[( mEditingEdgeStartIndex + 1 ) % n] = QgsPointXY( p1.x() + dx, p1.y() + dy );
  mActiveLayer->beginEditCommand( "Move edge" );
  updateFeatureGeometry( feat, pts );
  mActiveLayer->updateFeature( feat );
  mActiveLayer->endEditCommand();
  mActiveLayer->triggerRepaint();
}

// 提交整个面要素平移结果
void PointEdit::finishFaceMove( const QgsPointXY &newPos )
{
  QgsFeature feat;
  mActiveLayer->getFeatures( QgsFeatureRequest( mMovingFeatureId ) ).nextFeature( feat );
  double dx = newPos.x() - mInitialClickPoint.x(), dy = newPos.y() - mInitialClickPoint.y();
  QList<QgsPointXY> newPts;
  for ( const auto &pt : mOriginalFacePts )
    newPts.append( QgsPointXY( pt.x() + dx, pt.y() + dy ) );
  mActiveLayer->beginEditCommand( "Move face" );
  updateFeatureGeometry( feat, newPts );
  mActiveLayer->updateFeature( feat );
  mActiveLayer->endEditCommand();
  mActiveLayer->triggerRepaint();
}

// 删除指定位置的节点
bool PointEdit::deleteVertexAt( const QgsPointXY &pt )
{
  if ( mActiveLayer->selectedFeatureCount() == 0 )
    return false;
  QgsFeature feat;
  mActiveLayer->getFeatures( QgsFeatureRequest( *mActiveLayer->selectedFeatureIds().begin() ) ).nextFeature( feat );
  QgsGeometry geom = feat.geometry();
  int atV = -1, bef = -1, aft = -1;
  double sqDist = -1;
  geom.closestVertex( pt, atV, bef, aft, sqDist );
  if ( atV != -1 && sqDist < pow( 10.0 * canvas()->mapUnitsPerPixel(), 2 ) )
  {
    if ( geom.constGet()->vertexCount() <= 4 )
      return false;
    mActiveLayer->beginEditCommand( "Delete vertex" );
    if ( geom.deleteVertex( atV ) )
    {
      feat.setGeometry( geom );
      mActiveLayer->updateFeature( feat );
      mActiveLayer->endEditCommand();
      mActiveLayer->triggerRepaint();
      return true;
    }
    mActiveLayer->destroyEditCommand();
  }
  return false;
}

// 删除指定位置的面要素（带边缘保护）
bool PointEdit::deleteFaceAt( const QgsPointXY &pt )
{
  if ( !mActiveLayer || mActiveLayer->selectedFeatureCount() == 0 )
    return false;

  QgsFeatureId fid = pointInFeature( pt );
  if ( fid != FID_NULL && mActiveLayer->selectedFeatureIds().contains( fid ) )
  {
    QgsFeature feat;
    mActiveLayer->getFeatures( QgsFeatureRequest( fid ) ).nextFeature( feat );
    QgsPoint snapPoint;
    QgsVertexId vId;
    int leftOf = 0;
    double distToEdge = feat.geometry().constGet()->closestSegment( QgsPoint( pt ), snapPoint, vId, &leftOf, -1.0 );
    double safeThreshold = 20.0 * canvas()->mapUnitsPerPixel();

    if ( distToEdge > safeThreshold )
    {
      int res = QMessageBox::question( canvas(), tr( "确认删除" ), tr( "确定要删除选中的整个要素吗？" ), QMessageBox::Yes | QMessageBox::No );
      if ( res == QMessageBox::Yes )
      {
        mActiveLayer->beginEditCommand( tr( "Delete Face" ) );
        mActiveLayer->deleteFeature( fid );
        mActiveLayer->endEditCommand();
        mActiveLayer->triggerRepaint();
        return true;
      }
    }
  }
  return false;
}

// 在选中要素的边线上增加节点
bool PointEdit::addVertexToSelected( const QgsPointXY &pt )
{
  if ( mActiveLayer->selectedFeatureCount() == 0 )
    return false;
  QgsFeature feat;
  mActiveLayer->getFeatures( QgsFeatureRequest( *mActiveLayer->selectedFeatureIds().begin() ) ).nextFeature( feat );
  QgsGeometry geom = feat.geometry();
  QgsPoint snap;
  QgsVertexId vId;
  int left;
  double tol = 10.0 * canvas()->mapUnitsPerPixel();
  if ( geom.constGet()->closestSegment( QgsPoint( pt ), snap, vId, &left, tol ) <= tol )
  {
    mActiveLayer->beginEditCommand( "Add vertex" );
    QList<QgsPointXY> pts;
    getGeometryPoints( geom, pts );
    pts.insert( vId.vertex, pt );
    updateFeatureGeometry( feat, pts );
    mActiveLayer->updateFeature( feat );
    mActiveLayer->endEditCommand();
    mActiveLayer->triggerRepaint();
    return true;
  }
  return false;
}

// 回车保存编辑内容
void PointEdit::saveAllEdits()
{
  if ( !mActiveLayer )
    return;

  // 弹出确认对话框
  QMessageBox::StandardButton res = QMessageBox::question(
    canvas(),
    tr( "保存编辑" ),
    tr( "确定要提交对图层 [%1] 的所有编辑改动吗？" ).arg( mActiveLayer->name() ),
    QMessageBox::Yes | QMessageBox::No
  );

  if ( res == QMessageBox::Yes )
  {
    // commitChanges 会将内存中的编辑真正写入数据源
    if ( mActiveLayer->commitChanges() )
    {
      QMessageBox::information( canvas(), tr( "保存成功" ), tr( "编辑已成功保存并提交。" ) );
      // 提交后图层会自动退出编辑模式，这里手动重新开启，方便用户继续编辑
      mActiveLayer->startEditing();
    }
    else
    {
      QMessageBox::critical( canvas(), tr( "保存失败" ), tr( "无法保存编辑内容，错误详情：\n%1" ).arg( mActiveLayer->commitErrors().join( "\n" ) ) );
    }
  }
}

// =======================辅助工具函数=====================================
// 执行矩形范围内的点选操作
void PointEdit::selectAtPoint( const QgsPointXY &pt, bool add )
{
  double r = 5.0 * canvas()->mapUnitsPerPixel();
  QgsRectangle rect( pt.x() - r, pt.y() - r, pt.x() + r, pt.y() + r );
  QgsFeatureIds ids = add ? mActiveLayer->selectedFeatureIds() : QgsFeatureIds();
  QgsFeatureIterator it( mActiveLayer->getFeatures( QgsFeatureRequest( rect ) ) );
  QgsFeature feat;
  while ( it.nextFeature( feat ) )
    ids.insert( feat.id() );
  mActiveLayer->selectByIds( ids );
  canvas()->refresh();
}

// 探测指定坐标下的要素ID
QgsFeatureId PointEdit::pointInFeature( const QgsPointXY &pt )
{
  double r = 5.0 * canvas()->mapUnitsPerPixel();
  QgsFeatureRequest req;
  req.setFilterRect( QgsRectangle( pt.x() - r, pt.y() - r, pt.x() + r, pt.y() + r ) ).setLimit( 1 );
  QgsFeatureIterator it = mActiveLayer->getFeatures( req );
  QgsFeature f;
  return it.nextFeature( f ) ? f.id() : FID_NULL;
}

// 查找最近的节点
bool PointEdit::findClosestVertex( const QgsPointXY &pt, QgsFeatureId &fid, int &vertexIndex, double tolerance, const QgsFeatureIds &targetIds )
{
  if ( targetIds.isEmpty() )
    return false; // 如果没选中，直接返回

  double minDist2 = tolerance * tolerance;
  bool found = false;

  // 使用 QgsFeatureRequest 过滤，只获取已选中的要素
  QgsFeatureIterator it = mActiveLayer->getFeatures( QgsFeatureRequest().setFilterFids( targetIds ) );
  QgsFeature feat;
  while ( it.nextFeature( feat ) )
  {
    QgsGeometry geom = feat.geometry();
    int idx = 0;
    for ( auto vIt = geom.vertices_begin(); vIt != geom.vertices_end(); ++vIt, ++idx )
    {
      double d2 = pt.sqrDist( *vIt );
      if ( d2 <= minDist2 )
      {
        minDist2 = d2;
        fid = feat.id();
        vertexIndex = idx;
        found = true;
      }
    }
  }
  return found;
}

// 查找最近的边
bool PointEdit::findClosestEdge( const QgsPointXY &pt, QgsFeatureId &fid, int &startVertexIndex, double tolerance, const QgsFeatureIds &targetIds )
{
  if ( targetIds.isEmpty() )
    return false;

  QgsFeatureIterator it = mActiveLayer->getFeatures( QgsFeatureRequest().setFilterFids( targetIds ) );
  QgsFeature feat;
  while ( it.nextFeature( feat ) )
  {
    QList<QgsPointXY> pts;
    if ( !getGeometryPoints( feat.geometry(), pts ) )
      continue;
    for ( int i = 0; i < pts.size(); ++i )
    {
      int j = ( i + 1 ) % pts.size();
      QgsGeometry edge = QgsGeometry::fromPolylineXY( { pts[i], pts[j] } );
      if ( edge.distance( QgsGeometry::fromPointXY( pt ) ) < tolerance )
      {
        fid = feat.id();
        startVertexIndex = i;
        return true;
      }
    }
  }
  return false;
}

// 将几何对象转换为点列表（处理闭合环）
bool PointEdit::getGeometryPoints( const QgsGeometry &geom, QList<QgsPointXY> &pts )
{
  pts.clear();
  if ( geom.isEmpty() )
    return false;
  QVector<QgsPointXY> raw;
  if ( geom.isMultipart() )
    raw = geom.asMultiPolygon()[0][0];
  else
    raw = geom.asPolygon()[0];
  if ( !raw.isEmpty() && raw.first() == raw.last() )
    raw.removeLast();
  pts = raw.toList();
  return !pts.isEmpty();
}

// 将点列表写回几何对象并更新要素
void PointEdit::updateFeatureGeometry( QgsFeature &feat, const QList<QgsPointXY> &pts )
{
  QVector<QgsPointXY> v = QVector<QgsPointXY>::fromList( pts );
  if ( v.isEmpty() )
    return;
  if ( v.first() != v.last() )
    v.append( v.first() );
  QgsPolygonXY p;
  p.append( v );
  feat.setGeometry( QgsGeometry::fromPolygonXY( p ) );
}

// 获取指定节点的前后相邻点
bool PointEdit::getAdjacentPoints( const QgsGeometry &geom, int vertexIndex, QgsPointXY &prevPt, QgsPointXY &nextPt )
{
  QList<QgsPointXY> pts;
  if ( !getGeometryPoints( geom, pts ) )
    return false;
  int n = pts.size();
  if ( n < 3 )
    return false;
  prevPt = pts[( vertexIndex - 1 + n ) % n];
  nextPt = pts[( vertexIndex + 1 ) % n];
  return true;
}
