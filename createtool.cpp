#include "createtool.h"

#include <qgsmapcanvas.h>
#include <qgsproject.h>
#include <qgsmapmouseevent.h>
#include <qgsgeometry.h>
#include <qgsfeature.h>
#include <qgsvectorlayer.h>
#include <qgspointcloudlayer.h>
#include <QVariant>
#include <QDebug>
#include <QMessageBox>
#include <qgspointcloudindex.h>
#include <qgspointcloudblock.h>
#include <qgspointcloudattribute.h>
#include <qgspointcloudrequest.h>
#include <qgspoint.h>
#include <memory>
#include <qgsgeometryengine.h>
#include <qgsrubberband.h>
#include <numeric>
#include <qgsmessagelog.h>
#include <cmath>
#include <QList>
#include <QgsGeometryUtils.h>
#include <qgis.h>
#include <qgslayertree.h>
// ==================== 构造 / 析构 ====================

CreateTool::CreateTool( QgsMapCanvas *canvas )
  : QgsMapTool( canvas )
{
  setCursor( Qt::CrossCursor );
  mRubberBand = new QgsRubberBand( canvas, Qgis::GeometryType::Polygon );

  // 初始化分割预览线（橙色虚线）
  mSplitLineBand = new QgsRubberBand( canvas, Qgis::GeometryType::Line );
  mSplitLineBand->setLineStyle( Qt::DashLine );
  mSplitLineBand->setColor( QColor( 255, 165, 0 ) );
  mSplitLineBand->setWidth( 2 );

  mSettingsWidget = nullptr;
}

CreateTool::~CreateTool()
{
}

// ==================== 激活 / 停用 ====================

void CreateTool::activate()
{
  QgsMapTool::activate();

  if ( !mSettingsWidget )
    setupUi();

  refreshLayerCombos();

  if ( mSettingsWidget )
  {
    mSettingsWidget->show();
    mSettingsWidget->raise();
    mSettingsWidget->activateWindow();
  }
}

void CreateTool::deactivate()
{
  if ( mSplitLineBand )
    mSplitLineBand->reset( Qgis::GeometryType::Line );
  mIsSplitting = false;
  clearDebugMarkers();
  cancelDigitizing();
  QgsMapTool::deactivate();
}

// ==================== UI 初始化与配置 ====================

void CreateTool::setupUi()
{
  mSettingsWidget = new QWidget();
  mUI.setupUi( mSettingsWidget );
  mSettingsWidget->setWindowTitle( "设置" );

  // 初始化控件状态（根据 CheckBox 初始状态）
  updateWidgetInteractivity( mUI.heightcheckBox->isChecked(), mUI.mergecheckBox->isChecked() );

  // 高度模块联动
  connect( mUI.heightcheckBox, &QCheckBox::toggled, this, [this]( bool checked ) {
    mUI.fieldcombo->setEnabled( checked );
    mUI.pointcloudcombo->setEnabled( checked );
    mUI.field->setEnabled( checked );
    mUI.pointcloud->setEnabled( checked );
  } );

  // 融合模块联动
  connect( mUI.mergecheckBox, &QCheckBox::toggled, this, [this]( bool checked ) {
    mUI.mergelineEdit->setEnabled( checked );
    mUI.merge->setEnabled( checked );
  } );

  connect( mUI.vectorcombo, QOverload<int>::of( &QComboBox::currentIndexChanged ), this, &CreateTool::updateFields );

  // 确认按钮：保存设置并验证
  connect( mUI.setting, &QPushButton::clicked, this, [this]() {
    if ( !mUI.vectorcombo )
      return;

    QString vId = mUI.vectorcombo->currentData().toString();
    mVectorLayer = qobject_cast<QgsVectorLayer *>( QgsProject::instance()->mapLayer( vId ) );

    if ( !mVectorLayer )
    {
      QMessageBox::warning( mSettingsWidget, "错误", "请选择有效的矢量图层！" );
      return;
    }

    QString info = QString( "<b>基础配置：</b><br>目标图层：%1<br><br>" ).arg( mVectorLayer->name() );

    // 高度初始化模块
    if ( mUI.heightcheckBox->isChecked() )
    {
      QString pId = mUI.pointcloudcombo->currentData().toString();
      mPCLayer = qobject_cast<QgsPointCloudLayer *>( QgsProject::instance()->mapLayer( pId ) );
      mTargetFieldName = mUI.fieldcombo->currentText();

      if ( mPCLayer )
      {
        info += QString( "<font color='green'>● 高度初始化：已开启</font><br>"
                         "&nbsp;&nbsp;保存字段：%1<br>"
                         "&nbsp;&nbsp;点云源：%2<br><br>" )
                  .arg( mTargetFieldName )
                  .arg( mPCLayer->name() );
      }
      else
      {
        QMessageBox::warning( mSettingsWidget, "设置不完整", "已开启高度功能但未选择有效的点云图层！" );
        return;
      }
    }
    else
    {
      info += "<font color='gray'>○ 高度初始化：未开启</font><br><br>";
      mPCLayer = nullptr;
    }

    // 融合设置模块
    if ( mUI.mergecheckBox->isChecked() )
    {
      QString dist = mUI.mergelineEdit->text();
      if ( dist.isEmpty() )
        dist = "0 (未设置)";

      info += QString( "<font color='green'>● 融合功能：已开启</font><br>"
                       "&nbsp;&nbsp;融合距离：%1<br>" )
                .arg( dist );
    }
    else
    {
      info += "<font color='gray'>○ 融合功能：未开启</font><br>";
    }

    if ( !mVectorLayer->isEditable() )
      mVectorLayer->startEditing();

    QMessageBox::information( mSettingsWidget, "配置确认", info );
    mSettingsWidget->hide();
  } );

  if ( mUI.vectorcombo->count() > 0 )
    updateFields( 0 );
}

void CreateTool::updateWidgetInteractivity( bool heightEnabled, bool mergeEnabled )
{
  mUI.fieldcombo->setEnabled( heightEnabled );
  mUI.pointcloudcombo->setEnabled( heightEnabled );
  mUI.field->setEnabled( heightEnabled );
  mUI.pointcloud->setEnabled( heightEnabled );

  mUI.mergelineEdit->setEnabled( mergeEnabled );
  mUI.merge->setEnabled( mergeEnabled );
}

void CreateTool::refreshLayerCombos()
{
  if ( !mUI.vectorcombo || !mUI.pointcloudcombo )
    return;

  mUI.vectorcombo->blockSignals( true );
  mUI.pointcloudcombo->blockSignals( true );

  mUI.vectorcombo->clear();
  mUI.pointcloudcombo->clear();

  // 添加空选项，强制用户手动选择
  mUI.vectorcombo->addItem( "请选择矢量图层...", QVariant() );
  mUI.pointcloudcombo->addItem( "请选择点云图层...", QVariant() );

  auto layers = QgsProject::instance()->mapLayers().values();
  for ( QgsMapLayer *layer : layers )
  {
    if ( layer->type() == Qgis::LayerType::Vector )
      mUI.vectorcombo->addItem( layer->name(), layer->id() );
    else if ( layer->type() == Qgis::LayerType::PointCloud )
      mUI.pointcloudcombo->addItem( layer->name(), layer->id() );
  }

  mUI.vectorcombo->setCurrentIndex( 0 );
  mUI.pointcloudcombo->setCurrentIndex( 0 );

  mUI.vectorcombo->blockSignals( false );
  mUI.pointcloudcombo->blockSignals( false );

  mUI.fieldcombo->clear();
}

void CreateTool::updateFields( int index )
{
  if ( !mUI.fieldcombo || !mUI.vectorcombo )
    return;

  mUI.fieldcombo->clear();

  if ( index <= 0 )
    return;

  QString layerId = mUI.vectorcombo->currentData().toString();
  QgsVectorLayer *vLayer = qobject_cast<QgsVectorLayer *>( QgsProject::instance()->mapLayer( layerId ) );

  if ( vLayer )
  {
    mUI.fieldcombo->addItem( "请选择字段..." );
    for ( const QgsField &f : vLayer->fields() )
      mUI.fieldcombo->addItem( f.name() );
  }
}

// ==================== 地图交互事件 ====================

void CreateTool::canvasPressEvent( QgsMapMouseEvent *e )
{
  if ( !mCanvas || !mVectorLayer )
    return;

  QgsPointXY mapPt = toMapCoordinates( e->pos() );

  double splitTol = mCanvas->mapUnitsPerPixel() * 3.0;
  double sqrSplitTol = splitTol * splitTol;
  double selectTol = mCanvas->mapUnitsPerPixel() * 10.0;

  QgsFeatureIds selectedIds = mVectorLayer->selectedFeatureIds();

  // ==================== 1. 正在进行的模式处理（状态锁） ====================

  if ( mIsSplitting )
  {
    if ( e->button() == Qt::LeftButton )
    {
      QgsFeature targetFeat = mVectorLayer->getFeature( mTargetFeatureId );
      if ( targetFeat.isValid() )
      {
        QgsPointXY snapEnd;
        int nextV = 0;
        int lOf = 0;
        targetFeat.geometry().closestSegmentWithContext( mapPt, snapEnd, nextV, &lOf );
        performSplit( snapEnd );
      }
    }
    mIsSplitting = false;
    if ( mSplitLineBand )
      mSplitLineBand->reset( Qgis::GeometryType::Line );
    return;
  }

  if ( mIsDigitizing )
  {
    if ( e->button() == Qt::LeftButton )
    {
      mPoints.append( mapPt );
      mRubberBand->addPoint( mapPt );
    }
    else if ( e->button() == Qt::RightButton )
    {
      finishCurrentFeatureWithHeight();
      mIsDigitizing = false;
    }
    return;
  }

  // ==================== 2. 核心判定：分割起点捕捉（仅针对已选中的要素） ====================
  if ( e->button() == Qt::LeftButton && !selectedIds.isEmpty() )
  {
    QgsRectangle searchRect( mapPt.x() - splitTol, mapPt.y() - splitTol, mapPt.x() + splitTol, mapPt.y() + splitTol );

    QgsFeatureIterator selIt = mVectorLayer->getFeatures(
      QgsFeatureRequest().setFilterRect( searchRect ).setFilterFids( selectedIds )
    );

    QgsFeature selFeat;
    while ( selIt.nextFeature( selFeat ) )
    {
      QgsGeometry geom = selFeat.geometry();
      QgsPointXY snapPoint;
      int nextVertexIndex = 0;
      int leftOf = 0;

      double sqrDist = geom.closestSegmentWithContext( mapPt, snapPoint, nextVertexIndex, &leftOf );

      if ( sqrDist >= 0 && sqrDist < sqrSplitTol )
      {
        mTargetFeatureId = selFeat.id();
        mSplitStartPoint = snapPoint;
        mIsSplitting = true;

        if ( mSplitLineBand )
        {
          mSplitLineBand->reset( Qgis::GeometryType::Line );
          mSplitLineBand->addPoint( mSplitStartPoint );
          mSplitLineBand->addPoint( mSplitStartPoint );
        }
        return;
      }
    }
  }

  // ==================== 3. 图层级交互：面选择、自动融合 ====================
  if ( e->button() == Qt::LeftButton )
  {
    QgsRectangle selectRect( mapPt.x() - selectTol, mapPt.y() - selectTol, mapPt.x() + selectTol, mapPt.y() + selectTol );

    QgsFeatureIterator it = mVectorLayer->getFeatures( QgsFeatureRequest().setFilterRect( selectRect ) );
    QgsFeature feat;
    bool clickedOnFace = false;

    while ( it.nextFeature( feat ) )
    {
      if ( feat.geometry().contains( QgsGeometry::fromPointXY( mapPt ) ) )
      {
        clickedOnFace = true;
        bool shiftPressed = e->modifiers() & Qt::ShiftModifier;

        if ( shiftPressed )
        {
          if ( selectedIds.contains( feat.id() ) )
            mVectorLayer->deselect( feat.id() );
          else
            mVectorLayer->select( feat.id() );
        }
        else
        {
          if ( !selectedIds.contains( feat.id() ) )
          {
            mVectorLayer->removeSelection();
            mVectorLayer->select( feat.id() );
          }
        }

        if ( mVectorLayer->selectedFeatureCount() == 2 )
        {
          snapTwoSelectedFeatures();
        }
        break;
      }
    }

    // ==================== 4. 空白区域操作（新建多边形或清空选择） ====================
    if ( !clickedOnFace )
    {
      if ( !selectedIds.isEmpty() )
      {
        mVectorLayer->removeSelection();
      }
      else
      {
        mIsDigitizing = true;
        mPoints.clear();
        mPoints.append( mapPt );
        mRubberBand->reset( Qgis::GeometryType::Polygon );
        mRubberBand->addPoint( mapPt );
      }
    }
    mCanvas->refresh();
  }
}

void CreateTool::canvasMoveEvent( QgsMapMouseEvent *e )
{
  QgsPointXY mapPt = toMapCoordinates( e->pos() );

  // 分割辅助线预览
  if ( mIsSplitting && mSplitLineBand )
  {
    if ( mSplitLineBand->numberOfVertices() > 1 )
      mSplitLineBand->removeLastPoint();
    mSplitLineBand->addPoint( mapPt );
    return;
  }

  // 数字化预览
  if ( !mIsDigitizing || mPoints.isEmpty() )
    return;

  if ( mRubberBand->numberOfVertices() > mPoints.size() )
    mRubberBand->removeLastPoint();

  mRubberBand->addPoint( mapPt );
}

void CreateTool::keyPressEvent( QKeyEvent *e )
{
  if ( e->key() == Qt::Key_Escape )
  {
    cancelDigitizing();
    if ( mVectorLayer )
      mVectorLayer->removeSelection();
  }
  else if ( e->key() == Qt::Key_Delete || e->key() == Qt::Key_Backspace )
  {
    if ( mVectorLayer && mVectorLayer->isEditable() && mVectorLayer->selectedFeatureCount() > 0 )
    {
      if ( QMessageBox::question( mCanvas, "确认删除", QString( "确定要删除选中的 %1 个要素吗？" ).arg( mVectorLayer->selectedFeatureCount() ) ) == QMessageBox::Yes )
      {
        mVectorLayer->deleteSelectedFeatures();
        mVectorLayer->triggerRepaint();
      }
    }
  }
  else if ( e->key() == Qt::Key_Return || e->key() == Qt::Key_Enter )
  {
    if ( mVectorLayer && mVectorLayer->isEditable() )
    {
      mVectorLayer->commitChanges();
      mVectorLayer->startEditing();
      qDebug() << "Changes committed.";
    }
  }

  QgsMapTool::keyPressEvent( e );
}

// ==================== 核心功能：高度计算 ====================

double CreateTool::calculateZFromPointCloud( const QgsGeometry &geom )
{
  if ( geom.isNull() || geom.isEmpty() || !geom.isGeosValid() )
    return 0.0;
  if ( !mPCLayer || !mPCLayer->dataProvider() )
    return 0.0;

  clearDebugMarkers();

  // 准备点云数据
  QgsPointCloudIndex index = mPCLayer->dataProvider()->index();
  QgsRectangle extent = geom.boundingBox();
  QList<QgsPointCloudNodeId> nodeIds;
  collectNodes( index, index.root(), extent, nodeIds );
  nodeIds = nodeIds.toSet().toList();

  double xSc = index.scale().x(), ySc = index.scale().y(), zSc = index.scale().z();
  double xOff = index.offset().x(), yOff = index.offset().y(), zOff = index.offset().z();

  QgsPointCloudRequest request;
  request.setFilterRect( extent );
  const QgsPointCloudAttributeCollection attributes = index.attributes();
  request.setAttributes( attributes );
  int recordSize = attributes.pointRecordSize();

  QList<QVector3D> allPoints;
  double minZ = std::numeric_limits<double>::max();
  std::unique_ptr<QgsGeometryEngine> engine( QgsGeometry::createGeometryEngine( geom.constGet() ) );
  if ( engine )
    engine->prepareGeometry();

  for ( const QgsPointCloudNodeId &nodeId : nodeIds )
  {
    std::unique_ptr<QgsPointCloudBlock> block( index.nodeData( nodeId, request ) );
    if ( !block )
      continue;
    const char *dataPtr = block->data();
    for ( int i = 0; i < block->pointCount(); ++i )
    {
      const char *ptr = dataPtr + ( i * recordSize );
      int32_t ix, iy, iz;
      std::memcpy( &ix, ptr, 4 );
      std::memcpy( &iy, ptr + 4, 4 );
      std::memcpy( &iz, ptr + 8, 4 );
      double x = ( ix * xSc ) + xOff, y = ( iy * ySc ) + yOff, z = ( iz * zSc ) + zOff;
      QgsPoint p( x, y );
      if ( engine && engine->contains( &p ) )
      {
        allPoints.append( QVector3D( x, y, z ) );
        if ( z < minZ )
          minZ = z;
      }
    }
  }

  if ( allPoints.isEmpty() )
    return 0.0;

  // 调试开关（法线碎线）
  bool showDebug = false;
  QgsVectorLayer *debugLayer = nullptr;
  int samplingCounter = 0;

  if ( showDebug )
  {
    debugLayer = getOrCreateDebugLayer();
    if ( debugLayer )
      debugLayer->startEditing();
  }

  QList<double> roofZValues;
  const double nzThreshold = 0.90;
  const int minNeighbors = 5;

  // 计算法线并提取屋顶点高度
  for ( const QVector3D &currentPoint : allPoints )
  {
    if ( currentPoint.z() < minZ + 2.5 )
      continue;

    QList<QVector3D> neighbors;
    for ( const QVector3D &other : allPoints )
    {
      float dx = currentPoint.x() - other.x(), dy = currentPoint.y() - other.y();
      if ( ( dx * dx + dy * dy ) < 1.0 )
        neighbors.append( other );
      if ( neighbors.size() > 15 )
        break;
    }

    if ( neighbors.size() >= minNeighbors )
    {
      double sumX = 0, sumY = 0, sumZ = 0;
      for ( const auto &n : neighbors )
      {
        sumX += n.x();
        sumY += n.y();
        sumZ += n.z();
      }
      double mX = sumX / neighbors.size(), mY = sumY / neighbors.size(), mZ = sumZ / neighbors.size();
      double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
      for ( const auto &n : neighbors )
      {
        double dx = n.x() - mX, dy = n.y() - mY, dz = n.z() - mZ;
        xx += dx * dx;
        xy += dx * dy;
        xz += dx * dz;
        yy += dy * dy;
        yz += dy * dz;
        zz += dz * dz;
      }
      QVector3D normal = computeNormal( xx, xy, xz, yy, yz, zz );

      if ( std::abs( normal.z() ) > nzThreshold )
      {
        roofZValues.append( currentPoint.z() );

        if ( showDebug && debugLayer )
        {
          samplingCounter++;
          if ( samplingCounter % 20 == 0 )
          {
            QgsPoint pStart( currentPoint.x(), currentPoint.y(), currentPoint.z() );
            double len = 5.0;
            QgsPoint pEnd( currentPoint.x() + normal.x() * len, currentPoint.y() + normal.y() * len, currentPoint.z() + normal.z() * len );
            QgsLineString *line = new QgsLineString();
            line->setPoints( QgsPointSequence() << pStart << pEnd );
            QgsFeature f( debugLayer->fields() );
            f.setGeometry( QgsGeometry( line ) );
            debugLayer->addFeature( f );
          }
        }
      }
    }
  }

  if ( showDebug && debugLayer )
  {
    debugLayer->commitChanges();
    debugLayer->triggerRepaint();
  }

  if ( roofZValues.isEmpty() )
    return 0.0;
  return std::accumulate( roofZValues.begin(), roofZValues.end(), 0.0 ) / roofZValues.size();
}

// ==================== 辅助函数：点云节点收集 ====================

void CreateTool::collectNodes( const QgsPointCloudIndex &index, const QgsPointCloudNodeId &nodeId, const QgsRectangle &extent, QList<QgsPointCloudNodeId> &nodes )
{
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

// ==================== 辅助函数：法线计算 ====================

QVector3D CreateTool::computeNormal( double xx, double xy, double xz, double yy, double yz, double zz )
{
  double detX = yy * zz - yz * yz;
  double detY = xx * zz - xz * xz;
  double detZ = xx * yy - xy * xy;

  double maxDet = std::max( { detX, detY, detZ } );

  if ( maxDet <= 0 )
    return QVector3D( 0, 0, 1 );

  QVector3D normal;
  if ( maxDet == detX )
    normal = QVector3D( detX, xz * yz - xy * zz, xy * yz - xz * yy );
  else if ( maxDet == detY )
    normal = QVector3D( xz * yz - xy * zz, detY, xy * xz - yz * xx );
  else
    normal = QVector3D( xy * yz - xz * yy, xy * xz - yz * xx, detZ );

  return normal.normalized();
}

// ==================== 调试辅助：法线可视化图层 ====================

QgsVectorLayer *CreateTool::getOrCreateDebugLayer()
{
  QString layerName = "Normal_Lines_Debug";
  QList<QgsMapLayer *> layers = QgsProject::instance()->mapLayersByName( layerName );
  if ( !layers.isEmpty() )
    return qobject_cast<QgsVectorLayer *>( layers.at( 0 ) );

  QString uri = QString( "LineString?crs=%1&field=id:int&z=yes" )
                  .arg( mCanvas->mapSettings().destinationCrs().authid() );

  mDebugLayer = new QgsVectorLayer( uri, layerName, "memory" );
  QgsProject::instance()->addMapLayer( mDebugLayer );
  return mDebugLayer;
}

void CreateTool::clearDebugMarkers()
{
  for ( QgsRubberBand *rb : mDebugMarkers )
  {
    if ( mCanvas && mCanvas->scene() )
      mCanvas->scene()->removeItem( rb );
    delete rb;
  }
  mDebugMarkers.clear();
}

// ==================== 核心功能：新建多边形 ====================

void CreateTool::finishCurrentFeatureWithHeight()
{
  if ( mPoints.size() < 3 || !mVectorLayer )
    return;

  qDebug() << "======= [DIGITIZE FINAL COMPLETE START] =======";

  try
  {
    // 1. 初始化原始点
    QVector<QgsPointXY> correctedPoints;
    for ( int i = 0; i < mPoints.size(); ++i )
      correctedPoints.append( mPoints[i] );
    if ( correctedPoints.first() != correctedPoints.last() )
      correctedPoints.append( correctedPoints.first() );

    // 2. 获取融合配置
    bool useMerge = mUI.mergecheckBox && mUI.mergecheckBox->isChecked();
    bool ok = false;
    double threshold = mUI.mergelineEdit->text().toDouble( &ok );
    if ( !useMerge || !ok || threshold <= 0 )
      useMerge = false;

    QList<QgsFeature> nearbyFeatures;
    struct SnapInfo
    {
        QgsFeatureId fid = -1;
        int vIdx = -1;
    };
    QVector<SnapInfo> snapLog( correctedPoints.size() );

    QSet<QgsFeatureId> involvedFids;

    // 3. 融合/吸附逻辑
    if ( useMerge )
    {
      mVectorLayer->updateExtents();
      QgsGeometry tempGeom = QgsGeometry::fromPolylineXY( correctedPoints );
      if ( !tempGeom.isNull() )
      {
        QgsRectangle searchRect = tempGeom.boundingBox().buffered( threshold );
        QgsFeatureRequest req;
        req.setFilterRect( searchRect );
        QgsFeatureIterator it = mVectorLayer->getFeatures( req );
        QgsFeature f;
        while ( it.nextFeature( f ) )
          nearbyFeatures.append( f );

        for ( int i = 0; i < correctedPoints.size(); ++i )
        {
          QgsPointXY oldP = correctedPoints[i];
          double bestDistSqr = threshold * threshold;

          for ( const QgsFeature &fB : nearbyFeatures )
          {
            QgsGeometry gB = fB.geometry();
            int vIdx, pIdx, rIdx;
            double dV2;
            QgsPointXY vNear = gB.closestVertex( oldP, vIdx, pIdx, rIdx, dV2 );

            // 顶点吸附
            if ( vIdx != -1 && dV2 < bestDistSqr )
            {
              bestDistSqr = dV2;
              correctedPoints[i] = vNear;
              snapLog[i] = { fB.id(), vIdx };
              involvedFids.insert( fB.id() );
            }

            // 线段吸附
            if ( snapLog[i].vIdx == -1 )
            {
              QgsPointXY eP;
              int nIdx;
              double dE2 = gB.closestSegmentWithContext( oldP, eP, nIdx );
              if ( dE2 < bestDistSqr )
              {
                bestDistSqr = dE2;
                correctedPoints[i] = eP;
                snapLog[i] = { fB.id(), -1 };
                involvedFids.insert( fB.id() );
              }
            }
          }
        }
        correctedPoints[correctedPoints.size() - 1] = correctedPoints[0];
      }
    }

    // 4. 构建最终几何环
    QgsPolylineXY finalRing;
    for ( int i = 0; i < correctedPoints.size() - 1; ++i )
    {
      finalRing.append( correctedPoints[i] );
      // 拓扑缝合逻辑
      if ( useMerge && snapLog[i].fid != -1 && snapLog[i].fid == snapLog[i + 1].fid )
      {
        int iS = snapLog[i].vIdx;
        int iE = snapLog[i + 1].vIdx;
        if ( iS != -1 && iE != -1 && std::abs( iS - iE ) > 1 && std::abs( iS - iE ) < 3 )
        {
          auto itFeat = std::find_if( nearbyFeatures.begin(), nearbyFeatures.end(), [&]( const QgsFeature &f ) { return f.id() == snapLog[i].fid; } );
          if ( itFeat != nearbyFeatures.end() )
          {
            const QgsAbstractGeometry *gB = itFeat->geometry().constGet();
            int totalV = gB->vertexCount();
            int forward = ( iE - iS + totalV ) % totalV;
            int backward = ( iS - iE + totalV ) % totalV;
            if ( forward <= backward )
            {
              for ( int k = 1; k < forward; ++k )
                finalRing.append( QgsPointXY( gB->vertexAt( QgsVertexId( 0, 0, ( iS + k ) % totalV ) ).x(), gB->vertexAt( QgsVertexId( 0, 0, ( iS + k ) % totalV ) ).y() ) );
            }
            else
            {
              for ( int k = 1; k < backward; ++k )
                finalRing.append( QgsPointXY( gB->vertexAt( QgsVertexId( 0, 0, ( iS - k + totalV ) % totalV ) ).x(), gB->vertexAt( QgsVertexId( 0, 0, ( iS - k + totalV ) % totalV ) ).y() ) );
            }
          }
        }
      }
    }
    if ( !finalRing.isEmpty() && finalRing.last() != finalRing.first() )
      finalRing.append( finalRing.first() );

    // 5. 生成最终几何
    QgsPolygonXY finalPolygon;
    finalPolygon << finalRing;
    QgsGeometry finalGeom = QgsGeometry::fromPolygonXY( finalPolygon );
    if ( !finalGeom.isGeosValid() )
      finalGeom = finalGeom.makeValid();

    // 6. 开启编辑事务
    if ( !mVectorLayer->isEditable() )
      mVectorLayer->startEditing();

    mVectorLayer->beginEditCommand( "Create Feature and Refresh Snap Heights" );

    // 7. 处理高度计算逻辑
    int fieldIdx = mVectorLayer->fields().indexOf( mTargetFieldName );
    bool doHeight = mUI.heightcheckBox && mUI.heightcheckBox->isChecked() && mPCLayer && fieldIdx != -1;

    // 计算高度
    QgsFeature fNew( mVectorLayer->fields() );
    fNew.setGeometry( finalGeom );
    if ( doHeight )
    {
      double zNew = calculateZFromPointCloud( finalGeom );
      fNew.setAttribute( fieldIdx, zNew );
    }

    if ( doHeight )
    {
      for ( QgsFeatureId fid : involvedFids )
      {
        QgsFeature adjFeat;
        if ( mVectorLayer->getFeatures( QgsFeatureRequest( fid ) ).nextFeature( adjFeat ) )
        {
          double zAdj = calculateZFromPointCloud( adjFeat.geometry() );
          mVectorLayer->changeAttributeValue( fid, fieldIdx, zAdj );
          qDebug() << "Updated neighboring feature" << fid << "height to" << zAdj;
        }
      }
    }

    // 8. 提交并结束
    if ( mVectorLayer->addFeature( fNew ) )
    {
      mVectorLayer->endEditCommand();
      mVectorLayer->triggerRepaint();
      refresh3DView();
      qDebug() << "Success: New feature and neighbors updated.";
    }
    else
    {
      mVectorLayer->destroyEditCommand();
      qDebug() << "Error: addFeature failed.";
    }
  }
  catch ( const std::exception &e )
  {
    qDebug() << "Fatal Error in finish function:" << e.what();
    if ( mVectorLayer )
      mVectorLayer->destroyEditCommand();
  }

  qDebug() << "======= [DIGITIZE FINAL COMPLETE END] =======";
  cancelDigitizing();
}

void CreateTool::cancelDigitizing( bool clearFinished )
{
  mIsDigitizing = false;
  mIsSplitting = false;
  mPoints.clear();
  if ( mRubberBand )
    mRubberBand->reset( Qgis::GeometryType::Polygon );
  if ( mSplitLineBand )
    mSplitLineBand->reset( Qgis::GeometryType::Line );
}

// ==================== 核心功能：两要素融合吸附 ====================
void CreateTool::snapTwoSelectedFeatures()
{
  if ( !mVectorLayer || !mVectorLayer->isEditable() )
    return;

  // 1. 获取选中的两个要素
  QgsFeatureIterator it = mVectorLayer->getSelectedFeatures();
  QList<QgsFeature> selectedFeatures;
  QgsFeature f;
  while ( it.nextFeature( f ) )
    selectedFeatures.append( f );

  if ( selectedFeatures.size() != 2 )
    return;

  QgsFeature featA = selectedFeatures[0];
  QgsFeature featB = selectedFeatures[1];

  // 获取原始几何，用于后续对比
  QgsGeometry geomA = featA.geometry();
  QgsGeometry geomB = featB.geometry();

  // 2. 严格阈值逻辑
  double threshold = 0.0;
  bool ok = false;
  if ( mUI.mergecheckBox && mUI.mergecheckBox->isChecked() )
  {
    threshold = mUI.mergelineEdit->text().toDouble( &ok );
  }

  if ( !ok || threshold <= 0 )
  {
    qDebug() << "Auto-snap ABORTED: No valid threshold provided.";
    return;
  }

  double thresholdSq = threshold * threshold;
  const QgsAbstractGeometry *absGeomA = geomA.constGet();
  const QgsAbstractGeometry *absGeomB = geomB.constGet();
  int vertexCountA = absGeomA->vertexCount();

  // 3. 记录移动状态 (针对 A 要素)
  struct SnapResult
  {
      bool moved = false;
      double minDistSq = 1e18;
      QgsPoint pos;
  };
  QVector<SnapResult> resultsA( vertexCountA );
  for ( int i = 0; i < vertexCountA; ++i )
    resultsA[i].pos = absGeomA->vertexAt( QgsVertexId( 0, 0, i ) );

  // 4. 计算吸附逻辑 (将 A 的边界向 B 对齐)
  for ( int i = 0; i < vertexCountA - 1; ++i )
  {
    QgsPoint pA1 = absGeomA->vertexAt( QgsVertexId( 0, 0, i ) );
    QgsPoint pA2 = absGeomA->vertexAt( QgsVertexId( 0, 0, i + 1 ) );

    double angleA = std::atan2( pA2.y() - pA1.y(), pA2.x() - pA1.x() );
    QgsGeometry edgeA = QgsGeometry::fromPolylineXY( QgsPolylineXY() << QgsPointXY( pA1.x(), pA1.y() ) << QgsPointXY( pA2.x(), pA2.y() ) );

    for ( int j = 0; j < absGeomB->vertexCount() - 1; ++j )
    {
      QgsPoint pB1 = absGeomB->vertexAt( QgsVertexId( 0, 0, j ) );
      QgsPoint pB2 = absGeomB->vertexAt( QgsVertexId( 0, 0, j + 1 ) );

      double angleB = std::atan2( pB2.y() - pB1.y(), pB2.x() - pB1.x() );
      double angleDiff = std::abs( angleA - angleB );
      while ( angleDiff > M_PI )
        angleDiff -= M_PI;

      if ( angleDiff > 0.35 && angleDiff < ( M_PI - 0.35 ) )
        continue;

      QgsGeometry edgeB = QgsGeometry::fromPolylineXY( QgsPolylineXY() << QgsPointXY( pB1.x(), pB1.y() ) << QgsPointXY( pB2.x(), pB2.y() ) );
      double currentEdgeDist = edgeA.distance( edgeB );

      if ( currentEdgeDist < threshold )
      {
        int indices[2] = { i, i + 1 };
        QgsPointXY ptsToTest[2] = { QgsPointXY( pA1.x(), pA1.y() ), QgsPointXY( pA2.x(), pA2.y() ) };

        for ( int k = 0; k < 2; ++k )
        {
          int idx = indices[k];
          if ( currentEdgeDist > resultsA[idx].minDistSq )
            continue;

          QgsPointXY currentPt = ptsToTest[k];
          double d1Sq = currentPt.sqrDist( QgsPointXY( pB1.x(), pB1.y() ) );
          double d2Sq = currentPt.sqrDist( QgsPointXY( pB2.x(), pB2.y() ) );

          if ( d1Sq < thresholdSq || d2Sq < thresholdSq )
          {
            QgsPointXY target = ( d1Sq < d2Sq ) ? QgsPointXY( pB1.x(), pB1.y() ) : QgsPointXY( pB2.x(), pB2.y() );
            resultsA[idx].pos = QgsPoint( target.x(), target.y() );
            resultsA[idx].moved = true;
            resultsA[idx].minDistSq = currentEdgeDist;
          }
          else
          {
            double dx = pB2.x() - pB1.x(), dy = pB2.y() - pB1.y();
            double L2 = dx * dx + dy * dy;
            if ( L2 > 1e-7 )
            {
              double r = ( ( currentPt.x() - pB1.x() ) * dx + ( currentPt.y() - pB1.y() ) * dy ) / L2;
              QgsPoint targetPos;

              if ( r < -3.0 || r > 4.0 )
              {
                targetPos = ( r < 0 ) ? pB1 : pB2;
              }
              else
              {
                targetPos = QgsPoint( pB1.x() + r * dx, pB1.y() + r * dy );
              }

              if ( currentPt.sqrDist( QgsPointXY( targetPos.x(), targetPos.y() ) ) < thresholdSq )
              {
                resultsA[idx].pos = targetPos;
                resultsA[idx].moved = true;
                resultsA[idx].minDistSq = currentEdgeDist;
              }
            }
          }
        }
      }
    }
  }

  // 5. 应用修改并重新计算两个要素的高度
  bool changed = false;
  for ( const auto &r : resultsA )
    if ( r.moved )
    {
      changed = true;
      break;
    }

  if ( changed )
  {
    // 处理首尾闭合点
    if ( resultsA[0].moved )
      resultsA[vertexCountA - 1] = resultsA[0];
    else if ( resultsA[vertexCountA - 1].moved )
      resultsA[0] = resultsA[vertexCountA - 1];

    QgsPolylineXY newRingA;
    for ( const auto &r : resultsA )
      newRingA << QgsPointXY( r.pos.x(), r.pos.y() );

    QgsGeometry finalGeomA = QgsGeometry::fromPolygonXY( QgsPolygonXY() << newRingA );
    if ( !finalGeomA.isGeosValid() )
      finalGeomA = finalGeomA.makeValid();

    // 开启编辑事务
    mVectorLayer->beginEditCommand( "Dual-Feature Snap and Re-Height" );

    // A. 更新 A 的几何
    mVectorLayer->changeGeometry( featA.id(), finalGeomA );

    // B. 计算高度更新逻辑
    if ( mUI.heightcheckBox && mUI.heightcheckBox->isChecked() && mPCLayer )
    {
      int fieldIdx = mVectorLayer->fields().indexOf( mTargetFieldName );
      if ( fieldIdx != -1 )
      {
        // 计算并更新 A 的高度 (基于新几何)
        double newZA = calculateZFromPointCloud( finalGeomA );
        mVectorLayer->changeAttributeValue( featA.id(), fieldIdx, newZA );

        // 计算并更新 B 的高度 (基于 B 的当前几何)
        double newZB = calculateZFromPointCloud( geomB );
        mVectorLayer->changeAttributeValue( featB.id(), fieldIdx, newZB );

        qDebug() << QString( "Snap height updated - A: %1, B: %2" ).arg( newZA ).arg( newZB );
      }
    }

    mVectorLayer->endEditCommand();
    mVectorLayer->triggerRepaint();
    refresh3DView();
  }
}

// ==================== 核心功能：分割要素 ====================

void CreateTool::performSplit( const QgsPointXY &snapEnd )
{
  if ( !mVectorLayer || mTargetFeatureId == -1 )
    return;

  QgsFeature targetFeat = mVectorLayer->getFeature( mTargetFeatureId );
  if ( !targetFeat.isValid() )
    return;

  QgsGeometry targetGeom = targetFeat.geometry();

  double dx = snapEnd.x() - mSplitStartPoint.x();
  double dy = snapEnd.y() - mSplitStartPoint.y();
  double len = std::sqrt( dx * dx + dy * dy );
  if ( len < 1e-6 )
    return;

  // 动态延伸量
  double pixelOffset = mCanvas->mapUnitsPerPixel() * 50.0;
  double offset = std::max( pixelOffset, 0.1 );

  QgsPointXY p1( mSplitStartPoint.x() - ( dx / len ) * offset, mSplitStartPoint.y() - ( dy / len ) * offset );
  QgsPointXY p2( snapEnd.x() + ( dx / len ) * offset, snapEnd.y() + ( dy / len ) * offset );

  QVector<QgsPointXY> splitLinePoints;
  splitLinePoints << p1 << p2;

  QVector<QgsGeometry> newGeometries;
  QVector<QgsPointXY> topoPoints;

  Qgis::GeometryOperationResult result = targetGeom.splitGeometry( splitLinePoints, newGeometries, false, topoPoints );

  if ( result == Qgis::GeometryOperationResult::Success )
  {
    mVectorLayer->beginEditCommand( QString( "分割要素 %1" ).arg( mTargetFeatureId ) );

    mVectorLayer->changeGeometry( targetFeat.id(), targetGeom );

    int fieldIdx = mVectorLayer->fields().indexOf( mTargetFieldName );
    bool doHeight = mUI.heightcheckBox && mUI.heightcheckBox->isChecked() && mPCLayer && fieldIdx != -1;

    if ( doHeight )
      mVectorLayer->changeAttributeValue( targetFeat.id(), fieldIdx, calculateZFromPointCloud( targetGeom ) );

    for ( const QgsGeometry &part : newGeometries )
    {
      QgsFeature newFeat( targetFeat );
      newFeat.setGeometry( part );
      if ( doHeight )
        newFeat.setAttribute( fieldIdx, calculateZFromPointCloud( part ) );
      mVectorLayer->addFeature( newFeat );
    }

    mVectorLayer->endEditCommand();
    mVectorLayer->triggerRepaint();
    refresh3DView();
  }
  else
  {
    QString errorDetail;
    if ( result == Qgis::GeometryOperationResult::GeometryEngineError )
      errorDetail = "(几何引擎错误)";
    else if ( result == Qgis::GeometryOperationResult::InvalidInputGeometryType )
      errorDetail = "(无效的几何类型)";

    QMessageBox::warning( mCanvas, "分割失败", "切割线未完全贯穿要素 " + errorDetail );
    qDebug() << "Split Result Code:" << static_cast<int>( result );
  }
}

// ==================== 辅助函数：吸附到几何最近点 ====================

QgsPointXY CreateTool::getSnappedPoint( const QgsGeometry &geom, const QgsPointXY &mapPt, double tolerance )
{
  if ( geom.isEmpty() )
    return mapPt;

  int vIdx, pIdx, rIdx;
  double d2;

  // 1. 获取最近顶点
  QgsPointXY snapPoint = geom.closestVertex( mapPt, vIdx, pIdx, rIdx, d2 );
  double distToVertex = std::sqrt( d2 );

  // 顶点优先：只要在容差范围内，直接锁定顶点
  if ( distToVertex < tolerance )
  {
    return snapPoint;
  }

  // 2. 只有离顶点较远时，才尝试吸附到边上的投影点
  QgsPoint segmentPoint;
  QgsVertexId vId;
  double distEdge = geom.constGet()->closestSegment( QgsPoint( mapPt ), segmentPoint, vId );

  // 稍微放宽边的感应范围 (1.5倍)，增强吸附感
  if ( distEdge < tolerance * 2.0 )
  {
    return QgsPointXY( segmentPoint.x(), segmentPoint.y() );
  }

  return mapPt;
}

void CreateTool::refresh3DView()
{
  if ( mVectorLayer )
  {
    mVectorLayer->triggerRepaint();
  }

  QgsProject::instance()->layerTreeRoot()->findLayer( mVectorLayer->id() )->setItemVisibilityChecked( true );
}
