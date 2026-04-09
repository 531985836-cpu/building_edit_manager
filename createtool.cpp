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

CreateTool::CreateTool( QgsMapCanvas *canvas )
  : QgsMapTool( canvas )
{
  setCursor( Qt::CrossCursor );
  mRubberBand = new QgsRubberBand( canvas, Qgis::GeometryType::Polygon );

  mSettingsWidget = nullptr;
}

CreateTool::~CreateTool()
{
}

void CreateTool::activate()
{
  QgsMapTool::activate();

  // 1. 完全模仿例子逻辑：如果窗口没创建，则进行初始化
  if ( !mSettingsWidget )
  {
    setupUi();
  }

  // 2. 勾选工具时刷新图层列表
  refreshLayerCombos();

  // 3. 弹出窗口
  if ( mSettingsWidget )
  {
    mSettingsWidget->show();
    mSettingsWidget->raise();
    mSettingsWidget->activateWindow();
  }
}

void CreateTool::setupUi()
{
  mSettingsWidget = new QWidget();
  mUI.setupUi( mSettingsWidget );
  mSettingsWidget->setWindowTitle( "设置" );

  // --- 1. 初始化控件状态 (根据当前 CheckBox 状态设置) ---
  updateWidgetInteractivity( mUI.heightcheckBox->isChecked(), mUI.mergecheckBox->isChecked() );

  // --- 2. 绑定信号槽：点击 CheckBox 时自动切换下方控件的禁用/启用状态 ---

  // 高度模块联动
  connect( mUI.heightcheckBox, &QCheckBox::toggled, this, [this]( bool checked ) {
    mUI.fieldcombo->setEnabled( checked );
    mUI.pointcloudcombo->setEnabled( checked );
    mUI.field->setEnabled( checked );      // 标签也变灰
    mUI.pointcloud->setEnabled( checked ); // 标签也变灰
  } );

  // 融合模块联动
  connect( mUI.mergecheckBox, &QCheckBox::toggled, this, [this]( bool checked ) {
    mUI.mergelineEdit->setEnabled( checked );
    mUI.merge->setEnabled( checked ); // 标签也变灰
  } );

  // --- 3. 其他原有逻辑 ---
  connect( mUI.vectorcombo, QOverload<int>::of( &QComboBox::currentIndexChanged ), this, &CreateTool::updateFields );

// 确认按钮功能：保存、验证并复现设置内容
  connect( mUI.setting, &QPushButton::clicked, this, [this]() {
    if ( !mUI.vectorcombo )
      return;

    // 1. 获取基础矢量图层
    QString vId = mUI.vectorcombo->currentData().toString();
    mVectorLayer = qobject_cast<QgsVectorLayer *>( QgsProject::instance()->mapLayer( vId ) );

    if ( !mVectorLayer )
    {
      QMessageBox::warning( mSettingsWidget, "错误", "请选择有效的矢量图层！" );
      return;
    }

    // 2. 构造复现内容字符串
    QString info = QString( "<b>基础配置：</b><br>目标图层：%1<br><br>" ).arg( mVectorLayer->name() );

    // --- 高度初始化模块复现 ---
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
      mPCLayer = nullptr; // 确保关闭时不使用旧的点云指针
    }

    // --- 融合设置模块复现 ---
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

    // 3. 确保图层处于编辑状态
    if ( !mVectorLayer->isEditable() )
      mVectorLayer->startEditing();

    // 4. 弹出复现对话框
    QMessageBox::information( mSettingsWidget, "配置确认", info );

    // 5. 隐藏设置窗口
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

  // --- 关键修改：添加初始空选项 ---
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

  // 将当前索引指向这个“请选择...”项（索引为0）
  mUI.vectorcombo->setCurrentIndex( 0 );
  mUI.pointcloudcombo->setCurrentIndex( 0 );

  mUI.vectorcombo->blockSignals( false );
  mUI.pointcloudcombo->blockSignals( false );

  // 初始时清空字段列表，不要自动刷新
  mUI.fieldcombo->clear();
}

void CreateTool::updateFields( int index )
{
  if ( !mUI.fieldcombo || !mUI.vectorcombo )
    return;

  mUI.fieldcombo->clear();

  // 如果 index 为 0 (即选中的是 "请选择...")，直接返回
  if ( index <= 0 )
    return;

  QString layerId = mUI.vectorcombo->currentData().toString();
  QgsVectorLayer *vLayer = qobject_cast<QgsVectorLayer *>( QgsProject::instance()->mapLayer( layerId ) );

  if ( vLayer )
  {
    // 同样可以给字段下拉框加一个初始项
    mUI.fieldcombo->addItem( "请选择字段..." );
    for ( const QgsField &f : vLayer->fields() )
    {
      mUI.fieldcombo->addItem( f.name() );
    }
  }
}

void CreateTool::canvasPressEvent( QgsMapMouseEvent *e )
{
  QgsPointXY mapPt = toMapCoordinates( e->pos() );

  // --- 1. 数字化状态逻辑 ---
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
    }
    return;
  }

  // --- 2. 非数字化状态逻辑 ---
  if ( e->button() == Qt::LeftButton )
  {
    if ( !mVectorLayer )
      return;

    double searchRadius = mCanvas->mapUnitsPerPixel() * 5.0;
    QgsRectangle searchRect( mapPt.x() - searchRadius, mapPt.y() - searchRadius, mapPt.x() + searchRadius, mapPt.y() + searchRadius );

    QgsFeatureIterator it = mVectorLayer->getFeatures( QgsFeatureRequest().setFilterRect( searchRect ) );
    QgsFeature feat;

    if ( it.nextFeature( feat ) )
    {
      // --- 情况 A: 选中了要素 ---
      QgsFeatureIds selectedIds = mVectorLayer->selectedFeatureIds();
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
        mVectorLayer->removeSelection();
        mVectorLayer->select( feat.id() );
      }

      mCanvas->refresh();

      // 如果选中数量等于 2，触发自动吸附和重新计算高度
      if ( mVectorLayer->selectedFeatureCount() == 2 )
      {
        snapTwoSelectedFeatures();
      }
    }
    else
    {
      // --- 情况 B: 点击空白处，恢复新建逻辑 ---
      mVectorLayer->removeSelection();
      mCanvas->refresh();

      mIsDigitizing = true;
      mRubberBand->reset( Qgis::GeometryType::Polygon );
      mPoints.append( mapPt );
      mRubberBand->addPoint( mapPt );
    }
  }
}

void CreateTool::canvasMoveEvent( QgsMapMouseEvent *e )
{
  if ( !mIsDigitizing || mPoints.isEmpty() )
    return;

  QgsPointXY mapPt = toMapCoordinates( e->pos() );

  if ( mRubberBand->numberOfVertices() > mPoints.size() )
    mRubberBand->removeLastPoint();

  mRubberBand->addPoint( mapPt );
}

double CreateTool::calculateZFromPointCloud( const QgsGeometry &geom )
{
  if ( geom.isNull() || geom.isEmpty() || !geom.isGeosValid() )
    return 0.0;
  if ( !mPCLayer || !mPCLayer->dataProvider() )
    return 0.0;

  clearDebugMarkers();

  // --- 1. 准备点云数据 ---
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

  // ==========================================================
  // 【检查功能控制开关】
  // 设置为 true 则生成 3D 法线碎线，设置为 false 则完全关闭该功能
  // ==========================================================
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

  // --- 2. 计算法线与高度 ---
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

        // --- 绘制法线碎线 ---
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

  // --- 3. 提交调试图层修改 ---
  if ( showDebug && debugLayer )
  {
    debugLayer->commitChanges();
    debugLayer->triggerRepaint();
  }

  if ( roofZValues.isEmpty() )
    return 0.0;
  return std::accumulate( roofZValues.begin(), roofZValues.end(), 0.0 ) / roofZValues.size();
}

// --- 2. 修复 finishCurrentFeatureWithHeight 里的缝合逻辑 ---
void CreateTool::finishCurrentFeatureWithHeight()
{
    if ( mPoints.size() < 3 || !mVectorLayer )
        return;

    // 1. 初始化面要素环 (abcda)
    QgsPolylineXY newRing;
    for ( const QgsPointXY &pt : mPoints )
        newRing.append( pt );
    if ( newRing.first() != newRing.last() )
        newRing.append( newRing.first() );

    double threshold = 2.0;
    if ( mUI.mergecheckBox && mUI.mergecheckBox->isChecked() )
    {
        double t = mUI.mergelineEdit->text().toDouble();
        if ( t > 0 ) threshold = t;
    }
    double thresholdSqr = std::pow( threshold, 2 );

    // 空间检索
    QgsFeatureRequest req;
    QgsGeometry searchArea = QgsGeometry::fromPolygonXY( QgsPolygonXY() << newRing );
    req.setFilterRect( searchArea.boundingBox().buffered( threshold + 1.0 ) );
    QgsFeatureIterator it = mVectorLayer->getFeatures( req );
    QList<QgsFeature> nearbyFeatures;
    QgsFeature fNearby;
    while ( it.nextFeature( fNearby ) ) nearbyFeatures.append( fNearby );

    QVector<QgsPointXY> correctedPoints = newRing;

    // 用于记录吸附信息的结构，方便后续加点拟合
    struct SnapInfo {
        QgsFeatureId fid = -1;
        int vertexIdx = -1;
    };
    QVector<SnapInfo> snapLog(correctedPoints.size());

    // --- 3. 核心对齐逻辑 ---
    for ( int i = 0; i < correctedPoints.size() - 1; ++i )
    {
        QgsPointXY &pA = correctedPoints[i];
        double bestDistSqr = thresholdSqr;
        QgsPointXY bestTarget = pA;
        bool foundSnap = false;

        for ( const QgsFeature &fB : nearbyFeatures )
        {
            QgsGeometry gB = fB.geometry();
            if ( gB.isNull() ) continue;

            // 首先探测点到边的最近距离
            QgsPointXY edgeProj;
            int nextEdgeIdx;
            double dEdge2 = gB.closestSegmentWithContext( pA, edgeProj, nextEdgeIdx );

            // A. 只有当点到边的距离进入阈值，才激活吸附逻辑
            if ( dEdge2 < thresholdSqr )
            {
                // B. 尝试探测顶点 (点对点优先)
                int vIdx, pIdx, rIdx;
                double dVert2;
                QgsPointXY vNear = gB.closestVertex( pA, vIdx, pIdx, rIdx, dVert2 );

                if ( vIdx != -1 && dVert2 < thresholdSqr )
                {
                    // 情况 1：点离顶点近 -> 直接锁定到顶点
                    if ( dVert2 < bestDistSqr ) {
                        bestDistSqr = dVert2;
                        bestTarget = vNear;
                        snapLog[i] = { fB.id(), vIdx };
                        foundSnap = true;
                    }
                }
                else
                {
                    // 情况 2：点离顶点远，但离边近 -> 吸附到延长线上
                    const QgsAbstractGeometry* absGeom = gB.constGet();
                    if ( absGeom && nextEdgeIdx > 0 && nextEdgeIdx < absGeom->vertexCount() )
                    {
                        QgsPoint lp1 = absGeom->vertexAt( QgsVertexId( 0, 0, nextEdgeIdx - 1 ) );
                        QgsPoint lp2 = absGeom->vertexAt( QgsVertexId( 0, 0, nextEdgeIdx ) );
                        
                        // 计算直线投影 (向量法)
                        double dx = lp2.x() - lp1.x();
                        double dy = lp2.y() - lp1.y();
                        double L2 = dx * dx + dy * dy;

                        if ( L2 > 1e-6 )
                        {
                            // 计算投影比例 r (r 可以在 0-1 之外，即延长线上)
                            double r = ( ( pA.x() - lp1.x() ) * dx + ( pA.y() - lp1.y() ) * dy ) / L2;
                            QgsPointXY lineProj( lp1.x() + r * dx, lp1.y() + r * dy );
                            
                            if ( dEdge2 < bestDistSqr ) {
                                bestDistSqr = dEdge2;
                                bestTarget = lineProj;
                                snapLog[i] = { fB.id(), -1 }; // 记录为边吸附
                                foundSnap = true;
                            }
                        }
                    }
                }
            }
        }

        if ( foundSnap )
        {
            pA = bestTarget;
            if ( i == 0 ) correctedPoints[correctedPoints.size() - 1] = pA;
        }
    }

    // --- 4. 自动加点拟合逻辑 (ab点间插值) ---
    QVector<QgsPointXY> finalPoints;
    for ( int i = 0; i < correctedPoints.size() - 1; ++i )
    {
        finalPoints.append(correctedPoints[i]);

        // 逻辑：如果连续两个点吸到了同一个要素的顶点，且中间有跳过的点
        if ( snapLog[i].fid != -1 && snapLog[i].fid == snapLog[i+1].fid && 
             snapLog[i].vertexIdx != -1 && snapLog[i+1].vertexIdx != -1 )
        {
            int idxStart = snapLog[i].vertexIdx;
            int idxEnd = snapLog[i+1].vertexIdx;

            if ( std::abs(idxStart - idxEnd) > 1 )
            {
                auto itFeat = std::find_if(nearbyFeatures.begin(), nearbyFeatures.end(), 
                              [&](const QgsFeature& f){ return f.id() == snapLog[i].fid; });
                
                if ( itFeat != nearbyFeatures.end() )
                {
                    const QgsAbstractGeometry* geomB = itFeat->geometry().constGet();
                    int step = (idxEnd > idxStart) ? 1 : -1;
                    for ( int k = idxStart + step; k != idxEnd; k += step )
                    {
                        QgsPoint pMid = geomB->vertexAt( QgsVertexId(0, 0, k) );
                        finalPoints.append( QgsPointXY(pMid.x(), pMid.y()) );
                    }
                }
            }
        }
    }
    finalPoints.append(correctedPoints.last());

    // 5. 几何提交
    QgsGeometry finalGeom = QgsGeometry::fromPolygonXY( QgsPolygonXY() << finalPoints );
    if ( finalGeom.get() ) finalGeom.get()->removeDuplicateNodes();
    
    // 提交到图层
    if ( !mVectorLayer->isEditable() ) mVectorLayer->startEditing();
    QgsFeature fNew( mVectorLayer->fields() );
    fNew.setGeometry( finalGeom );

    if ( mUI.heightcheckBox && mUI.heightcheckBox->isChecked() && mPCLayer ) {
        double z = calculateZFromPointCloud( finalGeom );
        int idx = mVectorLayer->fields().indexOf( mTargetFieldName );
        if ( idx != -1 ) fNew.setAttribute( idx, z );
    }

    if ( mVectorLayer->addFeature( fNew ) ) mVectorLayer->triggerRepaint();
    cancelDigitizing();
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
    // 删除选中的要素
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
      mVectorLayer->startEditing(); // 提交后继续保持编辑状态
      qDebug() << "Changes committed.";
    }
  }

  QgsMapTool::keyPressEvent( e );
}

// 清空当前正在绘制的顶点
void CreateTool::cancelDigitizing( bool clearFinished )

{
  mIsDigitizing = false;
  mPoints.clear();

  if ( mRubberBand )
    mRubberBand->reset( Qgis::GeometryType::Polygon );

  Q_UNUSED( clearFinished );
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

// 在 deactivate() 和 cancelDigitizing() 中也调用一下
void CreateTool::deactivate()
{
  clearDebugMarkers();
  cancelDigitizing();
  QgsMapTool::deactivate();
}

void CreateTool::collectNodes( const QgsPointCloudIndex &index, const QgsPointCloudNodeId &nodeId, const QgsRectangle &extent, QList<QgsPointCloudNodeId> &nodes )
{
  // 如果当前节点 ID 无效，直接返回
  if ( !nodeId.isValid() )
    return;

  // 1. 将当前节点加入列表（QgsPointCloudBlock 在读取时会根据 request.filterRect 自动过滤掉不在范围内的点）
  nodes.append( nodeId );

  // 2. 递归深度限制：QGIS 默认最大深度通常为 18-20，这里我们检查所有存在的子节点
  // 如果当前节点没有点或者超深，就不再往下搜，但通常只要 hasNode 为真就应该搜寻
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
    {
      // 注意：这里为了解决你搜不到点的问题，我们先减少空间剔除的干扰
      // 让 index.nodeData 内置的裁剪逻辑去处理
      collectNodes( index, childId, extent, nodes );
    }
  }
}

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

QgsVectorLayer *CreateTool::getOrCreateDebugLayer()
{
  QString layerName = "Normal_Lines_Debug";
  QList<QgsMapLayer *> layers = QgsProject::instance()->mapLayersByName( layerName );
  if ( !layers.isEmpty() )
    return qobject_cast<QgsVectorLayer *>( layers.at( 0 ) );

  // 关键点：使用 LineString 并添加 z=yes 开启三维坐标支持
  QString uri = QString( "LineString?crs=%1&field=id:int&z=yes" )
                  .arg( mCanvas->mapSettings().destinationCrs().authid() );

  mDebugLayer = new QgsVectorLayer( uri, layerName, "memory" );
  QgsProject::instance()->addMapLayer( mDebugLayer );
  return mDebugLayer;
}

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
  QgsGeometry geomA = featA.geometry();
  QgsGeometry geomB = featB.geometry();

  // 2. 初始化吸附阈值
  double threshold = 2.0;
  if ( mUI.mergecheckBox && mUI.mergecheckBox->isChecked() )
  {
    double t = mUI.mergelineEdit->text().toDouble();
    if ( t > 0 )
      threshold = t;
  }
  double thresholdSq = threshold * threshold;

  const QgsAbstractGeometry *absGeomA = geomA.constGet();
  const QgsAbstractGeometry *absGeomB = geomB.constGet();
  int vertexCountA = absGeomA->vertexCount();

  // 3. 定义结构记录每个顶点的移动状态与最小匹配距离（防止重复覆盖）
  struct SnapResult
  {
      bool moved = false;
      double minDistSq = 1e18;
      QgsPoint pos;
  };
  QVector<SnapResult> results( vertexCountA );
  for ( int i = 0; i < vertexCountA; ++i )
    results[i].pos = absGeomA->vertexAt( QgsVertexId( 0, 0, i ) );

  // 4. 遍历要素 A 的每一条边
  for ( int i = 0; i < vertexCountA - 1; ++i )
  {
    QgsPoint pA1 = absGeomA->vertexAt( QgsVertexId( 0, 0, i ) );
    QgsPoint pA2 = absGeomA->vertexAt( QgsVertexId( 0, 0, i + 1 ) );
    QgsPointXY pA1XY( pA1.x(), pA1.y() ), pA2XY( pA2.x(), pA2.y() );

    // 计算 A 边方位角用于平行度过滤
    double angleA = std::atan2( pA2.y() - pA1.y(), pA2.x() - pA1.x() );
    QgsPolylineXY lineA;
    lineA << pA1XY << pA2XY;
    QgsGeometry edgeA = QgsGeometry::fromPolylineXY( lineA );

    // 5. 与要素 B 的每一条边进行比对
    for ( int j = 0; j < absGeomB->vertexCount() - 1; ++j )
    {
      QgsPoint pB1 = absGeomB->vertexAt( QgsVertexId( 0, 0, j ) );
      QgsPoint pB2 = absGeomB->vertexAt( QgsVertexId( 0, 0, j + 1 ) );
      QgsPointXY pB1XY( pB1.x(), pB1.y() ), pB2XY( pB2.x(), pB2.y() );

      // 平行度过滤：非平行边不进行吸附（阈值约 20 度）
      double angleB = std::atan2( pB2.y() - pB1.y(), pB2.x() - pB1.x() );
      double angleDiff = std::abs( angleA - angleB );
      while ( angleDiff > M_PI )
        angleDiff -= M_PI;
      if ( angleDiff > 0.35 && angleDiff < ( M_PI - 0.35 ) )
        continue;

      QgsPolylineXY lineB;
      lineB << pB1XY << pB2XY;
      QgsGeometry edgeB = QgsGeometry::fromPolylineXY( lineB );
      double currentEdgeDist = edgeA.distance( edgeB );

      // 6. 执行距离内的点吸附逻辑
      if ( currentEdgeDist < threshold )
      {
        QgsPointXY ptsToTest[2] = { pA1XY, pA2XY };
        int indices[2] = { i, i + 1 };

        for ( int k = 0; k < 2; ++k )
        {
          int idx = indices[k];
          if ( currentEdgeDist > results[idx].minDistSq )
            continue;

          QgsPointXY currentPt = ptsToTest[k];
          double d1Sq = currentPt.sqrDist( pB1XY );
          double d2Sq = currentPt.sqrDist( pB2XY );

          if ( d1Sq < thresholdSq || d2Sq < thresholdSq )
          {
            // 优先点点重合
            QgsPointXY target = ( d1Sq < d2Sq ) ? pB1XY : pB2XY;
            results[idx].pos = QgsPoint( target.x(), target.y() );
            results[idx].moved = true;
            results[idx].minDistSq = currentEdgeDist;
          }
          else
          {
            // 计算投影比例 r
            double dx = pB2XY.x() - pB1XY.x();
            double dy = pB2XY.y() - pB1XY.y();
            double L2 = dx * dx + dy * dy;
            if ( L2 > 1e-7 )
            {
              double r = ( ( currentPt.x() - pB1XY.x() ) * dx + ( currentPt.y() - pB1XY.y() ) * dy ) / L2;
              QgsPoint targetPos;
              // 投影限制与回退机制
              if ( r < -3.0 || r > 4.0 )
                targetPos = QgsPoint( ( r < 0 ? pB1XY.x() : pB2XY.x() ), ( r < 0 ? pB1XY.y() : pB2XY.y() ) );
              else
                targetPos = QgsPoint( pB1XY.x() + r * dx, pB1XY.y() + r * dy );

              if ( currentPt.sqrDist( QgsPointXY( targetPos.x(), targetPos.y() ) ) < thresholdSq )
              {
                results[idx].pos = targetPos;
                results[idx].moved = true;
                results[idx].minDistSq = currentEdgeDist;
              }
            }
          }
        }
      }
    }
  }

  // 7. 组装新几何体并更新图层
  bool changed = false;
  for ( const auto &r : results )
    if ( r.moved )
    {
      changed = true;
      break;
    }

  if ( changed )
  {
    // 维持多边形首尾闭合
    if ( results[0].moved )
      results[vertexCountA - 1] = results[0];
    else if ( results[vertexCountA - 1].moved )
      results[0] = results[vertexCountA - 1];

    QgsPolylineXY newRing;
    for ( const auto &r : results )
      newRing << QgsPointXY( r.pos.x(), r.pos.y() );

    QgsPolygonXY newPolygon;
    newPolygon << newRing;
    QgsGeometry finalGeom = QgsGeometry::fromPolygonXY( newPolygon );

    mVectorLayer->beginEditCommand( "Advanced Edge Snap" );
    mVectorLayer->changeGeometry( featA.id(), finalGeom );
    mVectorLayer->endEditCommand();
    mVectorLayer->triggerRepaint();
  }
}
