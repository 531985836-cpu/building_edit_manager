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

  if ( e->button() == Qt::LeftButton )
  {
    if ( !mIsDigitizing )
    {
      mIsDigitizing = true;
      mRubberBand->reset( Qgis::GeometryType::Polygon );
    }

    mPoints.append( mapPt );
    mRubberBand->addPoint( mapPt );
  }
  else if ( e->button() == Qt::RightButton )
  {
    // 右键完成当前面要素，并计算高度
    finishCurrentFeatureWithHeight();
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

  // 1. 获取新要素 A 的原始顶点并闭合
  QgsPolylineXY ringA;
  for ( const QgsPointXY &pt : mPoints )
    ringA.append( pt );
  if ( ringA.first() != ringA.last() )
    ringA.append( ringA.first() );

  QgsGeometry geomA = QgsGeometry::fromPolygonXY( QgsPolygonXY() << ringA );
  if ( !geomA.isGeosValid() )
    geomA = geomA.makeValid();

  int nA = ringA.size() - 1;

  // 存储所有待融合的几何体
  QVector<QgsGeometry> allGeoms;
  allGeoms.append( geomA );

  QgsFeatureIds idsToDelete;
  QgsAttributes lastAttributes;
  bool anyMergeOccurred = false;

  // 2. 检索周围要素并生成桥接面 (基于中点判定)
  if ( mUI.mergecheckBox && mUI.mergecheckBox->isChecked() )
  {
    double threshold = mUI.mergelineEdit->text().toDouble();
    if ( threshold <= 0 )
      threshold = 2.0;
    double thresholdSqr = std::pow( threshold, 2 );

    QgsFeatureRequest req;
    req.setFilterRect( geomA.boundingBox().buffered( threshold + 2.0 ) );
    QgsFeatureIterator it = mVectorLayer->getFeatures( req );

    QgsFeature feat;
    while ( it.nextFeature( feat ) )
    {
      if ( !feat.hasGeometry() )
        continue;

      QgsGeometry gB = feat.geometry();
      QgsPolylineXY rB;
      if ( gB.isMultipart() )
      {
        QgsMultiPolygonXY mp = gB.asMultiPolygon();
        if ( !mp.isEmpty() )
          rB = mp.at( 0 ).at( 0 );
      }
      else
      {
        QgsPolygonXY pB = gB.asPolygon();
        if ( !pB.isEmpty() )
          rB = pB.at( 0 );
      }

      if ( rB.size() < 3 )
        continue;
      if ( rB.first() == rB.last() )
        rB.removeLast();
      int nB = rB.size();

      bool featNeedsMerging = false;

      for ( int i = 0; i < nA; ++i )
      {
        int nextA = ( i + 1 ) % nA;
        QgsPointXY midA( ( ringA[i].x() + ringA[nextA].x() ) / 2.0, ( ringA[i].y() + ringA[nextA].y() ) / 2.0 );

        for ( int j = 0; j < nB; ++j )
        {
          int nextB = ( j + 1 ) % nB;
          QgsPointXY midB( ( rB[j].x() + rB[nextB].x() ) / 2.0, ( rB[j].y() + rB[nextB].y() ) / 2.0 );

          if ( midA.sqrDist( midB ) < thresholdSqr )
          {
            QgsPolylineXY bridgeRing;
            double dDirect = ringA[i].sqrDist( rB[j] ) + ringA[nextA].sqrDist( rB[nextB] );
            double dCross = ringA[i].sqrDist( rB[nextB] ) + ringA[nextA].sqrDist( rB[j] );

            if ( dDirect <= dCross )
              bridgeRing << ringA[i] << rB[j] << rB[nextB] << ringA[nextA] << ringA[i];
            else
              bridgeRing << ringA[i] << rB[nextB] << rB[j] << ringA[nextA] << ringA[i];

            QgsGeometry bridgeGeom = QgsGeometry::fromPolygonXY( QgsPolygonXY() << bridgeRing );

            // 优化：微缓冲防止由于坐标微差导致的融合失败（要素消失的主因）
            bridgeGeom = bridgeGeom.buffer( 0.0001, 5 );

            if ( !bridgeGeom.isNull() )
            {
              allGeoms.append( bridgeGeom );
              featNeedsMerging = true;
            }
          }
        }
      }

      if ( featNeedsMerging )
      {
        allGeoms.append( gB.makeValid() );
        idsToDelete << feat.id();
        lastAttributes = feat.attributes();
        anyMergeOccurred = true;
      }
    }
  }

  // 3. 执行最终融合 (解决要素消失的关键点)
  QgsGeometry finalGeom;
  if ( anyMergeOccurred )
  {
    finalGeom = QgsGeometry::unaryUnion( allGeoms );

    // --- 修正编译错误的部分 ---
    // 检查是否为面要素。如果不是面（例如产生了零面积的线），强行转换
    if ( finalGeom.type() != Qgis::GeometryType::Polygon )
    {
      finalGeom = finalGeom.convertToType( Qgis::GeometryType::Polygon );
    }

    finalGeom = finalGeom.makeValid();

    // 兜底机制：如果融合后几何完全坏了（为空），则返回初始面 A，确保要素不消失
    if ( finalGeom.isEmpty() || finalGeom.isNull() )
    {
      finalGeom = geomA;
      anyMergeOccurred = false;
    }
  }
  else
  {
    finalGeom = geomA;
  }

  // 4. 数据落地
  if ( !mVectorLayer->isEditable() )
    mVectorLayer->startEditing();

  if ( anyMergeOccurred && !idsToDelete.isEmpty() )
  {
    mVectorLayer->deleteFeatures( idsToDelete );
  }

  QgsFeature f( mVectorLayer->fields() );
  f.setGeometry( finalGeom );
  if ( anyMergeOccurred )
  {
    f.setAttributes( lastAttributes );
  }

  // 5. 高度计算
  if ( mUI.heightcheckBox && mUI.heightcheckBox->isChecked() && mPCLayer )
  {
    double z = calculateZFromPointCloud( finalGeom );
    int idx = mVectorLayer->fields().indexOf( mTargetFieldName );
    if ( idx != -1 )
      f.setAttribute( idx, z );
  }

  if ( mVectorLayer->addFeature( f ) )
  {
    mVectorLayer->triggerRepaint();
  }

  cancelDigitizing();
}

void CreateTool::keyPressEvent( QKeyEvent *e )
{
  if ( e->key() == Qt::Key_Escape )
  {
    cancelDigitizing();
  }
  else if ( e->key() == Qt::Key_Return || e->key() == Qt::Key_Enter )
  {
    // 回车提交图层编辑，不再计算高度
    if ( mVectorLayer && mVectorLayer->isEditable() )
    {
      mVectorLayer->commitChanges();
      qDebug() << "Vector layer changes committed.";
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
