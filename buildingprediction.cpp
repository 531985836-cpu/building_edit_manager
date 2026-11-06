#include "buildingprediction.h"
#include "buildingpredictionclient.h"
#include "ui_createtool.h"

#include <qgis.h>
#include <qgsfieldproxymodel.h>
#include <qgsmaplayer.h>
#include <qgspointcloudlayer.h>
#include <qgsvectorlayer.h>
#include <qgsvectordataprovider.h>
#include <qgsvectorfilewriter.h>
#include <qgsvectorlayerutils.h>
#include <qgsproviderregistry.h>
#include <qgsproject.h>
#include <qgssettings.h>
#include <qgscoordinatetransform.h>
#include <qgseditformconfig.h>
#include <QCryptographicHash>
#include <QDir>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QMessageBox>
#include <QSaveFile>
#include <QStandardPaths>
#include <QUuid>

BuildingPrediction::BuildingPrediction( Ui::Form &ui, QObject *parent )
  : QObject( parent )
  , mUi( ui )
{
  mClient = new BuildingPredictionClient( this );
  // Native QGIS models follow project layer additions, removals and renames.
  mUi.predictionPointCloudComboBox->setFilters( Qgis::LayerFilter::PointCloudLayer );
  mUi.predictionPointCloudComboBox->setAllowEmptyLayer( true, tr( "请选择点云图层..." ) );
  mUi.predictionPointCloudComboBox->setLayer( nullptr );

  mUi.predictionTargetLayerComboBox->setFilters( Qgis::LayerFilter::PolygonLayer | Qgis::LayerFilter::WritableLayer );
  mUi.predictionTargetLayerComboBox->setAllowEmptyLayer( true, tr( "请选择目标面图层..." ) );
  mUi.predictionTargetLayerComboBox->setLayer( nullptr );

  mUi.predictionHeightFieldComboBox->setFilters( QgsFieldProxyModel::Numeric | QgsFieldProxyModel::HideReadOnly );
  mUi.predictionHeightFieldComboBox->setAllowEmptyFieldName( true );

  connect( mUi.predictionPointCloudComboBox, &QgsMapLayerComboBox::layerChanged,
           this, &BuildingPrediction::settingsChanged );
  connect( mUi.predictionTargetLayerComboBox, &QgsMapLayerComboBox::layerChanged,
           this, &BuildingPrediction::setTargetLayer );
  connect( mUi.predictionHeightFieldComboBox, &QgsFieldComboBox::fieldChanged,
           this, &BuildingPrediction::settingsChanged );
  connect( mUi.predictionHeightCheckBox, &QCheckBox::toggled, this, [this]() {
    updateHeightControls();
    emit settingsChanged();
  } );

  QgsSettings settings;
  mUi.predictionServiceUrlLineEdit->setText( settings.value( QStringLiteral( "buildingPrediction/serviceUrl" ), QStringLiteral( "http://127.0.0.1:8000" ) ).toString() );
  mDirectory = settings.value( QStringLiteral( "buildingPrediction/lastDirectory" ) ).toString();
  connect( this, &BuildingPrediction::settingsChanged, this, &BuildingPrediction::updateActions );
  connect( mUi.predictionConnectButton, &QPushButton::clicked, this, [this]() {
    mUi.predictionStatusLabel->setText( tr( "正在测试实验室服务连接..." ) );
    mClient->checkConnection( QUrl( mUi.predictionServiceUrlLineEdit->text().trimmed() ), mUi.predictionTokenLineEdit->text() );
  } );
  for ( QLineEdit *edit : {mUi.predictionServiceUrlLineEdit, mUi.predictionTokenLineEdit} )
  {
    connect( edit, &QLineEdit::textChanged, this, [this]() {
      mClient->invalidateConnection();
      mUi.predictionStatusLabel->setText( tr( "服务设置已改变，请重新测试连接。" ) );
      updateActions();
    } );
  }
  connect( mClient, &BuildingPredictionClient::connectionChecked, this, [this]( bool connected, const QString &message ) {
    mUi.predictionStatusLabel->setText( message );
    if ( connected )
    {
      QgsSettings settings;
      settings.setValue( QStringLiteral( "buildingPrediction/serviceUrl" ), mUi.predictionServiceUrlLineEdit->text().trimmed() );
    }
    updateActions();
  } );
  connect( mClient, &BuildingPredictionClient::activeChanged, this, &BuildingPrediction::updateActions );
  connect( mClient, &BuildingPredictionClient::progressChanged, this, [this]( const QString &message, int percent ) {
    mUi.predictionStatusLabel->setText( message );
    mUi.predictionProgressBar->setRange( 0, 100 );
    mUi.predictionProgressBar->setValue( percent );
  } );
  connect( mClient, &BuildingPredictionClient::failed, this, &BuildingPrediction::showError );
  connect( mClient, &BuildingPredictionClient::jobAssigned, this, [this]( const QString &jobId ) {
    mTask.insert( QStringLiteral( "job_id" ), jobId );
    saveTask();
  } );
  connect( mClient, &BuildingPredictionClient::resultReady, this, &BuildingPrediction::importResult );
  connect( mUi.startPredictionButton, &QPushButton::clicked, this, &BuildingPrediction::startPrediction );
  connect( mUi.resumePredictionButton, &QPushButton::clicked, this, &BuildingPrediction::resumePrediction );

  setTargetLayer( nullptr );
}

QgsPointCloudLayer *BuildingPrediction::pointCloudLayer() const
{
  return qobject_cast<QgsPointCloudLayer *>( mUi.predictionPointCloudComboBox->currentLayer() );
}

QgsVectorLayer *BuildingPrediction::targetLayer() const
{
  return qobject_cast<QgsVectorLayer *>( mUi.predictionTargetLayerComboBox->currentLayer() );
}

bool BuildingPrediction::heightInitializationEnabled() const
{
  return mUi.predictionHeightCheckBox->isChecked();
}

QString BuildingPrediction::heightField() const
{
  return heightInitializationEnabled() ? mUi.predictionHeightFieldComboBox->currentField() : QString();
}

void BuildingPrediction::setTargetLayer( QgsMapLayer *layer )
{
  disconnect( mFieldsChangedConnection );
  mUi.predictionHeightFieldComboBox->setLayer( layer );
  // Do not silently choose a numeric identifier when no height field exists.
  mUi.predictionHeightFieldComboBox->setField( QString() );
  if ( QgsVectorLayer *vectorLayer = qobject_cast<QgsVectorLayer *>( layer ) )
  {
    mUi.predictionHeightFieldComboBox->setField( QStringLiteral( "height" ) );
    // The field model itself tracks added/deleted/renamed fields and rollbacks.
    mFieldsChangedConnection = connect( vectorLayer, &QgsVectorLayer::updatedFields, this, [this]() {
      updateHeightControls();
      emit settingsChanged();
    } );
  }
  updateHeightControls();
  updateActions();
  emit settingsChanged();
}

void BuildingPrediction::updateHeightControls()
{
  const bool enabled = heightInitializationEnabled() && targetLayer();
  mUi.predictionHeightFieldLabel->setEnabled( enabled );
  mUi.predictionHeightFieldComboBox->setEnabled( enabled );
}

void BuildingPrediction::updateActions()
{
  const bool busy = mClient->isActive();
  mUi.predictionServiceGroup->setEnabled( !busy );
  mUi.predictionInputGroup->setEnabled( !busy );
  mUi.predictionHeightGroup->setEnabled( !busy );
  mUi.startPredictionButton->setEnabled( !busy && mClient->isConnected() && pointCloudLayer() && targetLayer() &&
                                        ( !heightInitializationEnabled() || !heightField().isEmpty() ) );
  mUi.resumePredictionButton->setEnabled( !busy && mClient->isConnected() &&
                                         QFileInfo::exists( QDir( mDirectory ).filePath( QStringLiteral( "task.json" ) ) ) );
}

void BuildingPrediction::showError( const QString &message )
{
  mUi.predictionStatusLabel->setText( message );
  updateActions();
}

bool BuildingPrediction::saveTask()
{
  QSaveFile file( QDir( mDirectory ).filePath( QStringLiteral( "task.json" ) ) );
  const QByteArray bytes = QJsonDocument( mTask ).toJson();
  const bool ok = file.open( QIODevice::WriteOnly ) && file.write( bytes ) == bytes.size() && file.commit();
  if ( ok )
  {
    QgsSettings settings;
    settings.setValue( QStringLiteral( "buildingPrediction/lastDirectory" ), mDirectory );
  }
  else
    showError( tr( "无法保存任务记录。请检查本机缓存目录权限：%1" ).arg( mDirectory ) );
  return ok;
}

bool BuildingPrediction::loadTask()
{
  QFile file( QDir( mDirectory ).filePath( QStringLiteral( "task.json" ) ) );
  if ( !file.open( QIODevice::ReadOnly ) )
    return false;
  mTask = QJsonDocument::fromJson( file.readAll() ).object();
  return !mTask.value( QStringLiteral( "job_id" ) ).toString().isEmpty();
}

void BuildingPrediction::startPrediction()
{
  QgsPointCloudLayer *cloud = pointCloudLayer();
  QgsVectorLayer *target = targetLayer();
  if ( !cloud || !cloud->isValid() || !target || !target->isValid() || !target->crs().isValid() ||
       !target->dataProvider() || target->readOnly() ||
       !target->dataProvider()->capabilities().testFlag( Qgis::VectorProviderCapability::AddFeatures ) )
  {
    showError( tr( "请选择有效点云和允许新增要素、具有有效坐标系的目标面图层。" ) );
    return;
  }
  const QVariantMap parts = QgsProviderRegistry::instance()->decodeUri( cloud->providerType(), cloud->source() );
  QString cloudPath = parts.value( QStringLiteral( "path" ), cloud->source() ).toString();
  const QUrl cloudUrl( cloudPath );
  if ( cloudUrl.isLocalFile() )
    cloudPath = cloudUrl.toLocalFile();
  if ( !cloudPath.endsWith( QLatin1String( ".copc.laz" ), Qt::CaseInsensitive ) || !QFileInfo( cloudPath ).isFile() )
  {
    showError( tr( "当前远程入口需要本机的 .copc.laz 文件。普通 LAS/LAZ、EPT 和网络点云需先转换为 COPC。" ) );
    return;
  }
  const QString height = heightField();
  if ( heightInitializationEnabled() && ( height.isEmpty() || target->primaryKeyAttributes().contains( target->fields().indexFromName( height ) ) ) )
  {
    showError( tr( "请选择用于保存屋顶高程的数值字段，不能使用图层主键。" ) );
    return;
  }
  QJsonArray fields, signature;
  for ( int index = 0; index < target->fields().size(); ++index )
  {
    const QgsField field = target->fields().at( index );
    signature.append( QJsonObject{{QStringLiteral( "name" ), field.name()}, {QStringLiteral( "type" ), static_cast<int>( field.type() )}} );
    const Qgis::FieldOrigin origin = target->fields().fieldOrigin( index );
    if ( origin != Qgis::FieldOrigin::Expression && origin != Qgis::FieldOrigin::Join && !target->editFormConfig().readOnly( index ) )
      fields.append( field.name() );
  }
  if ( !height.isEmpty() && !fields.contains( height ) )
  {
    showError( tr( "所选高程字段不可写。" ) );
    return;
  }
  mDirectory = QDir( QStandardPaths::writableLocation( QStandardPaths::AppLocalDataLocation ) ).filePath(
                 QStringLiteral( "building_prediction/" ) + QUuid::createUuid().toString( QUuid::WithoutBraces ) );
  if ( !QDir().mkpath( mDirectory ) )
  {
    showError( tr( "无法创建本机任务缓存目录。" ) );
    return;
  }
  const QString snapshot = QDir( mDirectory ).filePath( QStringLiteral( "target.gpkg" ) );
  QgsVectorFileWriter::SaveVectorOptions options;
  options.driverName = QStringLiteral( "GPKG" );
  options.layerName = QStringLiteral( "target" );
  options.fileEncoding = QStringLiteral( "UTF-8" );
  QString fid = QStringLiteral( "__spt_snapshot_id" );
  while ( target->fields().indexFromName( fid ) >= 0 )
    fid += '_';
  options.layerOptions = QStringList{QStringLiteral( "FID=" ) + fid};
  QString error;
  // A read-only snapshot includes current edit-buffer features. Never uploads
  // the original datasource file or modifies the user's target at this stage.
  mUi.predictionStatusLabel->setText( tr( "正在创建目标图层快照..." ) );
  if ( QgsVectorFileWriter::writeAsVectorFormatV3( target, snapshot, QgsProject::instance()->transformContext(), options, &error ) != QgsVectorFileWriter::NoError )
  {
    showError( tr( "目标图层快照导出失败：%1" ).arg( error ) );
    return;
  }
  mTask = QJsonObject{{QStringLiteral( "target_id" ), target->id()},
                     {QStringLiteral( "target_source_sha256" ), QString::fromLatin1( QCryptographicHash::hash( target->source().toUtf8(), QCryptographicHash::Sha256 ).toHex() )},
                     {QStringLiteral( "target_crs_wkt" ), target->crs().toWkt()},
                     {QStringLiteral( "field_signature" ), signature}, {QStringLiteral( "target_fields" ), fields},
                     {QStringLiteral( "height_field" ), height},
                     {QStringLiteral( "server_url" ), mUi.predictionServiceUrlLineEdit->text().trimmed()},
                     {QStringLiteral( "imported" ), false}};
  if ( !saveTask() )
    return;
  mClient->startJob( cloudPath, snapshot,
                    QJsonObject{{QStringLiteral( "pointcloud_name" ), QFileInfo( cloudPath ).fileName()},
                                {QStringLiteral( "height_field" ), height}, {QStringLiteral( "target_fields" ), fields}}, mDirectory );
}

void BuildingPrediction::resumePrediction()
{
  if ( !loadTask() )
  {
    showError( tr( "没有可恢复的任务记录。" ) );
    return;
  }
  if ( mTask.value( QStringLiteral( "imported" ) ).toBool() )
  {
    showError( tr( "该任务已写入，不会重复添加。结果缓存：%1" ).arg( mDirectory ) );
    return;
  }
  if ( mTask.value( QStringLiteral( "server_url" ) ).toString() != mUi.predictionServiceUrlLineEdit->text().trimmed() )
  {
    showError( tr( "请连接该任务原来的服务地址，再恢复任务。" ) );
    return;
  }
  mClient->resumeJob( mTask.value( QStringLiteral( "job_id" ) ).toString(), mDirectory );
}

void BuildingPrediction::importResult( const QString &jobId, const QString &file, const QJsonObject &manifest )
{
  QgsVectorLayer *target = qobject_cast<QgsVectorLayer *>( QgsProject::instance()->mapLayer( mTask.value( QStringLiteral( "target_id" ) ).toString() ) );
  if ( !target || !target->isValid() || target->readOnly() || !target->dataProvider() ||
       !target->dataProvider()->capabilities().testFlag( Qgis::VectorProviderCapability::AddFeatures ) ||
       target->crs().toWkt() != mTask.value( QStringLiteral( "target_crs_wkt" ) ).toString() ||
       QString::fromLatin1( QCryptographicHash::hash( target->source().toUtf8(), QCryptographicHash::Sha256 ).toHex() ) != mTask.value( QStringLiteral( "target_source_sha256" ) ).toString() )
  {
    showError( tr( "原目标图层已移除或数据源/坐标系改变，结果未自动写入。缓存保留在：%1" ).arg( file ) );
    return;
  }
  QStringList applied = target->customProperty( QStringLiteral( "buildingPrediction/importedJobs" ) ).toStringList();
  if ( applied.contains( jobId ) || mTask.value( QStringLiteral( "imported" ) ).toBool() )
  {
    showError( tr( "该任务已经写入，已阻止重复添加。" ) );
    return;
  }
  for ( const QJsonValue &entry : mTask.value( QStringLiteral( "field_signature" ) ).toArray() )
  {
    const QJsonObject field = entry.toObject();
    const int index = target->fields().indexFromName( field.value( QStringLiteral( "name" ) ).toString() );
    if ( index < 0 || static_cast<int>( target->fields().at( index ).type() ) != field.value( QStringLiteral( "type" ) ).toInt() )
    {
      showError( tr( "目标图层字段已改变，结果未写入。请检查后恢复任务；结果缓存：%1" ).arg( file ) );
      return;
    }
  }
  QgsVectorLayer source( file + QStringLiteral( "|layername=predicted_buildings" ), QStringLiteral( "remote_prediction" ), QStringLiteral( "ogr" ) );
  const qint64 expected = static_cast<qint64>( manifest.value( QStringLiteral( "feature_count" ) ).toDouble( -1 ) );
  if ( !source.isValid() || !source.crs().isValid() || expected < 0 || source.featureCount() != expected ||
       manifest.value( QStringLiteral( "height_field" ) ).toString() != mTask.value( QStringLiteral( "height_field" ) ).toString() )
  {
    showError( tr( "返回图层、要素数量或高程字段校验失败，目标图层未修改。" ) );
    return;
  }
  const bool ownedEditSession = !target->isEditable();
  if ( ownedEditSession && !target->startEditing() )
  {
    showError( tr( "无法开启目标图层编辑，下载结果已保留。" ) );
    return;
  }
  target->beginEditCommand( tr( "导入实验室建筑预测" ) );
  const QgsAttributeList primaryKeys = target->primaryKeyAttributes();
  const QString height = mTask.value( QStringLiteral( "height_field" ) ).toString();
  qint64 read = 0, added = 0;
  bool ok = true;
  try
  {
    QgsCoordinateTransform transform( source.crs(), target->crs(), QgsProject::instance()->transformContext() );
    QgsExpressionContext context = target->createExpressionContext();
    QgsFeature feature;
    QgsFeatureIterator iterator = source.getFeatures();
    QgsVectorLayerUtils::QgsFeaturesDataList batch;
    auto flush = [&]() {
      QgsFeatureList features = QgsVectorLayerUtils::createFeatures( target, batch, &context );
      if ( features.size() != batch.size() )
        return false;
      const int heightIndex = height.isEmpty() ? -1 : target->fields().indexFromName( height );
      for ( int i = 0; i < features.size(); ++i )
      {
        // An invalid estimate must stay NULL, even if the layer defines a
        // default height. Valid estimates must not be replaced by a default.
        if ( heightIndex >= 0 && batch.at( i ).attributes().contains( heightIndex ) )
          features[i].setAttribute( heightIndex, batch.at( i ).attributes().value( heightIndex ) );
      }
      const bool result = target->addFeatures( features );
      batch.clear();
      return result;
    };
    while ( iterator.nextFeature( feature ) )
    {
      ++read;
      QgsGeometry geometry = feature.geometry();
      if ( geometry.isEmpty() || !geometry.isGeosValid() )
      {
        ok = false;
        break;
      }
      geometry.transform( transform );
      QgsAttributeMap attributes;
      for ( const QJsonValue &value : mTask.value( QStringLiteral( "target_fields" ) ).toArray() )
      {
        const QString name = value.toString();
        const int index = target->fields().indexFromName( name );
        if ( index >= 0 && source.fields().indexFromName( name ) >= 0 && !primaryKeys.contains( index ) && !target->editFormConfig().readOnly( index ) )
        {
          const QVariant attribute = feature.attribute( name );
          if ( !attribute.isNull() || name == height )
            attributes.insert( index, attribute );
        }
      }
      const QVector<QgsGeometry> parts = geometry.coerceToType( target->wkbType() );
      if ( parts.isEmpty() )
      {
        ok = false;
        break;
      }
      for ( const QgsGeometry &part : parts )
      {
        batch.append( QgsVectorLayerUtils::QgsFeatureData( part, attributes ) );
        ++added;
        if ( batch.size() >= 250 )
        {
          ok = flush();
          if ( !ok )
            break;
        }
      }
      if ( !ok )
        break;
    }
    if ( ok && !batch.isEmpty() )
      ok = flush();
  }
  catch ( const QgsCsException & )
  {
    ok = false;
  }
  if ( !ok || read != expected )
  {
    target->destroyEditCommand();
    if ( ownedEditSession )
      target->rollBack();
    showError( tr( "结果导入失败，已撤销本次新增要素。结果文件仍保留在：%1" ).arg( file ) );
    return;
  }
  target->endEditCommand();
  applied.append( jobId );
  target->setCustomProperty( QStringLiteral( "buildingPrediction/importedJobs" ), applied );
  mTask.insert( QStringLiteral( "imported" ), true );
  saveTask();
  bool saved = false;
  const bool canAutoSave = ownedEditSession && QgsProject::instance()->transactionMode() == Qgis::TransactionMode::Disabled;
  if ( canAutoSave )
    saved = target->commitChanges();
  target->updateExtents();
  target->triggerRepaint();
  mUi.predictionProgressBar->setValue( 100 );
  QString message = tr( "已追加 %1 个建筑到“%2”。" ).arg( added ).arg( target->name() );
  if ( canAutoSave && !saved )
    message += tr( "保存失败，请在图层中检查并保存编辑：%1" ).arg( target->commitErrors().join( ';' ) );
  else if ( !canAutoSave )
    message += tr( "请保存图层编辑；已有编辑或工程事务组未自动提交。" );
  else
    message += tr( "已保存。" );
  const int failedHeights = manifest.value( QStringLiteral( "height_failed" ) ).toInt();
  if ( failedHeights )
    message += tr( "其中 %1 个建筑没有有效屋顶高程，字段留空。" ).arg( failedHeights );
  mUi.predictionStatusLabel->setText( message );
  updateActions();
}
