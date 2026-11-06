#include "buildingpredictionclient.h"

#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QNetworkReply>
#include <QNetworkAccessManager>
#include <QNetworkProxy>
#include <QNetworkRequest>
#include <QSaveFile>
#include <QUuid>

namespace
{
  QString responseError( QNetworkReply *reply, const QByteArray &body )
  {
    const QJsonObject object = QJsonDocument::fromJson( body ).object();
    const QString detail = object.value( QStringLiteral( "detail" ) ).toString();
    return detail.isEmpty() ? reply->errorString() : detail;
  }
}

BuildingPredictionClient::BuildingPredictionClient( QObject *parent )
  : QObject( parent )
{
  // Lab requests use the explicitly configured endpoint directly. A dedicated
  // manager avoids QGIS/system proxy factories (including incompatible proxy
  // types) without changing proxy behavior for any other part of QGIS.
  mNetwork = new QNetworkAccessManager( this );
  mNetwork->setProxy( QNetworkProxy( QNetworkProxy::NoProxy ) );
  mPollTimer.setInterval( 2000 );
  connect( &mPollTimer, &QTimer::timeout, this, &BuildingPredictionClient::poll );
}

BuildingPredictionClient::~BuildingPredictionClient()
{
  mPollTimer.stop();
  if ( mReply )
  {
    disconnect( mReply, nullptr, this, nullptr );
    mReply->abort();
  }
  if ( mDownload )
    mDownload->cancelWriting();
  // Deliberately never requests termination/deletion of any remote job.
}

QUrl BuildingPredictionClient::endpoint( const QString &route ) const
{
  QUrl url = mBaseUrl;
  QString prefix = url.path();
  while ( prefix.endsWith( '/' ) )
    prefix.chop( 1 );
  url.setPath( prefix + route );
  return url;
}

QString BuildingPredictionClient::jobRoute() const
{
  return QStringLiteral( "/v1/jobs/" ) + mJobId;
}

void BuildingPredictionClient::checkConnection( const QUrl &url, const QString &token )
{
  if ( isActive() )
    return;
  mConnected = false;
  if ( !url.isValid() || url.host().isEmpty() ||
       ( url.scheme() != QLatin1String( "http" ) && url.scheme() != QLatin1String( "https" ) ) ||
       !url.userInfo().isEmpty() || url.hasQuery() || url.hasFragment() || token.trimmed().isEmpty() )
  {
    emit connectionChecked( false, tr( "请填写有效的服务地址和访问密钥。" ) );
    return;
  }
  mBaseUrl = url;
  mToken = token.trimmed().toUtf8();
  mTesting = true;
  emit activeChanged();
  jsonRequest( "GET", QStringLiteral( "/v1/health" ), {}, [this]( const QJsonObject &object ) {
    mTesting = false;
    mConnected = object.value( QStringLiteral( "api_version" ) ).toInt() == 1 &&
                 object.value( QStringLiteral( "service" ) ).toString() == QLatin1String( "spt-buildings" ) &&
                 object.value( QStringLiteral( "checkpoint_exists" ) ).toBool();
    emit connectionChecked( mConnected, mConnected ? tr( "实验室服务已连接" ) : tr( "服务版本不匹配，或实验室缺少模型权重。" ) );
    emit activeChanged();
  } );
}

void BuildingPredictionClient::jsonRequest( const QByteArray &method, const QString &route,
                                           const QJsonObject &body, JsonCallback callback )
{
  QNetworkRequest request( endpoint( route ) );
  request.setRawHeader( "Authorization", "Bearer " + mToken );
  request.setHeader( QNetworkRequest::ContentTypeHeader, QStringLiteral( "application/json" ) );
  request.setAttribute( QNetworkRequest::RedirectPolicyAttribute, QNetworkRequest::ManualRedirectPolicy );
  request.setTransferTimeout( 30000 );
  QNetworkReply *reply = method == "GET"
                          ? mNetwork->get( request )
                          : mNetwork->post( request, QJsonDocument( body ).toJson( QJsonDocument::Compact ) );
  mReply = reply;
  connect( reply, &QNetworkReply::finished, this, [this, reply, callback]() {
    const QByteArray bytes = reply->readAll();
    const int status = reply->attribute( QNetworkRequest::HttpStatusCodeAttribute ).toInt();
    const bool ok = reply->error() == QNetworkReply::NoError && status >= 200 && status < 300;
    const QString error = responseError( reply, bytes );
    if ( mReply == reply )
      mReply.clear();
    reply->deleteLater();
    if ( !ok )
    {
      fail( tr( "服务请求失败：%1" ).arg( error ) );
      return;
    }
    QJsonParseError parse;
    const QJsonDocument document = QJsonDocument::fromJson( bytes, &parse );
    if ( parse.error != QJsonParseError::NoError || !document.isObject() )
    {
      fail( tr( "服务器返回了无效的任务数据。" ) );
      return;
    }
    callback( document.object() );
  } );
}

void BuildingPredictionClient::startJob( const QString &pointCloud, const QString &targetSnapshot,
                                        const QJsonObject &options, const QString &localDirectory )
{
  if ( !mConnected || isActive() )
    return;
  mActive = true;
  mJobId.clear();
  mDirectory = localDirectory;
  emit activeChanged();
  emit progressChanged( tr( "创建实验室任务" ), 0 );
  jsonRequest( "POST", QStringLiteral( "/v1/jobs" ), options,
               [this, pointCloud, targetSnapshot]( const QJsonObject &object ) {
    mJobId = object.value( QStringLiteral( "job_id" ) ).toString();
    if ( QUuid( mJobId ).isNull() || QUuid( mJobId ).toString( QUuid::WithoutBraces ) != mJobId )
    {
      fail( tr( "服务器返回的任务编号无效。" ) );
      return;
    }
    emit jobAssigned( mJobId );
    upload( pointCloud, QStringLiteral( "pointcloud" ), [this, targetSnapshot]() {
      upload( targetSnapshot, QStringLiteral( "target" ), [this]() {
        jsonRequest( "POST", jobRoute() + QStringLiteral( "/start" ), {}, [this]( const QJsonObject & ) {
          mPollTimer.start();
          poll();
        } );
      } );
    } );
  } );
}

void BuildingPredictionClient::upload( const QString &path, const QString &kind, std::function<void()> next )
{
  QFile *file = new QFile( path );
  if ( !file->open( QIODevice::ReadOnly ) || file->size() <= 0 )
  {
    delete file;
    fail( tr( "无法读取上传文件：%1" ).arg( path ) );
    return;
  }
  const qint64 expected = file->size();
  QNetworkRequest request( endpoint( jobRoute() + '/' + kind ) );
  request.setRawHeader( "Authorization", "Bearer " + mToken );
  request.setHeader( QNetworkRequest::ContentTypeHeader, QStringLiteral( "application/octet-stream" ) );
  request.setHeader( QNetworkRequest::ContentLengthHeader, expected );
  request.setAttribute( QNetworkRequest::DoNotBufferUploadDataAttribute, true );
  request.setAttribute( QNetworkRequest::RedirectPolicyAttribute, QNetworkRequest::ManualRedirectPolicy );
  request.setTransferTimeout( 300000 );
  QNetworkReply *reply = mNetwork->put( request, file );
  file->setParent( reply );
  mReply = reply;
  const QString label = kind == QLatin1String( "pointcloud" ) ? tr( "上传点云" ) : tr( "上传目标图层快照" );
  emit progressChanged( label, 0 );
  connect( reply, &QNetworkReply::uploadProgress, this, [this, label]( qint64 sent, qint64 total ) {
    emit progressChanged( label, total > 0 ? static_cast<int>( 100.0 * sent / total ) : 0 );
  } );
  connect( reply, &QNetworkReply::finished, this, [this, reply, expected, next]() {
    const QByteArray bytes = reply->readAll();
    const int code = reply->attribute( QNetworkRequest::HttpStatusCodeAttribute ).toInt();
    const QJsonObject response = QJsonDocument::fromJson( bytes ).object();
    const bool ok = reply->error() == QNetworkReply::NoError && code >= 200 && code < 300 &&
                    static_cast<qint64>( response.value( QStringLiteral( "bytes" ) ).toDouble() ) == expected;
    const QString error = responseError( reply, bytes );
    mReply.clear();
    reply->deleteLater();
    if ( !ok )
    {
      fail( tr( "上传失败或文件不完整：%1。远端文件已保留，可重新开始一个任务。" ).arg( error ) );
      return;
    }
    next();
  } );
}

void BuildingPredictionClient::resumeJob( const QString &jobId, const QString &localDirectory )
{
  if ( !mConnected || isActive() || QUuid( jobId ).isNull() )
    return;
  mDirectory = localDirectory;
  mJobId = jobId;
  mActive = true;
  emit activeChanged();
  mPollTimer.start();
  poll();
}

void BuildingPredictionClient::poll()
{
  if ( !mActive || mReply )
    return;
  jsonRequest( "GET", jobRoute() + QStringLiteral( "/status" ), {}, [this]( const QJsonObject &object ) {
    const QString state = object.value( QStringLiteral( "status" ) ).toString();
    const QJsonObject progress = object.value( QStringLiteral( "progress" ) ).toObject();
    if ( state == QLatin1String( "failed" ) || state == QLatin1String( "interrupted" ) || state == QLatin1String( "uploading" ) )
    {
      fail( progress.value( QStringLiteral( "message" ) ).toString(
              object.value( QStringLiteral( "message" ) ).toString( tr( "任务上传未完成，请重新开始。" ) ) ) );
      return;
    }
    if ( state == QLatin1String( "completed" ) )
    {
      mPollTimer.stop();
      jsonRequest( "GET", jobRoute() + QStringLiteral( "/result-info" ), {}, [this]( const QJsonObject &manifest ) { download( manifest ); } );
      return;
    }
    if ( state != QLatin1String( "queued" ) && state != QLatin1String( "running" ) )
    {
      fail( tr( "未知的远程任务状态：%1" ).arg( state ) );
      return;
    }
    emit progressChanged( progress.value( QStringLiteral( "message" ) ).toString(
                            object.value( QStringLiteral( "message" ) ).toString() ),
                          qBound( 0, static_cast<int>( progress.value( QStringLiteral( "percent" ) ).toDouble() ), 100 ) );
  } );
}

void BuildingPredictionClient::download( const QJsonObject &manifest )
{
  const qint64 expected = static_cast<qint64>( manifest.value( QStringLiteral( "bytes" ) ).toDouble() );
  const QByteArray digest = manifest.value( QStringLiteral( "sha256" ) ).toString().toLatin1();
  if ( !manifest.value( QStringLiteral( "complete" ) ).toBool() || expected <= 0 || digest.size() != 64 ||
       manifest.value( QStringLiteral( "job_id" ) ).toString() != mJobId ||
       manifest.value( QStringLiteral( "new_layer" ) ).toString() != QLatin1String( "predicted_buildings" ) )
  {
    fail( tr( "结果清单不完整，拒绝写入目标图层。" ) );
    return;
  }
  const QString output = QDir( mDirectory ).filePath( QStringLiteral( "result.gpkg" ) );
  mDownload.reset( new QSaveFile( output ) );
  mDownload->setDirectWriteFallback( false );
  if ( !mDownload->open( QIODevice::WriteOnly ) )
  {
    fail( tr( "无法创建本地结果缓存。" ) );
    return;
  }
  mDownloaded = 0;
  mDownloadHash.reset( new QCryptographicHash( QCryptographicHash::Sha256 ) );
  QNetworkRequest request( endpoint( jobRoute() + QStringLiteral( "/result" ) ) );
  request.setRawHeader( "Authorization", "Bearer " + mToken );
  request.setAttribute( QNetworkRequest::RedirectPolicyAttribute, QNetworkRequest::ManualRedirectPolicy );
  request.setTransferTimeout( 300000 );
  QNetworkReply *reply = mNetwork->get( request );
  reply->setReadBufferSize( 1024 * 1024 );
  mReply = reply;
  auto consume = [this, reply]() {
    const QByteArray block = reply->readAll();
    if ( !mActive || !mDownload || block.isEmpty() )
      return;
    if ( mDownload->write( block ) != block.size() )
    {
      fail( tr( "本地磁盘写入失败，目标图层未修改。" ) );
      return;
    }
    mDownloadHash->addData( block );
    mDownloaded += block.size();
  };
  connect( reply, &QNetworkReply::readyRead, this, consume );
  connect( reply, &QNetworkReply::downloadProgress, this, [this]( qint64 received, qint64 total ) {
    emit progressChanged( tr( "下载拟合与高程结果" ), total > 0 ? static_cast<int>( 100.0 * received / total ) : 0 );
  } );
  connect( reply, &QNetworkReply::finished, this, [this, reply, consume, manifest, output, expected, digest]() {
    consume();
    const int code = reply->attribute( QNetworkRequest::HttpStatusCodeAttribute ).toInt();
    const bool transportOk = reply->error() == QNetworkReply::NoError && code == 200;
    mReply.clear();
    reply->deleteLater();
    if ( !mActive )
      return;
    if ( !transportOk || mDownloaded != expected || mDownloadHash->result().toHex() != digest || !mDownload->commit() )
    {
      fail( tr( "结果下载失败或 SHA256 校验不通过。目标图层未修改，可恢复任务重新下载。" ) );
      return;
    }
    mDownload.reset();
    QSaveFile info( QDir( mDirectory ).filePath( QStringLiteral( "result.json" ) ) );
    const QByteArray infoBytes = QJsonDocument( manifest ).toJson();
    if ( !info.open( QIODevice::WriteOnly ) || info.write( infoBytes ) != infoBytes.size() || !info.commit() )
    {
      fail( tr( "无法保存结果清单，已下载的结果保留在：%1" ).arg( output ) );
      return;
    }
    mActive = false;
    emit activeChanged();
    emit resultReady( mJobId, output, manifest );
  } );
}

void BuildingPredictionClient::fail( const QString &message )
{
  mPollTimer.stop();
  const bool testing = mTesting;
  mTesting = false;
  mActive = false;
  if ( mDownload )
  {
    mDownload->cancelWriting();
    mDownload.reset();
  }
  if ( mReply )
  {
    QNetworkReply *reply = mReply;
    mReply.clear();
    reply->abort();
  }
  if ( testing )
    emit connectionChecked( false, message );
  emit activeChanged();
  emit failed( message );
}
