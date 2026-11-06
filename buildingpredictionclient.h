#pragma once

#include <QJsonObject>
#include <QObject>
#include <QPointer>
#include <QTimer>
#include <QUrl>
#include <functional>
#include <memory>

class QCryptographicHash;
class QNetworkReply;
class QNetworkAccessManager;
class QSaveFile;

// File/job transport only. Never accesses project layers from a worker thread.
class BuildingPredictionClient : public QObject
{
    Q_OBJECT
  public:
    explicit BuildingPredictionClient( QObject *parent = nullptr );
    ~BuildingPredictionClient() override;
    bool isConnected() const { return mConnected; }
    bool isActive() const { return mActive || mTesting; }
    void invalidateConnection() { if ( !isActive() ) mConnected = false; }
    void checkConnection( const QUrl &url, const QString &token );
    void startJob( const QString &pointCloud, const QString &targetSnapshot,
                   const QJsonObject &options, const QString &localDirectory );
    void resumeJob( const QString &jobId, const QString &localDirectory );

  signals:
    void connectionChecked( bool connected, const QString &message );
    void activeChanged();
    void progressChanged( const QString &message, int percent );
    void jobAssigned( const QString &jobId );
    void resultReady( const QString &jobId, const QString &file, const QJsonObject &manifest );
    void failed( const QString &message );

  private:
    using JsonCallback = std::function<void( const QJsonObject & )>;
    void jsonRequest( const QByteArray &method, const QString &route, const QJsonObject &body, JsonCallback callback );
    void upload( const QString &path, const QString &kind, std::function<void()> next );
    void poll();
    void download( const QJsonObject &manifest );
    void fail( const QString &message );
    QUrl endpoint( const QString &route ) const;
    QString jobRoute() const;

    QUrl mBaseUrl;
    QByteArray mToken;
    bool mConnected = false;
    bool mActive = false;
    bool mTesting = false;
    QString mJobId;
    QString mDirectory;
    QTimer mPollTimer;
    QNetworkAccessManager *mNetwork = nullptr;
    QPointer<QNetworkReply> mReply;
    std::unique_ptr<QSaveFile> mDownload;
    std::unique_ptr<QCryptographicHash> mDownloadHash;
    qint64 mDownloaded = 0;
};
