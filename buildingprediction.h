#pragma once

#include <QObject>
#include <QString>
#include <QJsonObject>

class QgsMapLayer;
class QgsPointCloudLayer;
class QgsVectorLayer;
class BuildingPredictionClient;

namespace Ui
{
  class Form;
}

// Main-thread UI, layer snapshots and result import. HTTP transport is separate.
class BuildingPrediction : public QObject
{
    Q_OBJECT

  public:
    explicit BuildingPrediction( Ui::Form &ui, QObject *parent = nullptr );

    QgsPointCloudLayer *pointCloudLayer() const;
    QgsVectorLayer *targetLayer() const;
    bool heightInitializationEnabled() const;
    QString heightField() const;

  signals:
    void settingsChanged();

  private:
    void setTargetLayer( QgsMapLayer *layer );
    void updateHeightControls();
    void updateActions();
    void startPrediction();
    void resumePrediction();
    void importResult( const QString &jobId, const QString &file, const QJsonObject &manifest );
    bool saveTask();
    bool loadTask();
    void showError( const QString &message );

    Ui::Form &mUi;
    QMetaObject::Connection mFieldsChangedConnection;
    BuildingPredictionClient *mClient = nullptr;
    QJsonObject mTask;
    QString mDirectory;
};
