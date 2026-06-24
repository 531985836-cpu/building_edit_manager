#pragma once

#include <QObject>
#include <qgsfeatureid.h>

class QgsVectorLayer;

class BuildingEditPreviewBus : public QObject
{
    Q_OBJECT

  public:
    static BuildingEditPreviewBus *instance();

  signals:
    void heightPreviewChanged( QgsVectorLayer *layer, const QgsFeatureIds &fids, const QString &heightFieldName, double height );
    void heightPreviewFinished( QgsVectorLayer *layer, const QgsFeatureIds &fids, const QString &heightFieldName, double height );
    void roofModelChanged( QgsVectorLayer *layer, QgsFeatureId fid );
};
