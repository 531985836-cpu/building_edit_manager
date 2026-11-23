#pragma once

#include "buildingroof.h"

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
    void roofPreviewChanged( QgsVectorLayer *layer, QgsFeatureId fid, const BuildingRoof::Mesh &mesh, bool dragging );
    void roofModelChanged( QgsVectorLayer *layer, QgsFeatureId fid );
    void buildingTriangleMeshModeChanged( QgsVectorLayer *layer, QgsFeatureId fid, bool enabled );
};
