#pragma once

#include <qgsfeature.h>
#include <qgsmaptool.h>
#include <QPointer>
#include <QWidget>

#include "roofmodelsettings.h"
#include "ui_roofedit.h"

class QgisInterface;
class QgsMapCanvas;
class QgsPointCloudLayer;
class QgsVectorLayer;
class QTimer;

class RoofEditTool : public QgsMapTool
{
    Q_OBJECT

  public:
    explicit RoofEditTool( QgsMapCanvas *canvas, QgisInterface *iface );
    ~RoofEditTool() override;
    void activate() override;
    void deactivate() override;
    void canvasPressEvent( QgsMapMouseEvent *event ) override;

  protected:
    bool eventFilter( QObject *object, QEvent *event ) override;

  private:
    void setupUi();
    void refreshLayerRefs();
    bool isCandidateBuildingLayer( QgsVectorLayer *layer ) const;
    void refreshPointCloudCombo();
    void hideLegacyPointLayers();
    bool selectBuildingAt( const QgsPointXY &point );
    void loadRoofSelection();
    RoofModelSettings::Selection currentSelection() const;
    void scheduleRoofPreview();
    void previewRoofSelection();
    void clearRoofPreview();
    void deleteRoofModel();
    void saveRoofModel();
    void notifyRoofModelChanged();
    void setTriangleMeshModeEnabled( bool enabled );
    void updateTriangleMeshModeForCurrentBuilding();

    QPointer<QgisInterface> mIface;
    QPointer<QgsMapCanvas> mCanvas;
    QPointer<QWidget> mWidget;
    Ui::roofedit mUI;
    QPointer<QgsVectorLayer> mActiveLayer;
    QgsFeature mCurrentBuilding;
    QTimer *mRoofPreviewTimer = nullptr;
};
