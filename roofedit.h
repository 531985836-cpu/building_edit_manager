#pragma once

#include <qgsfeature.h>
#include <qgsmaptool.h>
#include <QPointer>
#include <QWidget>
#include <memory>

#include "roofmodelsettings.h"
#include "ui_roofedit.h"

class QgisInterface;
class QgsMapCanvas;
class QgsPointCloudLayer;
class QgsVectorLayer;
class QTimer;
class RoofInteraction;
namespace RoofAccuracy { class Controller; }

class RoofEditTool : public QgsMapTool
{
    Q_OBJECT

  public:
    explicit RoofEditTool( QgsMapCanvas *canvas, QgisInterface *iface );
    ~RoofEditTool() override;
    void activate() override;
    void deactivate() override;
    void canvasPressEvent( QgsMapMouseEvent *event ) override;
    void canvasDoubleClickEvent( QgsMapMouseEvent *event ) override;
    void canvasMoveEvent( QgsMapMouseEvent *event ) override;
    void canvasReleaseEvent( QgsMapMouseEvent *event ) override;
    void wheelEvent( QWheelEvent *event ) override;
    void keyPressEvent( QKeyEvent *event ) override;

  protected:
    bool eventFilter( QObject *object, QEvent *event ) override;

  private:
    void setupUi();
    QgsVectorLayer *buildingLayerForSearch() const;
    void endBuildingEdit();
    bool isCandidateBuildingLayer( QgsVectorLayer *layer ) const;
    void refreshPointCloudCombo();
    void hideLegacyPointLayers();
    bool selectBuildingAt( const QgsPointXY &point );
    void loadRoofSelection();
    RoofModelSettings::Selection currentSelection() const;
    void scheduleRoofPreview();
    void scheduleDragPreview();
    void previewRoofSelection();
    void clearRoofPreview();
    void deleteRoofModel();
    void saveRoofModel();
    void notifyRoofModelChanged();
    void setTriangleMeshModeEnabled( bool enabled );
    void updateTriangleMeshModeForCurrentBuilding();
    void syncAccuracyContext();

    QPointer<QgisInterface> mIface;
    QPointer<QgsMapCanvas> mCanvas;
    QPointer<QWidget> mWidget;
    Ui::roofedit mUI;
    std::unique_ptr<RoofInteraction> mInteraction;
    QPointer<QgsVectorLayer> mActiveLayer;
    QgsFeature mCurrentBuilding;
    QTimer *mRoofPreviewTimer = nullptr;
    bool mLiveRoofPreview = false;
    RoofAccuracy::Controller *mAccuracy = nullptr;
};
