#pragma once

#include <qgsfeature.h>
#include <qgsfeatureid.h>
#include <qgsmaptool.h>
#include <qgspoint.h>
#include <qgspointxy.h>
#include <qgspointcloudindex.h>
#include <QElapsedTimer>
#include <QPointer>
#include <QWidget>

#include "ui_roofedit.h"

class QgsMapCanvas;
class Qgs3DMapCanvas;
class QgsMapMouseEvent;
class QgsPointCloudLayer;
class QgsRubberBand;
class QgsVectorLayer;
class QgisInterface;
class QKeyEvent;
class QWheelEvent;

namespace Qt3DCore
{
  class QEntity;
  class QTransform;
}

namespace Qt3DExtras
{
}

class RoofEditTool : public QgsMapTool
{
    Q_OBJECT

  public:
    explicit RoofEditTool( QgsMapCanvas *canvas, QgisInterface *iface );
    ~RoofEditTool() override;

    void activate() override;
    void deactivate() override;
    void canvasPressEvent( QgsMapMouseEvent *e ) override;
    void canvasMoveEvent( QgsMapMouseEvent *e ) override;
    void canvasDoubleClickEvent( QgsMapMouseEvent *e ) override;
    void wheelEvent( QWheelEvent *e ) override;
    void keyPressEvent( QKeyEvent *e ) override;

  private slots:
    void deleteSelectedPoint();
    void clearCurrentBuildingPoints();
    void onPointSelectionChanged();
    void confirmPointEditPreview();

  private:
    void setupUi();
    void refreshLayerRefs();
    void refreshPointCloudCombo();
    bool isCandidateBuildingLayer( QgsVectorLayer *layer ) const;
    bool selectBuildingAt( const QgsPointXY &mapPoint );
    void addRoofPointAt( const QgsPointXY &mapPoint );
    QgsFeatureId findRoofPointAt( const QgsPointXY &mapPoint ) const;
    void deleteRoofPoint( QgsFeatureId fid );
    bool selectRoofPoint( QgsFeatureId fid );
    bool selectRoofPointAt( const QgsPointXY &mapPoint );
    bool updateRoofPoint( QgsFeatureId fid, const QgsPoint &point );
    bool applyRoofPointToLayer( QgsFeatureId fid, const QgsPoint &point, bool commitNow, bool refreshTable );
    bool beginPointEditPreview( QgsFeatureId fid );
    bool previewRoofPoint( QgsFeatureId fid, const QgsPoint &point );
    void cancelPointEditPreview();
    void updatePointEditTip( const QString &text = QString() );
    void clearPointSelection();
    void updateSelectedPointTableRow( const QgsPoint &point );
    void updatePointPreviewDisplay( const QgsPoint &point );
    void clearPointPreviewDisplay();
    Qgs3DMapCanvas *active3DCanvas() const;
    void ensure3DPreviewEntity();
    void update3DPreviewPoint( const QgsPoint &point, bool force = false );
    void clear3DPreviewPoint();
    bool adjustSelectedPointHeight( QWheelEvent *event );
    double wheelStep() const;
    bool isHeightEditEnabled() const;
    bool isPositionEditEnabled() const;
    bool handlePositionEditClick( const QgsPointXY &mapPoint );
    double currentMovingPreviewZ() const;
    void startPositionPreview( QgsFeatureId fid );
    void updatePositionPreview( const QgsPointXY &mapPoint );
    void clearPositionPreview();
    void ensurePointLayer();
    void ensurePointLayerIn3DView();
    void refreshPointTable();
    void setupPointLayerRenderer();
    QString currentPointType() const;
    QColor colorForPointType( const QString &type ) const;
    double estimateHeightFromPointCloud( const QgsPointXY &mapPoint ) const;
    void collectNodes( const QgsPointCloudIndex &index, const QgsPointCloudNodeId &nodeId, const QgsRectangle &extent, QList<QgsPointCloudNodeId> &nodes ) const;

    QPointer<QgisInterface> mIface = nullptr;
    QPointer<QgsMapCanvas> mCanvas = nullptr;
    QPointer<QWidget> mWidget = nullptr;
    Ui::roofedit mUI;

    QPointer<QgsVectorLayer> mActiveLayer = nullptr;
    QPointer<QgsPointCloudLayer> mPointCloudLayer = nullptr;
    QPointer<QgsVectorLayer> mPointLayer = nullptr;
    QPointer<QgsRubberBand> mPointPreviewRubberBand = nullptr;
    QPointer<Qgs3DMapCanvas> mPreviewCanvas3D = nullptr;
    QPointer<Qt3DCore::QEntity> mPreviewRootEntity = nullptr;
    QPointer<Qt3DCore::QEntity> mPreviewPointEntity = nullptr;
    QPointer<Qt3DCore::QTransform> mPreviewPointTransform = nullptr;

    QgsFeature mCurrentBuilding;
    QgsFeatureId mSelectedPointFid = FID_NULL;
    QgsFeatureId mMovingPointFid = FID_NULL;
    QgsFeatureId mPreviewPointFid = FID_NULL;
    QgsPoint mMovingOriginalPoint;
    QgsPoint mPreviewOriginalPoint;
    QgsPoint mPreviewCurrentPoint;
    QElapsedTimer mMoveThrottleTimer;
    QElapsedTimer m3DPreviewThrottleTimer;
    bool mHasPointEditPreview = false;
};
