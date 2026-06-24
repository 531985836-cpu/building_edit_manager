#pragma once

#include <qgsmaptool.h>
#include <qgsrubberband.h>
#include <qgsfeature.h>
#include <QWidget>
#include <QPoint>
#include <QMatrix4x4>
#include <QPointer>
#include <QTimer>

#include "buildingroof.h"
#include "ui_threedview.h"

class QgsVectorLayer;
class QgsMapLayer;
class QgsMapCanvas;
class QgsPointCloudLayer;
class QgsRasterLayer;
class QgisInterface;
namespace Qt3DCore
{
  class QEntity;
}
namespace Qt3DRender
{
  class QAttribute;
  class QBuffer;
  class QGeometry;
  class QGeometryRenderer;
}
namespace Qt3DExtras
{
  class QPhongMaterial;
}

struct MeshData
{
    QVector<QgsPoint> vertices;
    QVector<int> indices;
    bool isEmpty() const { return vertices.isEmpty(); }
};

class BuildMesh
{
  public:
    static MeshData build( const QgsGeometry &geom, double height );
};

class ThreeDViewTool : public QgsMapTool
{
    Q_OBJECT

  public:
    explicit ThreeDViewTool( QgsMapCanvas *canvas, QgisInterface *iface );
    ~ThreeDViewTool() override;

    void deactivate() override;
    void activate() override;

  protected:
    bool eventFilter( QObject *obj, QEvent *event ) override;
    bool mDebugShowTempLayer = false;

  private:
    // UI 交互
    void showFieldSelectUI();
    void setupUI();
    void refreshLayerList();
    void updateFieldsCombo();
    void confirmSelection();
    void cancelSelection();

    // 3D 构建与同步
    void updateFeature3D( const QgsFeature &originFeat );
    void refreshMemoryData();
    QgsFeatureList buildBuildingFromMesh( const MeshData &mesh, const QMatrix4x4 &mat );
    void ensureLayerIn3DView( QgsMapLayer *layer );
    void addLoadedPointCloudLayersTo3DView();
    void configurePointCloud3DRenderer( QgsPointCloudLayer *layer );
    void cleanup3DState();
    void refresh3DCanvases();
    void applyFeature3DUpdate( QgsFeatureId fid );
    void flushPendingFeatureUpdates();
    void removeTempFeatures( const QgsFeatureIds &fids );
    QList<BuildingRoof::RoofPoint> roofPointsForFeature( const QgsFeature &feature, bool *fromSavedLayer = nullptr );
    double savedRoofBaseHeight( QgsFeatureId fid, double currentHeight );
    MeshData buildMeshForFeature( const QgsFeature &feature, double height );
    void ensurePreviewEntity();
    void updatePreviewEntity( QgsVectorLayer *layer, const QgsFeatureIds &fids, const QString &heightFieldName, double height );
    void clearPreviewEntity();

  private slots:
    void addVectorData();
    void addPointCloudData();
    void addRasterData();
    void onFeatureUpdated( QgsFeatureId fid );
    void onFeaturesDeleted( const QgsFeatureIds &fids );
    void onLayerChanged( int index );
    void onFeatureAdded( QgsFeatureId fid );
    void onHeightPreviewChanged( QgsVectorLayer *layer, const QgsFeatureIds &fids, const QString &heightFieldName, double height );
    void onHeightPreviewFinished( QgsVectorLayer *layer, const QgsFeatureIds &fids, const QString &heightFieldName, double height );
    void onRoofModelChanged( QgsVectorLayer *layer, QgsFeatureId fid );

  private:
    // UI 成员
    QPointer<QWidget> mWidget = nullptr;
    Ui::threedview mUI;

    // 核心数据
    QPointer<QgsMapCanvas> mCanvas = nullptr;
    QPointer<QgisInterface> mIface = nullptr;

    QPointer<QgsVectorLayer> mActiveLayer = nullptr;
    QPointer<QgsVectorLayer> mTempLayer = nullptr;
    QString mSelectedHeightField;

    // 鼠标交互状态
    QgsRubberBand *mRubberBand = nullptr;
    bool mDragging = false;
    bool mIsBoxSelecting = false;
    QgsPointXY mStartPoint;
    QPoint mStartScreenPoint;

    // 存储映射
    QMap<QString, QgsVectorLayer *> mTempLayersMap;
    QgsFeatureIds mPendingFeatureUpdates;
    QTimer *mFeatureUpdateTimer = nullptr;
    QgsFeatureIds mPreviewFids;
    QPointer<Qt3DCore::QEntity> mPreviewEntity = nullptr;
    QPointer<Qt3DRender::QGeometryRenderer> mPreviewRenderer = nullptr;
    QPointer<Qt3DRender::QGeometry> mPreviewGeometry = nullptr;
    QPointer<Qt3DRender::QBuffer> mPreviewVertexBuffer = nullptr;
    QPointer<Qt3DRender::QAttribute> mPreviewPositionAttribute = nullptr;
    QPointer<Qt3DRender::QAttribute> mPreviewNormalAttribute = nullptr;
    QPointer<Qt3DExtras::QPhongMaterial> mPreviewMaterial = nullptr;
};
