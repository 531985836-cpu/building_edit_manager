#pragma once

#include <qgsmaptool.h>
#include <qgsrubberband.h> 
#include <qgsfeature.h> 
#include <QWidget>
#include <QPoint> 
#include <QMatrix4x4>

#include "ui_threedview.h"

class QgsVectorLayer;
class QgsMapCanvas;
class QgisInterface;

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

  private slots:
    void onFeatureUpdated( QgsFeatureId fid );
    void onFeaturesDeleted( const QgsFeatureIds &fids );
    void onLayerChanged( int index );
    void onFeatureAdded( QgsFeatureId fid );

  private:
    // UI 成员
    QWidget *mWidget = nullptr;
    Ui::threedview mUI;

    // 核心数据
    QgsMapCanvas *mCanvas = nullptr;
    QgisInterface *mIface = nullptr;

    QgsVectorLayer *mActiveLayer = nullptr;
    QgsVectorLayer *mTempLayer = nullptr;
    QString mSelectedHeightField;

    // 鼠标交互状态
    QgsRubberBand *mRubberBand = nullptr;
    bool mDragging = false;
    bool mIsBoxSelecting = false;
    QgsPointXY mStartPoint;
    QPoint mStartScreenPoint;

    // 存储映射
    QMap<QString, QgsVectorLayer *> mTempLayersMap;
};
