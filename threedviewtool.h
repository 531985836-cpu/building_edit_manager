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

/**
 * @brief 按照参考代码逻辑定义的网格数据结构
 */
struct MeshData
{
    QVector<QVector3D> vertices;
    QVector<int> indices;
    bool isEmpty() const { return vertices.isEmpty() || indices.isEmpty(); }
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

<<<<<<< HEAD
    // 3D 构建与同步
=======
<<<<<<< HEAD
>>>>>>> 63dd1b9f77ae99b3223420186805982d8863bfd5
    void updateFeature3D( const QgsFeature &originFeat );
    void refreshMemoryData();
    QgsFeatureList buildBuildingFromMesh( const MeshData &mesh, const QMatrix4x4 &mat );

  private slots:
<<<<<<< HEAD
    void onFeatureUpdated( QgsFeatureId fid );
    void onFeaturesDeleted( const QgsFeatureIds &fids );
    void onLayerChanged( int index );
    void onFeatureAdded( QgsFeatureId fid );
=======
    void onFeatureUpdated( QgsFeatureId fid );           // 监听几何/属性修改
    void onFeaturesDeleted( const QgsFeatureIds &fids ); // 监听删除
=======
    QgsFeatureList buildBuildingFromMesh( const MeshData &mesh, const QMatrix4x4 &mat );
>>>>>>> f644786332709f1cc37fdca583a1742748c8ba08
>>>>>>> 63dd1b9f77ae99b3223420186805982d8863bfd5

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

<<<<<<< HEAD
    // 存储映射
    QMap<QString, QgsVectorLayer *> mTempLayersMap;
=======
    // UI 成员
    QWidget *mWidget = nullptr;
    Ui::threedview mUI;
<<<<<<< HEAD

    QgisInterface *mIface = nullptr;
=======
>>>>>>> f644786332709f1cc37fdca583a1742748c8ba08
>>>>>>> 63dd1b9f77ae99b3223420186805982d8863bfd5
};
