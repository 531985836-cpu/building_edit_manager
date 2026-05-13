#pragma once

#include <qgsmaptool.h>
#include <qgsrubberband.h>
#include <qgsrectangle.h>
#include <qgsfields.h>
#include <qgsfeature.h>
#include <QWidget>
#include <QPoint>
#include <QMatrix4x4>
#include <QVector3D>

// 引入 UI 头文件
#include "ui_threedview.h"

class QgsVectorLayer;
class QgsMapCanvas;
class QgsMapMouseEvent;
class QgisInterface;

/**
 * @brief 按照参考代码逻辑定义的网格数据结构
 */
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
    // ===== UI 交互与逻辑控制 =====
    void showFieldSelectUI();
    void setupUI();           // UI 信号初始化
    void refreshLayerList();  // 刷新图层 ComboBox (参考 CreateTool)
    void updateFieldsCombo(); // 刷新字段 ComboBox (参考 CreateTool)
    void confirmSelection();  // 执行后续计算
    void cancelSelection();

    // ===== 3D 数据处理 =====
    void updateFeature3D( const QgsFeature &originFeat );
    void refreshMemoryData();
    QgsFeatureList buildBuildingFromMesh( const MeshData &mesh, const QMatrix4x4 &mat );

  private slots:
    void onFeatureUpdated( QgsFeatureId fid );
    void onFeaturesDeleted( const QgsFeatureIds &fids );
    void onLayerChanged( int index ); // 图层切换联动槽函数

  private:
    // ===== UI 成员 =====
    QWidget *mWidget = nullptr;
    Ui::threedview mUI;

    // ===== 核心数据与状态 =====
    QgsMapCanvas *mCanvas = nullptr;
    QgisInterface *mIface = nullptr;

    QgsVectorLayer *mActiveLayer = nullptr; // 选中的目标矢量图层
    QgsVectorLayer *mTempLayer = nullptr;   // 3D 内存图层
    QString mSelectedHeightField;           // 选中的高度字段名

    // ===== 鼠标交互状态 (保持不动) =====
    QgsRubberBand *mRubberBand = nullptr;
    bool mDragging = false;
    bool mIsBoxSelecting = false;
    QgsPointXY mStartPoint;
    QPoint mStartScreenPoint;

    // 存储 原始图层ID -> 对应的内存Mesh图层
    QMap<QString, QgsVectorLayer *> mTempLayersMap;
};
