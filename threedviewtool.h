#pragma once

#include <qgsmaptool.h>
#include <qgsrubberband.h>
#include <qgsrectangle.h>
#include <qgsfields.h>
#include <qgsfeature.h> // 必须包含以识别 QgsFeatureList
#include <QWidget>
#include <QPoint>
#include <QMatrix4x4> // 必须包含以识别 QMatrix4x4
#include <QVector3D>  // 必须包含以识别 QVector3D

// 引入 UI 头文件
#include "ui_threedview.h"

class QgsVectorLayer;
class QgsMapCanvas;
class QgsMapMouseEvent;
class QgisInterface;

/**
 * @brief 按照参考代码逻辑定义的网格数据结构
 */
// 1. 定义数据结构（必须在类使用它之前定义）
struct MeshData
{
    QVector<QgsPoint> vertices; // 修改为 QgsPoint 以保持双精度
    QVector<int> indices;
    bool isEmpty() const { return vertices.isEmpty(); }
};

// 2. 定义 BuildMesh 类
class BuildMesh
{
  public:
    // 设为 static 方便直接调用
    static MeshData build( const QgsGeometry &geom, double height );
};

class ThreeDViewTool : public QgsMapTool
{
    Q_OBJECT

  public:
    explicit ThreeDViewTool( QgsMapCanvas *canvas, QgisInterface *iface );
    ~ThreeDViewTool() override;

    // 地图画布事件
    void canvasPressEvent( QgsMapMouseEvent *e ) override;
    void canvasMoveEvent( QgsMapMouseEvent *e ) override;
    void canvasReleaseEvent( QgsMapMouseEvent *e ) override;
    void deactivate() override;

  protected:
    // 用于捕获 UI 窗口的回车(保存)和 ESC(退出) 键
    bool eventFilter( QObject *obj, QEvent *event ) override;

  private:
    // 选择逻辑
    void selectAtPoint( const QgsPointXY &point );
    void selectByRectangle( const QgsRectangle &rect );

    // 橡皮筋辅助线
    void createRubberBand();
    void clearRubberBand();

    // UI 交互逻辑
    void showFieldSelectUI();
    void confirmSelection(); // 对应保存操作
    void cancelSelection();  // 对应退出操作

    void updateFeature3D( const QgsFeature &originFeat );
    void refreshMemoryData();

    QgsVectorLayer *mTempLayer = nullptr; // 持久化存储 3D 内存图层
    QString mSelectedHeightField;         // 记录用户选择的高度字段
    QgsFeatureList buildBuildingFromMesh( const MeshData &mesh, const QMatrix4x4 &mat );

  private slots:
    void onFeatureUpdated( QgsFeatureId fid );           // 监听几何/属性修改
    void onFeaturesDeleted( const QgsFeatureIds &fids ); // 监听删除

  private:
    // 核心数据
    QgsVectorLayer *mActiveLayer = nullptr;
    QgsRubberBand *mRubberBand = nullptr;

    // 鼠标交互状态
    bool mDragging = false;
    bool mIsBoxSelecting = false;
    QgsPointXY mStartPoint;
    QPoint mStartScreenPoint;

    // UI 成员
    QWidget *mWidget = nullptr;
    Ui::threedview mUI;

    QgisInterface *mIface = nullptr;
};
