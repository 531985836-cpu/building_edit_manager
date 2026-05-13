#ifndef CREATETOOL_H
#define CREATETOOL_H

#include <qgsmaptool.h>
#include <qgsrubberband.h>
#include <qgsvectorlayer.h>
#include <qgspointcloudlayer.h>
#include <qgsfeature.h>
#include <qgspointxy.h>
#include <qgspointcloudindex.h>
#include <QList>
#include <QKeyEvent>
#include <QWidget>
#include <QVector3D>

#include "ui_createtool.h"

class QgsMapCanvas;
class QgsMapMouseEvent;

class CreateTool : public QgsMapTool
{
    Q_OBJECT

  public:
    explicit CreateTool( QgsMapCanvas *canvas );
    ~CreateTool() override;

    // --- 地图工具核心生命周期 ---
    void activate() override;
    void deactivate() override;
    void canvasPressEvent( QgsMapMouseEvent *e ) override;
    void canvasMoveEvent( QgsMapMouseEvent *e ) override;
    void keyPressEvent( QKeyEvent *e ) override;

  private:
    // --- 要素数字化与几何编辑逻辑 ---
    void finishCurrentFeatureWithHeight();
    void cancelDigitizing( bool clearFinished = true );
    void snapTwoSelectedFeatures();

    // --- 点云处理与高度计算核心 ---
    void collectNodes( const QgsPointCloudIndex &index, const QgsPointCloudNodeId &nodeId, const QgsRectangle &extent, QList<QgsPointCloudNodeId> &nodes );
    double calculateZFromPointCloud( const QgsGeometry &geom );
    QVector3D computeNormal( double xx, double xy, double xz, double yy, double yz, double zz );
    void refresh3DView();

    // --- 分割功能逻辑 ---
    void performSplit( const QgsPointXY &endPoint );
    QgsPointXY getSnappedPoint( const QgsGeometry &geom, const QgsPointXY &mapPt, double tolerance );

    // --- UI 管理与界面交互 ---
    void setupUi();
    void updateFields( int index );
    void refreshLayerCombos();
    void updateWidgetInteractivity( bool heightEnabled, bool mergeEnabled );

    // --- 调试辅助与 3D 渲染 ---
    QgsVectorLayer *getOrCreateDebugLayer();
    void clearDebugMarkers();

  private:
    // --- 核心组件与数字化状态 ---
    QgsVectorLayer *mVectorLayer = nullptr;
    QgsPointCloudLayer *mPCLayer = nullptr;
    QgsRubberBand *mRubberBand = nullptr;
    QList<QgsPointXY> mPoints;
    bool mIsDigitizing = false;

    // --- 分割功能状态变量 ---
    QgsPointXY mSplitStartPoint;
    QgsFeatureId mTargetFeatureId = -1;
    bool mIsSplitting = false;
    QgsRubberBand *mSplitLineBand = nullptr;

    // --- UI 插件窗体与属性字段 ---
    Ui::Form mUI;
    QWidget *mSettingsWidget = nullptr;
    QString mTargetFieldName;

    // --- 调试相关对象 ---
    QgsVectorLayer *mDebugLayer = nullptr;
    QList<QgsRubberBand *> mDebugMarkers;
};

#endif
