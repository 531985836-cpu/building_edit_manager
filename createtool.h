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

    void activate() override;

  protected:
    void canvasPressEvent( QgsMapMouseEvent *e ) override;
    void canvasMoveEvent( QgsMapMouseEvent *e ) override;
    void keyPressEvent( QKeyEvent *e ) override;
    void deactivate() override;

  private:
    // 核心：手动递归收集节点 ID，使用 QGIS 3.44 原始 API
    void collectNodes( const QgsPointCloudIndex &index, const QgsPointCloudNodeId &nodeId, const QgsRectangle &extent, QList<QgsPointCloudNodeId> &nodes );

    double calculateZFromPointCloud( const QgsGeometry &geom );
    void finishCurrentFeatureWithHeight();
    void cancelDigitizing( bool clearFinished = true );
    void initLayer();
    void updateWidgetInteractivity( bool heightEnabled, bool mergeEnabled );

    void setupUi();
    void updateFields( int index );
    void refreshLayerCombos();

  private:
    QgsVectorLayer *mVectorLayer = nullptr;
    QgsPointCloudLayer *mPCLayer = nullptr;
    QgsRubberBand *mRubberBand = nullptr;
    QList<QgsPointXY> mPoints;
    bool mIsDigitizing = false;

    Ui::Form mUI;
    QWidget *mSettingsWidget = nullptr;
    QString mTargetFieldName;
    QVector3D computeNormal( double xx, double xy, double xz, double yy, double yz, double zz );

    QList<QgsRubberBand *> mDebugMarkers;
    void clearDebugMarkers();

    // --- 新增：用于 3D 显示的辅助函数 ---
    QgsVectorLayer *getOrCreateDebugLayer();
    void setup3DRendering( QgsVectorLayer *layer );

    // 建议增加一个成员变量记录调试图层，避免重复查找
    QgsVectorLayer *mDebugLayer = nullptr;
};

#endif
