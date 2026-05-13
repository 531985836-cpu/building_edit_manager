#ifndef POINTEDIT_H
#define POINTEDIT_H

#include <qgsmaptool.h>
#include <qgsrubberband.h>
#include <qgsfeature.h>
#include <qgsvectorlayer.h>
#include <qgspointxy.h>
#include <qgsrectangle.h>
#include <QKeyEvent>

class QgsMapCanvas;
class QgsMapMouseEvent;

class PointEdit : public QgsMapTool
{
    Q_OBJECT

  public:
    explicit PointEdit( QgsMapCanvas *canvas );
    ~PointEdit() override;
    void deactivate() override;

  protected:
    // --- 事件分发 ---
    void canvasPressEvent( QgsMapMouseEvent *e ) override;
    void canvasMoveEvent( QgsMapMouseEvent *e ) override;
    void canvasReleaseEvent( QgsMapMouseEvent *e ) override;
    void canvasDoubleClickEvent( QgsMapMouseEvent *e ) override;
    void keyPressEvent( QKeyEvent *e ) override;

  private:
    // --- 图层与资源初始化 ---
    void startEditingLayer( QgsVectorLayer *layer );
    void createRubberBand();
    void clearRubberBand();
    void saveAllEdits();

    // --- 空间探测逻辑 ---
    bool findClosestVertex( const QgsPointXY &pt, QgsFeatureId &fid, int &vertexIndex, double tolerance, const QgsFeatureIds &targetIds );
    bool findClosestEdge( const QgsPointXY &pt, QgsFeatureId &fid, int &startVertexIndex, double tolerance, const QgsFeatureIds &targetIds );
    QgsFeatureId pointInFeature( const QgsPointXY &pt );

    // --- 编辑交互逻辑 (Edit) ---
    void selectAtPoint( const QgsPointXY &pt, bool add );
    void finishEditVertex( const QgsPointXY &newPos );
    void finishEditEdge( const QgsPointXY &newPos );
    void finishFaceMove( const QgsPointXY &newPos );

    // --- 双击增删逻辑 ---
    bool addVertexToSelected( const QgsPointXY &pt );
    bool deleteVertexAt( const QgsPointXY &pt );
    bool deleteFaceAt( const QgsPointXY &pt );

    // --- 几何辅助工具 ---
    bool getGeometryPoints( const QgsGeometry &geom, QList<QgsPointXY> &pts );
    void updateFeatureGeometry( QgsFeature &feat, const QList<QgsPointXY> &pts );
    bool getAdjacentPoints( const QgsGeometry &geom, int vertexIndex, QgsPointXY &prevPt, QgsPointXY &nextPt );

  private:
    enum EditMode
    {
      NoneMode,
      VertexMode,
      EdgeMode,
      FaceMode,
    };
    EditMode mCurrentMode = NoneMode;

    QgsVectorLayer *mActiveLayer = nullptr;
    QgsRubberBand *mRubberBand = nullptr;     // 主预览
    QgsRubberBand *mTempRubber = nullptr;     // 编辑虚线
    QgsRubberBand *mDigitizeRubber = nullptr; // 新建绿面

    // 状态状态变量
    QgsFeatureId mDraggingFeatureId = -1;
    int mDraggingVertexIndex = -1;
    QgsFeatureId mEditingFeatureId = -1;
    int mEditingEdgeStartIndex = -1;
    QgsFeatureId mMovingFeatureId = -1;

    QList<QgsPointXY> mOriginalFacePts;
    QgsPointXY mInitialClickPoint;
    bool mShiftPressed = false;
};

#endif
