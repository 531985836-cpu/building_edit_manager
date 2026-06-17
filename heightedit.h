#pragma once

#include <qgsmaptool.h>
#include <qgsmapcanvas.h>
#include <qgsrubberband.h>
#include <QWidget>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QEvent>
#include <QKeyEvent>
#include <QSlider>
#include <QMap>
#include <QPoint>
#include <QTimer>
#include <QPointer>

#include "ui_heightedit.h"

class QgsVectorLayer;
class QgsMapMouseEvent;
class QWheelEvent;

class HeightEditTool : public QgsMapTool
{
    Q_OBJECT
  public:
    explicit HeightEditTool( QgsMapCanvas *canvas );
    ~HeightEditTool() override;

    void canvasPressEvent( QgsMapMouseEvent *e ) override;
    void canvasMoveEvent( QgsMapMouseEvent *e ) override;
    void canvasReleaseEvent( QgsMapMouseEvent *e ) override;

    void keyPressEvent( QKeyEvent *e ) override;
    void keyReleaseEvent( QKeyEvent *e ) override;
    void wheelEvent( QWheelEvent *e ) override;
    void deactivate() override;

  protected:
    bool eventFilter( QObject *obj, QEvent *event ) override;

  private:
    // ---------------- 选择逻辑 ----------------
    void selectAtPoint( const QgsPointXY &point, bool addToSelection = false );
    void toggleSelectionAtPoint( const QgsPointXY &point );
    void selectByRectangle( const QgsRectangle &rect, bool addToSelection = false );
    void createRubberBand();
    void clearRubberBand();
    void startEditingLayer( QgsVectorLayer *layer );

    void showSelectedAttributes();
    void applySliderValueToLayer( double heightValue );
    void previewHeightValue( double heightValue );
    void cancelHeightPreview();
    void saveHeightPreview();
    void setWheelStep( double step );
    bool adjustHeightByWheel( QWheelEvent *event );
    void updateHeightColumnText( double heightValue );
    void initSliderCache(); // 初始化滑块缓存
    double mReferenceHeight = 0.0;
    static constexpr double SLIDER_SCALE = 10.0; // 0.1 精度

    // ---------------- 缓存 ----------------
    struct HeightContext
    {
        QgsFeatureId fid;
        double originalValue;
        double currentValue;
        double originalHeight; // 初始高度，用于滑块偏移计算
        double currentHeight;  // 当前高度，用于实时修改和保存
    };
    QMap<QgsFeatureId, HeightContext> mHeightCache;

    QString mHeightFieldName = "height";
    int mHeightFieldColumn = 0;
    QMap<QgsFeatureId, double> mInitialFieldValues; // 滑块字段初始值

    // ---------------- 状态 ----------------
    bool mShiftPressed = false;
    bool mDragging = false;
    bool mIsBoxSelecting = false;

    QgsPointXY mStartPoint;
    QPoint mStartScreenPoint;

    QgsRubberBand *mRubberBand = nullptr;
    QPointer<QgsVectorLayer> mActiveLayer = nullptr;

    Ui::heightedit mUI;
    QPointer<QWidget> mWidget = nullptr;
    QTimer *mSliderUpdateTimer = nullptr;
    int mPendingSliderValue = 0;
    bool mHasPendingSliderValue = false;
    bool mHasPreviewHeight = false;
    double mPreviewHeight = 0.0;
    double mWheelStep = 0.0;

  private slots:
    void onCellChanged( int row, int column );
    void onSliderChanged( int value );
    void flushPendingSliderValue();
};
