#pragma once

#include <qgsmaptool.h>
#include <qgsrubberband.h>
#include <QWidget>
#include <QPoint>
#include <QEvent>
#include <QKeyEvent>

#include <Qt3DCore/QEntity>
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DRender/QCamera>
#include <Qt3DExtras/QOrbitCameraController>
#include <Qt3DRender/QSceneLoader>

#include "ui_threedview.h"

class QgsVectorLayer;
class QgsMapMouseEvent;

class ThreeDViewTool : public QgsMapTool
{
    Q_OBJECT

  public:
    explicit ThreeDViewTool( QgsMapCanvas *canvas );
    ~ThreeDViewTool() override;

    void canvasPressEvent( QgsMapMouseEvent *e ) override;
    void canvasMoveEvent( QgsMapMouseEvent *e ) override;
    void canvasReleaseEvent( QgsMapMouseEvent *e ) override;
    void deactivate() override;

  protected:
    bool eventFilter( QObject *obj, QEvent *event ) override;

  private:
    void selectAtPoint( const QgsPointXY &point );
    void selectByRectangle( const QgsRectangle &rect );

    void createRubberBand();
    void clearRubberBand();

    void showFieldSelectUI();
    void confirmSelection();
    void cancelSelection();

    // ==================== Qt3D 修改部分 ====================
    class ThreeDWindow : public Qt3DExtras::Qt3DWindow
    {
      public:
        explicit ThreeDWindow( QScreen *screen = nullptr );
        void buildScene( QgsVectorLayer *layer, const QgsFeatureIds &ids, const QString &field );

      private:
        Qt3DCore::QEntity *mRootEntity;
        Qt3DCore::QEntity *mBuildingsEntity;
    };

    ThreeDWindow *m3DWindow = nullptr;

  private:
    QgsVectorLayer *mActiveLayer = nullptr;

    bool mDragging = false;
    bool mIsBoxSelecting = false;
    QgsPointXY mStartPoint;
    QPoint mStartScreenPoint;
    QgsRubberBand *mRubberBand = nullptr;

    QWidget *mWidget = nullptr;
    Ui::threedview mUI;
    QString mSelectedField;
};
