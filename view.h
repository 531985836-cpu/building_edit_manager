#ifndef VIEW_H
#define VIEW_H

#include <qgsmaptool.h>
#include <qgsmapcanvas.h>
#include <qgsvectorlayer.h>
#include <qgsfeature.h>
#include <qgsgeometry.h>
#include <qgisinterface.h>

class View : public QgsMapTool
{
    Q_OBJECT

  public:
    View( QgsMapCanvas *canvas, QgisInterface *iface );
    ~View() override;

  private:
    void showSideView( const QgsFeature &feat );

    QgsGeometry buildSideGeometry(
      const QgsGeometry &geom,
      double height,
      double angleDeg
    );

  protected:
    void canvasPressEvent( QgsMapMouseEvent *e ) override;

    void canvasMoveEvent( QgsMapMouseEvent *e ) override;

    void canvasReleaseEvent( QgsMapMouseEvent *e ) override;
  private:
    QgisInterface *mIface = nullptr;

    // 当前激活面图层
    QgsVectorLayer *mActiveLayer = nullptr;

    // 侧视图图层
    QgsVectorLayer *mSideLayer = nullptr;

    // 高度字段名称
    QString mHeightField = "height";
    QgsMapCanvas *mSideCanvas = nullptr;
    double mViewAngle = 0.0;
    bool mRightButtonPressed = false;
    QgsFeature mCurrentFeature;
    QPoint mLastMousePos;
};

#endif // VIEW_H
