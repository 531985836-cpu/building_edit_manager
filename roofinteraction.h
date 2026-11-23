#pragma once

#include "roofmodelsettings.h"

#include <QPointer>
#include <QPoint>
#include <functional>
#include <memory>
#include <optional>

class QKeyEvent;
class QEvent;
class QWheelEvent;
class QgisInterface;
class QgsMapCanvas;
class QgsMapMouseEvent;
class QgsRubberBand;
namespace Ui { class roofedit; }

class RoofInteraction
{
  public:
    RoofInteraction( QgsMapCanvas *canvas, QgisInterface *iface, Ui::roofedit &ui,
                     QObject *context, std::function<void()> changed, std::function<void()> previewChanged = {} );
    ~RoofInteraction();

    void load( QgsVectorLayer *layer, const QgsFeature &building, const RoofModelSettings::Selection &saved );
    void reset();
    void clearRegions();
    void addToSelection( RoofModelSettings::Selection &selection ) const;
    bool addDragToSelection( RoofModelSettings::Selection &selection );
    bool hasRegions() const { return !mRegions.isEmpty(); }
    bool hasSelectedRegion() const { return mAdjustingRegionIndex >= 0; }
    bool hasBuilding() const;
    bool hasUnfinishedEdit() const;
    bool hasRemovedSavedRegions() const;
    bool press( QgsMapMouseEvent *event );
    bool release( QgsMapMouseEvent *event );
    bool doubleClick( QgsMapMouseEvent *event );
    void canvasEvent( QEvent *event );
    void move( QgsMapMouseEvent *event );
    bool wheel( QWheelEvent *event );
    bool keyPress( QKeyEvent *event );
    void undo();
    bool cancelDrag();

    static BuildingRoof::MeshResult build( const QgsGeometry &geometry,
                                           const RoofModelSettings::Selection &selection );

  private:
    struct RegionPoint
    {
      QgsPointXY point;
      int boundaryVertex = -1;
      bool intermediate = false;
    };

    struct UndoState
    {
      QVariantList regions;
      int selectedRegion = -1;
      bool ridgeSelected = false;
    };

    bool setRegions( QVariantList regions );
    bool setRegion( int index, QVariantMap region );
    void removeLastSketchPoint();
    int selectedRegion() const;
    void updateDeleteRidgeButton();
    void deleteSelectedRidge();

    void setDrawing( bool enabled );
    void cancelDraft();
    bool switchRegion( const QgsPointXY &mapPoint );
    int regionAt( const QgsPointXY &mapPoint ) const;
    void selectRegion( int index );
    void deselectRegion();
    void addRegion( const QgsGeometry &region, bool remainder = false );
    void finishRegionSketch( bool allowSplit );
    bool editRegionVertex( QgsMapMouseEvent *event );
    bool canMoveRegionVertices() const;
    QgsGeometry activeRegionShape() const;
    std::optional<QgsPointXY> regionVertexAt( const QgsGeometry &shape, const QgsPointXY &mapPoint ) const;
    std::optional<QgsPointXY> regionEdgeAt( const QgsGeometry &shape, const QgsPointXY &mapPoint ) const;
    enum class VertexEdit { Move, Insert, Remove };
    bool modifyRegionVertex( VertexEdit operation, const QgsPointXY &from, QgsPointXY to = {},
                             QVariantList *preview = nullptr );
    RegionPoint regionPoint( QgsMapMouseEvent *event ) const;
    QVector<RegionPoint> traceBoundary( const RegionPoint &from, const RegionPoint &to ) const;
    bool splitRegions( const QgsPolylineXY &vertices );
    void updateRegionBoundaries();
    void highlightBoundary( int vertex );
    void updateOutline( const QgsPointXY *cursor = nullptr, const QVector<RegionPoint> &trace = {} );
    void setRidgeDrawing( bool enabled );
    void cancelRidge();
    bool pressRidge( QgsMapMouseEvent *event );
    void pickRidge( int index, QgsMapMouseEvent *event );
    bool selectRidgeRegion( int index );
    int ridgeAt( const QgsPointXY &mapPoint ) const;
    QgsGeometry visibleRidge( int index ) const;
    void updateStoredRidges();
    bool startRidgeAdjustment();
    QVector<BuildingRoof::RoofSample> collectRidgeSamples( const QgsGeometry &component ) const;
    bool fitDrawnRidge( const QgsGeometry &component, RoofModelSettings::Type type, QgsPolylineXY &ridge,
                       double &eave, double &lift, QVector<BuildingRoof::RoofSample> &samples,
                       BuildingRoof::GableSideWidths &widths ) const;
    bool restoreRidge();
    void setRidgeHeights( double eave, double lift );
    void previewRidge( QgsPolylineXY ridge, const QgsGeometry &component, RoofModelSettings::Type type,
                       double eave, double lift, QVector<BuildingRoof::RoofSample> samples,
                       BuildingRoof::GableSideWidths widths );
    void applyRidge();
    QVariantMap ridgeRegionValue( const QgsPolylineXY &ridge, const BuildingRoof::GableSideWidths &widths ) const;
    void notifyPreviewChanged();
    void startRidgeDrag( QgsMapMouseEvent *event );
    QgsPolylineXY draggedRidge( QgsMapMouseEvent *event ) const;
    QgsPointXY ridgePoint( QgsMapMouseEvent *event ) const;
    void updateRidgeOutline( const QgsPointXY *cursor = nullptr, const QgsPolylineXY &preview = {} );
    void message( const QString &text, Qgis::MessageLevel level = Qgis::MessageLevel::Info ) const;

    QPointer<QgsMapCanvas> mCanvas;
    QPointer<QgisInterface> mIface;
    QPointer<QgsVectorLayer> mLayer;
    Ui::roofedit &mUi;
    std::function<void()> mChanged;
    std::function<void()> mPreviewChanged;
    std::unique_ptr<QgsRubberBand> mOutline;
    std::unique_ptr<QgsRubberBand> mBoundaryOutline;
    std::unique_ptr<QgsRubberBand> mBoundaryHighlight;
    std::unique_ptr<QgsRubberBand> mVertexHandles;
    std::unique_ptr<QgsRubberBand> mRidgeOutline;
    std::unique_ptr<QgsRubberBand> mStoredRidgeOutline;
    QgsGeometry mFootprint;
    QgsGeometry mRegionBoundary;
    QgsGeometry mMapRegionBoundary;
    QgsMultiPolylineXY mBoundaryPaths;
    QVariantList mRegions;
    QVector<UndoState> mUndoHistory;
    bool mHadSavedRegions = false;
    QVector<RegionPoint> mVertices;
    QgsGeometry mAdjustingRegion;
    int mAdjustingRegionIndex = -1;
    std::optional<QgsPointXY> mMovingVertex;
    std::optional<QgsPointXY> mVertexPreview;
    std::optional<QPoint> mVertexPress;
    std::optional<QPoint> mMiddlePress;
    QgsPolylineXY mRidgeVertices;
    QgsGeometry mRidgeBoundary;
    QgsGeometry mMapRidgeBoundary;
    int mRidgeRegion = -1;
    bool mAdjustingRidge = false;
    RoofModelSettings::Type mRidgeType = RoofModelSettings::Type::None;
    QVector<BuildingRoof::RoofSample> mRidgeSamples;
    BuildingRoof::GableSideWidths mGableWidths{};
    QgsGeometry mRidgeComponent;
    std::optional<QgsPointXY> mRidgeDragStart;
    QgsPolylineXY mRidgeDragPreview;
    int mDraggedRidgeVertex = -1;
    double mBaseHeight = 3.0;
};
