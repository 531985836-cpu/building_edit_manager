#pragma once

#include <qgsgeometry.h>
#include <qgspoint.h>

#include <QList>
#include <QPair>
#include <QString>
#include <QVector>

class BuildingRoof
{
  public:
    static constexpr double RIDGE_HEIGHT_AVERAGE_THRESHOLD = 5.0;

    struct RoofPoint
    {
      QgsPoint point;
      QString type;
    };

    struct RoofSample
    {
      QgsPoint point;
    };

    struct Mesh
    {
      QVector<QgsPoint> vertices;
      QVector<int> indices;
      QVector<QPair<QgsPoint, QgsPoint>> structureLines;
      bool isEmpty() const { return vertices.isEmpty() || indices.isEmpty(); }
    };

    struct MeshResult
    {
      bool success = false;
      QString error;
      Mesh mesh;
    };

    struct RoofPlaneSegment
    {
      int id = 0;
      bool meetsAreaReference = false;
      double projectedAreaRatio = 0.0;
      double normalX = 0.0;
      double normalY = 0.0;
      double normalZ = 1.0;
      QVector<QgsPoint> points;
    };

    struct RoofPlaneSegmentation
    {
      QVector<RoofPlaneSegment> segments;
      QVector<QgsPoint> unclassifiedPoints;
    };

    struct RoofSegmentationOptions
    {
      bool horizontalOnly = false;
      bool useRoofHeightBand = true;
      bool splitConnectedComponents = true;
      bool allowVerticalPlanes = false; // Use orthogonal 3D plane fitting, including facades.
      double areaReferenceRatio = 0.06;
      int minimumInlierCount = 24;
      double minimumInlierFraction = 0.0;
      double planeResidualThreshold = 0.0;
      int maximumPlanes = 0; // Zero means no plane-count limit.
    };

    // Automatic roof builders use only marker types/counts; input marker coordinates are ignored.
    static MeshResult buildFlatReliefPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints, const QVector<RoofSample> &pointCloudSamples = QVector<RoofSample>() );
    static MeshResult buildClusteredFlatTopHippedRoofPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints, const QVector<RoofSample> &pointCloudSamples = QVector<RoofSample>() );
    static MeshResult buildCurvedRoofPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints, const QVector<RoofSample> &pointCloudSamples = QVector<RoofSample>() );
    static MeshResult buildApexRoofPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints, const QVector<RoofSample> &pointCloudSamples = QVector<RoofSample>() );
    static MeshResult buildGabledRoofPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints, const QVector<RoofSample> &pointCloudSamples = QVector<RoofSample>() );
    static MeshResult buildHippedRoofPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints, const QVector<RoofSample> &pointCloudSamples = QVector<RoofSample>() );
    static MeshResult buildRoofPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints, const QVector<RoofSample> &pointCloudSamples );
    static RoofPlaneSegmentation segmentRoofCloud( const QgsGeometry &buildingGeometry, const QVector<RoofSample> &pointCloudSamples,
                                                   const RoofSegmentationOptions &options = RoofSegmentationOptions() );
    static RoofPlaneSegmentation segmentFlatTopRoofCloud( const QgsGeometry &buildingGeometry, const QVector<RoofSample> &pointCloudSamples,
                                                          QVector<RoofSample> *roofSamples = nullptr );
    static RoofPlaneSegmentation segmentRoofPlanesForDebug( const QgsGeometry &buildingGeometry, const QVector<RoofSample> &pointCloudSamples, bool preserveSmallEndPlanes = false );
};
