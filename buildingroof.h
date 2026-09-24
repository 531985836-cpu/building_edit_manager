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

    struct Result
    {
      bool success = false;
      QString error;
      QgsGeometry geometry;
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
      bool accepted = false;
      double projectedAreaRatio = 0.0;
      QVector<QgsPoint> points;
    };

    struct RoofPlaneSegmentation
    {
      QVector<RoofPlaneSegment> segments;
      QVector<QgsPoint> unclassifiedPoints;
    };

    static Result buildSingleSlopeRoof( const QgsGeometry &buildingGeometry, const QList<RoofPoint> &roofPoints );
    static MeshResult buildSingleSlopePrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints );
    static MeshResult buildFlatReliefPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints, const QVector<RoofSample> &pointCloudSamples = QVector<RoofSample>() );
    static MeshResult buildClusteredFlatTopHippedRoofPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints, const QVector<RoofSample> &pointCloudSamples = QVector<RoofSample>() );
    static MeshResult buildCurvedRoofPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints, const QVector<RoofSample> &pointCloudSamples = QVector<RoofSample>() );
    static MeshResult buildApexRoofPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints, const QVector<RoofSample> &pointCloudSamples = QVector<RoofSample>() );
    static MeshResult buildGabledRoofPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints, const QVector<RoofSample> &pointCloudSamples = QVector<RoofSample>() );
    static MeshResult buildMultiRidgePrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints );
    static MeshResult buildHippedRoofPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints, const QVector<RoofSample> &pointCloudSamples = QVector<RoofSample>() );
    static RoofPlaneSegmentation segmentRoofPlanesForDebug( const QgsGeometry &buildingGeometry, const QVector<RoofSample> &pointCloudSamples, bool preserveSmallEndPlanes = false );
};
