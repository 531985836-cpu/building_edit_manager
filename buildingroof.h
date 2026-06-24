#pragma once

#include <qgsgeometry.h>
#include <qgspoint.h>

#include <QList>
#include <QString>
#include <QVector>

class BuildingRoof
{
  public:
    struct RoofPoint
    {
      QgsPoint point;
      QString type;
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
      bool isEmpty() const { return vertices.isEmpty() || indices.isEmpty(); }
    };

    struct MeshResult
    {
      bool success = false;
      QString error;
      Mesh mesh;
    };

    static Result buildSingleSlopeRoof( const QgsGeometry &buildingGeometry, const QList<RoofPoint> &roofPoints );
    static MeshResult buildSingleSlopePrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints );
    static MeshResult buildGabledRoofPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints );
    static MeshResult buildMultiRidgePrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints );
    static MeshResult buildHippedRoofPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints );
};
