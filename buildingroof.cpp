#include "buildingroof.h"

#include <qgsgeometrycollection.h>
#include <qgsexception.h>
#include <qgslinestring.h>
#include <qgspolygon.h>
#include <qgsspatialindex.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <tuple>

namespace
{
  struct AnchorPoint
  {
    QgsPointXY point;
    double z = 0.0;
  };

  struct ProfileAnchor
  {
    double s = 0.0;
    double z = 0.0;
  };

  struct FittedBarrelProfile
  {
    QVector<ProfileAnchor> anchors;
    double normalX = 0.0;
    double normalY = 0.0;
  };

  struct LineInterval
  {
    double start = 0.0;
    double end = 0.0;
  };

  struct BentGableSegment
  {
    QgsPointXY start;
    QgsPointXY end;
    QgsPoint ridgePoint;
    double dirX = 0.0;
    double dirY = 0.0;
    double normalX = 0.0;
    double normalY = 0.0;
    double length = 0.0;
    double sameSideLimit = 0.0;
    double oppositeLimit = 0.0;
  };

  struct BentGableLayout
  {
    bool success = false;
    QgsPointXY primaryEnd;
    QgsPointXY bend;
    QgsPointXY secondaryEnd;
    double splitNormalX = 0.0;
    double splitNormalY = 0.0;
  };

  struct ClusterSample
  {
    QgsPointXY point;
    double u = 0.0;
    double v = 0.0;
    int cluster = -1;
    bool visited = false;
  };

  double profileRoofZ( const QVector<ProfileAnchor> &anchors, double s );
  void appendProfileAnchor( QVector<ProfileAnchor> &anchors, double s, double z );
  bool pointInRing( const QVector<QgsPointXY> &ring, const QgsPointXY &point );
  double cross2d( double ax, double ay, double bx, double by );
  double ringExtentSize( const QVector<QgsPointXY> &ring );
  bool makeBentSegment( const QgsPointXY &start, const QgsPointXY &end, double ridgeHeight, const QVector<QgsPointXY> &ring, const QgsPoint &boundaryPoint, BentGableSegment &segment );
  BentGableLayout findBentGabledLayout( const QVector<QgsPointXY> &inputRing, const QgsPointXY &ridgeXY, double dirX, double dirY );
  QgsPointXY pullPointInsideRing( const QVector<QgsPointXY> &ring, const QgsPointXY &candidate, const QgsPointXY &fallback );

  bool isGroundPointType( const QString &type )
  {
    return type.contains( QStringLiteral( "地面" ) )
           || type.contains( QStringLiteral( "鍦伴潰" ) )
           || type.contains( QStringLiteral( "ground" ), Qt::CaseInsensitive );
  }

  bool isRidgePointType( const QString &type )
  {
    return type.contains( QStringLiteral( "屋脊" ) )
           || type.contains( QStringLiteral( "灞嬭剨" ) )
           || type.contains( QStringLiteral( "ridge" ), Qt::CaseInsensitive );
  }

  bool isVertexPointType( const QString &type )
  {
    return type.contains( QStringLiteral( "顶点" ) )
           || type.contains( QStringLiteral( "vertex" ), Qt::CaseInsensitive );
  }

  bool isSurfacePointType( const QString &type )
  {
    return type.contains( QStringLiteral( "曲面点" ) )
           || type.contains( QStringLiteral( "surface" ), Qt::CaseInsensitive );
  }

  QList<BuildingRoof::RoofPoint> boundaryPoints( const QList<BuildingRoof::RoofPoint> &roofPoints )
  {
    QList<BuildingRoof::RoofPoint> points;
    for ( const BuildingRoof::RoofPoint &roofPoint : roofPoints )
    {
      if ( !isGroundPointType( roofPoint.type )
           && !isRidgePointType( roofPoint.type )
           && !isVertexPointType( roofPoint.type )
           && !isSurfacePointType( roofPoint.type ) )
        points.append( roofPoint );
    }
    return points;
  }

  bool hasRidgePoint( const QList<BuildingRoof::RoofPoint> &roofPoints )
  {
    for ( const BuildingRoof::RoofPoint &roofPoint : roofPoints )
    {
      if ( isRidgePointType( roofPoint.type ) )
        return true;
    }
    return false;
  }

  QList<BuildingRoof::RoofPoint> ridgePoints( const QList<BuildingRoof::RoofPoint> &roofPoints )
  {
    QList<BuildingRoof::RoofPoint> points;
    for ( const BuildingRoof::RoofPoint &roofPoint : roofPoints )
    {
      if ( isRidgePointType( roofPoint.type ) )
        points.append( roofPoint );
    }
    return points;
  }

  QList<BuildingRoof::RoofPoint> vertexPoints( const QList<BuildingRoof::RoofPoint> &roofPoints )
  {
    QList<BuildingRoof::RoofPoint> points;
    for ( const BuildingRoof::RoofPoint &roofPoint : roofPoints )
    {
      if ( isVertexPointType( roofPoint.type ) )
        points.append( roofPoint );
    }
    return points;
  }

  QList<BuildingRoof::RoofPoint> surfacePoints( const QList<BuildingRoof::RoofPoint> &roofPoints )
  {
    QList<BuildingRoof::RoofPoint> points;
    for ( const BuildingRoof::RoofPoint &roofPoint : roofPoints )
    {
      if ( isSurfacePointType( roofPoint.type ) )
        points.append( roofPoint );
    }
    return points;
  }

  QgsPolygonXY firstPolygon( const QgsGeometry &geometry )
  {
    QgsPolygonXY polygon = geometry.asPolygon();
    if ( polygon.isEmpty() )
    {
      const QgsMultiPolygonXY multiPolygon = geometry.asMultiPolygon();
      if ( !multiPolygon.isEmpty() )
        polygon = multiPolygon.first();
    }
    return polygon;
  }

  QVector<QgsPointXY> exteriorRing( const QgsPolygonXY &polygon )
  {
    QVector<QgsPointXY> ring;
    if ( polygon.isEmpty() )
      return ring;

    for ( const QgsPointXY &point : polygon.first() )
    {
      if ( ring.isEmpty() || point != ring.last() )
        ring.append( point );
    }

    if ( ring.size() >= 2 && ring.first() == ring.last() )
      ring.removeLast();

    double area = 0.0;
    for ( int i = 0; i < ring.size(); ++i )
    {
      const QgsPointXY &a = ring[i];
      const QgsPointXY &b = ring[( i + 1 ) % ring.size()];
      area += a.x() * b.y() - b.x() * a.y();
    }
    if ( area < 0.0 )
      std::reverse( ring.begin(), ring.end() );

    return ring;
  }

  QgsGeometry polygonGeometryFromRing( const QVector<QgsPointXY> &ring )
  {
    if ( ring.size() < 3 )
      return QgsGeometry();

    QgsPolylineXY exterior;
    exterior.reserve( ring.size() + 1 );
    for ( const QgsPointXY &point : ring )
      exterior.append( point );
    exterior.append( ring.first() );

    QgsPolygonXY polygon;
    polygon.append( exterior );
    return QgsGeometry::fromPolygonXY( polygon );
  }

  QVector<QVector<QgsPointXY>> exteriorRingsFromGeometry( const QgsGeometry &geometry )
  {
    QVector<QVector<QgsPointXY>> rings;
    if ( geometry.isNull() || geometry.isEmpty() )
      return rings;

    const QgsPolygonXY polygon = geometry.asPolygon();
    if ( !polygon.isEmpty() )
    {
      const QVector<QgsPointXY> ring = exteriorRing( polygon );
      if ( ring.size() >= 3 )
        rings.append( ring );
      return rings;
    }

    const QgsMultiPolygonXY multiPolygon = geometry.asMultiPolygon();
    for ( const QgsPolygonXY &part : multiPolygon )
    {
      const QVector<QgsPointXY> ring = exteriorRing( part );
      if ( ring.size() >= 3 )
        rings.append( ring );
    }
    return rings;
  }

  QVector<QgsPointXY> openRing( const QgsPolylineXY &ring )
  {
    QVector<QgsPointXY> points;
    for ( const QgsPointXY &point : ring )
    {
      if ( points.isEmpty() || point != points.last() )
        points.append( point );
    }
    if ( points.size() > 1 && points.first() == points.last() )
      points.removeLast();
    return points;
  }

  QVector<QgsPointXY> geometryRingPoints( const QgsGeometry &geometry )
  {
    QVector<QgsPointXY> points;
    auto appendPolygon = [&points]( const QgsPolygonXY &polygon ) {
      for ( const QgsPolylineXY &ring : polygon )
      {
        for ( const QgsPointXY &point : openRing( ring ) )
        {
          bool exists = false;
          for ( const QgsPointXY &existing : points )
          {
            if ( existing == point )
            {
              exists = true;
              break;
            }
          }
          if ( !exists )
            points.append( point );
        }
      }
    };

    const QgsPolygonXY polygon = geometry.asPolygon();
    if ( !polygon.isEmpty() )
      appendPolygon( polygon );
    else
    {
      const QgsMultiPolygonXY multiPolygon = geometry.asMultiPolygon();
      for ( const QgsPolygonXY &part : multiPolygon )
        appendPolygon( part );
    }
    return points;
  }

  QVector<int> triangulateRing( const QVector<QgsPointXY> &ring )
  {
    QVector<int> result;
    if ( ring.size() < 3 )
      return result;

    auto pointInTri = []( const QgsPointXY &p, const QgsPointXY &a, const QgsPointXY &b, const QgsPointXY &c ) {
      const double c1 = ( b.x() - a.x() ) * ( p.y() - a.y() ) - ( b.y() - a.y() ) * ( p.x() - a.x() );
      const double c2 = ( c.x() - b.x() ) * ( p.y() - b.y() ) - ( c.y() - b.y() ) * ( p.x() - b.x() );
      const double c3 = ( a.x() - c.x() ) * ( p.y() - c.y() ) - ( a.y() - c.y() ) * ( p.x() - c.x() );
      return c1 >= -1e-10 && c2 >= -1e-10 && c3 >= -1e-10;
    };

    QVector<int> vertices;
    for ( int i = 0; i < ring.size(); ++i )
      vertices.push_back( i );

    while ( vertices.size() > 3 )
    {
      bool earFound = false;
      for ( int i = 0; i < vertices.size(); ++i )
      {
        const int prev = vertices[( i - 1 + vertices.size() ) % vertices.size()];
        const int curr = vertices[i];
        const int next = vertices[( i + 1 ) % vertices.size()];
        const QgsPointXY &a = ring[prev];
        const QgsPointXY &b = ring[curr];
        const QgsPointXY &c = ring[next];

        const double cross = ( b.x() - a.x() ) * ( c.y() - a.y() ) - ( b.y() - a.y() ) * ( c.x() - a.x() );
        if ( cross <= 0.0 )
          continue;

        bool hasInside = false;
        for ( int j = 0; j < vertices.size(); ++j )
        {
          const int test = vertices[j];
          if ( test == prev || test == curr || test == next )
            continue;
          if ( pointInTri( ring[test], a, b, c ) )
          {
            hasInside = true;
            break;
          }
        }

        if ( hasInside )
          continue;

        result << prev << curr << next;
        vertices.removeAt( i );
        earFound = true;
        break;
      }

      if ( !earFound )
        break;
    }

    if ( vertices.size() == 3 )
      result << vertices[0] << vertices[1] << vertices[2];

    return result;
  }

  void appendVerticalWall( BuildingRoof::Mesh &mesh, const QVector<QgsPointXY> &ring, double lowerZ, double upperZ )
  {
    if ( ring.size() < 2 || std::fabs( upperZ - lowerZ ) <= 1e-8 )
      return;

    const int count = ring.size();
    for ( int i = 0; i < count; ++i )
    {
      const QgsPointXY &a = ring[i];
      const QgsPointXY &b = ring[( i + 1 ) % count];
      const int offset = mesh.vertices.size();
      mesh.vertices.append( QgsPoint( a.x(), a.y(), lowerZ ) );
      mesh.vertices.append( QgsPoint( b.x(), b.y(), lowerZ ) );
      mesh.vertices.append( QgsPoint( a.x(), a.y(), upperZ ) );
      mesh.vertices.append( QgsPoint( b.x(), b.y(), upperZ ) );
      mesh.indices << offset << offset + 1 << offset + 2;
      mesh.indices << offset + 2 << offset + 1 << offset + 3;
    }
  }

  void appendVerticalWallSegment( BuildingRoof::Mesh &mesh, const QgsPointXY &a, const QgsPointXY &b, double lowerZ, double upperZ )
  {
    if ( std::hypot( b.x() - a.x(), b.y() - a.y() ) <= 1e-8 || upperZ - lowerZ <= 1e-8 )
      return;

    const int offset = mesh.vertices.size();
    mesh.vertices.append( QgsPoint( a.x(), a.y(), lowerZ ) );
    mesh.vertices.append( QgsPoint( b.x(), b.y(), lowerZ ) );
    mesh.vertices.append( QgsPoint( a.x(), a.y(), upperZ ) );
    mesh.vertices.append( QgsPoint( b.x(), b.y(), upperZ ) );
    mesh.indices << offset << offset + 1 << offset + 2;
    mesh.indices << offset + 2 << offset + 1 << offset + 3;
  }

  void appendVerticalWallsAlongLines( BuildingRoof::Mesh &mesh, const QgsGeometry &geometry, double lowerZ, double upperZ )
  {
    auto appendLine = [&]( const QgsPolylineXY &line ) {
      for ( int i = 1; i < line.size(); ++i )
        appendVerticalWallSegment( mesh, line.at( i - 1 ), line.at( i ), lowerZ, upperZ );
    };

    const QgsPolylineXY line = geometry.asPolyline();
    if ( !line.isEmpty() )
    {
      appendLine( line );
      return;
    }
    const QgsMultiPolylineXY lines = geometry.asMultiPolyline();
    for ( const QgsPolylineXY &part : lines )
      appendLine( part );
    if ( !lines.isEmpty() )
      return;
    for ( const QgsGeometry &part : geometry.asGeometryCollection() )
    {
      const QgsPolylineXY partLine = part.asPolyline();
      if ( !partLine.isEmpty() )
        appendLine( partLine );
    }
  }

  void appendHorizontalRingSurface( BuildingRoof::Mesh &mesh, const QVector<QgsPointXY> &ring, double z, bool flip = false )
  {
    if ( ring.size() < 3 )
      return;

    QVector<QgsPointXY> localRing = ring;
    double area = 0.0;
    for ( int i = 0; i < localRing.size(); ++i )
    {
      const QgsPointXY &a = localRing[i];
      const QgsPointXY &b = localRing[( i + 1 ) % localRing.size()];
      area += a.x() * b.y() - b.x() * a.y();
    }
    if ( area < 0.0 )
      std::reverse( localRing.begin(), localRing.end() );

    const int offset = mesh.vertices.size();
    for ( const QgsPointXY &point : localRing )
      mesh.vertices.append( QgsPoint( point.x(), point.y(), z ) );

    const QVector<int> triangles = triangulateRing( localRing );
    for ( int i = 0; i + 2 < triangles.size(); i += 3 )
    {
      if ( flip )
        mesh.indices << offset + triangles[i] << offset + triangles[i + 2] << offset + triangles[i + 1];
      else
        mesh.indices << offset + triangles[i] << offset + triangles[i + 1] << offset + triangles[i + 2];
    }
  }

  void appendApexRoofSurface( BuildingRoof::Mesh &mesh, const QVector<QgsPointXY> &ring, double eaveHeight, const QgsPoint &apexPoint )
  {
    if ( ring.size() < 3 )
      return;

    const int count = ring.size();
    for ( int i = 0; i < count; ++i )
    {
      const QgsPointXY &a = ring[i];
      const QgsPointXY &b = ring[( i + 1 ) % count];
      const int offset = mesh.vertices.size();
      mesh.vertices.append( QgsPoint( a.x(), a.y(), eaveHeight ) );
      mesh.vertices.append( QgsPoint( b.x(), b.y(), eaveHeight ) );
      mesh.vertices.append( apexPoint );
      mesh.indices << offset << offset + 1 << offset + 2;
    }
  }

  void appendApexStructureLines( BuildingRoof::Mesh &mesh, const QVector<QgsPointXY> &ring, double eaveHeight, const QgsPoint &apexPoint, int maxSpokes )
  {
    if ( ring.size() < 3 || maxSpokes <= 0 )
      return;

    const int count = ring.size();
    const int targetCount = std::min( count, maxSpokes );
    QVector<int> emittedIndices;
    emittedIndices.reserve( targetCount );
    for ( int i = 0; i < targetCount; ++i )
    {
      const int ringIndex = targetCount == count ? i : std::min( count - 1, static_cast<int>( std::floor( static_cast<double>( i ) * count / targetCount ) ) );
      if ( emittedIndices.contains( ringIndex ) )
        continue;
      emittedIndices.append( ringIndex );

      const QgsPointXY &ringPoint = ring.at( ringIndex );
      const QgsPoint eavePoint( ringPoint.x(), ringPoint.y(), eaveHeight );
      mesh.structureLines.append( qMakePair( apexPoint, eavePoint ) );
    }
  }

  double distanceToRing2( const QVector<QgsPointXY> &ring, const QgsPointXY &point );

  void appendHorizontalGeometrySurface( BuildingRoof::Mesh &mesh, const QgsGeometry &geometry, double z, bool flip = false )
  {
    if ( geometry.isNull() || geometry.isEmpty() )
      return;

    const QVector<QgsPointXY> points = geometryRingPoints( geometry );
    if ( points.size() < 3 )
      return;

    QgsMultiPointXY pointSet;
    for ( const QgsPointXY &point : points )
      pointSet.append( point );

    QgsGeometry tin = QgsGeometry::fromMultiPointXY( pointSet ).delaunayTriangulation( 0.0, false );
    QVector<QgsGeometry> triangles = tin.asGeometryCollection();
    if ( triangles.isEmpty() && !tin.isNull() )
      triangles.append( tin );

    for ( const QgsGeometry &triangleGeometry : triangles )
    {
      const QgsPolygonXY triangle = triangleGeometry.asPolygon();
      if ( triangle.isEmpty() || triangle.first().size() < 4 )
        continue;

      const QgsPolylineXY triangleRing = triangle.first();
      const QgsPointXY a = triangleRing.at( 0 );
      const QgsPointXY b = triangleRing.at( 1 );
      const QgsPointXY c = triangleRing.at( 2 );
      const QgsPointXY centroid( ( a.x() + b.x() + c.x() ) / 3.0, ( a.y() + b.y() + c.y() ) / 3.0 );
      if ( !geometry.contains( QgsGeometry::fromPointXY( centroid ) ) )
        continue;

      const int offset = mesh.vertices.size();
      mesh.vertices.append( QgsPoint( a.x(), a.y(), z ) );
      mesh.vertices.append( QgsPoint( b.x(), b.y(), z ) );
      mesh.vertices.append( QgsPoint( c.x(), c.y(), z ) );
      if ( flip )
        mesh.indices << offset << offset + 2 << offset + 1;
      else
        mesh.indices << offset << offset + 1 << offset + 2;
    }
  }

  void appendClippedHorizontalGeometrySurface( BuildingRoof::Mesh &mesh, const QgsGeometry &geometry, double z )
  {
    const QVector<QgsPointXY> points = geometryRingPoints( geometry );
    if ( points.size() < 3 )
      return;

    QgsGeometry tin;
    try
    {
      tin = geometry.constrainedDelaunayTriangulation();
    }
    catch ( const QgsNotSupportedException & )
    {
      // Older GEOS builds use the clipped Delaunay fallback below.
    }
    const bool constrained = !tin.isNull() && !tin.isEmpty();
    if ( !constrained )
    {
      QgsMultiPointXY pointSet;
      for ( const QgsPointXY &point : points )
        pointSet.append( point );
      tin = QgsGeometry::fromMultiPointXY( pointSet ).delaunayTriangulation( 0.0, false );
    }
    QVector<QgsGeometry> triangles = tin.asGeometryCollection();
    if ( triangles.isEmpty() && !tin.isNull() )
      triangles.append( tin );

    for ( const QgsGeometry &triangle : triangles )
    {
      const QgsGeometry clipped = constrained ? triangle : triangle.intersection( geometry );
      if ( clipped.isNull() || clipped.isEmpty() )
        continue;

      QVector<QgsPolygonXY> polygons;
      const QgsPolygonXY polygon = clipped.asPolygon();
      if ( !polygon.isEmpty() )
        polygons.append( polygon );
      else
        polygons = clipped.asMultiPolygon();
      for ( const QgsPolygonXY &part : polygons )
      {
        const QVector<QgsPointXY> ring = exteriorRing( part );
        const QVector<int> indices = triangulateRing( ring );
        if ( indices.isEmpty() )
          continue;
        const int offset = mesh.vertices.size();
        for ( const QgsPointXY &point : ring )
          mesh.vertices.append( QgsPoint( point.x(), point.y(), z ) );
        for ( int i = 0; i + 2 < indices.size(); i += 3 )
          mesh.indices << offset + indices.at( i ) << offset + indices.at( i + 1 ) << offset + indices.at( i + 2 );
      }
    }
  }

  double distanceToRings2( const QVector<QVector<QgsPointXY>> &rings, const QgsPointXY &point )
  {
    double bestDistance = std::numeric_limits<double>::max();
    for ( const QVector<QgsPointXY> &ring : rings )
    {
      if ( ring.size() >= 2 )
        bestDistance = std::min( bestDistance, distanceToRing2( ring, point ) );
    }
    return bestDistance;
  }

  double flatTopHippedRoofZ( const QgsPointXY &point, const QVector<QgsPointXY> &outerRing, const QVector<QVector<QgsPointXY>> &topRings, double baseHeight, double topHeight )
  {
    const double outerDistance = std::sqrt( std::max( 0.0, distanceToRing2( outerRing, point ) ) );
    const double topDistance = std::sqrt( std::max( 0.0, distanceToRings2( topRings, point ) ) );
    const double total = outerDistance + topDistance;
    if ( total <= 1e-8 )
      return topHeight;

    const double t = std::max( 0.0, std::min( 1.0, outerDistance / total ) );
    return baseHeight + ( topHeight - baseHeight ) * t;
  }

  void appendSlopedFlatTopHippedSurface( BuildingRoof::Mesh &mesh, const QgsGeometry &geometry, const QVector<QgsPointXY> &outerRing, const QVector<QVector<QgsPointXY>> &topRings, double baseHeight, double topHeight )
  {
    if ( geometry.isNull() || geometry.isEmpty() || outerRing.size() < 3 || topRings.isEmpty() )
      return;

    const QVector<QgsPointXY> points = geometryRingPoints( geometry );
    if ( points.size() < 3 )
      return;

    QgsMultiPointXY pointSet;
    for ( const QgsPointXY &point : points )
      pointSet.append( point );

    QgsGeometry tin = QgsGeometry::fromMultiPointXY( pointSet ).delaunayTriangulation( 0.0, false );
    QVector<QgsGeometry> triangles = tin.asGeometryCollection();
    if ( triangles.isEmpty() && !tin.isNull() )
      triangles.append( tin );

    for ( const QgsGeometry &triangleGeometry : triangles )
    {
      const QgsPolygonXY triangle = triangleGeometry.asPolygon();
      if ( triangle.isEmpty() || triangle.first().size() < 4 )
        continue;

      const QgsPolylineXY triangleRing = triangle.first();
      const QgsPointXY a = triangleRing.at( 0 );
      const QgsPointXY b = triangleRing.at( 1 );
      const QgsPointXY c = triangleRing.at( 2 );
      const QgsPointXY centroid( ( a.x() + b.x() + c.x() ) / 3.0, ( a.y() + b.y() + c.y() ) / 3.0 );
      if ( !geometry.contains( QgsGeometry::fromPointXY( centroid ) ) )
        continue;

      const int offset = mesh.vertices.size();
      mesh.vertices.append( QgsPoint( a.x(), a.y(), flatTopHippedRoofZ( a, outerRing, topRings, baseHeight, topHeight ) ) );
      mesh.vertices.append( QgsPoint( b.x(), b.y(), flatTopHippedRoofZ( b, outerRing, topRings, baseHeight, topHeight ) ) );
      mesh.vertices.append( QgsPoint( c.x(), c.y(), flatTopHippedRoofZ( c, outerRing, topRings, baseHeight, topHeight ) ) );
      mesh.indices << offset << offset + 1 << offset + 2;
    }
  }

  double signedDistanceToLine( const QgsPointXY &point, const QgsPoint &linePoint, double normalX, double normalY )
  {
    return ( point.x() - linePoint.x() ) * normalX + ( point.y() - linePoint.y() ) * normalY;
  }

  bool nearlySamePoint( const QgsPointXY &a, const QgsPointXY &b )
  {
    return std::hypot( a.x() - b.x(), a.y() - b.y() ) <= 1e-8;
  }

  void appendUniquePoint( QVector<QgsPointXY> &points, const QgsPointXY &point )
  {
    if ( points.isEmpty() || !nearlySamePoint( points.last(), point ) )
      points.append( point );
  }

  void appendPointIfAbsent( QVector<QgsPointXY> &points, const QgsPointXY &point )
  {
    for ( const QgsPointXY &existing : points )
    {
      if ( nearlySamePoint( existing, point ) )
        return;
    }
    points.append( point );
  }

  double profileDistance( const QgsPointXY &point, double normalX, double normalY )
  {
    return point.x() * normalX + point.y() * normalY;
  }

  QVector<QgsPointXY> clipPolygonByProfileDistance( const QVector<QgsPointXY> &polygon, double normalX, double normalY, double threshold, bool keepGreater )
  {
    QVector<QgsPointXY> result;
    if ( polygon.isEmpty() )
      return result;

    constexpr double epsilon = 1e-8;
    auto inside = [&]( const QgsPointXY &point ) {
      const double s = profileDistance( point, normalX, normalY );
      return keepGreater ? s >= threshold - epsilon : s <= threshold + epsilon;
    };

    for ( int i = 0; i < polygon.size(); ++i )
    {
      const QgsPointXY &current = polygon[i];
      const QgsPointXY &previous = polygon[( i - 1 + polygon.size() ) % polygon.size()];
      const bool currentInside = inside( current );
      const bool previousInside = inside( previous );
      const double currentS = profileDistance( current, normalX, normalY );
      const double previousS = profileDistance( previous, normalX, normalY );

      if ( currentInside != previousInside )
      {
        const double denom = currentS - previousS;
        if ( std::fabs( denom ) > 1e-12 )
        {
          const double t = ( threshold - previousS ) / denom;
          appendUniquePoint( result, QgsPointXY( previous.x() + t * ( current.x() - previous.x() ), previous.y() + t * ( current.y() - previous.y() ) ) );
        }
      }

      if ( currentInside )
        appendUniquePoint( result, current );
    }

    if ( result.size() > 1 && nearlySamePoint( result.first(), result.last() ) )
      result.removeLast();
    return result;
  }

  QVector<QgsPointXY> clipPolygonByProfileRange( const QVector<QgsPointXY> &polygon, double normalX, double normalY, double minS, double maxS )
  {
    QVector<QgsPointXY> result = clipPolygonByProfileDistance( polygon, normalX, normalY, minS, true );
    result = clipPolygonByProfileDistance( result, normalX, normalY, maxS, false );
    return result;
  }

  QVector<QgsPointXY> ringWithProfileIntersections( const QVector<QgsPointXY> &ring, double normalX, double normalY, const QVector<double> &profileDistances )
  {
    QVector<QgsPointXY> result;
    constexpr double epsilon = 1e-8;

    for ( int i = 0; i < ring.size(); ++i )
    {
      const QgsPointXY &a = ring[i];
      const QgsPointXY &b = ring[( i + 1 ) % ring.size()];
      const double sa = profileDistance( a, normalX, normalY );
      const double sb = profileDistance( b, normalX, normalY );

      appendUniquePoint( result, a );
      if ( std::fabs( sb - sa ) <= 1e-12 )
        continue;

      QVector<QPair<double, QgsPointXY>> intersections;
      for ( double profileS : profileDistances )
      {
        if ( ( profileS - sa ) * ( profileS - sb ) < -epsilon )
        {
          const double t = ( profileS - sa ) / ( sb - sa );
          intersections.append( qMakePair( t, QgsPointXY( a.x() + t * ( b.x() - a.x() ), a.y() + t * ( b.y() - a.y() ) ) ) );
        }
      }
      std::sort( intersections.begin(), intersections.end(), []( const QPair<double, QgsPointXY> &lhs, const QPair<double, QgsPointXY> &rhs ) {
        return lhs.first < rhs.first;
      } );
      for ( const QPair<double, QgsPointXY> &intersection : intersections )
        appendUniquePoint( result, intersection.second );
    }

    if ( result.size() > 1 && nearlySamePoint( result.first(), result.last() ) )
      result.removeLast();
    return result;
  }

  double pointSegmentDistance2( const QgsPointXY &point, const QgsPointXY &a, const QgsPointXY &b )
  {
    const double dx = b.x() - a.x();
    const double dy = b.y() - a.y();
    const double len2 = dx * dx + dy * dy;
    if ( len2 <= 1e-12 )
      return std::pow( point.x() - a.x(), 2.0 ) + std::pow( point.y() - a.y(), 2.0 );

    const double t = std::max( 0.0, std::min( 1.0, ( ( point.x() - a.x() ) * dx + ( point.y() - a.y() ) * dy ) / len2 ) );
    const double px = a.x() + t * dx;
    const double py = a.y() + t * dy;
    return std::pow( point.x() - px, 2.0 ) + std::pow( point.y() - py, 2.0 );
  }

  double pointSegmentParameter( const QgsPointXY &point, const QgsPointXY &a, const QgsPointXY &b )
  {
    const double dx = b.x() - a.x();
    const double dy = b.y() - a.y();
    const double len2 = dx * dx + dy * dy;
    if ( len2 <= 1e-12 )
      return 0.5;
    return std::max( 0.0, std::min( 1.0, ( ( point.x() - a.x() ) * dx + ( point.y() - a.y() ) * dy ) / len2 ) );
  }

  double distanceToRing2( const QVector<QgsPointXY> &ring, const QgsPointXY &point )
  {
    double bestDistance = std::numeric_limits<double>::max();
    for ( int i = 0; i < ring.size(); ++i )
      bestDistance = std::min( bestDistance, pointSegmentDistance2( point, ring[i], ring[( i + 1 ) % ring.size()] ) );
    return bestDistance;
  }

  QVector<int> dbscanRegionQuery( const QVector<ClusterSample> &samples, int index, double eps )
  {
    QVector<int> neighbors;
    const double eps2 = eps * eps;
    const QgsPointXY &point = samples.at( index ).point;
    for ( int i = 0; i < samples.size(); ++i )
    {
      const double dx = point.x() - samples.at( i ).point.x();
      const double dy = point.y() - samples.at( i ).point.y();
      if ( dx * dx + dy * dy <= eps2 )
        neighbors.append( i );
    }
    return neighbors;
  }

  void dbscanExpandCluster( QVector<ClusterSample> &samples, int index, QVector<int> neighbors, int clusterId, double eps, int minPts )
  {
    samples[index].cluster = clusterId;
    for ( int cursor = 0; cursor < neighbors.size(); ++cursor )
    {
      const int neighborIndex = neighbors.at( cursor );
      ClusterSample &neighbor = samples[neighborIndex];
      if ( !neighbor.visited )
      {
        neighbor.visited = true;
        const QVector<int> nextNeighbors = dbscanRegionQuery( samples, neighborIndex, eps );
        if ( nextNeighbors.size() >= minPts )
        {
          for ( int next : nextNeighbors )
          {
            if ( !neighbors.contains( next ) )
              neighbors.append( next );
          }
        }
      }
      if ( neighbor.cluster < 0 )
        neighbor.cluster = clusterId;
    }
  }

  int assignDbscanClusters( QVector<ClusterSample> &samples, double eps, int minPts )
  {
    int clusterId = 0;
    for ( int i = 0; i < samples.size(); ++i )
    {
      if ( samples[i].visited )
        continue;

      samples[i].visited = true;
      const QVector<int> neighbors = dbscanRegionQuery( samples, i, eps );
      if ( neighbors.size() < minPts )
        continue;

      dbscanExpandCluster( samples, i, neighbors, clusterId, eps, minPts );
      ++clusterId;
    }
    return clusterId;
  }

  double completePlaneDbscanRadius( const QVector<QgsPoint> &points, double extent )
  {
    QgsSpatialIndex neighborIndex;
    bool indexed = true;
    for ( int i = 0; i < points.size(); ++i )
    {
      const QgsPoint &point = points.at( i );
      if ( !neighborIndex.addFeature( i, QgsRectangle( point.x(), point.y(), point.x(), point.y() ) ) )
      {
        indexed = false;
        break;
      }
    }
    QVector<double> eighthNeighborDistances;
    const int stride = std::max( 1, static_cast<int>( std::ceil( points.size() / 400.0 ) ) );
    for ( int i = 0; i < points.size(); i += stride )
    {
      double nearest[8];
      std::fill( nearest, nearest + 8, std::numeric_limits<double>::max() );
      const auto considerNeighbor = [&]( int j ) {
        if ( i == j )
          return;
        const double distance = std::hypot( points.at( i ).x() - points.at( j ).x(),
                                            points.at( i ).y() - points.at( j ).y() );
        if ( distance >= nearest[7] )
          return;
        int position = 7;
        while ( position > 0 && distance < nearest[position - 1] )
        {
          nearest[position] = nearest[position - 1];
          --position;
        }
        nearest[position] = distance;
      };
      if ( indexed )
      {
        // Include the query point, then discard it to retain eight other neighbors.
        const QgsPoint &center = points.at( i );
        for ( QgsFeatureId id : neighborIndex.nearestNeighbor( QgsPointXY( center.x(), center.y() ), 9 ) )
          considerNeighbor( static_cast<int>( id ) );
      }
      else
      {
        for ( int j = 0; j < points.size(); ++j )
          considerNeighbor( j );
      }
      if ( std::isfinite( nearest[7] ) )
        eighthNeighborDistances.append( nearest[7] );
    }
    if ( eighthNeighborDistances.isEmpty() )
      return std::max( 0.15, std::min( 0.50, extent * 0.015 ) );
    std::sort( eighthNeighborDistances.begin(), eighthNeighborDistances.end() );
    const double radius = eighthNeighborDistances.at( eighthNeighborDistances.size() / 2 ) * 1.35;
    return std::min( std::max( 0.50, extent * 0.08 ),
                     std::max( std::max( 0.15, std::min( 0.50, extent * 0.015 ) ), radius ) );
  }

  int assignDbscanClustersSpatial( QVector<ClusterSample> &samples, double eps, int minPts, double verticalTolerance )
  {
    if ( samples.isEmpty() || eps <= 0.0 )
      return 0;
    double originX = samples.first().point.x();
    double originY = samples.first().point.y();
    for ( const ClusterSample &sample : samples )
    {
      originX = std::min( originX, sample.point.x() );
      originY = std::min( originY, sample.point.y() );
    }
    auto cell = [&]( const QgsPointXY &point ) {
      return std::make_pair( static_cast<int>( std::floor( ( point.x() - originX ) / eps ) ),
                             static_cast<int>( std::floor( ( point.y() - originY ) / eps ) ) );
    };
    std::map<std::pair<int, int>, QVector<int>> grid;
    for ( int i = 0; i < samples.size(); ++i )
      grid[cell( samples.at( i ).point )].append( i );

    auto neighborsOf = [&]( int index ) {
      QVector<int> neighbors;
      const std::pair<int, int> location = cell( samples.at( index ).point );
      const QgsPointXY &point = samples.at( index ).point;
      const double eps2 = eps * eps;
      for ( int dx = -1; dx <= 1; ++dx )
      {
        for ( int dy = -1; dy <= 1; ++dy )
        {
          const auto found = grid.find( std::make_pair( location.first + dx, location.second + dy ) );
          if ( found == grid.end() )
            continue;
          for ( int candidate : found->second )
          {
            const QgsPointXY &other = samples.at( candidate ).point;
            const double deltaX = point.x() - other.x();
            const double deltaY = point.y() - other.y();
            if ( deltaX * deltaX + deltaY * deltaY <= eps2
                 && std::fabs( samples.at( index ).u - samples.at( candidate ).u ) <= verticalTolerance )
              neighbors.append( candidate );
          }
        }
      }
      return neighbors;
    };

    int clusterCount = 0;
    QVector<unsigned char> queued( samples.size(), 0 );
    for ( int i = 0; i < samples.size(); ++i )
    {
      if ( samples.at( i ).visited )
        continue;
      samples[i].visited = true;
      QVector<int> neighbors = neighborsOf( i );
      if ( neighbors.size() < minPts )
        continue;
      samples[i].cluster = clusterCount;
      QVector<int> queue;
      for ( int neighbor : neighbors )
      {
        if ( !queued.at( neighbor ) )
        {
          queued[neighbor] = 1;
          queue.append( neighbor );
        }
      }
      for ( int cursor = 0; cursor < queue.size(); ++cursor )
      {
        const int neighbor = queue.at( cursor );
        if ( !samples.at( neighbor ).visited )
        {
          samples[neighbor].visited = true;
          const QVector<int> next = neighborsOf( neighbor );
          if ( next.size() >= minPts )
          {
            for ( int candidate : next )
            {
              if ( !queued.at( candidate ) )
              {
                queued[candidate] = 1;
                queue.append( candidate );
              }
            }
          }
        }
        if ( samples.at( neighbor ).cluster < 0 )
          samples[neighbor].cluster = clusterCount;
      }
      for ( int index : queue )
        queued[index] = 0;
      ++clusterCount;
    }
    return clusterCount;
  }

  QVector<QgsPointXY> largestClusterBox( const QVector<ClusterSample> &samples, int clusterCount, double axisX, double axisY, double normalX, double normalY, double padding )
  {
    if ( clusterCount <= 0 )
      return {};

    QVector<int> counts( clusterCount, 0 );
    for ( const ClusterSample &sample : samples )
    {
      if ( sample.cluster >= 0 && sample.cluster < clusterCount )
        ++counts[sample.cluster];
    }

    int bestCluster = 0;
    for ( int i = 1; i < counts.size(); ++i )
    {
      if ( counts.at( i ) > counts.at( bestCluster ) )
        bestCluster = i;
    }
    if ( counts.at( bestCluster ) < 3 )
      return {};

    double minU = std::numeric_limits<double>::max();
    double maxU = -std::numeric_limits<double>::max();
    double minV = std::numeric_limits<double>::max();
    double maxV = -std::numeric_limits<double>::max();
    for ( const ClusterSample &sample : samples )
    {
      if ( sample.cluster != bestCluster )
        continue;
      minU = std::min( minU, sample.u );
      maxU = std::max( maxU, sample.u );
      minV = std::min( minV, sample.v );
      maxV = std::max( maxV, sample.v );
    }

    minU -= padding;
    maxU += padding;
    minV -= padding;
    maxV += padding;
    auto fromLocal = [&]( double u, double v ) {
      return QgsPointXY( axisX * u + normalX * v, axisY * u + normalY * v );
    };
    return QVector<QgsPointXY>{ fromLocal( minU, minV ), fromLocal( maxU, minV ), fromLocal( maxU, maxV ), fromLocal( minU, maxV ) };
  }

  QVector<QgsPointXY> regularizeReliefRingToFootprint( const QVector<QgsPointXY> &reliefRing, const QVector<QgsPointXY> &footprintRing, double axisX, double axisY, double normalX, double normalY, double snapTolerance )
  {
    if ( reliefRing.size() < 3 || footprintRing.size() < 3 )
      return reliefRing;

    constexpr double directionTolerance = 0.9659258262890683; // cos(15 degrees)
    double bestScore = -1.0;
    double bestAxisX = axisX;
    double bestAxisY = axisY;
    for ( int i = 0; i < footprintRing.size(); ++i )
    {
      const QgsPointXY &a = footprintRing.at( i );
      const QgsPointXY &b = footprintRing.at( ( i + 1 ) % footprintRing.size() );
      double edgeX = b.x() - a.x();
      double edgeY = b.y() - a.y();
      const double length = std::hypot( edgeX, edgeY );
      if ( length <= 1e-8 )
        continue;

      edgeX /= length;
      edgeY /= length;
      const double signedDot = edgeX * axisX + edgeY * axisY;
      const double score = std::fabs( signedDot );
      if ( score > bestScore )
      {
        bestScore = score;
        bestAxisX = signedDot < 0.0 ? -edgeX : edgeX;
        bestAxisY = signedDot < 0.0 ? -edgeY : edgeY;
      }
    }

    if ( bestScore >= directionTolerance )
    {
      axisX = bestAxisX;
      axisY = bestAxisY;
      normalX = -axisY;
      normalY = axisX;
    }

    double minU = std::numeric_limits<double>::max();
    double maxU = -std::numeric_limits<double>::max();
    double minV = std::numeric_limits<double>::max();
    double maxV = -std::numeric_limits<double>::max();
    for ( const QgsPointXY &point : reliefRing )
    {
      const double u = point.x() * axisX + point.y() * axisY;
      const double v = point.x() * normalX + point.y() * normalY;
      minU = std::min( minU, u );
      maxU = std::max( maxU, u );
      minV = std::min( minV, v );
      maxV = std::max( maxV, v );
    }

    if ( maxU - minU <= 1e-8 || maxV - minV <= 1e-8 )
      return reliefRing;

    auto snapToFootprintLine = [&]( double value, bool snapU ) {
      double bestValue = value;
      double bestDistance = std::max( 0.0, snapTolerance );
      for ( int i = 0; i < footprintRing.size(); ++i )
      {
        const QgsPointXY &a = footprintRing.at( i );
        const QgsPointXY &b = footprintRing.at( ( i + 1 ) % footprintRing.size() );
        double edgeX = b.x() - a.x();
        double edgeY = b.y() - a.y();
        const double length = std::hypot( edgeX, edgeY );
        if ( length <= 1e-8 )
          continue;

        edgeX /= length;
        edgeY /= length;
        const double parallelToAxis = std::fabs( edgeX * axisX + edgeY * axisY );
        const double parallelToNormal = std::fabs( edgeX * normalX + edgeY * normalY );
        if ( snapU )
        {
          if ( parallelToNormal < directionTolerance )
            continue;
          const double edgeValue = a.x() * axisX + a.y() * axisY;
          const double distance = std::fabs( edgeValue - value );
          if ( distance <= bestDistance )
          {
            bestDistance = distance;
            bestValue = edgeValue;
          }
        }
        else
        {
          if ( parallelToAxis < directionTolerance )
            continue;
          const double edgeValue = a.x() * normalX + a.y() * normalY;
          const double distance = std::fabs( edgeValue - value );
          if ( distance <= bestDistance )
          {
            bestDistance = distance;
            bestValue = edgeValue;
          }
        }
      }
      return bestValue;
    };

    const double snappedMinU = snapToFootprintLine( minU, true );
    const double snappedMaxU = snapToFootprintLine( maxU, true );
    const double snappedMinV = snapToFootprintLine( minV, false );
    const double snappedMaxV = snapToFootprintLine( maxV, false );
    if ( snappedMaxU - snappedMinU > 1e-8 )
    {
      minU = snappedMinU;
      maxU = snappedMaxU;
    }
    if ( snappedMaxV - snappedMinV > 1e-8 )
    {
      minV = snappedMinV;
      maxV = snappedMaxV;
    }

    auto fromLocal = [&]( double u, double v ) {
      return QgsPointXY( axisX * u + normalX * v, axisY * u + normalY * v );
    };
    return QVector<QgsPointXY>{ fromLocal( minU, minV ), fromLocal( maxU, minV ), fromLocal( maxU, maxV ), fromLocal( minU, maxV ) };
  }

  QVector<QgsPointXY> regularizeRingAsFootprintInset( const QVector<QgsPointXY> &sourceRing, const QVector<QgsPointXY> &footprintRing )
  {
    if ( sourceRing.size() < 3 || footprintRing.size() < 3 )
      return sourceRing;

    const double extent = ringExtentSize( footprintRing );
    const double minInset = std::max( extent * 0.005, 0.02 );

    struct OffsetLine
    {
      QgsPointXY point;
      double dirX = 0.0;
      double dirY = 0.0;
    };

    QVector<OffsetLine> lines;
    lines.reserve( footprintRing.size() );
    for ( int i = 0; i < footprintRing.size(); ++i )
    {
      const QgsPointXY &a = footprintRing.at( i );
      const QgsPointXY &b = footprintRing.at( ( i + 1 ) % footprintRing.size() );
      double dirX = b.x() - a.x();
      double dirY = b.y() - a.y();
      const double length = std::hypot( dirX, dirY );
      if ( length <= 1e-8 )
        return sourceRing;

      dirX /= length;
      dirY /= length;
      const double inwardX = -dirY;
      const double inwardY = dirX;

      double inset = std::numeric_limits<double>::max();
      for ( const QgsPointXY &point : sourceRing )
      {
        const double distance = ( point.x() - a.x() ) * inwardX + ( point.y() - a.y() ) * inwardY;
        if ( distance > 1e-8 )
          inset = std::min( inset, distance );
      }

      if ( !std::isfinite( inset ) )
        return sourceRing;
      inset = std::max( inset, minInset );
      lines.append( OffsetLine{ QgsPointXY( a.x() + inwardX * inset, a.y() + inwardY * inset ), dirX, dirY } );
    }

    auto intersectLines = []( const OffsetLine &first, const OffsetLine &second, QgsPointXY &intersection ) {
      const double den = cross2d( first.dirX, first.dirY, second.dirX, second.dirY );
      if ( std::fabs( den ) <= 1e-10 )
        return false;

      const double relX = second.point.x() - first.point.x();
      const double relY = second.point.y() - first.point.y();
      const double t = cross2d( relX, relY, second.dirX, second.dirY ) / den;
      intersection = QgsPointXY( first.point.x() + first.dirX * t, first.point.y() + first.dirY * t );
      return true;
    };

    QVector<QgsPointXY> result;
    result.reserve( lines.size() );
    for ( int i = 0; i < lines.size(); ++i )
    {
      QgsPointXY intersection;
      if ( !intersectLines( lines.at( i ), lines.at( ( i + 1 ) % lines.size() ), intersection ) )
        return sourceRing;
      if ( !pointInRing( footprintRing, intersection ) )
        return sourceRing;
      appendUniquePoint( result, intersection );
    }

    if ( result.size() < 3 )
      return sourceRing;
    return result;
  }

  QVector<QgsPointXY> clusterBoxNearPoint( const QVector<ClusterSample> &samples, int clusterCount, const QgsPointXY &targetPoint, double axisX, double axisY, double normalX, double normalY, double padding, double maxSpan )
  {
    if ( clusterCount <= 0 )
      return {};

    QVector<int> counts( clusterCount, 0 );
    QVector<double> minDistance2( clusterCount, std::numeric_limits<double>::max() );
    for ( const ClusterSample &sample : samples )
    {
      if ( sample.cluster < 0 || sample.cluster >= clusterCount )
        continue;

      ++counts[sample.cluster];
      const double dx = sample.point.x() - targetPoint.x();
      const double dy = sample.point.y() - targetPoint.y();
      minDistance2[sample.cluster] = std::min( minDistance2[sample.cluster], dx * dx + dy * dy );
    }

    int bestCluster = -1;
    for ( int i = 0; i < clusterCount; ++i )
    {
      if ( counts.at( i ) < 3 )
        continue;
      if ( bestCluster < 0 || minDistance2.at( i ) < minDistance2.at( bestCluster ) )
        bestCluster = i;
    }
    if ( bestCluster < 0 )
      return {};

    double minU = std::numeric_limits<double>::max();
    double maxU = -std::numeric_limits<double>::max();
    double minV = std::numeric_limits<double>::max();
    double maxV = -std::numeric_limits<double>::max();
    for ( const ClusterSample &sample : samples )
    {
      if ( sample.cluster != bestCluster )
        continue;
      minU = std::min( minU, sample.u );
      maxU = std::max( maxU, sample.u );
      minV = std::min( minV, sample.v );
      maxV = std::max( maxV, sample.v );
    }

    minU -= padding;
    maxU += padding;
    minV -= padding;
    maxV += padding;

    const double targetU = targetPoint.x() * axisX + targetPoint.y() * axisY;
    const double targetV = targetPoint.x() * normalX + targetPoint.y() * normalY;
    auto clampSpan = []( double &minValue, double &maxValue, double center, double spanLimit ) {
      if ( spanLimit <= 1e-8 || maxValue - minValue <= spanLimit )
        return;

      double newMin = center - spanLimit * 0.5;
      double newMax = center + spanLimit * 0.5;
      if ( newMin < minValue )
      {
        newMax += minValue - newMin;
        newMin = minValue;
      }
      if ( newMax > maxValue )
      {
        newMin -= newMax - maxValue;
        newMax = maxValue;
      }
      minValue = std::max( minValue, newMin );
      maxValue = std::min( maxValue, newMax );
    };
    clampSpan( minU, maxU, targetU, maxSpan );
    clampSpan( minV, maxV, targetV, maxSpan );

    auto fromLocal = [&]( double u, double v ) {
      return QgsPointXY( axisX * u + normalX * v, axisY * u + normalY * v );
    };
    return QVector<QgsPointXY>{ fromLocal( minU, minV ), fromLocal( maxU, minV ), fromLocal( maxU, maxV ), fromLocal( minU, maxV ) };
  }

  double estimateDbscanEps( const QVector<ClusterSample> &samples, double fallback )
  {
    if ( samples.size() < 2 )
      return fallback;

    QVector<double> nearestDistances;
    nearestDistances.reserve( samples.size() );
    const int stride = std::max( 1, static_cast<int>( std::ceil( samples.size() / 256.0 ) ) );
    for ( int i = 0; i < samples.size(); i += stride )
    {
      double best = std::numeric_limits<double>::max();
      for ( int j = 0; j < samples.size(); ++j )
      {
        if ( i == j )
          continue;
        const double dx = samples.at( i ).point.x() - samples.at( j ).point.x();
        const double dy = samples.at( i ).point.y() - samples.at( j ).point.y();
        best = std::min( best, std::hypot( dx, dy ) );
      }
      if ( std::isfinite( best ) )
        nearestDistances.append( best );
    }

    if ( nearestDistances.isEmpty() )
      return fallback;

    std::sort( nearestDistances.begin(), nearestDistances.end() );
    const double median = nearestDistances.at( nearestDistances.size() / 2 );
    return std::max( fallback, median * 2.5 );
  }

  bool nearestEdgeDirection( const QVector<QgsPointXY> &ring, const QgsPoint &boundaryPoint, double &dirX, double &dirY )
  {
    if ( ring.size() < 2 )
      return false;

    const QgsPointXY query( boundaryPoint.x(), boundaryPoint.y() );
    double bestDistance = std::numeric_limits<double>::max();
    int bestIndex = -1;
    for ( int i = 0; i < ring.size(); ++i )
    {
      const QgsPointXY &a = ring[i];
      const QgsPointXY &b = ring[( i + 1 ) % ring.size()];
      const double distance = pointSegmentDistance2( query, a, b );
      if ( distance < bestDistance )
      {
        bestDistance = distance;
        bestIndex = i;
      }
    }

    if ( bestIndex < 0 )
      return false;

    const QgsPointXY &a = ring[bestIndex];
    const QgsPointXY &b = ring[( bestIndex + 1 ) % ring.size()];
    dirX = b.x() - a.x();
    dirY = b.y() - a.y();
    const double length = std::hypot( dirX, dirY );
    if ( length <= 1e-12 )
      return false;

    dirX /= length;
    dirY /= length;
    return true;
  }

  QVector<QgsPointXY> ringWithRidgeIntersections( const QVector<QgsPointXY> &ring, const QgsPoint &ridgePoint, double normalX, double normalY )
  {
    QVector<QgsPointXY> result;
    constexpr double epsilon = 1e-8;

    for ( int i = 0; i < ring.size(); ++i )
    {
      const QgsPointXY &a = ring[i];
      const QgsPointXY &b = ring[( i + 1 ) % ring.size()];
      const double da = signedDistanceToLine( a, ridgePoint, normalX, normalY );
      const double db = signedDistanceToLine( b, ridgePoint, normalX, normalY );

      appendUniquePoint( result, a );
      if ( da * db < -epsilon )
      {
        const double t = da / ( da - db );
        appendUniquePoint( result, QgsPointXY( a.x() + t * ( b.x() - a.x() ), a.y() + t * ( b.y() - a.y() ) ) );
      }
    }

    if ( result.size() > 1 && nearlySamePoint( result.first(), result.last() ) )
      result.removeLast();
    return result;
  }

  QVector<QgsPointXY> clipRingByRidgeSide( const QVector<QgsPointXY> &ring, const QgsPoint &ridgePoint, double normalX, double normalY, bool keepPositive )
  {
    QVector<QgsPointXY> result;
    if ( ring.isEmpty() )
      return result;

    constexpr double epsilon = 1e-8;
    auto inside = [&]( const QgsPointXY &point ) {
      const double distance = signedDistanceToLine( point, ridgePoint, normalX, normalY );
      return keepPositive ? distance >= -epsilon : distance <= epsilon;
    };

    for ( int i = 0; i < ring.size(); ++i )
    {
      const QgsPointXY &current = ring[i];
      const QgsPointXY &previous = ring[( i - 1 + ring.size() ) % ring.size()];
      const bool currentInside = inside( current );
      const bool previousInside = inside( previous );

      if ( currentInside != previousInside )
      {
        const double dPrev = signedDistanceToLine( previous, ridgePoint, normalX, normalY );
        const double dCurr = signedDistanceToLine( current, ridgePoint, normalX, normalY );
        const double t = dPrev / ( dPrev - dCurr );
        appendUniquePoint( result, QgsPointXY( previous.x() + t * ( current.x() - previous.x() ), previous.y() + t * ( current.y() - previous.y() ) ) );
      }

      if ( currentInside )
        appendUniquePoint( result, current );
    }

    if ( result.size() > 1 && nearlySamePoint( result.first(), result.last() ) )
      result.removeLast();
    return result;
  }

  double gabledTopZ( const QgsPointXY &point, const QgsPoint &boundaryPoint, const QgsPoint &ridgePoint, double normalX, double normalY, double sameSideLimit, double oppositeLimit, double baseHeight )
  {
    const double roofRise = ridgePoint.z() - boundaryPoint.z();
    const double ridgeHeight = baseHeight + roofRise;
    const double distance = signedDistanceToLine( point, ridgePoint, normalX, normalY );
    const double boundaryDistance = signedDistanceToLine( QgsPointXY( boundaryPoint.x(), boundaryPoint.y() ), ridgePoint, normalX, normalY );

    if ( std::fabs( distance ) <= 1e-8 )
      return ridgeHeight;

    double limit = sameSideLimit;
    if ( distance * boundaryDistance < 0.0 )
      limit = oppositeLimit > 1e-8 ? oppositeLimit : sameSideLimit;

    if ( limit <= 1e-8 )
      return baseHeight;

    const double ratio = std::min( 1.0, std::fabs( distance ) / limit );
    return std::max( baseHeight, ridgeHeight - roofRise * ratio );
  }

  void appendTriangulatedRoofSurface( BuildingRoof::Mesh &mesh, const QVector<QgsPointXY> &polygon, const QgsPoint &boundaryPoint, const QgsPoint &ridgePoint, double normalX, double normalY, double sameSideLimit, double oppositeLimit, double baseHeight )
  {
    if ( polygon.size() < 3 )
      return;

    QVector<QgsPointXY> localRing = polygon;
    double area = 0.0;
    for ( int i = 0; i < localRing.size(); ++i )
    {
      const QgsPointXY &a = localRing[i];
      const QgsPointXY &b = localRing[( i + 1 ) % localRing.size()];
      area += a.x() * b.y() - b.x() * a.y();
    }
    if ( area < 0.0 )
      std::reverse( localRing.begin(), localRing.end() );

    const int vertexOffset = mesh.vertices.size();
    for ( const QgsPointXY &point : localRing )
      mesh.vertices.append( QgsPoint( point.x(), point.y(), gabledTopZ( point, boundaryPoint, ridgePoint, normalX, normalY, sameSideLimit, oppositeLimit, baseHeight ) ) );

    const QVector<int> triangles = triangulateRing( localRing );
    for ( int i = 0; i + 2 < triangles.size(); i += 3 )
      mesh.indices << vertexOffset + triangles[i] << vertexOffset + triangles[i + 1] << vertexOffset + triangles[i + 2];
  }

  void appendAnchor( QVector<AnchorPoint> &anchors, const QgsPointXY &point, double z )
  {
    for ( AnchorPoint &anchor : anchors )
    {
      if ( nearlySamePoint( anchor.point, point ) )
      {
        anchor.z = z;
        return;
      }
    }
    anchors.append( AnchorPoint{ point, z } );
  }

  double nearestAnchorZ( const QVector<AnchorPoint> &anchors, const QgsPointXY &point, double fallbackZ )
  {
    double bestDistance = std::numeric_limits<double>::max();
    double bestZ = fallbackZ;
    for ( const AnchorPoint &anchor : anchors )
    {
      const double dx = point.x() - anchor.point.x();
      const double dy = point.y() - anchor.point.y();
      const double distance = dx * dx + dy * dy;
      if ( distance < bestDistance )
      {
        bestDistance = distance;
        bestZ = anchor.z;
      }
    }
    return bestZ;
  }

  bool pointInRing( const QVector<QgsPointXY> &ring, const QgsPointXY &point )
  {
    if ( ring.size() < 3 )
      return false;

    bool inside = false;
    for ( int i = 0, j = ring.size() - 1; i < ring.size(); j = i++ )
    {
      const QgsPointXY &a = ring[i];
      const QgsPointXY &b = ring[j];
      if ( pointSegmentDistance2( point, a, b ) <= 1e-12 )
        return true;

      const bool intersects = ( ( a.y() > point.y() ) != ( b.y() > point.y() ) )
                              && ( point.x() < ( b.x() - a.x() ) * ( point.y() - a.y() ) / ( b.y() - a.y() + 1e-30 ) + a.x() );
      if ( intersects )
        inside = !inside;
    }
    return inside;
  }

  double cross2d( double ax, double ay, double bx, double by )
  {
    return ax * by - ay * bx;
  }

  double lineParameter( const QgsPointXY &origin, double dirX, double dirY, const QgsPointXY &point )
  {
    return ( point.x() - origin.x() ) * dirX + ( point.y() - origin.y() ) * dirY;
  }

  QgsPointXY pointOnLine( const QgsPointXY &origin, double dirX, double dirY, double t )
  {
    return QgsPointXY( origin.x() + t * dirX, origin.y() + t * dirY );
  }

  void appendUniqueValue( QVector<double> &values, double value )
  {
    for ( double existing : values )
    {
      if ( std::fabs( existing - value ) <= 1e-7 )
        return;
    }
    values.append( value );
  }

  bool pointOnSegment2d( const QgsPointXY &point, const QgsPointXY &a, const QgsPointXY &b, double *segmentT = nullptr )
  {
    const double dx = b.x() - a.x();
    const double dy = b.y() - a.y();
    const double len2 = dx * dx + dy * dy;
    if ( len2 <= 1e-12 )
      return nearlySamePoint( point, a );

    const double t = ( ( point.x() - a.x() ) * dx + ( point.y() - a.y() ) * dy ) / len2;
    if ( t < -1e-8 || t > 1.0 + 1e-8 )
      return false;

    const QgsPointXY projected( a.x() + t * dx, a.y() + t * dy );
    if ( !nearlySamePoint( point, projected ) )
      return false;

    if ( segmentT )
      *segmentT = std::max( 0.0, std::min( 1.0, t ) );
    return true;
  }

  QVector<QgsPointXY> ringWithInsertedBoundaryPoints( const QVector<QgsPointXY> &ring, const QVector<QgsPointXY> &points )
  {
    QVector<QgsPointXY> result;
    if ( ring.isEmpty() )
      return result;

    for ( int i = 0; i < ring.size(); ++i )
    {
      const QgsPointXY &a = ring[i];
      const QgsPointXY &b = ring[( i + 1 ) % ring.size()];
      appendUniquePoint( result, a );

      QVector<QPair<double, QgsPointXY>> inserts;
      for ( const QgsPointXY &point : points )
      {
        double t = 0.0;
        if ( pointOnSegment2d( point, a, b, &t ) && t > 1e-8 && t < 1.0 - 1e-8 )
          inserts.append( qMakePair( t, point ) );
      }

      std::sort( inserts.begin(), inserts.end(), []( const QPair<double, QgsPointXY> &lhs, const QPair<double, QgsPointXY> &rhs ) {
        return lhs.first < rhs.first;
      } );
      for ( const QPair<double, QgsPointXY> &insert : inserts )
        appendUniquePoint( result, insert.second );
    }

    if ( result.size() > 1 && nearlySamePoint( result.first(), result.last() ) )
      result.removeLast();
    return result;
  }

  void appendVerticalWallWithoutSharedEdges( BuildingRoof::Mesh &mesh, const QgsPointXY &a, const QgsPointXY &b,
                                              const QVector<QVector<QgsPointXY>> &boundaries,
                                              double lowerZ, double upperZ, double tolerance )
  {
    const double length = std::hypot( b.x() - a.x(), b.y() - a.y() );
    if ( length <= tolerance || upperZ - lowerZ <= 1e-8 )
      return;
    const double ux = ( b.x() - a.x() ) / length;
    const double uy = ( b.y() - a.y() ) / length;
    QVector<LineInterval> shared;
    for ( const QVector<QgsPointXY> &boundary : boundaries )
      for ( int edge = 0; edge < boundary.size(); ++edge )
      {
        const QgsPointXY &c = boundary.at( edge );
        const QgsPointXY &d = boundary.at( ( edge + 1 ) % boundary.size() );
        // Only a collinear overlap removes a wall; a crossing or a shared corner does not.
        if ( std::fabs( cross2d( ux, uy, c.x() - a.x(), c.y() - a.y() ) ) > tolerance
             || std::fabs( cross2d( ux, uy, d.x() - a.x(), d.y() - a.y() ) ) > tolerance )
          continue;
        const double first = ( c.x() - a.x() ) * ux + ( c.y() - a.y() ) * uy;
        const double second = ( d.x() - a.x() ) * ux + ( d.y() - a.y() ) * uy;
        const double start = std::max( 0.0, std::min( first, second ) );
        const double end = std::min( length, std::max( first, second ) );
        if ( end - start > tolerance )
          shared.append( LineInterval{ start, end } );
      }
    std::sort( shared.begin(), shared.end(), []( const LineInterval &lhs, const LineInterval &rhs ) {
      return lhs.start < rhs.start;
    } );
    auto appendInterval = [&]( double start, double end ) {
      if ( end - start > tolerance )
        appendVerticalWallSegment( mesh, QgsPointXY( a.x() + ux * start, a.y() + uy * start ),
                                   QgsPointXY( a.x() + ux * end, a.y() + uy * end ), lowerZ, upperZ );
    };
    double cursor = 0.0;
    for ( const LineInterval &interval : shared )
    {
      appendInterval( cursor, interval.start );
      cursor = std::max( cursor, interval.end );
    }
    appendInterval( cursor, length );
  }

  QVector<LineInterval> lineInsideRingIntervals( const QVector<QgsPointXY> &ring, const QgsPointXY &origin, double dirX, double dirY )
  {
    QVector<double> values;
    for ( int i = 0; i < ring.size(); ++i )
    {
      const QgsPointXY &a = ring[i];
      const QgsPointXY &b = ring[( i + 1 ) % ring.size()];
      const double edgeX = b.x() - a.x();
      const double edgeY = b.y() - a.y();
      const double den = cross2d( dirX, dirY, edgeX, edgeY );
      const double relX = a.x() - origin.x();
      const double relY = a.y() - origin.y();

      if ( std::fabs( den ) <= 1e-12 )
      {
        if ( std::fabs( cross2d( relX, relY, dirX, dirY ) ) <= 1e-8 )
        {
          appendUniqueValue( values, lineParameter( origin, dirX, dirY, a ) );
          appendUniqueValue( values, lineParameter( origin, dirX, dirY, b ) );
        }
        continue;
      }

      const double t = cross2d( relX, relY, edgeX, edgeY ) / den;
      const double u = cross2d( relX, relY, dirX, dirY ) / den;
      if ( u >= -1e-8 && u <= 1.0 + 1e-8 )
        appendUniqueValue( values, t );
    }

    std::sort( values.begin(), values.end() );
    QVector<LineInterval> intervals;
    for ( int i = 0; i + 1 < values.size(); ++i )
    {
      const double a = values.at( i );
      const double b = values.at( i + 1 );
      if ( b - a <= 1e-7 )
        continue;

      const QgsPointXY mid = pointOnLine( origin, dirX, dirY, 0.5 * ( a + b ) );
      if ( pointInRing( ring, mid ) )
        intervals.append( LineInterval{ a, b } );
    }
    return intervals;
  }

  bool intervalContains( const LineInterval &interval, double value )
  {
    return value >= interval.start - 1e-7 && value <= interval.end + 1e-7;
  }

  bool findIntervalContaining( const QVector<LineInterval> &intervals, double first, double second, LineInterval &interval )
  {
    for ( const LineInterval &candidate : intervals )
    {
      if ( intervalContains( candidate, first ) && intervalContains( candidate, second ) )
      {
        interval = candidate;
        return true;
      }
    }
    return false;
  }

  bool isConcaveVertex( const QVector<QgsPointXY> &ring, int index )
  {
    const QgsPointXY &previous = ring[( index - 1 + ring.size() ) % ring.size()];
    const QgsPointXY &current = ring[index];
    const QgsPointXY &next = ring[( index + 1 ) % ring.size()];
    const double ax = current.x() - previous.x();
    const double ay = current.y() - previous.y();
    const double bx = next.x() - current.x();
    const double by = next.y() - current.y();
    return cross2d( ax, ay, bx, by ) < -1e-8;
  }

  bool adjacentRingIndices( int a, int b, int count )
  {
    return a == b || ( a + 1 ) % count == b || ( b + 1 ) % count == a;
  }

  bool segmentInsideRing( const QVector<QgsPointXY> &ring, const QgsPointXY &a, const QgsPointXY &b )
  {
    for ( double t : { 0.25, 0.5, 0.75 } )
    {
      const QgsPointXY sample( a.x() + t * ( b.x() - a.x() ), a.y() + t * ( b.y() - a.y() ) );
      if ( !pointInRing( ring, sample ) )
        return false;
    }
    return true;
  }

  double ringExtentSize( const QVector<QgsPointXY> &ring )
  {
    if ( ring.isEmpty() )
      return 1.0;

    double minX = ring.first().x();
    double maxX = ring.first().x();
    double minY = ring.first().y();
    double maxY = ring.first().y();
    for ( const QgsPointXY &point : ring )
    {
      minX = std::min( minX, point.x() );
      maxX = std::max( maxX, point.x() );
      minY = std::min( minY, point.y() );
      maxY = std::max( maxY, point.y() );
    }
    return std::max( 1.0, std::hypot( maxX - minX, maxY - minY ) );
  }

  struct AutoGabledRidge
  {
    bool success = false;
    QString error;
    QgsPoint boundaryPoint;
    QgsPoint ridgePoint;
    double dirX = 0.0;
    double dirY = 0.0;
  };

  struct AutoHippedRidge
  {
    bool success = false;
    QString error;
    QgsPoint firstPoint;
    QgsPoint secondPoint;
    double dirX = 0.0;
    double dirY = 0.0;
    bool preferredGabledSeed = false;
  };

  struct HippedRidgeBandSample
  {
    double t = 0.0;
    QgsPointXY point;
    double z = 0.0;
  };

  struct HippedNormalRoofSample
  {
    QgsPoint point;
    double horizontalNormalX = 0.0;
    double horizontalNormalY = 0.0;
    double normalZ = 0.0;
  };

  struct HippedConstrainedRidgeFit
  {
    bool success = false;
    double lineOffset = 0.0;
    double startT = 0.0;
    double endT = 0.0;
    double ridgeHeight = 0.0;
    double angleOffset = 0.0;
    double score = std::numeric_limits<double>::max();
  };

  struct AutoHeightBand
  {
    bool success = false;
    QString error;
    double baseHeight = 0.0;
    double ridgeHeight = 0.0;
    double lowHeight = 0.0;
    double highHeight = 0.0;
    double binWidth = 0.0;
    QVector<double> filteredHeights;
  };

  double sortedPercentile( const QVector<double> &values, double percentile )
  {
    if ( values.isEmpty() )
      return 0.0;

    const double clamped = std::max( 0.0, std::min( 1.0, percentile ) );
    const int index = std::max( 0, std::min( values.size() - 1, static_cast<int>( std::round( clamped * ( values.size() - 1 ) ) ) ) );
    return values.at( index );
  }

  bool hasSupportedBinRun( const QVector<int> &smoothedCounts, int index, int minSupport )
  {
    if ( index < 0 || index >= smoothedCounts.size() || smoothedCounts.at( index ) < minSupport )
      return false;

    const bool previousSupported = index > 0 && smoothedCounts.at( index - 1 ) >= minSupport;
    const bool nextSupported = index + 1 < smoothedCounts.size() && smoothedCounts.at( index + 1 ) >= minSupport;
    return previousSupported || nextSupported || smoothedCounts.at( index ) >= minSupport * 2;
  }

  AutoHeightBand inferAutoGabledHeightBand( const QVector<QgsPointXY> &ring, const QVector<BuildingRoof::RoofSample> &pointCloudSamples )
  {
    AutoHeightBand band;
    QVector<double> heights;
    heights.reserve( pointCloudSamples.size() );
    for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
    {
      const QgsPoint &point = sample.point;
      if ( pointInRing( ring, QgsPointXY( point.x(), point.y() ) ) )
        heights.append( point.z() );
    }

    if ( heights.size() < 30 )
    {
      band.error = QStringLiteral( "Not enough point-cloud samples to infer the gabled roof ridge." );
      return band;
    }

    std::sort( heights.begin(), heights.end() );
    const double coarseLow = sortedPercentile( heights, 0.02 );
    const double coarseHigh = sortedPercentile( heights, 0.995 );
    QVector<double> clippedHeights;
    clippedHeights.reserve( heights.size() );
    for ( double height : heights )
    {
      if ( height >= coarseLow && height <= coarseHigh )
        clippedHeights.append( height );
    }

    if ( clippedHeights.size() < 20 )
    {
      band.error = QStringLiteral( "Not enough filtered point-cloud samples to infer the gabled roof ridge." );
      return band;
    }

    std::sort( clippedHeights.begin(), clippedHeights.end() );
    const double minZ = clippedHeights.first();
    const double maxZ = clippedHeights.last();
    const double range = maxZ - minZ;
    if ( range <= 1e-6 )
    {
      band.error = QStringLiteral( "The point-cloud height range is too small for a gabled roof." );
      return band;
    }

    band.binWidth = std::max( 0.15, std::min( 0.50, range / 48.0 ) );
    const int binCount = std::max( 3, static_cast<int>( std::ceil( range / band.binWidth ) ) + 1 );
    QVector<int> counts( binCount, 0 );
    for ( double height : clippedHeights )
    {
      const int index = std::max( 0, std::min( binCount - 1, static_cast<int>( std::floor( ( height - minZ ) / band.binWidth ) ) ) );
      ++counts[index];
    }

    QVector<int> smoothedCounts( binCount, 0 );
    for ( int i = 0; i < binCount; ++i )
    {
      smoothedCounts[i] = counts.at( i );
      if ( i > 0 )
        smoothedCounts[i] += counts.at( i - 1 );
      if ( i + 1 < binCount )
        smoothedCounts[i] += counts.at( i + 1 );
    }

    const int minSupport = std::max( 5, static_cast<int>( std::ceil( clippedHeights.size() * 0.003 ) ) );
    int firstSupported = -1;
    int lastSupported = -1;
    for ( int i = 0; i < binCount; ++i )
    {
      if ( hasSupportedBinRun( smoothedCounts, i, minSupport ) )
      {
        firstSupported = i;
        break;
      }
    }
    for ( int i = binCount - 1; i >= 0; --i )
    {
      if ( hasSupportedBinRun( smoothedCounts, i, minSupport ) )
      {
        lastSupported = i;
        break;
      }
    }

    if ( firstSupported < 0 || lastSupported < firstSupported )
    {
      firstSupported = 0;
      lastSupported = binCount - 1;
    }

    band.lowHeight = minZ + firstSupported * band.binWidth;
    band.highHeight = minZ + ( lastSupported + 1 ) * band.binWidth;
    for ( double height : clippedHeights )
    {
      if ( height >= band.lowHeight && height <= band.highHeight )
        band.filteredHeights.append( height );
    }
    if ( band.filteredHeights.size() < 20 )
      band.filteredHeights = clippedHeights;
    std::sort( band.filteredHeights.begin(), band.filteredHeights.end() );

    const double lowPeakLimit = sortedPercentile( band.filteredHeights, 0.45 );
    const double lowPeakFloor = sortedPercentile( band.filteredHeights, 0.05 );
    int bestBaseBin = -1;
    int bestBaseScore = -1;
    for ( int i = firstSupported; i <= lastSupported; ++i )
    {
      const double center = minZ + ( i + 0.5 ) * band.binWidth;
      if ( center < lowPeakFloor || center > lowPeakLimit )
        continue;
      if ( smoothedCounts.at( i ) > bestBaseScore )
      {
        bestBaseScore = smoothedCounts.at( i );
        bestBaseBin = i;
      }
    }
    band.baseHeight = bestBaseBin >= 0 ? minZ + ( bestBaseBin + 0.5 ) * band.binWidth : sortedPercentile( band.filteredHeights, 0.15 );

    const double ridgeSearchFloor = sortedPercentile( band.filteredHeights, 0.70 );
    int ridgeBin = -1;
    for ( int i = lastSupported; i >= firstSupported; --i )
    {
      const double center = minZ + ( i + 0.5 ) * band.binWidth;
      if ( center < ridgeSearchFloor )
        break;
      if ( smoothedCounts.at( i ) >= minSupport )
      {
        ridgeBin = i;
        break;
      }
    }

    if ( ridgeBin >= 0 )
    {
      int ridgeStart = ridgeBin;
      while ( ridgeStart > firstSupported )
      {
        const double center = minZ + ( ridgeStart - 0.5 ) * band.binWidth;
        if ( center < ridgeSearchFloor || smoothedCounts.at( ridgeStart - 1 ) < std::max( 2, minSupport / 2 ) )
          break;
        --ridgeStart;
      }

      int ridgeEnd = ridgeBin;
      while ( ridgeEnd + 1 <= lastSupported && smoothedCounts.at( ridgeEnd + 1 ) >= std::max( 2, minSupport / 2 ) )
        ++ridgeEnd;

      double ridgeSum = 0.0;
      int ridgeCount = 0;
      const double ridgeLow = minZ + ridgeStart * band.binWidth;
      const double ridgeHigh = minZ + ( ridgeEnd + 1 ) * band.binWidth;
      for ( double height : band.filteredHeights )
      {
        if ( height >= ridgeLow && height <= ridgeHigh )
        {
          ridgeSum += height;
          ++ridgeCount;
        }
      }
      band.ridgeHeight = ridgeCount > 0 ? ridgeSum / ridgeCount : minZ + ( ridgeBin + 0.5 ) * band.binWidth;
    }
    else
    {
      band.ridgeHeight = sortedPercentile( band.filteredHeights, 0.92 );
    }

    if ( band.ridgeHeight <= band.baseHeight + std::max( 0.15, band.binWidth ) )
      band.ridgeHeight = sortedPercentile( band.filteredHeights, 0.90 );
    if ( band.ridgeHeight <= band.baseHeight + 1e-6 )
    {
      band.error = QStringLiteral( "Cannot separate gabled roof eave and ridge heights from point-cloud samples." );
      return band;
    }

    band.success = true;
    return band;
  }

  bool fitAutoGabledRidgeCandidate( const QVector<QgsPointXY> &ring, const QVector<QgsPoint> &highPoints, double baseHeight, double ridgeHeight, AutoGabledRidge &candidate, double &score )
  {
    if ( highPoints.size() < 8 )
      return false;

    double meanX = 0.0;
    double meanY = 0.0;
    for ( const QgsPoint &point : highPoints )
    {
      meanX += point.x();
      meanY += point.y();
    }
    meanX /= highPoints.size();
    meanY /= highPoints.size();

    double covXX = 0.0;
    double covXY = 0.0;
    double covYY = 0.0;
    for ( const QgsPoint &point : highPoints )
    {
      const double dx = point.x() - meanX;
      const double dy = point.y() - meanY;
      covXX += dx * dx;
      covXY += dx * dy;
      covYY += dy * dy;
    }
    covXX /= highPoints.size();
    covXY /= highPoints.size();
    covYY /= highPoints.size();

    const double trace = covXX + covYY;
    const double determinant = covXX * covYY - covXY * covXY;
    const double discriminant = std::max( 0.0, trace * trace * 0.25 - determinant );
    const double lambda1 = trace * 0.5 + std::sqrt( discriminant );
    const double lambda2 = trace * 0.5 - std::sqrt( discriminant );
    if ( lambda1 <= 1e-10 )
      return false;

    const double linearity = ( lambda1 - std::max( 0.0, lambda2 ) ) / lambda1;
    if ( linearity < 0.35 )
      return false;

    double dirX = covXY;
    double dirY = lambda1 - covXX;
    if ( std::hypot( dirX, dirY ) <= 1e-10 )
    {
      dirX = lambda1 - covYY;
      dirY = covXY;
    }

    const double dirLength = std::hypot( dirX, dirY );
    if ( dirLength <= 1e-10 )
      return false;
    dirX /= dirLength;
    dirY /= dirLength;

    QVector<double> parameters;
    parameters.reserve( highPoints.size() );
    const QgsPointXY roughOrigin( meanX, meanY );
    for ( const QgsPoint &point : highPoints )
      parameters.append( lineParameter( roughOrigin, dirX, dirY, QgsPointXY( point.x(), point.y() ) ) );
    std::sort( parameters.begin(), parameters.end() );

    const double ridgeLength = sortedPercentile( parameters, 0.95 ) - sortedPercentile( parameters, 0.05 );
    const double extent = ringExtentSize( ring );
    if ( ridgeLength < std::max( extent * 0.12, 0.8 ) )
      return false;

    const QVector<LineInterval> intervals = lineInsideRingIntervals( ring, roughOrigin, dirX, dirY );
    if ( intervals.isEmpty() )
      return false;

    LineInterval bestInterval = intervals.first();
    for ( const LineInterval &interval : intervals )
    {
      if ( interval.end - interval.start > bestInterval.end - bestInterval.start )
        bestInterval = interval;
    }

    const QgsPointXY ridgeXY = pointOnLine( roughOrigin, dirX, dirY, ( bestInterval.start + bestInterval.end ) * 0.5 );
    const double normalX = -dirY;
    const double normalY = dirX;
    const double ridgeS = ridgeXY.x() * normalX + ridgeXY.y() * normalY;
    double maxPositive = 0.0;
    double maxNegative = 0.0;
    QgsPointXY bestBoundaryXY = ring.first();
    double bestBoundaryScore = -1.0;
    for ( const QgsPointXY &point : ring )
    {
      const double distance = point.x() * normalX + point.y() * normalY - ridgeS;
      if ( distance >= 0.0 )
        maxPositive = std::max( maxPositive, distance );
      else
        maxNegative = std::max( maxNegative, -distance );
    }

    for ( int i = 0; i < ring.size(); ++i )
    {
      const QgsPointXY &a = ring.at( i );
      const QgsPointXY &b = ring.at( ( i + 1 ) % ring.size() );
      const double edgeX = b.x() - a.x();
      const double edgeY = b.y() - a.y();
      const double edgeLength = std::hypot( edgeX, edgeY );
      if ( edgeLength <= 1e-8 )
        continue;

      const double edgeDirX = edgeX / edgeLength;
      const double edgeDirY = edgeY / edgeLength;
      const double parallel = std::fabs( edgeDirX * dirX + edgeDirY * dirY );
      if ( parallel < 0.75 )
        continue;

      const QgsPointXY midPoint( ( a.x() + b.x() ) * 0.5, ( a.y() + b.y() ) * 0.5 );
      const double distance = std::fabs( midPoint.x() * normalX + midPoint.y() * normalY - ridgeS );
      const double boundaryScore = distance * ( 0.50 + parallel ) + edgeLength * 0.05;
      if ( boundaryScore > bestBoundaryScore )
      {
        bestBoundaryScore = boundaryScore;
        bestBoundaryXY = midPoint;
      }
    }

    if ( bestBoundaryScore < 0.0 )
    {
      double bestBoundaryDistance = 0.0;
      for ( const QgsPointXY &point : ring )
      {
        const double distance = point.x() * normalX + point.y() * normalY - ridgeS;
        if ( std::fabs( distance ) > std::fabs( bestBoundaryDistance ) )
        {
          bestBoundaryDistance = distance;
          bestBoundaryXY = point;
        }
      }
    }

    const double totalWidth = maxPositive + maxNegative;
    const double minWidth = std::min( maxPositive, maxNegative );
    if ( totalWidth <= 1e-8 || minWidth < std::max( 0.35, totalWidth * 0.08 ) )
      return false;

    const double sideBalance = minWidth / std::max( maxPositive, maxNegative );
    score = linearity * highPoints.size() * std::min( 3.0, ridgeLength / extent ) * std::max( 0.2, sideBalance );

    candidate.success = true;
    candidate.boundaryPoint = QgsPoint( bestBoundaryXY.x(), bestBoundaryXY.y(), baseHeight );
    candidate.ridgePoint = QgsPoint( ridgeXY.x(), ridgeXY.y(), ridgeHeight );
    candidate.dirX = dirX;
    candidate.dirY = dirY;
    return true;
  }

  AutoGabledRidge inferAutoGabledRidge( const QVector<QgsPointXY> &ring, const QVector<BuildingRoof::RoofSample> &pointCloudSamples )
  {
    AutoGabledRidge best;
    const AutoHeightBand heightBand = inferAutoGabledHeightBand( ring, pointCloudSamples );
    if ( !heightBand.success )
    {
      best.error = heightBand.error;
      return best;
    }

    QVector<BuildingRoof::RoofSample> filteredSamples;
    filteredSamples.reserve( pointCloudSamples.size() );
    for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
    {
      const QgsPoint &point = sample.point;
      if ( point.z() < heightBand.lowHeight || point.z() > heightBand.highHeight )
        continue;
      if ( !pointInRing( ring, QgsPointXY( point.x(), point.y() ) ) )
        continue;
      filteredSamples.append( sample );
    }

    if ( filteredSamples.size() < 20 )
    {
      best.error = QStringLiteral( "Not enough filtered point-cloud samples to infer the gabled roof ridge." );
      return best;
    }

    QVector<double> percentiles;
    percentiles << 0.72 << 0.78 << 0.84 << 0.90;
    const double localHighBand = std::max( heightBand.binWidth * 2.0, 0.35 );
    double bestScore = -1.0;

    for ( double percentile : percentiles )
    {
      const double threshold = sortedPercentile( heightBand.filteredHeights, percentile );
      QVector<QgsPoint> highPoints;
      for ( const BuildingRoof::RoofSample &sample : filteredSamples )
      {
        if ( sample.point.z() >= threshold )
          highPoints.append( sample.point );
      }

      AutoGabledRidge candidate;
      double candidateScore = 0.0;
      if ( fitAutoGabledRidgeCandidate( ring, highPoints, heightBand.baseHeight, heightBand.ridgeHeight, candidate, candidateScore )
           && candidateScore > bestScore )
      {
        bestScore = candidateScore;
        best = candidate;
      }
    }

    QVector<QgsPoint> ridgeBandPoints;
    for ( const BuildingRoof::RoofSample &sample : filteredSamples )
    {
      if ( std::fabs( sample.point.z() - heightBand.ridgeHeight ) <= localHighBand )
        ridgeBandPoints.append( sample.point );
    }

    AutoGabledRidge candidate;
    double candidateScore = 0.0;
    if ( fitAutoGabledRidgeCandidate( ring, ridgeBandPoints, heightBand.baseHeight, heightBand.ridgeHeight, candidate, candidateScore )
         && candidateScore > bestScore )
    {
      bestScore = candidateScore;
      best = candidate;
    }

    if ( !best.success )
      best.error = QStringLiteral( "Cannot find a reliable automatic gabled roof ridge line from filtered point-cloud heights." );
    return best;
  }

  QVector<double> verticalWallLikeHeightsNearEdge( const QVector<QgsPoint> &edgePoints, double edgeDirX, double edgeDirY, double edgeNormalX, double edgeNormalY, double extent )
  {
    QVector<double> wallHeights;
    if ( edgePoints.size() < 8 )
      return wallHeights;

    const double neighborRadius = std::min( 1.00, std::max( 0.45, extent * 0.018 ) );
    const double neighborRadius2 = neighborRadius * neighborRadius;
    const int minNeighbors = edgePoints.size() < 18 ? 5 : 7;

    for ( int i = 0; i < edgePoints.size(); ++i )
    {
      const QgsPoint &center = edgePoints.at( i );
      double meanU = 0.0;
      double meanV = 0.0;
      double meanZ = 0.0;
      int count = 0;

      for ( const QgsPoint &neighbor : edgePoints )
      {
        const double dx = neighbor.x() - center.x();
        const double dy = neighbor.y() - center.y();
        if ( dx * dx + dy * dy > neighborRadius2 )
          continue;

        meanU += neighbor.x() * edgeDirX + neighbor.y() * edgeDirY;
        meanV += neighbor.x() * edgeNormalX + neighbor.y() * edgeNormalY;
        meanZ += neighbor.z();
        ++count;
      }

      if ( count < minNeighbors )
        continue;

      meanU /= count;
      meanV /= count;
      meanZ /= count;

      double varU = 0.0;
      double varV = 0.0;
      double varZ = 0.0;
      for ( const QgsPoint &neighbor : edgePoints )
      {
        const double dx = neighbor.x() - center.x();
        const double dy = neighbor.y() - center.y();
        if ( dx * dx + dy * dy > neighborRadius2 )
          continue;

        const double du = neighbor.x() * edgeDirX + neighbor.y() * edgeDirY - meanU;
        const double dv = neighbor.x() * edgeNormalX + neighbor.y() * edgeNormalY - meanV;
        const double dz = neighbor.z() - meanZ;
        varU += du * du;
        varV += dv * dv;
        varZ += dz * dz;
      }

      varU /= count;
      varV /= count;
      varZ /= count;

      const double minPlaneVariance = std::min( varU, varZ );
      const bool thinAcrossEdge = varV <= std::max( 0.006, minPlaneVariance * 0.35 );
      const bool verticalSpread = varZ >= std::max( 0.06, varV * 5.0 );
      if ( thinAcrossEdge && verticalSpread )
        wallHeights.append( center.z() );
    }

    std::sort( wallHeights.begin(), wallHeights.end() );
    return wallHeights;
  }

  double upperWeightedMean( const QVector<double> &sortedHeights, double lowerPercentile, double upperPercentile, double fallbackHeight )
  {
    if ( sortedHeights.isEmpty() )
      return fallbackHeight;

    const int last = sortedHeights.size() - 1;
    const int start = std::max( 0, std::min( last, static_cast<int>( std::round( std::max( 0.0, std::min( 1.0, lowerPercentile ) ) * last ) ) ) );
    const int end = std::max( start, std::min( last, static_cast<int>( std::round( std::max( 0.0, std::min( 1.0, upperPercentile ) ) * last ) ) ) );

    double weightedSum = 0.0;
    double weightSum = 0.0;
    const double span = std::max( 1, end - start );
    for ( int i = start; i <= end; ++i )
    {
      const double rank = static_cast<double>( i - start ) / span;
      const double weight = 1.0 + rank * 2.0;
      weightedSum += sortedHeights.at( i ) * weight;
      weightSum += weight;
    }

    return weightSum > 0.0 ? weightedSum / weightSum : fallbackHeight;
  }

  double denseUpperWallHeight( const QVector<double> &sortedWallHeights, double fallbackHeight, double ridgeHeight, double heightRange )
  {
    if ( sortedWallHeights.isEmpty() )
      return fallbackHeight;
    if ( sortedWallHeights.size() < 8 )
      return sortedPercentile( sortedWallHeights, 0.75 );

    const double ridgeBuffer = std::max( 0.25, heightRange * 0.04 );
    const double low = sortedPercentile( sortedWallHeights, 0.05 );
    const double high = std::min( sortedPercentile( sortedWallHeights, 0.98 ), ridgeHeight - ridgeBuffer );
    if ( high <= low + 1e-6 )
      return std::min( upperWeightedMean( sortedWallHeights, 0.65, 0.90, sortedPercentile( sortedWallHeights, 0.75 ) ), ridgeHeight - ridgeBuffer );

    const double binWidth = std::max( 0.20, ( high - low ) / 24.0 );
    const int binCount = std::max( 2, static_cast<int>( std::ceil( ( high - low ) / binWidth ) ) + 1 );
    QVector<int> counts( binCount, 0 );
    int filteredCount = 0;
    for ( double height : sortedWallHeights )
    {
      if ( height < low || height > high )
        continue;
      const int index = std::max( 0, std::min( binCount - 1, static_cast<int>( std::floor( ( height - low ) / binWidth ) ) ) );
      ++counts[index];
      ++filteredCount;
    }

    if ( filteredCount < 5 )
      return std::min( upperWeightedMean( sortedWallHeights, 0.65, 0.90, sortedPercentile( sortedWallHeights, 0.75 ) ), ridgeHeight - ridgeBuffer );

    const int minSupport = std::max( 3, filteredCount / 18 );
    for ( int i = binCount - 1; i >= 0; --i )
    {
      const int support = counts.at( i )
                          + ( i > 0 ? counts.at( i - 1 ) : 0 )
                          + ( i + 1 < binCount ? counts.at( i + 1 ) : 0 );
      if ( support < minSupport )
        continue;

      QVector<double> bandHeights;
      const double bandLow = low + std::max( 0, i - 1 ) * binWidth;
      const double bandHigh = low + ( std::min( binCount - 1, i + 1 ) + 1 ) * binWidth;
      for ( double height : sortedWallHeights )
      {
        if ( height >= bandLow && height <= bandHigh )
          bandHeights.append( height );
      }
      if ( !bandHeights.isEmpty() )
      {
        std::sort( bandHeights.begin(), bandHeights.end() );
        return std::min( upperWeightedMean( bandHeights, 0.60, 0.90, sortedPercentile( bandHeights, 0.70 ) ), ridgeHeight - ridgeBuffer );
      }
    }

    return std::min( upperWeightedMean( sortedWallHeights, 0.65, 0.90, sortedPercentile( sortedWallHeights, 0.75 ) ), ridgeHeight - ridgeBuffer );
  }

  bool automaticBoundaryPointFromRidgeLine( const QVector<QgsPointXY> &ring, const QVector<BuildingRoof::RoofSample> &pointCloudSamples, const QgsPoint &ridgePoint, double dirX, double dirY, const QgsPoint &fallbackBoundaryPoint, QgsPoint &boundaryPoint )
  {
    const double dirLength = std::hypot( dirX, dirY );
    if ( ring.size() < 2 || dirLength <= 1e-10 )
      return false;

    dirX /= dirLength;
    dirY /= dirLength;
    const double normalX = -dirY;
    const double normalY = dirX;
    const double ridgeOffset = ridgePoint.x() * normalX + ridgePoint.y() * normalY;
    const double extent = ringExtentSize( ring );
    const double edgeBandDistance = std::min( 1.00, std::max( 0.45, extent * 0.015 ) );
    const double edgeBandDistance2 = edgeBandDistance * edgeBandDistance;
    const double heightEdgeBandDistance = std::min( edgeBandDistance, std::min( 0.35, std::max( 0.18, extent * 0.006 ) ) );
    const double heightEdgeBandDistance2 = heightEdgeBandDistance * heightEdgeBandDistance;

    QVector<double> allHeights;
    allHeights.reserve( pointCloudSamples.size() );
    for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
    {
      const QgsPoint &point = sample.point;
      if ( pointInRing( ring, QgsPointXY( point.x(), point.y() ) ) )
        allHeights.append( point.z() );
    }

    if ( allHeights.size() < 8 )
      return false;

    std::sort( allHeights.begin(), allHeights.end() );
    const double lowClip = sortedPercentile( allHeights, 0.02 );
    const double highClip = sortedPercentile( allHeights, 0.999 );
    const double globalEaveHeight = sortedPercentile( allHeights, 0.18 );
    const double heightRange = std::max( 0.30, highClip - lowClip );

    double maxRidgeDistance = 0.0;
    double longestParallelEdge = 0.0;
    for ( int i = 0; i < ring.size(); ++i )
    {
      const QgsPointXY &a = ring.at( i );
      const QgsPointXY &b = ring.at( ( i + 1 ) % ring.size() );
      const double edgeX = b.x() - a.x();
      const double edgeY = b.y() - a.y();
      const double edgeLength = std::hypot( edgeX, edgeY );
      if ( edgeLength <= 1e-8 )
        continue;

      const double edgeDirX = edgeX / edgeLength;
      const double edgeDirY = edgeY / edgeLength;
      const double parallel = std::fabs( edgeDirX * dirX + edgeDirY * dirY );
      if ( parallel >= 0.65 )
        longestParallelEdge = std::max( longestParallelEdge, edgeLength );

      const QgsPointXY midPoint( ( a.x() + b.x() ) * 0.5, ( a.y() + b.y() ) * 0.5 );
      const double distance = std::fabs( midPoint.x() * normalX + midPoint.y() * normalY - ridgeOffset );
      maxRidgeDistance = std::max( maxRidgeDistance, distance );
    }

    if ( maxRidgeDistance <= 1e-8 )
      return false;
    if ( longestParallelEdge <= 1e-8 )
      longestParallelEdge = 0.0;

    int bestEdge = -1;
    double bestScore = -1.0;
    double bestBoundaryZ = fallbackBoundaryPoint.z();
    QgsPointXY bestBoundaryXY;
    bool hasBestBoundaryXY = false;
    for ( int i = 0; i < ring.size(); ++i )
    {
      const QgsPointXY &a = ring.at( i );
      const QgsPointXY &b = ring.at( ( i + 1 ) % ring.size() );
      const double edgeX = b.x() - a.x();
      const double edgeY = b.y() - a.y();
      const double edgeLength = std::hypot( edgeX, edgeY );
      if ( edgeLength <= 1e-8 )
        continue;

      const double edgeDirX = edgeX / edgeLength;
      const double edgeDirY = edgeY / edgeLength;
      const double parallel = std::fabs( edgeDirX * dirX + edgeDirY * dirY );
      if ( parallel < 0.65 )
        continue;
      if ( longestParallelEdge > 1e-8 && edgeLength < longestParallelEdge * 0.35 )
        continue;

      const QgsPointXY midPoint( ( a.x() + b.x() ) * 0.5, ( a.y() + b.y() ) * 0.5 );
      const double distance = std::fabs( midPoint.x() * normalX + midPoint.y() * normalY - ridgeOffset );
      if ( distance < std::max( 0.25, maxRidgeDistance * 0.45 ) )
        continue;

      QVector<double> edgeHeights;
      QVector<QgsPoint> edgePoints;
      QVector<double> heightEdgeHeights;
      QVector<QgsPoint> heightEdgePoints;
      edgeHeights.reserve( pointCloudSamples.size() );
      edgePoints.reserve( pointCloudSamples.size() );
      heightEdgeHeights.reserve( pointCloudSamples.size() );
      heightEdgePoints.reserve( pointCloudSamples.size() );
      for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
      {
        const QgsPoint &point = sample.point;
        const QgsPointXY pointXY( point.x(), point.y() );
        if ( !pointInRing( ring, pointXY ) )
          continue;
        const double edgeDistance2 = pointSegmentDistance2( pointXY, a, b );
        if ( edgeDistance2 > edgeBandDistance2 )
          continue;
        if ( point.z() < lowClip || point.z() > highClip )
          continue;
        edgeHeights.append( point.z() );
        edgePoints.append( point );
        if ( edgeDistance2 <= heightEdgeBandDistance2 )
        {
          heightEdgeHeights.append( point.z() );
          heightEdgePoints.append( point );
        }
      }

      double edgeZ = globalEaveHeight;
      double supportScore = 0.10;
      double wallSupportScore = 0.0;
      const double edgeNormalX = -edgeDirY;
      const double edgeNormalY = edgeDirX;
      const QVector<QgsPoint> &boundaryHeightPoints = heightEdgePoints.size() >= 8 ? heightEdgePoints : edgePoints;
      const QVector<double> &boundaryHeightValues = heightEdgeHeights.size() >= 5 ? heightEdgeHeights : edgeHeights;
      const QVector<double> wallHeights = verticalWallLikeHeightsNearEdge( boundaryHeightPoints, edgeDirX, edgeDirY, edgeNormalX, edgeNormalY, extent );
      if ( wallHeights.size() >= 5 )
      {
        edgeZ = denseUpperWallHeight( wallHeights, sortedPercentile( wallHeights, 0.75 ), ridgePoint.z(), heightRange );
        wallSupportScore = std::min( 1.0, static_cast<double>( wallHeights.size() ) / std::max( 6.0, edgeLength / std::max( heightEdgeBandDistance, 1e-8 ) ) );
        supportScore = std::max( supportScore, wallSupportScore );
      }
      if ( boundaryHeightValues.size() >= 5 )
      {
        QVector<double> sortedBoundaryHeights = boundaryHeightValues;
        std::sort( sortedBoundaryHeights.begin(), sortedBoundaryHeights.end() );
        if ( wallHeights.size() < 5 )
          edgeZ = upperWeightedMean( sortedBoundaryHeights, 0.60, 0.85, sortedPercentile( sortedBoundaryHeights, 0.70 ) );
        supportScore = std::min( 1.0, static_cast<double>( edgeHeights.size() ) / std::max( 8.0, edgeLength / std::max( edgeBandDistance, 1e-8 ) * 2.0 ) );
        if ( wallHeights.size() >= 5 )
          supportScore = std::max( supportScore, wallSupportScore );
      }

      QVector<double> supportParameters;
      const double supportHeightTolerance = std::max( 0.25, heightRange * 0.035 );
      supportParameters.reserve( boundaryHeightPoints.size() );
      for ( const QgsPoint &point : boundaryHeightPoints )
      {
        if ( std::fabs( point.z() - edgeZ ) > supportHeightTolerance )
          continue;
        supportParameters.append( pointSegmentParameter( QgsPointXY( point.x(), point.y() ), a, b ) );
      }
      if ( supportParameters.size() < 5 )
      {
        supportParameters.clear();
        for ( const QgsPoint &point : boundaryHeightPoints )
          supportParameters.append( pointSegmentParameter( QgsPointXY( point.x(), point.y() ), a, b ) );
      }

      QgsPointXY candidateBoundaryXY = midPoint;
      if ( supportParameters.size() >= 5 )
      {
        std::sort( supportParameters.begin(), supportParameters.end() );
        const double supportT = sortedPercentile( supportParameters, 0.50 );
        candidateBoundaryXY = QgsPointXY( a.x() + ( b.x() - a.x() ) * supportT, a.y() + ( b.y() - a.y() ) * supportT );
      }

      const double distanceScore = distance / maxRidgeDistance;
      const double lengthScore = longestParallelEdge > 1e-8 ? std::min( 1.0, edgeLength / longestParallelEdge ) : 0.5;
      const double heightGapScore = std::max( 0.0, std::min( 1.0, ( ridgePoint.z() - edgeZ ) / heightRange ) );
      const double score = distanceScore * 2.0 + parallel * 1.2 + lengthScore * 0.8 + supportScore * 1.0 + wallSupportScore * 1.4 + heightGapScore * 0.8;
      if ( score > bestScore )
      {
        bestScore = score;
        bestEdge = i;
        bestBoundaryZ = edgeZ;
        bestBoundaryXY = candidateBoundaryXY;
        hasBestBoundaryXY = true;
      }
    }

    if ( bestEdge < 0 )
    {
      double bestDistance = -1.0;
      for ( int i = 0; i < ring.size(); ++i )
      {
        const QgsPointXY &a = ring.at( i );
        const QgsPointXY &b = ring.at( ( i + 1 ) % ring.size() );
        const QgsPointXY midPoint( ( a.x() + b.x() ) * 0.5, ( a.y() + b.y() ) * 0.5 );
        const double distance = std::fabs( midPoint.x() * normalX + midPoint.y() * normalY - ridgeOffset );
        if ( distance > bestDistance )
        {
          bestDistance = distance;
          bestEdge = i;
          bestBoundaryXY = midPoint;
          hasBestBoundaryXY = true;
        }
      }
      bestBoundaryZ = globalEaveHeight;
    }

    if ( bestEdge < 0 )
      return false;

    const QgsPointXY &edgeStart = ring.at( bestEdge );
    const QgsPointXY &edgeEnd = ring.at( ( bestEdge + 1 ) % ring.size() );
    const QgsPointXY boundaryXY = hasBestBoundaryXY
                                    ? bestBoundaryXY
                                    : QgsPointXY( ( edgeStart.x() + edgeEnd.x() ) * 0.5, ( edgeStart.y() + edgeEnd.y() ) * 0.5 );
    double boundaryZ = bestBoundaryZ;

    if ( boundaryZ >= ridgePoint.z() - 1e-6 )
      boundaryZ = ridgePoint.z() - std::max( 0.30, ringExtentSize( ring ) * 0.01 );

    boundaryPoint = QgsPoint( boundaryXY.x(), boundaryXY.y(), boundaryZ );
    return true;
  }

  struct LeastSquaresRoofPlane
  {
    bool success = false;
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    double rmse = 0.0;
    int count = 0;
  };

  struct GabledBoundarySlopePlanes
  {
    bool success = false;
    LeastSquaresRoofPlane firstPlane;
    LeastSquaresRoofPlane secondPlane;
    bool hasSideSigns = false;
    double firstPlaneSideSign = 0.0;
    double secondPlaneSideSign = 0.0;
  };

  bool solveSymmetric3x3( double matrix[3][4], double &x, double &y, double &z )
  {
    for ( int pivot = 0; pivot < 3; ++pivot )
    {
      int bestRow = pivot;
      double bestAbs = std::fabs( matrix[pivot][pivot] );
      for ( int row = pivot + 1; row < 3; ++row )
      {
        const double value = std::fabs( matrix[row][pivot] );
        if ( value > bestAbs )
        {
          bestAbs = value;
          bestRow = row;
        }
      }

      if ( bestAbs <= 1e-10 )
        return false;

      if ( bestRow != pivot )
      {
        for ( int col = pivot; col < 4; ++col )
          std::swap( matrix[pivot][col], matrix[bestRow][col] );
      }

      const double divisor = matrix[pivot][pivot];
      for ( int col = pivot; col < 4; ++col )
        matrix[pivot][col] /= divisor;

      for ( int row = 0; row < 3; ++row )
      {
        if ( row == pivot )
          continue;
        const double factor = matrix[row][pivot];
        for ( int col = pivot; col < 4; ++col )
          matrix[row][col] -= factor * matrix[pivot][col];
      }
    }

    x = matrix[0][3];
    y = matrix[1][3];
    z = matrix[2][3];
    return std::isfinite( x ) && std::isfinite( y ) && std::isfinite( z );
  }

  double roofPlaneZ( const LeastSquaresRoofPlane &plane, const QgsPointXY &point )
  {
    return plane.a * point.x() + plane.b * point.y() + plane.c;
  }

  bool refineBoundaryPointHeightFromSlopePlane( QgsPoint &boundaryPoint, double slopeHeight, const QgsPoint &ridgePoint, double extent )
  {
    if ( !std::isfinite( slopeHeight ) || slopeHeight <= 0.0 )
      return false;

    const double ridgeBuffer = std::max( 0.25, extent * 0.008 );
    const double maxHeight = ridgePoint.z() - ridgeBuffer;
    if ( slopeHeight >= maxHeight )
      slopeHeight = maxHeight;
    if ( slopeHeight <= 0.0 )
      return false;

    const double oldHeight = boundaryPoint.z();
    double refinedHeight = slopeHeight;
    if ( std::isfinite( oldHeight ) && oldHeight > 0.0 )
    {
      const double rise = std::max( 0.30, ridgePoint.z() - oldHeight );
      const double maxAdjustment = std::max( 0.45, std::min( 1.25, rise * 0.35 ) );
      const double diff = slopeHeight - oldHeight;
      if ( std::fabs( diff ) > maxAdjustment )
        slopeHeight = oldHeight + ( diff > 0.0 ? maxAdjustment : -maxAdjustment );
      refinedHeight = slopeHeight * 0.55 + oldHeight * 0.45;
    }

    if ( refinedHeight >= maxHeight )
      refinedHeight = maxHeight;
    if ( refinedHeight <= 0.0 )
      return false;

    boundaryPoint.setZ( refinedHeight );
    return true;
  }

  LeastSquaresRoofPlane fitLeastSquaresRoofPlane( const QVector<QgsPoint> &points )
  {
    LeastSquaresRoofPlane plane;
    if ( points.size() < 8 )
      return plane;

    double originX = 0.0;
    double originY = 0.0;
    for ( const QgsPoint &point : points )
    {
      originX += point.x();
      originY += point.y();
    }
    originX /= points.size();
    originY /= points.size();

    double sumX = 0.0;
    double sumY = 0.0;
    double sumZ = 0.0;
    double sumXX = 0.0;
    double sumXY = 0.0;
    double sumYY = 0.0;
    double sumXZ = 0.0;
    double sumYZ = 0.0;
    for ( const QgsPoint &point : points )
    {
      const double x = point.x() - originX;
      const double y = point.y() - originY;
      const double z = point.z();
      sumX += x;
      sumY += y;
      sumZ += z;
      sumXX += x * x;
      sumXY += x * y;
      sumYY += y * y;
      sumXZ += x * z;
      sumYZ += y * z;
    }

    double matrix[3][4] = {
      { sumXX, sumXY, sumX, sumXZ },
      { sumXY, sumYY, sumY, sumYZ },
      { sumX, sumY, static_cast<double>( points.size() ), sumZ }
    };

    double localA = 0.0;
    double localB = 0.0;
    double localC = 0.0;
    if ( !solveSymmetric3x3( matrix, localA, localB, localC ) )
      return plane;

    double squaredErrorSum = 0.0;
    for ( const QgsPoint &point : points )
    {
      const double predictedZ = localA * ( point.x() - originX ) + localB * ( point.y() - originY ) + localC;
      const double error = point.z() - predictedZ;
      squaredErrorSum += error * error;
    }

    plane.success = true;
    plane.a = localA;
    plane.b = localB;
    plane.c = localC - localA * originX - localB * originY;
    plane.rmse = std::sqrt( squaredErrorSum / points.size() );
    plane.count = points.size();
    return plane;
  }

  bool planeFromThreePoints( const QgsPoint &p0, const QgsPoint &p1, const QgsPoint &p2, LeastSquaresRoofPlane &plane )
  {
    const double ux = p1.x() - p0.x();
    const double uy = p1.y() - p0.y();
    const double uz = p1.z() - p0.z();
    const double vx = p2.x() - p0.x();
    const double vy = p2.y() - p0.y();
    const double vz = p2.z() - p0.z();

    const double nx = uy * vz - uz * vy;
    const double ny = uz * vx - ux * vz;
    const double nz = ux * vy - uy * vx;
    const double normalLength = std::sqrt( nx * nx + ny * ny + nz * nz );
    if ( normalLength <= 1e-10 )
      return false;

    const double verticalComponent = std::fabs( nz ) / normalLength;
    if ( verticalComponent < 0.20 )
      return false;

    plane.success = true;
    plane.a = -nx / nz;
    plane.b = -ny / nz;
    plane.c = ( nx * p0.x() + ny * p0.y() + nz * p0.z() ) / nz;
    plane.rmse = 0.0;
    plane.count = 3;
    return std::isfinite( plane.a ) && std::isfinite( plane.b ) && std::isfinite( plane.c );
  }

  struct HippedCandidateRoofTriangle
  {
    QgsPointXY a;
    QgsPointXY b;
    QgsPointXY c;
    LeastSquaresRoofPlane plane;
  };

  struct HippedWeightedRoofSample
  {
    QgsPoint point;
    double weight = 1.0;
  };

  struct RoofHeightTriangle
  {
    QgsPoint a;
    QgsPoint b;
    QgsPoint c;
    double denominator = 0.0;
  };

  QVector<RoofHeightTriangle> roofHeightTriangles( const BuildingRoof::Mesh &mesh, bool includeGroundSurface = false )
  {
    QVector<RoofHeightTriangle> triangles;
    for ( int i = 0; i + 2 < mesh.indices.size(); i += 3 )
    {
      const QgsPoint &a = mesh.vertices.at( mesh.indices.at( i ) );
      const QgsPoint &b = mesh.vertices.at( mesh.indices.at( i + 1 ) );
      const QgsPoint &c = mesh.vertices.at( mesh.indices.at( i + 2 ) );
      const double denominator = cross2d( b.x() - a.x(), b.y() - a.y(), c.x() - a.x(), c.y() - a.y() );
      if ( std::fabs( denominator ) <= 1e-10 || ( !includeGroundSurface && std::max( { a.z(), b.z(), c.z() } ) <= 1e-8 ) )
        continue;
      triangles.append( RoofHeightTriangle{ a, b, c, denominator } );
    }
    return triangles;
  }

  double roofHeightAt( const QVector<RoofHeightTriangle> &triangles, const QgsPoint &point )
  {
    double height = -std::numeric_limits<double>::infinity();
    for ( const RoofHeightTriangle &triangle : triangles )
    {
      const double dx = point.x() - triangle.a.x();
      const double dy = point.y() - triangle.a.y();
      const double u = cross2d( dx, dy, triangle.c.x() - triangle.a.x(), triangle.c.y() - triangle.a.y() ) / triangle.denominator;
      const double v = cross2d( triangle.b.x() - triangle.a.x(), triangle.b.y() - triangle.a.y(), dx, dy ) / triangle.denominator;
      if ( u < -1e-7 || v < -1e-7 || u + v > 1.0 + 1e-7 )
        continue;
      height = std::max( height, triangle.a.z() + u * ( triangle.b.z() - triangle.a.z() ) + v * ( triangle.c.z() - triangle.a.z() ) );
    }
    return height;
  }

  bool pointInsideTriangle2d( const QgsPointXY &point, const QgsPointXY &a, const QgsPointXY &b, const QgsPointXY &c )
  {
    const double ab = cross2d( b.x() - a.x(), b.y() - a.y(), point.x() - a.x(), point.y() - a.y() );
    const double bc = cross2d( c.x() - b.x(), c.y() - b.y(), point.x() - b.x(), point.y() - b.y() );
    const double ca = cross2d( a.x() - c.x(), a.y() - c.y(), point.x() - c.x(), point.y() - c.y() );
    const double tolerance = 1e-8 * std::max( 1.0, std::fabs( ab ) + std::fabs( bc ) + std::fabs( ca ) );
    const bool hasNegative = ab < -tolerance || bc < -tolerance || ca < -tolerance;
    const bool hasPositive = ab > tolerance || bc > tolerance || ca > tolerance;
    return !( hasNegative && hasPositive );
  }

  double hippedCandidateAnchorHeight( const QgsPointXY &point, const QgsPointXY &firstRidge, const QgsPointXY &secondRidge, double eaveHeight, double ridgeHeight, double toleranceSquared )
  {
    const double firstDx = point.x() - firstRidge.x();
    const double firstDy = point.y() - firstRidge.y();
    const double secondDx = point.x() - secondRidge.x();
    const double secondDy = point.y() - secondRidge.y();
    if ( firstDx * firstDx + firstDy * firstDy <= toleranceSquared
         || secondDx * secondDx + secondDy * secondDy <= toleranceSquared )
      return ridgeHeight;
    return eaveHeight;
  }

  QVector<HippedCandidateRoofTriangle> hippedCandidateRoofTriangles( const QVector<QgsPointXY> &ring, const QgsPointXY &firstRidge, const QgsPointXY &secondRidge, double eaveHeight, double ridgeHeight )
  {
    QVector<HippedCandidateRoofTriangle> result;
    if ( ring.size() < 3 || !pointInRing( ring, firstRidge ) || !pointInRing( ring, secondRidge ) )
      return result;

    QgsMultiPointXY pointSet;
    for ( const QgsPointXY &point : ring )
      pointSet.append( point );
    pointSet.append( firstRidge );
    pointSet.append( secondRidge );

    const QgsGeometry tin = QgsGeometry::fromMultiPointXY( pointSet ).delaunayTriangulation( 0.0, false );
    QVector<QgsGeometry> geometries = tin.asGeometryCollection();
    if ( geometries.isEmpty() && !tin.isNull() )
      geometries.append( tin );

    const double extent = ringExtentSize( ring );
    const double xyTolerance = std::max( 1e-7, extent * 1e-8 );
    const double toleranceSquared = xyTolerance * xyTolerance;
    result.reserve( geometries.size() );
    for ( const QgsGeometry &geometry : geometries )
    {
      const QgsPolygonXY triangle = geometry.asPolygon();
      if ( triangle.isEmpty() || triangle.first().size() < 4 )
        continue;

      const QgsPolylineXY triangleRing = triangle.first();
      const QgsPointXY a = triangleRing.at( 0 );
      const QgsPointXY b = triangleRing.at( 1 );
      const QgsPointXY c = triangleRing.at( 2 );
      const QgsPointXY centroid( ( a.x() + b.x() + c.x() ) / 3.0,
                                 ( a.y() + b.y() + c.y() ) / 3.0 );
      if ( !pointInRing( ring, centroid ) )
        continue;

      const QgsPoint pointA( a.x(), a.y(), hippedCandidateAnchorHeight( a, firstRidge, secondRidge, eaveHeight, ridgeHeight, toleranceSquared ) );
      const QgsPoint pointB( b.x(), b.y(), hippedCandidateAnchorHeight( b, firstRidge, secondRidge, eaveHeight, ridgeHeight, toleranceSquared ) );
      const QgsPoint pointC( c.x(), c.y(), hippedCandidateAnchorHeight( c, firstRidge, secondRidge, eaveHeight, ridgeHeight, toleranceSquared ) );
      LeastSquaresRoofPlane plane;
      if ( !planeFromThreePoints( pointA, pointB, pointC, plane ) )
        continue;
      result.append( HippedCandidateRoofTriangle{ a, b, c, plane } );
    }
    return result;
  }

  double hippedCandidateRoofLoss( const QVector<QgsPointXY> &ring, const QVector<HippedWeightedRoofSample> &roofPoints, const QgsPointXY &firstRidge, const QgsPointXY &secondRidge, double eaveHeight, double ridgeHeight, double robustDelta )
  {
    const QVector<HippedCandidateRoofTriangle> triangles = hippedCandidateRoofTriangles( ring, firstRidge, secondRidge, eaveHeight, ridgeHeight );
    if ( triangles.isEmpty() )
      return std::numeric_limits<double>::max();

    double loss = 0.0;
    double weightSum = 0.0;
    int count = 0;
    for ( const HippedWeightedRoofSample &sample : roofPoints )
    {
      const QgsPoint &point = sample.point;
      const QgsPointXY pointXY( point.x(), point.y() );
      double bestDistance = std::numeric_limits<double>::max();
      for ( const HippedCandidateRoofTriangle &triangle : triangles )
      {
        if ( !pointInsideTriangle2d( pointXY, triangle.a, triangle.b, triangle.c ) )
          continue;
        const double verticalResidual = point.z() - roofPlaneZ( triangle.plane, pointXY );
        const double normalLength = std::sqrt( triangle.plane.a * triangle.plane.a + triangle.plane.b * triangle.plane.b + 1.0 );
        bestDistance = std::min( bestDistance, std::fabs( verticalResidual ) / normalLength );
      }
      if ( !std::isfinite( bestDistance ) || bestDistance == std::numeric_limits<double>::max() )
        continue;

      const double robustLoss = bestDistance <= robustDelta
                                  ? 0.5 * bestDistance * bestDistance
                                  : robustDelta * ( bestDistance - 0.5 * robustDelta );
      loss += sample.weight * robustLoss;
      weightSum += sample.weight;
      ++count;
    }

    const int minCount = std::max( 20, roofPoints.size() / 2 );
    return count >= minCount && weightSum > 0.0 ? loss / weightSum : std::numeric_limits<double>::max();
  }

  double roofPlaneResidual( const LeastSquaresRoofPlane &plane, const QgsPoint &point )
  {
    return std::fabs( point.z() - ( plane.a * point.x() + plane.b * point.y() + plane.c ) );
  }

  double robustPlaneThreshold( const QVector<QgsPoint> &points )
  {
    if ( points.isEmpty() )
      return 0.25;

    double minZ = std::numeric_limits<double>::max();
    double maxZ = std::numeric_limits<double>::lowest();
    for ( const QgsPoint &point : points )
    {
      minZ = std::min( minZ, point.z() );
      maxZ = std::max( maxZ, point.z() );
    }

    const double heightRange = std::max( 0.0, maxZ - minZ );
    return std::max( 0.15, std::min( 0.45, heightRange * 0.06 ) );
  }

  double pointCloudXySpread( const QVector<QgsPoint> &points )
  {
    if ( points.isEmpty() )
      return 0.0;

    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double minY = std::numeric_limits<double>::max();
    double maxY = std::numeric_limits<double>::lowest();
    for ( const QgsPoint &point : points )
    {
      minX = std::min( minX, point.x() );
      maxX = std::max( maxX, point.x() );
      minY = std::min( minY, point.y() );
      maxY = std::max( maxY, point.y() );
    }
    return std::hypot( maxX - minX, maxY - minY );
  }

  bool roofPlaneHasSpatialSupport( const QVector<QgsPoint> &points, double supportWidth )
  {
    if ( points.size() < 12 )
      return false;
    const QgsPoint &origin = points.first();
    double sumX = 0.0;
    double sumY = 0.0;
    double sumXX = 0.0;
    double sumXY = 0.0;
    double sumYY = 0.0;
    for ( const QgsPoint &point : points )
    {
      const double x = point.x() - origin.x();
      const double y = point.y() - origin.y();
      sumX += x;
      sumY += y;
      sumXX += x * x;
      sumXY += x * y;
      sumYY += y * y;
    }
    const double xx = sumXX / points.size() - std::pow( sumX / points.size(), 2.0 );
    const double xy = sumXY / points.size() - sumX * sumY / ( points.size() * static_cast<double>( points.size() ) );
    const double yy = sumYY / points.size() - std::pow( sumY / points.size(), 2.0 );
    const double discriminant = std::hypot( xx - yy, 2.0 * xy );
    const double major = std::max( 0.0, ( xx + yy + discriminant ) * 0.5 );
    const double minor = std::max( 0.0, ( xx + yy - discriminant ) * 0.5 );
    return major >= std::pow( supportWidth * 0.5, 2.0 ) && minor >= std::pow( supportWidth * 0.1, 2.0 );
  }

  QVector<QgsPoint> robustPlaneInliers( const QVector<QgsPoint> &points, const LeastSquaresRoofPlane &plane, double threshold )
  {
    QVector<QgsPoint> inliers;
    inliers.reserve( points.size() );
    for ( const QgsPoint &point : points )
    {
      if ( roofPlaneResidual( plane, point ) <= threshold )
        inliers.append( point );
    }
    return inliers;
  }

  double chooseBoundarySlopeHeight( const GabledBoundarySlopePlanes &planes, const QgsPoint &boundaryPoint, const QgsPoint &ridgePoint, double extent )
  {
    if ( !planes.success )
      return std::numeric_limits<double>::quiet_NaN();

    const QgsPointXY boundaryXY( boundaryPoint.x(), boundaryPoint.y() );
    const double ridgeBuffer = std::max( 0.25, extent * 0.008 );
    const double maxHeight = ridgePoint.z() - ridgeBuffer;
    const double oldHeight = boundaryPoint.z();
    QVector<double> candidates;
    if ( planes.firstPlane.success )
      candidates.append( roofPlaneZ( planes.firstPlane, boundaryXY ) );
    if ( planes.secondPlane.success )
      candidates.append( roofPlaneZ( planes.secondPlane, boundaryXY ) );

    double bestHeight = std::numeric_limits<double>::quiet_NaN();
    double bestScore = std::numeric_limits<double>::max();
    for ( double height : candidates )
    {
      if ( !std::isfinite( height ) || height <= 0.0 || height >= maxHeight )
        continue;

      double score = std::fabs( height - oldHeight );
      if ( height > oldHeight )
        score *= 1.35;
      if ( score < bestScore )
      {
        bestScore = score;
        bestHeight = height;
      }
    }

    return bestHeight;
  }

  double dominantSideSign( const QVector<QgsPoint> &points, const QgsPoint &ridgePoint, double normalX, double normalY )
  {
    QVector<double> distances;
    distances.reserve( points.size() );
    for ( const QgsPoint &point : points )
    {
      const double distance = signedDistanceToLine( QgsPointXY( point.x(), point.y() ), ridgePoint, normalX, normalY );
      if ( std::fabs( distance ) > 1e-7 )
        distances.append( distance );
    }

    if ( distances.isEmpty() )
      return 0.0;

    std::sort( distances.begin(), distances.end() );
    const double medianDistance = sortedPercentile( distances, 0.50 );
    if ( std::fabs( medianDistance ) <= 1e-8 )
      return 0.0;
    return medianDistance > 0.0 ? 1.0 : -1.0;
  }

  const LeastSquaresRoofPlane *planeForRidgeSide( const GabledBoundarySlopePlanes &planes, bool positiveSide )
  {
    if ( !planes.success || !planes.firstPlane.success || !planes.secondPlane.success || !planes.hasSideSigns )
      return nullptr;

    const double sideSign = positiveSide ? 1.0 : -1.0;
    if ( planes.firstPlaneSideSign * sideSign > 0.0 )
      return &planes.firstPlane;
    if ( planes.secondPlaneSideSign * sideSign > 0.0 )
      return &planes.secondPlane;
    return nullptr;
  }

  double fittedGabledPlaneRoofZ( const LeastSquaresRoofPlane &plane, const QgsPointXY &point, double fallbackZ )
  {
    const double z = roofPlaneZ( plane, point );
    if ( !std::isfinite( z ) || z <= 0.0 )
      return fallbackZ;
    return z;
  }

  double leastSquaresGabledTopZForSide( const QgsPointXY &point, const GabledBoundarySlopePlanes &planes, const QgsPoint &ridgePoint, double normalX, double normalY, double fallbackZ )
  {
    const double distance = signedDistanceToLine( point, ridgePoint, normalX, normalY );
    const LeastSquaresRoofPlane *plane = planeForRidgeSide( planes, distance > 0.0 );
    if ( !plane )
      return fallbackZ;
    return fittedGabledPlaneRoofZ( *plane, point, fallbackZ );
  }

  bool leastSquaresPlanesReliableForClippedMesh( const GabledBoundarySlopePlanes &planes, const QgsPoint &boundaryPoint, const QgsPoint &ridgePoint, double normalX, double normalY )
  {
    const LeastSquaresRoofPlane *positivePlane = planeForRidgeSide( planes, true );
    const LeastSquaresRoofPlane *negativePlane = planeForRidgeSide( planes, false );
    if ( !positivePlane || !negativePlane )
      return false;

    const double positiveSlope = positivePlane->a * normalX + positivePlane->b * normalY;
    const double negativeSlope = negativePlane->a * normalX + negativePlane->b * normalY;
    if ( positiveSlope >= -1e-5 || negativeSlope <= 1e-5 )
      return false;

    const double rise = ridgePoint.z() - boundaryPoint.z();
    if ( !std::isfinite( rise ) || rise <= 0.20 )
      return false;

    const QgsPointXY boundaryXY( boundaryPoint.x(), boundaryPoint.y() );
    const bool positiveBoundarySide = signedDistanceToLine( boundaryXY, ridgePoint, normalX, normalY ) > 0.0;
    const LeastSquaresRoofPlane *boundaryPlane = planeForRidgeSide( planes, positiveBoundarySide );
    if ( !boundaryPlane )
      return false;

    const double boundaryPlaneZ = fittedGabledPlaneRoofZ( *boundaryPlane, boundaryXY, boundaryPoint.z() );
    if ( !std::isfinite( boundaryPlaneZ ) || boundaryPlaneZ <= 0.0 || boundaryPlaneZ >= ridgePoint.z() - 0.10 )
      return false;

    return true;
  }

  void appendTriangulatedLeastSquaresRoofSurface( BuildingRoof::Mesh &mesh, const QVector<QgsPointXY> &polygon, const LeastSquaresRoofPlane &plane, double fallbackZ )
  {
    if ( polygon.size() < 3 )
      return;

    QVector<QgsPointXY> localRing = polygon;
    double area = 0.0;
    for ( int i = 0; i < localRing.size(); ++i )
    {
      const QgsPointXY &a = localRing[i];
      const QgsPointXY &b = localRing[( i + 1 ) % localRing.size()];
      area += a.x() * b.y() - b.x() * a.y();
    }
    if ( area < 0.0 )
      std::reverse( localRing.begin(), localRing.end() );

    const int vertexOffset = mesh.vertices.size();
    for ( const QgsPointXY &point : localRing )
      mesh.vertices.append( QgsPoint( point.x(), point.y(), fittedGabledPlaneRoofZ( plane, point, fallbackZ ) ) );

    const QVector<int> triangles = triangulateRing( localRing );
    for ( int i = 0; i + 2 < triangles.size(); i += 3 )
      mesh.indices << vertexOffset + triangles[i] << vertexOffset + triangles[i + 1] << vertexOffset + triangles[i + 2];
  }

  LeastSquaresRoofPlane fitRobustRoofPlane( const QVector<QgsPoint> &points, QVector<QgsPoint> *inliersOut = nullptr )
  {
    if ( inliersOut )
      inliersOut->clear();
    LeastSquaresRoofPlane fallbackPlane = fitLeastSquaresRoofPlane( points );
    if ( points.size() < 24 )
    {
      if ( inliersOut )
        *inliersOut = points;
      return fallbackPlane;
    }

    const double threshold = robustPlaneThreshold( points );
    const double candidateSpread = pointCloudXySpread( points );
    const double minInlierSpread = candidateSpread * 0.25;
    const int minInliers = std::max( 8, static_cast<int>( std::ceil( points.size() * 0.28 ) ) );
    const int iterations = points.size() > 120 ? 260 : 160;
    quint32 state = 2166136261u ^ static_cast<quint32>( points.size() * 16777619u );
    auto nextIndex = [&state, &points]() -> int {
      state = state * 1664525u + 1013904223u;
      return static_cast<int>( state % static_cast<quint32>( points.size() ) );
    };

    QVector<QgsPoint> bestInliers;
    double bestScore = std::numeric_limits<double>::max();
    int bestCount = 0;

    for ( int iter = 0; iter < iterations; ++iter )
    {
      const int i0 = nextIndex();
      int i1 = nextIndex();
      int i2 = nextIndex();
      if ( i1 == i0 )
        i1 = ( i1 + 1 ) % points.size();
      if ( i2 == i0 || i2 == i1 )
        i2 = ( i2 + points.size() / 2 + 1 ) % points.size();
      if ( i2 == i0 || i2 == i1 )
        continue;

      LeastSquaresRoofPlane candidatePlane;
      if ( !planeFromThreePoints( points.at( i0 ), points.at( i1 ), points.at( i2 ), candidatePlane ) )
        continue;

      QVector<QgsPoint> inliers;
      inliers.reserve( points.size() );
      double squaredError = 0.0;
      for ( const QgsPoint &point : points )
      {
        const double residual = roofPlaneResidual( candidatePlane, point );
        if ( residual <= threshold )
        {
          inliers.append( point );
          squaredError += residual * residual;
        }
      }

      if ( inliers.size() < minInliers )
        continue;
      if ( pointCloudXySpread( inliers ) < minInlierSpread )
        continue;

      const double meanError = squaredError / std::max( 1, inliers.size() );
      const double coverage = candidateSpread <= 1e-8 ? 1.0 : std::min( 1.0, pointCloudXySpread( inliers ) / candidateSpread );
      const double score = meanError + ( 1.0 - coverage ) * threshold * 0.25 - inliers.size() * 0.0005;
      if ( inliers.size() > bestCount || ( inliers.size() == bestCount && score < bestScore ) )
      {
        bestCount = inliers.size();
        bestScore = score;
        bestInliers = inliers;
      }
    }

    if ( bestInliers.size() < minInliers )
    {
      if ( inliersOut )
        *inliersOut = points;
      return fallbackPlane;
    }

    LeastSquaresRoofPlane refinedPlane = fitLeastSquaresRoofPlane( bestInliers );
    if ( !refinedPlane.success )
    {
      if ( inliersOut )
        *inliersOut = points;
      return fallbackPlane;
    }

    QVector<QgsPoint> refinedInliers = robustPlaneInliers( points, refinedPlane, threshold * 1.15 );
    if ( refinedInliers.size() >= minInliers )
    {
      const LeastSquaresRoofPlane secondRefinedPlane = fitLeastSquaresRoofPlane( refinedInliers );
      if ( secondRefinedPlane.success )
        refinedPlane = secondRefinedPlane;
    }

    if ( fallbackPlane.success && refinedPlane.count < std::max( minInliers, static_cast<int>( std::ceil( points.size() * 0.22 ) ) ) )
    {
      if ( inliersOut )
        *inliersOut = points;
      return fallbackPlane;
    }

    if ( inliersOut )
    {
      QVector<QgsPoint> finalInliers = robustPlaneInliers( points, refinedPlane, threshold * 1.15 );
      *inliersOut = finalInliers.size() >= minInliers ? finalInliers : bestInliers;
    }

    return refinedPlane;
  }

  struct AreaFilteredRoofPlaneFit
  {
    bool success = false;
    LeastSquaresRoofPlane plane;
    QVector<QgsPoint> points;
    double projectedAreaRatio = 0.0;
  };

  double pointSetProjectedArea( const QVector<QgsPoint> &points, double cellSize, double originX, double originY )
  {
    if ( points.isEmpty() || cellSize <= 1e-8 )
      return 0.0;

    std::set<std::pair<int, int>> occupiedCells;
    for ( const QgsPoint &point : points )
    {
      const int x = static_cast<int>( std::floor( ( point.x() - originX ) / cellSize ) );
      const int y = static_cast<int>( std::floor( ( point.y() - originY ) / cellSize ) );
      occupiedCells.insert( std::make_pair( x, y ) );
    }
    return occupiedCells.size() * cellSize * cellSize;
  }

  QVector<QVector<QgsPoint>> projectedPointComponents( const QVector<QgsPoint> &points, double cellSize, double originX, double originY )
  {
    QVector<QVector<QgsPoint>> components;
    if ( points.isEmpty() || cellSize <= 1e-8 )
      return components;

    using Cell = std::pair<int, int>;
    std::map<Cell, QVector<QgsPoint>> pointsByCell;
    for ( const QgsPoint &point : points )
    {
      const int x = static_cast<int>( std::floor( ( point.x() - originX ) / cellSize ) );
      const int y = static_cast<int>( std::floor( ( point.y() - originY ) / cellSize ) );
      pointsByCell[Cell( x, y )].append( point );
    }

    std::set<Cell> visited;
    for ( const auto &entry : pointsByCell )
    {
      if ( visited.find( entry.first ) != visited.end() )
        continue;

      QVector<QgsPoint> component;
      std::queue<Cell> pending;
      pending.push( entry.first );
      visited.insert( entry.first );
      while ( !pending.empty() )
      {
        const Cell cell = pending.front();
        pending.pop();
        const auto cellPoints = pointsByCell.find( cell );
        if ( cellPoints != pointsByCell.end() )
          component += cellPoints->second;

        for ( int dx = -1; dx <= 1; ++dx )
        {
          for ( int dy = -1; dy <= 1; ++dy )
          {
            if ( dx == 0 && dy == 0 )
              continue;
            const Cell neighbour( cell.first + dx, cell.second + dy );
            if ( pointsByCell.find( neighbour ) == pointsByCell.end() || visited.find( neighbour ) != visited.end() )
              continue;
            visited.insert( neighbour );
            pending.push( neighbour );
          }
        }
      }
      if ( !component.isEmpty() )
        components.append( component );
    }

    std::sort( components.begin(), components.end(), [cellSize, originX, originY]( const QVector<QgsPoint> &left, const QVector<QgsPoint> &right ) {
      const double leftArea = pointSetProjectedArea( left, cellSize, originX, originY );
      const double rightArea = pointSetProjectedArea( right, cellSize, originX, originY );
      if ( std::fabs( leftArea - rightArea ) > 1e-9 )
        return leftArea > rightArea;
      return left.size() > right.size();
    } );
    return components;
  }

  QVector<QgsPoint> largestProjectedPointComponent( const QVector<QgsPoint> &points, double cellSize, double originX, double originY )
  {
    const QVector<QVector<QgsPoint>> components = projectedPointComponents( points, cellSize, originX, originY );
    return components.isEmpty() ? QVector<QgsPoint>() : components.first();
  }

  AreaFilteredRoofPlaneFit fitAreaFilteredRoofPlane( const QVector<QgsPoint> &points, double minimumProjectedAreaRatio = 0.15 )
  {
    AreaFilteredRoofPlaneFit result;
    if ( points.size() < 8 )
      return result;

    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double maxY = std::numeric_limits<double>::lowest();
    for ( const QgsPoint &point : points )
    {
      minX = std::min( minX, point.x() );
      minY = std::min( minY, point.y() );
      maxX = std::max( maxX, point.x() );
      maxY = std::max( maxY, point.y() );
    }
    const double extent = std::hypot( maxX - minX, maxY - minY );
    const double cellSize = std::max( 0.20, std::min( 0.60, extent * 0.025 ) );
    const double referenceArea = pointSetProjectedArea( points, cellSize, minX, minY );
    if ( referenceArea <= 1e-8 )
      return result;

    QVector<QgsPoint> remainingPoints = points;
    bool removedSmallPlane = false;
    for ( int pass = 0; pass < 4 && remainingPoints.size() >= 8; ++pass )
    {
      QVector<QgsPoint> planePoints;
      const LeastSquaresRoofPlane plane = fitRobustRoofPlane( remainingPoints, &planePoints );
      if ( !plane.success || planePoints.size() < 8 )
        break;

      const QVector<QgsPoint> connectedPlanePoints = largestProjectedPointComponent( planePoints, cellSize, minX, minY );
      if ( connectedPlanePoints.size() < 8 )
        break;
      const double projectedArea = pointSetProjectedArea( connectedPlanePoints, cellSize, minX, minY );
      const double areaRatio = std::min( 1.0, projectedArea / referenceArea );
      if ( areaRatio >= minimumProjectedAreaRatio )
      {
        result.success = true;
        result.plane = fitLeastSquaresRoofPlane( connectedPlanePoints );
        if ( !result.plane.success )
          result.plane = plane;
        result.points = connectedPlanePoints;
        result.projectedAreaRatio = areaRatio;
        return result;
      }

      removedSmallPlane = true;
      const double threshold = robustPlaneThreshold( planePoints ) * 1.15;
      QVector<QgsPoint> filteredPoints;
      filteredPoints.reserve( remainingPoints.size() - planePoints.size() );
      for ( const QgsPoint &point : remainingPoints )
      {
        if ( roofPlaneResidual( plane, point ) > threshold )
          filteredPoints.append( point );
      }
      if ( filteredPoints.size() >= remainingPoints.size() || filteredPoints.size() < 8 )
        break;
      remainingPoints = filteredPoints;
    }

    const QVector<QgsPoint> &fallbackPoints = removedSmallPlane && remainingPoints.size() >= 8 ? remainingPoints : points;
    QVector<QgsPoint> fallbackInliers;
    const LeastSquaresRoofPlane fallbackPlane = fitRobustRoofPlane( fallbackPoints, &fallbackInliers );
    if ( fallbackPlane.success && fallbackInliers.size() >= 8 )
    {
      const QVector<QgsPoint> connectedFallbackInliers = largestProjectedPointComponent( fallbackInliers, cellSize, minX, minY );
      const double fallbackAreaRatio = std::min( 1.0, pointSetProjectedArea( connectedFallbackInliers, cellSize, minX, minY ) / referenceArea );
      if ( connectedFallbackInliers.size() >= 8 && fallbackAreaRatio >= minimumProjectedAreaRatio )
      {
        result.success = true;
        result.plane = fitLeastSquaresRoofPlane( connectedFallbackInliers );
        if ( !result.plane.success )
          result.plane = fallbackPlane;
        result.points = connectedFallbackInliers;
        result.projectedAreaRatio = fallbackAreaRatio;
      }
    }
    return result;
  }

  struct SpatialRoofPlane
  {
    bool success = false;
    QgsPoint origin;
    double normal[3] = { 0.0, 0.0, 1.0 };
    double rmse = 0.0;
    double majorSpread = 0.0;
    double minorSpread = 0.0;
  };

  double spatialRoofPlaneResidual( const SpatialRoofPlane &plane, const QgsPoint &point )
  {
    return std::fabs( plane.normal[0] * ( point.x() - plane.origin.x() )
                      + plane.normal[1] * ( point.y() - plane.origin.y() )
                      + plane.normal[2] * ( point.z() - plane.origin.z() ) );
  }

  SpatialRoofPlane fitSpatialRoofPlane( const QVector<QgsPoint> &points )
  {
    SpatialRoofPlane result;
    if ( points.size() < 3 )
      return result;
    double center[3] = { 0.0, 0.0, 0.0 };
    for ( const QgsPoint &point : points )
    {
      center[0] += point.x();
      center[1] += point.y();
      center[2] += point.z();
    }
    for ( double &value : center )
      value /= points.size();
    double covariance[3][3] = {};
    for ( const QgsPoint &point : points )
    {
      const double delta[3] = { point.x() - center[0], point.y() - center[1], point.z() - center[2] };
      for ( int row = 0; row < 3; ++row )
        for ( int column = 0; column < 3; ++column )
          covariance[row][column] += delta[row] * delta[column] / points.size();
    }
    double vectors[3][3] = { { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0 } };
    // The smallest covariance eigenvector is the orthogonal least-squares plane normal.
    for ( int iteration = 0; iteration < 32; ++iteration )
    {
      int p = 0;
      int q = 1;
      for ( int row = 0; row < 3; ++row )
        for ( int column = row + 1; column < 3; ++column )
          if ( std::fabs( covariance[row][column] ) > std::fabs( covariance[p][q] ) )
          {
            p = row;
            q = column;
          }
      if ( std::fabs( covariance[p][q] ) <= 1e-12 )
        break;
      const double angle = 0.5 * std::atan2( 2.0 * covariance[p][q], covariance[q][q] - covariance[p][p] );
      const double c = std::cos( angle );
      const double s = std::sin( angle );
      const double pp = covariance[p][p];
      const double qq = covariance[q][q];
      const double pq = covariance[p][q];
      for ( int k = 0; k < 3; ++k )
      {
        if ( k != p && k != q )
        {
          const double kp = covariance[k][p];
          const double kq = covariance[k][q];
          covariance[k][p] = covariance[p][k] = c * kp - s * kq;
          covariance[k][q] = covariance[q][k] = s * kp + c * kq;
        }
        const double vp = vectors[k][p];
        const double vq = vectors[k][q];
        vectors[k][p] = c * vp - s * vq;
        vectors[k][q] = s * vp + c * vq;
      }
      covariance[p][p] = c * c * pp - 2.0 * s * c * pq + s * s * qq;
      covariance[q][q] = s * s * pp + 2.0 * s * c * pq + c * c * qq;
      covariance[p][q] = covariance[q][p] = 0.0;
    }
    int order[3] = { 0, 1, 2 };
    std::sort( order, order + 3, [&]( int lhs, int rhs ) { return covariance[lhs][lhs] < covariance[rhs][rhs]; } );
    result.origin = QgsPoint( center[0], center[1], center[2] );
    for ( int k = 0; k < 3; ++k )
      result.normal[k] = vectors[k][order[0]];
    if ( result.normal[2] < 0.0 )
      for ( double &value : result.normal )
        value = -value;
    result.rmse = std::sqrt( std::max( 0.0, covariance[order[0]][order[0]] ) );
    result.minorSpread = std::sqrt( std::max( 0.0, covariance[order[1]][order[1]] ) );
    result.majorSpread = std::sqrt( std::max( 0.0, covariance[order[2]][order[2]] ) );
    result.success = std::isfinite( result.rmse ) && result.minorSpread > 1e-8;
    return result;
  }

  bool extractSpatialRansacRoofPlane( const QVector<QgsPoint> &points, int minimumInliers, double threshold,
                                     bool horizontalOnly, SpatialRoofPlane &plane, QVector<QgsPoint> &inliers )
  {
    if ( points.size() < std::max( 3, minimumInliers ) )
      return false;
    quint32 state = 2166136261u ^ static_cast<quint32>( points.size() * 16777619u );
    auto nextIndex = [&]() {
      state = state * 1664525u + 1013904223u;
      return static_cast<int>( state % static_cast<quint32>( points.size() ) );
    };
    int bestCount = 0;
    SpatialRoofPlane bestCandidate;
    double bestError = std::numeric_limits<double>::max();
    const int iterations = horizontalOnly ? 700 : 520;
    const double horizontalNormal = 1.0 / std::sqrt( 1.0 + 0.15 * 0.15 );
    for ( int iteration = 0; iteration < iterations; ++iteration )
    {
      const int a = nextIndex();
      const int b = nextIndex();
      const int c = nextIndex();
      if ( a == b || a == c || b == c )
        continue;
      const QgsPoint &origin = points.at( a );
      const double u[3] = { points.at( b ).x() - origin.x(), points.at( b ).y() - origin.y(), points.at( b ).z() - origin.z() };
      const double v[3] = { points.at( c ).x() - origin.x(), points.at( c ).y() - origin.y(), points.at( c ).z() - origin.z() };
      SpatialRoofPlane candidate;
      candidate.origin = origin;
      candidate.normal[0] = u[1] * v[2] - u[2] * v[1];
      candidate.normal[1] = u[2] * v[0] - u[0] * v[2];
      candidate.normal[2] = u[0] * v[1] - u[1] * v[0];
      const double length = std::sqrt( candidate.normal[0] * candidate.normal[0] + candidate.normal[1] * candidate.normal[1]
                                       + candidate.normal[2] * candidate.normal[2] );
      if ( length <= 1e-10 )
        continue;
      for ( double &value : candidate.normal )
        value /= length;
      if ( horizontalOnly && std::fabs( candidate.normal[2] ) < horizontalNormal )
        continue;
      int supportCount = 0;
      double error = 0.0;
      int remainingPoints = points.size();
      for ( const QgsPoint &point : points )
      {
        --remainingPoints;
        const double residual = spatialRoofPlaneResidual( candidate, point );
        if ( residual <= threshold )
        {
          ++supportCount;
          error += residual * residual;
        }
        if ( supportCount + remainingPoints < bestCount )
          break;
      }
      if ( supportCount >= minimumInliers && ( supportCount > bestCount
           || ( supportCount == bestCount && error < bestError ) ) )
      {
        bestCount = supportCount;
        bestCandidate = candidate;
        bestError = error;
      }
    }
    if ( bestCount < minimumInliers )
      return false;
    QVector<QgsPoint> best;
    best.reserve( bestCount );
    for ( const QgsPoint &point : points )
      if ( spatialRoofPlaneResidual( bestCandidate, point ) <= threshold )
        best.append( point );
    for ( int iteration = 0; iteration < 2; ++iteration )
    {
      plane = fitSpatialRoofPlane( best );
      if ( !plane.success || ( horizontalOnly && std::fabs( plane.normal[2] ) < horizontalNormal ) )
        return false;
      QVector<QgsPoint> refined;
      for ( const QgsPoint &point : points )
        if ( spatialRoofPlaneResidual( plane, point ) <= threshold )
          refined.append( point );
      if ( refined.size() < minimumInliers )
        return false;
      best = refined;
    }
    plane = fitSpatialRoofPlane( best );
    inliers = best;
    return plane.success && ( !horizontalOnly || std::fabs( plane.normal[2] ) >= horizontalNormal );
  }

  bool extractRansacRoofPlane( const QVector<QgsPoint> &points, int minimumInliers, LeastSquaresRoofPlane &plane, QVector<QgsPoint> &inliers,
                               bool horizontalOnly = false, double residualThreshold = 0.0 )
  {
    plane = LeastSquaresRoofPlane();
    inliers.clear();
    if ( points.size() < std::max( 3, minimumInliers ) )
      return false;

    const double threshold = residualThreshold > 0.0 ? residualThreshold : robustPlaneThreshold( points );
    const int iterations = horizontalOnly ? 700 : ( points.size() > 200 ? 520 : 320 );
    quint32 state = 2166136261u ^ static_cast<quint32>( points.size() * 16777619u );
    auto nextIndex = [&state, &points]() -> int {
      state = state * 1664525u + 1013904223u;
      return static_cast<int>( state % static_cast<quint32>( points.size() ) );
    };

    int bestInlierCount = 0;
    LeastSquaresRoofPlane bestCandidate;
    double bestMeanError = std::numeric_limits<double>::max();
    for ( int iteration = 0; iteration < iterations; ++iteration )
    {
      const int firstIndex = nextIndex();
      int secondIndex = nextIndex();
      int thirdIndex = nextIndex();
      if ( secondIndex == firstIndex )
        secondIndex = ( secondIndex + 1 ) % points.size();
      if ( thirdIndex == firstIndex || thirdIndex == secondIndex )
        thirdIndex = ( thirdIndex + points.size() / 2 + 1 ) % points.size();
      if ( thirdIndex == firstIndex || thirdIndex == secondIndex )
        continue;

      LeastSquaresRoofPlane candidate;
      if ( !planeFromThreePoints( points.at( firstIndex ), points.at( secondIndex ), points.at( thirdIndex ), candidate ) )
        continue;

      const double slope = std::hypot( candidate.a, candidate.b );
      if ( !std::isfinite( slope ) || slope > ( horizontalOnly ? 0.15 : 5.0 ) )
        continue;

      int candidateInlierCount = 0;
      double squaredError = 0.0;
      int remainingPoints = points.size();
      for ( const QgsPoint &point : points )
      {
        --remainingPoints;
        const double residual = roofPlaneResidual( candidate, point );
        if ( residual <= threshold )
        {
          ++candidateInlierCount;
          squaredError += residual * residual;
        }
        if ( candidateInlierCount + remainingPoints < bestInlierCount )
          break;
      }
      if ( candidateInlierCount < minimumInliers )
        continue;

      const double meanError = squaredError / candidateInlierCount;
      if ( candidateInlierCount > bestInlierCount
           || ( candidateInlierCount == bestInlierCount && meanError < bestMeanError ) )
      {
        bestInlierCount = candidateInlierCount;
        bestCandidate = candidate;
        bestMeanError = meanError;
      }
    }

    if ( bestInlierCount < minimumInliers )
      return false;

    QVector<QgsPoint> bestInliers;
    bestInliers.reserve( bestInlierCount );
    for ( const QgsPoint &point : points )
      if ( roofPlaneResidual( bestCandidate, point ) <= threshold )
        bestInliers.append( point );
    plane = fitLeastSquaresRoofPlane( bestInliers );
    if ( !plane.success || ( horizontalOnly && std::hypot( plane.a, plane.b ) > 0.15 ) )
      return false;

    QVector<QgsPoint> refinedInliers = robustPlaneInliers( points, plane, threshold * 1.15 );
    if ( refinedInliers.size() >= minimumInliers )
    {
      const LeastSquaresRoofPlane refinedPlane = fitLeastSquaresRoofPlane( refinedInliers );
      if ( refinedPlane.success )
        plane = refinedPlane;
      inliers = refinedInliers;
    }
    else
    {
      inliers = bestInliers;
    }
    return true;
  }

  struct HorizontalRoofRegion
  {
    bool success = false;
    double height = 0.0;
    double areaRatio = 0.0;
    double clusterRadius = 0.0;
    LeastSquaresRoofPlane plane;
    QVector<QgsPoint> points;
  };

  QVector<HorizontalRoofRegion> segmentedHorizontalRoofRegions( const QVector<QgsPointXY> &ring,
                                                          const QVector<BuildingRoof::RoofSample> &pointCloudSamples,
                                                          QVector<BuildingRoof::RoofSample> *qualifiedSamples = nullptr,
                                                          QVector<BuildingRoof::RoofSample> *surfaceSamples = nullptr,
                                                          const BuildingRoof::RoofPlaneSegmentation *planeSegmentation = nullptr,
                                                          bool expandPlaneSupport = false, bool separateHeightLevels = false );
  QgsGeometry reliefGeometryFromPoints( const HorizontalRoofRegion &region, const QgsGeometry &footprint, double extent,
                                        bool preserveHoles );

  QVector<HorizontalRoofRegion> horizontalRoofRegions( const QVector<QgsPointXY> &ring, const QVector<BuildingRoof::RoofSample> &pointCloudSamples,
                                                      bool useHeightHint, double hintHeight = 0.0 )
  {
    QVector<HorizontalRoofRegion> regions;
    if ( ring.size() < 3 || pointCloudSamples.size() < 24 )
      return regions;

    QVector<double> insideHeights;
    insideHeights.reserve( pointCloudSamples.size() );
    for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
    {
      const QgsPoint &point = sample.point;
      if ( std::isfinite( point.z() ) && point.z() >= 0.0 && pointInRing( ring, QgsPointXY( point.x(), point.y() ) ) )
        insideHeights.append( point.z() );
    }
    if ( insideHeights.size() < 24 )
      return regions;
    std::sort( insideHeights.begin(), insideHeights.end() );
    const double heightSpan = sortedPercentile( insideHeights, 0.995 ) - sortedPercentile( insideHeights, 0.05 );
    const double searchRange = std::max( 0.75, std::min( 2.0, heightSpan * 0.18 ) );

    QVector<QgsPoint> candidates;
    candidates.reserve( insideHeights.size() );
    for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
    {
      const QgsPoint &point = sample.point;
      if ( std::isfinite( point.z() ) && point.z() >= 0.0 && ( !useHeightHint || std::fabs( point.z() - hintHeight ) <= searchRange )
           && pointInRing( ring, QgsPointXY( point.x(), point.y() ) ) )
        candidates.append( point );
    }
    if ( candidates.size() < 24 )
      return regions;
    QVector<BuildingRoof::RoofSample> candidateSamples;
    candidateSamples.reserve( candidates.size() );
    for ( const QgsPoint &point : candidates )
      candidateSamples.append( BuildingRoof::RoofSample{ point } );
    return segmentedHorizontalRoofRegions( ring, candidateSamples );
  }

  HorizontalRoofRegion horizontalRoofRegionNearHint( const QVector<HorizontalRoofRegion> &regions, const QgsPointXY &hint, double extent )
  {
    HorizontalRoofRegion result;
    double bestScore = std::numeric_limits<double>::max();
    for ( const HorizontalRoofRegion &region : regions )
    {
      double nearestDistance2 = std::numeric_limits<double>::max();
      for ( const QgsPoint &point : region.points )
      {
        const double dx = point.x() - hint.x();
        const double dy = point.y() - hint.y();
        nearestDistance2 = std::min( nearestDistance2, dx * dx + dy * dy );
      }
      if ( std::sqrt( nearestDistance2 ) > std::max( 0.80, extent * 0.25 ) )
        continue;
      const double score = std::sqrt( nearestDistance2 ) / std::max( 1.0, extent ) - std::min( 1.0, region.areaRatio ) * 0.05;
      if ( score < bestScore )
      {
        result = region;
        bestScore = score;
      }
    }
    return result;
  }

  HorizontalRoofRegion horizontalRoofRegionNearPoint( const QVector<QgsPointXY> &ring, const QVector<BuildingRoof::RoofSample> &pointCloudSamples, const QgsPoint &hint )
  {
    return horizontalRoofRegionNearHint( horizontalRoofRegions( ring, pointCloudSamples, true, hint.z() ),
                                         QgsPointXY( hint.x(), hint.y() ), ringExtentSize( ring ) );
  }

  struct FlatReliefRegions
  {
    bool success = false;
    bool boundaryConnected = false;
    HorizontalRoofRegion outer;
    HorizontalRoofRegion inner;
    QgsGeometry initialReliefGeometry;
    double score = 0.0;
  };

  bool horizontalRoofRegionsTouch( const HorizontalRoofRegion &first, const HorizontalRoofRegion &second )
  {
    if ( first.points.isEmpty() || second.points.isEmpty() )
      return false;
    const double radius = std::max( first.clusterRadius, second.clusterRadius ) * 1.5;
    if ( radius <= 0.0 )
      return false;
    const QVector<QgsPoint> &indexed = first.points.size() <= second.points.size() ? first.points : second.points;
    const QVector<QgsPoint> &searched = first.points.size() <= second.points.size() ? second.points : first.points;
    double originX = indexed.first().x();
    double originY = indexed.first().y();
    for ( const QgsPoint &point : indexed )
    {
      originX = std::min( originX, point.x() );
      originY = std::min( originY, point.y() );
    }
    auto cell = [&]( const QgsPoint &point ) {
      return std::make_pair( static_cast<int>( std::floor( ( point.x() - originX ) / radius ) ),
                             static_cast<int>( std::floor( ( point.y() - originY ) / radius ) ) );
    };
    std::map<std::pair<int, int>, QVector<int>> grid;
    for ( int i = 0; i < indexed.size(); ++i )
      grid[cell( indexed.at( i ) )].append( i );
    const int minimumContacts = std::max( 3, static_cast<int>( std::ceil( indexed.size() * 0.005 ) ) );
    int contacts = 0;
    for ( const QgsPoint &point : searched )
    {
      const std::pair<int, int> location = cell( point );
      bool pointTouches = false;
      for ( int dx = -1; dx <= 1; ++dx )
      {
        for ( int dy = -1; dy <= 1 && !pointTouches; ++dy )
        {
          const auto found = grid.find( std::make_pair( location.first + dx, location.second + dy ) );
          if ( found == grid.end() )
            continue;
          for ( int index : found->second )
          {
            const QgsPoint &other = indexed.at( index );
            const double offsetX = point.x() - other.x();
            const double offsetY = point.y() - other.y();
            if ( offsetX * offsetX + offsetY * offsetY <= radius * radius )
            {
              pointTouches = true;
              break;
            }
          }
        }
      }
      if ( pointTouches && ++contacts >= minimumContacts )
        return true;
    }
    return false;
  }

  QVector<HorizontalRoofRegion> mergedHorizontalRoofRegions( const QVector<HorizontalRoofRegion> &regions, const QVector<QgsPointXY> &ring,
                                                             bool requireTouching = false )
  {
    QVector<HorizontalRoofRegion> merged = regions;
    bool changed = true;
    while ( changed )
    {
      changed = false;
      for ( int i = 0; i < merged.size() && !changed; ++i )
      {
        for ( int j = i + 1; j < merged.size(); ++j )
        {
          const HorizontalRoofRegion &first = merged.at( i );
          const HorizontalRoofRegion &second = merged.at( j );
          const bool touching = horizontalRoofRegionsTouch( first, second );
          if ( requireTouching && ( !touching || std::fabs( first.height - second.height ) > 0.12 ) )
            continue;
          const double normalDot = ( 1.0 + first.plane.a * second.plane.a + first.plane.b * second.plane.b )
                                   / std::sqrt( ( 1.0 + first.plane.a * first.plane.a + first.plane.b * first.plane.b )
                                                * ( 1.0 + second.plane.a * second.plane.a + second.plane.b * second.plane.b ) );
          if ( !first.plane.success || !second.plane.success || normalDot < 0.9962 )
            continue;
          double firstResidual = 0.0;
          double secondResidual = 0.0;
          for ( const QgsPoint &point : first.points )
            firstResidual += roofPlaneResidual( second.plane, point );
          for ( const QgsPoint &point : second.points )
            secondResidual += roofPlaneResidual( first.plane, point );
          const double maximumMergeDistance = touching ? 0.15 : 0.10;
          const double mergeTolerance = std::max( 0.08, std::min( maximumMergeDistance, 2.0 * std::max( first.plane.rmse, second.plane.rmse ) ) );
          if ( firstResidual / first.points.size() > mergeTolerance || secondResidual / second.points.size() > mergeTolerance )
            continue;
          QVector<QgsPoint> combinedPoints = first.points;
          combinedPoints += second.points;
          const LeastSquaresRoofPlane combinedPlane = fitLeastSquaresRoofPlane( combinedPoints );
          if ( !combinedPlane.success || std::hypot( combinedPlane.a, combinedPlane.b ) > 0.15
               || combinedPlane.rmse > std::max( 0.10, std::max( first.plane.rmse, second.plane.rmse ) * 1.5 ) )
            continue;
          HorizontalRoofRegion &combined = merged[i];
          const HorizontalRoofRegion &other = merged.at( j );
          const int oldCount = combined.points.size();
          combined.height = ( combined.height * oldCount + other.height * other.points.size() ) / ( oldCount + other.points.size() );
          combined.areaRatio = std::min( 1.0, combined.areaRatio + other.areaRatio );
          combined.clusterRadius = std::max( combined.clusterRadius, other.clusterRadius );
          combined.plane = combinedPlane;
          combined.points.swap( combinedPoints );
          merged.removeAt( j );
          changed = true;
          break;
        }
      }
    }
    const QgsGeometry footprint = polygonGeometryFromRing( ring );
    const double footprintArea = std::max( 1e-8, footprint.area() );
    for ( HorizontalRoofRegion &region : merged )
    {
      const QgsGeometry support = reliefGeometryFromPoints( region, footprint, ringExtentSize( ring ), true );
      region.areaRatio = support.isNull() ? 0.0 : std::min( 1.0, support.area() / footprintArea );
    }
    std::stable_sort( merged.begin(), merged.end(), []( const HorizontalRoofRegion &a, const HorizontalRoofRegion &b ) {
      return a.areaRatio > b.areaRatio;
    } );
    return merged;
  }

  QVector<HorizontalRoofRegion> segmentedHorizontalRoofRegions( const QVector<QgsPointXY> &ring,
                                                            const QVector<BuildingRoof::RoofSample> &pointCloudSamples,
                                                            QVector<BuildingRoof::RoofSample> *qualifiedSamples,
                                                            QVector<BuildingRoof::RoofSample> *surfaceSamples,
                                                            const BuildingRoof::RoofPlaneSegmentation *planeSegmentation,
                                                            bool expandPlaneSupport, bool separateHeightLevels )
  {
    QVector<HorizontalRoofRegion> regions;
    if ( ring.size() < 3 )
      return regions;

    if ( pointCloudSamples.size() < 24 )
      return regions;

    const double extent = ringExtentSize( ring );
    double originX = ring.first().x();
    double originY = ring.first().y();
    for ( const QgsPointXY &point : ring )
    {
      originX = std::min( originX, point.x() );
      originY = std::min( originY, point.y() );
    }
    const double footprintArea = std::max( 1e-8, std::fabs( polygonGeometryFromRing( ring ).area() ) );
    const double cellSize = std::max( 0.15, std::min( 0.60, extent * 0.02 ) );
    BuildingRoof::RoofSegmentationOptions options;
    options.horizontalOnly = false;
    options.useRoofHeightBand = false;
    options.splitConnectedComponents = false;
    options.areaReferenceRatio = 0.01;
    options.minimumInlierFraction = 0.0;
    const BuildingRoof::RoofPlaneSegmentation segmentation = planeSegmentation
                                                             ? *planeSegmentation
                                                             : BuildingRoof::segmentRoofCloud( polygonGeometryFromRing( ring ), pointCloudSamples, options );
    for ( const BuildingRoof::RoofPlaneSegment &segment : segmentation.segments )
    {
      if ( std::fabs( segment.normalZ ) < 0.20 )
        continue;
      const QVector<QgsPoint> &planeInliers = segment.points;
      const LeastSquaresRoofPlane plane = fitLeastSquaresRoofPlane( planeInliers );
      if ( !plane.success && !separateHeightLevels )
        continue;
      if ( qualifiedSamples )
      {
        for ( const QgsPoint &point : planeInliers )
          qualifiedSamples->append( BuildingRoof::RoofSample{ point } );
      }
      if ( surfaceSamples && plane.success )
      {
        for ( const QgsPoint &point : planeInliers )
          surfaceSamples->append( BuildingRoof::RoofSample{ QgsPoint( point.x(), point.y(), plane.a * point.x() + plane.b * point.y() + plane.c ) } );
      }
      if ( !separateHeightLevels && std::hypot( plane.a, plane.b ) > 0.15 )
        continue;

      QVector<QgsPoint> planeCandidates = planeInliers;
      std::set<std::tuple<double, double, double>> seedPoints;
      const double expansionTolerance = std::max( 0.20, std::min( 0.40, plane.rmse * 3.0 ) );
      if ( expandPlaneSupport )
      {
        for ( const QgsPoint &point : planeInliers )
          seedPoints.insert( std::make_tuple( point.x(), point.y(), point.z() ) );
        planeCandidates.clear();
        for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
        {
          const QgsPoint &point = sample.point;
          if ( std::isfinite( point.x() ) && std::isfinite( point.y() ) && std::isfinite( point.z() ) && point.z() >= 0.0
               && pointInRing( ring, QgsPointXY( point.x(), point.y() ) ) && roofPlaneResidual( plane, point ) <= expansionTolerance )
            planeCandidates.append( point );
        }
      }
      QVector<ClusterSample> samples;
      samples.reserve( planeCandidates.size() );
      for ( const QgsPoint &point : planeCandidates )
        samples.append( ClusterSample{ QgsPointXY( point.x(), point.y() ),
                                       separateHeightLevels ? point.z() : point.z() - plane.a * point.x() - plane.b * point.y() - plane.c,
                                       0.0, -1, false } );
      const double radius = completePlaneDbscanRadius( planeInliers, extent );
      const double verticalTolerance = std::max( 0.10, std::min( 0.16, radius * 0.15 + 0.07 ) );
      const int clusterCount = assignDbscanClustersSpatial( samples, radius, 4, verticalTolerance );
      QVector<QVector<QgsPoint>> clusters( clusterCount );
      for ( int index = 0; index < samples.size(); ++index )
      {
        const int cluster = samples.at( index ).cluster;
        if ( cluster >= 0 && cluster < clusterCount )
          clusters[cluster].append( planeCandidates.at( index ) );
      }
      for ( QVector<QgsPoint> &points : clusters )
      {
        if ( points.size() < 12 )
          continue;
        if ( expandPlaneSupport )
        {
          int seedSupport = 0;
          for ( const QgsPoint &point : points )
            seedSupport += seedPoints.count( std::make_tuple( point.x(), point.y(), point.z() ) ) != 0 ? 1 : 0;
          if ( seedSupport < 3 )
            continue;
        }
        const LeastSquaresRoofPlane fitted = fitLeastSquaresRoofPlane( points );
        const double maximumRmse = expandPlaneSupport ? expansionTolerance : std::max( 0.15, robustPlaneThreshold( points ) );
        if ( !fitted.success || std::hypot( fitted.a, fitted.b ) > 0.15
             || fitted.rmse > maximumRmse )
          continue;

        double heightSum = 0.0;
        for ( const QgsPoint &point : points )
          heightSum += point.z();
        HorizontalRoofRegion region;
        region.success = true;
        region.height = heightSum / points.size();
        region.areaRatio = std::min( 1.0, pointSetProjectedArea( points, cellSize, originX, originY ) / footprintArea );
        region.clusterRadius = radius;
        region.plane = fitted;
        region.points.swap( points );
        regions.append( region );
      }
    }
    return mergedHorizontalRoofRegions( regions, ring, separateHeightLevels );
  }

  QVector<HorizontalRoofRegion> segmentedFlatReliefPlanes( const QVector<QgsPointXY> &ring,
                                                           const QVector<BuildingRoof::RoofSample> &pointCloudSamples )
  {
    BuildingRoof::RoofSegmentationOptions options;
    options.horizontalOnly = true;
    options.useRoofHeightBand = false;
    options.splitConnectedComponents = false;
    options.areaReferenceRatio = 0.01;
    options.minimumInlierFraction = 0.0;
    options.planeResidualThreshold = 0.15;
    const BuildingRoof::RoofPlaneSegmentation segmentation = BuildingRoof::segmentRoofCloud( polygonGeometryFromRing( ring ), pointCloudSamples, options );
    // Restore the relief-specific sequence: horizontal RANSAC, height-aware DBSCAN, then per-cluster least squares.
    return segmentedHorizontalRoofRegions( ring, pointCloudSamples, nullptr, nullptr, &segmentation, false, true );
  }

  QVector<BuildingRoof::RoofSample> roofSurfaceObservations( const QVector<QgsPointXY> &ring,
                                                            const QVector<BuildingRoof::RoofSample> &samples )
  {
    // Score supported upper surfaces once per XY cell, independently of point density and lower returns.
    const QgsRectangle bounds = polygonGeometryFromRing( ring ).boundingBox();
    const double cellSize = std::max( 0.15, std::min( 0.75, ringExtentSize( ring ) * 0.02 ) );
    std::map<std::pair<int, int>, QgsPoint> cells;
    for ( const BuildingRoof::RoofSample &sample : samples )
    {
      const QgsPoint &point = sample.point;
      if ( !std::isfinite( point.x() ) || !std::isfinite( point.y() ) || !std::isfinite( point.z() ) || point.z() < 0.0
           || !pointInRing( ring, QgsPointXY( point.x(), point.y() ) ) )
        continue;
      const auto key = std::make_pair( static_cast<int>( std::floor( ( point.x() - bounds.xMinimum() ) / cellSize ) ),
                                       static_cast<int>( std::floor( ( point.y() - bounds.yMinimum() ) / cellSize ) ) );
      const auto found = cells.find( key );
      if ( found == cells.end() || point.z() > found->second.z() )
        cells[key] = point;
    }
    QVector<BuildingRoof::RoofSample> observations;
    observations.reserve( static_cast<int>( cells.size() ) );
    for ( const auto &cell : cells )
      observations.append( BuildingRoof::RoofSample{ cell.second } );
    return observations;
  }

  double roofSurfaceObservationLoss( const BuildingRoof::Mesh &mesh, const QVector<BuildingRoof::RoofSample> &observations, double scale )
  {
    const QVector<RoofHeightTriangle> triangles = roofHeightTriangles( mesh, true );
    if ( triangles.isEmpty() || observations.isEmpty() )
      return std::numeric_limits<double>::max();
    double loss = 0.0;
    for ( const BuildingRoof::RoofSample &sample : observations )
    {
      const double residual = std::fabs( roofHeightAt( triangles, sample.point ) - sample.point.z() ) / scale;
      loss += std::min( 16.0, residual * residual );
    }
    return loss / observations.size();
  }

  QgsGeometry alignReliefPartToFootprint( const QgsPolygonXY &part, const QVector<QgsPointXY> &footprintRing,
                                         double axisX, double axisY, double minimumEdgeLength, double snapTolerance )
  {
    const QVector<QgsPointXY> ring = exteriorRing( part );
    if ( ring.size() < 4 )
      return QgsGeometry();

    const QgsPointXY origin = footprintRing.first();
    auto toLocal = [&]( const QgsPointXY &point ) {
      const double x = point.x() - origin.x();
      const double y = point.y() - origin.y();
      return QgsPointXY( x * axisX + y * axisY, -x * axisY + y * axisX );
    };
    auto fromLocal = [&]( double u, double v ) {
      return QgsPointXY( origin.x() + u * axisX - v * axisY, origin.y() + u * axisY + v * axisX );
    };

    struct AxisRun
    {
      bool alongAxis = false;
      double offset = 0.0;
      double weight = 0.0;
    };
    QVector<AxisRun> runs;
    for ( int i = 0; i < ring.size(); ++i )
    {
      const QgsPointXY a = toLocal( ring.at( i ) );
      const QgsPointXY b = toLocal( ring.at( ( i + 1 ) % ring.size() ) );
      const double du = b.x() - a.x();
      const double dv = b.y() - a.y();
      const double weight = std::hypot( du, dv );
      if ( weight <= 1e-8 )
        continue;
      const bool alongAxis = std::fabs( du ) >= std::fabs( dv );
      const double offset = alongAxis ? ( a.y() + b.y() ) * 0.5 : ( a.x() + b.x() ) * 0.5;
      if ( !runs.isEmpty() && runs.last().alongAxis == alongAxis )
      {
        AxisRun &run = runs.last();
        run.offset = ( run.offset * run.weight + offset * weight ) / ( run.weight + weight );
        run.weight += weight;
      }
      else
        runs.append( AxisRun{ alongAxis, offset, weight } );
    }
    if ( runs.size() > 1 && runs.first().alongAxis == runs.last().alongAxis )
    {
      AxisRun &first = runs.first();
      const AxisRun last = runs.takeLast();
      first.offset = ( first.offset * first.weight + last.offset * last.weight ) / ( first.weight + last.weight );
      first.weight += last.weight;
    }
    if ( runs.size() < 4 || runs.size() % 2 != 0 )
      return QgsGeometry();

    for ( AxisRun &run : runs )
    {
      double bestDistance = snapTolerance;
      for ( int edge = 0; edge < footprintRing.size(); ++edge )
      {
        const QgsPointXY a = toLocal( footprintRing.at( edge ) );
        const QgsPointXY b = toLocal( footprintRing.at( ( edge + 1 ) % footprintRing.size() ) );
        const double du = b.x() - a.x();
        const double dv = b.y() - a.y();
        const double length = std::hypot( du, dv );
        if ( length < minimumEdgeLength )
          continue;
        const double alignment = run.alongAxis ? std::fabs( du ) / length : std::fabs( dv ) / length;
        if ( alignment < 0.9659258262890683 )
          continue;
        const double edgeOffset = run.alongAxis ? ( a.y() + b.y() ) * 0.5 : ( a.x() + b.x() ) * 0.5;
        const double distance = std::fabs( run.offset - edgeOffset );
        if ( distance < bestDistance )
        {
          bestDistance = distance;
          run.offset = edgeOffset;
        }
      }
    }

    auto geometryFromRuns = [&]( const QVector<AxisRun> &candidate ) {
      if ( candidate.size() < 4 || candidate.size() % 2 != 0 )
        return QgsGeometry();
      QVector<QgsPointXY> alignedRing;
      for ( int i = 0; i < candidate.size(); ++i )
      {
        const AxisRun &previous = candidate.at( ( i - 1 + candidate.size() ) % candidate.size() );
        const AxisRun &current = candidate.at( i );
        if ( previous.alongAxis == current.alongAxis )
          return QgsGeometry();
        const double u = current.alongAxis ? previous.offset : current.offset;
        const double v = current.alongAxis ? current.offset : previous.offset;
        alignedRing.append( fromLocal( u, v ) );
      }
      const QgsGeometry geometry = polygonGeometryFromRing( alignedRing );
      return geometry.isGeosValid() && geometry.area() > 1e-8 ? geometry : QgsGeometry();
    };

    const QgsGeometry source = QgsGeometry::fromPolygonXY( part );
    for ( int iteration = 0; iteration < 64 && runs.size() > 4; ++iteration )
    {
      const QgsGeometry current = geometryFromRuns( runs );
      if ( current.isNull() )
        break;
      const QVector<QgsPointXY> currentRing = openRing( firstPolygon( current ).first() );
      int shortestEdge = -1;
      double shortestLength = minimumEdgeLength;
      for ( int i = 0; i < currentRing.size(); ++i )
      {
        const QgsPointXY &a = currentRing.at( i );
        const QgsPointXY &b = currentRing.at( ( i + 1 ) % currentRing.size() );
        const double length = std::hypot( b.x() - a.x(), b.y() - a.y() );
        if ( length < shortestLength )
        {
          shortestLength = length;
          shortestEdge = i;
        }
      }
      if ( shortestEdge < 0 )
        break;

      double bestOverlap = 0.0;
      QVector<AxisRun> bestRuns;
      for ( int neighbor : { ( shortestEdge - 1 + runs.size() ) % runs.size(), ( shortestEdge + 1 ) % runs.size() } )
      {
        QVector<AxisRun> candidate;
        for ( int i = 0; i < runs.size(); ++i )
        {
          if ( i != shortestEdge && i != neighbor )
            candidate.append( runs.at( i ) );
        }
        const QgsGeometry polygon = geometryFromRuns( candidate );
        if ( polygon.isNull() )
          continue;
        const QgsGeometry overlap = polygon.intersection( source );
        const double unionArea = polygon.area() + source.area() - overlap.area();
        const double score = unionArea > 1e-8 ? overlap.area() / unionArea : 0.0;
        if ( score > bestOverlap )
        {
          bestOverlap = score;
          bestRuns = candidate;
        }
      }
      if ( bestOverlap < 0.55 )
        break;
      runs = bestRuns;
    }

    const QgsGeometry aligned = geometryFromRuns( runs );
    if ( aligned.isNull() )
      return QgsGeometry();
    const QgsGeometry overlap = aligned.intersection( source );
    const double unionArea = aligned.area() + source.area() - overlap.area();
    return unionArea > 1e-8 && overlap.area() / unionArea >= 0.60 ? aligned : QgsGeometry();
  }

  QgsGeometry alignReliefGeometryToFootprint( const QgsGeometry &geometry, const QVector<QgsPointXY> &footprintRing,
                                             double minimumEdgeLength, double snapTolerance )
  {
    if ( footprintRing.size() < 3 )
      return QgsGeometry();
    double axisX = 0.0;
    double axisY = 0.0;
    double longestEdge = 0.0;
    for ( int i = 0; i < footprintRing.size(); ++i )
    {
      const QgsPointXY &a = footprintRing.at( i );
      const QgsPointXY &b = footprintRing.at( ( i + 1 ) % footprintRing.size() );
      const double length = std::hypot( b.x() - a.x(), b.y() - a.y() );
      if ( length > longestEdge )
      {
        longestEdge = length;
        axisX = ( b.x() - a.x() ) / length;
        axisY = ( b.y() - a.y() ) / length;
      }
    }
    if ( longestEdge <= 1e-8 )
      return QgsGeometry();

    QVector<QgsPolygonXY> parts;
    const QgsPolygonXY polygon = geometry.asPolygon();
    if ( !polygon.isEmpty() )
      parts.append( polygon );
    else
      parts = geometry.asMultiPolygon();
    QVector<QgsGeometry> alignedParts;
    for ( const QgsPolygonXY &part : parts )
    {
      QgsGeometry aligned = alignReliefPartToFootprint( part, footprintRing, axisX, axisY, minimumEdgeLength, snapTolerance );
      if ( aligned.isNull() )
        aligned = QgsGeometry::fromPolygonXY( part );
      alignedParts.append( aligned );
    }
    if ( alignedParts.isEmpty() )
      return QgsGeometry();
    return alignedParts.size() == 1 ? alignedParts.first() : QgsGeometry::unaryUnion( alignedParts );
  }

  QgsGeometry alignConcaveReliefToFootprint( const QgsGeometry &source, const QVector<QgsPointXY> &footprintRing,
                                            double extent, bool boundaryConnected, const QVector<QgsPoint> &reliefPoints )
  {
    QgsPolygonXY sourcePolygon = source.asPolygon();
    if ( sourcePolygon.isEmpty() )
    {
      const QgsMultiPolygonXY parts = source.asMultiPolygon();
      if ( parts.size() == 1 )
        sourcePolygon = parts.first();
    }
    if ( sourcePolygon.size() != 1 || footprintRing.size() < 3 )
      return QgsGeometry();

    double axisX = 0.0;
    double axisY = 0.0;
    double longestEdge = 0.0;
    for ( int i = 0; i < footprintRing.size(); ++i )
    {
      const QgsPointXY &a = footprintRing.at( i );
      const QgsPointXY &b = footprintRing.at( ( i + 1 ) % footprintRing.size() );
      const double length = std::hypot( b.x() - a.x(), b.y() - a.y() );
      if ( length > longestEdge )
      {
        longestEdge = length;
        axisX = ( b.x() - a.x() ) / length;
        axisY = ( b.y() - a.y() ) / length;
      }
    }
    if ( longestEdge <= 1e-8 )
      return QgsGeometry();

    const double minimumEdgeLength = std::max( 0.20, std::min( 1.50, extent * 0.025 ) );
    const double snapTolerance = boundaryConnected ? std::max( 0.20, std::min( 1.0, extent * 0.04 ) ) : 0.0;
    QgsGeometry aligned = alignReliefPartToFootprint( sourcePolygon, footprintRing, axisX, axisY, minimumEdgeLength, snapTolerance );
    if ( aligned.isNull() )
    {
      const QgsGeometry simplified = source.simplify( minimumEdgeLength * 0.5 );
      QgsPolygonXY simplifiedPolygon = simplified.asPolygon();
      if ( simplifiedPolygon.isEmpty() )
      {
        const QgsMultiPolygonXY parts = simplified.asMultiPolygon();
        if ( parts.size() == 1 )
          simplifiedPolygon = parts.first();
      }
      if ( simplified.isGeosValid() && simplifiedPolygon.size() == 1 )
        aligned = alignReliefPartToFootprint( simplifiedPolygon, footprintRing, axisX, axisY, minimumEdgeLength, snapTolerance );
    }
    if ( aligned.isNull() || !aligned.isGeosValid() )
      return QgsGeometry();

    const QgsGeometry footprint = polygonGeometryFromRing( footprintRing );
    const QVector<QgsPointXY> alignedRing = exteriorRing( firstPolygon( aligned ) );
    double minU = std::numeric_limits<double>::max();
    double maxU = std::numeric_limits<double>::lowest();
    double minV = std::numeric_limits<double>::max();
    double maxV = std::numeric_limits<double>::lowest();
    const QgsPointXY &origin = footprintRing.first();
    for ( const QgsPointXY &point : alignedRing )
    {
      const double dx = point.x() - origin.x();
      const double dy = point.y() - origin.y();
      const double u = dx * axisX + dy * axisY;
      const double v = -dx * axisY + dy * axisX;
      minU = std::min( minU, u );
      maxU = std::max( maxU, u );
      minV = std::min( minV, v );
      maxV = std::max( maxV, v );
    }
    if ( maxU > minU && maxV > minV )
    {
      auto fromLocal = [&]( double u, double v ) {
        return QgsPointXY( origin.x() + u * axisX - v * axisY, origin.y() + u * axisY + v * axisX );
      };
      const QgsGeometry rectangle = polygonGeometryFromRing( QVector<QgsPointXY>{
        fromLocal( minU, minV ), fromLocal( maxU, minV ),
        fromLocal( maxU, maxV ), fromLocal( minU, maxV ) } );
      const QgsGeometry rectangleOverlap = rectangle.intersection( aligned );
      if ( rectangle.isGeosValid() && rectangle.area() > 1e-8
           && rectangleOverlap.area() / rectangle.area() >= 0.94
           && rectangle.difference( footprint ).area() <= std::max( 1e-6, rectangle.area() * 1e-5 ) )
        aligned = rectangle;
    }

    const QgsGeometry overlap = aligned.intersection( source );
    const double unionArea = aligned.area() + source.area() - overlap.area();
    if ( unionArea <= 1e-8 || overlap.area() / unionArea < 0.60 )
      return QgsGeometry();

    if ( boundaryConnected )
    {
      QVector<QgsPointXY> openedRing = exteriorRing( firstPolygon( aligned ) );
      const int minimumSupport = std::max( 4, static_cast<int>( std::ceil( reliefPoints.size() * 0.005 ) ) );
      const double overshoot = std::max( 0.01, extent * 0.001 );
      for ( int edgeIndex = 0; edgeIndex < footprintRing.size(); ++edgeIndex )
      {
        const QgsPointXY &a = footprintRing.at( edgeIndex );
        const QgsPointXY &b = footprintRing.at( ( edgeIndex + 1 ) % footprintRing.size() );
        const QgsGeometry edge = QgsGeometry::fromPolylineXY( QgsPolylineXY{ a, b } );
        if ( source.distance( edge ) > snapTolerance || aligned.distance( edge ) > snapTolerance )
          continue;

        int support = 0;
        double firstAlong = 1.0;
        double lastAlong = 0.0;
        for ( const QgsPoint &point : reliefPoints )
        {
          const QgsPointXY xy( point.x(), point.y() );
          if ( pointSegmentDistance2( xy, a, b ) > snapTolerance * snapTolerance )
            continue;
          const double along = pointSegmentParameter( xy, a, b );
          firstAlong = std::min( firstAlong, along );
          lastAlong = std::max( lastAlong, along );
          ++support;
        }
        const double edgeLength = std::hypot( b.x() - a.x(), b.y() - a.y() );
        if ( support < minimumSupport || ( lastAlong - firstAlong ) * edgeLength < std::max( 0.10, extent * 0.01 ) )
          continue;

        auto local = [&]( const QgsPointXY &point ) {
          const double dx = point.x() - origin.x();
          const double dy = point.y() - origin.y();
          return QgsPointXY( dx * axisX + dy * axisY, -dx * axisY + dy * axisX );
        };
        const QgsPointXY localA = local( a );
        const QgsPointXY localB = local( b );
        const bool alongU = std::fabs( localB.x() - localA.x() ) >= std::fabs( localB.y() - localA.y() );
        const double axisSpan = alongU ? std::fabs( localB.x() - localA.x() ) : std::fabs( localB.y() - localA.y() );
        if ( axisSpan / edgeLength < 0.9659258262890683 )
          continue;

        double minOffset = std::numeric_limits<double>::max();
        double maxOffset = std::numeric_limits<double>::lowest();
        for ( const QgsPointXY &point : openedRing )
        {
          const QgsPointXY coordinates = local( point );
          const double offset = alongU ? coordinates.y() : coordinates.x();
          minOffset = std::min( minOffset, offset );
          maxOffset = std::max( maxOffset, offset );
        }
        const double edgeOffset = alongU ? ( localA.y() + localB.y() ) * 0.5
                                         : ( localA.x() + localB.x() ) * 0.5;
        const bool moveMinimum = std::fabs( edgeOffset - minOffset ) < std::fabs( edgeOffset - maxOffset );
        const double oldOffset = moveMinimum ? minOffset : maxOffset;
        if ( std::fabs( edgeOffset - oldOffset ) > snapTolerance )
          continue;
        const double target = moveMinimum
                              ? ( alongU ? std::min( localA.y(), localB.y() ) : std::min( localA.x(), localB.x() ) ) - overshoot
                              : ( alongU ? std::max( localA.y(), localB.y() ) : std::max( localA.x(), localB.x() ) ) + overshoot;
        for ( QgsPointXY &point : openedRing )
        {
          QgsPointXY coordinates = local( point );
          const double offset = alongU ? coordinates.y() : coordinates.x();
          if ( std::fabs( offset - oldOffset ) > 1e-6 )
            continue;
          if ( alongU )
            coordinates.setY( target );
          else
            coordinates.setX( target );
          point = QgsPointXY( origin.x() + coordinates.x() * axisX - coordinates.y() * axisY,
                              origin.y() + coordinates.x() * axisY + coordinates.y() * axisX );
        }
      }
      const QgsGeometry extended = polygonGeometryFromRing( openedRing );
      if ( extended.isNull() || !extended.isGeosValid() )
        return QgsGeometry();
      aligned = extended.intersection( footprint );
      if ( aligned.isNull() || aligned.isEmpty() || !aligned.isGeosValid()
           || exteriorRingsFromGeometry( aligned ).size() != 1 )
        return QgsGeometry();
    }

    if ( aligned.difference( footprint ).area() > std::max( 1e-6, aligned.area() * 1e-5 ) )
      return QgsGeometry();
    QgsPolylineXY outline;
    for ( const QgsPointXY &point : footprintRing )
      outline.append( point );
    outline.append( footprintRing.first() );
    const QgsGeometry outlineGeometry = QgsGeometry::fromPolylineXY( outline );
    const double contactLength = aligned.intersection( outlineGeometry ).length();
    if ( boundaryConnected ? contactLength < std::max( 0.10, extent * 0.01 )
                           : aligned.intersects( outlineGeometry ) )
      return QgsGeometry();
    if ( boundaryConnected )
    {
      for ( int i = 0; i < footprintRing.size(); ++i )
      {
        const QgsGeometry edge = QgsGeometry::fromPolylineXY(
          QgsPolylineXY{ footprintRing.at( i ), footprintRing.at( ( i + 1 ) % footprintRing.size() ) } );
        if ( aligned.intersection( edge ).length() > 1e-8 && source.distance( edge ) > snapTolerance )
          return QgsGeometry();
      }
    }
    return aligned;
  }

  QgsGeometry reliefGeometryFromPoints( const HorizontalRoofRegion &region, const QgsGeometry &footprint, double extent,
                                        bool preserveHoles = false )
  {
    if ( region.points.size() < 12 || footprint.isNull() || footprint.isEmpty() )
      return QgsGeometry();

    QVector<ClusterSample> spacingSamples;
    const int spacingStride = std::max( 1, static_cast<int>( std::ceil( region.points.size() / 600.0 ) ) );
    for ( int index = 0; index < region.points.size(); index += spacingStride )
    {
      const QgsPoint &point = region.points.at( index );
      spacingSamples.append( ClusterSample{ QgsPointXY( point.x(), point.y() ), 0.0, 0.0, -1, false } );
    }
    const double fallbackSpacing = std::max( 0.20, std::min( 0.75, extent * 0.025 ) );
    const double spacing = estimateDbscanEps( spacingSamples, fallbackSpacing );
    const double padding = preserveHoles ? std::max( 0.15, std::min( 0.80, region.clusterRadius * 0.55 ) )
                                         : std::min( std::max( 0.45, extent * 0.04 ), std::max( fallbackSpacing, spacing * 0.65 ) );
    const double minimumEdgeLength = std::max( 0.20, std::min( 1.50, extent * 0.025 ) );

    QgsMultiPointXY points;
    std::set<std::pair<int, int>> occupiedCells;
    const QgsRectangle bounds = footprint.boundingBox();
    const double cellSize = std::max( 0.05, padding * 0.25 );
    for ( const QgsPoint &point : region.points )
    {
      const int x = static_cast<int>( std::floor( ( point.x() - bounds.xMinimum() ) / cellSize ) );
      const int y = static_cast<int>( std::floor( ( point.y() - bounds.yMinimum() ) / cellSize ) );
      if ( occupiedCells.insert( std::make_pair( x, y ) ).second )
        points.append( QgsPointXY( point.x(), point.y() ) );
    }
    if ( points.size() < 3 )
      return QgsGeometry();

    QgsGeometry buffered = QgsGeometry::fromMultiPointXY( points ).buffer( padding, 2 );
    if ( buffered.isNull() || buffered.isEmpty() )
      return QgsGeometry();
    const QVector<QgsPointXY> footprintRing = exteriorRing( firstPolygon( footprint ) );
    if ( !preserveHoles )
    {
      const QgsGeometry simplified = buffered.simplify( minimumEdgeLength * 0.75 );
      if ( !simplified.isNull() && !simplified.isEmpty() && simplified.isGeosValid()
           && simplified.area() >= buffered.area() * 0.70 )
        buffered = simplified;
    }

    if ( !preserveHoles )
    {
      const QgsGeometry aligned = alignReliefGeometryToFootprint( buffered, footprintRing, minimumEdgeLength,
                                                                  std::max( 0.15, padding * 0.80 ) );
      if ( !aligned.isNull() && !aligned.isEmpty() && aligned.isGeosValid() )
      {
        const QgsGeometry overlap = aligned.intersection( buffered );
        const double unionArea = aligned.area() + buffered.area() - overlap.area();
        if ( unionArea > 1e-8 && overlap.area() / unionArea >= 0.60 )
          buffered = aligned;
      }
    }

    const QgsGeometry clipped = buffered.intersection( footprint );
    if ( clipped.isNull() || clipped.isEmpty() )
      return QgsGeometry();
    if ( preserveHoles )
      return clipped;

    QVector<QgsGeometry> parts;
    auto appendFilledPart = [&parts]( const QgsPolygonXY &polygon ) {
      if ( polygon.isEmpty() )
        return;
      QgsPolygonXY filledPolygon;
      filledPolygon.append( polygon.first() );
      parts.append( QgsGeometry::fromPolygonXY( filledPolygon ) );
    };
    const QgsPolygonXY polygon = clipped.asPolygon();
    if ( !polygon.isEmpty() )
      appendFilledPart( polygon );
    else
    {
      for ( const QgsPolygonXY &part : clipped.asMultiPolygon() )
        appendFilledPart( part );
      if ( parts.isEmpty() )
      {
        for ( const QgsGeometry &part : clipped.asGeometryCollection() )
          appendFilledPart( part.asPolygon() );
      }
    }
    if ( parts.isEmpty() )
      return QgsGeometry();
    return parts.size() == 1 ? parts.first() : QgsGeometry::unaryUnion( parts );
  }

  struct FlatReliefBoundaryCoverage
  {
    double fraction = 0.0;
    int nearPointCount = 0;
  };

  FlatReliefBoundaryCoverage flatReliefBoundaryCoverage( const QVector<QgsPointXY> &ring,
                                                         const QVector<QgsPoint> &points, double tolerance )
  {
    FlatReliefBoundaryCoverage result;
    if ( ring.size() < 3 || points.isEmpty() )
      return result;

    QVector<double> edgeOffsets;
    edgeOffsets.reserve( ring.size() + 1 );
    edgeOffsets.append( 0.0 );
    for ( int edge = 0; edge < ring.size(); ++edge )
      edgeOffsets.append( edgeOffsets.last() + std::hypot( ring.at( ( edge + 1 ) % ring.size() ).x() - ring.at( edge ).x(),
                                                        ring.at( ( edge + 1 ) % ring.size() ).y() - ring.at( edge ).y() ) );
    const double perimeter = edgeOffsets.last();
    if ( perimeter <= 1e-8 )
      return result;

    const int binCount = 64;
    QVector<unsigned char> covered( binCount, 0 );
    const double tolerance2 = tolerance * tolerance;
    for ( const QgsPoint &point : points )
    {
      const QgsPointXY xy( point.x(), point.y() );
      double bestDistance2 = tolerance2;
      int nearestEdge = -1;
      for ( int edge = 0; edge < ring.size(); ++edge )
      {
        if ( edgeOffsets.at( edge + 1 ) - edgeOffsets.at( edge ) <= 1e-8 )
          continue;
        const double distance2 = pointSegmentDistance2( xy, ring.at( edge ), ring.at( ( edge + 1 ) % ring.size() ) );
        if ( distance2 <= bestDistance2 )
        {
          bestDistance2 = distance2;
          nearestEdge = edge;
        }
      }
      if ( nearestEdge < 0 )
        continue;
      ++result.nearPointCount;
      const double along = std::max( 0.0, std::min( 1.0, pointSegmentParameter( xy, ring.at( nearestEdge ),
                                                                               ring.at( ( nearestEdge + 1 ) % ring.size() ) ) ) );
      const double position = edgeOffsets.at( nearestEdge )
                              + along * ( edgeOffsets.at( nearestEdge + 1 ) - edgeOffsets.at( nearestEdge ) );
      covered[std::min( binCount - 1, static_cast<int>( position / perimeter * binCount ) )] = 1;
    }
    int coveredBins = 0;
    for ( unsigned char occupied : covered )
      coveredBins += occupied != 0;
    result.fraction = static_cast<double>( coveredBins ) / binCount;
    return result;
  }

  bool flatReliefSurfaceMostlyConnected( const QgsGeometry &surface )
  {
    if ( !surface.asPolygon().isEmpty() )
      return true;
    const QgsMultiPolygonXY parts = surface.asMultiPolygon();
    if ( parts.isEmpty() || surface.area() <= 1e-8 )
      return false;
    double largestArea = 0.0;
    for ( const QgsPolygonXY &part : parts )
      largestArea = std::max( largestArea, QgsGeometry::fromPolygonXY( part ).area() );
    return largestArea >= surface.area() * 0.75;
  }

  bool flatReliefOpenRecessSupported( const QgsGeometry &highSurface, const QgsGeometry &lowSurface,
                                     const QgsGeometry &outline, double tolerance, double minimumOpening )
  {
    const double openingLength = lowSurface.buffer( tolerance, 1 ).intersection( outline ).length();
    if ( openingLength < minimumOpening )
      return false;
    const QgsGeometry outlineBand = outline.buffer( std::max( 1e-6, tolerance * 0.1 ), 1 );
    const QgsGeometry highBand = highSurface.buffer( tolerance, 1 );
    double interiorLength = 0.0;
    double adjacentLength = 0.0;
    for ( const QVector<QgsPointXY> &ring : exteriorRingsFromGeometry( lowSurface ) )
    {
      QgsPolylineXY closedRing;
      for ( const QgsPointXY &point : ring )
        closedRing.append( point );
      closedRing.append( ring.first() );
      const QgsGeometry interiorBoundary = QgsGeometry::fromPolylineXY( closedRing ).difference( outlineBand );
      interiorLength += interiorBoundary.length();
      adjacentLength += interiorBoundary.intersection( highBand ).length();
    }
    if ( interiorLength < minimumOpening )
      return false;
    return adjacentLength >= interiorLength * 0.55;
  }

  bool flatReliefHasSupportedOpening( const QVector<QgsPointXY> &ring, const HorizontalRoofRegion &high,
                                      const HorizontalRoofRegion &low, double tolerance )
  {
    const int minimumSupport = std::max( 4, static_cast<int>( std::ceil( low.points.size() * 0.005 ) ) );
    const double minimumOpening = std::max( 0.10, ringExtentSize( ring ) * 0.01 );
    for ( int edge = 0; edge < ring.size(); ++edge )
    {
      const QgsPointXY &a = ring.at( edge );
      const QgsPointXY &b = ring.at( ( edge + 1 ) % ring.size() );
      const double length = std::hypot( b.x() - a.x(), b.y() - a.y() );
      if ( length <= 1e-8 )
        continue;
      QVector<double> along;
      QVector<double> distances;
      for ( const QgsPoint &point : low.points )
      {
        const QgsPointXY xy( point.x(), point.y() );
        const double distance2 = pointSegmentDistance2( xy, a, b );
        if ( distance2 <= tolerance * tolerance )
        {
          along.append( pointSegmentParameter( xy, a, b ) );
          distances.append( std::sqrt( distance2 ) );
        }
      }
      if ( along.size() < minimumSupport )
        continue;
      std::sort( along.begin(), along.end() );
      std::sort( distances.begin(), distances.end() );
      const double start = sortedPercentile( along, 0.05 );
      const double end = sortedPercentile( along, 0.95 );
      if ( ( end - start ) * length < minimumOpening )
        continue;

      // An observed strip of high-plane points between the low plane and the outline closes the recess.
      const double barrierDepth = std::max( tolerance * 0.25, sortedPercentile( distances, 0.10 ) );
      const double openingLength = ( end - start ) * length;
      const double neighborhood = std::max( high.clusterRadius, low.clusterRadius );
      const double binWidth = std::max( 0.05, neighborhood * 0.50 );
      const int binCount = std::max( 4, std::min( 128, static_cast<int>( std::ceil( openingLength / binWidth ) ) ) );
      QVector<unsigned char> barrierBins( binCount, 0 );
      for ( const QgsPoint &point : high.points )
      {
        const QgsPointXY xy( point.x(), point.y() );
        const double t = pointSegmentParameter( xy, a, b );
        if ( t >= start && t <= end && pointSegmentDistance2( xy, a, b ) <= barrierDepth * barrierDepth )
        {
          const int bin = std::max( 0, std::min( binCount - 1, static_cast<int>( ( t - start ) / ( end - start ) * binCount ) ) );
          barrierBins[bin] = 1;
        }
      }
      int coveredBins = 0;
      int gap = 0;
      int largestGap = 0;
      for ( unsigned char covered : barrierBins )
      {
        coveredBins += covered != 0;
        gap = covered ? 0 : gap + 1;
        largestGap = std::max( largestGap, gap );
      }
      // Points at the two ends of a U-shaped roof are not a continuous strip across its opening.
      if ( coveredBins < binCount * 0.75
           || largestGap * openingLength / binCount >= std::max( minimumOpening, neighborhood * 1.50 ) )
        return true;
    }
    return false;
  }

  QVector<FlatReliefRegions> classifyFlatReliefByHighSurface( const QVector<QgsPointXY> &ring,
                                                            const QVector<HorizontalRoofRegion> &regions )
  {
    QVector<FlatReliefRegions> candidates;
    if ( ring.size() < 3 || regions.size() < 2 )
      return candidates;
    const QgsGeometry footprint = polygonGeometryFromRing( ring );
    if ( footprint.area() <= 1e-8 )
      return candidates;
    const double extent = ringExtentSize( ring );
    const double boundaryTolerance = std::max( 0.20, std::min( 1.0, extent * 0.04 ) );
    QgsPolylineXY outline;
    for ( const QgsPointXY &point : ring )
      outline.append( point );
    outline.append( ring.first() );
    const QgsGeometry outlineGeometry = QgsGeometry::fromPolylineXY( outline );

    QVector<QgsGeometry> surfacesWithHoles;
    QVector<QgsGeometry> filledSurfaces;
    QVector<FlatReliefBoundaryCoverage> boundaryCoverage;
    for ( const HorizontalRoofRegion &region : regions )
    {
      surfacesWithHoles.append( reliefGeometryFromPoints( region, footprint, extent, true ) );
      filledSurfaces.append( reliefGeometryFromPoints( region, footprint, extent ) );
      boundaryCoverage.append( flatReliefBoundaryCoverage( ring, region.points, boundaryTolerance ) );
    }

    auto consider = [&]( const HorizontalRoofRegion &outer, const HorizontalRoofRegion &inner,
                         const QgsGeometry &geometry, bool boundaryConnected, double score ) {
      FlatReliefRegions candidate;
      candidate.success = true;
      candidate.boundaryConnected = boundaryConnected;
      candidate.outer = outer;
      candidate.inner = inner;
      candidate.initialReliefGeometry = geometry;
      candidate.score = score;
      candidates.append( candidate );
    };

    for ( int highIndex = 0; highIndex < regions.size(); ++highIndex )
    {
      const HorizontalRoofRegion &high = regions.at( highIndex );
      const QgsGeometry &highSurface = surfacesWithHoles.at( highIndex );
      const QgsGeometry &raisedSurface = filledSurfaces.at( highIndex );
      const FlatReliefBoundaryCoverage &highBoundary = boundaryCoverage.at( highIndex );
      if ( highSurface.isNull() || highSurface.isEmpty() || raisedSurface.isNull() || raisedSurface.isEmpty() )
        continue;

      const QgsGeometry outsideHighSurface = footprint.difference( highSurface );
      const bool highIsInterior = raisedSurface.distance( outlineGeometry ) > boundaryTolerance;

      for ( int lowIndex = 0; lowIndex < regions.size(); ++lowIndex )
      {
        if ( lowIndex == highIndex )
          continue;
        const HorizontalRoofRegion &low = regions.at( lowIndex );
        if ( high.height - low.height <= 0.20 )
          continue;
        const QgsGeometry &lowSurface = filledSurfaces.at( lowIndex );
        const FlatReliefBoundaryCoverage &lowBoundary = boundaryCoverage.at( lowIndex );
        if ( lowSurface.isNull() || lowSurface.isEmpty() || lowSurface.area() <= 1e-8 )
          continue;

        const bool highIsOuter = flatReliefSurfaceMostlyConnected( highSurface )
                                 && ( ( highBoundary.fraction >= 0.22 && highBoundary.fraction >= lowBoundary.fraction + 0.08 )
                                      || ( high.areaRatio >= 0.40 && highBoundary.fraction >= 0.12
                                           && highBoundary.fraction >= lowBoundary.fraction + 0.04 ) );
        const bool lowIsOuter = flatReliefSurfaceMostlyConnected( lowSurface )
                                && ( ( lowBoundary.fraction >= 0.22 && lowBoundary.fraction >= highBoundary.fraction + 0.08 )
                                     || ( low.areaRatio >= 0.40 && lowBoundary.fraction >= 0.12
                                          && lowBoundary.fraction >= highBoundary.fraction + 0.04 ) );
        const bool lowTouchesBoundary = lowBoundary.nearPointCount >= std::max( 4, static_cast<int>( std::ceil( low.points.size() * 0.005 ) ) )
                                        && flatReliefSurfaceMostlyConnected( lowSurface )
                                        && flatReliefHasSupportedOpening( ring, high, low, boundaryTolerance );
        const double overlapArea = highSurface.intersection( lowSurface ).area();
        const double pairCoverage = std::min( 1.0, ( highSurface.area() + lowSurface.area() - overlapArea ) / footprint.area() );
        const double overlapFraction = overlapArea / std::max( 1e-8, std::min( highSurface.area(), lowSurface.area() ) );
        const double areaSupport = pairCoverage * 0.60 - std::min( 1.0, overlapFraction ) * 0.15;

        // The old open-recess fallback is valid only when this pair's high plane is the exterior roof.
        const bool highHasOuterSupport = !highIsInterior && !lowIsOuter && highBoundary.nearPointCount >= 4
                                         && highBoundary.fraction > lowBoundary.fraction;
        if ( highHasOuterSupport && lowTouchesBoundary && high.areaRatio > low.areaRatio * 1.10
             && lowSurface.distance( outlineGeometry ) <= boundaryTolerance )
        {
          const double outsideFraction = lowSurface.intersection( outsideHighSurface ).area() / lowSurface.area();
          if ( outsideFraction >= 0.55 )
          {
            consider( high, low, lowSurface, true,
                      highBoundary.fraction * 0.20 + areaSupport + outsideFraction * 0.20 );
            // Do not suppress raised candidates involving other low planes at the same building.
            continue;
          }
        }

        if ( highIsOuter )
        {
          if ( lowTouchesBoundary )
          {
            const double outsideFraction = lowSurface.intersection( outsideHighSurface ).area() / lowSurface.area();
            int highInsideLow = 0;
            for ( const QgsPoint &point : high.points )
              highInsideLow += lowSurface.intersects( QgsGeometry::fromPointXY( QgsPointXY( point.x(), point.y() ) ) );
            if ( highInsideLow <= high.points.size() * 0.25
                 && flatReliefOpenRecessSupported( highSurface, lowSurface, outlineGeometry, boundaryTolerance,
                                                  std::max( 0.10, extent * 0.01 ) ) )
              consider( high, low, lowSurface, true,
                        highBoundary.fraction * 0.20 + areaSupport + outsideFraction * 0.20 );
          }
          else
          {
            QVector<QgsPolygonXY> surfaceParts;
            const QgsPolygonXY singlePart = highSurface.asPolygon();
            if ( !singlePart.isEmpty() )
              surfaceParts.append( singlePart );
            else
              surfaceParts = highSurface.asMultiPolygon();
            QVector<QgsGeometry> supportedHoles;
            for ( const QgsPolygonXY &part : surfaceParts )
            {
              for ( int holeIndex = 1; holeIndex < part.size(); ++holeIndex )
              {
                const QgsPolylineXY &holeRing = part.at( holeIndex );
                QgsPolygonXY holePolygon;
                holePolygon.append( holeRing );
                const QgsGeometry hole = QgsGeometry::fromPolygonXY( holePolygon ).intersection( footprint );
                if ( hole.isNull() || hole.isEmpty() || hole.area() <= 1e-8 )
                  continue;
                int support = 0;
                for ( const QgsPoint &point : low.points )
                  support += pointInRing( holeRing, QgsPointXY( point.x(), point.y() ) );
                if ( support > 0 && lowSurface.intersection( hole ).area() > 1e-8 )
                  supportedHoles.append( hole );
                const double supportFraction = static_cast<double>( support ) / low.points.size();
                if ( support < 12 || supportFraction < 0.60
                     || lowSurface.intersection( hole ).area() / lowSurface.area() < 0.55 )
                  continue;
                consider( high, low, hole, false,
                          highBoundary.fraction * 0.20 + areaSupport + supportFraction * 0.20 );
              }
            }
            if ( supportedHoles.size() > 1 )
            {
              const QgsGeometry holes = QgsGeometry::unaryUnion( supportedHoles );
              int support = 0;
              for ( const QgsPoint &point : low.points )
                support += holes.intersects( QgsGeometry::fromPointXY( QgsPointXY( point.x(), point.y() ) ) );
              const double supportFraction = static_cast<double>( support ) / low.points.size();
              if ( support >= 12 && supportFraction >= 0.60 && lowSurface.intersection( holes ).area() / lowSurface.area() >= 0.55 )
                consider( high, low, holes, false,
                          highBoundary.fraction * 0.20 + areaSupport + supportFraction * 0.20 );
            }
          }
        }

        // An interior upper face must still be tried when the surrounding lower face has sparse boundary samples.
        if ( lowIsOuter || highIsInterior )
        {
          int outsideSupport = 0;
          for ( const QgsPoint &point : low.points )
            outsideSupport += !raisedSurface.intersects( QgsGeometry::fromPointXY( QgsPointXY( point.x(), point.y() ) ) );
          const double outsideFraction = static_cast<double>( outsideSupport ) / low.points.size();
          if ( outsideSupport < 12 || outsideFraction < 0.60 )
            continue;
          consider( low, high, raisedSurface, false,
                    lowBoundary.fraction * 0.20 + areaSupport + outsideFraction * 0.20 );
        }
      }
    }
    std::stable_sort( candidates.begin(), candidates.end(), []( const FlatReliefRegions &a, const FlatReliefRegions &b ) {
      return a.score > b.score;
    } );
    return candidates;
  }

  QgsGeometry refineReliefBoundaryFromPlanes( const QgsGeometry &source, const QgsGeometry &footprint,
                                               const QVector<QgsPoint> &reliefPoints, const QVector<QgsPoint> &mainPoints,
                                               double extent )
  {
    if ( source.isNull() || source.isEmpty() || reliefPoints.size() < 12 || mainPoints.size() < 12 )
      return source;

    auto sampleSpacing = []( const QVector<QgsPoint> &points ) {
      QVector<ClusterSample> samples;
      const int stride = std::max( 1, static_cast<int>( std::ceil( points.size() / 600.0 ) ) );
      for ( int i = 0; i < points.size(); i += stride )
        samples.append( ClusterSample{ QgsPointXY( points.at( i ).x(), points.at( i ).y() ), 0.0, 0.0, -1, false } );
      return estimateDbscanEps( samples, 0.0 );
    };
    const double spacing = std::max( sampleSpacing( reliefPoints ), sampleSpacing( mainPoints ) );
    if ( spacing <= 1e-8 )
      return source;
    const double searchDistance = std::min( extent * 0.25, spacing * 2.0 );
    const QVector<QgsPointXY> footprintRing = exteriorRing( firstPolygon( footprint ) );

    QVector<QgsPolygonXY> polygons;
    const QgsPolygonXY polygon = source.asPolygon();
    if ( !polygon.isEmpty() )
      polygons.append( polygon );
    else
      polygons = source.asMultiPolygon();

    QVector<QgsGeometry> refinedParts;
    for ( const QgsPolygonXY &polygonPart : polygons )
    {
      const QVector<QgsPointXY> ring = exteriorRing( polygonPart );
      const QgsGeometry originalPart = QgsGeometry::fromPolygonXY( polygonPart );
      if ( ring.size() < 4 || ring.size() > 128 )
      {
        refinedParts.append( originalPart );
        continue;
      }

      double signedArea = 0.0;
      const QgsPointXY &origin = ring.first();
      for ( int i = 0; i < ring.size(); ++i )
      {
        const QgsPointXY &a = ring.at( i );
        const QgsPointXY &b = ring.at( ( i + 1 ) % ring.size() );
        signedArea += ( a.x() - origin.x() ) * ( b.y() - origin.y() )
                      - ( b.x() - origin.x() ) * ( a.y() - origin.y() );
      }
      struct ShiftedEdge
      {
        QgsPointXY start;
        QgsPointXY end;
      };
      QVector<ShiftedEdge> edges;
      bool changed = false;
      for ( int i = 0; i < ring.size(); ++i )
      {
        const QgsPointXY &a = ring.at( i );
        const QgsPointXY &b = ring.at( ( i + 1 ) % ring.size() );
        const double dx = b.x() - a.x();
        const double dy = b.y() - a.y();
        const double length = std::hypot( dx, dy );
        if ( length <= 1e-8 )
        {
          edges.append( ShiftedEdge{ a, b } );
          continue;
        }
        const double outwardX = signedArea >= 0.0 ? dy / length : -dy / length;
        const double outwardY = signedArea >= 0.0 ? -dx / length : dx / length;
        const QgsPointXY midpoint( ( a.x() + b.x() ) * 0.5, ( a.y() + b.y() ) * 0.5 );
        double shift = 0.0;
        if ( length > spacing && distanceToRing2( footprintRing, midpoint ) > 1e-10 )
        {
          QVector<double> innerDistances;
          QVector<double> outerDistances;
          const double endTrim = std::min( length * 0.12, spacing );
          auto collect = [&]( const QVector<QgsPoint> &points, QVector<double> &distances ) {
            for ( const QgsPoint &point : points )
            {
              const double px = point.x() - a.x();
              const double py = point.y() - a.y();
              const double along = ( px * dx + py * dy ) / length;
              if ( along < endTrim || along > length - endTrim )
                continue;
              const double distance = px * outwardX + py * outwardY;
              if ( std::fabs( distance ) <= searchDistance )
                distances.append( distance );
            }
          };
          collect( reliefPoints, innerDistances );
          collect( mainPoints, outerDistances );
          if ( innerDistances.size() >= 4 && outerDistances.size() >= 4 )
          {
            std::sort( innerDistances.begin(), innerDistances.end() );
            std::sort( outerDistances.begin(), outerDistances.end() );
            const double innerEdge = sortedPercentile( innerDistances, 0.90 );
            const double outerEdge = sortedPercentile( outerDistances, 0.10 );
            if ( outerEdge > innerEdge && outerEdge - innerEdge <= spacing * 2.0 )
            {
              shift = ( innerEdge + outerEdge ) * 0.5;
              changed = changed || std::fabs( shift ) > 1e-8;
            }
          }
        }
        edges.append( ShiftedEdge{ QgsPointXY( a.x() + outwardX * shift, a.y() + outwardY * shift ),
                                   QgsPointXY( b.x() + outwardX * shift, b.y() + outwardY * shift ) } );
      }
      if ( !changed )
      {
        refinedParts.append( originalPart );
        continue;
      }

      QVector<QgsPointXY> refinedRing;
      for ( int i = 0; i < edges.size(); ++i )
      {
        const ShiftedEdge &previous = edges.at( ( i - 1 + edges.size() ) % edges.size() );
        const ShiftedEdge &current = edges.at( i );
        const double prevX = previous.end.x() - previous.start.x();
        const double prevY = previous.end.y() - previous.start.y();
        const double currX = current.end.x() - current.start.x();
        const double currY = current.end.y() - current.start.y();
        const double denominator = cross2d( prevX, prevY, currX, currY );
        if ( std::fabs( denominator ) < std::hypot( prevX, prevY ) * std::hypot( currX, currY ) * 0.1 )
          break;
        const double relX = current.start.x() - previous.start.x();
        const double relY = current.start.y() - previous.start.y();
        const double t = cross2d( relX, relY, currX, currY ) / denominator;
        const QgsPointXY vertex( previous.start.x() + prevX * t, previous.start.y() + prevY * t );
        if ( !std::isfinite( vertex.x() ) || !std::isfinite( vertex.y() )
             || std::hypot( vertex.x() - ring.at( i ).x(), vertex.y() - ring.at( i ).y() ) > searchDistance * 2.0 )
          break;
        refinedRing.append( vertex );
      }
      if ( refinedRing.size() != ring.size() )
      {
        refinedParts.append( originalPart );
        continue;
      }
      const QgsGeometry candidate = polygonGeometryFromRing( refinedRing ).intersection( footprint );
      if ( candidate.isNull() || candidate.isEmpty() || !candidate.isGeosValid()
           || exteriorRingsFromGeometry( candidate ).size() != 1 )
      {
        refinedParts.append( originalPart );
        continue;
      }
      bool lostFootprintContact = false;
      for ( int edge = 0; edge < footprintRing.size(); ++edge )
      {
        const QgsGeometry outlineEdge = QgsGeometry::fromPolylineXY(
          QgsPolylineXY{ footprintRing.at( edge ), footprintRing.at( ( edge + 1 ) % footprintRing.size() ) } );
        if ( originalPart.intersection( outlineEdge ).length() > 1e-8
             && candidate.intersection( outlineEdge ).length() <= 1e-8 )
        {
          lostFootprintContact = true;
          break;
        }
      }
      if ( lostFootprintContact )
      {
        refinedParts.append( originalPart );
        continue;
      }
      const QgsGeometry overlap = candidate.intersection( originalPart );
      const double unionArea = candidate.area() + originalPart.area() - overlap.area();
      if ( unionArea > 1e-8 && overlap.area() / unionArea >= 0.60 )
        refinedParts.append( candidate );
      else
        refinedParts.append( originalPart );
    }
    if ( refinedParts.isEmpty() )
      return source;
    const QgsGeometry refined = refinedParts.size() == 1 ? refinedParts.first() : QgsGeometry::unaryUnion( refinedParts );
    int support = 0;
    for ( const QgsPoint &point : reliefPoints )
    {
      if ( refined.intersects( QgsGeometry::fromPointXY( QgsPointXY( point.x(), point.y() ) ) ) )
        ++support;
    }
    return support >= static_cast<int>( std::ceil( reliefPoints.size() * 0.60 ) ) ? refined : source;
  }

  QgsGeometry connectReliefToFootprintConsistently( const QgsGeometry &source, const QgsGeometry &footprint,
                                                     const QVector<QgsPoint> &reliefPoints, double extent )
  {
    if ( source.isNull() || source.isEmpty() )
      return source;
    const QVector<QgsPointXY> footprintRing = exteriorRing( firstPolygon( footprint ) );
    if ( footprintRing.size() < 3 )
      return QgsGeometry();

    QVector<ClusterSample> spacingSamples;
    const int stride = std::max( 1, static_cast<int>( std::ceil( reliefPoints.size() / 600.0 ) ) );
    for ( int i = 0; i < reliefPoints.size(); i += stride )
      spacingSamples.append( ClusterSample{ QgsPointXY( reliefPoints.at( i ).x(), reliefPoints.at( i ).y() ), 0.0, 0.0, -1, false } );
    const double spacing = estimateDbscanEps( spacingSamples, 0.0 );
    if ( spacing <= 1e-8 )
      return source;
    const double contactDepth = std::min( extent * 0.20, spacing * 2.0 );

    QVector<QgsPolygonXY> polygons;
    const QgsPolygonXY polygon = source.asPolygon();
    if ( !polygon.isEmpty() )
      polygons.append( polygon );
    else
      polygons = source.asMultiPolygon();
    QVector<QgsGeometry> connectedParts;
    for ( const QgsPolygonXY &polygonPart : polygons )
    {
      QgsGeometry part = QgsGeometry::fromPolygonXY( polygonPart );
      for ( int edge = 0; edge < footprintRing.size(); ++edge )
      {
        const QgsPointXY &a = footprintRing.at( edge );
        const QgsPointXY &b = footprintRing.at( ( edge + 1 ) % footprintRing.size() );
        const double dx = b.x() - a.x();
        const double dy = b.y() - a.y();
        const double length = std::hypot( dx, dy );
        if ( length <= 1e-8 )
          continue;
        const QgsGeometry outlineEdge = QgsGeometry::fromPolylineXY( QgsPolylineXY{ a, b } );
        if ( !part.intersects( outlineEdge ) )
          continue;

        const double axisX = dx / length;
        const double axisY = dy / length;
        const QgsPointXY midpoint( ( a.x() + b.x() ) * 0.5, ( a.y() + b.y() ) * 0.5 );
        const double probe = std::min( spacing * 0.25, length * 0.01 );
        const bool leftIsInside = pointInRing( footprintRing, QgsPointXY( midpoint.x() - axisY * probe,
                                                                           midpoint.y() + axisX * probe ) );
        const double inwardX = leftIsInside ? -axisY : axisY;
        const double inwardY = leftIsInside ? axisX : -axisX;
        double minAlong = length;
        double maxAlong = 0.0;
        const QVector<QgsPointXY> partRing = exteriorRing( firstPolygon( part ) );
        for ( const QgsPointXY &point : partRing )
        {
          const double px = point.x() - a.x();
          const double py = point.y() - a.y();
          const double along = px * axisX + py * axisY;
          const double inward = px * inwardX + py * inwardY;
          if ( along < -spacing || along > length + spacing || inward < -1e-7 || inward > contactDepth )
            continue;
          minAlong = std::min( minAlong, std::max( 0.0, along ) );
          maxAlong = std::max( maxAlong, std::min( length, along ) );
        }
        if ( maxAlong - minAlong <= 1e-8 )
          return QgsGeometry();

        const QVector<QgsPointXY> opening{
          QgsPointXY( a.x() + axisX * minAlong, a.y() + axisY * minAlong ),
          QgsPointXY( a.x() + axisX * maxAlong, a.y() + axisY * maxAlong ),
          QgsPointXY( a.x() + axisX * maxAlong + inwardX * contactDepth,
                      a.y() + axisY * maxAlong + inwardY * contactDepth ),
          QgsPointXY( a.x() + axisX * minAlong + inwardX * contactDepth,
                      a.y() + axisY * minAlong + inwardY * contactDepth ) };
        const QgsGeometry bridge = polygonGeometryFromRing( opening ).intersection( footprint );
        const QgsGeometry connected = part.combine( bridge );
        if ( connected.isNull() || connected.isEmpty() || !connected.isGeosValid()
             || exteriorRingsFromGeometry( connected ).size() != 1 )
          return QgsGeometry();
        const QgsGeometry contact = connected.intersection( outlineEdge );
        if ( contact.length() < ( maxAlong - minAlong ) * 0.95 )
          return QgsGeometry();
        part = connected;
      }
      connectedParts.append( part );
    }
    if ( connectedParts.isEmpty() )
      return QgsGeometry();
    return connectedParts.size() == 1 ? connectedParts.first() : QgsGeometry::unaryUnion( connectedParts );
  }

  struct FrustumSideSample
  {
    QgsPoint point;
    double inwardDistance = 0.0;
    double alongEdge = 0.0;
  };

  struct FittedFlatTopHippedRoof
  {
    bool success = false;
    double eaveHeight = 0.0;
    double topHeight = 0.0;
    QVector<QgsPointXY> topRing;
    QVector<bool> boundaryContacts = QVector<bool>( 4, false );
  };

  QVector<bool> flatTopBoundaryContacts( const QVector<QgsPointXY> &ring, const HorizontalRoofRegion &topRegion )
  {
    QVector<bool> contacts( 4, false );
    if ( ring.size() != 4 || topRegion.points.size() < 12 )
      return contacts;
    const double tolerance = std::max( 0.10, std::min( 0.50, topRegion.clusterRadius * 0.55 ) );
    for ( int edge = 0; edge < 4; ++edge )
    {
      const QgsPointXY &a = ring.at( edge );
      const QgsPointXY &b = ring.at( ( edge + 1 ) % 4 );
      const double length = std::hypot( b.x() - a.x(), b.y() - a.y() );
      if ( length <= 1e-8 )
        continue;
      const double ux = ( b.x() - a.x() ) / length;
      const double uy = ( b.y() - a.y() ) / length;
      std::set<int> occupied;
      int count = 0;
      double minAlong = length;
      double maxAlong = 0.0;
      for ( const QgsPoint &point : topRegion.points )
      {
        const double distance = cross2d( ux, uy, point.x() - a.x(), point.y() - a.y() );
        const double along = ( point.x() - a.x() ) * ux + ( point.y() - a.y() ) * uy;
        if ( distance >= -1e-7 && distance <= tolerance && along >= 0.0 && along <= length )
        {
          ++count;
          minAlong = std::min( minAlong, along );
          maxAlong = std::max( maxAlong, along );
          occupied.insert( static_cast<int>( std::floor( along / ( tolerance * 2.0 ) ) ) );
        }
      }
      const double coveredBins = std::max( 1.0, ( maxAlong - minAlong ) / ( tolerance * 2.0 ) + 1.0 );
      // Require a supported boundary run, not an isolated point close to the footprint.
      contacts[edge] = count >= 6 && occupied.size() >= 3 && occupied.size() >= coveredBins * 0.50
                       && maxAlong - minAlong >= std::max( 0.30, length * 0.20 );
    }
    return contacts;
  }

  QVector<QVector<FrustumSideSample>> flatTopSlopeCandidates( const QVector<QgsPointXY> &ring,
                                                             const BuildingRoof::RoofPlaneSegmentation &segmentation,
                                                             const HorizontalRoofRegion &topRegion, const QVector<bool> &boundaryContacts )
  {
    QVector<QVector<FrustumSideSample>> sides( 4 );
    if ( ring.size() != 4 || boundaryContacts.size() != 4 || !topRegion.success || topRegion.points.isEmpty() )
      return sides;
    double edgeX[4];
    double edgeY[4];
    double offsets[4];
    double bestScore[4] = { 0.0, 0.0, 0.0, 0.0 };
    const double extent = ringExtentSize( ring );
    const double cellSize = std::max( 0.15, std::min( 0.60, extent * 0.02 ) );
    const double adjacencyTolerance = std::max( 0.25, std::max( topRegion.clusterRadius, extent * 0.01 ) );
    for ( int edge = 0; edge < 4; ++edge )
    {
      const QgsPointXY &a = ring.at( edge );
      const QgsPointXY &b = ring.at( ( edge + 1 ) % 4 );
      const double length = std::hypot( b.x() - a.x(), b.y() - a.y() );
      if ( length <= 1e-8 )
        return sides;
      edgeX[edge] = ( b.x() - a.x() ) / length;
      edgeY[edge] = ( b.y() - a.y() ) / length;
      QVector<double> distances;
      for ( const QgsPoint &point : topRegion.points )
        distances.append( cross2d( edgeX[edge], edgeY[edge], point.x() - a.x(), point.y() - a.y() ) );
      std::sort( distances.begin(), distances.end() );
      offsets[edge] = std::max( 0.01, sortedPercentile( distances, 0.02 ) );
    }
    for ( const BuildingRoof::RoofPlaneSegment &segment : segmentation.segments )
    {
      if ( std::fabs( segment.normalZ ) < 0.20 )
        continue;
      const LeastSquaresRoofPlane plane = fitLeastSquaresRoofPlane( segment.points );
      const double gradient = std::hypot( plane.a, plane.b );
      if ( !plane.success || gradient <= 0.03 || gradient > 5.0 )
        continue;
      int face = -1;
      double alignment = 0.85;
      for ( int edge = 0; edge < 4; ++edge )
      {
        if ( boundaryContacts.at( edge ) )
          continue;
        const double dot = ( -edgeY[edge] * plane.a + edgeX[edge] * plane.b ) / gradient;
        if ( dot > alignment )
        {
          alignment = dot;
          face = edge;
        }
      }
      if ( face < 0 )
        continue;
      double closestTopDistance = std::numeric_limits<double>::max();
      for ( const QgsPoint &point : topRegion.points )
        closestTopDistance = std::min( closestTopDistance, roofPlaneResidual( plane, point ) / gradient );
      if ( closestTopDistance > adjacencyTolerance )
        continue;
      QVector<FrustumSideSample> candidates;
      QVector<QgsPoint> supportedPoints;
      for ( const QgsPoint &point : segment.points )
      {
        if ( point.z() >= topRegion.height - 0.08 )
          continue;
        int nearestEdge = -1;
        double fraction = std::numeric_limits<double>::max();
        double distance = 0.0;
        for ( int edge = 0; edge < 4; ++edge )
        {
          if ( boundaryContacts.at( edge ) )
            continue;
          const double d = cross2d( edgeX[edge], edgeY[edge], point.x() - ring.at( edge ).x(), point.y() - ring.at( edge ).y() );
          const double progress = d / offsets[edge];
          if ( progress < fraction )
          {
            fraction = progress;
            nearestEdge = edge;
            distance = d;
          }
        }
        if ( nearestEdge != face || distance <= std::max( 0.02, extent * 0.001 ) || fraction > 1.10 )
          continue;
        const double along = ( point.x() - ring.at( face ).x() ) * edgeX[face] + ( point.y() - ring.at( face ).y() ) * edgeY[face];
        candidates.append( FrustumSideSample{ point, distance, along } );
        supportedPoints.append( point );
      }
      if ( candidates.size() < 8 )
        continue;
      const double area = pointSetProjectedArea( supportedPoints, cellSize, ring.first().x(), ring.first().y() );
      const double score = area * alignment / ( 1.0 + closestTopDistance / adjacencyTolerance ) / ( 1.0 + plane.rmse / 0.15 );
      if ( score > bestScore[face] )
      {
        bestScore[face] = score;
        sides[face] = candidates;
      }
    }
    return sides;
  }

  QVector<QgsPointXY> frustumFootprintRing( QVector<QgsPointXY> ring )
  {
    const double tolerance = std::max( 1e-7, ringExtentSize( ring ) * 1e-7 );
    bool changed = true;
    while ( changed && ring.size() > 4 )
    {
      changed = false;
      for ( int i = 0; i < ring.size(); ++i )
      {
        if ( pointSegmentDistance2( ring.at( i ), ring.at( ( i - 1 + ring.size() ) % ring.size() ),
                                   ring.at( ( i + 1 ) % ring.size() ) ) <= tolerance * tolerance )
        {
          ring.removeAt( i );
          changed = true;
          break;
        }
      }
    }
    return ring;
  }

  QVector<QgsPointXY> flatTopRingFromOffsets( const QVector<QgsPointXY> &ring, const QVector<double> &offsets )
  {
    QVector<QgsPointXY> topRing;
    if ( ring.size() != 4 || offsets.size() != 4 )
      return topRing;
    for ( int vertex = 0; vertex < 4; ++vertex )
    {
      const int previous = ( vertex + 3 ) % 4;
      const QgsPointXY &a = ring.at( previous );
      const QgsPointXY &b = ring.at( vertex );
      const QgsPointXY &c = ring.at( ( vertex + 1 ) % 4 );
      const double previousLength = std::hypot( b.x() - a.x(), b.y() - a.y() );
      const double currentLength = std::hypot( c.x() - b.x(), c.y() - b.y() );
      if ( previousLength <= 1e-8 || currentLength <= 1e-8 )
        return {};
      const double prevX = ( b.x() - a.x() ) / previousLength;
      const double prevY = ( b.y() - a.y() ) / previousLength;
      const double currX = ( c.x() - b.x() ) / currentLength;
      const double currY = ( c.y() - b.y() ) / currentLength;
      const double denominator = cross2d( prevX, prevY, currX, currY );
      if ( std::fabs( denominator ) <= 1e-8 )
        return {};
      const QgsPointXY prevOrigin( a.x() - prevY * offsets.at( previous ), a.y() + prevX * offsets.at( previous ) );
      const QgsPointXY currOrigin( b.x() - currY * offsets.at( vertex ), b.y() + currX * offsets.at( vertex ) );
      const double t = cross2d( currOrigin.x() - prevOrigin.x(), currOrigin.y() - prevOrigin.y(), currX, currY ) / denominator;
      const QgsPointXY point( prevOrigin.x() + t * prevX, prevOrigin.y() + t * prevY );
      if ( !std::isfinite( point.x() ) || !std::isfinite( point.y() ) || !pointInRing( ring, point ) )
        return {};
      for ( int edge = 0; edge < 4; ++edge )
      {
        const QgsPointXY &edgeA = ring.at( edge );
        const QgsPointXY &edgeB = ring.at( ( edge + 1 ) % 4 );
        const double length = std::hypot( edgeB.x() - edgeA.x(), edgeB.y() - edgeA.y() );
        if ( length <= 1e-8
             || cross2d( edgeB.x() - edgeA.x(), edgeB.y() - edgeA.y(), point.x() - edgeA.x(), point.y() - edgeA.y() ) / length < offsets.at( edge ) - 1e-7 )
          return {};
      }
      topRing.append( point );
    }
    const QgsGeometry topGeometry = polygonGeometryFromRing( topRing );
    const double footprintArea = polygonGeometryFromRing( ring ).area();
    if ( topGeometry.isNull() || !topGeometry.isGeosValid() || topGeometry.area() <= footprintArea * 0.0001
         || topGeometry.area() >= footprintArea * 0.995 )
      return {};
    return topRing;
  }

  FittedFlatTopHippedRoof fitConstrainedFlatTopRoof( const QVector<QgsPointXY> &ring,
                                                    const QVector<BuildingRoof::RoofSample> &pointCloudSamples,
                                                    const HorizontalRoofRegion &topRegion, double eaveEstimate,
                                                    const QVector<bool> &boundaryContacts )
  {
    FittedFlatTopHippedRoof result;
    if ( ring.size() != 4 || boundaryContacts.size() != 4 || !topRegion.success || topRegion.points.size() < 12 )
      return result;
    double topHeight = 0.0;
    for ( const QgsPoint &point : topRegion.points )
      topHeight += point.z();
    topHeight /= topRegion.points.size();
    if ( topHeight <= eaveEstimate + 0.15 )
      return result;

    double edgeX[4];
    double edgeY[4];
    QVector<double> offsets( 4, 0.0 );
    const double extent = ringExtentSize( ring );
    const double minimumOffset = std::max( 0.01, extent * 0.0005 );
    for ( int edge = 0; edge < 4; ++edge )
    {
      const QgsPointXY &a = ring.at( edge );
      const QgsPointXY &b = ring.at( ( edge + 1 ) % 4 );
      const double length = std::hypot( b.x() - a.x(), b.y() - a.y() );
      if ( length <= 1e-8 )
        return result;
      edgeX[edge] = ( b.x() - a.x() ) / length;
      edgeY[edge] = ( b.y() - a.y() ) / length;
      QVector<double> distances;
      for ( const QgsPoint &point : topRegion.points )
        distances.append( cross2d( edgeX[edge], edgeY[edge], point.x() - a.x(), point.y() - a.y() ) );
      std::sort( distances.begin(), distances.end() );
      offsets[edge] = boundaryContacts.at( edge ) ? 0.0 : std::max( minimumOffset, sortedPercentile( distances, 0.02 ) );
    }
    if ( flatTopRingFromOffsets( ring, offsets ).isEmpty() )
      return result;
    const QVector<double> observedOffsets = offsets;
    const double spacing = std::max( extent * 0.003, std::sqrt( polygonGeometryFromRing( ring ).area() / std::max( 1, pointCloudSamples.size() ) ) );
    const double initialRise = topHeight - eaveEstimate;
    const double delta = std::max( 0.15, std::min( 0.50, initialRise * 0.10 ) );
    struct Sample
    {
      double distances[4];
      double z = 0.0;
      double weight = 1.0;
    };
    QVector<Sample> samples;
    auto appendSample = [&]( const QgsPoint &point, double weight ) {
      if ( !std::isfinite( point.z() ) || point.z() < 0.0
           || !pointInRing( ring, QgsPointXY( point.x(), point.y() ) ) )
        return;
      Sample sample;
      sample.z = point.z();
      sample.weight = weight;
      for ( int edge = 0; edge < 4; ++edge )
        sample.distances[edge] = cross2d( edgeX[edge], edgeY[edge], point.x() - ring.at( edge ).x(), point.y() - ring.at( edge ).y() );
      samples.append( sample );
    };
    for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
      appendSample( sample.point, 1.0 );
    for ( const QgsPoint &point : topRegion.points )
      appendSample( point, 2.0 );
    auto fractionAt = []( const Sample &sample, const QVector<double> &insets ) {
      double fraction = 1.0;
      for ( int edge = 0; edge < 4; ++edge )
        if ( insets.at( edge ) > 1e-8 )
          fraction = std::min( fraction, sample.distances[edge] / insets.at( edge ) );
      return std::max( 0.0, fraction );
    };
    double eaveHeight = eaveEstimate;
    auto fitEaveHeight = [&]() {
      for ( int iteration = 0; iteration < 4; ++iteration )
      {
        double denominator = 0.0;
        double numerator = 0.0;
        int slopeSupport = 0;
        for ( const Sample &sample : samples )
        {
          const double fraction = fractionAt( sample, offsets );
          if ( fraction <= 0.03 || fraction >= 0.97 )
            continue;
          const double q = 1.0 - fraction;
          const double residual = sample.z - topHeight * fraction - eaveHeight * q;
          const double weight = sample.weight * std::min( 1.0, delta / std::max( 1e-8, std::fabs( residual ) ) );
          denominator += weight * q * q;
          numerator += weight * q * ( sample.z - topHeight * fraction );
          ++slopeSupport;
        }
        if ( denominator <= 1e-8 || slopeSupport < 8 )
          return false;
        eaveHeight = numerator / denominator;
        if ( !std::isfinite( eaveHeight ) || eaveHeight <= 0.0 || eaveHeight >= topHeight - 0.15 )
          return false;
      }
      return true;
    };
    auto lossAt = [&]( const QVector<double> &insets ) {
      double loss = 0.0;
      double weightSum = 0.0;
      for ( const Sample &sample : samples )
      {
        const double fraction = fractionAt( sample, insets );
        const double residual = std::fabs( sample.z - eaveHeight - ( topHeight - eaveHeight ) * fraction );
        loss += sample.weight * ( residual <= delta ? residual * residual : delta * ( 2.0 * residual - delta ) );
        weightSum += sample.weight;
      }
      if ( weightSum <= 0.0 )
        return std::numeric_limits<double>::max();
      loss /= weightSum;
      for ( int edge = 0; edge < 4; ++edge )
      {
        if ( boundaryContacts.at( edge ) )
          continue;
        const double change = ( insets.at( edge ) - observedOffsets.at( edge ) ) / std::max( spacing, observedOffsets.at( edge ) );
        loss += delta * delta * 0.01 * change * change;
      }
      return loss;
    };

    // Only real slopes participate; boundary-connected upper edges keep an exact zero inset.
    for ( int iteration = 0; iteration < 8; ++iteration )
    {
      if ( !fitEaveHeight() )
        return result;
      for ( int edge = 0; edge < 4; ++edge )
      {
        if ( boundaryContacts.at( edge ) )
          continue;
        const double step = std::max( spacing, observedOffsets.at( edge ) * 0.20 ) * std::pow( 0.55, iteration / 2 );
        const double limit = std::max( spacing * 3.0, observedOffsets.at( edge ) * 0.50 );
        const double current = offsets.at( edge );
        double best = lossAt( offsets );
        QVector<double> bestOffsets = offsets;
        for ( double direction : { -1.0, 1.0 } )
        {
          QVector<double> candidate = offsets;
          candidate[edge] = std::max( minimumOffset, current + direction * step );
          if ( std::fabs( candidate.at( edge ) - observedOffsets.at( edge ) ) > limit
               || flatTopRingFromOffsets( ring, candidate ).isEmpty() )
            continue;
          const double loss = lossAt( candidate );
          if ( loss < best )
          {
            best = loss;
            bestOffsets = candidate;
          }
        }
        offsets = bestOffsets;
      }
    }
    if ( !fitEaveHeight() )
      return result;
    int slopeSupport = 0;
    for ( const Sample &sample : samples )
    {
      const double fraction = fractionAt( sample, offsets );
      if ( fraction > 0.03 && fraction < 0.97
           && std::fabs( sample.z - eaveHeight - ( topHeight - eaveHeight ) * fraction ) <= delta * 2.0 )
        ++slopeSupport;
    }
    if ( slopeSupport < 8 )
      return result;
    result.topRing = flatTopRingFromOffsets( ring, offsets );
    if ( result.topRing.isEmpty() )
      return result;
    result.eaveHeight = eaveHeight;
    result.topHeight = topHeight;
    result.boundaryContacts = boundaryContacts;
    result.success = true;
    return result;
  }

  FittedFlatTopHippedRoof fitFivePlaneFlatTopRoof( const QVector<QgsPointXY> &ring, const QVector<QVector<FrustumSideSample>> &sideCandidates,
                                                   const HorizontalRoofRegion &topRegion, double eaveEstimate,
                                                   const QVector<bool> &boundaryContacts )
  {
    FittedFlatTopHippedRoof result;
    if ( ring.size() != 4 || sideCandidates.size() != 4 || boundaryContacts.size() != 4
         || !topRegion.success || topRegion.points.size() < 12 )
      return result;

    double topHeightSum = 0.0;
    for ( const QgsPoint &point : topRegion.points )
      topHeightSum += point.z();
    const double topHeight = topHeightSum / topRegion.points.size();
    const double extent = ringExtentSize( ring );
    const double roughRise = std::max( 0.30, topHeight - eaveEstimate );
    const double residualThreshold = std::max( 0.15, std::min( 0.40, roughRise * 0.10 ) );

    QVector<QVector<FrustumSideSample>> sideInliers( 4 );
    double sumD[4] = { 0.0, 0.0, 0.0, 0.0 };
    double sumD2[4] = { 0.0, 0.0, 0.0, 0.0 };
    double sumDZ[4] = { 0.0, 0.0, 0.0, 0.0 };
    double sumZ = 0.0;
    double totalWeight = 0.0;
    for ( int edge = 0; edge < 4; ++edge )
    {
      if ( boundaryContacts.at( edge ) )
        continue;
      const QVector<FrustumSideSample> &candidates = sideCandidates.at( edge );
      if ( candidates.size() < 8 )
        return result;
      // Jointly solve the supported RANSAC slopes with a shared eave level.
      const double weight = 1.0 / candidates.size();
      for ( const FrustumSideSample &sample : candidates )
      {
        sideInliers[edge].append( sample );
        const double d = sample.inwardDistance;
        sumD[edge] += weight * d;
        sumD2[edge] += weight * d * d;
        sumDZ[edge] += weight * d * sample.point.z();
        sumZ += weight * sample.point.z();
        totalWeight += weight;
      }
      if ( sumD2[edge] <= 1e-8 )
        return result;
    }

    double denominator = totalWeight;
    double numerator = sumZ;
    for ( int edge = 0; edge < 4; ++edge )
    {
      if ( boundaryContacts.at( edge ) )
        continue;
      denominator -= sumD[edge] * sumD[edge] / sumD2[edge];
      numerator -= sumD[edge] * sumDZ[edge] / sumD2[edge];
    }
    if ( denominator <= 1e-6 )
      return result;
    const double eaveHeight = numerator / denominator;
    if ( !std::isfinite( eaveHeight ) || eaveHeight <= 0.0 || eaveHeight >= topHeight - 0.20 )
      return result;

    QVector<double> offsets( 4, 0.0 );
    for ( int edge = 0; edge < 4; ++edge )
    {
      if ( boundaryContacts.at( edge ) )
        continue;
      const double slope = ( sumDZ[edge] - eaveHeight * sumD[edge] ) / sumD2[edge];
      if ( !std::isfinite( slope ) || slope <= 0.03 || slope > 5.0 )
        return result;
      double squaredError = 0.0;
      for ( const FrustumSideSample &sample : sideInliers.at( edge ) )
      {
        const double residual = ( sample.point.z() - eaveHeight - slope * sample.inwardDistance ) / std::sqrt( 1.0 + slope * slope );
        squaredError += residual * residual;
      }
      if ( std::sqrt( squaredError / sideInliers.at( edge ).size() ) > residualThreshold * 1.5 )
        return result;
      offsets[edge] = ( topHeight - eaveHeight ) / slope;
      if ( offsets[edge] <= std::max( 0.08, extent * 0.005 ) )
        return result;
    }

    const QVector<QgsPointXY> topRing = flatTopRingFromOffsets( ring, offsets );
    if ( topRing.isEmpty() )
      return result;

    const QgsGeometry topGeometry = polygonGeometryFromRing( topRing );
    const double footprintArea = polygonGeometryFromRing( ring ).area();
    if ( topGeometry.isNull() || !topGeometry.isGeosValid() || topGeometry.area() < footprintArea * 0.02
         || topGeometry.area() > footprintArea * 0.995 )
      return result;
    int supportedTopPoints = 0;
    for ( const QgsPoint &point : topRegion.points )
    {
      if ( pointInRing( topRing, QgsPointXY( point.x(), point.y() ) ) )
        ++supportedTopPoints;
    }
    if ( supportedTopPoints < static_cast<int>( std::ceil( topRegion.points.size() * 0.85 ) ) )
      return result;

    for ( int edge = 0; edge < 4; ++edge )
    {
      int slopePointsInsideTop = 0;
      for ( const FrustumSideSample &sample : sideInliers.at( edge ) )
      {
        if ( pointInRing( topRing, QgsPointXY( sample.point.x(), sample.point.y() ) ) )
          ++slopePointsInsideTop;
      }
      if ( slopePointsInsideTop > sideInliers.at( edge ).size() * 0.15 )
        return result;
    }

    result.success = true;
    result.eaveHeight = eaveHeight;
    result.topHeight = topHeight;
    result.topRing = topRing;
    result.boundaryContacts = boundaryContacts;
    return result;
  }

  bool evaluateFlatTopRoofCandidate( const QVector<QgsPointXY> &ring, const FittedFlatTopHippedRoof &roof,
                                     const HorizontalRoofRegion &topRegion,
                                     const QVector<BuildingRoof::RoofSample> &samples,
                                     const QVector<BuildingRoof::RoofSample> &observations, double &score )
  {
    score = std::numeric_limits<double>::max();
    if ( !roof.success || ring.size() != 4 || roof.topRing.size() != 4 || roof.boundaryContacts.size() != 4 || observations.size() < 12 )
      return false;
    const double rise = roof.topHeight - roof.eaveHeight;
    if ( rise <= 0.15 )
      return false;
    const double tolerance = std::max( 0.30, std::min( 0.60, std::max( rise * 0.08, topRegion.plane.rmse * 3.0 ) ) );
    const double topMargin = std::max( 0.08, rise * 0.04 );
    double edgeX[4];
    double edgeY[4];
    double lengths[4];
    QVector<double> offsets( 4 );
    for ( int edge = 0; edge < 4; ++edge )
    {
      const QgsPointXY &a = ring.at( edge );
      const QgsPointXY &b = ring.at( ( edge + 1 ) % 4 );
      lengths[edge] = std::hypot( b.x() - a.x(), b.y() - a.y() );
      if ( lengths[edge] <= 1e-8 )
        return false;
      edgeX[edge] = ( b.x() - a.x() ) / lengths[edge];
      edgeY[edge] = ( b.y() - a.y() ) / lengths[edge];
      offsets[edge] = cross2d( edgeX[edge], edgeY[edge], roof.topRing.at( edge ).x() - a.x(), roof.topRing.at( edge ).y() - a.y() );
      if ( roof.boundaryContacts.at( edge ) )
      {
        if ( std::fabs( offsets.at( edge ) ) > 1e-6 )
          return false;
        offsets[edge] = 0.0;
      }
      else if ( offsets.at( edge ) <= 1e-8 )
        return false;
    }
    if ( flatTopRingFromOffsets( ring, offsets ).isEmpty() )
      return false;
    auto faceAt = [&]( const QgsPoint &point, double &fraction ) {
      fraction = std::numeric_limits<double>::max();
      int face = 0;
      for ( int edge = 0; edge < 4; ++edge )
      {
        if ( roof.boundaryContacts.at( edge ) )
          continue;
        const double distance = cross2d( edgeX[edge], edgeY[edge], point.x() - ring.at( edge ).x(), point.y() - ring.at( edge ).y() );
        const double progress = distance / offsets.at( edge );
        if ( progress < fraction )
        {
          fraction = progress;
          face = edge;
        }
      }
      return face;
    };
    int supportedTop = 0;
    for ( const QgsPoint &point : topRegion.points )
    {
      double fraction = 0.0;
      faceAt( point, fraction );
      supportedTop += fraction >= 0.95 ? 1 : 0;
    }
    if ( supportedTop < static_cast<int>( std::ceil( topRegion.points.size() * 0.80 ) ) )
      return false;

    struct FaceSupport
    {
      int total = 0;
      int matched = 0;
      int matchedCells = 0;
      double sumDistance = 0.0;
      double sumDistance2 = 0.0;
      double sumHeight = 0.0;
      double sumDistanceHeight = 0.0;
      double minAlong = std::numeric_limits<double>::max();
      double maxAlong = std::numeric_limits<double>::lowest();
      double minDistance = std::numeric_limits<double>::max();
      double maxDistance = std::numeric_limits<double>::lowest();
    };
    FaceSupport support[4];
    for ( const BuildingRoof::RoofSample &sample : samples )
    {
      const QgsPoint &point = sample.point;
      double fraction = 0.0;
      const int face = faceAt( point, fraction );
      if ( fraction <= 0.05 || fraction >= 0.95 || point.z() >= roof.topHeight - topMargin
           || point.z() < roof.eaveHeight - tolerance )
        continue;
      FaceSupport &entry = support[face];
      ++entry.total;
      const double predicted = roof.eaveHeight + rise * fraction;
      if ( std::fabs( point.z() - predicted ) > tolerance )
        continue;
      ++entry.matched;
      const double distance = fraction * offsets.at( face );
      entry.sumDistance += distance;
      entry.sumDistance2 += distance * distance;
      entry.sumHeight += point.z();
      entry.sumDistanceHeight += distance * point.z();
      const double along = ( point.x() - ring.at( face ).x() ) * edgeX[face] + ( point.y() - ring.at( face ).y() ) * edgeY[face];
      entry.minAlong = std::min( entry.minAlong, along );
      entry.maxAlong = std::max( entry.maxAlong, along );
      entry.minDistance = std::min( entry.minDistance, distance );
      entry.maxDistance = std::max( entry.maxDistance, distance );
    }

    int matchedCells = 0;
    int aboveTopCells = 0;
    double loss = 0.0;
    QVector<double> errors;
    errors.reserve( observations.size() );
    for ( const BuildingRoof::RoofSample &sample : observations )
    {
      const QgsPoint &point = sample.point;
      double fraction = 0.0;
      const int face = faceAt( point, fraction );
      const double predicted = roof.eaveHeight + rise * std::max( 0.0, std::min( 1.0, fraction ) );
      const double error = std::fabs( predicted - point.z() );
      const double residual = error / 0.25;
      loss += residual <= 1.0 ? residual * residual : 2.0 * residual - 1.0;
      errors.append( error );
      aboveTopCells += point.z() > roof.topHeight + tolerance ? 1 : 0;
      if ( error <= tolerance )
      {
        ++matchedCells;
        if ( fraction > 0.05 && fraction < 0.95 && point.z() < roof.topHeight - topMargin )
          ++support[face].matchedCells;
      }
    }
    // A relatively best candidate must still explain the roof, not a lower platform or a few slope points.
    if ( matchedCells < static_cast<int>( std::ceil( observations.size() * 0.65 ) )
         || aboveTopCells > observations.size() * 0.25 )
      return false;
    std::sort( errors.begin(), errors.end() );
    if ( sortedPercentile( errors, 0.80 ) > tolerance * 2.0 )
      return false;
    for ( int face = 0; face < 4; ++face )
    {
      if ( roof.boundaryContacts.at( face ) )
        continue;
      const FaceSupport &entry = support[face];
      if ( entry.matched < 8 || entry.matchedCells < 3 || entry.matched < entry.total * 0.50
           || entry.maxAlong - entry.minAlong < std::max( 0.20, lengths[face] * 0.10 )
           || entry.maxDistance - entry.minDistance < std::max( 0.05, std::min( 0.20, offsets.at( face ) * 0.12 ) ) )
        return false;
      const double denominator = entry.matched * entry.sumDistance2 - entry.sumDistance * entry.sumDistance;
      if ( denominator <= 1e-8 )
        return false;
      const double observedSlope = ( entry.matched * entry.sumDistanceHeight - entry.sumDistance * entry.sumHeight ) / denominator;
      const double fittedSlope = rise / offsets.at( face );
      if ( !std::isfinite( observedSlope ) || observedSlope <= 0.03
           || std::fabs( std::atan( observedSlope ) - std::atan( fittedSlope ) ) > 20.0 * 3.14159265358979323846 / 180.0 )
        return false;
    }
    score = loss / observations.size();
    return std::isfinite( score );
  }

  struct GabledFitContext
  {
    AutoHeightBand heightBand;
    bool segmented = false;
    BuildingRoof::RoofPlaneSegmentation segmentation;
    QVector<LeastSquaresRoofPlane> planes;
  };

  bool leastSquaresGabledRidgePointFromPointCloud( const QVector<QgsPointXY> &ring, const QVector<BuildingRoof::RoofSample> &pointCloudSamples, const QgsPoint &boundaryPoint, QgsPoint &ridgePoint, double *ridgeDirX = nullptr, double *ridgeDirY = nullptr, GabledBoundarySlopePlanes *boundarySlopePlanes = nullptr, GabledFitContext *fitContext = nullptr )
  {
    double baseDirX = 0.0;
    double baseDirY = 0.0;
    if ( !nearestEdgeDirection( ring, boundaryPoint, baseDirX, baseDirY ) )
      return false;

    const double baseDirLength = std::hypot( baseDirX, baseDirY );
    if ( baseDirLength <= 1e-10 )
      return false;
    baseDirX /= baseDirLength;
    baseDirY /= baseDirLength;
    const double normalX = -baseDirY;
    const double normalY = baseDirX;

    const AutoHeightBand heightBand = fitContext ? fitContext->heightBand : inferAutoGabledHeightBand( ring, pointCloudSamples );
    if ( !heightBand.success || heightBand.filteredHeights.size() < 20 )
      return false;

    const double extent = ringExtentSize( ring );
    const double roofFloor = std::max( sortedPercentile( heightBand.filteredHeights, 0.35 ),
                                      heightBand.baseHeight + std::max( 0.20, ( heightBand.ridgeHeight - heightBand.baseHeight ) * 0.16 ) );
    QVector<QgsPoint> roofPoints;
    QVector<double> offsets;
    roofPoints.reserve( pointCloudSamples.size() );
    offsets.reserve( pointCloudSamples.size() );
    for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
    {
      const QgsPoint &point = sample.point;
      if ( point.z() < roofFloor || point.z() < heightBand.lowHeight || point.z() > heightBand.highHeight )
        continue;
      if ( !pointInRing( ring, QgsPointXY( point.x(), point.y() ) ) )
        continue;

      roofPoints.append( point );
      offsets.append( point.x() * normalX + point.y() * normalY );
    }

    if ( roofPoints.size() < 40 )
      return false;

    GabledFitContext localContext;
    GabledFitContext &context = fitContext ? *fitContext : localContext;
    if ( !context.segmented )
    {
      BuildingRoof::RoofSegmentationOptions segmentationOptions;
      segmentationOptions.minimumInlierFraction = 0.0;
      context.segmentation = BuildingRoof::segmentRoofCloud( polygonGeometryFromRing( ring ), pointCloudSamples, segmentationOptions );
      context.planes.reserve( context.segmentation.segments.size() );
      for ( const BuildingRoof::RoofPlaneSegment &segment : context.segmentation.segments )
        context.planes.append( segment.points.size() >= 12 ? fitLeastSquaresRoofPlane( segment.points ) : LeastSquaresRoofPlane() );
      context.segmented = true;
    }
    const BuildingRoof::RoofPlaneSegmentation &dominantSegmentation = context.segmentation;
    double dominantPairScore = std::numeric_limits<double>::max();
    QgsPoint dominantRidgePoint;
    double dominantDirX = baseDirX;
    double dominantDirY = baseDirY;
    GabledBoundarySlopePlanes dominantSlopePlanes;
    bool dominantPairFound = false;
    for ( int firstIndex = 0; firstIndex < dominantSegmentation.segments.size(); ++firstIndex )
    {
      const BuildingRoof::RoofPlaneSegment &firstSegment = dominantSegmentation.segments.at( firstIndex );
      if ( firstSegment.points.size() < 12 )
        continue;
      // Preserve both horizontal gradient components; the footprint only guides pair selection.
      const LeastSquaresRoofPlane &firstPlane = context.planes.at( firstIndex );
      if ( !firstPlane.success )
        continue;

      for ( int secondIndex = firstIndex + 1; secondIndex < dominantSegmentation.segments.size(); ++secondIndex )
      {
        const BuildingRoof::RoofPlaneSegment &secondSegment = dominantSegmentation.segments.at( secondIndex );
        if ( secondSegment.points.size() < 12 )
          continue;
        const LeastSquaresRoofPlane &secondPlane = context.planes.at( secondIndex );
        if ( !secondPlane.success )
          continue;

        const double lineA = firstPlane.a - secondPlane.a;
        const double lineB = firstPlane.b - secondPlane.b;
        const double lineC = firstPlane.c - secondPlane.c;
        const double lineLength = std::hypot( lineA, lineB );
        if ( lineLength <= 1e-10 )
          continue;

        double dirX = -lineB / lineLength;
        double dirY = lineA / lineLength;
        const double alignment = std::fabs( dirX * baseDirX + dirY * baseDirY );
        if ( alignment < 0.65 )
          continue;
        if ( dirX * baseDirX + dirY * baseDirY < 0.0 )
        {
          dirX = -dirX;
          dirY = -dirY;
        }

        const double denominator = lineA * lineA + lineB * lineB;
        const QgsPointXY lineOrigin( -lineA * lineC / denominator, -lineB * lineC / denominator );
        const QVector<LineInterval> intervals = lineInsideRingIntervals( ring, lineOrigin, dirX, dirY );
        if ( intervals.isEmpty() )
          continue;
        LineInterval bestInterval = intervals.first();
        for ( const LineInterval &interval : intervals )
        {
          if ( interval.end - interval.start > bestInterval.end - bestInterval.start )
            bestInterval = interval;
        }
        if ( bestInterval.end - bestInterval.start < std::max( 0.8, extent * 0.12 ) )
          continue;

        const QgsPointXY ridgeXY = pointOnLine( lineOrigin, dirX, dirY, 0.5 * ( bestInterval.start + bestInterval.end ) );
        const double ridgeHeight = 0.5 * ( roofPlaneZ( firstPlane, ridgeXY ) + roofPlaneZ( secondPlane, ridgeXY ) );
        if ( !std::isfinite( ridgeHeight )
             || ridgeHeight <= heightBand.baseHeight + std::max( 0.20, heightBand.binWidth )
             || ridgeHeight > heightBand.highHeight + std::max( 0.30, heightBand.binWidth * 2.0 ) )
          continue;

        const QgsPoint candidateRidgePoint( ridgeXY.x(), ridgeXY.y(), ridgeHeight );
        const double ridgeNormalX = -dirY;
        const double ridgeNormalY = dirX;
        const double firstSideSign = dominantSideSign( firstSegment.points, candidateRidgePoint, ridgeNormalX, ridgeNormalY );
        const double secondSideSign = dominantSideSign( secondSegment.points, candidateRidgePoint, ridgeNormalX, ridgeNormalY );
        if ( firstSideSign * secondSideSign >= 0.0 )
          continue;
        const double firstAcrossSlope = firstPlane.a * ridgeNormalX + firstPlane.b * ridgeNormalY;
        const double secondAcrossSlope = secondPlane.a * ridgeNormalX + secondPlane.b * ridgeNormalY;
        if ( firstAcrossSlope * firstSideSign >= -0.005 || secondAcrossSlope * secondSideSign >= -0.005 )
          continue;

        const double combinedAreaRatio = firstSegment.projectedAreaRatio + secondSegment.projectedAreaRatio;
        const double balance = static_cast<double>( std::min( firstSegment.points.size(), secondSegment.points.size() ) )
                               / std::max( firstSegment.points.size(), secondSegment.points.size() );
        const double ridgeHeightPenalty = std::fabs( ridgeHeight - heightBand.ridgeHeight )
                                          / std::max( 0.30, heightBand.highHeight - heightBand.lowHeight );
        const double score = ( firstPlane.rmse * firstSegment.points.size() + secondPlane.rmse * secondSegment.points.size() )
                             / std::max( 1, firstSegment.points.size() + secondSegment.points.size() )
                             + ( 1.0 - alignment ) * 0.45
                             + ( 1.0 - balance ) * 0.15
                             + ridgeHeightPenalty * 0.25
                             + std::max( 0.0, 0.10 - combinedAreaRatio ) * 2.0
                             - std::min( 0.70, combinedAreaRatio ) * 0.08;
        if ( score >= dominantPairScore )
          continue;

        dominantPairScore = score;
        dominantRidgePoint = candidateRidgePoint;
        dominantDirX = dirX;
        dominantDirY = dirY;
        dominantSlopePlanes.success = true;
        dominantSlopePlanes.firstPlane = firstPlane;
        dominantSlopePlanes.secondPlane = secondPlane;
        dominantSlopePlanes.firstPlaneSideSign = firstSideSign;
        dominantSlopePlanes.secondPlaneSideSign = secondSideSign;
        dominantSlopePlanes.hasSideSigns = true;
        dominantPairFound = true;
      }
    }

    if ( dominantPairFound )
    {
      ridgePoint = dominantRidgePoint;
      if ( ridgeDirX )
        *ridgeDirX = dominantDirX;
      if ( ridgeDirY )
        *ridgeDirY = dominantDirY;
      if ( boundarySlopePlanes )
        *boundarySlopePlanes = dominantSlopePlanes;
      return true;
    }

    std::sort( offsets.begin(), offsets.end() );
    const int minSideCount = std::max( 12, static_cast<int>( std::ceil( roofPoints.size() * 0.18 ) ) );
    double bestScore = std::numeric_limits<double>::max();
    QgsPoint bestRidgePoint;
    double bestDirX = baseDirX;
    double bestDirY = baseDirY;
    GabledBoundarySlopePlanes bestBoundarySlopePlanes;
    bool found = false;

    QVector<double> splitPercentiles;
    splitPercentiles << 0.35 << 0.40 << 0.45 << 0.50 << 0.55 << 0.60 << 0.65;
    for ( double percentile : splitPercentiles )
    {
      const double splitOffset = sortedPercentile( offsets, percentile );
      QVector<QgsPoint> firstSide;
      QVector<QgsPoint> secondSide;
      firstSide.reserve( roofPoints.size() );
      secondSide.reserve( roofPoints.size() );
      for ( const QgsPoint &point : roofPoints )
      {
        const double offset = point.x() * normalX + point.y() * normalY;
        if ( offset <= splitOffset )
          firstSide.append( point );
        else
          secondSide.append( point );
      }

      if ( firstSide.size() < minSideCount || secondSide.size() < minSideCount )
        continue;

      const AreaFilteredRoofPlaneFit firstFit = fitAreaFilteredRoofPlane( firstSide );
      const AreaFilteredRoofPlaneFit secondFit = fitAreaFilteredRoofPlane( secondSide );
      if ( !firstFit.success || !secondFit.success )
        continue;
      const LeastSquaresRoofPlane &firstPlane = firstFit.plane;
      const LeastSquaresRoofPlane &secondPlane = secondFit.plane;

      const double lineA = firstPlane.a - secondPlane.a;
      const double lineB = firstPlane.b - secondPlane.b;
      const double lineC = firstPlane.c - secondPlane.c;
      const double lineLength = std::hypot( lineA, lineB );
      if ( lineLength <= 1e-10 )
        continue;

      double dirX = -lineB / lineLength;
      double dirY = lineA / lineLength;
      const double alignment = std::fabs( dirX * baseDirX + dirY * baseDirY );
      if ( alignment < 0.45 )
        continue;
      if ( dirX * baseDirX + dirY * baseDirY < 0.0 )
      {
        dirX = -dirX;
        dirY = -dirY;
      }

      const double denominator = lineA * lineA + lineB * lineB;
      const QgsPointXY lineOrigin( -lineA * lineC / denominator, -lineB * lineC / denominator );
      const QVector<LineInterval> intervals = lineInsideRingIntervals( ring, lineOrigin, dirX, dirY );
      if ( intervals.isEmpty() )
        continue;

      LineInterval bestInterval = intervals.first();
      for ( const LineInterval &interval : intervals )
      {
        if ( interval.end - interval.start > bestInterval.end - bestInterval.start )
          bestInterval = interval;
      }
      if ( bestInterval.end - bestInterval.start < std::max( 0.8, extent * 0.12 ) )
        continue;

      const QgsPointXY ridgeXY = pointOnLine( lineOrigin, dirX, dirY, ( bestInterval.start + bestInterval.end ) * 0.5 );
      double ridgeHeight = 0.5 * ( roofPlaneZ( firstPlane, ridgeXY ) + roofPlaneZ( secondPlane, ridgeXY ) );
      if ( !std::isfinite( ridgeHeight ) )
        continue;
      if ( ridgeHeight <= heightBand.baseHeight + std::max( 0.20, heightBand.binWidth ) )
        continue;

      const double balance = static_cast<double>( std::min( firstFit.points.size(), secondFit.points.size() ) ) / std::max( firstFit.points.size(), secondFit.points.size() );
      const double ridgeHeightPenalty = std::fabs( ridgeHeight - heightBand.ridgeHeight ) / std::max( 0.30, heightBand.highHeight - heightBand.lowHeight );
      const double fittedPointCount = std::max( 1, firstFit.points.size() + secondFit.points.size() );
      const double score = ( firstPlane.rmse * firstFit.points.size() + secondPlane.rmse * secondFit.points.size() ) / fittedPointCount
                           + ridgeHeightPenalty * 0.35
                           + ( 1.0 - alignment ) * 0.50
                           + ( 1.0 - balance ) * 0.20;
      if ( score < bestScore )
      {
        bestScore = score;
        bestRidgePoint = QgsPoint( ridgeXY.x(), ridgeXY.y(), ridgeHeight );
        bestDirX = dirX;
        bestDirY = dirY;
        bestBoundarySlopePlanes.success = true;
        bestBoundarySlopePlanes.firstPlane = firstPlane;
        bestBoundarySlopePlanes.secondPlane = secondPlane;
        const double ridgeNormalX = -dirY;
        const double ridgeNormalY = dirX;
        bestBoundarySlopePlanes.firstPlaneSideSign = dominantSideSign( firstFit.points, bestRidgePoint, ridgeNormalX, ridgeNormalY );
        bestBoundarySlopePlanes.secondPlaneSideSign = dominantSideSign( secondFit.points, bestRidgePoint, ridgeNormalX, ridgeNormalY );
        bestBoundarySlopePlanes.hasSideSigns = bestBoundarySlopePlanes.firstPlaneSideSign * bestBoundarySlopePlanes.secondPlaneSideSign < 0.0;
        found = true;
      }
    }

    if ( !found )
      return false;

    ridgePoint = bestRidgePoint;
    if ( ridgeDirX )
      *ridgeDirX = bestDirX;
    if ( ridgeDirY )
      *ridgeDirY = bestDirY;
    if ( boundarySlopePlanes )
      *boundarySlopePlanes = bestBoundarySlopePlanes;
    return true;
  }

  bool topHeightRidgePointFromPointCloud( const QVector<QgsPointXY> &ring, const QVector<BuildingRoof::RoofSample> &pointCloudSamples, const QgsPoint &boundaryPoint, QgsPoint &ridgePoint, double *ridgeDirX = nullptr, double *ridgeDirY = nullptr )
  {
    QVector<QgsPoint> points;
    points.reserve( pointCloudSamples.size() );
    for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
    {
      const QgsPoint &point = sample.point;
      if ( pointInRing( ring, QgsPointXY( point.x(), point.y() ) ) )
        points.append( point );
    }

    if ( points.size() < 8 )
      return false;

    std::sort( points.begin(), points.end(), []( const QgsPoint &left, const QgsPoint &right ) {
      return left.z() > right.z();
    } );

    const int topCount = std::max( 8, static_cast<int>( std::ceil( points.size() * 0.08 ) ) );
    const int count = std::min( points.size(), topCount );
    QVector<QgsPoint> topPoints;
    topPoints.reserve( count );
    double sumX = 0.0;
    double sumY = 0.0;
    double sumZ = 0.0;
    for ( int i = 0; i < count; ++i )
    {
      const QgsPoint &point = points.at( i );
      sumX += point.x();
      sumY += point.y();
      sumZ += point.z();
      topPoints.append( point );
    }

    const QgsPoint fallbackPoint( sumX / count, sumY / count, sumZ / count );

    double dirX = 0.0;
    double dirY = 0.0;
    if ( !nearestEdgeDirection( ring, boundaryPoint, dirX, dirY ) )
    {
      ridgePoint = fallbackPoint;
      if ( ridgeDirX )
        *ridgeDirX = 0.0;
      if ( ridgeDirY )
        *ridgeDirY = 0.0;
      return true;
    }

    const double dirLength = std::hypot( dirX, dirY );
    if ( dirLength <= 1e-10 )
    {
      ridgePoint = fallbackPoint;
      if ( ridgeDirX )
        *ridgeDirX = 0.0;
      if ( ridgeDirY )
        *ridgeDirY = 0.0;
      return true;
    }
    dirX /= dirLength;
    dirY /= dirLength;
    const double normalX = -dirY;
    const double normalY = dirX;

    QVector<double> offsets;
    QVector<double> parameters;
    offsets.reserve( topPoints.size() );
    parameters.reserve( topPoints.size() );
    for ( const QgsPoint &point : topPoints )
    {
      offsets.append( point.x() * normalX + point.y() * normalY );
      parameters.append( point.x() * dirX + point.y() * dirY );
    }

    std::sort( offsets.begin(), offsets.end() );
    std::sort( parameters.begin(), parameters.end() );
    const double centerOffset = sortedPercentile( offsets, 0.50 );
    const double preferredT = sortedPercentile( parameters, 0.50 );
    const QgsPointXY lineOrigin( normalX * centerOffset, normalY * centerOffset );

    const QVector<LineInterval> intervals = lineInsideRingIntervals( ring, lineOrigin, dirX, dirY );
    if ( intervals.isEmpty() )
    {
      ridgePoint = fallbackPoint;
      return true;
    }

    LineInterval bestInterval = intervals.first();
    bool foundPreferredInterval = false;
    for ( const LineInterval &interval : intervals )
    {
      if ( intervalContains( interval, preferredT ) )
      {
        bestInterval = interval;
        foundPreferredInterval = true;
        break;
      }
      if ( !foundPreferredInterval && interval.end - interval.start > bestInterval.end - bestInterval.start )
        bestInterval = interval;
    }

    const double ridgeT = foundPreferredInterval ? preferredT : ( bestInterval.start + bestInterval.end ) * 0.5;
    const QgsPointXY centeredPoint = pointOnLine( lineOrigin, dirX, dirY, ridgeT );
    ridgePoint = QgsPoint( centeredPoint.x(), centeredPoint.y(), fallbackPoint.z() );
    if ( ridgeDirX )
      *ridgeDirX = dirX;
    if ( ridgeDirY )
      *ridgeDirY = dirY;
    return true;
  }

  bool topHeightBentRidgePointFromPointCloud( const QVector<QgsPointXY> &ring, const QVector<BuildingRoof::RoofSample> &pointCloudSamples, const QgsPoint &boundaryPoint, QgsPoint &ridgePoint, double *ridgeDirX = nullptr, double *ridgeDirY = nullptr )
  {
    QVector<QgsPoint> points;
    points.reserve( pointCloudSamples.size() );
    for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
    {
      const QgsPoint &point = sample.point;
      if ( pointInRing( ring, QgsPointXY( point.x(), point.y() ) ) )
        points.append( point );
    }

    if ( points.size() < 8 )
      return false;

    std::sort( points.begin(), points.end(), []( const QgsPoint &left, const QgsPoint &right ) {
      return left.z() > right.z();
    } );

    const int topCount = std::max( 8, static_cast<int>( std::ceil( points.size() * 0.08 ) ) );
    const int count = std::min( points.size(), topCount );
    QVector<QgsPoint> topPoints;
    topPoints.reserve( count );
    for ( int i = 0; i < count; ++i )
      topPoints.append( points.at( i ) );

    double dirX = 0.0;
    double dirY = 0.0;
    if ( !nearestEdgeDirection( ring, boundaryPoint, dirX, dirY ) )
      return topHeightRidgePointFromPointCloud( ring, pointCloudSamples, boundaryPoint, ridgePoint, ridgeDirX, ridgeDirY );

    const double dirLength = std::hypot( dirX, dirY );
    if ( dirLength <= 1e-10 )
      return topHeightRidgePointFromPointCloud( ring, pointCloudSamples, boundaryPoint, ridgePoint, ridgeDirX, ridgeDirY );
    dirX /= dirLength;
    dirY /= dirLength;
    const double normalX = -dirY;
    const double normalY = dirX;

    QVector<double> offsets;
    offsets.reserve( topPoints.size() );
    for ( const QgsPoint &point : topPoints )
      offsets.append( point.x() * normalX + point.y() * normalY );

    std::sort( offsets.begin(), offsets.end() );
    const double minOffset = offsets.first();
    const double maxOffset = offsets.last();
    const double extent = ringExtentSize( ring );
    const double binWidth = std::max( 0.20, extent * 0.012 );
    const int binCount = std::max( 2, static_cast<int>( std::ceil( ( maxOffset - minOffset ) / binWidth ) ) + 1 );
    QVector<int> counts( binCount, 0 );
    for ( double offset : offsets )
    {
      const int index = std::max( 0, std::min( binCount - 1, static_cast<int>( std::floor( ( offset - minOffset ) / binWidth ) ) ) );
      ++counts[index];
    }

    int bestBin = -1;
    int bestScore = 0;
    for ( int i = 0; i < binCount; ++i )
    {
      const int previous = i > 0 ? counts.at( i - 1 ) : 0;
      const int next = i + 1 < binCount ? counts.at( i + 1 ) : 0;
      const int score = previous + counts.at( i ) + next;
      if ( score > bestScore )
      {
        bestScore = score;
        bestBin = i;
      }
    }

    const int minSupport = std::max( 5, count / 12 );
    if ( bestBin < 0 || bestScore < minSupport )
      return topHeightRidgePointFromPointCloud( ring, pointCloudSamples, boundaryPoint, ridgePoint, ridgeDirX, ridgeDirY );

    const double roughOffset = minOffset + ( bestBin + 0.5 ) * binWidth;
    const double tolerance = std::max( binWidth * 1.5, extent * 0.015 );
    QVector<double> parameters;
    parameters.reserve( topPoints.size() );
    double offsetSum = 0.0;
    double zSum = 0.0;
    int fitCount = 0;
    for ( const QgsPoint &point : topPoints )
    {
      const double offset = point.x() * normalX + point.y() * normalY;
      if ( std::fabs( offset - roughOffset ) > tolerance )
        continue;

      offsetSum += offset;
      zSum += point.z();
      parameters.append( point.x() * dirX + point.y() * dirY );
      ++fitCount;
    }

    if ( fitCount < minSupport || parameters.isEmpty() )
      return topHeightRidgePointFromPointCloud( ring, pointCloudSamples, boundaryPoint, ridgePoint, ridgeDirX, ridgeDirY );

    std::sort( parameters.begin(), parameters.end() );
    const double centerOffset = offsetSum / fitCount;
    const double preferredT = sortedPercentile( parameters, 0.50 );
    const QgsPointXY lineOrigin( normalX * centerOffset, normalY * centerOffset );
    const QVector<LineInterval> intervals = lineInsideRingIntervals( ring, lineOrigin, dirX, dirY );
    if ( intervals.isEmpty() )
      return topHeightRidgePointFromPointCloud( ring, pointCloudSamples, boundaryPoint, ridgePoint, ridgeDirX, ridgeDirY );

    LineInterval bestInterval = intervals.first();
    bool foundPreferredInterval = false;
    for ( const LineInterval &interval : intervals )
    {
      if ( intervalContains( interval, preferredT ) )
      {
        bestInterval = interval;
        foundPreferredInterval = true;
        break;
      }
      if ( !foundPreferredInterval && interval.end - interval.start > bestInterval.end - bestInterval.start )
        bestInterval = interval;
    }

    const double ridgeT = foundPreferredInterval
                            ? std::max( bestInterval.start, std::min( bestInterval.end, preferredT ) )
                            : ( bestInterval.start + bestInterval.end ) * 0.5;
    const QgsPointXY centeredPoint = pointOnLine( lineOrigin, dirX, dirY, ridgeT );
    ridgePoint = QgsPoint( centeredPoint.x(), centeredPoint.y(), zSum / fitCount );
    if ( ridgeDirX )
      *ridgeDirX = dirX;
    if ( ridgeDirY )
      *ridgeDirY = dirY;
    return true;
  }

  struct LeastSquaresBentSegmentRidge
  {
    bool success = false;
    QgsPoint point;
    double dirX = 0.0;
    double dirY = 0.0;
    bool boundaryHeightSuccess = false;
    double boundaryHeight = 0.0;
    double score = 0.0;
    int supportCount = 0;
  };

  LeastSquaresBentSegmentRidge fitLeastSquaresBentSegmentRidge( const QVector<QgsPointXY> &ring, const BentGableSegment &segment, const QVector<QgsPoint> &points, const QgsPoint &boundaryPoint )
  {
    LeastSquaresBentSegmentRidge result;
    if ( points.size() < 24 )
      return result;

    QVector<QgsPoint> firstSide;
    QVector<QgsPoint> secondSide;
    firstSide.reserve( points.size() );
    secondSide.reserve( points.size() );
    for ( const QgsPoint &point : points )
    {
      const QgsPointXY pointXY( point.x(), point.y() );
      const double along = lineParameter( segment.start, segment.dirX, segment.dirY, pointXY );
      if ( along < -0.25 || along > segment.length + 0.25 )
        continue;

      const double distance = signedDistanceToLine( pointXY, segment.ridgePoint, segment.normalX, segment.normalY );
      if ( std::fabs( distance ) < std::max( 0.08, ringExtentSize( ring ) * 0.002 ) )
        continue;
      if ( distance <= 0.0 )
        firstSide.append( point );
      else
        secondSide.append( point );
    }

    const int minSideCount = std::max( 8, static_cast<int>( std::ceil( points.size() * 0.16 ) ) );
    if ( firstSide.size() < minSideCount || secondSide.size() < minSideCount )
      return result;

    const AreaFilteredRoofPlaneFit firstFit = fitAreaFilteredRoofPlane( firstSide );
    const AreaFilteredRoofPlaneFit secondFit = fitAreaFilteredRoofPlane( secondSide );
    if ( !firstFit.success || !secondFit.success )
      return result;
    const LeastSquaresRoofPlane &firstPlane = firstFit.plane;
    const LeastSquaresRoofPlane &secondPlane = secondFit.plane;

    const double lineA = firstPlane.a - secondPlane.a;
    const double lineB = firstPlane.b - secondPlane.b;
    const double lineC = firstPlane.c - secondPlane.c;
    const double lineLength = std::hypot( lineA, lineB );
    if ( lineLength <= 1e-10 )
      return result;

    double intersectionDirX = -lineB / lineLength;
    double intersectionDirY = lineA / lineLength;
    const double alignment = std::fabs( intersectionDirX * segment.dirX + intersectionDirY * segment.dirY );
    if ( alignment < 0.90 )
      return result;

    const QgsPointXY preferredPoint( ( segment.start.x() + segment.end.x() ) * 0.5, ( segment.start.y() + segment.end.y() ) * 0.5 );
    const double denominator = lineA * lineA + lineB * lineB;
    QgsPointXY fittedPoint( preferredPoint.x() - lineA * ( lineA * preferredPoint.x() + lineB * preferredPoint.y() + lineC ) / denominator,
                            preferredPoint.y() - lineB * ( lineA * preferredPoint.x() + lineB * preferredPoint.y() + lineC ) / denominator );
    if ( !pointInRing( ring, fittedPoint ) )
      return result;

    const double fittedAlong = lineParameter( segment.start, segment.dirX, segment.dirY, fittedPoint );
    const double crossOffset = signedDistanceToLine( fittedPoint, segment.ridgePoint, segment.normalX, segment.normalY );
    const double maxCrossOffset = std::max( 0.20, ringExtentSize( ring ) * 0.05 );
    if ( fittedAlong < -0.25 || fittedAlong > segment.length + 0.25 || std::fabs( crossOffset ) > maxCrossOffset )
      return result;

    const double height = 0.5 * ( roofPlaneZ( firstPlane, fittedPoint ) + roofPlaneZ( secondPlane, fittedPoint ) );
    if ( !std::isfinite( height ) )
      return result;

    result.success = true;
    result.point = QgsPoint( fittedPoint.x(), fittedPoint.y(), height );
    result.dirX = intersectionDirX;
    result.dirY = intersectionDirY;
    result.supportCount = firstFit.points.size() + secondFit.points.size();
    const QgsPointXY boundaryXY( boundaryPoint.x(), boundaryPoint.y() );
    const double boundaryAlong = lineParameter( segment.start, segment.dirX, segment.dirY, boundaryXY );
    if ( boundaryAlong >= -0.50 && boundaryAlong <= segment.length + 0.50 )
    {
      const double boundaryDistance = signedDistanceToLine( boundaryXY, segment.ridgePoint, segment.normalX, segment.normalY );
      const LeastSquaresRoofPlane &boundaryPlane = boundaryDistance <= 0.0 ? firstPlane : secondPlane;
      const double boundaryHeight = roofPlaneZ( boundaryPlane, boundaryXY );
      if ( std::isfinite( boundaryHeight ) && boundaryHeight > 0.0 && boundaryHeight < height )
      {
        result.boundaryHeightSuccess = true;
        result.boundaryHeight = boundaryHeight;
      }
    }
    result.score = ( firstPlane.rmse * firstFit.points.size() + secondPlane.rmse * secondFit.points.size() ) / std::max( 1, result.supportCount )
                   + ( 1.0 - alignment ) * 0.35;
    return result;
  }

  BentGableLayout joinFittedBentRidges( const QVector<QgsPointXY> &ring, const BentGableLayout &initial,
                                        const LeastSquaresBentSegmentRidge &primary, const LeastSquaresBentSegmentRidge &secondary )
  {
    BentGableLayout result;
    const double denominator = cross2d( primary.dirX, primary.dirY, secondary.dirX, secondary.dirY );
    if ( !primary.success || !secondary.success || std::fabs( denominator ) <= 1e-6 )
      return result;
    const double t = cross2d( secondary.point.x() - primary.point.x(), secondary.point.y() - primary.point.y(),
                              secondary.dirX, secondary.dirY ) / denominator;
    const QgsPointXY bend = pointOnLine( QgsPointXY( primary.point.x(), primary.point.y() ), primary.dirX, primary.dirY, t );
    const double extent = ringExtentSize( ring );
    if ( !pointInRing( ring, bend )
         || std::hypot( bend.x() - initial.bend.x(), bend.y() - initial.bend.y() ) > std::max( 0.50, extent * 0.15 ) )
      return result;

    auto fittedEnd = [&]( double dirX, double dirY, const QgsPointXY &initialEnd, QgsPointXY &end ) {
      if ( dirX * ( initialEnd.x() - bend.x() ) + dirY * ( initialEnd.y() - bend.y() ) < 0.0 )
      {
        dirX = -dirX;
        dirY = -dirY;
      }
      LineInterval interval;
      if ( !findIntervalContaining( lineInsideRingIntervals( ring, bend, dirX, dirY ), 0.0, 0.0, interval )
           || interval.end <= std::max( 1e-6, extent * 0.01 ) )
        return false;
      end = pointOnLine( bend, dirX, dirY, interval.end );
      return segmentInsideRing( ring, bend, end );
    };
    if ( !fittedEnd( primary.dirX, primary.dirY, initial.primaryEnd, result.primaryEnd )
         || !fittedEnd( secondary.dirX, secondary.dirY, initial.secondaryEnd, result.secondaryEnd ) )
      return result;
    result.bend = bend;
    const double primaryLength = std::hypot( result.primaryEnd.x() - bend.x(), result.primaryEnd.y() - bend.y() );
    const double secondaryLength = std::hypot( result.secondaryEnd.x() - bend.x(), result.secondaryEnd.y() - bend.y() );
    result.splitNormalX = ( result.primaryEnd.x() - bend.x() ) / primaryLength - ( result.secondaryEnd.x() - bend.x() ) / secondaryLength;
    result.splitNormalY = ( result.primaryEnd.y() - bend.y() ) / primaryLength - ( result.secondaryEnd.y() - bend.y() ) / secondaryLength;
    const double normalLength = std::hypot( result.splitNormalX, result.splitNormalY );
    if ( normalLength <= 1e-8 )
      return BentGableLayout();
    result.splitNormalX /= normalLength;
    result.splitNormalY /= normalLength;
    result.success = true;
    return result;
  }

  bool leastSquaresBentGabledRidgePointFromPointCloud( const QVector<QgsPointXY> &inputRing, const QVector<BuildingRoof::RoofSample> &pointCloudSamples, const QgsPoint &boundaryPoint, QgsPoint &ridgePoint, double *ridgeDirX = nullptr, double *ridgeDirY = nullptr, double *boundaryHeightFromSlope = nullptr, BentGableLayout *fittedLayoutOut = nullptr )
  {
    double dirX = 0.0;
    double dirY = 0.0;
    if ( !nearestEdgeDirection( inputRing, boundaryPoint, dirX, dirY ) )
      return false;

    QgsPoint roughRidgePoint;
    if ( !topHeightBentRidgePointFromPointCloud( inputRing, pointCloudSamples, boundaryPoint, roughRidgePoint ) )
      return false;

    const BentGableLayout layout = findBentGabledLayout( inputRing, QgsPointXY( roughRidgePoint.x(), roughRidgePoint.y() ), dirX, dirY );
    if ( !layout.success )
      return false;

    QVector<QgsPointXY> ring = ringWithInsertedBoundaryPoints( inputRing, QVector<QgsPointXY>{ layout.primaryEnd, layout.secondaryEnd } );
    BentGableSegment primarySegment;
    BentGableSegment secondarySegment;
    if ( !makeBentSegment( layout.primaryEnd, layout.bend, roughRidgePoint.z(), ring, boundaryPoint, primarySegment ) )
      return false;
    if ( !makeBentSegment( layout.bend, layout.secondaryEnd, roughRidgePoint.z(), ring, boundaryPoint, secondarySegment ) )
      return false;

    const AutoHeightBand heightBand = inferAutoGabledHeightBand( ring, pointCloudSamples );
    if ( !heightBand.success || heightBand.filteredHeights.size() < 20 )
      return false;

    const double roofFloor = std::max( sortedPercentile( heightBand.filteredHeights, 0.35 ),
                                      heightBand.baseHeight + std::max( 0.20, ( heightBand.ridgeHeight - heightBand.baseHeight ) * 0.16 ) );
    const QgsPointXY splitXY( layout.bend.x(), layout.bend.y() );
    const double primarySide = signedDistanceToLine( layout.primaryEnd, QgsPoint( splitXY.x(), splitXY.y(), roughRidgePoint.z() ), layout.splitNormalX, layout.splitNormalY );
    if ( std::fabs( primarySide ) <= 1e-8 )
      return false;

    QVector<QgsPoint> primaryPoints;
    QVector<QgsPoint> secondaryPoints;
    primaryPoints.reserve( pointCloudSamples.size() );
    secondaryPoints.reserve( pointCloudSamples.size() );
    for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
    {
      const QgsPoint &point = sample.point;
      const QgsPointXY pointXY( point.x(), point.y() );
      if ( point.z() < roofFloor || point.z() < heightBand.lowHeight || point.z() > heightBand.highHeight )
        continue;
      if ( !pointInRing( ring, pointXY ) )
        continue;

      const double splitSide = signedDistanceToLine( pointXY, QgsPoint( splitXY.x(), splitXY.y(), roughRidgePoint.z() ), layout.splitNormalX, layout.splitNormalY );
      if ( splitSide * primarySide >= 0.0 )
        primaryPoints.append( point );
      else
        secondaryPoints.append( point );
    }

    const LeastSquaresBentSegmentRidge primaryRidge = fitLeastSquaresBentSegmentRidge( ring, primarySegment, primaryPoints, boundaryPoint );
    const LeastSquaresBentSegmentRidge secondaryRidge = fitLeastSquaresBentSegmentRidge( ring, secondarySegment, secondaryPoints, boundaryPoint );
    if ( !primaryRidge.success || !secondaryRidge.success )
      return false;

    const BentGableLayout fittedLayout = joinFittedBentRidges( inputRing, layout, primaryRidge, secondaryRidge );
    if ( !fittedLayout.success )
      return false;

    const double weightedHeight = ( primaryRidge.point.z() * primaryRidge.supportCount + secondaryRidge.point.z() * secondaryRidge.supportCount )
                                  / std::max( 1, primaryRidge.supportCount + secondaryRidge.supportCount );
    if ( weightedHeight <= heightBand.baseHeight + std::max( 0.20, heightBand.binWidth ) )
      return false;

    ridgePoint = QgsPoint( primaryRidge.point.x(), primaryRidge.point.y(), weightedHeight );
    if ( fittedLayoutOut )
      *fittedLayoutOut = fittedLayout;
    if ( ridgeDirX )
      *ridgeDirX = primaryRidge.dirX;
    if ( ridgeDirY )
      *ridgeDirY = primaryRidge.dirY;
    if ( boundaryHeightFromSlope )
    {
      double heightSum = 0.0;
      int heightWeight = 0;
      if ( primaryRidge.boundaryHeightSuccess )
      {
        heightSum += primaryRidge.boundaryHeight * primaryRidge.supportCount;
        heightWeight += primaryRidge.supportCount;
      }
      if ( secondaryRidge.boundaryHeightSuccess )
      {
        heightSum += secondaryRidge.boundaryHeight * secondaryRidge.supportCount;
        heightWeight += secondaryRidge.supportCount;
      }
      if ( heightWeight > 0 )
        *boundaryHeightFromSlope = heightSum / heightWeight;
    }
    return true;
  }

  QList<BuildingRoof::RoofPoint> topHeightGabledRoofPoints( const QVector<QgsPointXY> &ring, const QList<BuildingRoof::RoofPoint> &boundaries, const QList<BuildingRoof::RoofPoint> &ridges, const QVector<BuildingRoof::RoofSample> &pointCloudSamples, GabledBoundarySlopePlanes *slopePlanesOut = nullptr, double *ridgeDirXOut = nullptr, double *ridgeDirYOut = nullptr, GabledFitContext *fitContext = nullptr )
  {
    QList<BuildingRoof::RoofPoint> points;
    if ( boundaries.size() != 1 || ( ridges.size() != 1 && ridges.size() != 3 ) )
      return points;

    QgsPoint ridgePoint;
    double ridgeDirX = 0.0;
    double ridgeDirY = 0.0;
    GabledBoundarySlopePlanes boundarySlopePlanes;
    BentGableLayout bentLayout;
    double bentBoundaryHeightFromSlope = std::numeric_limits<double>::quiet_NaN();
    bool ridgePointFound = false;
    if ( ridges.size() == 3 )
    {
      ridgePointFound = leastSquaresBentGabledRidgePointFromPointCloud( ring, pointCloudSamples, boundaries.first().point, ridgePoint, &ridgeDirX, &ridgeDirY, &bentBoundaryHeightFromSlope, &bentLayout );
      if ( !ridgePointFound )
      {
        ridgePointFound = topHeightBentRidgePointFromPointCloud( ring, pointCloudSamples, boundaries.first().point, ridgePoint, &ridgeDirX, &ridgeDirY );
        if ( ridgePointFound )
          bentLayout = findBentGabledLayout( ring, QgsPointXY( ridgePoint.x(), ridgePoint.y() ), ridgeDirX, ridgeDirY );
      }
      ridgePointFound = ridgePointFound && bentLayout.success;
    }
    else
    {
      ridgePointFound = leastSquaresGabledRidgePointFromPointCloud( ring, pointCloudSamples, boundaries.first().point, ridgePoint, &ridgeDirX, &ridgeDirY, &boundarySlopePlanes, fitContext );
      if ( !ridgePointFound )
        ridgePointFound = topHeightRidgePointFromPointCloud( ring, pointCloudSamples, boundaries.first().point, ridgePoint, &ridgeDirX, &ridgeDirY );
    }
    if ( !ridgePointFound )
      return points;

    QgsPoint boundaryPoint = boundaries.first().point;
    QgsPoint automaticBoundaryPoint = boundaryPoint;
    if ( automaticBoundaryPointFromRidgeLine( ring, pointCloudSamples, ridgePoint, ridgeDirX, ridgeDirY, boundaries.first().point, automaticBoundaryPoint ) )
    {
      if ( ridges.size() == 3 )
        boundaryPoint = QgsPoint( boundaries.first().point.x(), boundaries.first().point.y(), automaticBoundaryPoint.z() );
      else
        boundaryPoint = automaticBoundaryPoint;
    }
    else
    {
      boundaryPoint = boundaries.first().point;
    }

    double slopeBoundaryHeight = std::numeric_limits<double>::quiet_NaN();
    if ( ridges.size() == 3 )
    {
      slopeBoundaryHeight = bentBoundaryHeightFromSlope;
    }
    else if ( boundarySlopePlanes.success )
    {
      slopeBoundaryHeight = chooseBoundarySlopeHeight( boundarySlopePlanes, boundaryPoint, ridgePoint, ringExtentSize( ring ) );
    }
    refineBoundaryPointHeightFromSlopePlane( boundaryPoint, slopeBoundaryHeight, ridgePoint, ringExtentSize( ring ) );

    points.append( BuildingRoof::RoofPoint{ boundaryPoint, boundaries.first().type } );
    if ( ridges.size() == 3 )
    {
      const QgsPointXY ridgePositions[3] = { bentLayout.primaryEnd, bentLayout.bend, bentLayout.secondaryEnd };
      for ( int i = 0; i < 3; ++i )
        points.append( BuildingRoof::RoofPoint{ QgsPoint( ridgePositions[i].x(), ridgePositions[i].y(), ridgePoint.z() ), ridges.at( i ).type } );
    }
    else
      points.append( BuildingRoof::RoofPoint{ ridgePoint, ridges.first().type } );

    if ( slopePlanesOut && ridges.size() != 3 && boundarySlopePlanes.success )
      *slopePlanesOut = boundarySlopePlanes;
    if ( ridgeDirXOut )
      *ridgeDirXOut = ridgeDirX;
    if ( ridgeDirYOut )
      *ridgeDirYOut = ridgeDirY;
    return points;
  }

  double averagedHippedRidgeEndParameter( const QVector<HippedRidgeBandSample> &sortedSamples, bool lowerEnd, const QgsPointXY &lineOrigin, double dirX, double dirY, double fallbackT, double minT, double maxT )
  {
    if ( sortedSamples.size() < 6 )
      return std::max( minT, std::min( maxT, fallbackT ) );

    const int tailCount = std::min( sortedSamples.size() / 2, std::max( 4, static_cast<int>( std::ceil( sortedSamples.size() * 0.18 ) ) ) );
    if ( tailCount <= 0 )
      return std::max( minT, std::min( maxT, fallbackT ) );

    double sumX = 0.0;
    double sumY = 0.0;
    double weightSum = 0.0;
    for ( int i = 0; i < tailCount; ++i )
    {
      const int index = lowerEnd ? i : sortedSamples.size() - 1 - i;
      const HippedRidgeBandSample &sample = sortedSamples.at( index );
      const double endWeight = 1.0 + static_cast<double>( tailCount - i ) / tailCount;
      sumX += sample.point.x() * endWeight;
      sumY += sample.point.y() * endWeight;
      weightSum += endWeight;
    }

    if ( weightSum <= 0.0 )
      return std::max( minT, std::min( maxT, fallbackT ) );

    const QgsPointXY averagedPoint( sumX / weightSum, sumY / weightSum );
    const double averagedT = lineParameter( lineOrigin, dirX, dirY, averagedPoint );
    return std::max( minT, std::min( maxT, averagedT ) );
  }

  bool fitAutoHippedRidgeCandidate( const QVector<QgsPointXY> &ring, const QVector<QgsPoint> &highPoints, double dirX, double dirY, double fallbackRidgeHeight, AutoHippedRidge &candidate, double &score )
  {
    if ( highPoints.size() < 8 )
      return false;

    const double dirLength = std::hypot( dirX, dirY );
    if ( dirLength <= 1e-10 )
      return false;
    dirX /= dirLength;
    dirY /= dirLength;
    const double normalX = -dirY;
    const double normalY = dirX;
    const double extent = ringExtentSize( ring );

    QVector<double> offsets;
    offsets.reserve( highPoints.size() );
    for ( const QgsPoint &point : highPoints )
      offsets.append( point.x() * normalX + point.y() * normalY );
    std::sort( offsets.begin(), offsets.end() );

    const double minOffset = offsets.first();
    const double maxOffset = offsets.last();
    const double binWidth = std::max( 0.20, extent * 0.012 );
    const int binCount = std::max( 2, static_cast<int>( std::ceil( ( maxOffset - minOffset ) / binWidth ) ) + 1 );
    QVector<int> counts( binCount, 0 );
    for ( double offset : offsets )
    {
      const int index = std::max( 0, std::min( binCount - 1, static_cast<int>( std::floor( ( offset - minOffset ) / binWidth ) ) ) );
      ++counts[index];
    }

    int bestBin = -1;
    int bestScore = 0;
    for ( int i = 0; i < binCount; ++i )
    {
      const int previous = i > 0 ? counts.at( i - 1 ) : 0;
      const int next = i + 1 < binCount ? counts.at( i + 1 ) : 0;
      const int localScore = previous + counts.at( i ) + next;
      if ( localScore > bestScore )
      {
        bestScore = localScore;
        bestBin = i;
      }
    }

    const int minSupport = std::max( 6, highPoints.size() / 12 );
    if ( bestBin < 0 || bestScore < minSupport )
      return false;

    const double roughOffset = minOffset + ( bestBin + 0.5 ) * binWidth;
    const double tolerance = std::max( binWidth * 1.5, extent * 0.015 );
    QVector<double> parameters;
    QVector<double> heights;
    QVector<HippedRidgeBandSample> bandSamples;
    parameters.reserve( highPoints.size() );
    heights.reserve( highPoints.size() );
    bandSamples.reserve( highPoints.size() );
    double offsetSum = 0.0;
    int fitCount = 0;
    for ( const QgsPoint &point : highPoints )
    {
      const double offset = point.x() * normalX + point.y() * normalY;
      if ( std::fabs( offset - roughOffset ) > tolerance )
        continue;

      const double parameter = point.x() * dirX + point.y() * dirY;
      offsetSum += offset;
      parameters.append( parameter );
      heights.append( point.z() );
      bandSamples.append( HippedRidgeBandSample{ parameter, QgsPointXY( point.x(), point.y() ), point.z() } );
      ++fitCount;
    }

    if ( fitCount < minSupport || parameters.size() < 6 )
      return false;

    std::sort( parameters.begin(), parameters.end() );
    std::sort( heights.begin(), heights.end() );
    std::sort( bandSamples.begin(), bandSamples.end(), []( const HippedRidgeBandSample &left, const HippedRidgeBandSample &right ) {
      return left.t < right.t;
    } );
    const double centerOffset = offsetSum / fitCount;
    const QgsPointXY lineOrigin( normalX * centerOffset, normalY * centerOffset );
    const double preferredT = sortedPercentile( parameters, 0.50 );
    const QVector<LineInterval> intervals = lineInsideRingIntervals( ring, lineOrigin, dirX, dirY );
    if ( intervals.isEmpty() )
      return false;

    LineInterval bestInterval = intervals.first();
    bool foundPreferredInterval = false;
    for ( const LineInterval &interval : intervals )
    {
      if ( intervalContains( interval, preferredT ) )
      {
        bestInterval = interval;
        foundPreferredInterval = true;
        break;
      }
      if ( !foundPreferredInterval && interval.end - interval.start > bestInterval.end - bestInterval.start )
        bestInterval = interval;
    }

    const double intervalLength = bestInterval.end - bestInterval.start;
    if ( intervalLength < std::max( 1.0, extent * 0.12 ) )
      return false;

    const double trim = std::min( intervalLength * 0.24, std::max( 0.35, extent * 0.08 ) );
    const double minT = bestInterval.start + trim;
    const double maxT = bestInterval.end - trim;
    if ( maxT <= minT + 1e-6 )
      return false;

    const double fallbackStartT = sortedPercentile( parameters, 0.12 );
    const double fallbackEndT = sortedPercentile( parameters, 0.88 );
    double startT = averagedHippedRidgeEndParameter( bandSamples, true, lineOrigin, dirX, dirY, fallbackStartT, minT, maxT );
    double endT = averagedHippedRidgeEndParameter( bandSamples, false, lineOrigin, dirX, dirY, fallbackEndT, minT, maxT );
    if ( endT < startT )
      std::swap( startT, endT );

    const double minRidgeLength = std::max( 0.70, intervalLength * 0.12 );
    if ( endT - startT < minRidgeLength )
    {
      const double centerT = std::max( minT, std::min( maxT, preferredT ) );
      startT = std::max( minT, centerT - minRidgeLength * 0.5 );
      endT = std::min( maxT, centerT + minRidgeLength * 0.5 );
      if ( endT - startT < minRidgeLength * 0.65 )
        return false;
    }

    const double maxRidgeLength = ( maxT - minT ) * 0.96;
    if ( endT - startT > maxRidgeLength )
    {
      const double centerT = ( startT + endT ) * 0.5;
      startT = std::max( minT, centerT - maxRidgeLength * 0.5 );
      endT = std::min( maxT, centerT + maxRidgeLength * 0.5 );
    }

    const double ridgeHeight = upperWeightedMean( heights, 0.55, 0.92, fallbackRidgeHeight );
    const QgsPointXY firstXY = pointOnLine( lineOrigin, dirX, dirY, startT );
    const QgsPointXY secondXY = pointOnLine( lineOrigin, dirX, dirY, endT );
    if ( !pointInRing( ring, firstXY ) || !pointInRing( ring, secondXY ) )
      return false;

    candidate.success = true;
    candidate.firstPoint = QgsPoint( firstXY.x(), firstXY.y(), ridgeHeight );
    candidate.secondPoint = QgsPoint( secondXY.x(), secondXY.y(), ridgeHeight );
    candidate.dirX = dirX;
    candidate.dirY = dirY;

    const double observedLength = sortedPercentile( parameters, 0.95 ) - sortedPercentile( parameters, 0.05 );
    const double ridgeLengthScore = std::max( 0.1, std::min( 1.0, ( endT - startT ) / std::max( intervalLength, 1e-8 ) ) );
    const double supportScore = static_cast<double>( fitCount ) / highPoints.size();
    score = fitCount * ridgeLengthScore * std::min( 2.0, observedLength / std::max( 1.0, extent * 0.15 ) ) * std::max( 0.25, supportScore );
    return true;
  }

  bool fitHippedRidgeCandidateOnKnownLine( const QVector<QgsPointXY> &ring, const QVector<QgsPoint> &roofPoints, const QVector<QgsPoint> &highPoints, const QgsPoint &ridgePoint, double dirX, double dirY, double fallbackRidgeHeight, AutoHippedRidge &candidate, double &score )
  {
    if ( highPoints.size() < 8 )
      return false;

    const double dirLength = std::hypot( dirX, dirY );
    if ( dirLength <= 1e-10 )
      return false;
    dirX /= dirLength;
    dirY /= dirLength;
    const double normalX = -dirY;
    const double normalY = dirX;
    const double extent = ringExtentSize( ring );
    const QgsPointXY lineOrigin( ridgePoint.x(), ridgePoint.y() );

    const QVector<LineInterval> intervals = lineInsideRingIntervals( ring, lineOrigin, dirX, dirY );
    if ( intervals.isEmpty() )
      return false;

    LineInterval bestInterval = intervals.first();
    bool foundZeroInterval = false;
    for ( const LineInterval &interval : intervals )
    {
      if ( intervalContains( interval, 0.0 ) )
      {
        bestInterval = interval;
        foundZeroInterval = true;
        break;
      }
      if ( !foundZeroInterval && interval.end - interval.start > bestInterval.end - bestInterval.start )
        bestInterval = interval;
    }

    const double intervalLength = bestInterval.end - bestInterval.start;
    if ( intervalLength < std::max( 1.0, extent * 0.12 ) )
      return false;

    const double trim = std::min( intervalLength * 0.24, std::max( 0.35, extent * 0.08 ) );
    const double minT = bestInterval.start + trim;
    const double maxT = bestInterval.end - trim;
    if ( maxT <= minT + 1e-6 )
      return false;

    QVector<double> parameters;
    QVector<double> heights;
    QVector<HippedRidgeBandSample> bandSamples;
    parameters.reserve( highPoints.size() );
    heights.reserve( highPoints.size() );
    bandSamples.reserve( highPoints.size() );
    const double tolerance = std::max( 0.24, std::min( 0.75, extent * 0.025 ) );
    for ( const QgsPoint &point : highPoints )
    {
      const QgsPointXY pointXY( point.x(), point.y() );
      const double sideDistance = signedDistanceToLine( pointXY, ridgePoint, normalX, normalY );
      if ( std::fabs( sideDistance ) > tolerance )
        continue;

      const double parameter = lineParameter( lineOrigin, dirX, dirY, pointXY );
      if ( parameter < bestInterval.start - tolerance || parameter > bestInterval.end + tolerance )
        continue;

      parameters.append( parameter );
      heights.append( point.z() );
      bandSamples.append( HippedRidgeBandSample{ parameter, pointXY, point.z() } );
    }

    const int minSupport = std::max( 6, highPoints.size() / 20 );
    if ( parameters.size() < minSupport || bandSamples.size() < 6 )
      return false;

    std::sort( parameters.begin(), parameters.end() );
    std::sort( heights.begin(), heights.end() );
    std::sort( bandSamples.begin(), bandSamples.end(), []( const HippedRidgeBandSample &left, const HippedRidgeBandSample &right ) {
      return left.t < right.t;
    } );

    const double preferredT = sortedPercentile( parameters, 0.50 );
    const double fallbackStartT = sortedPercentile( parameters, 0.12 );
    const double fallbackEndT = sortedPercentile( parameters, 0.88 );
    double startT = averagedHippedRidgeEndParameter( bandSamples, true, lineOrigin, dirX, dirY, fallbackStartT, minT, maxT );
    double endT = averagedHippedRidgeEndParameter( bandSamples, false, lineOrigin, dirX, dirY, fallbackEndT, minT, maxT );
    if ( endT < startT )
      std::swap( startT, endT );

    const double ridgeHeight = upperWeightedMean( heights, 0.55, 0.92, fallbackRidgeHeight );
    const double candidateHeight = std::isfinite( ridgePoint.z() ) && ridgePoint.z() > 0.0
                                     ? ridgePoint.z()
                                     : ridgeHeight;

    const double footprintHalfWidth = [&]() {
      double halfWidth = 0.0;
      for ( const QgsPointXY &ringPoint : ring )
        halfWidth = std::max( halfWidth, std::fabs( signedDistanceToLine( ringPoint, ridgePoint, normalX, normalY ) ) );
      return halfWidth;
    }();

    if ( roofPoints.size() >= 24 && footprintHalfWidth > 1e-8 )
    {
      const double endpointBand = std::max( 0.35, std::min( intervalLength * 0.18, extent * 0.10 ) );
      const double endFaceWidthTolerance = std::max( 0.20, footprintHalfWidth * 0.08 );
      QVector<QgsPoint> startEndPoints;
      QVector<QgsPoint> finishEndPoints;
      startEndPoints.reserve( roofPoints.size() / 2 );
      finishEndPoints.reserve( roofPoints.size() / 2 );

      for ( const QgsPoint &point : roofPoints )
      {
        const QgsPointXY pointXY( point.x(), point.y() );
        const double along = lineParameter( lineOrigin, dirX, dirY, pointXY );
        const double sideDistance = std::fabs( signedDistanceToLine( pointXY, ridgePoint, normalX, normalY ) );
        const double startFaceWidth = footprintHalfWidth * std::max( 0.0, std::min( 1.0, ( startT + endpointBand - along ) / ( std::max( 0.30, startT - bestInterval.start ) + endpointBand ) ) ) + endFaceWidthTolerance;
        const double finishFaceWidth = footprintHalfWidth * std::max( 0.0, std::min( 1.0, ( along - ( endT - endpointBand ) ) / ( std::max( 0.30, bestInterval.end - endT ) + endpointBand ) ) ) + endFaceWidthTolerance;
        if ( along >= bestInterval.start - endpointBand * 0.35 && along <= startT + endpointBand && sideDistance <= startFaceWidth )
          startEndPoints.append( point );
        if ( along >= endT - endpointBand && along <= bestInterval.end + endpointBand * 0.35 && sideDistance <= finishFaceWidth )
          finishEndPoints.append( point );
      }

      const int minEndPlaneCount = std::max( 8, static_cast<int>( std::ceil( roofPoints.size() * 0.035 ) ) );
      if ( startEndPoints.size() >= minEndPlaneCount && finishEndPoints.size() >= minEndPlaneCount )
      {
        const AreaFilteredRoofPlaneFit startFit = fitAreaFilteredRoofPlane( startEndPoints, 0.05 );
        const AreaFilteredRoofPlaneFit finishFit = fitAreaFilteredRoofPlane( finishEndPoints, 0.05 );
        const LeastSquaresRoofPlane &startPlane = startFit.plane;
        const LeastSquaresRoofPlane &finishPlane = finishFit.plane;
        const double startDenominator = startPlane.a * dirX + startPlane.b * dirY;
        const double finishDenominator = finishPlane.a * dirX + finishPlane.b * dirY;
        if ( startFit.success && finishFit.success
             && startDenominator > 1e-8 && finishDenominator < -1e-8 )
        {
          const double fittedStartT = ( candidateHeight - startPlane.a * lineOrigin.x() - startPlane.b * lineOrigin.y() - startPlane.c ) / startDenominator;
          const double fittedEndT = ( candidateHeight - finishPlane.a * lineOrigin.x() - finishPlane.b * lineOrigin.y() - finishPlane.c ) / finishDenominator;
          const double endpointTolerance = std::max( 0.45, std::min( intervalLength * 0.22, extent * 0.14 ) );
          if ( std::isfinite( fittedStartT ) && fittedStartT >= minT - endpointTolerance && fittedStartT <= maxT + endpointTolerance )
            startT = std::max( minT, std::min( maxT, fittedStartT * 0.80 + startT * 0.20 ) );
          if ( std::isfinite( fittedEndT ) && fittedEndT >= minT - endpointTolerance && fittedEndT <= maxT + endpointTolerance )
            endT = std::max( minT, std::min( maxT, fittedEndT * 0.80 + endT * 0.20 ) );
          if ( endT < startT )
            std::swap( startT, endT );

        }
      }
    }

    const double minRidgeLength = std::max( 0.70, intervalLength * 0.12 );
    if ( endT - startT < minRidgeLength )
    {
      const double centerT = std::max( minT, std::min( maxT, preferredT ) );
      startT = std::max( minT, centerT - minRidgeLength * 0.5 );
      endT = std::min( maxT, centerT + minRidgeLength * 0.5 );
      if ( endT - startT < minRidgeLength * 0.65 )
        return false;
    }

    const double maxRidgeLength = ( maxT - minT ) * 0.96;
    if ( endT - startT > maxRidgeLength )
    {
      const double centerT = ( startT + endT ) * 0.5;
      startT = std::max( minT, centerT - maxRidgeLength * 0.5 );
      endT = std::min( maxT, centerT + maxRidgeLength * 0.5 );
    }

    const QgsPointXY firstXY = pointOnLine( lineOrigin, dirX, dirY, startT );
    const QgsPointXY secondXY = pointOnLine( lineOrigin, dirX, dirY, endT );
    if ( !pointInRing( ring, firstXY ) || !pointInRing( ring, secondXY ) )
      return false;

    candidate.success = true;
    candidate.firstPoint = QgsPoint( firstXY.x(), firstXY.y(), candidateHeight );
    candidate.secondPoint = QgsPoint( secondXY.x(), secondXY.y(), candidate.firstPoint.z() );
    candidate.dirX = dirX;
    candidate.dirY = dirY;
    candidate.preferredGabledSeed = true;

    const double observedLength = sortedPercentile( parameters, 0.95 ) - sortedPercentile( parameters, 0.05 );
    const double ridgeLengthScore = std::max( 0.1, std::min( 1.0, ( endT - startT ) / std::max( intervalLength, 1e-8 ) ) );
    const double supportScore = static_cast<double>( parameters.size() ) / highPoints.size();
    score = parameters.size() * ridgeLengthScore * std::min( 2.0, observedLength / std::max( 1.0, extent * 0.15 ) ) * std::max( 0.25, supportScore ) + 1000.0;
    return true;
  }

  LeastSquaresRoofPlane averagedRoofPlane( const LeastSquaresRoofPlane &firstPlane, const LeastSquaresRoofPlane &secondPlane )
  {
    LeastSquaresRoofPlane plane;
    if ( !firstPlane.success || !secondPlane.success )
      return plane;

    plane.success = true;
    plane.a = ( firstPlane.a + secondPlane.a ) * 0.5;
    plane.b = ( firstPlane.b + secondPlane.b ) * 0.5;
    plane.c = ( firstPlane.c + secondPlane.c ) * 0.5;
    plane.rmse = ( firstPlane.rmse + secondPlane.rmse ) * 0.5;
    plane.count = firstPlane.count + secondPlane.count;
    return plane;
  }

  bool planeIntersectionParameterOnLine( const LeastSquaresRoofPlane &ridgePlane, const LeastSquaresRoofPlane &endPlane, const QgsPointXY &lineOrigin, double dirX, double dirY, double &t )
  {
    if ( !ridgePlane.success || !endPlane.success )
      return false;

    const double diffA = ridgePlane.a - endPlane.a;
    const double diffB = ridgePlane.b - endPlane.b;
    const double diffC = ridgePlane.c - endPlane.c;
    const double denominator = diffA * dirX + diffB * dirY;
    if ( std::fabs( denominator ) <= 1e-10 )
      return false;

    const double numerator = diffA * lineOrigin.x() + diffB * lineOrigin.y() + diffC;
    t = -numerator / denominator;
    return std::isfinite( t );
  }

  double averagedFiniteHeight( const QVector<double> &heights, double fallbackHeight )
  {
    double sum = 0.0;
    int count = 0;
    for ( double height : heights )
    {
      if ( std::isfinite( height ) && height > 0.0 )
      {
        sum += height;
        ++count;
      }
    }
    return count > 0 ? sum / count : fallbackHeight;
  }

  QVector<HippedNormalRoofSample> estimateHippedRoofNormalSamples( const QVector<QgsPoint> &roofPoints, double extent )
  {
    QVector<HippedNormalRoofSample> normalSamples;
    if ( roofPoints.size() < 16 )
      return normalSamples;

    const int maxCenters = 3600;
    const int centerStride = std::max( 1, static_cast<int>( std::ceil( static_cast<double>( roofPoints.size() ) / maxCenters ) ) );
    const double neighborRadius = std::min( 1.30, std::max( 0.45, extent * 0.024 ) );
    const double neighborRadius2 = neighborRadius * neighborRadius;
    const int minNeighbors = roofPoints.size() < 80 ? 7 : 9;
    normalSamples.reserve( std::min( roofPoints.size(), maxCenters ) );

    using Cell = std::pair<qint64, qint64>;
    const double originX = roofPoints.first().x();
    const double originY = roofPoints.first().y();
    const auto cellForPoint = [&]( const QgsPoint &point ) -> Cell {
      return { static_cast<qint64>( std::floor( ( point.x() - originX ) / neighborRadius ) ),
               static_cast<qint64>( std::floor( ( point.y() - originY ) / neighborRadius ) ) };
    };
    std::map<Cell, QVector<int>> grid;
    for ( int i = 0; i < roofPoints.size(); ++i )
      grid[cellForPoint( roofPoints.at( i ) )].append( i );

    for ( int i = 0; i < roofPoints.size(); i += centerStride )
    {
      const QgsPoint &center = roofPoints.at( i );
      const Cell cell = cellForPoint( center );
      QVector<int> neighborIndices;
      for ( int dx = -1; dx <= 1; ++dx )
      {
        for ( int dy = -1; dy <= 1; ++dy )
        {
          const auto found = grid.find( Cell{ cell.first + dx, cell.second + dy } );
          if ( found == grid.end() )
            continue;
          for ( int index : found->second )
          {
            const QgsPoint &neighbor = roofPoints.at( index );
            const double x = neighbor.x() - center.x();
            const double y = neighbor.y() - center.y();
            if ( x * x + y * y <= neighborRadius2 )
              neighborIndices.append( index );
          }
        }
      }
      // Preserve the original summation order in the least-squares fit.
      std::sort( neighborIndices.begin(), neighborIndices.end() );
      QVector<QgsPoint> neighbors;
      neighbors.reserve( neighborIndices.size() );
      for ( int index : neighborIndices )
        neighbors.append( roofPoints.at( index ) );

      if ( neighbors.size() < minNeighbors )
        continue;

      const LeastSquaresRoofPlane localPlane = fitLeastSquaresRoofPlane( neighbors );
      if ( !localPlane.success )
        continue;

      const double localThreshold = std::max( 0.12, std::min( 0.42, robustPlaneThreshold( neighbors ) * 1.35 ) );
      if ( localPlane.rmse > localThreshold )
        continue;

      double nx = -localPlane.a;
      double ny = -localPlane.b;
      double nz = 1.0;
      const double length = std::sqrt( nx * nx + ny * ny + nz * nz );
      if ( length <= 1e-10 )
        continue;
      nx /= length;
      ny /= length;
      nz /= length;
      if ( nz < 0.42 )
        continue;

      const double horizontalLength = std::hypot( nx, ny );
      if ( horizontalLength < 0.035 )
        continue;

      normalSamples.append( HippedNormalRoofSample{ center, nx / horizontalLength, ny / horizontalLength, nz } );
    }

    return normalSamples;
  }

  int hippedNormalGroup( const HippedNormalRoofSample &sample, double dirX, double dirY, double normalX, double normalY )
  {
    const double scores[4] = {
      sample.horizontalNormalX * normalX + sample.horizontalNormalY * normalY,
      -( sample.horizontalNormalX * normalX + sample.horizontalNormalY * normalY ),
      -( sample.horizontalNormalX * dirX + sample.horizontalNormalY * dirY ),
      sample.horizontalNormalX * dirX + sample.horizontalNormalY * dirY
    };

    int bestIndex = -1;
    double bestScore = 0.0;
    for ( int i = 0; i < 4; ++i )
    {
      if ( scores[i] > bestScore )
      {
        bestScore = scores[i];
        bestIndex = i;
      }
    }

    return bestScore >= 0.38 ? bestIndex : -1;
  }

  bool refineHippedRidgeWithLeastSquaresPlanes( const QVector<QgsPointXY> &ring, const QVector<BuildingRoof::RoofSample> &pointCloudSamples, const AutoHeightBand &heightBand, const AutoHippedRidge &roughRidge, AutoHippedRidge &refinedRidge )
  {
    if ( !roughRidge.success || ring.size() < 3 || pointCloudSamples.size() < 40 )
      return false;

    double dirX = roughRidge.dirX;
    double dirY = roughRidge.dirY;
    const double dirLength = std::hypot( dirX, dirY );
    if ( dirLength <= 1e-10 )
      return false;
    dirX /= dirLength;
    dirY /= dirLength;
    const double normalX = -dirY;
    const double normalY = dirX;
    const double extent = ringExtentSize( ring );

    QgsPointXY roughStart( roughRidge.firstPoint.x(), roughRidge.firstPoint.y() );
    QgsPointXY roughEnd( roughRidge.secondPoint.x(), roughRidge.secondPoint.y() );
    if ( lineParameter( roughStart, dirX, dirY, roughEnd ) < 0.0 )
      std::swap( roughStart, roughEnd );

    const double roughLength = lineParameter( roughStart, dirX, dirY, roughEnd );
    if ( roughLength < std::max( 0.70, extent * 0.08 ) )
      return false;

    const QVector<LineInterval> roughIntervals = lineInsideRingIntervals( ring, roughStart, dirX, dirY );
    if ( roughIntervals.isEmpty() )
      return false;

    const double roughMidT = roughLength * 0.5;
    LineInterval ridgeInterval = roughIntervals.first();
    bool foundInterval = false;
    for ( const LineInterval &interval : roughIntervals )
    {
      if ( intervalContains( interval, roughMidT ) )
      {
        ridgeInterval = interval;
        foundInterval = true;
        break;
      }
      if ( !foundInterval && interval.end - interval.start > ridgeInterval.end - ridgeInterval.start )
        ridgeInterval = interval;
    }

    const double intervalLength = ridgeInterval.end - ridgeInterval.start;
    if ( intervalLength < std::max( 1.0, extent * 0.12 ) )
      return false;

    const double trim = std::min( intervalLength * 0.24, std::max( 0.35, extent * 0.08 ) );
    const double minT = ridgeInterval.start + trim;
    const double maxT = ridgeInterval.end - trim;
    if ( maxT <= minT + 1e-6 )
      return false;

    const double roughRidgeHeight = ( roughRidge.firstPoint.z() + roughRidge.secondPoint.z() ) * 0.5;
    const double roofFloor = std::max( sortedPercentile( heightBand.filteredHeights, 0.35 ),
                                      heightBand.baseHeight + std::max( 0.20, ( roughRidgeHeight - heightBand.baseHeight ) * 0.16 ) );
    double footprintHalfWidth = 0.0;
    for ( const QgsPointXY &ringPoint : ring )
      footprintHalfWidth = std::max( footprintHalfWidth, std::fabs( signedDistanceToLine( ringPoint, QgsPoint( roughStart.x(), roughStart.y(), roughRidgeHeight ), normalX, normalY ) ) );
    if ( footprintHalfWidth <= 1e-8 )
      return false;

    const double ridgeWeightRadius = std::max( 0.28, std::min( 0.90, extent * 0.045 ) );
    QVector<double> rawInsideHeights;
    QVector<double> rawRidgeHeights;
    rawInsideHeights.reserve( pointCloudSamples.size() );
    rawRidgeHeights.reserve( pointCloudSamples.size() / 8 );
    for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
    {
      const QgsPoint &point = sample.point;
      if ( !std::isfinite( point.z() ) )
        continue;
      const QgsPointXY pointXY( point.x(), point.y() );
      if ( !pointInRing( ring, pointXY ) )
        continue;
      rawInsideHeights.append( point.z() );
      if ( pointSegmentDistance2( pointXY, roughStart, roughEnd ) <= std::pow( ridgeWeightRadius * 1.35, 2.0 ) )
        rawRidgeHeights.append( point.z() );
    }
    std::sort( rawInsideHeights.begin(), rawInsideHeights.end() );
    std::sort( rawRidgeHeights.begin(), rawRidgeHeights.end() );

    const double endpointInwardBand = std::max( 0.24, std::min( roughLength * 0.10, extent * 0.05 ) );
    const double endpointOuterTolerance = std::max( 0.15, extent * 0.015 );
    const double startSpan = std::max( 0.30, -ridgeInterval.start );
    const double finishSpan = std::max( 0.30, ridgeInterval.end - roughLength );
    const double endFaceWidthTolerance = std::max( 0.20, footprintHalfWidth * 0.08 );
    const double sideStart = std::min( roughLength * 0.35, std::max( endpointInwardBand * 1.60, extent * 0.06 ) );
    const double sideEnd = roughLength - sideStart;

    QVector<QgsPoint> positiveSidePoints;
    QVector<QgsPoint> negativeSidePoints;
    QVector<QgsPoint> startEndPoints;
    QVector<QgsPoint> finishEndPoints;
    QVector<QgsPoint> roofCandidatePoints;
    roofCandidatePoints.reserve( pointCloudSamples.size() );

    for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
    {
      const QgsPoint &point = sample.point;
      if ( point.z() < roofFloor || point.z() < heightBand.lowHeight || point.z() > heightBand.highHeight )
        continue;
      const QgsPointXY pointXY( point.x(), point.y() );
      if ( !pointInRing( ring, pointXY ) )
        continue;

      roofCandidatePoints.append( point );
    }

    if ( roofCandidatePoints.size() < 40 )
      return false;

    const QVector<HippedNormalRoofSample> normalSamples = estimateHippedRoofNormalSamples( roofCandidatePoints, extent );
    if ( normalSamples.size() < 24 )
      return false;

    positiveSidePoints.reserve( normalSamples.size() );
    negativeSidePoints.reserve( normalSamples.size() );
    startEndPoints.reserve( normalSamples.size() );
    finishEndPoints.reserve( normalSamples.size() );

    for ( const HippedNormalRoofSample &sample : normalSamples )
    {
      const QgsPoint &point = sample.point;
      const QgsPointXY pointXY( point.x(), point.y() );
      const double along = lineParameter( roughStart, dirX, dirY, pointXY );
      const double sideDistance = signedDistanceToLine( pointXY, QgsPoint( roughStart.x(), roughStart.y(), roughRidgeHeight ), normalX, normalY );
      const double absSideDistance = std::fabs( sideDistance );
      const double startFaceWidth = footprintHalfWidth * std::max( 0.0, std::min( 1.0, ( endpointInwardBand - along ) / ( startSpan + endpointInwardBand ) ) ) + endFaceWidthTolerance;
      const double finishFaceWidth = footprintHalfWidth * std::max( 0.0, std::min( 1.0, ( along - ( roughLength - endpointInwardBand ) ) / ( finishSpan + endpointInwardBand ) ) ) + endFaceWidthTolerance;
      const int normalGroup = hippedNormalGroup( sample, dirX, dirY, normalX, normalY );
      if ( normalGroup == 2 && along >= ridgeInterval.start - endpointOuterTolerance && along <= endpointInwardBand && absSideDistance <= startFaceWidth )
        startEndPoints.append( point );
      if ( normalGroup == 3 && along >= roughLength - endpointInwardBand && along <= ridgeInterval.end + endpointOuterTolerance && absSideDistance <= finishFaceWidth )
        finishEndPoints.append( point );
      if ( along >= sideStart && along <= sideEnd )
      {
        if ( normalGroup == 0 && sideDistance >= 0.0 )
          positiveSidePoints.append( point );
        else if ( normalGroup == 1 && sideDistance < 0.0 )
          negativeSidePoints.append( point );
      }
    }

    const int minSideCount = std::max( 12, static_cast<int>( std::ceil( ( positiveSidePoints.size() + negativeSidePoints.size() ) * 0.10 ) ) );
    if ( positiveSidePoints.size() < minSideCount || negativeSidePoints.size() < minSideCount || startEndPoints.size() < 10 || finishEndPoints.size() < 10 )
      return false;

    const AreaFilteredRoofPlaneFit positiveFit = fitAreaFilteredRoofPlane( positiveSidePoints );
    const AreaFilteredRoofPlaneFit negativeFit = fitAreaFilteredRoofPlane( negativeSidePoints );
    const AreaFilteredRoofPlaneFit startFit = fitAreaFilteredRoofPlane( startEndPoints, 0.05 );
    const AreaFilteredRoofPlaneFit finishFit = fitAreaFilteredRoofPlane( finishEndPoints, 0.05 );
    if ( !positiveFit.success || !negativeFit.success || !startFit.success || !finishFit.success )
      return false;
    positiveSidePoints = positiveFit.points;
    negativeSidePoints = negativeFit.points;
    startEndPoints = startFit.points;
    finishEndPoints = finishFit.points;
    LeastSquaresRoofPlane positivePlane = positiveFit.plane;
    LeastSquaresRoofPlane negativePlane = negativeFit.plane;
    LeastSquaresRoofPlane startPlane = startFit.plane;
    LeastSquaresRoofPlane finishPlane = finishFit.plane;
    double startSlopeAlong = startPlane.a * dirX + startPlane.b * dirY;
    double finishSlopeAlong = finishPlane.a * dirX + finishPlane.b * dirY;
    if ( startSlopeAlong <= 1e-5 || finishSlopeAlong >= -1e-5 )
      return false;
    double positiveSideSlope = positivePlane.a * normalX + positivePlane.b * normalY;
    double negativeSideSlope = negativePlane.a * normalX + negativePlane.b * normalY;
    if ( positiveSideSlope >= -1e-5 || negativeSideSlope <= 1e-5 )
      return false;

    const double sideLineA = positivePlane.a - negativePlane.a;
    const double sideLineB = positivePlane.b - negativePlane.b;
    const double sideLineLength = std::hypot( sideLineA, sideLineB );
    if ( sideLineLength <= 1e-10 )
      return false;

    double fittedDirX = -sideLineB / sideLineLength;
    double fittedDirY = sideLineA / sideLineLength;
    const double alignment = std::fabs( fittedDirX * dirX + fittedDirY * dirY );
    if ( alignment < 0.45 )
      return false;

    const QgsPointXY roughLineOrigin = roughStart;
    const double ridgeDirX = dirX;
    const double ridgeDirY = dirY;
    const double roughStartT = 0.0;
    const double roughEndT = roughLength;

    const LeastSquaresRoofPlane ridgePlane = averagedRoofPlane( positivePlane, negativePlane );
    double startT = std::max( minT, std::min( maxT, roughStartT ) );
    double endT = std::max( minT, std::min( maxT, roughEndT ) );
    double planeStartT = startT;
    double planeEndT = endT;
    const double maxEndpointMove = std::max( 0.45, std::min( roughLength * 0.22, extent * 0.12 ) );
    const double maxLineMove = std::max( 0.25, std::min( footprintHalfWidth * 0.28, extent * 0.10 ) );
    double seedLineOffset = 0.0;
    double preferredLineOffset = 0.0;
    const double sideOffsetDenominator = sideLineA * normalX + sideLineB * normalY;
    if ( std::fabs( sideOffsetDenominator ) > 1e-10 )
    {
      const QgsPointXY roughMidPoint = pointOnLine( roughLineOrigin, ridgeDirX, ridgeDirY, roughMidT );
      const double sideOffsetNumerator = sideLineA * roughMidPoint.x() + sideLineB * roughMidPoint.y() + positivePlane.c - negativePlane.c;
      const double fittedLineOffset = -sideOffsetNumerator / sideOffsetDenominator;
      if ( std::isfinite( fittedLineOffset ) && std::fabs( fittedLineOffset ) <= maxLineMove )
      {
        preferredLineOffset = fittedLineOffset;
        seedLineOffset = preferredLineOffset;
      }
    }

    QVector<double> seedLineOffsets;
    seedLineOffsets.reserve( 5 );
    auto appendSeedLineOffset = [&]( double offset ) {
      if ( !std::isfinite( offset ) )
        return;
      offset = std::max( -maxLineMove, std::min( maxLineMove, offset ) );
      for ( double existingOffset : seedLineOffsets )
      {
        if ( std::fabs( existingOffset - offset ) <= std::max( 0.03, maxLineMove * 0.08 ) )
          return;
      }
      seedLineOffsets.append( offset );
    };
    appendSeedLineOffset( seedLineOffset );
    appendSeedLineOffset( 0.0 );

    QVector<double> highPointLineOffsets;
    highPointLineOffsets.reserve( roofCandidatePoints.size() );
    const double highLineThreshold = std::max( sortedPercentile( heightBand.filteredHeights, 0.82 ),
                                              heightBand.ridgeHeight - std::max( heightBand.binWidth * 1.5, 0.25 ) );
    const double highLineStart = std::max( ridgeInterval.start, -maxEndpointMove );
    const double highLineEnd = std::min( ridgeInterval.end, roughLength + maxEndpointMove );
    for ( const QgsPoint &point : roofCandidatePoints )
    {
      if ( point.z() < highLineThreshold )
        continue;
      const QgsPointXY pointXY( point.x(), point.y() );
      const double along = lineParameter( roughLineOrigin, ridgeDirX, ridgeDirY, pointXY );
      if ( along < highLineStart || along > highLineEnd )
        continue;
      const double lineOffset = signedDistanceToLine( pointXY, QgsPoint( roughLineOrigin.x(), roughLineOrigin.y(), roughRidgeHeight ), normalX, normalY );
      if ( std::fabs( lineOffset ) <= maxLineMove )
        highPointLineOffsets.append( lineOffset );
    }
    if ( highPointLineOffsets.size() >= 6 )
    {
      std::sort( highPointLineOffsets.begin(), highPointLineOffsets.end() );
      appendSeedLineOffset( sortedPercentile( highPointLineOffsets, 0.50 ) );
      appendSeedLineOffset( sortedPercentile( highPointLineOffsets, 0.40 ) );
      appendSeedLineOffset( sortedPercentile( highPointLineOffsets, 0.60 ) );
    }

    QgsPointXY lineOrigin( roughLineOrigin.x() + seedLineOffset * normalX,
                           roughLineOrigin.y() + seedLineOffset * normalY );
    const double expandedMinT = minT - maxEndpointMove;
    const double expandedMaxT = maxT + maxEndpointMove;
    double preferredStartT = startT;
    double preferredEndT = endT;
    if ( planeIntersectionParameterOnLine( ridgePlane, startPlane, lineOrigin, ridgeDirX, ridgeDirY, planeStartT )
         && planeStartT >= expandedMinT && planeStartT <= expandedMaxT
         && std::fabs( planeStartT - roughStartT ) <= maxEndpointMove )
    {
      preferredStartT = std::max( minT, std::min( maxT, planeStartT ) );
      startT = std::max( minT, std::min( maxT, planeStartT * 0.75 + roughStartT * 0.25 ) );
    }
    if ( planeIntersectionParameterOnLine( ridgePlane, finishPlane, lineOrigin, ridgeDirX, ridgeDirY, planeEndT )
         && planeEndT >= expandedMinT && planeEndT <= expandedMaxT
         && std::fabs( planeEndT - roughEndT ) <= maxEndpointMove )
    {
      preferredEndT = std::max( minT, std::min( maxT, planeEndT ) );
      endT = std::max( minT, std::min( maxT, planeEndT * 0.75 + roughEndT * 0.25 ) );
    }

    double maxHeight = std::min( heightBand.highHeight, heightBand.ridgeHeight + std::max( 0.35, heightBand.binWidth * 2.0 ) );
    if ( rawRidgeHeights.size() >= 8 && rawInsideHeights.size() >= 20 )
    {
      const double rawRidgeUpper = sortedPercentile( rawRidgeHeights, 0.90 );
      const double rawGlobalCeiling = sortedPercentile( rawInsideHeights, 0.995 );
      maxHeight = std::max( maxHeight, std::min( rawRidgeUpper, rawGlobalCeiling ) );
    }
    const double minHeight = heightBand.baseHeight + std::max( 0.25, ( heightBand.ridgeHeight - heightBand.baseHeight ) * 0.40 );
    const double robustDelta = std::max( 0.16, std::min( 0.55, ( maxHeight - minHeight ) * 0.12 ) );
    auto huberLoss = [robustDelta]( double residual ) {
      const double absResidual = std::fabs( residual );
      if ( absResidual <= robustDelta )
        return 0.5 * absResidual * absResidual;
      return robustDelta * ( absResidual - 0.5 * robustDelta );
    };

    auto constrainedRidgeHeight = [&]( const QgsPointXY &candidateLineOrigin, double candidateStartT, double candidateEndT ) {
      const QgsPointXY candidateFirstXY = pointOnLine( candidateLineOrigin, ridgeDirX, ridgeDirY, candidateStartT );
      const QgsPointXY candidateSecondXY = pointOnLine( candidateLineOrigin, ridgeDirX, ridgeDirY, candidateEndT );
      QVector<double> heights;
      heights.reserve( positiveSidePoints.size() + negativeSidePoints.size() + startEndPoints.size() + finishEndPoints.size() );
      auto appendHeights = [&]( const QVector<QgsPoint> &points, int group ) {
        for ( const QgsPoint &point : points )
        {
          const QgsPointXY pointXY( point.x(), point.y() );
          double delta = 0.0;
          if ( group == 0 )
          {
            const double sideDistance = signedDistanceToLine( pointXY, QgsPoint( candidateLineOrigin.x(), candidateLineOrigin.y(), roughRidgeHeight ), normalX, normalY );
            const QgsPointXY ridgeProjection( pointXY.x() - sideDistance * normalX, pointXY.y() - sideDistance * normalY );
            delta = roofPlaneZ( positivePlane, pointXY ) - roofPlaneZ( positivePlane, ridgeProjection );
          }
          else if ( group == 1 )
          {
            const double sideDistance = signedDistanceToLine( pointXY, QgsPoint( candidateLineOrigin.x(), candidateLineOrigin.y(), roughRidgeHeight ), normalX, normalY );
            const QgsPointXY ridgeProjection( pointXY.x() - sideDistance * normalX, pointXY.y() - sideDistance * normalY );
            delta = roofPlaneZ( negativePlane, pointXY ) - roofPlaneZ( negativePlane, ridgeProjection );
          }
          else if ( group == 2 )
            delta = roofPlaneZ( startPlane, pointXY ) - roofPlaneZ( startPlane, candidateFirstXY );
          else
            delta = roofPlaneZ( finishPlane, pointXY ) - roofPlaneZ( finishPlane, candidateSecondXY );
          const double height = point.z() - delta;
          if ( std::isfinite( height ) )
            heights.append( height );
        }
      };
      appendHeights( positiveSidePoints, 0 );
      appendHeights( negativeSidePoints, 1 );
      appendHeights( startEndPoints, 2 );
      appendHeights( finishEndPoints, 3 );
      if ( heights.size() < 24 )
        return roughRidgeHeight;

      QVector<double> nearRidgeHeights;
      nearRidgeHeights.reserve( roofCandidatePoints.size() );
      const double nearRidgeHalfWidth = std::max( 0.20, std::min( footprintHalfWidth * 0.16, extent * 0.045 ) );
      const double nearRidgeEndTolerance = std::max( 0.20, extent * 0.015 );
      for ( const QgsPoint &point : roofCandidatePoints )
      {
        const QgsPointXY pointXY( point.x(), point.y() );
        const double along = lineParameter( candidateLineOrigin, ridgeDirX, ridgeDirY, pointXY );
        if ( along < candidateStartT - nearRidgeEndTolerance || along > candidateEndT + nearRidgeEndTolerance )
          continue;
        const double sideDistance = signedDistanceToLine( pointXY, QgsPoint( candidateLineOrigin.x(), candidateLineOrigin.y(), roughRidgeHeight ), normalX, normalY );
        if ( std::fabs( sideDistance ) <= nearRidgeHalfWidth && point.z() >= minHeight )
          nearRidgeHeights.append( point.z() );
      }

      std::sort( heights.begin(), heights.end() );
      const double low = sortedPercentile( heights, 0.25 );
      const double high = sortedPercentile( heights, 0.75 );
      double sum = 0.0;
      int count = 0;
      for ( double height : heights )
      {
        if ( height >= low && height <= high )
        {
          sum += height;
          ++count;
        }
      }
      double fittedHeight = count > 0 ? sum / count : sortedPercentile( heights, 0.50 );
      if ( nearRidgeHeights.size() >= 6 )
      {
        std::sort( nearRidgeHeights.begin(), nearRidgeHeights.end() );
        const double ridgeSupportHeight = sortedPercentile( nearRidgeHeights, 0.74 );
        if ( std::isfinite( ridgeSupportHeight ) && ridgeSupportHeight > fittedHeight )
          fittedHeight = fittedHeight * 0.65 + ridgeSupportHeight * 0.35;
      }
      return std::max( minHeight, std::min( maxHeight, fittedHeight ) );
    };

    auto scoreConstrainedModel = [&]( double candidateLineOffset, double candidateStartT, double candidateEndT, HippedConstrainedRidgeFit &fit ) {
      if ( std::fabs( candidateLineOffset ) > maxLineMove )
        return false;
      if ( candidateEndT <= candidateStartT )
        return false;

      const QgsPointXY candidateLineOrigin( roughLineOrigin.x() + candidateLineOffset * normalX,
                                            roughLineOrigin.y() + candidateLineOffset * normalY );
      const QgsPointXY candidateFirstXY = pointOnLine( candidateLineOrigin, ridgeDirX, ridgeDirY, candidateStartT );
      const QgsPointXY candidateSecondXY = pointOnLine( candidateLineOrigin, ridgeDirX, ridgeDirY, candidateEndT );
      if ( !pointInRing( ring, candidateFirstXY ) || !pointInRing( ring, candidateSecondXY ) )
        return false;

      const double candidateLength = candidateEndT - candidateStartT;
      const double minLength = std::max( 0.70, intervalLength * 0.10 );
      const double maxLength = ( maxT - minT ) * 0.98;
      if ( candidateLength < minLength || candidateLength > maxLength )
        return false;

      const double candidateRidgeHeight = constrainedRidgeHeight( candidateLineOrigin, candidateStartT, candidateEndT );
      if ( !std::isfinite( candidateRidgeHeight ) || candidateRidgeHeight <= heightBand.baseHeight + 0.20 )
        return false;

      double loss = 0.0;
      int count = 0;
      auto addResiduals = [&]( const QVector<QgsPoint> &points, int group ) {
        for ( const QgsPoint &point : points )
        {
          const QgsPointXY pointXY( point.x(), point.y() );
          double predictedZ = candidateRidgeHeight;
          if ( group == 0 )
          {
            const double sideDistance = signedDistanceToLine( pointXY, QgsPoint( candidateLineOrigin.x(), candidateLineOrigin.y(), candidateRidgeHeight ), normalX, normalY );
            const QgsPointXY ridgeProjection( pointXY.x() - sideDistance * normalX, pointXY.y() - sideDistance * normalY );
            predictedZ += roofPlaneZ( positivePlane, pointXY ) - roofPlaneZ( positivePlane, ridgeProjection );
          }
          else if ( group == 1 )
          {
            const double sideDistance = signedDistanceToLine( pointXY, QgsPoint( candidateLineOrigin.x(), candidateLineOrigin.y(), candidateRidgeHeight ), normalX, normalY );
            const QgsPointXY ridgeProjection( pointXY.x() - sideDistance * normalX, pointXY.y() - sideDistance * normalY );
            predictedZ += roofPlaneZ( negativePlane, pointXY ) - roofPlaneZ( negativePlane, ridgeProjection );
          }
          else if ( group == 2 )
            predictedZ += roofPlaneZ( startPlane, pointXY ) - roofPlaneZ( startPlane, candidateFirstXY );
          else
            predictedZ += roofPlaneZ( finishPlane, pointXY ) - roofPlaneZ( finishPlane, candidateSecondXY );

          if ( !std::isfinite( predictedZ ) )
            continue;
          loss += huberLoss( point.z() - predictedZ );
          ++count;
        }
      };
      addResiduals( positiveSidePoints, 0 );
      addResiduals( negativeSidePoints, 1 );
      addResiduals( startEndPoints, 2 );
      addResiduals( finishEndPoints, 3 );
      if ( count < 24 )
        return false;

      const double startMove = ( candidateStartT - preferredStartT ) / std::max( maxEndpointMove, 1e-6 );
      const double endMove = ( candidateEndT - preferredEndT ) / std::max( maxEndpointMove, 1e-6 );
      const double lineMove = ( candidateLineOffset - preferredLineOffset ) / std::max( maxLineMove, 1e-6 );
      const double heightMove = ( candidateRidgeHeight - roughRidgeHeight ) / std::max( maxHeight - minHeight, 1e-6 );
      double score = loss / count;
      score += 0.020 * ( startMove * startMove + endMove * endMove );
      score += 0.030 * lineMove * lineMove;
      score += 0.015 * heightMove * heightMove;

      fit.success = true;
      fit.lineOffset = candidateLineOffset;
      fit.startT = candidateStartT;
      fit.endT = candidateEndT;
      fit.ridgeHeight = candidateRidgeHeight;
      fit.score = score;
      return true;
    };

    HippedConstrainedRidgeFit bestFit;
    auto tryCandidate = [&]( double candidateLineOffset, double candidateStartT, double candidateEndT ) {
      candidateLineOffset = std::max( -maxLineMove, std::min( maxLineMove, candidateLineOffset ) );
      candidateStartT = std::max( minT, std::min( maxT, candidateStartT ) );
      candidateEndT = std::max( minT, std::min( maxT, candidateEndT ) );
      HippedConstrainedRidgeFit fit;
      if ( scoreConstrainedModel( candidateLineOffset, candidateStartT, candidateEndT, fit ) && fit.score < bestFit.score )
        bestFit = fit;
    };

    auto refitSlopePlanesFromCurrentRidge = [&]() {
      if ( endT <= startT )
        return false;

      QVector<QgsPoint> refitPositiveSidePoints;
      QVector<QgsPoint> refitNegativeSidePoints;
      QVector<QgsPoint> refitStartEndPoints;
      QVector<QgsPoint> refitFinishEndPoints;
      refitPositiveSidePoints.reserve( normalSamples.size() );
      refitNegativeSidePoints.reserve( normalSamples.size() );
      refitStartEndPoints.reserve( normalSamples.size() );
      refitFinishEndPoints.reserve( normalSamples.size() );

      const double currentLength = endT - startT;
      const double refitEndBand = std::max( 0.25, std::min( currentLength * 0.18, extent * 0.09 ) );
      const double refitSideInset = std::max( 0.18, refitEndBand * 0.45 );
      const double refitStartSpan = std::max( 0.30, startT - ridgeInterval.start );
      const double refitFinishSpan = std::max( 0.30, ridgeInterval.end - endT );

      for ( const HippedNormalRoofSample &sample : normalSamples )
      {
        const QgsPoint &point = sample.point;
        const QgsPointXY pointXY( point.x(), point.y() );
        const double along = lineParameter( lineOrigin, ridgeDirX, ridgeDirY, pointXY );
        const double sideDistance = signedDistanceToLine( pointXY, QgsPoint( lineOrigin.x(), lineOrigin.y(), roughRidgeHeight ), normalX, normalY );
        const double absSideDistance = std::fabs( sideDistance );
        const int normalGroup = hippedNormalGroup( sample, ridgeDirX, ridgeDirY, normalX, normalY );

        const double startFaceWidth = footprintHalfWidth * std::max( 0.0, std::min( 1.0, ( startT + refitEndBand - along ) / ( refitStartSpan + refitEndBand ) ) ) + endFaceWidthTolerance;
        const double finishFaceWidth = footprintHalfWidth * std::max( 0.0, std::min( 1.0, ( along - ( endT - refitEndBand ) ) / ( refitFinishSpan + refitEndBand ) ) ) + endFaceWidthTolerance;
        if ( normalGroup == 2 && along >= ridgeInterval.start - endpointOuterTolerance && along <= startT + refitEndBand && absSideDistance <= startFaceWidth )
          refitStartEndPoints.append( point );
        if ( normalGroup == 3 && along >= endT - refitEndBand && along <= ridgeInterval.end + endpointOuterTolerance && absSideDistance <= finishFaceWidth )
          refitFinishEndPoints.append( point );
        if ( along >= startT + refitSideInset && along <= endT - refitSideInset )
        {
          if ( normalGroup == 0 && sideDistance >= 0.0 )
            refitPositiveSidePoints.append( point );
          else if ( normalGroup == 1 && sideDistance < 0.0 )
            refitNegativeSidePoints.append( point );
        }
      }

      const int refitMinSideCount = std::max( 10, static_cast<int>( std::ceil( ( refitPositiveSidePoints.size() + refitNegativeSidePoints.size() ) * 0.10 ) ) );
      if ( refitPositiveSidePoints.size() < refitMinSideCount || refitNegativeSidePoints.size() < refitMinSideCount || refitStartEndPoints.size() < 8 || refitFinishEndPoints.size() < 8 )
        return false;

      const AreaFilteredRoofPlaneFit refitPositiveFit = fitAreaFilteredRoofPlane( refitPositiveSidePoints );
      const AreaFilteredRoofPlaneFit refitNegativeFit = fitAreaFilteredRoofPlane( refitNegativeSidePoints );
      const AreaFilteredRoofPlaneFit refitStartFit = fitAreaFilteredRoofPlane( refitStartEndPoints, 0.05 );
      const AreaFilteredRoofPlaneFit refitFinishFit = fitAreaFilteredRoofPlane( refitFinishEndPoints, 0.05 );
      if ( !refitPositiveFit.success || !refitNegativeFit.success || !refitStartFit.success || !refitFinishFit.success )
        return false;
      const LeastSquaresRoofPlane &refitPositivePlane = refitPositiveFit.plane;
      const LeastSquaresRoofPlane &refitNegativePlane = refitNegativeFit.plane;
      const LeastSquaresRoofPlane &refitStartPlane = refitStartFit.plane;
      const LeastSquaresRoofPlane &refitFinishPlane = refitFinishFit.plane;

      const double refitStartSlopeAlong = refitStartPlane.a * ridgeDirX + refitStartPlane.b * ridgeDirY;
      const double refitFinishSlopeAlong = refitFinishPlane.a * ridgeDirX + refitFinishPlane.b * ridgeDirY;
      const double refitPositiveSideSlope = refitPositivePlane.a * normalX + refitPositivePlane.b * normalY;
      const double refitNegativeSideSlope = refitNegativePlane.a * normalX + refitNegativePlane.b * normalY;
      if ( refitStartSlopeAlong <= 1e-5 || refitFinishSlopeAlong >= -1e-5 || refitPositiveSideSlope >= -1e-5 || refitNegativeSideSlope <= 1e-5 )
        return false;

      const double refitSideLineA = refitPositivePlane.a - refitNegativePlane.a;
      const double refitSideLineB = refitPositivePlane.b - refitNegativePlane.b;
      const double refitSideLineLength = std::hypot( refitSideLineA, refitSideLineB );
      if ( refitSideLineLength <= 1e-10 )
        return false;
      const double refitFittedDirX = -refitSideLineB / refitSideLineLength;
      const double refitFittedDirY = refitSideLineA / refitSideLineLength;
      if ( std::fabs( refitFittedDirX * ridgeDirX + refitFittedDirY * ridgeDirY ) < 0.40 )
        return false;

      positiveSidePoints = refitPositiveFit.points;
      negativeSidePoints = refitNegativeFit.points;
      startEndPoints = refitStartFit.points;
      finishEndPoints = refitFinishFit.points;
      positivePlane = refitPositivePlane;
      negativePlane = refitNegativePlane;
      startPlane = refitStartPlane;
      finishPlane = refitFinishPlane;
      startSlopeAlong = refitStartSlopeAlong;
      finishSlopeAlong = refitFinishSlopeAlong;
      positiveSideSlope = refitPositiveSideSlope;
      negativeSideSlope = refitNegativeSideSlope;

      const double refitSideOffsetDenominator = refitSideLineA * normalX + refitSideLineB * normalY;
      if ( std::fabs( refitSideOffsetDenominator ) > 1e-10 )
      {
        const double refitMidT = ( startT + endT ) * 0.5;
        const QgsPointXY refitBaseMidPoint = pointOnLine( roughLineOrigin, ridgeDirX, ridgeDirY, refitMidT );
        const double refitSideOffsetNumerator = refitSideLineA * refitBaseMidPoint.x() + refitSideLineB * refitBaseMidPoint.y() + refitPositivePlane.c - refitNegativePlane.c;
        const double refitLineOffset = -refitSideOffsetNumerator / refitSideOffsetDenominator;
        if ( std::isfinite( refitLineOffset ) && std::fabs( refitLineOffset ) <= maxLineMove )
        {
          preferredLineOffset = refitLineOffset;
          appendSeedLineOffset( preferredLineOffset );
          lineOrigin = QgsPointXY( roughLineOrigin.x() + preferredLineOffset * normalX,
                                   roughLineOrigin.y() + preferredLineOffset * normalY );
        }
      }

      const LeastSquaresRoofPlane refitRidgePlane = averagedRoofPlane( positivePlane, negativePlane );
      double refitPlaneStartT = preferredStartT;
      double refitPlaneEndT = preferredEndT;
      if ( planeIntersectionParameterOnLine( refitRidgePlane, startPlane, lineOrigin, ridgeDirX, ridgeDirY, refitPlaneStartT )
           && refitPlaneStartT >= expandedMinT && refitPlaneStartT <= expandedMaxT
           && std::fabs( refitPlaneStartT - roughStartT ) <= maxEndpointMove )
      {
        preferredStartT = std::max( minT, std::min( maxT, refitPlaneStartT ) );
        startT = preferredStartT;
      }
      if ( planeIntersectionParameterOnLine( refitRidgePlane, finishPlane, lineOrigin, ridgeDirX, ridgeDirY, refitPlaneEndT )
           && refitPlaneEndT >= expandedMinT && refitPlaneEndT <= expandedMaxT
           && std::fabs( refitPlaneEndT - roughEndT ) <= maxEndpointMove )
      {
        preferredEndT = std::max( minT, std::min( maxT, refitPlaneEndT ) );
        endT = preferredEndT;
      }
      return true;
    };

    const double coarseStep = std::max( 0.12, maxEndpointMove / 5.0 );
    const double coarseLineStep = std::max( 0.05, maxLineMove / 5.0 );
    const int coarseLineRadius = 5;
    for ( double baseLineOffset : seedLineOffsets )
    {
      for ( int oi = -coarseLineRadius; oi <= coarseLineRadius; ++oi )
      {
        const double candidateLineOffset = baseLineOffset + oi * coarseLineStep;
        for ( int si = -5; si <= 5; ++si )
        {
          for ( int ei = -5; ei <= 5; ++ei )
            tryCandidate( candidateLineOffset, startT + si * coarseStep, endT + ei * coarseStep );
        }
      }
    }
    tryCandidate( 0.0, startT, endT );

    if ( !bestFit.success )
    {
      const double fallbackStep = std::max( 0.12, ( maxT - minT ) / 8.0 );
      for ( double candidateLineOffset : seedLineOffsets )
      {
        for ( int si = 0; si <= 8; ++si )
        {
          const double candidateStartT = minT + si * fallbackStep;
          for ( int ei = si + 1; ei <= 8; ++ei )
            tryCandidate( candidateLineOffset, candidateStartT, minT + ei * fallbackStep );
        }
      }
    }

    if ( bestFit.success )
    {
      for ( double refineStep : { coarseStep * 0.45, coarseStep * 0.20 } )
      {
        const double centerLineOffset = bestFit.lineOffset;
        const double centerStartT = bestFit.startT;
        const double centerEndT = bestFit.endT;
        const double lineStep = std::max( 0.03, refineStep * 0.45 );
        const int lineRadius = 2;
        for ( int si = -2; si <= 2; ++si )
        {
          for ( int ei = -2; ei <= 2; ++ei )
          {
            for ( int oi = -lineRadius; oi <= lineRadius; ++oi )
              tryCandidate( centerLineOffset + oi * lineStep, centerStartT + si * refineStep, centerEndT + ei * refineStep );
          }
        }
      }
      lineOrigin = QgsPointXY( roughLineOrigin.x() + bestFit.lineOffset * normalX,
                               roughLineOrigin.y() + bestFit.lineOffset * normalY );
      startT = bestFit.startT;
      endT = bestFit.endT;
    }

    if ( bestFit.success )
    {
      const HippedConstrainedRidgeFit firstPassFit = bestFit;
      const QgsPointXY firstPassLineOrigin = lineOrigin;
      const double firstPassStartT = startT;
      const double firstPassEndT = endT;

      if ( refitSlopePlanesFromCurrentRidge() )
      {
        HippedConstrainedRidgeFit secondPassFit;
        bestFit = secondPassFit;
        const double secondEndpointStep = std::max( 0.08, coarseStep * 0.35 );
        const double secondLineStep = std::max( 0.025, coarseLineStep * 0.35 );
        const int secondLineRadius = 3;
        for ( int oi = -secondLineRadius; oi <= secondLineRadius; ++oi )
        {
          for ( int si = -3; si <= 3; ++si )
          {
            for ( int ei = -3; ei <= 3; ++ei )
              tryCandidate( firstPassFit.lineOffset + oi * secondLineStep, firstPassStartT + si * secondEndpointStep, firstPassEndT + ei * secondEndpointStep );
          }
        }

        if ( bestFit.success )
        {
          for ( double refineStep : { secondEndpointStep * 0.45, secondEndpointStep * 0.20 } )
          {
            const double centerLineOffset = bestFit.lineOffset;
            const double centerStartT = bestFit.startT;
            const double centerEndT = bestFit.endT;
            const double lineStep = std::max( 0.02, refineStep * 0.40 );
            const int lineRadius = 2;
            for ( int si = -2; si <= 2; ++si )
            {
              for ( int ei = -2; ei <= 2; ++ei )
              {
                for ( int oi = -lineRadius; oi <= lineRadius; ++oi )
                  tryCandidate( centerLineOffset + oi * lineStep, centerStartT + si * refineStep, centerEndT + ei * refineStep );
              }
            }
          }
          lineOrigin = QgsPointXY( roughLineOrigin.x() + bestFit.lineOffset * normalX,
                                   roughLineOrigin.y() + bestFit.lineOffset * normalY );
          startT = bestFit.startT;
          endT = bestFit.endT;
        }
        else
        {
          bestFit = firstPassFit;
          lineOrigin = firstPassLineOrigin;
          startT = firstPassStartT;
          endT = firstPassEndT;
        }
      }
    }

    const double generatedInset = std::max( 0.05, extent * 0.002 );
    const double generatedMinT = ridgeInterval.start + generatedInset;
    const double generatedMaxT = ridgeInterval.end - generatedInset;
    const double generatedIntervalLength = generatedMaxT - generatedMinT;
    if ( generatedIntervalLength <= std::max( 0.75, extent * 0.08 ) )
      return false;
    const double minRidgeLength = std::max( 0.70, generatedIntervalLength * 0.08 );
    const double maxRidgeLength = generatedIntervalLength * 0.98;
    const double structureIntervalTolerance = std::max( 0.35, std::min( 1.20, extent * 0.060 ) );
    QVector<HippedWeightedRoofSample> generatedRoofPoints;
    generatedRoofPoints.reserve( normalSamples.size() );
    for ( const HippedNormalRoofSample &sample : normalSamples )
      generatedRoofPoints.append( HippedWeightedRoofSample{ sample.point, 1.0 } );

    const double rawStructureThreshold = std::max( sortedPercentile( heightBand.filteredHeights, 0.78 ),
                                                   roughRidgeHeight - std::max( 0.45, heightBand.binWidth * 2.5 ) );
    const double rawStructureCeiling = rawInsideHeights.size() >= 20
                                         ? sortedPercentile( rawInsideHeights, 0.995 )
                                         : heightBand.highHeight;
    QVector<double> rawStructureParameters;
    rawStructureParameters.reserve( rawRidgeHeights.size() );
    for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
    {
      const QgsPoint &point = sample.point;
      if ( point.z() < rawStructureThreshold || point.z() > rawStructureCeiling )
        continue;
      const QgsPointXY pointXY( point.x(), point.y() );
      if ( !pointInRing( ring, pointXY ) )
        continue;
      const double parameter = lineParameter( roughStart, ridgeDirX, ridgeDirY, pointXY );
      if ( parameter < ridgeInterval.start - structureIntervalTolerance || parameter > ridgeInterval.end + structureIntervalTolerance )
        continue;
      const double lineDistance = std::fabs( signedDistanceToLine( pointXY, QgsPoint( roughStart.x(), roughStart.y(), roughRidgeHeight ), normalX, normalY ) );
      if ( lineDistance > ridgeWeightRadius * 1.35 )
        continue;
      rawStructureParameters.append( parameter );
    }
    std::sort( rawStructureParameters.begin(), rawStructureParameters.end() );

    double outlineDirX = ridgeDirX;
    double outlineDirY = ridgeDirY;
    double maxOutlineEdgeLength = 0.0;
    for ( int i = 0; i < ring.size(); ++i )
      maxOutlineEdgeLength = std::max( maxOutlineEdgeLength, std::hypot( ring[( i + 1 ) % ring.size()].x() - ring[i].x(), ring[( i + 1 ) % ring.size()].y() - ring[i].y() ) );

    double bestOutlineScore = -1.0;
    for ( int i = 0; i < ring.size(); ++i )
    {
      double edgeX = ring[( i + 1 ) % ring.size()].x() - ring[i].x();
      double edgeY = ring[( i + 1 ) % ring.size()].y() - ring[i].y();
      const double edgeLength = std::hypot( edgeX, edgeY );
      if ( edgeLength <= 1e-8 || maxOutlineEdgeLength <= 1e-8 )
        continue;
      edgeX /= edgeLength;
      edgeY /= edgeLength;
      double alignment = edgeX * ridgeDirX + edgeY * ridgeDirY;
      if ( std::fabs( alignment ) < 0.8660254037844386 )
        continue;
      if ( alignment < 0.0 )
      {
        edgeX = -edgeX;
        edgeY = -edgeY;
        alignment = -alignment;
      }
      const double lengthWeight = std::sqrt( edgeLength / maxOutlineEdgeLength );
      const double outlineScore = alignment * ( 0.35 + 0.65 * lengthWeight );
      if ( outlineScore > bestOutlineScore )
      {
        bestOutlineScore = outlineScore;
        outlineDirX = edgeX;
        outlineDirY = edgeY;
      }
    }

    constexpr double maxAngleOffset = 0.2094395102393196; // 12 degrees
    const double outlineAngleOffset = std::max( -maxAngleOffset, std::min( maxAngleOffset,
      std::atan2( ridgeDirX * outlineDirY - ridgeDirY * outlineDirX,
                  ridgeDirX * outlineDirX + ridgeDirY * outlineDirY ) ) );
    const double outlinePenaltyWeight = std::max( 0.0025, robustDelta * robustDelta * 0.10 );
    const QgsPointXY roughPivot = pointOnLine( roughLineOrigin, ridgeDirX, ridgeDirY, roughMidT );
    double finalRidgeDirX = ridgeDirX;
    double finalRidgeDirY = ridgeDirY;

    HippedConstrainedRidgeFit generatedBest;
    auto tryGeneratedRoof = [&]( double candidateLineOffset, double candidateStartT, double candidateEndT, double candidateHeight, double candidateAngleOffset ) {
      candidateLineOffset = std::max( -maxLineMove, std::min( maxLineMove, candidateLineOffset ) );
      candidateStartT = std::max( generatedMinT, std::min( generatedMaxT, candidateStartT ) );
      candidateEndT = std::max( generatedMinT, std::min( generatedMaxT, candidateEndT ) );
      candidateHeight = std::max( minHeight, std::min( maxHeight, candidateHeight ) );
      candidateAngleOffset = std::max( -maxAngleOffset, std::min( maxAngleOffset, candidateAngleOffset ) );
      if ( candidateEndT - candidateStartT < minRidgeLength || candidateEndT - candidateStartT > maxRidgeLength )
        return;

      const double angleCos = std::cos( candidateAngleOffset );
      const double angleSin = std::sin( candidateAngleOffset );
      const double candidateDirX = ridgeDirX * angleCos - ridgeDirY * angleSin;
      const double candidateDirY = ridgeDirX * angleSin + ridgeDirY * angleCos;
      const double candidateNormalX = -candidateDirY;
      const double candidateNormalY = candidateDirX;
      const QgsPointXY candidatePivot( roughPivot.x() + candidateLineOffset * candidateNormalX,
                                       roughPivot.y() + candidateLineOffset * candidateNormalY );
      const QgsPointXY candidateFirst = pointOnLine( candidatePivot, candidateDirX, candidateDirY, candidateStartT - roughMidT );
      const QgsPointXY candidateSecond = pointOnLine( candidatePivot, candidateDirX, candidateDirY, candidateEndT - roughMidT );
      if ( !pointInRing( ring, candidateFirst ) || !pointInRing( ring, candidateSecond ) )
        return;

      double candidateScore = hippedCandidateRoofLoss( ring, generatedRoofPoints, candidateFirst, candidateSecond,
                                                       heightBand.baseHeight, candidateHeight, robustDelta );
      const double outlineDot = std::max( -1.0, std::min( 1.0, std::fabs( candidateDirX * outlineDirX + candidateDirY * outlineDirY ) ) );
      const double outlineAngle = std::acos( outlineDot );
      candidateScore += outlinePenaltyWeight * std::pow( outlineAngle / maxAngleOffset, 2.0 );
      if ( !std::isfinite( candidateScore ) || candidateScore >= generatedBest.score )
        return;

      generatedBest.success = true;
      generatedBest.lineOffset = candidateLineOffset;
      generatedBest.startT = candidateStartT;
      generatedBest.endT = candidateEndT;
      generatedBest.ridgeHeight = candidateHeight;
      generatedBest.angleOffset = candidateAngleOffset;
      generatedBest.score = candidateScore;
    };

    const double currentLineOffset = ( lineOrigin.x() - roughLineOrigin.x() ) * normalX
                                     + ( lineOrigin.y() - roughLineOrigin.y() ) * normalY;
    const double currentHeight = bestFit.success ? bestFit.ridgeHeight : roughRidgeHeight;
    tryGeneratedRoof( currentLineOffset, startT, endT, currentHeight, 0.0 );
    tryGeneratedRoof( currentLineOffset, startT, endT, currentHeight, outlineAngleOffset );
    tryGeneratedRoof( 0.0, roughStartT, roughEndT, roughRidgeHeight, 0.0 );

    if ( rawStructureParameters.size() >= 8 )
    {
      for ( const QPair<double, double> &percentilePair : {
              qMakePair( 0.05, 0.95 ),
              qMakePair( 0.10, 0.90 ),
              qMakePair( 0.15, 0.85 ) } )
      {
        const double seedStartT = sortedPercentile( rawStructureParameters, percentilePair.first );
        const double seedEndT = sortedPercentile( rawStructureParameters, percentilePair.second );
        tryGeneratedRoof( currentLineOffset, seedStartT, seedEndT, currentHeight, 0.0 );
        tryGeneratedRoof( currentLineOffset, seedStartT, seedEndT, currentHeight, outlineAngleOffset );
      }

      const double parameterBinWidth = std::max( 0.15, std::min( 0.55, extent * 0.018 ) );
      const int parameterBinCount = std::max( 3, static_cast<int>( std::ceil( generatedIntervalLength / parameterBinWidth ) ) );
      QVector<int> parameterCounts( parameterBinCount, 0 );
      for ( double parameter : rawStructureParameters )
      {
        if ( parameter < generatedMinT || parameter > generatedMaxT )
          continue;
        const int index = std::max( 0, std::min( parameterBinCount - 1,
          static_cast<int>( std::floor( ( parameter - generatedMinT ) / generatedIntervalLength * parameterBinCount ) ) ) );
        ++parameterCounts[index];
      }

      QVector<int> smoothedParameterCounts( parameterBinCount, 0 );
      int peakIndex = 0;
      for ( int i = 0; i < parameterBinCount; ++i )
      {
        smoothedParameterCounts[i] = parameterCounts.at( i );
        if ( i > 0 )
          smoothedParameterCounts[i] += parameterCounts.at( i - 1 );
        if ( i + 1 < parameterBinCount )
          smoothedParameterCounts[i] += parameterCounts.at( i + 1 );
        if ( smoothedParameterCounts.at( i ) > smoothedParameterCounts.at( peakIndex ) )
          peakIndex = i;
      }

      const int runSupport = std::max( 2, smoothedParameterCounts.at( peakIndex ) / 5 );
      int runStart = peakIndex;
      int runEnd = peakIndex;
      while ( runStart > 0 && smoothedParameterCounts.at( runStart - 1 ) >= runSupport )
        --runStart;
      while ( runEnd + 1 < parameterBinCount && smoothedParameterCounts.at( runEnd + 1 ) >= runSupport )
        ++runEnd;
      const double denseStartT = generatedMinT + generatedIntervalLength * runStart / parameterBinCount;
      const double denseEndT = generatedMinT + generatedIntervalLength * ( runEnd + 1 ) / parameterBinCount;
      tryGeneratedRoof( currentLineOffset, denseStartT, denseEndT, currentHeight, 0.0 );
      tryGeneratedRoof( currentLineOffset, denseStartT, denseEndT, currentHeight, outlineAngleOffset );
    }

    if ( generatedBest.success )
    {
      const int endpointScanSteps = std::max( 16, std::min( 30,
        static_cast<int>( std::ceil( generatedIntervalLength / std::max( 0.30, extent * 0.018 ) ) ) ) );
      for ( int pass = 0; pass < 2; ++pass )
      {
        const HippedConstrainedRidgeFit startCenter = generatedBest;
        for ( int i = 0; i <= endpointScanSteps; ++i )
        {
          const double candidateStartT = generatedMinT + generatedIntervalLength * i / endpointScanSteps;
          if ( startCenter.endT - candidateStartT < minRidgeLength )
            continue;
          tryGeneratedRoof( startCenter.lineOffset, candidateStartT, startCenter.endT,
                            startCenter.ridgeHeight, startCenter.angleOffset );
        }

        const HippedConstrainedRidgeFit endCenter = generatedBest;
        for ( int i = 0; i <= endpointScanSteps; ++i )
        {
          const double candidateEndT = generatedMinT + generatedIntervalLength * i / endpointScanSteps;
          if ( candidateEndT - endCenter.startT < minRidgeLength )
            continue;
          tryGeneratedRoof( endCenter.lineOffset, endCenter.startT, candidateEndT,
                            endCenter.ridgeHeight, endCenter.angleOffset );
        }
      }
    }

    if ( generatedBest.success )
    {
      double lineStep = std::max( 0.035, maxLineMove * 0.42 );
      double endpointStep = std::max( 0.07, generatedIntervalLength * 0.10 );
      double heightStep = std::max( 0.035, ( maxHeight - minHeight ) * 0.18 );
      double angleStep = maxAngleOffset * 0.32;
      for ( int iteration = 0; iteration < 7; ++iteration )
      {
        for ( int direction : { -1, 1 } )
        {
          const HippedConstrainedRidgeFit center = generatedBest;
          tryGeneratedRoof( center.lineOffset + direction * lineStep, center.startT, center.endT, center.ridgeHeight, center.angleOffset );
        }
        for ( int direction : { -1, 1 } )
        {
          const HippedConstrainedRidgeFit center = generatedBest;
          tryGeneratedRoof( center.lineOffset, center.startT + direction * endpointStep, center.endT, center.ridgeHeight, center.angleOffset );
        }
        for ( int direction : { -1, 1 } )
        {
          const HippedConstrainedRidgeFit center = generatedBest;
          tryGeneratedRoof( center.lineOffset, center.startT, center.endT + direction * endpointStep, center.ridgeHeight, center.angleOffset );
        }
        for ( int direction : { -1, 1 } )
        {
          const HippedConstrainedRidgeFit center = generatedBest;
          tryGeneratedRoof( center.lineOffset, center.startT, center.endT, center.ridgeHeight + direction * heightStep, center.angleOffset );
        }
        for ( int direction : { -1, 1 } )
        {
          const HippedConstrainedRidgeFit center = generatedBest;
          tryGeneratedRoof( center.lineOffset, center.startT, center.endT, center.ridgeHeight, center.angleOffset + direction * angleStep );
        }
        lineStep *= 0.45;
        endpointStep *= 0.45;
        heightStep *= 0.45;
        angleStep *= 0.45;
      }

      bestFit = generatedBest;
      const double angleCos = std::cos( bestFit.angleOffset );
      const double angleSin = std::sin( bestFit.angleOffset );
      finalRidgeDirX = ridgeDirX * angleCos - ridgeDirY * angleSin;
      finalRidgeDirY = ridgeDirX * angleSin + ridgeDirY * angleCos;
      const double finalNormalX = -finalRidgeDirY;
      const double finalNormalY = finalRidgeDirX;
      const QgsPointXY finalPivot( roughPivot.x() + bestFit.lineOffset * finalNormalX,
                                   roughPivot.y() + bestFit.lineOffset * finalNormalY );
      lineOrigin = pointOnLine( finalPivot, finalRidgeDirX, finalRidgeDirY, -roughMidT );
      startT = bestFit.startT;
      endT = bestFit.endT;
    }

    if ( endT <= startT )
      return false;

    if ( endT - startT < minRidgeLength )
    {
      const double centerT = std::max( minT, std::min( maxT, roughMidT ) );
      startT = std::max( minT, centerT - minRidgeLength * 0.5 );
      endT = std::min( maxT, centerT + minRidgeLength * 0.5 );
      if ( endT - startT < minRidgeLength * 0.65 )
        return false;
    }

    if ( endT - startT > maxRidgeLength )
    {
      const double centerT = ( startT + endT ) * 0.5;
      startT = std::max( minT, centerT - maxRidgeLength * 0.5 );
      endT = std::min( maxT, centerT + maxRidgeLength * 0.5 );
    }

    QgsPointXY firstXY = pointOnLine( lineOrigin, finalRidgeDirX, finalRidgeDirY, startT );
    QgsPointXY secondXY = pointOnLine( lineOrigin, finalRidgeDirX, finalRidgeDirY, endT );
    if ( !pointInRing( ring, firstXY ) )
      firstXY = pullPointInsideRing( ring, firstXY, roughStart );
    if ( !pointInRing( ring, secondXY ) )
      secondXY = pullPointInsideRing( ring, secondXY, roughEnd );

    double ridgeHeight = bestFit.success ? bestFit.ridgeHeight : averagedFiniteHeight( QVector<double>{
      roofPlaneZ( ridgePlane, firstXY ),
      roofPlaneZ( ridgePlane, secondXY ),
      roughRidgeHeight
    }, roughRidgeHeight );
    ridgeHeight = std::max( minHeight, std::min( maxHeight, ridgeHeight ) );

    if ( !std::isfinite( ridgeHeight ) || ridgeHeight <= heightBand.baseHeight + 0.20 )
      return false;

    refinedRidge = roughRidge;
    refinedRidge.success = true;
    refinedRidge.firstPoint = QgsPoint( firstXY.x(), firstXY.y(), ridgeHeight );
    refinedRidge.secondPoint = QgsPoint( secondXY.x(), secondXY.y(), ridgeHeight );
    refinedRidge.dirX = finalRidgeDirX;
    refinedRidge.dirY = finalRidgeDirY;
    return true;
  }

  bool topHeightHippedRidgeFromPointCloud( const QVector<QgsPointXY> &ring, const QVector<BuildingRoof::RoofSample> &pointCloudSamples, const QgsPoint &boundaryPoint, AutoHippedRidge &ridge )
  {
    double dirX = 0.0;
    double dirY = 0.0;
    if ( !nearestEdgeDirection( ring, boundaryPoint, dirX, dirY ) )
      return false;

    const AutoHeightBand heightBand = inferAutoGabledHeightBand( ring, pointCloudSamples );
    if ( !heightBand.success )
      return false;

    QVector<BuildingRoof::RoofSample> filteredSamples;
    filteredSamples.reserve( pointCloudSamples.size() );
    for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
    {
      const QgsPoint &point = sample.point;
      if ( point.z() < heightBand.lowHeight || point.z() > heightBand.highHeight )
        continue;
      if ( !pointInRing( ring, QgsPointXY( point.x(), point.y() ) ) )
        continue;
      filteredSamples.append( sample );
    }

    if ( filteredSamples.size() < 20 )
      return false;

    AutoHippedRidge best;
    double bestScore = -1.0;
    QVector<QgsPoint> roofPoints;
    roofPoints.reserve( filteredSamples.size() );
    for ( const BuildingRoof::RoofSample &sample : filteredSamples )
      roofPoints.append( sample.point );

    QgsPoint gabledRidgePoint;
    double gabledDirX = 0.0;
    double gabledDirY = 0.0;
    GabledBoundarySlopePlanes gabledSlopePlanes;
    const bool hasGabledMainRidge = leastSquaresGabledRidgePointFromPointCloud( ring, filteredSamples, boundaryPoint, gabledRidgePoint, &gabledDirX, &gabledDirY, &gabledSlopePlanes );
    QVector<double> percentiles;
    percentiles << 0.72 << 0.78 << 0.84 << 0.90;
    for ( double percentile : percentiles )
    {
      const double threshold = sortedPercentile( heightBand.filteredHeights, percentile );
      QVector<QgsPoint> highPoints;
      highPoints.reserve( filteredSamples.size() );
      for ( const BuildingRoof::RoofSample &sample : filteredSamples )
      {
        if ( sample.point.z() >= threshold )
          highPoints.append( sample.point );
      }

      if ( hasGabledMainRidge )
      {
        AutoHippedRidge gabledCandidate;
        double gabledCandidateScore = 0.0;
        if ( fitHippedRidgeCandidateOnKnownLine( ring, roofPoints, highPoints, gabledRidgePoint, gabledDirX, gabledDirY, heightBand.ridgeHeight, gabledCandidate, gabledCandidateScore )
             && ( !best.preferredGabledSeed || gabledCandidateScore > bestScore ) )
        {
          bestScore = gabledCandidateScore;
          best = gabledCandidate;
        }
      }

      AutoHippedRidge candidate;
      double candidateScore = 0.0;
      if ( fitAutoHippedRidgeCandidate( ring, highPoints, dirX, dirY, heightBand.ridgeHeight, candidate, candidateScore )
           && !best.preferredGabledSeed && candidateScore > bestScore )
      {
        bestScore = candidateScore;
        best = candidate;
      }
    }

    const double localHighBand = std::max( heightBand.binWidth * 2.0, 0.35 );
    QVector<QgsPoint> ridgeBandPoints;
    for ( const BuildingRoof::RoofSample &sample : filteredSamples )
    {
      if ( std::fabs( sample.point.z() - heightBand.ridgeHeight ) <= localHighBand )
        ridgeBandPoints.append( sample.point );
    }

    AutoHippedRidge candidate;
    double candidateScore = 0.0;
    if ( hasGabledMainRidge )
    {
      AutoHippedRidge gabledCandidate;
      double gabledCandidateScore = 0.0;
      if ( fitHippedRidgeCandidateOnKnownLine( ring, roofPoints, ridgeBandPoints, gabledRidgePoint, gabledDirX, gabledDirY, heightBand.ridgeHeight, gabledCandidate, gabledCandidateScore )
             && ( !best.preferredGabledSeed || gabledCandidateScore > bestScore ) )
      {
        bestScore = gabledCandidateScore;
        best = gabledCandidate;
      }
    }

    if ( fitAutoHippedRidgeCandidate( ring, ridgeBandPoints, dirX, dirY, heightBand.ridgeHeight, candidate, candidateScore )
         && !best.preferredGabledSeed && candidateScore > bestScore )
    {
      bestScore = candidateScore;
      best = candidate;
    }

    if ( !best.success )
      return false;

    AutoHippedRidge refined;
    if ( refineHippedRidgeWithLeastSquaresPlanes( ring, pointCloudSamples, heightBand, best, refined ) )
      ridge = refined;
    else
      ridge = best;
    return true;
  }

  QList<BuildingRoof::RoofPoint> topHeightHippedRoofPoints( const QVector<QgsPointXY> &ring, const QList<BuildingRoof::RoofPoint> &boundaries, const QList<BuildingRoof::RoofPoint> &ridges, const QVector<BuildingRoof::RoofSample> &pointCloudSamples )
  {
    QList<BuildingRoof::RoofPoint> points;
    if ( boundaries.size() != 1 || ridges.size() != 2 )
      return points;

    AutoHippedRidge ridge;
    if ( !topHeightHippedRidgeFromPointCloud( ring, pointCloudSamples, boundaries.first().point, ridge ) )
      return points;

    const QgsPoint ridgeMidPoint( ( ridge.firstPoint.x() + ridge.secondPoint.x() ) * 0.5,
                                  ( ridge.firstPoint.y() + ridge.secondPoint.y() ) * 0.5,
                                  ( ridge.firstPoint.z() + ridge.secondPoint.z() ) * 0.5 );
    QgsPoint boundaryPoint = boundaries.first().point;
    QgsPoint automaticBoundaryPoint = boundaryPoint;
    if ( automaticBoundaryPointFromRidgeLine( ring, pointCloudSamples, ridgeMidPoint, ridge.dirX, ridge.dirY, boundaries.first().point, automaticBoundaryPoint ) )
      boundaryPoint = automaticBoundaryPoint;

    points.append( BuildingRoof::RoofPoint{ boundaryPoint, boundaries.first().type } );
    points.append( BuildingRoof::RoofPoint{ ridge.firstPoint, ridges.at( 0 ).type } );
    points.append( BuildingRoof::RoofPoint{ ridge.secondPoint, ridges.at( 1 ).type } );
    return points;
  }

  QgsPointXY ringCentroidPoint( const QVector<QgsPointXY> &ring )
  {
    if ( ring.isEmpty() )
      return QgsPointXY();

    double twiceArea = 0.0;
    double centroidX = 0.0;
    double centroidY = 0.0;
    for ( int i = 0; i < ring.size(); ++i )
    {
      const QgsPointXY &a = ring.at( i );
      const QgsPointXY &b = ring.at( ( i + 1 ) % ring.size() );
      const double cross = a.x() * b.y() - b.x() * a.y();
      twiceArea += cross;
      centroidX += ( a.x() + b.x() ) * cross;
      centroidY += ( a.y() + b.y() ) * cross;
    }

    if ( std::fabs( twiceArea ) > 1e-10 )
    {
      const QgsPointXY centroid( centroidX / ( 3.0 * twiceArea ), centroidY / ( 3.0 * twiceArea ) );
      if ( pointInRing( ring, centroid ) )
        return centroid;
    }

    double sumX = 0.0;
    double sumY = 0.0;
    for ( const QgsPointXY &point : ring )
    {
      sumX += point.x();
      sumY += point.y();
    }
    const QgsPointXY averaged( sumX / ring.size(), sumY / ring.size() );
    if ( pointInRing( ring, averaged ) )
      return averaged;
    return ring.first();
  }

  QgsPointXY pullPointInsideRing( const QVector<QgsPointXY> &ring, const QgsPointXY &candidate, const QgsPointXY &fallback )
  {
    if ( pointInRing( ring, candidate ) )
      return candidate;
    if ( !pointInRing( ring, fallback ) )
      return candidate;

    QgsPointXY low = fallback;
    QgsPointXY high = candidate;
    for ( int i = 0; i < 32; ++i )
    {
      const QgsPointXY mid( ( low.x() + high.x() ) * 0.5, ( low.y() + high.y() ) * 0.5 );
      if ( pointInRing( ring, mid ) )
        low = mid;
      else
        high = mid;
    }
    return low;
  }

  bool automaticApexPointFromPointCloud( const QVector<QgsPointXY> &ring, const QVector<BuildingRoof::RoofSample> &pointCloudSamples, const QgsPoint &fallbackPoint, QgsPoint &apexPoint )
  {
    Q_UNUSED( fallbackPoint )

    QVector<QgsPoint> filteredSamples;
    QVector<double> allHeights;
    filteredSamples.reserve( pointCloudSamples.size() );
    allHeights.reserve( pointCloudSamples.size() );
    for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
    {
      const QgsPoint &point = sample.point;
      if ( !pointInRing( ring, QgsPointXY( point.x(), point.y() ) ) )
        continue;
      filteredSamples.append( point );
      allHeights.append( point.z() );
    }

    if ( filteredSamples.size() < 20 )
      return false;

    std::sort( allHeights.begin(), allHeights.end() );
    const double lowClip = sortedPercentile( allHeights, 0.02 );
    QVector<QgsPoint> clippedSamples;
    clippedSamples.reserve( filteredSamples.size() );
    for ( const QgsPoint &point : filteredSamples )
    {
      if ( point.z() >= lowClip )
        clippedSamples.append( point );
    }

    if ( clippedSamples.size() < 8 )
      return false;

    std::sort( clippedSamples.begin(), clippedSamples.end(), []( const QgsPoint &left, const QgsPoint &right ) {
      return left.z() > right.z();
    } );

    const int desiredTopCount = static_cast<int>( std::ceil( clippedSamples.size() * 0.01 ) );
    const int topCount = std::min( clippedSamples.size(), std::min( 20, std::max( 4, desiredTopCount ) ) );
    if ( topCount <= 0 )
      return false;

    double sumX = 0.0;
    double sumY = 0.0;
    double sumZ = 0.0;
    QVector<QgsPoint> topPoints;
    topPoints.reserve( topCount );
    for ( int i = 0; i < topCount; ++i )
    {
      const QgsPoint &point = clippedSamples.at( i );
      sumX += point.x();
      sumY += point.y();
      sumZ += point.z();
      topPoints.append( point );
    }

    const QgsPointXY topCenter( sumX / topCount, sumY / topCount );
    const QgsPointXY footprintCenter = ringCentroidPoint( ring );
    const double extent = ringExtentSize( ring );
    double spread = 0.0;
    for ( const QgsPoint &point : topPoints )
      spread += std::hypot( point.x() - topCenter.x(), point.y() - topCenter.y() );
    spread /= topPoints.size();

    const double normalizedSpread = spread / std::max( 1.0, extent * 0.12 );
    const double geometryWeight = std::max( 0.15, std::min( 0.45, normalizedSpread * 0.25 + 0.15 ) );
    QgsPointXY balancedPoint( topCenter.x() * ( 1.0 - geometryWeight ) + footprintCenter.x() * geometryWeight,
                              topCenter.y() * ( 1.0 - geometryWeight ) + footprintCenter.y() * geometryWeight );
    balancedPoint = pullPointInsideRing( ring, balancedPoint, footprintCenter );

    const double apexHeight = sumZ / topCount;
    apexPoint = QgsPoint( balancedPoint.x(), balancedPoint.y(), apexHeight );
    return true;
  }

  bool automaticBoundaryHeightFromPerimeterBand( const QVector<QgsPointXY> &ring, const QVector<BuildingRoof::RoofSample> &pointCloudSamples, const QgsPoint &apexPoint, double &boundaryHeight )
  {
    if ( ring.size() < 3 || pointCloudSamples.isEmpty() )
      return false;

    QVector<double> allHeights;
    allHeights.reserve( pointCloudSamples.size() );
    for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
    {
      const QgsPoint &point = sample.point;
      if ( pointInRing( ring, QgsPointXY( point.x(), point.y() ) ) )
        allHeights.append( point.z() );
    }
    if ( allHeights.size() < 12 )
      return false;

    std::sort( allHeights.begin(), allHeights.end() );
    const double lowClip = sortedPercentile( allHeights, 0.02 );
    const double highClip = sortedPercentile( allHeights, 0.995 );
    const double heightRange = std::max( 0.30, highClip - lowClip );
    const double extent = ringExtentSize( ring );
    const double firstBandDistance = std::min( 0.45, std::max( 0.16, extent * 0.006 ) );
    const double secondBandDistance = std::min( 0.90, std::max( firstBandDistance * 1.8, extent * 0.014 ) );

    auto collectBandHeights = [&]( double bandDistance ) {
      QVector<double> heights;
      const double bandDistance2 = bandDistance * bandDistance;
      heights.reserve( pointCloudSamples.size() );
      for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
      {
        const QgsPoint &point = sample.point;
        const QgsPointXY pointXY( point.x(), point.y() );
        if ( !pointInRing( ring, pointXY ) )
          continue;
        if ( point.z() < lowClip || point.z() > highClip )
          continue;
        if ( distanceToRing2( ring, pointXY ) > bandDistance2 )
          continue;
        heights.append( point.z() );
      }
      std::sort( heights.begin(), heights.end() );
      return heights;
    };

    QVector<double> perimeterHeights = collectBandHeights( firstBandDistance );
    if ( perimeterHeights.size() < 12 )
      perimeterHeights = collectBandHeights( secondBandDistance );
    if ( perimeterHeights.size() < 6 )
      return false;

    const double fallbackHeight = sortedPercentile( perimeterHeights, perimeterHeights.size() >= 20 ? 0.78 : 0.72 );
    double eaveHeight = perimeterHeights.size() >= 20
                          ? denseUpperWallHeight( perimeterHeights, fallbackHeight, apexPoint.z(), heightRange )
                          : upperWeightedMean( perimeterHeights, 0.60, 0.88, fallbackHeight );

    const double apexBuffer = std::max( 0.30, extent * 0.01 );
    if ( eaveHeight >= apexPoint.z() - 1e-6 )
      eaveHeight = apexPoint.z() - apexBuffer;

    if ( eaveHeight <= lowClip - 1e-6 )
      return false;

    boundaryHeight = eaveHeight;
    return true;
  }

  struct ApexSurfaceSample
  {
    double z = 0.0;
    double apexFraction = 0.0;
    double gradientSquared = 0.0;
    int sector = 0;
  };

  bool fitApexHeightFromRoofSurface( const QVector<QgsPointXY> &ring, const QVector<BuildingRoof::RoofSample> &pointCloudSamples, const QgsPointXY &apexXY, double eaveHeight, double &apexHeight )
  {
    QVector<double> insideHeights;
    insideHeights.reserve( pointCloudSamples.size() );
    for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
    {
      const QgsPoint &point = sample.point;
      if ( std::isfinite( point.z() ) && pointInRing( ring, QgsPointXY( point.x(), point.y() ) ) )
        insideHeights.append( point.z() );
    }
    if ( insideHeights.size() < 20 )
      return false;

    std::sort( insideHeights.begin(), insideHeights.end() );
    const double upperHeight = sortedPercentile( insideHeights, 0.995 );
    if ( upperHeight <= eaveHeight + 0.20 )
      return false;

    QVector<ApexSurfaceSample> roofSamples;
    roofSamples.reserve( pointCloudSamples.size() );
    for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
    {
      const QgsPoint &point = sample.point;
      if ( !std::isfinite( point.z() ) || point.z() < eaveHeight + 0.05 || point.z() > upperHeight )
        continue;
      const QgsPointXY pointXY( point.x(), point.y() );
      if ( !pointInRing( ring, pointXY ) )
        continue;

      for ( int edgeIndex = 0; edgeIndex < ring.size(); ++edgeIndex )
      {
        const QgsPointXY &a = ring.at( edgeIndex );
        const QgsPointXY &b = ring.at( ( edgeIndex + 1 ) % ring.size() );
        if ( !pointInsideTriangle2d( pointXY, a, b, apexXY ) )
          continue;

        const double edgeX = b.x() - a.x();
        const double edgeY = b.y() - a.y();
        const double denominator = cross2d( edgeX, edgeY, apexXY.x() - a.x(), apexXY.y() - a.y() );
        if ( std::fabs( denominator ) <= 1e-10 )
          continue;

        const double fraction = cross2d( edgeX, edgeY, point.x() - a.x(), point.y() - a.y() ) / denominator;
        if ( fraction < 0.18 || fraction > 1.001 )
          continue;

        const double gradientSquared = ( edgeX * edgeX + edgeY * edgeY ) / ( denominator * denominator );
        constexpr double pi = 3.14159265358979323846;
        const double angle = std::atan2( point.y() - apexXY.y(), point.x() - apexXY.x() );
        const int sector = std::max( 0, std::min( 7, static_cast<int>( ( angle + pi ) * ( 8.0 / ( 2.0 * pi ) ) ) ) );
        roofSamples.append( ApexSurfaceSample{ point.z(), std::min( 1.0, fraction ), gradientSquared, sector } );
        break;
      }
    }
    if ( roofSamples.size() < 16 )
      return false;

    QVector<std::pair<double, double>> riseEstimates;
    riseEstimates.reserve( roofSamples.size() );
    double totalWeight = 0.0;
    for ( const ApexSurfaceSample &sample : roofSamples )
    {
      const double rise = ( sample.z - eaveHeight ) / sample.apexFraction;
      if ( !std::isfinite( rise ) || rise <= 0.0 )
        continue;
      const double weight = sample.apexFraction * sample.apexFraction;
      riseEstimates.append( std::make_pair( rise, weight ) );
      totalWeight += weight;
    }
    if ( riseEstimates.size() < 16 || totalWeight <= 1e-10 )
      return false;

    std::sort( riseEstimates.begin(), riseEstimates.end(), []( const auto &left, const auto &right ) {
      return left.first < right.first;
    } );
    double seedRise = riseEstimates.last().first;
    double accumulatedWeight = 0.0;
    for ( const auto &estimate : riseEstimates )
    {
      accumulatedWeight += estimate.second;
      if ( accumulatedWeight >= totalWeight * 0.5 )
      {
        seedRise = estimate.first;
        break;
      }
    }

    auto orthogonalResidual = [eaveHeight]( const ApexSurfaceSample &sample, double rise ) {
      const double verticalResidual = sample.z - eaveHeight - sample.apexFraction * rise;
      return verticalResidual / std::sqrt( 1.0 + rise * rise * sample.gradientSquared );
    };

    const double minRise = std::max( 0.20, seedRise * 0.40 );
    const double maxRise = std::max( minRise + 0.25, std::max( seedRise * 1.8, ( upperHeight - eaveHeight ) * 1.6 ) );
    const double ransacTolerance = std::max( 0.18, std::min( 0.45, seedRise * 0.08 ) );
    unsigned int availableSectors = 0;
    for ( const ApexSurfaceSample &sample : roofSamples )
      availableSectors |= 1u << sample.sector;
    auto sectorCount = []( unsigned int sectors ) {
      int count = 0;
      while ( sectors )
      {
        count += sectors & 1u;
        sectors >>= 1;
      }
      return count;
    };

    QVector<ApexSurfaceSample> ransacInliers;
    double bestRansacRise = seedRise;
    double bestScore = -1.0;
    double bestSquaredError = std::numeric_limits<double>::max();
    quint32 randomState = 2166136261u ^ static_cast<quint32>( roofSamples.size() * 16777619u );
    const int iterations = std::min( 320, std::max( 80, roofSamples.size() ) );
    for ( int iteration = -1; iteration < iterations; ++iteration )
    {
      randomState = randomState * 1664525u + 1013904223u;
      const ApexSurfaceSample &hypothesis = roofSamples.at( randomState % roofSamples.size() );
      const double candidateRise = iteration < 0 ? seedRise : ( hypothesis.z - eaveHeight ) / hypothesis.apexFraction;
      if ( candidateRise < minRise || candidateRise > maxRise )
        continue;

      QVector<ApexSurfaceSample> inliers;
      inliers.reserve( roofSamples.size() );
      double squaredError = 0.0;
      unsigned int coveredSectors = 0;
      for ( const ApexSurfaceSample &sample : roofSamples )
      {
        const double residual = orthogonalResidual( sample, candidateRise );
        if ( std::fabs( residual ) > ransacTolerance )
          continue;
        inliers.append( sample );
        squaredError += residual * residual;
        coveredSectors |= 1u << sample.sector;
      }

      const double coverage = static_cast<double>( sectorCount( coveredSectors ) ) / std::max( 1, sectorCount( availableSectors ) );
      const double score = inliers.size() * ( 0.75 + 0.25 * coverage );
      if ( score > bestScore || ( score == bestScore && squaredError < bestSquaredError ) )
      {
        bestScore = score;
        bestSquaredError = squaredError;
        bestRansacRise = candidateRise;
        ransacInliers = inliers;
      }
    }

    const int minimumConsensus = std::max( 16, static_cast<int>( std::ceil( roofSamples.size() * 0.20 ) ) );
    const bool hasConsensus = ransacInliers.size() >= minimumConsensus;
    const QVector<ApexSurfaceSample> &fitSamples = hasConsensus ? ransacInliers : roofSamples;
    double fittedRise = hasConsensus ? bestRansacRise : seedRise;

    double inlierTolerance = ransacTolerance * 1.25;
    if ( !hasConsensus )
    {
      QVector<double> initialResiduals;
      initialResiduals.reserve( roofSamples.size() );
      for ( const ApexSurfaceSample &sample : roofSamples )
        initialResiduals.append( std::fabs( orthogonalResidual( sample, seedRise ) ) );
      std::sort( initialResiduals.begin(), initialResiduals.end() );
      inlierTolerance = std::max( 0.18, std::min( std::max( 0.45, seedRise * 0.12 ), sortedPercentile( initialResiduals, 0.50 ) * 2.5 ) );
    }

    for ( int pass = 0; pass < 2; ++pass )
    {
      QVector<ApexSurfaceSample> inliers;
      inliers.reserve( fitSamples.size() );
      for ( const ApexSurfaceSample &sample : fitSamples )
      {
        if ( std::fabs( orthogonalResidual( sample, fittedRise ) ) <= inlierTolerance )
          inliers.append( sample );
      }
      if ( inliers.size() < std::max( 12, static_cast<int>( std::ceil( fitSamples.size() * 0.20 ) ) ) )
        return false;

      auto squaredDistanceSum = [&inliers, &orthogonalResidual]( double rise ) {
        double sum = 0.0;
        for ( const ApexSurfaceSample &sample : inliers )
        {
          const double residual = orthogonalResidual( sample, rise );
          sum += residual * residual;
        }
        return sum;
      };

      constexpr int coarseSteps = 32;
      const double coarseStep = ( maxRise - minRise ) / coarseSteps;
      int bestStep = 0;
      double bestScore = std::numeric_limits<double>::max();
      for ( int step = 0; step <= coarseSteps; ++step )
      {
        const double score = squaredDistanceSum( minRise + coarseStep * step );
        if ( score < bestScore )
        {
          bestScore = score;
          bestStep = step;
        }
      }

      double low = minRise + coarseStep * std::max( 0, bestStep - 1 );
      double high = minRise + coarseStep * std::min( coarseSteps, bestStep + 1 );
      constexpr double goldenRatio = 0.6180339887498949;
      for ( int iteration = 0; iteration < 32; ++iteration )
      {
        const double left = high - ( high - low ) * goldenRatio;
        const double right = low + ( high - low ) * goldenRatio;
        if ( squaredDistanceSum( left ) < squaredDistanceSum( right ) )
          high = right;
        else
          low = left;
      }
      fittedRise = 0.5 * ( low + high );
    }

    if ( !std::isfinite( fittedRise ) || fittedRise <= 0.20 )
      return false;
    apexHeight = eaveHeight + fittedRise;
    return true;
  }

  QList<BuildingRoof::RoofPoint> topHeightApexRoofPoints( const QVector<QgsPointXY> &ring, const QList<BuildingRoof::RoofPoint> &boundaries, const QList<BuildingRoof::RoofPoint> &vertices,
                                                       const QVector<BuildingRoof::RoofSample> &pointCloudSamples, const QVector<BuildingRoof::RoofSample> &perimeterSamples )
  {
    QList<BuildingRoof::RoofPoint> points;
    if ( boundaries.size() != 1 || vertices.size() != 1 )
      return points;

    QgsPoint apexPoint;
    if ( !automaticApexPointFromPointCloud( ring, pointCloudSamples, QgsPoint(), apexPoint ) )
      return points;

    const QgsPointXY footprintCenter = ringCentroidPoint( ring );
    apexPoint.setX( footprintCenter.x() );
    apexPoint.setY( footprintCenter.y() );

    double boundaryHeight = 0.0;
    if ( !automaticBoundaryHeightFromPerimeterBand( ring, perimeterSamples, apexPoint, boundaryHeight ) )
      return points;
    const QgsPoint boundaryPoint( ring.first().x(), ring.first().y(), boundaryHeight );

    double fittedApexHeight = apexPoint.z();
    if ( fitApexHeightFromRoofSurface( ring, pointCloudSamples, footprintCenter, boundaryPoint.z(), fittedApexHeight ) )
      apexPoint.setZ( fittedApexHeight );

    if ( apexPoint.z() <= boundaryPoint.z() + 1e-6 )
      apexPoint.setZ( boundaryPoint.z() + std::max( 0.30, ringExtentSize( ring ) * 0.01 ) );

    points.append( BuildingRoof::RoofPoint{ boundaryPoint, boundaries.first().type } );
    points.append( BuildingRoof::RoofPoint{ apexPoint, vertices.first().type } );
    return points;
  }

  QList<BuildingRoof::RoofPoint> topHeightSingleSurfaceRoofPoints( const QVector<QgsPointXY> &ring, const QList<BuildingRoof::RoofPoint> &boundaries, const QList<BuildingRoof::RoofPoint> &surfaces,
                                                                const QVector<BuildingRoof::RoofSample> &pointCloudSamples, const QVector<BuildingRoof::RoofSample> &perimeterSamples )
  {
    QList<BuildingRoof::RoofPoint> points;
    if ( boundaries.size() != 1 || surfaces.size() != 1 )
      return points;

    QgsPoint surfacePoint;
    if ( !automaticApexPointFromPointCloud( ring, pointCloudSamples, QgsPoint(), surfacePoint ) )
      return points;

    const QgsPointXY center = ringCentroidPoint( ring );
    surfacePoint.setX( center.x() );
    surfacePoint.setY( center.y() );
    double boundaryHeight = 0.0;
    if ( !automaticBoundaryHeightFromPerimeterBand( ring, perimeterSamples, surfacePoint, boundaryHeight ) )
      return points;
    const QgsPoint boundaryPoint( ring.first().x(), ring.first().y(), boundaryHeight );

    if ( surfacePoint.z() <= boundaryPoint.z() + 1e-6 )
      surfacePoint.setZ( boundaryPoint.z() + std::max( 0.30, ringExtentSize( ring ) * 0.01 ) );

    points.append( BuildingRoof::RoofPoint{ boundaryPoint, boundaries.first().type } );
    points.append( BuildingRoof::RoofPoint{ surfacePoint, surfaces.first().type } );
    return points;
  }

  double maxDistanceToPointOnRing( const QVector<QgsPointXY> &ring, const QgsPointXY &center )
  {
    double maxDistance = 0.0;
    for ( const QgsPointXY &point : ring )
      maxDistance = std::max( maxDistance, std::hypot( point.x() - center.x(), point.y() - center.y() ) );
    return maxDistance;
  }

  double sphericalCapHeight( double distance, double supportRadius, double rise )
  {
    if ( rise <= 1e-8 || supportRadius <= 1e-8 )
      return 0.0;

    const double sphereRadius = ( supportRadius * supportRadius + rise * rise ) / ( 2.0 * rise );
    const double inside = sphereRadius * sphereRadius - distance * distance;
    if ( inside <= 0.0 )
      return 0.0;

    return std::max( 0.0, std::sqrt( inside ) - ( sphereRadius - rise ) );
  }

  double domeRoofZ( const QgsPointXY &point, const QgsPoint &surfacePoint, const QVector<QgsPointXY> &ring, double baseHeight )
  {
    const double rise = surfacePoint.z() - baseHeight;
    if ( rise <= 0.0 )
      return baseHeight;

    const QgsPointXY center( surfacePoint.x(), surfacePoint.y() );
    const double supportRadius = maxDistanceToPointOnRing( ring, center );
    const double distance = std::hypot( point.x() - center.x(), point.y() - center.y() );
    const double capRise = std::min( rise, supportRadius );
    return capRise > 1e-8 ? baseHeight + rise * sphericalCapHeight( distance, supportRadius, capRise ) / capRise : baseHeight;
  }

  QList<BuildingRoof::RoofPoint> fittedDomeRoofPoints( const QVector<QgsPointXY> &ring, const QList<BuildingRoof::RoofPoint> &boundaries,
                                                       const QList<BuildingRoof::RoofPoint> &surfaces,
                                                       const QVector<BuildingRoof::RoofSample> &pointCloudSamples, const QVector<BuildingRoof::RoofSample> &perimeterSamples )
  {
    QList<BuildingRoof::RoofPoint> result;
    if ( boundaries.size() != 1 || surfaces.size() != 1 || pointCloudSamples.size() < 48 )
      return result;

    const QgsPointXY center = ringCentroidPoint( ring );
    const double supportRadius = maxDistanceToPointOnRing( ring, center );
    if ( !pointInRing( ring, center ) || supportRadius <= 1e-6 )
      return result;

    QgsPoint approximateTop;
    if ( !automaticApexPointFromPointCloud( ring, pointCloudSamples, QgsPoint(), approximateTop ) )
      return result;
    double eaveHeight = 0.0;
    if ( !automaticBoundaryHeightFromPerimeterBand( ring, perimeterSamples, approximateTop, eaveHeight ) )
      return result;

    struct DomeSample
    {
      double radiusSquared = 0.0;
      double z = 0.0;
      int sector = 0;
    };
    QVector<DomeSample> samples;
    QVector<double> heights;
    samples.reserve( pointCloudSamples.size() );
    heights.reserve( pointCloudSamples.size() );
    constexpr double pi = 3.14159265358979323846;
    for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
    {
      const QgsPoint &point = sample.point;
      if ( !std::isfinite( point.z() ) || point.z() < std::max( 0.0, eaveHeight - 0.35 )
           || !pointInRing( ring, QgsPointXY( point.x(), point.y() ) ) )
        continue;
      const double dx = point.x() - center.x();
      const double dy = point.y() - center.y();
      const int sector = std::max( 0, std::min( 7, static_cast<int>( 8.0 * ( std::atan2( dy, dx ) + pi ) / ( 2.0 * pi ) ) ) );
      samples.append( DomeSample{ dx * dx + dy * dy, point.z(), sector } );
      heights.append( point.z() );
    }
    if ( samples.size() < 48 )
      return result;

    std::sort( heights.begin(), heights.end() );
    const double topEstimate = std::min( approximateTop.z(), sortedPercentile( heights, 0.98 ) );
    const double riseEstimate = topEstimate - eaveHeight;
    if ( riseEstimate < 0.25 || riseEstimate >= supportRadius )
      return result;

    const double threshold = std::max( 0.22, std::min( 0.60, riseEstimate * 0.08 ) );
    const int minimumInliers = std::max( 32, static_cast<int>( samples.size() * 0.12 ) );
    const double priorWeight = std::max( 1.0, samples.size() * 0.002 );
    double bestScore = std::numeric_limits<double>::max();
    double bestCenterZ = 0.0;
    double bestRadius = 0.0;

    for ( double topSeed : { sortedPercentile( heights, 0.94 ), topEstimate } )
    {
      for ( double eaveOffset : { -0.10, 0.0, 0.10 } )
      {
        const double edgeSeed = eaveHeight + eaveOffset * riseEstimate;
        const double rise = topSeed - edgeSeed;
        if ( rise <= 0.20 || rise >= supportRadius )
          continue;
        double radius = ( supportRadius * supportRadius + rise * rise ) / ( 2.0 * rise );
        double centerZ = topSeed - radius;
        radius = std::max( radius, supportRadius * 1.001 );

        for ( int iteration = 0; iteration < 6; ++iteration )
        {
          double sum00 = 0.0;
          double sum01 = 0.0;
          double sum11 = 0.0;
          double right0 = 0.0;
          double right1 = 0.0;
          int accepted = 0;
          for ( const DomeSample &sample : samples )
          {
            const double root = std::sqrt( std::max( 1e-8, radius * radius - sample.radiusSquared ) );
            const double residual = sample.z - centerZ - root;
            const double cutoff = iteration == 0 ? threshold * 3.0 : threshold * 1.5;
            if ( std::fabs( residual ) > cutoff )
              continue;
            const double weight = std::min( 1.0, threshold / std::max( 1e-8, std::fabs( residual ) ) );
            const double slope = radius / root;
            sum00 += weight;
            sum01 += weight * slope;
            sum11 += weight * slope * slope;
            right0 += weight * residual;
            right1 += weight * slope * residual;
            ++accepted;
          }
          if ( accepted < minimumInliers )
            break;

          const double edgeRoot = std::sqrt( std::max( 1e-8, radius * radius - supportRadius * supportRadius ) );
          const double edgeResidual = eaveHeight - centerZ - edgeRoot;
          const double edgeSlope = radius / edgeRoot;
          sum00 += priorWeight * 2.0;
          sum01 += priorWeight * ( 1.0 + edgeSlope );
          sum11 += priorWeight * ( 1.0 + edgeSlope * edgeSlope );
          right0 += priorWeight * ( topEstimate - centerZ - radius + edgeResidual );
          right1 += priorWeight * ( topEstimate - centerZ - radius + edgeSlope * edgeResidual );

          const double determinant = sum00 * sum11 - sum01 * sum01;
          if ( determinant <= 1e-10 )
            break;
          const double deltaCenter = ( right0 * sum11 - right1 * sum01 ) / determinant;
          const double deltaRadius = ( right1 * sum00 - right0 * sum01 ) / determinant;
          centerZ += std::max( -riseEstimate * 0.25, std::min( riseEstimate * 0.25, deltaCenter ) );
          radius = std::max( supportRadius * 1.001,
                             radius + std::max( -supportRadius * 0.25, std::min( supportRadius * 0.25, deltaRadius ) ) );
        }

        const double edgeZ = centerZ + std::sqrt( radius * radius - supportRadius * supportRadius );
        const double topZ = centerZ + radius;
        if ( !std::isfinite( edgeZ ) || !std::isfinite( topZ ) || edgeZ <= 0.0 || topZ <= edgeZ + 0.20
             || std::fabs( edgeZ - eaveHeight ) > std::max( 0.70, riseEstimate * 0.25 )
             || std::fabs( topZ - topEstimate ) > std::max( 0.80, riseEstimate * 0.30 ) )
          continue;

        int inliers = 0;
        unsigned int sectors = 0;
        unsigned int radialBands = 0;
        double error = 0.0;
        for ( const DomeSample &sample : samples )
        {
          const double predicted = centerZ + std::sqrt( std::max( 0.0, radius * radius - sample.radiusSquared ) );
          const double residual = sample.z - predicted;
          error += std::min( residual * residual, threshold * threshold );
          if ( std::fabs( residual ) > threshold )
            continue;
          ++inliers;
          sectors |= 1u << sample.sector;
          const int band = std::max( 0, std::min( 2, static_cast<int>( 3.0 * std::sqrt( sample.radiusSquared ) / supportRadius ) ) );
          radialBands |= 1u << band;
        }
        int coveredSectors = 0;
        int coveredBands = 0;
        for ( int i = 0; i < 8; ++i )
          coveredSectors += ( sectors >> i ) & 1u;
        for ( int i = 0; i < 3; ++i )
          coveredBands += ( radialBands >> i ) & 1u;
        if ( inliers < minimumInliers || coveredSectors < 4 || coveredBands < 2 )
          continue;

        const double score = error / samples.size() + 0.05 * std::pow( edgeZ - eaveHeight, 2.0 )
                             + 0.05 * std::pow( topZ - topEstimate, 2.0 );
        if ( score < bestScore )
        {
          bestScore = score;
          bestCenterZ = centerZ;
          bestRadius = radius;
        }
      }
    }
    if ( !std::isfinite( bestScore ) || bestScore == std::numeric_limits<double>::max() )
      return result;

    BuildingRoof::RoofPoint fittedBoundary = boundaries.first();
    fittedBoundary.point = QgsPoint( ring.first().x(), ring.first().y(), bestCenterZ + std::sqrt( bestRadius * bestRadius - supportRadius * supportRadius ) );
    BuildingRoof::RoofPoint fittedSurface = surfaces.first();
    fittedSurface.point = QgsPoint( center.x(), center.y(), bestCenterZ + bestRadius );
    result.append( fittedBoundary );
    result.append( fittedSurface );
    return result;
  }

  double balancedBarrelTopHeight( const QgsPoint &surfaceStart, const QgsPoint &surfaceEnd, double u, double axisLength )
  {
    Q_UNUSED( u )
    Q_UNUSED( axisLength )

    const double startZ = surfaceStart.z();
    const double endZ = surfaceEnd.z();
    return 0.5 * ( startZ + endZ );
  }

  bool snapDirectionToClosestRingEdge( const QVector<QgsPointXY> &ring, double &axisX, double &axisY )
  {
    constexpr double snapAngleRadians = 15.0 * 3.14159265358979323846 / 180.0;
    const double minDot = std::cos( snapAngleRadians );

    double bestAbsDot = -1.0;
    double bestSignedDot = 1.0;
    double bestX = axisX;
    double bestY = axisY;

    for ( int i = 0; i < ring.size(); ++i )
    {
      const QgsPointXY &a = ring.at( i );
      const QgsPointXY &b = ring.at( ( i + 1 ) % ring.size() );
      double edgeX = b.x() - a.x();
      double edgeY = b.y() - a.y();
      const double edgeLength = std::hypot( edgeX, edgeY );
      if ( edgeLength <= 1e-8 )
        continue;

      edgeX /= edgeLength;
      edgeY /= edgeLength;
      const double signedDot = axisX * edgeX + axisY * edgeY;
      const double absDot = std::fabs( signedDot );
      if ( absDot > bestAbsDot )
      {
        bestAbsDot = absDot;
        bestSignedDot = signedDot;
        bestX = edgeX;
        bestY = edgeY;
      }
    }

    if ( bestAbsDot < minDot )
      return false;

    axisX = bestSignedDot < 0.0 ? -bestX : bestX;
    axisY = bestSignedDot < 0.0 ? -bestY : bestY;
    return true;
  }

  double barrelProfileHeight( double s, double minS, double centerS, double maxS, double baseHeight, double topHeight )
  {
    const double rise = topHeight - baseHeight;
    if ( rise <= 1e-8 )
      return baseHeight;

    const double supportRadius = s < centerS ? centerS - minS : maxS - centerS;
    if ( supportRadius <= 1e-8 )
      return baseHeight;

    const double distance = std::min( supportRadius, std::fabs( s - centerS ) );
    const double capRise = std::min( rise, supportRadius );
    return baseHeight + rise * sphericalCapHeight( distance, supportRadius, capRise ) / capRise;
  }

  bool barrelProfileFrame( const QVector<QgsPointXY> &ring, const QgsPoint &surfaceStart, const QgsPoint &surfaceEnd, double &normalX, double &normalY, double &minS, double &centerS, double &maxS, double &axisLength )
  {
    if ( ring.size() < 3 )
      return false;

    const QgsPointXY a( surfaceStart.x(), surfaceStart.y() );
    const QgsPointXY b( surfaceEnd.x(), surfaceEnd.y() );
    const double dx = b.x() - a.x();
    const double dy = b.y() - a.y();
    axisLength = std::hypot( dx, dy );
    if ( axisLength <= 1e-8 )
      return false;

    double axisX = dx / axisLength;
    double axisY = dy / axisLength;
    snapDirectionToClosestRingEdge( ring, axisX, axisY );

    normalX = -axisY;
    normalY = axisX;
    minS = std::numeric_limits<double>::max();
    maxS = -std::numeric_limits<double>::max();
    for ( const QgsPointXY &ringPoint : ring )
    {
      const double s = profileDistance( ringPoint, normalX, normalY );
      minS = std::min( minS, s );
      maxS = std::max( maxS, s );
    }

    centerS = 0.5 * ( minS + maxS );
    return maxS - minS > 1e-8;
  }

  bool barrelQuadrilateralEnds( const QVector<QgsPointXY> &ring, double normalX, double normalY,
                                QgsPointXY &leftA, QgsPointXY &rightA, QgsPointXY &leftB, QgsPointXY &rightB )
  {
    QVector<QgsPointXY> corners = ring;
    for ( int pass = 0; pass < ring.size() && corners.size() > 4; ++pass )
    {
      bool removed = false;
      for ( int i = 0; i < corners.size(); ++i )
      {
        const QgsPointXY &a = corners.at( ( i - 1 + corners.size() ) % corners.size() );
        const QgsPointXY &b = corners.at( i );
        const QgsPointXY &c = corners.at( ( i + 1 ) % corners.size() );
        const double abX = b.x() - a.x();
        const double abY = b.y() - a.y();
        const double bcX = c.x() - b.x();
        const double bcY = c.y() - b.y();
        const double lengths = std::hypot( abX, abY ) * std::hypot( bcX, bcY );
        if ( lengths > 1e-12 && abX * bcX + abY * bcY > 0.0
             && std::fabs( cross2d( abX, abY, bcX, bcY ) ) / lengths < 0.03 )
        {
          corners.removeAt( i );
          removed = true;
          break;
        }
      }
      if ( !removed )
        break;
    }
    if ( corners.size() != 4 )
      return false;
    for ( int i = 0; i < 4; ++i )
    {
      const QgsPointXY &a = corners.at( i );
      const QgsPointXY &b = corners.at( ( i + 1 ) % 4 );
      const QgsPointXY &c = corners.at( ( i + 2 ) % 4 );
      if ( cross2d( b.x() - a.x(), b.y() - a.y(), c.x() - b.x(), c.y() - b.y() ) <= 1e-8 )
        return false;
    }

    int bestEdge = -1;
    double bestAlignment = 0.75;
    for ( int edge = 0; edge < 2; ++edge )
    {
      const QgsPointXY &a = corners.at( edge );
      const QgsPointXY &b = corners.at( edge + 1 );
      const QgsPointXY &c = corners.at( edge + 2 );
      const QgsPointXY &d = corners.at( ( edge + 3 ) % 4 );
      const double firstLength = std::hypot( b.x() - a.x(), b.y() - a.y() );
      const double secondLength = std::hypot( d.x() - c.x(), d.y() - c.y() );
      if ( firstLength <= 1e-8 || secondLength <= 1e-8 )
        continue;
      const double firstAlignment = std::fabs( ( b.x() - a.x() ) * normalX + ( b.y() - a.y() ) * normalY ) / firstLength;
      const double secondAlignment = std::fabs( ( d.x() - c.x() ) * normalX + ( d.y() - c.y() ) * normalY ) / secondLength;
      const double alignment = std::min( firstAlignment, secondAlignment );
      if ( alignment > bestAlignment )
      {
        bestAlignment = alignment;
        bestEdge = edge;
      }
    }
    if ( bestEdge < 0 )
      return false;

    leftA = corners.at( bestEdge );
    rightA = corners.at( ( bestEdge + 1 ) % 4 );
    leftB = corners.at( ( bestEdge + 2 ) % 4 );
    rightB = corners.at( ( bestEdge + 3 ) % 4 );
    if ( profileDistance( leftA, normalX, normalY ) > profileDistance( rightA, normalX, normalY ) )
      std::swap( leftA, rightA );
    if ( profileDistance( leftB, normalX, normalY ) > profileDistance( rightB, normalX, normalY ) )
      std::swap( leftB, rightB );
    return true;
  }

  double barrelProfileSAt( const QVector<QgsPointXY> &ring, const QgsPointXY &point,
                           double normalX, double normalY, double minS, double maxS )
  {
    QgsPointXY leftA, rightA, leftB, rightB;
    if ( !barrelQuadrilateralEnds( ring, normalX, normalY, leftA, rightA, leftB, rightB ) )
      return profileDistance( point, normalX, normalY );

    const double ux = rightA.x() - leftA.x();
    const double uy = rightA.y() - leftA.y();
    const double vx = leftB.x() - leftA.x();
    const double vy = leftB.y() - leftA.y();
    const double wx = rightB.x() - leftB.x() - ux;
    const double wy = rightB.y() - leftB.y() - uy;
    const double centerAxisX = ( leftB.x() + rightB.x() - leftA.x() - rightA.x() ) * 0.5;
    const double centerAxisY = ( leftB.y() + rightB.y() - leftA.y() - rightA.y() ) * 0.5;
    const double axisLength2 = centerAxisX * centerAxisX + centerAxisY * centerAxisY;
    double fraction = std::max( 0.0, std::min( 1.0, ( profileDistance( point, normalX, normalY ) - minS ) / ( maxS - minS ) ) );
    double along = axisLength2 > 1e-12
                     ? ( ( point.x() - leftA.x() - ux * 0.5 ) * centerAxisX
                         + ( point.y() - leftA.y() - uy * 0.5 ) * centerAxisY ) / axisLength2
                     : 0.5;
    along = std::max( 0.0, std::min( 1.0, along ) );
    for ( int iteration = 0; iteration < 8; ++iteration )
    {
      const double residualX = leftA.x() + fraction * ux + along * vx + fraction * along * wx - point.x();
      const double residualY = leftA.y() + fraction * uy + along * vy + fraction * along * wy - point.y();
      const double fractionX = ux + along * wx;
      const double fractionY = uy + along * wy;
      const double alongX = vx + fraction * wx;
      const double alongY = vy + fraction * wy;
      const double determinant = cross2d( fractionX, fractionY, alongX, alongY );
      if ( std::fabs( determinant ) <= 1e-12 )
        break;
      const double stepFraction = cross2d( residualX, residualY, alongX, alongY ) / determinant;
      const double stepAlong = cross2d( fractionX, fractionY, residualX, residualY ) / determinant;
      fraction = std::max( 0.0, std::min( 1.0, fraction - stepFraction ) );
      along = std::max( 0.0, std::min( 1.0, along - stepAlong ) );
      if ( std::fabs( stepFraction ) + std::fabs( stepAlong ) < 1e-9 )
        break;
    }
    return minS + ( maxS - minS ) * fraction;
  }

  QVector<ProfileAnchor> barrelProfileAnchors( const QVector<QgsPointXY> &ring, const QgsPoint &surfaceStart, const QgsPoint &surfaceEnd, double baseHeight, double &normalX, double &normalY )
  {
    QVector<ProfileAnchor> profileAnchors;
    double minS = 0.0;
    double centerS = 0.0;
    double maxS = 0.0;
    double axisLength = 0.0;
    if ( !barrelProfileFrame( ring, surfaceStart, surfaceEnd, normalX, normalY, minS, centerS, maxS, axisLength ) )
      return profileAnchors;

    const double topHeight = balancedBarrelTopHeight( surfaceStart, surfaceEnd, 0.0, axisLength );
    if ( topHeight <= baseHeight + 1e-8 )
      return profileAnchors;

    constexpr int sideSegments = 24;
    appendProfileAnchor( profileAnchors, minS, baseHeight );
    for ( int i = 1; i <= sideSegments; ++i )
    {
      const double s = minS + ( centerS - minS ) * i / sideSegments;
      appendProfileAnchor( profileAnchors, s, barrelProfileHeight( s, minS, centerS, maxS, baseHeight, topHeight ) );
    }
    for ( int i = 1; i <= sideSegments; ++i )
    {
      const double s = centerS + ( maxS - centerS ) * i / sideSegments;
      appendProfileAnchor( profileAnchors, s, barrelProfileHeight( s, minS, centerS, maxS, baseHeight, topHeight ) );
    }
    appendProfileAnchor( profileAnchors, maxS, baseHeight );

    std::sort( profileAnchors.begin(), profileAnchors.end(), []( const ProfileAnchor &lhs, const ProfileAnchor &rhs ) {
      return lhs.s < rhs.s;
    } );
    return profileAnchors;
  }

  bool automaticBarrelBounds( const QVector<QgsPointXY> &ring, const QVector<BuildingRoof::RoofSample> &pointCloudSamples,
                              double normalX, double normalY, double ridgeS, double eaveHeight, double topHeight,
                              double &minS, double &maxS )
  {
    const double footprintMin = minS;
    const double width = maxS - minS;
    const double rise = topHeight - eaveHeight;
    if ( width <= 1e-8 || rise < 0.30 )
      return false;

    const double axisX = normalY;
    const double axisY = -normalX;
    double minAxis = std::numeric_limits<double>::max();
    double maxAxis = -std::numeric_limits<double>::max();
    for ( const QgsPointXY &point : ring )
    {
      const double axis = profileDistance( point, axisX, axisY );
      minAxis = std::min( minAxis, axis );
      maxAxis = std::max( maxAxis, axis );
    }
    if ( maxAxis - minAxis <= 1e-8 )
      return false;

    constexpr int binCount = 64;
    struct Bin
    {
      int total = 0;
      int roof = 0;
      unsigned int axisMask = 0;
    };
    QVector<Bin> bins( binCount );
    const double roofMinimum = eaveHeight + std::max( 0.12, rise * 0.05 );
    const double roofMaximum = topHeight + std::max( 0.35, rise * 0.15 );
    for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
    {
      const QgsPoint &point = sample.point;
      if ( !std::isfinite( point.z() ) || !pointInRing( ring, QgsPointXY( point.x(), point.y() ) ) )
        continue;
      const double s = point.x() * normalX + point.y() * normalY;
      if ( s < footprintMin || s > maxS )
        continue;
      const int bin = std::max( 0, std::min( binCount - 1, static_cast<int>( binCount * ( s - footprintMin ) / width ) ) );
      ++bins[bin].total;
      if ( point.z() < roofMinimum || point.z() > roofMaximum )
        continue;
      ++bins[bin].roof;
      const double axis = point.x() * axisX + point.y() * axisY;
      const int axisBin = std::max( 0, std::min( 7, static_cast<int>( 8.0 * ( axis - minAxis ) / ( maxAxis - minAxis ) ) ) );
      bins[bin].axisMask |= 1u << axisBin;
    }

    const int ridgeBin = std::max( 0, std::min( binCount - 1, static_cast<int>( binCount * ( ridgeS - footprintMin ) / width ) ) );
    int referenceCount = 0;
    for ( int bin = std::max( 0, ridgeBin - binCount / 5 ); bin <= std::min( binCount - 1, ridgeBin + binCount / 5 ); ++bin )
      referenceCount = std::max( referenceCount, bins.at( bin ).roof );
    if ( referenceCount < 8 )
      return false;

    QVector<bool> supported( binCount, false );
    const int minimumRoofPoints = std::max( 4, static_cast<int>( std::ceil( referenceCount * 0.04 ) ) );
    for ( int bin = 0; bin < binCount; ++bin )
    {
      const Bin &entry = bins.at( bin );
      const unsigned int mask = entry.axisMask;
      supported[bin] = entry.roof >= minimumRoofPoints && entry.roof * 10 >= entry.total
                       && ( mask & ( mask - 1 ) ) != 0;
    }
    for ( int bin = 1; bin + 1 < binCount; ++bin )
    {
      if ( supported.at( bin - 1 ) && supported.at( bin + 1 ) )
        supported[bin] = true;
    }

    int seed = -1;
    for ( int offset = 0; offset <= binCount / 5 && seed < 0; ++offset )
    {
      for ( int candidate : { ridgeBin - offset, ridgeBin + offset } )
      {
        if ( candidate >= 0 && candidate < binCount && supported.at( candidate ) )
        {
          seed = candidate;
          break;
        }
      }
    }
    if ( seed < 0 )
      return false;

    int first = seed;
    int last = seed;
    while ( first > 0 && supported.at( first - 1 ) )
      --first;
    while ( last + 1 < binCount && supported.at( last + 1 ) )
      ++last;

    const double binWidth = width / binCount;
    const double roofMin = std::max( footprintMin, footprintMin + ( first - 0.5 ) * binWidth );
    const double roofMax = std::min( maxS, footprintMin + ( last + 1.5 ) * binWidth );
    if ( roofMax - roofMin < width * 0.35 || ridgeS - roofMin < width * 0.10 || roofMax - ridgeS < width * 0.10 )
      return false;
    minS = roofMin;
    maxS = roofMax;
    return true;
  }

  bool automaticBarrelBoundaryAndTop( const QVector<QgsPointXY> &ring, const QList<BuildingRoof::RoofPoint> &surfaces,
                                      const QVector<BuildingRoof::RoofSample> &pointCloudSamples, QgsPoint &boundaryPoint, QgsPoint &topPoint )
  {
    if ( surfaces.size() != 2 || pointCloudSamples.size() < 20 )
      return false;

    double normalX = 0.0;
    double normalY = 0.0;
    double minS = 0.0;
    double centerS = 0.0;
    double maxS = 0.0;
    double axisLength = 0.0;
    if ( !barrelProfileFrame( ring, surfaces.at( 0 ).point, surfaces.at( 1 ).point, normalX, normalY,
                              minS, centerS, maxS, axisLength )
         || !automaticApexPointFromPointCloud( ring, pointCloudSamples, surfaces.first().point, topPoint ) )
      return false;

    const QgsPoint ridgeMidpoint( ( surfaces.at( 0 ).point.x() + surfaces.at( 1 ).point.x() ) * 0.5,
                                  ( surfaces.at( 0 ).point.y() + surfaces.at( 1 ).point.y() ) * 0.5, topPoint.z() );
    const QgsPoint neutralBoundary( ring.first().x(), ring.first().y(), 0.0 );
    if ( !automaticBoundaryPointFromRidgeLine( ring, pointCloudSamples, ridgeMidpoint, normalY, -normalX,
                                                neutralBoundary, boundaryPoint ) )
      return false;

    return std::isfinite( boundaryPoint.z() ) && boundaryPoint.z() > 0.0
           && topPoint.z() > boundaryPoint.z() + 0.25;
  }

  QVector<QPair<double, double>> automaticBarrelAxisDirections( const QVector<QgsPointXY> &ring,
                                                               const QVector<BuildingRoof::RoofSample> &pointCloudSamples )
  {
    QVector<QPair<double, double>> directions;
    bool hasQuadrilateralFrame = false;
    for ( int edge = 0; edge < ring.size(); ++edge )
    {
      const QgsPointXY &a = ring.at( edge );
      const QgsPointXY &b = ring.at( ( edge + 1 ) % ring.size() );
      const double length = std::hypot( b.x() - a.x(), b.y() - a.y() );
      QgsPointXY leftA, rightA, leftB, rightB;
      if ( length > 1e-8 && barrelQuadrilateralEnds( ring, -( b.y() - a.y() ) / length, ( b.x() - a.x() ) / length,
                                                   leftA, rightA, leftB, rightB ) )
      {
        hasQuadrilateralFrame = true;
        break;
      }
    }
    auto appendDirection = [&]( double axisX, double axisY ) {
      double length = std::hypot( axisX, axisY );
      if ( length <= 1e-8 )
        return;
      axisX /= length;
      axisY /= length;
      QgsPointXY leftA, rightA, leftB, rightB;
      if ( barrelQuadrilateralEnds( ring, -axisY, axisX, leftA, rightA, leftB, rightB ) )
      {
        axisX = ( leftB.x() + rightB.x() - leftA.x() - rightA.x() ) * 0.5;
        axisY = ( leftB.y() + rightB.y() - leftA.y() - rightA.y() ) * 0.5;
        length = std::hypot( axisX, axisY );
        if ( length <= 1e-8 )
          return;
        axisX /= length;
        axisY /= length;
      }
      else if ( hasQuadrilateralFrame )
        return;
      for ( const auto &direction : directions )
      {
        if ( std::fabs( axisX * direction.first + axisY * direction.second ) > 0.99985 )
          return;
      }
      directions.append( qMakePair( axisX, axisY ) );
    };
    const double minimumLength = std::max( 1e-8, ringExtentSize( ring ) * 0.01 );
    for ( int edge = 0; edge < ring.size(); ++edge )
    {
      const QgsPointXY &a = ring.at( edge );
      const QgsPointXY &b = ring.at( ( edge + 1 ) % ring.size() );
      const double dx = b.x() - a.x();
      const double dy = b.y() - a.y();
      if ( std::hypot( dx, dy ) < minimumLength )
        continue;
      appendDirection( dx, dy );
      appendDirection( -dy, dx );
    }

    QVector<QgsPoint> points;
    QVector<double> heights;
    for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
    {
      const QgsPoint &point = sample.point;
      if ( std::isfinite( point.x() ) && std::isfinite( point.y() ) && std::isfinite( point.z() ) && point.z() >= 0.0
           && pointInRing( ring, QgsPointXY( point.x(), point.y() ) ) )
      {
        points.append( point );
        heights.append( point.z() );
      }
    }
    if ( heights.size() < 24 )
      return directions;
    std::sort( heights.begin(), heights.end() );
    const double ceiling = sortedPercentile( heights, 0.995 );
    const QgsPointXY origin = ringCentroidPoint( ring );
    for ( double quantile : { 0.60, 0.80, 0.92 } )
    {
      const double floor = sortedPercentile( heights, quantile );
      double sumX = 0.0;
      double sumY = 0.0;
      double sumXX = 0.0;
      double sumXY = 0.0;
      double sumYY = 0.0;
      int count = 0;
      for ( const QgsPoint &point : points )
      {
        if ( point.z() < floor || point.z() > ceiling )
          continue;
        const double x = point.x() - origin.x();
        const double y = point.y() - origin.y();
        sumX += x;
        sumY += y;
        sumXX += x * x;
        sumXY += x * y;
        sumYY += y * y;
        ++count;
      }
      if ( count < 12 )
        continue;
      const double xx = sumXX - sumX * sumX / count;
      const double xy = sumXY - sumX * sumY / count;
      const double yy = sumYY - sumY * sumY / count;
      if ( xx + yy <= 1e-8 )
        continue;
      const double angle = 0.5 * std::atan2( 2.0 * xy, xx - yy );
      appendDirection( std::cos( angle ), std::sin( angle ) );
      appendDirection( -std::sin( angle ), std::cos( angle ) );
    }
    if ( !hasQuadrilateralFrame )
    {
      for ( int degrees = 0; degrees < 180; degrees += 15 )
      {
        const double angle = degrees * 3.14159265358979323846 / 180.0;
        appendDirection( std::cos( angle ), std::sin( angle ) );
      }
    }
    return directions;
  }

  bool fitBarrelProfileFromPointCloud( const QVector<QgsPointXY> &ring, double axisX, double axisY,
                                       const QVector<BuildingRoof::RoofSample> &pointCloudSamples, FittedBarrelProfile &fitted, double &score )
  {
    if ( ring.size() < 3 || pointCloudSamples.size() < 80 )
      return false;
    const double axisLength = std::hypot( axisX, axisY );
    if ( axisLength <= 1e-8 )
      return false;
    axisX /= axisLength;
    axisY /= axisLength;
    const double normalX = -axisY;
    const double normalY = axisX;
    double minS = std::numeric_limits<double>::max();
    double maxS = -std::numeric_limits<double>::max();
    double minAxis = std::numeric_limits<double>::max();
    double maxAxis = -std::numeric_limits<double>::max();
    for ( const QgsPointXY &point : ring )
    {
      const double s = profileDistance( point, normalX, normalY );
      minS = std::min( minS, s );
      maxS = std::max( maxS, s );
      const double axis = profileDistance( point, axisX, axisY );
      minAxis = std::min( minAxis, axis );
      maxAxis = std::max( maxAxis, axis );
    }
    if ( maxS - minS <= 1e-8 || maxAxis - minAxis <= 1e-8 )
      return false;

    QVector<double> allHeights;
    for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
    {
      const QgsPoint &point = sample.point;
      if ( std::isfinite( point.x() ) && std::isfinite( point.y() ) && std::isfinite( point.z() ) && point.z() >= 0.0
           && pointInRing( ring, QgsPointXY( point.x(), point.y() ) ) )
        allHeights.append( point.z() );
    }
    if ( allHeights.size() < 80 )
      return false;
    std::sort( allHeights.begin(), allHeights.end() );
    const double heightSpan = sortedPercentile( allHeights, 0.95 ) - sortedPercentile( allHeights, 0.10 );
    const double residualScale = std::max( 0.25, heightSpan * 0.10 );
    const double heightCeiling = sortedPercentile( allHeights, 0.995 ) + residualScale;
    const int binCount = std::max( 12, std::min( 40, static_cast<int>( std::sqrt( static_cast<double>( allHeights.size() ) ) * 0.75 ) ) );
    const int axisBinCount = std::max( 2, std::min( 6, allHeights.size() / ( binCount * 12 ) ) );
    struct ProjectedSample
    {
      double fraction = 0.0;
      double height = 0.0;
    };
    QVector<QVector<ProjectedSample>> cells( binCount * axisBinCount );
    for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
    {
      const QgsPoint &point = sample.point;
      if ( !std::isfinite( point.x() ) || !std::isfinite( point.y() ) || !std::isfinite( point.z() ) || point.z() < 0.0 || point.z() > heightCeiling
           || !pointInRing( ring, QgsPointXY( point.x(), point.y() ) ) )
        continue;
      const double axis = profileDistance( QgsPointXY( point.x(), point.y() ), axisX, axisY );
      if ( axis < minAxis + ( maxAxis - minAxis ) * 0.08
           || axis > maxAxis - ( maxAxis - minAxis ) * 0.08 )
        continue;
      const double profileS = barrelProfileSAt( ring, QgsPointXY( point.x(), point.y() ), normalX, normalY, minS, maxS );
      const double fraction = std::max( 0.0, std::min( 1.0, ( profileS - minS ) / ( maxS - minS ) ) );
      const int bin = std::max( 0, std::min( binCount - 1,
                                            static_cast<int>( binCount * fraction ) ) );
      const int axisBin = std::max( 0, std::min( axisBinCount - 1,
                                                static_cast<int>( axisBinCount * ( axis - minAxis ) / ( maxAxis - minAxis ) ) ) );
      cells[bin * axisBinCount + axisBin].append( ProjectedSample{ fraction, point.z() } );
    }
    struct ProfileBin
    {
      double arch = 0.0;
      double height = 0.0;
      double weight = 0.0;
    };
    QVector<ProfileBin> profileBins;
    int leftBins = 0;
    int rightBins = 0;
    int occupiedBins = 0;
    // Preserve longitudinal observations: a barrel must explain every axis section with the same arch.
    for ( int bin = 0; bin < binCount; ++bin )
    {
      bool occupied = false;
      for ( int axisBin = 0; axisBin < axisBinCount; ++axisBin )
      {
        QVector<ProjectedSample> &cell = cells[bin * axisBinCount + axisBin];
        if ( cell.size() < 3 )
          continue;
        std::sort( cell.begin(), cell.end(), []( const ProjectedSample &a, const ProjectedSample &b ) { return a.height < b.height; } );
        const ProjectedSample &representative = cell.at( static_cast<int>( ( cell.size() - 1 ) * 0.80 ) );
        const double u = std::fabs( 2.0 * representative.fraction - 1.0 );
        profileBins.append( ProfileBin{ std::sqrt( std::max( 0.0, 1.0 - u * u ) ), representative.height,
                                       std::min( 2.0, std::sqrt( static_cast<double>( cell.size() ) / 3.0 ) ) } );
        occupied = true;
      }
      if ( occupied )
      {
        ++occupiedBins;
        if ( bin < binCount / 2 )
          ++leftBins;
        else
          ++rightBins;
      }
    }
    const int minimumSideBins = std::max( 2, binCount / 10 );
    if ( occupiedBins < std::max( 6, binCount / 3 ) || profileBins.size() < 8
         || leftBins < minimumSideBins || rightBins < minimumSideBins )
      return false;

    double eave = 0.0;
    double rise = 0.0;
    for ( int iteration = 0; iteration < 5; ++iteration )
    {
      double sumWeight = 0.0;
      double sumArch = 0.0;
      double sumArch2 = 0.0;
      double sumHeight = 0.0;
      double sumArchHeight = 0.0;
      for ( const ProfileBin &bin : profileBins )
      {
        const double residual = bin.height - eave - rise * bin.arch;
        const double weight = iteration == 0 ? bin.weight
                                               : bin.weight * std::min( 1.0, residualScale / std::max( 1e-8, std::fabs( residual ) ) );
        sumWeight += weight;
        sumArch += weight * bin.arch;
        sumArch2 += weight * bin.arch * bin.arch;
        sumHeight += weight * bin.height;
        sumArchHeight += weight * bin.arch * bin.height;
      }
      const double determinant = sumWeight * sumArch2 - sumArch * sumArch;
      if ( determinant <= 1e-8 )
        return false;
      eave = ( sumHeight * sumArch2 - sumArchHeight * sumArch ) / determinant;
      rise = ( sumWeight * sumArchHeight - sumArch * sumHeight ) / determinant;
    }
    if ( !std::isfinite( eave ) || !std::isfinite( rise ) || eave <= 0.0 || rise <= 0.25 )
      return false;

    int supportedBins = 0;
    double loss = 0.0;
    double totalWeight = 0.0;
    for ( const ProfileBin &bin : profileBins )
    {
      const double residual = std::fabs( bin.height - eave - rise * bin.arch );
      loss += bin.weight * ( residual <= residualScale ? residual * residual : residualScale * ( 2.0 * residual - residualScale ) );
      totalWeight += bin.weight;
      if ( residual <= std::max( 0.50, residualScale * 2.0 ) )
        ++supportedBins;
    }
    if ( supportedBins < static_cast<int>( std::ceil( profileBins.size() * 0.65 ) ) )
      return false;

    fitted.anchors.clear();
    constexpr int sideSegments = 24;
    for ( int i = 0; i <= 2 * sideSegments; ++i )
    {
      const double fraction = static_cast<double>( i ) / ( 2 * sideSegments );
      const double u = std::fabs( 2.0 * fraction - 1.0 );
      fitted.anchors.append( ProfileAnchor{ minS + ( maxS - minS ) * fraction,
                                            eave + rise * std::sqrt( std::max( 0.0, 1.0 - u * u ) ) } );
    }
    fitted.normalX = normalX;
    fitted.normalY = normalY;
    score = loss / totalWeight + residualScale * residualScale * ( binCount - occupiedBins ) / binCount;
    return true;
  }

  double barrelRoofZ( const QgsPointXY &point, const QgsPoint &surfaceStart, const QgsPoint &surfaceEnd, const QVector<QgsPointXY> &ring, double baseHeight )
  {
    double normalX = 0.0;
    double normalY = 0.0;
    const QVector<ProfileAnchor> profileAnchors = barrelProfileAnchors( ring, surfaceStart, surfaceEnd, baseHeight, normalX, normalY );
    if ( profileAnchors.isEmpty() )
      return baseHeight;

    return profileRoofZ( profileAnchors, barrelProfileSAt( ring, point, normalX, normalY,
                                                          profileAnchors.first().s, profileAnchors.last().s ) );
  }

  double curvedRoofZAt( const QgsPointXY &point, const QVector<QgsPointXY> &ring, const QList<BuildingRoof::RoofPoint> &surfaces, double baseHeight )
  {
    if ( surfaces.size() == 1 )
      return domeRoofZ( point, surfaces.first().point, ring, baseHeight );
    return barrelRoofZ( point, surfaces.at( 0 ).point, surfaces.at( 1 ).point, ring, baseHeight );
  }

  bool rayDistanceToRingBoundary( const QVector<QgsPointXY> &ring, const QgsPointXY &origin, double dirX, double dirY, double &distance )
  {
    const QVector<LineInterval> intervals = lineInsideRingIntervals( ring, origin, dirX, dirY );
    LineInterval containing;
    if ( !findIntervalContaining( intervals, 0.0, 0.0, containing ) )
      return false;

    distance = std::max( 0.0, containing.end );
    return distance > 1e-8;
  }

  void appendDomeRoofSurface( BuildingRoof::Mesh &mesh, const QVector<QgsPointXY> &ring, const QgsPoint &surfacePoint, double baseHeight )
  {
    const double rise = surfacePoint.z() - baseHeight;
    if ( ring.size() < 3 || rise <= 1e-8 )
      return;

    constexpr double twoPi = 6.28318530717958647692;
    const int radialSegments = 32;
    const QgsPointXY center( surfacePoint.x(), surfacePoint.y() );

    QVector<double> angles;
    angles.reserve( 96 + ring.size() );
    for ( int i = 0; i < 96; ++i )
      angles.append( twoPi * i / 96.0 );
    for ( const QgsPointXY &point : ring )
    {
      double angle = std::atan2( point.y() - center.y(), point.x() - center.x() );
      if ( angle < 0.0 )
        angle += twoPi;
      if ( angle >= twoPi - 1e-10 )
        angle = 0.0;
      angles.append( angle );
    }
    std::sort( angles.begin(), angles.end() );
    angles.erase( std::unique( angles.begin(), angles.end(), []( double a, double b ) {
      return std::fabs( a - b ) < 1e-10;
    } ), angles.end() );
    const int angularSegments = angles.size();

    QVector<double> rayDistances;
    rayDistances.reserve( angularSegments );
    double supportRadius = 0.0;
    for ( int i = 0; i < angularSegments; ++i )
    {
      const double angle = angles.at( i );
      double distance = 0.0;
      if ( !rayDistanceToRingBoundary( ring, center, std::cos( angle ), std::sin( angle ), distance ) )
        distance = 0.0;
      rayDistances.append( distance );
      supportRadius = std::max( supportRadius, distance );
    }
    if ( supportRadius <= 1e-8 )
      return;
    const double capRise = std::min( rise, supportRadius );

    const int centerIndex = mesh.vertices.size();
    mesh.vertices.append( QgsPoint( surfacePoint.x(), surfacePoint.y(), surfacePoint.z() ) );

    QVector<QVector<int>> rings;
    rings.reserve( radialSegments );
    for ( int r = 1; r <= radialSegments; ++r )
    {
      const double fraction = static_cast<double>( r ) / radialSegments;
      QVector<int> row;
      row.reserve( angularSegments );
      for ( int i = 0; i < angularSegments; ++i )
      {
        const double angle = angles.at( i );
        const double distance = rayDistances.at( i ) * fraction;
        const QgsPointXY point( center.x() + std::cos( angle ) * distance, center.y() + std::sin( angle ) * distance );
        const double z = baseHeight + rise * sphericalCapHeight( distance, supportRadius, capRise ) / capRise;
        row.append( mesh.vertices.size() );
        mesh.vertices.append( QgsPoint( point.x(), point.y(), z ) );
      }
      rings.append( row );
    }

    const QVector<int> &firstRing = rings.first();
    for ( int i = 0; i < angularSegments; ++i )
      mesh.indices << centerIndex << firstRing.at( i ) << firstRing.at( ( i + 1 ) % angularSegments );

    for ( int r = 1; r < rings.size(); ++r )
    {
      const QVector<int> &previous = rings.at( r - 1 );
      const QVector<int> &current = rings.at( r );
      for ( int i = 0; i < angularSegments; ++i )
      {
        const int next = ( i + 1 ) % angularSegments;
        mesh.indices << previous.at( i ) << current.at( i ) << previous.at( next );
        mesh.indices << previous.at( next ) << current.at( i ) << current.at( next );
      }
    }

    const int meridianStep = std::max( 1, angularSegments / 12 );
    for ( int i = 0; i < angularSegments; i += meridianStep )
    {
      int previous = centerIndex;
      for ( const QVector<int> &row : rings )
      {
        const int current = row.at( i );
        mesh.structureLines.append( qMakePair( mesh.vertices.at( previous ), mesh.vertices.at( current ) ) );
        previous = current;
      }
    }
    for ( int r : { radialSegments / 3, 2 * radialSegments / 3 } )
    {
      const QVector<int> &row = rings.at( r - 1 );
      for ( int i = 0; i < angularSegments; ++i )
        mesh.structureLines.append( qMakePair( mesh.vertices.at( row.at( i ) ), mesh.vertices.at( row.at( ( i + 1 ) % angularSegments ) ) ) );
    }
  }

  void appendBarrelRoofSurface( BuildingRoof::Mesh &mesh, const QVector<QgsPointXY> &ring,
                                const QVector<ProfileAnchor> &profileAnchors, double normalX, double normalY )
  {
    if ( profileAnchors.size() < 3 )
      return;

    QgsPointXY leftA, rightA, leftB, rightB;
    if ( barrelQuadrilateralEnds( ring, normalX, normalY, leftA, rightA, leftB, rightB ) )
    {
      const double profileMin = profileAnchors.first().s;
      const double profileWidth = profileAnchors.last().s - profileMin;
      if ( profileWidth <= 1e-8 )
        return;
      auto interpolate = []( const QgsPointXY &a, const QgsPointXY &b, double t ) {
        return QgsPointXY( a.x() + ( b.x() - a.x() ) * t, a.y() + ( b.y() - a.y() ) * t );
      };
      for ( int i = 0; i + 1 < profileAnchors.size(); ++i )
      {
        const double firstFraction = ( profileAnchors.at( i ).s - profileMin ) / profileWidth;
        const double secondFraction = ( profileAnchors.at( i + 1 ).s - profileMin ) / profileWidth;
        if ( secondFraction - firstFraction <= 1e-8 )
          continue;
        const QgsPointXY a = interpolate( leftA, rightA, firstFraction );
        const QgsPointXY b = interpolate( leftB, rightB, firstFraction );
        const QgsPointXY c = interpolate( leftB, rightB, secondFraction );
        const QgsPointXY d = interpolate( leftA, rightA, secondFraction );
        const int offset = mesh.vertices.size();
        mesh.vertices.append( QgsPoint( a.x(), a.y(), profileAnchors.at( i ).z ) );
        mesh.vertices.append( QgsPoint( b.x(), b.y(), profileAnchors.at( i ).z ) );
        mesh.vertices.append( QgsPoint( c.x(), c.y(), profileAnchors.at( i + 1 ).z ) );
        mesh.vertices.append( QgsPoint( d.x(), d.y(), profileAnchors.at( i + 1 ).z ) );
        if ( cross2d( b.x() - a.x(), b.y() - a.y(), d.x() - a.x(), d.y() - a.y() ) > 0.0 )
          mesh.indices << offset << offset + 1 << offset + 2 << offset << offset + 2 << offset + 3;
        else
          mesh.indices << offset << offset + 2 << offset + 1 << offset << offset + 3 << offset + 2;
      }
      return;
    }

    QVector<double> profileDistances;
    profileDistances.reserve( profileAnchors.size() );
    for ( const ProfileAnchor &anchor : profileAnchors )
      profileDistances.append( anchor.s );

    const QVector<QgsPointXY> profileRing = ringWithProfileIntersections( ring, normalX, normalY, profileDistances );
    for ( int i = 0; i + 1 < profileAnchors.size(); ++i )
    {
      const double a = profileAnchors.at( i ).s;
      const double b = profileAnchors.at( i + 1 ).s;
      if ( b - a <= 1e-8 )
        continue;

      QVector<QgsPointXY> strip = clipPolygonByProfileRange( profileRing, normalX, normalY, a, b );
      if ( strip.size() < 3 )
        continue;

      const int offset = mesh.vertices.size();
      for ( const QgsPointXY &point : strip )
        mesh.vertices.append( QgsPoint( point.x(), point.y(),
                                        profileRoofZ( profileAnchors, barrelProfileSAt( ring, point, normalX, normalY,
                                                                                       profileAnchors.first().s, profileAnchors.last().s ) ) ) );

      const QVector<int> triangles = triangulateRing( strip );
      for ( int t = 0; t + 2 < triangles.size(); t += 3 )
        mesh.indices << offset + triangles[t] << offset + triangles[t + 1] << offset + triangles[t + 2];
    }
  }

  void appendBarrelRoofSurface( BuildingRoof::Mesh &mesh, const QVector<QgsPointXY> &ring, const QgsPoint &surfaceStart, const QgsPoint &surfaceEnd, double baseHeight )
  {
    double normalX = 0.0;
    double normalY = 0.0;
    const QVector<ProfileAnchor> profileAnchors = barrelProfileAnchors( ring, surfaceStart, surfaceEnd, baseHeight, normalX, normalY );
    appendBarrelRoofSurface( mesh, ring, profileAnchors, normalX, normalY );
  }

  QVector<QgsPointXY> curvedRoofSamplePoints( const QVector<QgsPointXY> &ring, const QList<BuildingRoof::RoofPoint> &surfaces )
  {
    QVector<QgsPointXY> points = ring;
    for ( int i = 0; i < ring.size(); ++i )
    {
      const QgsPointXY &a = ring.at( i );
      const QgsPointXY &b = ring.at( ( i + 1 ) % ring.size() );
      for ( int j = 1; j < 8; ++j )
      {
        const double t = j / 8.0;
        appendPointIfAbsent( points, QgsPointXY( a.x() + t * ( b.x() - a.x() ), a.y() + t * ( b.y() - a.y() ) ) );
      }
    }

    for ( const BuildingRoof::RoofPoint &surface : surfaces )
      appendPointIfAbsent( points, QgsPointXY( surface.point.x(), surface.point.y() ) );

    if ( surfaces.size() == 2 )
    {
      const QgsPoint &a = surfaces.at( 0 ).point;
      const QgsPoint &b = surfaces.at( 1 ).point;
      for ( int i = 1; i < 16; ++i )
      {
        const double t = i / 16.0;
        const QgsPointXY sample( a.x() + t * ( b.x() - a.x() ), a.y() + t * ( b.y() - a.y() ) );
        if ( pointInRing( ring, sample ) )
          appendPointIfAbsent( points, sample );
      }
    }

    double minX = ring.first().x();
    double maxX = ring.first().x();
    double minY = ring.first().y();
    double maxY = ring.first().y();
    for ( const QgsPointXY &point : ring )
    {
      minX = std::min( minX, point.x() );
      maxX = std::max( maxX, point.x() );
      minY = std::min( minY, point.y() );
      maxY = std::max( maxY, point.y() );
    }

    const double width = std::max( 1e-8, maxX - minX );
    const double height = std::max( 1e-8, maxY - minY );
    const int gridX = std::max( 16, std::min( 64, static_cast<int>( std::ceil( width / std::max( width, height ) * 56.0 ) ) ) );
    const int gridY = std::max( 16, std::min( 64, static_cast<int>( std::ceil( height / std::max( width, height ) * 56.0 ) ) ) );
    for ( int ix = 1; ix < gridX; ++ix )
    {
      for ( int iy = 1; iy < gridY; ++iy )
      {
        const QgsPointXY sample( minX + width * ix / gridX, minY + height * iy / gridY );
        if ( pointInRing( ring, sample ) )
          appendPointIfAbsent( points, sample );
      }
    }
    return points;
  }

  void appendCurvedRoofSurface( BuildingRoof::Mesh &mesh, const QVector<QgsPointXY> &ring, const QList<BuildingRoof::RoofPoint> &surfaces, double baseHeight,
                                const FittedBarrelProfile *fittedBarrel = nullptr )
  {
    if ( ring.size() < 3 || surfaces.isEmpty() )
      return;

    if ( surfaces.size() == 1 )
      appendDomeRoofSurface( mesh, ring, surfaces.first().point, baseHeight );
    else if ( fittedBarrel )
      appendBarrelRoofSurface( mesh, ring, fittedBarrel->anchors, fittedBarrel->normalX, fittedBarrel->normalY );
    else
      appendBarrelRoofSurface( mesh, ring, surfaces.at( 0 ).point, surfaces.at( 1 ).point, baseHeight );
  }

  void appendCurvedRoofWall( BuildingRoof::Mesh &mesh, const QVector<QgsPointXY> &ring, const QList<BuildingRoof::RoofPoint> &surfaces, double baseHeight,
                             const FittedBarrelProfile *fittedBarrel = nullptr )
  {
    if ( ring.size() < 2 )
      return;

    QVector<QgsPointXY> wallRing = ring;
    if ( fittedBarrel )
    {
      QVector<double> profileDistances;
      profileDistances.reserve( fittedBarrel->anchors.size() );
      for ( const ProfileAnchor &anchor : fittedBarrel->anchors )
        profileDistances.append( anchor.s );
      wallRing = ringWithProfileIntersections( ring, fittedBarrel->normalX, fittedBarrel->normalY, profileDistances );
    }
    const int count = wallRing.size();
    for ( int i = 0; i < count; ++i )
    {
      const QgsPointXY &a = wallRing.at( i );
      const QgsPointXY &b = wallRing.at( ( i + 1 ) % count );
      const double edgeLength = std::hypot( b.x() - a.x(), b.y() - a.y() );
      const double targetLength = std::max( ringExtentSize( ring ) / 80.0, 1e-8 );
      const int segments = std::max( 1, std::min( 32, static_cast<int>( std::ceil( edgeLength / targetLength ) ) ) );
      for ( int segment = 0; segment < segments; ++segment )
      {
        const double t0 = static_cast<double>( segment ) / segments;
        const double t1 = static_cast<double>( segment + 1 ) / segments;
        const QgsPointXY p0( a.x() + ( b.x() - a.x() ) * t0, a.y() + ( b.y() - a.y() ) * t0 );
        const QgsPointXY p1( a.x() + ( b.x() - a.x() ) * t1, a.y() + ( b.y() - a.y() ) * t1 );
        const double topA = fittedBarrel ? profileRoofZ( fittedBarrel->anchors, barrelProfileSAt( ring, p0, fittedBarrel->normalX, fittedBarrel->normalY,
                                                                                                   fittedBarrel->anchors.first().s, fittedBarrel->anchors.last().s ) )
                                         : curvedRoofZAt( p0, ring, surfaces, baseHeight );
        const double topB = fittedBarrel ? profileRoofZ( fittedBarrel->anchors, barrelProfileSAt( ring, p1, fittedBarrel->normalX, fittedBarrel->normalY,
                                                                                                   fittedBarrel->anchors.first().s, fittedBarrel->anchors.last().s ) )
                                         : curvedRoofZAt( p1, ring, surfaces, baseHeight );
        const int offset = mesh.vertices.size();
        mesh.vertices.append( QgsPoint( p0.x(), p0.y(), 0.0 ) );
        mesh.vertices.append( QgsPoint( p1.x(), p1.y(), 0.0 ) );
        mesh.vertices.append( QgsPoint( p0.x(), p0.y(), topA ) );
        mesh.vertices.append( QgsPoint( p1.x(), p1.y(), topB ) );
        mesh.indices << offset << offset + 1 << offset + 2;
        mesh.indices << offset + 2 << offset + 1 << offset + 3;
      }
    }
  }

  QgsGeometry halfPlaneGeometry( const QVector<QgsPointXY> &referenceRing, const QgsPointXY &linePoint, double normalX, double normalY, bool keepPositive )
  {
    const double lineDirX = -normalY;
    const double lineDirY = normalX;
    const double side = keepPositive ? 1.0 : -1.0;
    const double extent = ringExtentSize( referenceRing ) * 8.0 + 100.0;

    const QgsPointXY a( linePoint.x() + lineDirX * extent, linePoint.y() + lineDirY * extent );
    const QgsPointXY b( linePoint.x() - lineDirX * extent, linePoint.y() - lineDirY * extent );
    const QgsPointXY c( b.x() + side * normalX * extent, b.y() + side * normalY * extent );
    const QgsPointXY d( a.x() + side * normalX * extent, a.y() + side * normalY * extent );
    return polygonGeometryFromRing( QVector<QgsPointXY>{ a, b, c, d } );
  }

  QgsGeometry intersectWithHalfPlane( const QgsGeometry &geometry, const QVector<QgsPointXY> &referenceRing, const QgsPointXY &linePoint, double normalX, double normalY, bool keepPositive )
  {
    if ( geometry.isNull() || geometry.isEmpty() )
      return QgsGeometry();
    return geometry.intersection( halfPlaneGeometry( referenceRing, linePoint, normalX, normalY, keepPositive ) );
  }

  bool lineSegmentIntersection( const QgsPointXY &linePoint, double lineDirX, double lineDirY, const QgsPointXY &segmentStart, const QgsPointXY &segmentEnd, QgsPointXY &intersection, double &lineT, double &segmentT )
  {
    const double segmentX = segmentEnd.x() - segmentStart.x();
    const double segmentY = segmentEnd.y() - segmentStart.y();
    const double den = cross2d( lineDirX, lineDirY, segmentX, segmentY );
    if ( std::fabs( den ) <= 1e-12 )
      return false;

    const double relX = segmentStart.x() - linePoint.x();
    const double relY = segmentStart.y() - linePoint.y();
    lineT = cross2d( relX, relY, segmentX, segmentY ) / den;
    segmentT = cross2d( relX, relY, lineDirX, lineDirY ) / den;
    if ( segmentT < -1e-8 || segmentT > 1.0 + 1e-8 )
      return false;

    intersection = pointOnLine( linePoint, lineDirX, lineDirY, lineT );
    return true;
  }

  bool findTransitionChord( const QVector<QgsPointXY> &ring, int cornerIndex, const QgsPointXY &ridgeXY, double dirX, double dirY, double secondaryDirX, double secondaryDirY, const QVector<LineInterval> &primaryIntervals, QgsPointXY &bend, double &bendT, double &splitNormalX, double &splitNormalY, double &chordScore )
  {
    const QgsPointXY &corner = ring[cornerIndex];
    const int count = ring.size();
    bool found = false;
    double bestScore = std::numeric_limits<double>::max();
    const double seamX = dirX + secondaryDirX, seamY = dirY + secondaryDirY;
    const double seamLength = std::hypot( seamX, seamY );
    if ( seamLength <= 1e-8 )
      return false;

    for ( int j = 0; j < count; ++j )
    {
      if ( adjacentRingIndices( cornerIndex, j, count ) )
        continue;

      const QgsPointXY &candidate = ring[j];
      if ( !segmentInsideRing( ring, corner, candidate ) )
        continue;

      QgsPointXY candidateBend;
      double candidateBendT = 0.0;
      double chordT = 0.0;
      if ( !lineSegmentIntersection( ridgeXY, dirX, dirY, corner, candidate, candidateBend, candidateBendT, chordT ) )
        continue;
      if ( chordT <= 1e-6 || chordT >= 1.0 - 1e-6 )
        continue;

      LineInterval primaryInterval;
      if ( !findIntervalContaining( primaryIntervals, 0.0, candidateBendT, primaryInterval ) )
        continue;

      const double chordX = candidate.x() - corner.x();
      const double chordY = candidate.y() - corner.y();
      const double chordLength = std::hypot( chordX, chordY );
      if ( chordLength <= 1e-8 )
        continue;

      const double chordDirX = chordX / chordLength;
      const double chordDirY = chordY / chordLength;
      const double normalX = -chordDirY, normalY = chordDirX;
      const double primarySide = normalX * dirX + normalY * dirY;
      const double secondarySide = normalX * secondaryDirX + normalY * secondaryDirY;
      if ( primarySide * secondarySide >= -1e-10 )
        continue;
      // Use outward arm directions to distinguish the cross-roof seam from its perpendicular bisector.
      const double seamAlignment = std::fabs( ( chordDirX * seamX + chordDirY * seamY ) / seamLength );
      const double centeredScore = std::fabs( chordT - 0.5 );
      const double score = 1.0 - seamAlignment + 0.10 * centeredScore
                           + 0.05 * chordLength / std::max( ringExtentSize( ring ), 1e-8 );
      if ( score < bestScore )
      {
        bestScore = score;
        bend = candidateBend;
        bendT = candidateBendT;
        splitNormalX = -chordDirY;
        splitNormalY = chordDirX;
        chordScore = score;
        found = true;
      }
    }

    return found;
  }

  void computeBentSegmentLimits( BentGableSegment &segment, const QVector<QgsPointXY> &ring, const QgsPoint &boundaryPoint )
  {
    const double boundaryDistance = signedDistanceToLine( QgsPointXY( boundaryPoint.x(), boundaryPoint.y() ), segment.ridgePoint, segment.normalX, segment.normalY );
    double positiveLimit = 0.0, negativeLimit = 0.0;
    auto includePoint = [&]( const QgsPointXY &point ) {
      const double distance = signedDistanceToLine( point, segment.ridgePoint, segment.normalX, segment.normalY );
      positiveLimit = std::max( positiveLimit, distance );
      negativeLimit = std::max( negativeLimit, -distance );
    };
    // Oblique end caps may have both useful eave intersections between existing vertices.
    for ( int edge = 0; edge < ring.size(); ++edge )
    {
      const QgsPointXY &a = ring[edge], &b = ring[( edge + 1 ) % ring.size()];
      const double alongA = lineParameter( segment.start, segment.dirX, segment.dirY, a );
      const double alongB = lineParameter( segment.start, segment.dirX, segment.dirY, b );
      const double delta = alongB - alongA;
      double first = 0.0, last = 1.0;
      if ( std::fabs( delta ) <= 1e-10 )
      {
        if ( alongA < -1e-7 || alongA > segment.length + 1e-7 )
          continue;
      }
      else
      {
        const double startT = -alongA / delta;
        const double endT = ( segment.length - alongA ) / delta;
        first = std::max( 0.0, std::min( startT, endT ) );
        last = std::min( 1.0, std::max( startT, endT ) );
        if ( first > last )
          continue;
      }
      includePoint( QgsPointXY( a.x() + ( b.x() - a.x() ) * first, a.y() + ( b.y() - a.y() ) * first ) );
      includePoint( QgsPointXY( a.x() + ( b.x() - a.x() ) * last, a.y() + ( b.y() - a.y() ) * last ) );
    }

    segment.sameSideLimit = boundaryDistance >= 0.0 ? positiveLimit : negativeLimit;
    segment.oppositeLimit = boundaryDistance >= 0.0 ? negativeLimit : positiveLimit;
    if ( segment.sameSideLimit <= 1e-8 )
      segment.sameSideLimit = std::max( std::fabs( boundaryDistance ), segment.oppositeLimit );
    if ( segment.oppositeLimit <= 1e-8 )
      segment.oppositeLimit = segment.sameSideLimit;
  }

  bool makeBentSegment( const QgsPointXY &start, const QgsPointXY &end, double ridgeHeight, const QVector<QgsPointXY> &ring, const QgsPoint &boundaryPoint, BentGableSegment &segment )
  {
    const double dx = end.x() - start.x();
    const double dy = end.y() - start.y();
    const double length = std::hypot( dx, dy );
    if ( length <= 1e-8 )
      return false;

    segment.start = start;
    segment.end = end;
    segment.length = length;
    segment.dirX = dx / length;
    segment.dirY = dy / length;
    segment.normalX = -segment.dirY;
    segment.normalY = segment.dirX;
    segment.ridgePoint = QgsPoint( start.x(), start.y(), ridgeHeight );
    computeBentSegmentLimits( segment, ring, boundaryPoint );
    return segment.sameSideLimit > 1e-8 || segment.oppositeLimit > 1e-8;
  }

  bool bentSecondaryDirection( const QVector<QgsPointXY> &ring, int cornerIndex, double primaryDirX, double primaryDirY, double &secondaryDirX, double &secondaryDirY )
  {
    const QgsPointXY &previous = ring[( cornerIndex - 1 + ring.size() ) % ring.size()];
    const QgsPointXY &corner = ring[cornerIndex];
    const QgsPointXY &next = ring[( cornerIndex + 1 ) % ring.size()];

    const double candidates[2][2] = {
      { previous.x() - corner.x(), previous.y() - corner.y() },
      { next.x() - corner.x(), next.y() - corner.y() }
    };

    double bestScore = std::numeric_limits<double>::max();
    bool found = false;
    for ( const auto &candidate : candidates )
    {
      const double length = std::hypot( candidate[0], candidate[1] );
      if ( length <= 1e-8 )
        continue;

      const double candidateX = candidate[0] / length;
      const double candidateY = candidate[1] / length;
      const double score = std::fabs( candidateX * primaryDirX + candidateY * primaryDirY );
      if ( score < bestScore )
      {
        bestScore = score;
        secondaryDirX = candidateX;
        secondaryDirY = candidateY;
        found = true;
      }
    }
    return found && std::fabs( cross2d( secondaryDirX, secondaryDirY, primaryDirX, primaryDirY ) ) > 1e-6;
  }

  BentGableLayout findBentGabledLayout( const QVector<QgsPointXY> &inputRing, const QgsPointXY &ridgeXY, double dirX, double dirY )
  {
    BentGableLayout layout;
    if ( inputRing.size() < 5 )
      return layout;
    const double extent = ringExtentSize( inputRing );
    double bestScore = std::numeric_limits<double>::max();
    for ( int i = 0; i < inputRing.size(); ++i )
    {
      if ( !isConcaveVertex( inputRing, i ) )
        continue;
      const QgsPointXY &corner = inputRing[i];
      const QgsPointXY &previous = inputRing[( i - 1 + inputRing.size() ) % inputRing.size()];
      const QgsPointXY &next = inputRing[( i + 1 ) % inputRing.size()];
      if ( std::hypot( previous.x() - corner.x(), previous.y() - corner.y() ) < extent * 0.01
           || std::hypot( next.x() - corner.x(), next.y() - corner.y() ) < extent * 0.01 )
        continue;

      double secondaryDirX = 0.0, secondaryDirY = 0.0;
      if ( !bentSecondaryDirection( inputRing, i, dirX, dirY, secondaryDirX, secondaryDirY ) )
        continue;
      double primarySign = 0.0, primaryAlignment = -1.0;
      for ( const QgsPointXY &adjacent : { previous, next } )
      {
        const double dx = adjacent.x() - corner.x(), dy = adjacent.y() - corner.y();
        const double projection = ( dx * dirX + dy * dirY ) / std::hypot( dx, dy );
        if ( std::fabs( projection ) > primaryAlignment )
        {
          primaryAlignment = std::fabs( projection );
          primarySign = projection;
        }
      }
      const double outwardX = primarySign >= 0.0 ? dirX : -dirX;
      const double outwardY = primarySign >= 0.0 ? dirY : -dirY;
      const QVector<LineInterval> primaryIntervals = lineInsideRingIntervals( inputRing, ridgeXY, outwardX, outwardY );
      if ( primaryIntervals.isEmpty() )
        continue;
      QgsPointXY bend;
      double bendT = 0.0, splitNormalX = 0.0, splitNormalY = 0.0, chordScore = 0.0;
      if ( !findTransitionChord( inputRing, i, ridgeXY, outwardX, outwardY, secondaryDirX, secondaryDirY,
                                 primaryIntervals, bend, bendT, splitNormalX, splitNormalY, chordScore ) )
        continue;
      LineInterval primaryInterval, secondaryInterval;
      if ( !findIntervalContaining( primaryIntervals, 0.0, bendT, primaryInterval )
           || !findIntervalContaining( lineInsideRingIntervals( inputRing, bend, secondaryDirX, secondaryDirY ), 0.0, 0.0, secondaryInterval ) )
        continue;
      const QgsPointXY primaryEnd = pointOnLine( ridgeXY, outwardX, outwardY, primaryInterval.end );
      const QgsPointXY secondaryEnd = pointOnLine( bend, secondaryDirX, secondaryDirY, secondaryInterval.end );
      if ( primaryInterval.end - bendT <= extent * 0.01 || secondaryInterval.end <= extent * 0.01
           || nearlySamePoint( primaryEnd, bend )
           || !segmentInsideRing( inputRing, bend, primaryEnd ) || !segmentInsideRing( inputRing, bend, secondaryEnd ) )
        continue;

      const double score = std::fabs( bendT ) / std::max( extent, 1e-8 ) + chordScore;
      if ( score < bestScore )
      {
        bestScore = score;
        layout.primaryEnd = primaryEnd;
        layout.bend = bend;
        layout.secondaryEnd = secondaryEnd;
        layout.splitNormalX = splitNormalX;
        layout.splitNormalY = splitNormalY;
        layout.success = true;
      }
    }
    return layout;
  }

  BentGableLayout bentLayoutFromRidgePoints( const QList<BuildingRoof::RoofPoint> &ridges )
  {
    BentGableLayout layout;
    if ( ridges.size() != 3 )
      return layout;
    layout.primaryEnd = QgsPointXY( ridges[0].point.x(), ridges[0].point.y() );
    layout.bend = QgsPointXY( ridges[1].point.x(), ridges[1].point.y() );
    layout.secondaryEnd = QgsPointXY( ridges[2].point.x(), ridges[2].point.y() );
    const double primaryLength = std::hypot( layout.primaryEnd.x() - layout.bend.x(), layout.primaryEnd.y() - layout.bend.y() );
    const double secondaryLength = std::hypot( layout.secondaryEnd.x() - layout.bend.x(), layout.secondaryEnd.y() - layout.bend.y() );
    if ( !std::isfinite( primaryLength ) || !std::isfinite( secondaryLength ) || primaryLength <= 1e-8 || secondaryLength <= 1e-8 )
      return layout;
    const double ux = ( layout.primaryEnd.x() - layout.bend.x() ) / primaryLength;
    const double uy = ( layout.primaryEnd.y() - layout.bend.y() ) / primaryLength;
    const double vx = ( layout.secondaryEnd.x() - layout.bend.x() ) / secondaryLength;
    const double vy = ( layout.secondaryEnd.y() - layout.bend.y() ) / secondaryLength;
    if ( std::fabs( cross2d( ux, uy, vx, vy ) ) <= 1e-6 )
      return layout;
    const double normalLength = std::hypot( ux - vx, uy - vy );
    layout.splitNormalX = ( ux - vx ) / normalLength;
    layout.splitNormalY = ( uy - vy ) / normalLength;
    layout.success = true;
    return layout;
  }

  BuildingRoof::MeshResult buildBentGabledRoofPrismMesh( const QVector<QgsPointXY> &inputRing, const QgsPoint &boundaryPoint,
                                                          const BentGableLayout &layout, double ridgeHeight )
  {
    BuildingRoof::MeshResult result;
    if ( inputRing.size() < 5 || !layout.success || !pointInRing( inputRing, layout.bend )
         || ridgeHeight <= boundaryPoint.z() + 1e-8 )
      return result;
    const QgsGeometry footprint = polygonGeometryFromRing( inputRing );
    const double footprintArea = footprint.area();
    if ( footprintArea <= 1e-8 )
      return result;

    const QgsGeometry primaryFootprint = intersectWithHalfPlane( footprint, inputRing, layout.bend, layout.splitNormalX, layout.splitNormalY, true );
    const QgsGeometry secondaryFootprint = intersectWithHalfPlane( footprint, inputRing, layout.bend, layout.splitNormalX, layout.splitNormalY, false );
    auto armRing = []( const QgsGeometry &geometry ) {
      QVector<QgsPointXY> points;
      for ( const QVector<QgsPointXY> &part : exteriorRingsFromGeometry( geometry ) )
        points += part;
      return points;
    };
    BentGableSegment primary, secondary;
    if ( !makeBentSegment( layout.primaryEnd, layout.bend, ridgeHeight, armRing( primaryFootprint ), boundaryPoint, primary )
         || !makeBentSegment( layout.bend, layout.secondaryEnd, ridgeHeight, armRing( secondaryFootprint ), boundaryPoint, secondary ) )
      return result;

    const double ux = -primary.dirX, uy = -primary.dirY;
    const double vx = secondary.dirX, vy = secondary.dirY;
    const double determinant = cross2d( ux, uy, vx, vy );
    if ( std::fabs( determinant ) <= 1e-6 )
      return result;

    struct RoofPlane
    {
      double a = 0.0, b = 0.0, c = 1.0;
    };
    struct RoofPatch
    {
      QgsGeometry geometry;
      RoofPlane plane;
    };
    auto fractionAt = [&]( const RoofPlane &plane, const QgsPointXY &point ) {
      return plane.c + plane.a * ( point.x() - layout.bend.x() ) + plane.b * ( point.y() - layout.bend.y() );
    };
    auto sidePlane = [&]( const BentGableSegment &segment, bool positive ) {
      const double boundaryDistance = signedDistanceToLine( QgsPointXY( boundaryPoint.x(), boundaryPoint.y() ),
                                                            segment.ridgePoint, segment.normalX, segment.normalY );
      const bool sameSide = positive == ( boundaryDistance >= 0.0 );
      const double width = sameSide ? segment.sameSideLimit : segment.oppositeLimit;
      const double sign = positive ? 1.0 : -1.0;
      RoofPlane plane;
      plane.a = -sign * segment.normalX / std::max( width, 1e-8 );
      plane.b = -sign * segment.normalY / std::max( width, 1e-8 );
      return plane;
    };
    const bool primaryInnerPositive = primary.normalX * vx + primary.normalY * vy > 0.0;
    const bool secondaryInnerPositive = secondary.normalX * ux + secondary.normalY * uy > 0.0;
    const RoofPlane primaryInner = sidePlane( primary, primaryInnerPositive );
    const RoofPlane primaryOuter = sidePlane( primary, !primaryInnerPositive );
    const RoofPlane secondaryInner = sidePlane( secondary, secondaryInnerPositive );
    const RoofPlane secondaryOuter = sidePlane( secondary, !secondaryInnerPositive );

    auto clip = [&]( const QgsGeometry &geometry, const QgsPointXY &origin, double nx, double ny, bool positive ) {
      const double length = std::hypot( nx, ny );
      if ( geometry.isNull() || geometry.isEmpty() || length <= 1e-12 )
        return QgsGeometry();
      return intersectWithHalfPlane( geometry, inputRing, origin, nx / length, ny / length, positive );
    };
    const double alphaX = vy / determinant, alphaY = -vx / determinant;
    const double betaX = -uy / determinant, betaY = ux / determinant;
    auto quadrant = [&]( bool alphaPositive, bool betaPositive ) {
      return clip( clip( footprint, layout.bend, alphaX, alphaY, alphaPositive ),
                   layout.bend, betaX, betaY, betaPositive );
    };

    QVector<RoofPatch> patches;
    auto appendPatch = [&]( const QgsGeometry &geometry, const RoofPlane &plane ) {
      if ( geometry.isNull() || geometry.isEmpty() )
        return;
      const double length2 = plane.a * plane.a + plane.b * plane.b;
      if ( length2 <= 1e-16 )
      {
        patches.append( RoofPatch{ geometry, plane } );
        return;
      }
      const QgsPointXY eaveOrigin( layout.bend.x() - plane.a * plane.c / length2,
                                   layout.bend.y() - plane.b * plane.c / length2 );
      const QgsGeometry aboveEave = clip( geometry, eaveOrigin, plane.a, plane.b, true );
      const QgsGeometry belowEave = clip( geometry, eaveOrigin, plane.a, plane.b, false );
      if ( !aboveEave.isEmpty() && aboveEave.area() > 1e-10 )
        patches.append( RoofPatch{ aboveEave, plane } );
      if ( !belowEave.isEmpty() && belowEave.area() > 1e-10 )
        patches.append( RoofPatch{ belowEave, RoofPlane{ 0.0, 0.0, 0.0 } } );
    };
    auto appendJoinedPlanes = [&]( const QgsGeometry &geometry, const RoofPlane &first, const RoofPlane &second, bool higher ) {
      const double nx = first.a - second.a, ny = first.b - second.b;
      if ( std::hypot( nx, ny ) <= 1e-12 )
      {
        appendPatch( geometry, first );
        return;
      }
      appendPatch( clip( geometry, layout.bend, nx, ny, higher ), first );
      appendPatch( clip( geometry, layout.bend, nx, ny, !higher ), second );
    };

    // The inner corner is a valley; the opposite corner is a hip. Split at equal-height lines before triangulating.
    appendPatch( quadrant( true, false ), primaryOuter );
    appendPatch( quadrant( false, true ), secondaryOuter );
    appendJoinedPlanes( quadrant( true, true ), primaryInner, secondaryInner, true );
    appendJoinedPlanes( quadrant( false, false ), primaryOuter, secondaryOuter, false );

    QVector<QgsPointXY> junctions{ layout.primaryEnd, layout.bend, layout.secondaryEnd };
    double coveredArea = 0.0;
    for ( const RoofPatch &patch : patches )
    {
      coveredArea += patch.geometry.area();
      for ( const QVector<QgsPointXY> &ring : exteriorRingsFromGeometry( patch.geometry ) )
        for ( const QgsPointXY &point : ring )
          appendPointIfAbsent( junctions, point );
    }
    const double areaTolerance = std::max( 1e-6, footprintArea * 1e-6 );
    if ( std::fabs( coveredArea - footprintArea ) > areaTolerance )
    {
      result.error = QStringLiteral( "The bent roof patches do not cover the building footprint." );
      return result;
    }

    const double baseHeight = boundaryPoint.z(), rise = ridgeHeight - baseHeight;
    double triangulatedArea = 0.0;
    for ( const RoofPatch &patch : patches )
    {
      for ( const QVector<QgsPointXY> &part : exteriorRingsFromGeometry( patch.geometry ) )
      {
        const QVector<QgsPointXY> ring = ringWithInsertedBoundaryPoints( part, junctions );
        const int offset = result.mesh.vertices.size();
        for ( const QgsPointXY &point : ring )
          result.mesh.vertices.append( QgsPoint( point.x(), point.y(), baseHeight + rise * std::max( 0.0, std::min( 1.0, fractionAt( patch.plane, point ) ) ) ) );
        const QVector<int> triangles = triangulateRing( ring );
        for ( int i = 0; i + 2 < triangles.size(); i += 3 )
        {
          const QgsPointXY &a = ring[triangles[i]], &b = ring[triangles[i + 1]], &c = ring[triangles[i + 2]];
          triangulatedArea += std::fabs( cross2d( b.x() - a.x(), b.y() - a.y(), c.x() - a.x(), c.y() - a.y() ) ) * 0.5;
          result.mesh.indices << offset + triangles[i] << offset + triangles[i + 1] << offset + triangles[i + 2];
        }
      }
    }
    if ( std::fabs( triangulatedArea - footprintArea ) > areaTolerance )
    {
      result.error = QStringLiteral( "The bent roof surfaces cannot be triangulated completely." );
      return result;
    }

    const QVector<QgsPointXY> wallRing = ringWithInsertedBoundaryPoints( inputRing, junctions );
    const int wallOffset = result.mesh.vertices.size();
    const double boundaryTolerance = std::max( 1e-7, ringExtentSize( inputRing ) * 1e-8 );
    for ( const QgsPointXY &point : wallRing )
    {
      const QgsGeometry query = QgsGeometry::fromPointXY( point );
      double top = -std::numeric_limits<double>::max();
      for ( const RoofPatch &patch : patches )
      {
        const double distance = patch.geometry.distance( query );
        if ( distance >= 0.0 && distance <= boundaryTolerance )
          top = std::max( top, baseHeight + rise * std::max( 0.0, std::min( 1.0, fractionAt( patch.plane, point ) ) ) );
      }
      if ( top < baseHeight - 1e-8 || !std::isfinite( top ) )
      {
        result.error = QStringLiteral( "The bent roof cannot connect to the outer walls." );
        return result;
      }
      result.mesh.vertices.append( QgsPoint( point.x(), point.y(), 0.0 ) );
      result.mesh.vertices.append( QgsPoint( point.x(), point.y(), top ) );
    }
    for ( int i = 0; i < wallRing.size(); ++i )
    {
      const int next = ( i + 1 ) % wallRing.size();
      result.mesh.indices << wallOffset + 2 * i << wallOffset + 2 * next << wallOffset + 2 * i + 1;
      result.mesh.indices << wallOffset + 2 * i + 1 << wallOffset + 2 * next << wallOffset + 2 * next + 1;
    }
    const QVector<int> bottom = triangulateRing( wallRing );
    for ( int i = 0; i + 2 < bottom.size(); i += 3 )
      result.mesh.indices << wallOffset + 2 * bottom[i] << wallOffset + 2 * bottom[i + 2] << wallOffset + 2 * bottom[i + 1];

    const QgsPoint bend( layout.bend.x(), layout.bend.y(), ridgeHeight );
    result.mesh.structureLines.append( qMakePair( QgsPoint( layout.primaryEnd.x(), layout.primaryEnd.y(), ridgeHeight ), bend ) );
    result.mesh.structureLines.append( qMakePair( bend, QgsPoint( layout.secondaryEnd.x(), layout.secondaryEnd.y(), ridgeHeight ) ) );
    result.success = !result.mesh.isEmpty();
    return result;
  }

  bool fitAutomaticBentRoofHeights( const QVector<QgsPointXY> &ring, const QVector<BuildingRoof::RoofSample> &samples,
                                     const AutoHeightBand &band, QList<BuildingRoof::RoofPoint> &points, QString *errorOut = nullptr )
  {
    auto fail = [&]( const QString &message ) {
      if ( errorOut )
        *errorOut = message;
      return false;
    };
    if ( !band.success || points.size() != 4 )
      return fail( QStringLiteral( "Cannot determine a supported roof height band and two ridge segments." ) );
    QgsPoint &boundary = points[0].point;
    const BentGableLayout layout = bentLayoutFromRidgePoints( ridgePoints( points ) );
    if ( !layout.success )
      return fail( QStringLiteral( "The two fitted ridge segments do not form a valid bend." ) );
    const QgsPoint boundaryBasis( boundary.x(), boundary.y(), 1.0 );
    const BuildingRoof::MeshResult basisMesh = buildBentGabledRoofPrismMesh( ring, boundaryBasis, layout, 2.0 );
    if ( !basisMesh.success )
      return fail( basisMesh.error.isEmpty() ? QStringLiteral( "Cannot construct the bent roof surface layout." ) : basisMesh.error );
    const QVector<RoofHeightTriangle> triangles = roofHeightTriangles( basisMesh.mesh );
    struct Observation
    {
      double fraction = 0.0, height = 0.0;
      int arm = 0;
    };
    QVector<Observation> observations;
    int armCounts[2] = { 0, 0 };
    const double roofFloor = band.baseHeight + std::max( 0.05, ( band.ridgeHeight - band.baseHeight ) * 0.04 );
    for ( const BuildingRoof::RoofSample &sample : samples )
    {
      const QgsPoint &point = sample.point;
      if ( point.z() < roofFloor || point.z() > band.highHeight || !pointInRing( ring, QgsPointXY( point.x(), point.y() ) ) )
        continue;
      const double fraction = roofHeightAt( triangles, point ) - 1.0;
      if ( fraction >= 0.0 && fraction <= 1.001 )
      {
        const int arm = ( point.x() - layout.bend.x() ) * layout.splitNormalX
                        + ( point.y() - layout.bend.y() ) * layout.splitNormalY >= 0.0 ? 0 : 1;
        observations.append( Observation{ fraction, point.z(), arm } );
        ++armCounts[arm];
      }
    }
    if ( observations.size() < 24 || armCounts[0] < 8 || armCounts[1] < 8 )
      return fail( QStringLiteral( "Both roof arms need supported point-cloud samples for height fitting." ) );
    double eave = band.baseHeight;
    double rise = band.ridgeHeight - band.baseHeight;
    const double scale = std::max( 0.20, rise * 0.10 );
    for ( int iteration = 0; iteration < 6; ++iteration )
    {
      double sumW = 0.0;
      double sumF = 0.0;
      double sumF2 = 0.0;
      double sumZ = 0.0;
      double sumFZ = 0.0;
      for ( const auto &observation : observations )
      {
        const double f = observation.fraction;
        const double z = observation.height;
        const double residual = z - eave - rise * f;
        const double weight = iteration == 0 ? 1.0 : std::min( 1.0, scale / std::max( 1e-8, std::fabs( residual ) ) );
        sumW += weight;
        sumF += weight * f;
        sumF2 += weight * f * f;
        sumZ += weight * z;
        sumFZ += weight * f * z;
      }
      const double determinant = sumW * sumF2 - sumF * sumF;
      if ( determinant <= 1e-8 )
        return fail( QStringLiteral( "The bent roof height samples do not provide enough profile variation." ) );
      eave = ( sumZ * sumF2 - sumFZ * sumF ) / determinant;
      rise = ( sumW * sumFZ - sumF * sumZ ) / determinant;
    }
    if ( !std::isfinite( eave ) || !std::isfinite( rise ) || eave <= 0.0 || rise <= 0.20
         || eave + rise > band.highHeight + std::max( 0.50, band.binWidth * 2.0 ) )
      return fail( QStringLiteral( "The fitted bent roof eave and ridge heights are inconsistent with the point cloud." ) );
    int supported[2] = { 0, 0 };
    const double tolerance = std::max( 0.25, rise * 0.15 );
    for ( const Observation &observation : observations )
      if ( std::fabs( observation.height - eave - rise * observation.fraction ) <= tolerance )
        ++supported[observation.arm];
    for ( int arm = 0; arm < 2; ++arm )
      if ( supported[arm] < std::max( 4, static_cast<int>( std::ceil( armCounts[arm] * 0.25 ) ) ) )
        return fail( QStringLiteral( "The fitted bent roof profile is not supported by both roof arms." ) );
    boundary.setZ( eave );
    for ( int i = 1; i < points.size(); ++i )
      points[i].point.setZ( eave + rise );
    return true;
  }

  double eaveHeightAt( const QVector<AnchorPoint> &eaveAnchors, const QgsPointXY &point, double fallbackZ )
  {
    return nearestAnchorZ( eaveAnchors, point, fallbackZ );
  }

  BuildingRoof::MeshResult buildAnchoredRoofPrismMesh( const QgsGeometry &buildingGeometry, const QVector<AnchorPoint> &eaveAnchors, const QVector<AnchorPoint> &roofAnchors, const QString &errorPrefix )
  {
    BuildingRoof::MeshResult result;
    const QgsPolygonXY polygon = firstPolygon( buildingGeometry );
    const QVector<QgsPointXY> ring = exteriorRing( polygon );
    if ( ring.size() < 3 )
    {
      result.error = errorPrefix + QStringLiteral( " footprint is invalid." );
      return result;
    }
    if ( eaveAnchors.isEmpty() || roofAnchors.isEmpty() )
    {
      result.error = errorPrefix + QStringLiteral( " requires boundary and ridge anchors." );
      return result;
    }

    const double fallbackEaveZ = eaveAnchors.first().z;
    for ( const QgsPointXY &point : ring )
    {
      result.mesh.vertices.append( QgsPoint( point.x(), point.y(), 0.0 ) );
      result.mesh.vertices.append( QgsPoint( point.x(), point.y(), eaveHeightAt( eaveAnchors, point, fallbackEaveZ ) ) );
    }

    const int count = ring.size();
    for ( int i = 0; i < count; ++i )
    {
      const int next = ( i + 1 ) % count;
      result.mesh.indices << 2 * i << 2 * next << 2 * i + 1;
      result.mesh.indices << 2 * i + 1 << 2 * next << 2 * next + 1;
    }

    const QVector<int> bottomTriangles = triangulateRing( ring );
    for ( int i = 0; i + 2 < bottomTriangles.size(); i += 3 )
      result.mesh.indices << 2 * bottomTriangles[i] << 2 * bottomTriangles[i + 2] << 2 * bottomTriangles[i + 1];

    QVector<AnchorPoint> allAnchors = roofAnchors;
    QgsMultiPointXY pointSet;
    for ( const QgsPointXY &point : ring )
    {
      const double z = eaveHeightAt( eaveAnchors, point, fallbackEaveZ );
      appendAnchor( allAnchors, point, z );
      pointSet.append( point );
    }
    for ( const AnchorPoint &anchor : roofAnchors )
      pointSet.append( anchor.point );

    QgsGeometry tin = QgsGeometry::fromMultiPointXY( pointSet ).delaunayTriangulation( 0.0, false );
    QVector<QgsGeometry> triangles = tin.asGeometryCollection();
    if ( triangles.isEmpty() && !tin.isNull() )
      triangles.append( tin );

    for ( const QgsGeometry &triangleGeometry : triangles )
    {
      const QgsPolygonXY triangle = triangleGeometry.asPolygon();
      if ( triangle.isEmpty() || triangle.first().size() < 4 )
        continue;

      const QgsPolylineXY triangleRing = triangle.first();
      const QgsPointXY a = triangleRing.at( 0 );
      const QgsPointXY b = triangleRing.at( 1 );
      const QgsPointXY c = triangleRing.at( 2 );
      const QgsPointXY centroid( ( a.x() + b.x() + c.x() ) / 3.0, ( a.y() + b.y() + c.y() ) / 3.0 );
      if ( !pointInRing( ring, centroid ) )
        continue;

      const int offset = result.mesh.vertices.size();
      result.mesh.vertices.append( QgsPoint( a.x(), a.y(), nearestAnchorZ( allAnchors, a, fallbackEaveZ ) ) );
      result.mesh.vertices.append( QgsPoint( b.x(), b.y(), nearestAnchorZ( allAnchors, b, fallbackEaveZ ) ) );
      result.mesh.vertices.append( QgsPoint( c.x(), c.y(), nearestAnchorZ( allAnchors, c, fallbackEaveZ ) ) );
      result.mesh.indices << offset << offset + 1 << offset + 2;
    }

    result.success = !result.mesh.isEmpty();
    if ( !result.success )
      result.error = errorPrefix + QStringLiteral( " mesh generation failed." );
    return result;
  }

  double profileRoofZ( const QVector<ProfileAnchor> &anchors, double s )
  {
    if ( anchors.isEmpty() )
      return 0.0;

    if ( s <= anchors.first().s )
      return anchors.first().z;
    if ( s >= anchors.last().s )
      return anchors.last().z;

    for ( int i = 0; i + 1 < anchors.size(); ++i )
    {
      const ProfileAnchor &a = anchors.at( i );
      const ProfileAnchor &b = anchors.at( i + 1 );
      if ( s < a.s || s > b.s )
        continue;

      const double span = b.s - a.s;
      if ( std::fabs( span ) <= 1e-12 )
        return std::max( a.z, b.z );
      const double t = ( s - a.s ) / span;
      return a.z + t * ( b.z - a.z );
    }

    return anchors.last().z;
  }

  void appendProfileAnchor( QVector<ProfileAnchor> &anchors, double s, double z )
  {
    for ( ProfileAnchor &anchor : anchors )
    {
      if ( std::fabs( anchor.s - s ) <= 1e-7 )
      {
        anchor.z = std::max( anchor.z, z );
        return;
      }
    }
    anchors.append( ProfileAnchor{ s, z } );
  }

  double separatedHeightTolerance( double heightDifference, double preferredTolerance )
  {
    if ( heightDifference <= 1e-8 )
      return preferredTolerance;

    const double maxSeparatedTolerance = std::max( heightDifference * 0.45, 0.01 );
    return std::min( preferredTolerance, maxSeparatedTolerance );
  }

  void averageRidgePairHeightIfClose( BuildingRoof::RoofPoint &first, BuildingRoof::RoofPoint &second )
  {
    if ( std::fabs( first.point.z() - second.point.z() ) > BuildingRoof::RIDGE_HEIGHT_AVERAGE_THRESHOLD )
      return;

    const double averageZ = 0.5 * ( first.point.z() + second.point.z() );
    first.point.setZ( averageZ );
    second.point.setZ( averageZ );
  }

}

BuildingRoof::RoofPlaneSegmentation BuildingRoof::segmentRoofCloud( const QgsGeometry &buildingGeometry, const QVector<RoofSample> &pointCloudSamples,
                                                                    const RoofSegmentationOptions &options )
{
  RoofPlaneSegmentation result;
  const QVector<QgsPointXY> ring = exteriorRing( firstPolygon( buildingGeometry ) );
  if ( ring.size() < 3 || pointCloudSamples.size() < 24 )
    return result;

  QVector<QgsPoint> insidePoints;
  QVector<double> insideHeights;
  insidePoints.reserve( pointCloudSamples.size() );
  insideHeights.reserve( pointCloudSamples.size() );
  for ( const RoofSample &sample : pointCloudSamples )
  {
    const QgsPoint &point = sample.point;
    if ( !std::isfinite( point.x() ) || !std::isfinite( point.y() ) || !std::isfinite( point.z() )
         || point.z() < 0.0 || !pointInRing( ring, QgsPointXY( point.x(), point.y() ) ) )
      continue;
    insidePoints.append( point );
    insideHeights.append( point.z() );
  }
  if ( insidePoints.size() < 24 )
    return result;

  QVector<QgsPoint> roofPoints;
  if ( options.useRoofHeightBand )
  {
    std::sort( insideHeights.begin(), insideHeights.end() );
    double roofFloor = sortedPercentile( insideHeights, 0.45 );
    double roofCeiling = sortedPercentile( insideHeights, 0.995 );
    const AutoHeightBand heightBand = inferAutoGabledHeightBand( ring, pointCloudSamples );
    if ( heightBand.success )
    {
      const double rise = std::max( 0.0, heightBand.ridgeHeight - heightBand.baseHeight );
      roofFloor = std::max( heightBand.lowHeight, heightBand.baseHeight + std::max( 0.08, rise * 0.04 ) );
      roofCeiling = heightBand.highHeight;
    }

    roofPoints.reserve( insidePoints.size() );
    for ( const QgsPoint &point : insidePoints )
    {
      if ( point.z() >= roofFloor && point.z() <= roofCeiling )
        roofPoints.append( point );
      else
        result.unclassifiedPoints.append( point );
    }
  }
  else
  {
    roofPoints = insidePoints;
  }
  if ( roofPoints.size() < 24 )
  {
    result.unclassifiedPoints = insidePoints;
    return result;
  }

  const double footprintArea = std::max( 1e-8, std::fabs( buildingGeometry.area() ) );
  const double extent = std::max( 1e-8, ringExtentSize( ring ) );
  const double estimatedSpacing = std::sqrt( footprintArea / std::max( 1, roofPoints.size() ) );
  const double cellSize = std::max( extent * 0.0125, std::min( extent * 0.04, estimatedSpacing * 0.90 ) );
  const QgsRectangle bounds = buildingGeometry.boundingBox();
  QVector<QgsPoint> remainingPoints = roofPoints;
  const double supportWidth = std::max( 0.20, std::min( 0.75, extent * 0.015 ) );
  const int densitySupport = static_cast<int>( std::ceil( supportWidth * supportWidth / ( estimatedSpacing * estimatedSpacing ) ) );
  const int minimumSupport = std::max( std::max( 12, options.minimumInlierCount ), densitySupport );
  for ( int segmentIndex = 0; ( options.maximumPlanes <= 0 || segmentIndex < options.maximumPlanes )
                             && remainingPoints.size() >= minimumSupport; ++segmentIndex )
  {
    const int minimumInliers = std::max( minimumSupport, static_cast<int>( std::ceil( remainingPoints.size() * options.minimumInlierFraction ) ) );
    LeastSquaresRoofPlane plane;
    SpatialRoofPlane spatialPlane;
    QVector<QgsPoint> planePoints;
    const double residualThreshold = options.planeResidualThreshold > 0.0 ? options.planeResidualThreshold : robustPlaneThreshold( remainingPoints );
    const bool extracted = options.allowVerticalPlanes
                             ? extractSpatialRansacRoofPlane( remainingPoints, minimumInliers, residualThreshold, options.horizontalOnly, spatialPlane, planePoints )
                             : extractRansacRoofPlane( remainingPoints, minimumInliers, plane, planePoints, options.horizontalOnly, options.planeResidualThreshold );
    if ( !extracted )
      break;

    const bool reliable = options.allowVerticalPlanes
                            ? spatialPlane.success && spatialPlane.rmse <= residualThreshold
                              && spatialPlane.majorSpread >= supportWidth * 0.50 && spatialPlane.minorSpread >= supportWidth * 0.10
                            : plane.success && plane.rmse <= robustPlaneThreshold( planePoints )
                              && roofPlaneHasSpatialSupport( planePoints, supportWidth );
    QVector<QVector<QgsPoint>> connectedComponents;
    if ( reliable && options.splitConnectedComponents )
      connectedComponents = projectedPointComponents( planePoints, cellSize, bounds.xMinimum(), bounds.yMinimum() );
    else if ( reliable )
      connectedComponents.append( planePoints );

    if ( connectedComponents.isEmpty() )
    {
      result.unclassifiedPoints += planePoints;
    }
    else
    {
      RoofPlaneSegment segment;
      segment.id = segmentIndex + 1;
      if ( options.allowVerticalPlanes )
      {
        segment.normalX = spatialPlane.normal[0];
        segment.normalY = spatialPlane.normal[1];
        segment.normalZ = spatialPlane.normal[2];
      }
      else
      {
        const double length = std::sqrt( 1.0 + plane.a * plane.a + plane.b * plane.b );
        segment.normalX = -plane.a / length;
        segment.normalY = -plane.b / length;
        segment.normalZ = 1.0 / length;
      }
      segment.points = connectedComponents.first();
      segment.projectedAreaRatio = std::min( 1.0, pointSetProjectedArea( segment.points, cellSize, bounds.xMinimum(), bounds.yMinimum() ) / footprintArea );
      segment.meetsAreaReference = segment.projectedAreaRatio >= options.areaReferenceRatio;
      result.segments.append( segment );

      QVector<QgsPoint> disconnectedPoints;
      for ( int componentIndex = 1; componentIndex < connectedComponents.size(); ++componentIndex )
        disconnectedPoints += connectedComponents.at( componentIndex );
      if ( !disconnectedPoints.isEmpty() )
      {
        RoofPlaneSegment disconnectedSegment;
        disconnectedSegment.id = segment.id;
        disconnectedSegment.normalX = segment.normalX;
        disconnectedSegment.normalY = segment.normalY;
        disconnectedSegment.normalZ = segment.normalZ;
        disconnectedSegment.meetsAreaReference = false;
        disconnectedSegment.points = disconnectedPoints;
        disconnectedSegment.projectedAreaRatio = std::min( 1.0, pointSetProjectedArea( disconnectedPoints, cellSize, bounds.xMinimum(), bounds.yMinimum() ) / footprintArea );
        result.segments.append( disconnectedSegment );
      }
    }

    std::multiset<std::tuple<double, double, double>> assignedPoints;
    for ( const QgsPoint &point : planePoints )
      assignedPoints.insert( std::make_tuple( point.x(), point.y(), point.z() ) );
    QVector<QgsPoint> nextPoints;
    nextPoints.reserve( remainingPoints.size() - planePoints.size() );
    for ( const QgsPoint &point : remainingPoints )
    {
      const auto assigned = assignedPoints.find( std::make_tuple( point.x(), point.y(), point.z() ) );
      if ( assigned == assignedPoints.end() )
        nextPoints.append( point );
      else
        assignedPoints.erase( assigned );
    }
    if ( nextPoints.size() >= remainingPoints.size() )
      break;
    remainingPoints = nextPoints;
  }

  result.unclassifiedPoints += remainingPoints;
  return result;
}

BuildingRoof::RoofPlaneSegmentation BuildingRoof::segmentFlatTopRoofCloud( const QgsGeometry &buildingGeometry, const QVector<RoofSample> &pointCloudSamples,
                                                                         QVector<RoofSample> *roofSamples )
{
  const QVector<QgsPointXY> ring = exteriorRing( firstPolygon( buildingGeometry ) );
  QVector<RoofSample> samples;
  samples.reserve( pointCloudSamples.size() );
  for ( const RoofSample &sample : pointCloudSamples )
  {
    const QgsPoint &point = sample.point;
    if ( std::isfinite( point.x() ) && std::isfinite( point.y() ) && std::isfinite( point.z() ) && point.z() >= 0.0
         && pointInRing( ring, QgsPointXY( point.x(), point.y() ) ) )
      samples.append( sample );
  }
  if ( roofSamples )
    *roofSamples = samples;
  RoofSegmentationOptions options;
  options.useRoofHeightBand = false;
  options.splitConnectedComponents = false;
  options.allowVerticalPlanes = true;
  options.areaReferenceRatio = 0.06;
  options.planeResidualThreshold = 0.25;
  return segmentRoofCloud( buildingGeometry, samples, options );
}

BuildingRoof::RoofPlaneSegmentation BuildingRoof::segmentRoofPlanesForDebug( const QgsGeometry &buildingGeometry, const QVector<RoofSample> &pointCloudSamples, bool preserveSmallEndPlanes )
{
  RoofSegmentationOptions options;
  options.areaReferenceRatio = preserveSmallEndPlanes ? 0.015 : 0.06;
  options.minimumInlierFraction = 0.0;
  return segmentRoofCloud( buildingGeometry, pointCloudSamples, options );
}

namespace
{
  QVector<BuildingRoof::RoofSample> roofFitSamples( const QgsGeometry &buildingGeometry,
                                                    const QVector<BuildingRoof::RoofSample> &pointCloudSamples )
  {
    if ( pointCloudSamples.size() < 24 )
      return pointCloudSamples;

    BuildingRoof::RoofSegmentationOptions options;
    options.useRoofHeightBand = false;
    options.splitConnectedComponents = false;
    const BuildingRoof::RoofPlaneSegmentation segmentation = BuildingRoof::segmentRoofCloud( buildingGeometry, pointCloudSamples, options );
    if ( segmentation.segments.isEmpty() )
      return pointCloudSamples;

    double lowHorizontalCutoff = -std::numeric_limits<double>::infinity();
    QVector<double> heights;
    heights.reserve( pointCloudSamples.size() );
    for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
    {
      if ( std::isfinite( sample.point.z() ) && sample.point.z() >= 0.0 )
        heights.append( sample.point.z() );
    }
    if ( !heights.isEmpty() )
    {
      std::sort( heights.begin(), heights.end() );
      const double span = sortedPercentile( heights, 0.90 ) - sortedPercentile( heights, 0.10 );
      lowHorizontalCutoff = sortedPercentile( heights, 0.90 ) - std::max( 0.50, span * 0.30 );
    }

    QVector<BuildingRoof::RoofSample> selected;
    selected.reserve( pointCloudSamples.size() );
    for ( const BuildingRoof::RoofPlaneSegment &segment : segmentation.segments )
    {
      if ( segment.meetsAreaReference )
      {
        const LeastSquaresRoofPlane plane = fitLeastSquaresRoofPlane( segment.points );
        if ( plane.success && std::hypot( plane.a, plane.b ) <= 0.15 )
        {
          double meanHeight = 0.0;
          for ( const QgsPoint &point : segment.points )
            meanHeight += point.z();
          meanHeight /= segment.points.size();
          if ( meanHeight < lowHorizontalCutoff )
            continue;
        }
      }
      for ( const QgsPoint &point : segment.points )
        selected.append( BuildingRoof::RoofSample{ point } );
    }
    for ( const QgsPoint &point : segmentation.unclassifiedPoints )
      selected.append( BuildingRoof::RoofSample{ point } );
    if ( selected.size() < 24 )
      return pointCloudSamples;
    return selected;
  }

  QVector<QgsPoint> automaticRoofBoundaryCandidates( const QVector<QgsPointXY> &ring, double eaveHeight, bool distinctDirections = true )
  {
    QVector<int> edges;
    for ( int edge = 0; edge < ring.size(); ++edge )
      edges.append( edge );
    auto edgeLength = [&]( int edge ) {
      const QgsPointXY &a = ring.at( edge );
      const QgsPointXY &b = ring.at( ( edge + 1 ) % ring.size() );
      return std::hypot( b.x() - a.x(), b.y() - a.y() );
    };
    std::stable_sort( edges.begin(), edges.end(), [&]( int first, int second ) {
      return edgeLength( first ) > edgeLength( second );
    } );

    QVector<QgsPoint> candidates;
    QVector<QPair<double, double>> directions;
    const double minimumLength = std::max( 1e-8, ringExtentSize( ring ) * 0.01 );
    for ( int edge : edges )
    {
      const double length = edgeLength( edge );
      if ( length < minimumLength )
        continue;
      const QgsPointXY &a = ring.at( edge );
      const QgsPointXY &b = ring.at( ( edge + 1 ) % ring.size() );
      const double dx = ( b.x() - a.x() ) / length;
      const double dy = ( b.y() - a.y() ) / length;
      bool duplicate = false;
      for ( const auto &direction : directions )
        duplicate |= std::fabs( dx * direction.first + dy * direction.second ) > 0.9986;
      if ( distinctDirections && duplicate )
        continue;
      directions.append( qMakePair( dx, dy ) );
      candidates.append( QgsPoint( ( a.x() + b.x() ) * 0.5, ( a.y() + b.y() ) * 0.5, eaveHeight ) );
    }
    return candidates;
  }

  // Compare candidate directions against the same roof-height samples, including missing coverage.
  double automaticRoofMeshScore( const BuildingRoof::Mesh &mesh, const QVector<BuildingRoof::RoofSample> &samples, const AutoHeightBand &band,
                                 bool requireSupportRatio = true )
  {
    if ( !band.success )
      return std::numeric_limits<double>::max();
    const QVector<RoofHeightTriangle> triangles = roofHeightTriangles( mesh );
    if ( triangles.isEmpty() )
      return std::numeric_limits<double>::max();

    const double rise = std::max( 0.30, band.ridgeHeight - band.baseHeight );
    const double floor = band.baseHeight + std::max( 0.05, rise * 0.04 );
    const double scale = std::max( 0.25, rise * 0.08 );
    double loss = 0.0;
    int count = 0;
    int supported = 0;
    for ( const BuildingRoof::RoofSample &sample : samples )
    {
      const QgsPoint &point = sample.point;
      if ( !std::isfinite( point.z() ) || point.z() < floor || point.z() > band.highHeight )
        continue;
      const double predicted = roofHeightAt( triangles, point );
      const double residual = std::fabs( point.z() - predicted ) / scale;
      loss += std::min( 16.0, residual * residual );
      supported += residual <= 1.0 ? 1 : 0;
      ++count;
    }
    if ( count < 12 || ( requireSupportRatio && supported < std::max( 12, static_cast<int>( std::ceil( count * 0.20 ) ) ) ) )
      return std::numeric_limits<double>::max();
    return loss / count;
  }

  BuildingRoof::MeshResult fittedFlatTopRoofMesh( const QVector<QgsPointXY> &ring, const FittedFlatTopHippedRoof &roof )
  {
    BuildingRoof::MeshResult result;
    if ( !roof.success || ring.size() != 4 || roof.topRing.size() != 4 || roof.boundaryContacts.size() != 4 )
      return result;
    double edgeX[4];
    double edgeY[4];
    double offsets[4];
    for ( int edge = 0; edge < 4; ++edge )
    {
      const QgsPointXY &a = ring.at( edge );
      const QgsPointXY &b = ring.at( ( edge + 1 ) % 4 );
      const double length = std::hypot( b.x() - a.x(), b.y() - a.y() );
      if ( length <= 1e-8 )
        return result;
      edgeX[edge] = ( b.x() - a.x() ) / length;
      edgeY[edge] = ( b.y() - a.y() ) / length;
      offsets[edge] = cross2d( edgeX[edge], edgeY[edge], roof.topRing.at( edge ).x() - a.x(), roof.topRing.at( edge ).y() - a.y() );
      if ( !roof.boundaryContacts.at( edge ) && offsets[edge] <= 1e-8 )
        return result;
    }
    auto boundaryHeight = [&]( const QgsPointXY &point ) {
      double fraction = 1.0;
      for ( int edge = 0; edge < 4; ++edge )
      {
        if ( roof.boundaryContacts.at( edge ) )
          continue;
        const double distance = cross2d( edgeX[edge], edgeY[edge], point.x() - ring.at( edge ).x(), point.y() - ring.at( edge ).y() );
        fraction = std::min( fraction, distance / offsets[edge] );
      }
      return roof.eaveHeight + ( roof.topHeight - roof.eaveHeight ) * std::max( 0.0, fraction );
    };
    auto appendWall = [&]( const QgsPoint &a, const QgsPoint &b ) {
      if ( std::hypot( b.x() - a.x(), b.y() - a.y() ) <= 1e-8 )
        return;
      const int offset = result.mesh.vertices.size();
      result.mesh.vertices << QgsPoint( a.x(), a.y(), 0.0 ) << QgsPoint( b.x(), b.y(), 0.0 ) << a << b;
      result.mesh.indices << offset << offset + 1 << offset + 2 << offset + 2 << offset + 1 << offset + 3;
      result.mesh.structureLines.append( qMakePair( a, b ) );
    };
    appendHorizontalRingSurface( result.mesh, ring, 0.0, true );
    for ( int edge = 0; edge < 4; ++edge )
    {
      const int next = ( edge + 1 ) % 4;
      const QgsPoint outerA( ring.at( edge ).x(), ring.at( edge ).y(), boundaryHeight( ring.at( edge ) ) );
      const QgsPoint outerB( ring.at( next ).x(), ring.at( next ).y(), boundaryHeight( ring.at( next ) ) );
      const QgsPoint innerA( roof.topRing.at( edge ).x(), roof.topRing.at( edge ).y(), roof.topHeight );
      const QgsPoint innerB( roof.topRing.at( next ).x(), roof.topRing.at( next ).y(), roof.topHeight );
      if ( roof.boundaryContacts.at( edge ) )
      {
        // These three wall runs follow the roof exactly; there is no zero-width slope or extra cap.
        appendWall( outerA, innerA );
        appendWall( innerA, innerB );
        appendWall( innerB, outerB );
        continue;
      }
      appendWall( outerA, outerB );
      const int offset = result.mesh.vertices.size();
      result.mesh.vertices << outerA << outerB << innerB << innerA;
      result.mesh.indices << offset << offset + 1 << offset + 2 << offset << offset + 2 << offset + 3;
      result.mesh.structureLines.append( qMakePair( innerA, innerB ) );
      result.mesh.structureLines.append( qMakePair( outerA, innerA ) );
    }
    appendHorizontalGeometrySurface( result.mesh, polygonGeometryFromRing( roof.topRing ), roof.topHeight );
    result.success = !result.mesh.isEmpty();
    return result;
  }

  BuildingRoof::MeshResult fallbackFlatTopRoofMesh( const QVector<QgsPointXY> &ring, const HorizontalRoofRegion &region,
                                                    const QVector<BuildingRoof::RoofSample> &observations, double &score )
  {
    BuildingRoof::MeshResult result;
    score = std::numeric_limits<double>::max();
    if ( ring.size() < 3 || !region.success || region.points.size() < 12 || observations.isEmpty() )
      return result;
    const QgsGeometry footprint = polygonGeometryFromRing( ring );
    const double footprintArea = footprint.area();
    if ( footprintArea <= 1e-8 )
      return result;
    const double extent = ringExtentSize( ring );
    const double radius = std::max( 0.15, region.clusterRadius );
    const QgsGeometry supportedTop = reliefGeometryFromPoints( region, footprint, extent, true );
    QgsPointXY center;
    double centerX = 0.0;
    double centerY = 0.0;
    for ( const QgsPoint &point : region.points )
    {
      centerX += point.x();
      centerY += point.y();
    }
    center = QgsPointXY( centerX / region.points.size(), centerY / region.points.size() );
    QVector<QPair<double, double>> directions;
    for ( int edge = 0; edge < ring.size(); ++edge )
    {
      const QgsPointXY &a = ring.at( edge );
      const QgsPointXY &b = ring.at( ( edge + 1 ) % ring.size() );
      const double length = std::hypot( b.x() - a.x(), b.y() - a.y() );
      if ( length <= 1e-8 )
        continue;
      const double axisX = ( b.x() - a.x() ) / length;
      const double axisY = ( b.y() - a.y() ) / length;
      bool duplicate = false;
      for ( const auto &direction : directions )
        duplicate |= std::fabs( axisX * direction.first + axisY * direction.second ) > 0.9986;
      if ( duplicate )
        continue;
      directions.append( qMakePair( axisX, axisY ) );
      const double normalX = -axisY;
      const double normalY = axisX;
      QVector<ClusterSample> topSamples;
      topSamples.reserve( region.points.size() );
      for ( const QgsPoint &point : region.points )
        topSamples.append( ClusterSample{ QgsPointXY( point.x(), point.y() ), point.x() * axisX + point.y() * axisY,
                                          point.x() * normalX + point.y() * normalY, 0, true } );
      QVector<QgsPointXY> topRing = clusterBoxNearPoint( topSamples, 1, center, axisX, axisY, normalX, normalY,
                                                       radius * 0.25, extent * 2.0 );
      if ( topRing.size() < 3 )
        continue;
      topRing = regularizeReliefRingToFootprint( topRing, ring, axisX, axisY, normalX, normalY,
                                                std::max( radius * 0.35, extent * 0.01 ) );
      // Preserve actual footprint contacts instead of forcing the old minimum inset on every side.
      const QgsGeometry topGeometry = polygonGeometryFromRing( topRing ).intersection( footprint );
      if ( topGeometry.isNull() || topGeometry.isEmpty() || !topGeometry.isGeosValid()
           || topGeometry.area() <= 1e-8 || topGeometry.area() >= footprintArea * 0.995 )
        continue;
      const QgsGeometry slopeGeometry = footprint.difference( topGeometry );
      const QVector<QVector<QgsPointXY>> topRings = exteriorRingsFromGeometry( topGeometry );
      if ( slopeGeometry.isNull() || slopeGeometry.isEmpty() || !slopeGeometry.isGeosValid() || topRings.isEmpty() )
        continue;

      QVector<BuildingRoof::RoofSample> slopeSamples;
      QVector<double> lowerHeights;
      for ( const BuildingRoof::RoofSample &sample : observations )
      {
        const QgsPoint &point = sample.point;
        if ( std::isfinite( point.z() ) && point.z() >= 0.0 && point.z() < region.height - 0.15
             && !topGeometry.contains( QgsGeometry::fromPointXY( QgsPointXY( point.x(), point.y() ) ) ) )
        {
          slopeSamples.append( sample );
          lowerHeights.append( point.z() );
        }
      }
      if ( lowerHeights.size() < 6 )
        continue;
      std::sort( lowerHeights.begin(), lowerHeights.end() );
      double baseHeight = sortedPercentile( lowerHeights, 0.10 );
      automaticBoundaryHeightFromPerimeterBand( ring, slopeSamples, QgsPoint( center.x(), center.y(), region.height ), baseHeight );
      if ( !std::isfinite( baseHeight ) || baseHeight <= 0.0 || baseHeight >= region.height - 0.15 )
        continue;

      BuildingRoof::MeshResult candidate;
      appendSlopedFlatTopHippedSurface( candidate.mesh, slopeGeometry, ring, topRings, baseHeight, region.height );
      if ( candidate.mesh.indices.isEmpty() )
        continue;
      const int slopeIndexCount = candidate.mesh.indices.size();
      appendHorizontalGeometrySurface( candidate.mesh, topGeometry, region.height );
      if ( candidate.mesh.indices.size() == slopeIndexCount )
        continue;
      appendHorizontalRingSurface( candidate.mesh, ring, 0.0, true );
      const QVector<QgsPointXY> wallRing = ringWithInsertedBoundaryPoints( ring, geometryRingPoints( topGeometry ) );
      for ( int wall = 0; wall < wallRing.size(); ++wall )
      {
        const QgsPointXY &wallA = wallRing.at( wall );
        const QgsPointXY &wallB = wallRing.at( ( wall + 1 ) % wallRing.size() );
        if ( nearlySamePoint( wallA, wallB ) )
          continue;
        const QgsPoint upperA( wallA.x(), wallA.y(), flatTopHippedRoofZ( wallA, ring, topRings, baseHeight, region.height ) );
        const QgsPoint upperB( wallB.x(), wallB.y(), flatTopHippedRoofZ( wallB, ring, topRings, baseHeight, region.height ) );
        const int offset = candidate.mesh.vertices.size();
        candidate.mesh.vertices << QgsPoint( wallA.x(), wallA.y(), 0.0 ) << QgsPoint( wallB.x(), wallB.y(), 0.0 ) << upperA << upperB;
        candidate.mesh.indices << offset << offset + 1 << offset + 2 << offset + 2 << offset + 1 << offset + 3;
        candidate.mesh.structureLines.append( qMakePair( upperA, upperB ) );
      }
      for ( const QVector<QgsPointXY> &topBoundary : topRings )
        for ( int vertex = 0; vertex < topBoundary.size(); ++vertex )
        {
          const QgsPointXY &first = topBoundary.at( vertex );
          const QgsPointXY &second = topBoundary.at( ( vertex + 1 ) % topBoundary.size() );
          candidate.mesh.structureLines.append( qMakePair( QgsPoint( first.x(), first.y(), region.height ),
                                                           QgsPoint( second.x(), second.y(), region.height ) ) );
        }

      // Rank approximate roofs without reinstating the independent four-slope acceptance gate.
      const QVector<RoofHeightTriangle> triangles = roofHeightTriangles( candidate.mesh );
      double loss = 0.0;
      for ( const BuildingRoof::RoofSample &sample : observations )
      {
        const double predicted = roofHeightAt( triangles, sample.point );
        const double residual = std::fabs( predicted - sample.point.z() ) / 0.35;
        loss += std::isfinite( residual ) ? ( residual <= 1.0 ? residual * residual : 2.0 * residual - 1.0 ) : 16.0;
      }
      const double overlap = supportedTop.isNull() ? 0.0 : supportedTop.intersection( topGeometry ).area();
      const double unionArea = supportedTop.area() + topGeometry.area() - overlap;
      const double coverage = unionArea > 1e-8 ? overlap / unionArea : 0.0;
      const double candidateScore = loss / observations.size() + 0.35 * ( 1.0 - coverage ) - 0.20 * region.areaRatio;
      if ( candidateScore < score )
      {
        score = candidateScore;
        candidate.success = true;
        result = candidate;
      }
    }
    return result;
  }
}

BuildingRoof::MeshResult BuildingRoof::buildRoofPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints, const QVector<RoofSample> &pointCloudSamples )
{
  const int boundaries = boundaryPoints( roofPoints ).size();
  const int ridges = ridgePoints( roofPoints ).size();
  const int vertices = vertexPoints( roofPoints ).size();
  const int surfaces = surfacePoints( roofPoints ).size();
  if ( boundaries == 1 && ridges == 0 && vertices == 0 && ( surfaces == 1 || surfaces == 2 ) )
    return buildCurvedRoofPrismMesh( buildingGeometry, buildingHeight, roofPoints, pointCloudSamples );
  if ( boundaries == 1 && ridges == 0 && vertices == 1 && surfaces == 0 )
    return buildApexRoofPrismMesh( buildingGeometry, buildingHeight, roofPoints, pointCloudSamples );
  if ( surfaces == 0 && vertices == 0 )
  {
    if ( boundaries == 2 && ridges == 0 )
      return buildFlatReliefPrismMesh( buildingGeometry, buildingHeight, roofPoints, pointCloudSamples );
    if ( boundaries == 2 && ridges == 1 )
      return buildClusteredFlatTopHippedRoofPrismMesh( buildingGeometry, buildingHeight, roofPoints, pointCloudSamples );
    if ( boundaries == 1 && ( ridges == 1 || ridges == 3 ) )
      return buildGabledRoofPrismMesh( buildingGeometry, buildingHeight, roofPoints, pointCloudSamples );
    if ( boundaries == 1 && ridges == 2 )
      return buildHippedRoofPrismMesh( buildingGeometry, buildingHeight, roofPoints, pointCloudSamples );
  }
  MeshResult result;
  result.error = QStringLiteral( "The marker types and counts do not identify a supported roof type." );
  return result;
}

static BuildingRoof::MeshResult buildFlatReliefCandidatePrismMesh( const QVector<QgsPointXY> &ring, const FlatReliefRegions &classified )
{
  using MeshResult = BuildingRoof::MeshResult;
  MeshResult result;
  if ( !classified.success )
  {
    result.error = QStringLiteral( "Point cloud does not contain distinct outer and inner horizontal roof regions." );
    return result;
  }
  const HorizontalRoofRegion &mainRegion = classified.outer;
  const HorizontalRoofRegion &reliefRegion = classified.inner;
  const double mainHeight = mainRegion.height;
  const double reliefHeight = reliefRegion.height;

  const bool isRaisedRelief = reliefHeight > mainHeight;
  const QgsGeometry footprintGeometry = polygonGeometryFromRing( ring );
  const QgsGeometry &initialReliefGeometry = classified.initialReliefGeometry;
  if ( initialReliefGeometry.isNull() || initialReliefGeometry.isEmpty() )
  {
    result.error = QStringLiteral( "Flat relief cluster does not intersect the footprint." );
    return result;
  }
  QgsGeometry refinedReliefGeometry = refineReliefBoundaryFromPlanes( initialReliefGeometry, footprintGeometry,
                                                                       reliefRegion.points, mainRegion.points, ringExtentSize( ring ) );
  if ( !isRaisedRelief )
  {
    refinedReliefGeometry = alignConcaveReliefToFootprint( refinedReliefGeometry, ring, ringExtentSize( ring ),
                                                           classified.boundaryConnected, reliefRegion.points );
    if ( refinedReliefGeometry.isNull() || refinedReliefGeometry.isEmpty() )
    {
      result.error = QStringLiteral( "The recessed roof boundary cannot be aligned with the building outline." );
      return result;
    }
  }
  const QgsGeometry reliefGeometry = !isRaisedRelief && classified.boundaryConnected
                                       ? refinedReliefGeometry
                                       : connectReliefToFootprintConsistently( refinedReliefGeometry, footprintGeometry,
                                                                               reliefRegion.points, ringExtentSize( ring ) );
  if ( reliefGeometry.isNull() || reliefGeometry.isEmpty() )
  {
    result.error = QStringLiteral( "The fitted relief cannot connect consistently to the building outline." );
    return result;
  }
  if ( reliefGeometry.area() >= footprintGeometry.area() * 0.98 )
  {
    result.error = QStringLiteral( "The fitted relief leaves no outer roof area inside the building footprint." );
    return result;
  }
  if ( reliefRegion.success )
  {
    int innerSupport = 0;
    for ( const QgsPoint &point : reliefRegion.points )
    {
      if ( reliefGeometry.intersects( QgsGeometry::fromPointXY( QgsPointXY( point.x(), point.y() ) ) ) )
        ++innerSupport;
    }
    if ( innerSupport < static_cast<int>( std::ceil( reliefRegion.points.size() * 0.60 ) ) )
    {
      result.error = QStringLiteral( "The fitted relief does not contain the inner roof point cloud." );
      return result;
    }
  }
  if ( !isRaisedRelief && mainRegion.success )
  {
    int outerPointsInside = 0;
    for ( const QgsPoint &point : mainRegion.points )
    {
      if ( reliefGeometry.intersects( QgsGeometry::fromPointXY( QgsPointXY( point.x(), point.y() ) ) ) )
        ++outerPointsInside;
    }
    if ( outerPointsInside > mainRegion.points.size() * 0.25 )
    {
      result.error = QStringLiteral( "The recessed area overlaps the outer roof ring." );
      return result;
    }
  }

  const QgsGeometry mainRoofGeometry = footprintGeometry.difference( reliefGeometry );
  if ( mainRoofGeometry.isNull() || mainRoofGeometry.isEmpty() )
  {
    result.error = QStringLiteral( "The fitted relief leaves no outer roof surface." );
    return result;
  }

  const double lowerZ = std::min( mainHeight, reliefHeight );
  const double upperZ = std::max( mainHeight, reliefHeight );
  const bool openConcaveRelief = !isRaisedRelief && classified.boundaryConnected;
  // The recess opens above its floor, not through the normal building walls below it.
  appendVerticalWall( result.mesh, ring, 0.0, lowerZ );
  const QgsGeometry &higherRoofGeometry = isRaisedRelief ? reliefGeometry : mainRoofGeometry;
  const QVector<QVector<QgsPointXY>> reliefRings = exteriorRingsFromGeometry( reliefGeometry );
  const QVector<QVector<QgsPointXY>> footprintRings{ ring };
  const double sharedEdgeTolerance = std::max( 1e-7, ringExtentSize( ring ) * 1e-8 );
  for ( int edge = 0; edge < ring.size(); ++edge )
  {
    if ( openConcaveRelief )
    {
      appendVerticalWallWithoutSharedEdges( result.mesh, ring.at( edge ), ring.at( ( edge + 1 ) % ring.size() ),
                                            reliefRings, lowerZ, upperZ, sharedEdgeTolerance );
      continue;
    }
    const QgsPolylineXY boundaryEdge{ ring.at( edge ), ring.at( ( edge + 1 ) % ring.size() ) };
    const QgsGeometry edgeGeometry = QgsGeometry::fromPolylineXY( boundaryEdge );
    const QgsGeometry highEdge = edgeGeometry.intersection( higherRoofGeometry );
    appendVerticalWallsAlongLines( result.mesh, highEdge, lowerZ, upperZ );
  }
  appendHorizontalRingSurface( result.mesh, ring, 0.0, true );
  const int mainRoofIndexStart = result.mesh.indices.size();
  appendClippedHorizontalGeometrySurface( result.mesh, mainRoofGeometry, mainHeight );
  if ( result.mesh.indices.size() == mainRoofIndexStart )
  {
    result.error = QStringLiteral( "The outer flat roof surface could not be triangulated." );
    return result;
  }
  const int reliefIndexStart = result.mesh.indices.size();
  appendClippedHorizontalGeometrySurface( result.mesh, reliefGeometry, reliefHeight );
  if ( result.mesh.indices.size() == reliefIndexStart )
  {
    result.error = QStringLiteral( "The relief roof surface could not be triangulated." );
    return result;
  }

  QgsPolylineXY footprintLine;
  for ( const QgsPointXY &point : ring )
    footprintLine.append( point );
  footprintLine.append( ring.first() );
  const QgsGeometry footprintBoundary = QgsGeometry::fromPolylineXY( footprintLine );
  for ( const QVector<QgsPointXY> &reliefPart : reliefRings )
  {
    if ( openConcaveRelief )
    {
      for ( int edge = 0; edge < reliefPart.size(); ++edge )
        appendVerticalWallWithoutSharedEdges( result.mesh, reliefPart.at( edge ), reliefPart.at( ( edge + 1 ) % reliefPart.size() ),
                                              footprintRings, lowerZ, upperZ, sharedEdgeTolerance );
      continue;
    }
    QgsPolylineXY reliefLine;
    for ( const QgsPointXY &point : reliefPart )
      reliefLine.append( point );
    reliefLine.append( reliefPart.first() );
    const QgsGeometry internalBoundary = QgsGeometry::fromPolylineXY( reliefLine ).difference( footprintBoundary );
    appendVerticalWallsAlongLines( result.mesh, internalBoundary, lowerZ, upperZ );
  }

  result.success = !result.mesh.isEmpty();
  if ( !result.success )
    result.error = QStringLiteral( "Flat relief roof mesh generation failed." );
  return result;
}

BuildingRoof::MeshResult BuildingRoof::buildFlatReliefPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints, const QVector<RoofSample> &pointCloudSamples )
{
  Q_UNUSED( buildingHeight )
  MeshResult result;
  if ( hasRidgePoint( roofPoints ) || boundaryPoints( roofPoints ).size() != 2 )
  {
    result.error = QStringLiteral( "Flat relief roof requires two boundary markers and no ridge marker." );
    return result;
  }
  const QVector<QgsPointXY> ring = exteriorRing( firstPolygon( buildingGeometry ) );
  if ( ring.size() < 3 )
  {
    result.error = QStringLiteral( "The building footprint cannot be used for flat relief roof mesh generation." );
    return result;
  }
  const QVector<HorizontalRoofRegion> regions = segmentedFlatReliefPlanes( ring, pointCloudSamples );
  if ( regions.size() < 2 )
  {
    result.error = QStringLiteral( "Only %1 supported horizontal roof region(s) were found; flat relief requires distinct high and low regions." ).arg( regions.size() );
    return result;
  }
  const QVector<FlatReliefRegions> candidates = classifyFlatReliefByHighSurface( ring, regions );
  if ( candidates.isEmpty() )
  {
    result.error = QStringLiteral( "%1 supported horizontal roof regions were found, but none formed a supported outer/inner pair or boundary-connected recess." ).arg( regions.size() );
    return result;
  }
  QVector<RoofSample> surfaceSamples;
  for ( const HorizontalRoofRegion &region : regions )
  {
    for ( const QgsPoint &point : region.points )
      surfaceSamples.append( RoofSample{ QgsPoint( point.x(), point.y(), region.plane.a * point.x() + region.plane.b * point.y() + region.plane.c ) } );
  }
  const QVector<RoofSample> observations = roofSurfaceObservations( ring, surfaceSamples );
  double bestScore = std::numeric_limits<double>::max();
  for ( const FlatReliefRegions &classified : candidates )
  {
    const MeshResult candidate = buildFlatReliefCandidatePrismMesh( ring, classified );
    if ( !candidate.success )
    {
      if ( !result.success && result.error.isEmpty() )
        result.error = candidate.error;
      continue;
    }
    const double loss = roofSurfaceObservationLoss( candidate.mesh, observations, 0.25 );
    if ( loss == std::numeric_limits<double>::max() )
      continue;
    const double score = loss - classified.score * 0.5;
    if ( score < bestScore )
    {
      bestScore = score;
      result = candidate;
    }
  }
  if ( !result.success && result.error.isEmpty() )
    result.error = QStringLiteral( "No horizontal-plane pair could generate a supported flat relief roof." );
  return result;
}

BuildingRoof::MeshResult BuildingRoof::buildClusteredFlatTopHippedRoofPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints, const QVector<RoofSample> &pointCloudSamples )
{
  Q_UNUSED( buildingHeight )
  MeshResult result;
  if ( boundaryPoints( roofPoints ).size() != 2 || ridgePoints( roofPoints ).size() != 1 )
  {
    result.error = QStringLiteral( "Flat-top hipped roof requires two boundary markers and one ridge marker." );
    return result;
  }

  const QVector<QgsPointXY> ring = frustumFootprintRing( exteriorRing( firstPolygon( buildingGeometry ) ) );
  if ( ring.size() < 3 )
  {
    result.error = QStringLiteral( "The building footprint cannot be used for flat-top roof generation." );
    return result;
  }
  QVector<RoofSample> roofSamples;
  const RoofPlaneSegmentation segmentation = segmentFlatTopRoofCloud( buildingGeometry, pointCloudSamples, &roofSamples );
  QVector<double> heights;
  for ( const RoofSample &sample : roofSamples )
  {
    if ( std::isfinite( sample.point.z() ) && sample.point.z() >= 0.0
         && pointInRing( ring, QgsPointXY( sample.point.x(), sample.point.y() ) ) )
      heights.append( sample.point.z() );
  }
  if ( heights.size() < 24 )
  {
    result.error = QStringLiteral( "Not enough point-cloud samples for the upper face and its sloping or boundary-connected sides." );
    return result;
  }
  std::sort( heights.begin(), heights.end() );
  const double topEstimate = sortedPercentile( heights, 0.995 );
  const QgsPointXY center = ringCentroidPoint( ring );
  double eaveEstimate = sortedPercentile( heights, 0.02 );
  automaticBoundaryHeightFromPerimeterBand( ring, roofSamples, QgsPoint( center.x(), center.y(), topEstimate ), eaveEstimate );

  const QVector<HorizontalRoofRegion> regions = segmentedHorizontalRoofRegions( ring, roofSamples, nullptr, nullptr, &segmentation, true );
  // Judge against the observed upper cloud, including roof points not recovered as a RANSAC plane.
  const QVector<RoofSample> observations = roofSurfaceObservations( ring, roofSamples );
  const QgsGeometry footprint = polygonGeometryFromRing( ring );
  if ( regions.isEmpty() )
  {
    result.error = QStringLiteral( "No supported horizontal upper-roof plane was found for the flat-top roof." );
    return result;
  }
  double bestScore = std::numeric_limits<double>::max();
  // Both fitting paths compete on the same spatial observations and top-face coverage.
  for ( const HorizontalRoofRegion &region : regions )
  {
    if ( !region.success || ring.size() != 4 )
      continue;
    const QVector<bool> boundaryContacts = flatTopBoundaryContacts( ring, region );
    const QVector<QVector<FrustumSideSample>> sides = flatTopSlopeCandidates( ring, segmentation, region, boundaryContacts );
    QVector<RoofSample> fitSamples;
    bool supportedSides = true;
    int slopeCount = 0;
    for ( int edge = 0; edge < sides.size(); ++edge )
    {
      if ( boundaryContacts.at( edge ) )
        continue;
      ++slopeCount;
      const QVector<FrustumSideSample> &side = sides.at( edge );
      supportedSides &= side.size() >= 8;
      for ( const FrustumSideSample &sample : side )
        fitSamples.append( RoofSample{ sample.point } );
    }
    if ( !supportedSides || slopeCount == 0 )
      continue;
    for ( const QgsPoint &point : region.points )
      fitSamples.append( RoofSample{ point } );
    double candidateEaveEstimate = eaveEstimate;
    if ( candidateEaveEstimate >= region.height - 0.20 )
    {
      QVector<double> lowerHeights;
      for ( double height : heights )
      {
        if ( height >= region.height - 0.20 )
          break;
        lowerHeights.append( height );
      }
      if ( lowerHeights.size() < 12 )
        continue;
      candidateEaveEstimate = sortedPercentile( lowerHeights, 0.75 );
    }
    for ( int pass = 0; pass < 2; ++pass )
    {
      const FittedFlatTopHippedRoof candidate = pass == 0
                                                ? fitFivePlaneFlatTopRoof( ring, sides, region, candidateEaveEstimate, boundaryContacts )
                                                : fitConstrainedFlatTopRoof( ring, fitSamples, region, candidateEaveEstimate, boundaryContacts );
      if ( !candidate.success )
        continue;
      double loss = std::numeric_limits<double>::max();
      if ( !evaluateFlatTopRoofCandidate( ring, candidate, region, fitSamples, observations, loss ) )
        continue;
      const MeshResult candidateMesh = fittedFlatTopRoofMesh( ring, candidate );
      if ( !candidateMesh.success )
        continue;
      const QgsGeometry supportedTop = reliefGeometryFromPoints( region, footprint, ringExtentSize( ring ), true );
      const QgsGeometry fittedTop = polygonGeometryFromRing( candidate.topRing );
      const double overlapArea = supportedTop.intersection( fittedTop ).area();
      const double unionArea = supportedTop.area() + fittedTop.area() - overlapArea;
      const double topCoverage = unionArea > 1e-8 ? overlapArea / unionArea : 0.0;
      if ( topCoverage < 0.45 )
        continue;
      const double score = loss + ( 1.0 - topCoverage ) * 0.35 - region.areaRatio * 0.20;
      if ( score < bestScore )
      {
        bestScore = score;
        result = candidateMesh;
      }
    }
  }
  if ( !result.success )
  {
    // This must remain outside the strict slope-support loop so sparse or missing sides can use the legacy geometry path.
    double fallbackScore = std::numeric_limits<double>::max();
    for ( const HorizontalRoofRegion &region : regions )
    {
      double candidateScore = std::numeric_limits<double>::max();
      const MeshResult candidate = fallbackFlatTopRoofMesh( ring, region, observations, candidateScore );
      if ( candidate.success && candidateScore < fallbackScore )
      {
        fallbackScore = candidateScore;
        result = candidate;
      }
    }
    if ( !result.success )
      result.error = QStringLiteral( "Neither the fitted nor the fallback flat-top roof could be built from a supported upper plane and lower roof samples." );
  }
  return result;
}

static BuildingRoof::MeshResult buildCurvedRoofPrismMeshFromKeypoints( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<BuildingRoof::RoofPoint> &roofPoints,
                                                                       const FittedBarrelProfile *fittedBarrel = nullptr )
{
  using MeshResult = BuildingRoof::MeshResult;
  using RoofPoint = BuildingRoof::RoofPoint;
  Q_UNUSED( buildingHeight )
  MeshResult result;

  const QList<RoofPoint> boundaries = boundaryPoints( roofPoints );
  const QList<RoofPoint> ridges = ridgePoints( roofPoints );
  const QList<RoofPoint> vertices = vertexPoints( roofPoints );
  const QList<RoofPoint> surfaces = surfacePoints( roofPoints );
  if ( boundaries.size() != 1 || !ridges.isEmpty() || !vertices.isEmpty() || ( surfaces.size() != 1 && surfaces.size() != 2 ) )
  {
    result.error = QStringLiteral( "Curved roof requires exactly one boundary point and one or two surface points." );
    return result;
  }

  const QgsPoint boundaryPoint = boundaries.first().point;
  for ( const RoofPoint &surface : surfaces )
  {
    if ( surface.point.z() <= boundaryPoint.z() + 1e-6 )
    {
      result.error = QStringLiteral( "Surface points must be higher than the boundary point." );
      return result;
    }
  }

  const QgsPolygonXY polygon = firstPolygon( buildingGeometry );
  const QVector<QgsPointXY> ring = exteriorRing( polygon );
  if ( ring.size() < 3 )
  {
    result.error = QStringLiteral( "The building footprint cannot be used for curved roof mesh generation." );
    return result;
  }

  for ( const RoofPoint &surface : surfaces )
  {
    if ( !pointInRing( ring, QgsPointXY( surface.point.x(), surface.point.y() ) ) )
    {
      result.error = QStringLiteral( "Surface points must lie inside the building footprint." );
      return result;
    }
  }

  if ( surfaces.size() == 2 )
  {
    const QgsPoint &a = surfaces.at( 0 ).point;
    const QgsPoint &b = surfaces.at( 1 ).point;
    if ( std::hypot( a.x() - b.x(), a.y() - b.y() ) <= 1e-8 )
    {
      result.error = QStringLiteral( "The two barrel roof surface points must have different positions." );
      return result;
    }
  }

  appendCurvedRoofWall( result.mesh, ring, surfaces, boundaryPoint.z(), fittedBarrel );
  appendHorizontalRingSurface( result.mesh, ring, 0.0, true );
  appendCurvedRoofSurface( result.mesh, ring, surfaces, boundaryPoint.z(), fittedBarrel );

  result.success = !result.mesh.isEmpty();
  if ( !result.success )
    result.error = QStringLiteral( "Curved roof mesh generation failed." );
  return result;
}

static BuildingRoof::MeshResult buildApexRoofPrismMeshFromKeypoints( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<BuildingRoof::RoofPoint> &roofPoints )
{
  using MeshResult = BuildingRoof::MeshResult;
  using RoofPoint = BuildingRoof::RoofPoint;
  Q_UNUSED( buildingHeight )
  MeshResult result;

  const QList<RoofPoint> boundaries = boundaryPoints( roofPoints );
  const QList<RoofPoint> ridges = ridgePoints( roofPoints );
  const QList<RoofPoint> vertices = vertexPoints( roofPoints );
  if ( boundaries.size() != 1 || !ridges.isEmpty() || vertices.size() != 1 )
  {
    result.error = QStringLiteral( "Apex roof requires exactly one boundary point and one vertex point." );
    return result;
  }

  const QgsPoint boundaryPoint = boundaries.first().point;
  const QgsPoint apexPoint = vertices.first().point;
  if ( apexPoint.z() <= boundaryPoint.z() + 1e-6 )
  {
    result.error = QStringLiteral( "The apex point must be higher than the boundary point." );
    return result;
  }

  const QgsPolygonXY polygon = firstPolygon( buildingGeometry );
  const QVector<QgsPointXY> ring = exteriorRing( polygon );
  if ( ring.size() < 3 )
  {
    result.error = QStringLiteral( "The building footprint cannot be used for apex roof mesh generation." );
    return result;
  }

  appendVerticalWall( result.mesh, ring, 0.0, boundaryPoint.z() );
  appendHorizontalRingSurface( result.mesh, ring, 0.0, true );
  appendApexRoofSurface( result.mesh, ring, boundaryPoint.z(), apexPoint );
  appendApexStructureLines( result.mesh, ring, boundaryPoint.z(), apexPoint, 64 );

  result.success = !result.mesh.isEmpty();
  if ( !result.success )
    result.error = QStringLiteral( "Apex roof mesh generation failed." );
  return result;
}

BuildingRoof::MeshResult BuildingRoof::buildCurvedRoofPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints, const QVector<RoofSample> &pointCloudSamples )
{
  MeshResult result;
  const QList<RoofPoint> boundaries = boundaryPoints( roofPoints );
  const QList<RoofPoint> ridges = ridgePoints( roofPoints );
  const QList<RoofPoint> vertices = vertexPoints( roofPoints );
  const QList<RoofPoint> surfaces = surfacePoints( roofPoints );

  const QgsPolygonXY polygon = firstPolygon( buildingGeometry );
  const QVector<QgsPointXY> ring = exteriorRing( polygon );
  if ( ring.size() < 3 || boundaries.size() != 1 || !ridges.isEmpty() || !vertices.isEmpty()
       || ( surfaces.size() != 1 && surfaces.size() != 2 ) )
  {
    result.error = QStringLiteral( "Curved roof requires one boundary marker and one or two surface markers." );
    return result;
  }
  const QVector<RoofSample> fitSamples = roofFitSamples( buildingGeometry, pointCloudSamples );
  if ( surfaces.size() == 2 )
  {
    double bestScore = std::numeric_limits<double>::max();
    double bestAxisX = 0.0;
    double bestAxisY = 0.0;
    const QgsPointXY center = ringCentroidPoint( ring );
    auto considerDirection = [&]( double axisX, double axisY ) {
      FittedBarrelProfile fittedProfile;
      double score = std::numeric_limits<double>::max();
      if ( !fitBarrelProfileFromPointCloud( ring, axisX, axisY, fitSamples, fittedProfile, score ) || score >= bestScore )
        return;
      axisX = fittedProfile.normalY;
      axisY = -fittedProfile.normalX;
      const QVector<LineInterval> intervals = lineInsideRingIntervals( ring, center, axisX, axisY );
      LineInterval interval;
      if ( !findIntervalContaining( intervals, 0.0, 0.0, interval ) )
        return;
      QList<RoofPoint> fittedPoints{ RoofPoint{ QgsPoint( ring.first().x(), ring.first().y(), fittedProfile.anchors.first().z ), boundaries.first().type } };
      const double fittedTop = fittedProfile.anchors.at( fittedProfile.anchors.size() / 2 ).z;
      for ( int i = 0; i < surfaces.size(); ++i )
      {
        const double t = interval.start + ( interval.end - interval.start ) * ( i == 0 ? 0.25 : 0.75 );
        const QgsPointXY point = pointOnLine( center, axisX, axisY, t );
        fittedPoints.append( RoofPoint{ QgsPoint( point.x(), point.y(), fittedTop ), surfaces.at( i ).type } );
      }
      const MeshResult candidate = buildCurvedRoofPrismMeshFromKeypoints( buildingGeometry, buildingHeight, fittedPoints, &fittedProfile );
      if ( !candidate.success )
        return;
      bestScore = score;
      bestAxisX = axisX;
      bestAxisY = axisY;
      result = candidate;
    };
    for ( const auto &direction : automaticBarrelAxisDirections( ring, fitSamples ) )
      considerDirection( direction.first, direction.second );
    QgsPointXY leftA, rightA, leftB, rightB;
    if ( result.success && !barrelQuadrilateralEnds( ring, -bestAxisY, bestAxisX, leftA, rightA, leftB, rightB ) )
    {
      for ( double degrees : { 5.0, 2.0, 0.5 } )
      {
        const double angle = degrees * 3.14159265358979323846 / 180.0;
        const double axisX = bestAxisX;
        const double axisY = bestAxisY;
        for ( double sign : { -1.0, 1.0 } )
          considerDirection( axisX * std::cos( angle ) - sign * axisY * std::sin( angle ),
                             sign * axisX * std::sin( angle ) + axisY * std::cos( angle ) );
      }
    }
    if ( !result.success )
      result.error = QStringLiteral( "The point cloud does not support an automatic barrel roof axis and symmetric profile." );
    return result;
  }
  const QList<RoofPoint> fittedPoints = fittedDomeRoofPoints( ring, boundaries, surfaces, fitSamples, pointCloudSamples );
  if ( !fittedPoints.isEmpty() )
  {
    result = buildCurvedRoofPrismMeshFromKeypoints( buildingGeometry, buildingHeight, fittedPoints );
    if ( result.success )
      return result;
  }
  const QList<RoofPoint> automaticPoints = topHeightSingleSurfaceRoofPoints( ring, boundaries, surfaces, fitSamples, pointCloudSamples );
  if ( !automaticPoints.isEmpty() )
    result = buildCurvedRoofPrismMeshFromKeypoints( buildingGeometry, buildingHeight, automaticPoints );
  if ( !result.success )
    result.error = QStringLiteral( "The point cloud does not support an automatic dome and eave height." );
  return result;
}

BuildingRoof::MeshResult BuildingRoof::buildApexRoofPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints, const QVector<RoofSample> &pointCloudSamples )
{
  MeshResult result;
  const QList<RoofPoint> boundaries = boundaryPoints( roofPoints );
  const QList<RoofPoint> ridges = ridgePoints( roofPoints );
  const QList<RoofPoint> vertices = vertexPoints( roofPoints );

  const QgsPolygonXY polygon = firstPolygon( buildingGeometry );
  const QVector<QgsPointXY> ring = exteriorRing( polygon );
  if ( !pointCloudSamples.isEmpty() && ring.size() >= 3 && boundaries.size() == 1 && ridges.isEmpty() && vertices.size() == 1 )
  {
    const QVector<RoofSample> fitSamples = roofFitSamples( buildingGeometry, pointCloudSamples );
    const QList<RoofPoint> automaticPoints = topHeightApexRoofPoints( ring, boundaries, vertices, fitSamples, pointCloudSamples );
    if ( !automaticPoints.isEmpty() )
    {
      MeshResult automaticResult = buildApexRoofPrismMeshFromKeypoints( buildingGeometry, buildingHeight, automaticPoints );
      if ( automaticResult.success )
        return automaticResult;
    }
  }

  result.error = QStringLiteral( "An apex roof requires one boundary marker, one apex marker and point-cloud support for automatic fitting." );
  return result;
}

static BuildingRoof::MeshResult buildGabledRoofPrismMeshFromKeypoints( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<BuildingRoof::RoofPoint> &roofPoints )
{
  using MeshResult = BuildingRoof::MeshResult;
  using RoofPoint = BuildingRoof::RoofPoint;
  Q_UNUSED( buildingHeight )
  MeshResult result;

  const QList<RoofPoint> boundary = boundaryPoints( roofPoints );
  QList<RoofPoint> ridges;
  for ( const RoofPoint &roofPoint : roofPoints )
  {
    if ( isRidgePointType( roofPoint.type ) )
      ridges.append( roofPoint );
  }

  if ( boundary.size() != 1 || ( ridges.size() != 1 && ridges.size() != 3 ) )
  {
    result.error = QStringLiteral( "Gabled roof requires one boundary point and either one or three ridge points." );
    return result;
  }

  const QgsPoint boundaryPoint = boundary.first().point;
  const QgsPolygonXY polygon = firstPolygon( buildingGeometry );
  QVector<QgsPointXY> ring = exteriorRing( polygon );
  if ( ring.size() < 3 )
  {
    result.error = QStringLiteral( "The building footprint cannot be used for gabled roof mesh generation." );
    return result;
  }

  if ( ridges.size() == 3 )
  {
    const BentGableLayout layout = bentLayoutFromRidgePoints( ridges );
    const double ridgeHeight = ridges.at( 1 ).point.z();
    if ( !layout.success || ridgeHeight <= boundaryPoint.z() + 1e-6 )
    {
      result.error = QStringLiteral( "The fitted bent roof needs two connected ridge segments above the eaves." );
      return result;
    }

    const MeshResult bentResult = buildBentGabledRoofPrismMesh( ring, boundaryPoint, layout, ridgeHeight );
    if ( bentResult.success )
      return bentResult;

    result.error = bentResult.error.isEmpty() ? QStringLiteral( "Bent gabled roof mesh generation failed." ) : bentResult.error;
    return result;
  }

  double dirX = 0.0;
  double dirY = 0.0;
  if ( !nearestEdgeDirection( ring, boundaryPoint, dirX, dirY ) )
  {
    result.error = QStringLiteral( "Cannot infer ridge direction from the boundary point." );
    return result;
  }

  const QgsPoint ridgePoint = ridges.first().point;
  if ( ridgePoint.z() <= boundaryPoint.z() + 1e-6 )
  {
    result.error = QStringLiteral( "The ridge point must be higher than the boundary point." );
    return result;
  }

  const double normalX = -dirY;
  const double normalY = dirX;
  const double boundaryDistance = signedDistanceToLine( QgsPointXY( boundaryPoint.x(), boundaryPoint.y() ), ridgePoint, normalX, normalY );
  if ( std::fabs( boundaryDistance ) <= 1e-6 )
  {
    result.error = QStringLiteral( "The boundary point must not lie on the ridge line." );
    return result;
  }

  ring = ringWithRidgeIntersections( ring, ridgePoint, normalX, normalY );
  double sameSideLimit = 0.0;
  double oppositeLimit = 0.0;
  for ( const QgsPointXY &point : ring )
  {
    const double distance = signedDistanceToLine( point, ridgePoint, normalX, normalY );
    if ( distance * boundaryDistance < 0.0 )
      oppositeLimit = std::max( oppositeLimit, std::fabs( distance ) );
    else
      sameSideLimit = std::max( sameSideLimit, std::fabs( distance ) );
  }
  if ( sameSideLimit <= 1e-8 )
    sameSideLimit = std::fabs( boundaryDistance );

  const double baseHeight = boundaryPoint.z();
  for ( const QgsPointXY &point : ring )
  {
    result.mesh.vertices.append( QgsPoint( point.x(), point.y(), 0.0 ) );
    result.mesh.vertices.append( QgsPoint( point.x(), point.y(), gabledTopZ( point, boundaryPoint, ridgePoint, normalX, normalY, sameSideLimit, oppositeLimit, baseHeight ) ) );
  }

  const int count = ring.size();
  for ( int i = 0; i < count; ++i )
  {
    const int next = ( i + 1 ) % count;
    result.mesh.indices << 2 * i << 2 * next << 2 * i + 1;
    result.mesh.indices << 2 * i + 1 << 2 * next << 2 * next + 1;
  }

  const QVector<int> bottomTriangles = triangulateRing( ring );
  for ( int i = 0; i + 2 < bottomTriangles.size(); i += 3 )
    result.mesh.indices << 2 * bottomTriangles[i] << 2 * bottomTriangles[i + 2] << 2 * bottomTriangles[i + 1];

  appendTriangulatedRoofSurface( result.mesh, clipRingByRidgeSide( ring, ridgePoint, normalX, normalY, true ), boundaryPoint, ridgePoint, normalX, normalY, sameSideLimit, oppositeLimit, baseHeight );
  appendTriangulatedRoofSurface( result.mesh, clipRingByRidgeSide( ring, ridgePoint, normalX, normalY, false ), boundaryPoint, ridgePoint, normalX, normalY, sameSideLimit, oppositeLimit, baseHeight );

  result.success = !result.mesh.isEmpty();
  if ( !result.success )
    result.error = QStringLiteral( "Gabled roof mesh generation failed." );
  return result;
}

static BuildingRoof::MeshResult buildLeastSquaresClippedGabledRoofPrismMesh( const QgsGeometry &buildingGeometry, const QList<BuildingRoof::RoofPoint> &roofPoints, const GabledBoundarySlopePlanes &slopePlanes, double dirX, double dirY )
{
  using MeshResult = BuildingRoof::MeshResult;
  using RoofPoint = BuildingRoof::RoofPoint;
  MeshResult result;

  if ( !slopePlanes.success || !slopePlanes.firstPlane.success || !slopePlanes.secondPlane.success || !slopePlanes.hasSideSigns )
  {
    result.error = QStringLiteral( "Least-squares gabled roof planes are not reliably separated by ridge side." );
    return result;
  }

  const QList<RoofPoint> boundary = boundaryPoints( roofPoints );
  const QList<RoofPoint> ridges = ridgePoints( roofPoints );
  if ( boundary.size() != 1 || ridges.size() != 1 )
  {
    result.error = QStringLiteral( "Least-squares clipped gabled roof requires one boundary point and one ridge point." );
    return result;
  }

  const QgsPoint boundaryPoint = boundary.first().point;
  const QgsPoint ridgePoint = ridges.first().point;
  const QgsPolygonXY polygon = firstPolygon( buildingGeometry );
  QVector<QgsPointXY> ring = exteriorRing( polygon );
  if ( ring.size() < 3 )
  {
    result.error = QStringLiteral( "The building footprint cannot be used for least-squares clipped gabled roof mesh generation." );
    return result;
  }

  const double dirLength = std::hypot( dirX, dirY );
  if ( dirLength <= 1e-10 )
  {
    if ( !nearestEdgeDirection( ring, boundaryPoint, dirX, dirY ) )
    {
      result.error = QStringLiteral( "Cannot infer ridge direction for least-squares clipped gabled roof mesh generation." );
      return result;
    }
  }
  else
  {
    dirX /= dirLength;
    dirY /= dirLength;
  }

  const double normalX = -dirY;
  const double normalY = dirX;
  const double boundaryDistance = signedDistanceToLine( QgsPointXY( boundaryPoint.x(), boundaryPoint.y() ), ridgePoint, normalX, normalY );
  if ( std::fabs( boundaryDistance ) <= 1e-6 )
  {
    result.error = QStringLiteral( "The boundary point must not lie on the ridge line." );
    return result;
  }
  if ( !leastSquaresPlanesReliableForClippedMesh( slopePlanes, boundaryPoint, ridgePoint, normalX, normalY ) )
  {
    result.error = QStringLiteral( "Least-squares gabled roof planes are not reliable enough for direct clipped mesh generation." );
    return result;
  }

  const LeastSquaresRoofPlane *positivePlane = planeForRidgeSide( slopePlanes, true );
  const LeastSquaresRoofPlane *negativePlane = planeForRidgeSide( slopePlanes, false );
  if ( !positivePlane || !negativePlane )
  {
    result.error = QStringLiteral( "Cannot match least-squares planes to the two clipped gabled roof sides." );
    return result;
  }

  ring = ringWithRidgeIntersections( ring, ridgePoint, normalX, normalY );
  const double fallbackZ = boundaryPoint.z();
  for ( const QgsPointXY &point : ring )
  {
    result.mesh.vertices.append( QgsPoint( point.x(), point.y(), 0.0 ) );
    result.mesh.vertices.append( QgsPoint( point.x(), point.y(), leastSquaresGabledTopZForSide( point, slopePlanes, ridgePoint, normalX, normalY, fallbackZ ) ) );
  }

  const int count = ring.size();
  for ( int i = 0; i < count; ++i )
  {
    const int next = ( i + 1 ) % count;
    result.mesh.indices << 2 * i << 2 * next << 2 * i + 1;
    result.mesh.indices << 2 * i + 1 << 2 * next << 2 * next + 1;
  }

  const QVector<int> bottomTriangles = triangulateRing( ring );
  for ( int i = 0; i + 2 < bottomTriangles.size(); i += 3 )
    result.mesh.indices << 2 * bottomTriangles[i] << 2 * bottomTriangles[i + 2] << 2 * bottomTriangles[i + 1];

  const QVector<QgsPointXY> positiveRoofRing = clipRingByRidgeSide( ring, ridgePoint, normalX, normalY, true );
  const QVector<QgsPointXY> negativeRoofRing = clipRingByRidgeSide( ring, ridgePoint, normalX, normalY, false );
  appendTriangulatedLeastSquaresRoofSurface( result.mesh, positiveRoofRing, *positivePlane, fallbackZ );
  appendTriangulatedLeastSquaresRoofSurface( result.mesh, negativeRoofRing, *negativePlane, fallbackZ );

  result.success = !result.mesh.isEmpty();
  if ( !result.success )
    result.error = QStringLiteral( "Least-squares clipped gabled roof mesh generation failed." );
  return result;
}

BuildingRoof::MeshResult BuildingRoof::buildGabledRoofPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints, const QVector<RoofSample> &pointCloudSamples )
{
  MeshResult result;
  const QList<RoofPoint> boundary = boundaryPoints( roofPoints );
  const QList<RoofPoint> ridges = ridgePoints( roofPoints );
  if ( boundary.size() != 1 || ( ridges.size() != 1 && ridges.size() != 3 ) )
  {
    result.error = QStringLiteral( "Gabled roof requires one boundary point and either one or three ridge points as type markers." );
    return result;
  }
  const QVector<QgsPointXY> ring = exteriorRing( firstPolygon( buildingGeometry ) );
  const AutoHeightBand band = inferAutoGabledHeightBand( ring, pointCloudSamples );
  GabledFitContext fitContext;
  fitContext.heightBand = band;
  double bestScore = std::numeric_limits<double>::max();
  QString bentFailure;
  int bentFailureStage = 0;
  auto rememberBentFailure = [&]( int stage, const QString &message ) {
    if ( ridges.size() == 3 && stage >= bentFailureStage && !message.isEmpty() )
    {
      bentFailureStage = stage;
      bentFailure = message;
    }
  };
  for ( const QgsPoint &candidateBoundary : automaticRoofBoundaryCandidates( ring, band.baseHeight, ridges.size() == 1 ) )
  {
    const QList<RoofPoint> boundaryHints{ RoofPoint{ candidateBoundary, boundary.first().type } };
    QList<RoofPoint> ridgeMarkers;
    for ( const RoofPoint &ridge : ridges )
      ridgeMarkers.append( RoofPoint{ QgsPoint(), ridge.type } );
    GabledBoundarySlopePlanes automaticSlopePlanes;
    double automaticRidgeDirX = 0.0;
    double automaticRidgeDirY = 0.0;
    QList<RoofPoint> automaticPoints = topHeightGabledRoofPoints( ring, boundaryHints, ridgeMarkers, pointCloudSamples,
                                                                &automaticSlopePlanes, &automaticRidgeDirX, &automaticRidgeDirY, &fitContext );
    if ( automaticPoints.isEmpty() )
    {
      rememberBentFailure( 1, QStringLiteral( "No supported two-arm ridge layout was found in the building footprint." ) );
      continue;
    }
    if ( ridges.size() == 3 )
    {
      QString heightFailure;
      if ( !fitAutomaticBentRoofHeights( ring, pointCloudSamples, band, automaticPoints, &heightFailure ) )
      {
        rememberBentFailure( 2, heightFailure );
        continue;
      }
    }
    MeshResult candidate;
    if ( ridges.size() == 1 && automaticSlopePlanes.success && automaticSlopePlanes.hasSideSigns )
      candidate = buildLeastSquaresClippedGabledRoofPrismMesh( buildingGeometry, automaticPoints, automaticSlopePlanes, automaticRidgeDirX, automaticRidgeDirY );
    if ( !candidate.success )
      candidate = buildGabledRoofPrismMeshFromKeypoints( buildingGeometry, buildingHeight, automaticPoints );
    if ( !candidate.success )
    {
      rememberBentFailure( 3, candidate.error );
      continue;
    }
    const double score = automaticRoofMeshScore( candidate.mesh, pointCloudSamples, band );
    if ( !( score < std::numeric_limits<double>::max() ) )
      rememberBentFailure( 4, QStringLiteral( "The constructed bent roof does not have enough point-cloud support." ) );
    if ( score < bestScore )
    {
      bestScore = score;
      result = candidate;
    }
  }
  if ( !result.success )
  {
    result.error = ridges.size() == 3
                     ? QStringLiteral( "Cannot build an automatic bent gabled roof: %1" ).arg( bentFailure )
                     : QStringLiteral( "The point cloud does not support an automatic gabled roof direction and profile." );
  }
  return result;
}

static BuildingRoof::MeshResult buildHippedRoofPrismMeshFromKeypoints( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<BuildingRoof::RoofPoint> &roofPoints )
{
  using MeshResult = BuildingRoof::MeshResult;
  using RoofPoint = BuildingRoof::RoofPoint;
  Q_UNUSED( buildingHeight )

  const QList<RoofPoint> boundaries = boundaryPoints( roofPoints );
  QList<RoofPoint> ridges = ridgePoints( roofPoints );
  if ( boundaries.size() != 1 || ridges.size() != 2 )
  {
    MeshResult result;
    result.error = QStringLiteral( "Hipped roof requires exactly one boundary point and two ridge points." );
    return result;
  }

  averageRidgePairHeightIfClose( ridges[0], ridges[1] );

  const RoofPoint boundary = boundaries.first();
  if ( ridges.at( 0 ).point.z() <= boundary.point.z() + 1e-6 || ridges.at( 1 ).point.z() <= boundary.point.z() + 1e-6 )
  {
    MeshResult result;
    result.error = QStringLiteral( "Hipped roof ridge points must be higher than the boundary point." );
    return result;
  }

  const QgsPolygonXY polygon = firstPolygon( buildingGeometry );
  const QVector<QgsPointXY> ring = exteriorRing( polygon );
  double edgeX = 0.0;
  double edgeY = 0.0;
  if ( !nearestEdgeDirection( ring, boundary.point, edgeX, edgeY ) )
  {
    MeshResult result;
    result.error = QStringLiteral( "Cannot infer hipped roof direction from the boundary point." );
    return result;
  }

  const double ridgeX = ridges.at( 1 ).point.x() - ridges.at( 0 ).point.x();
  const double ridgeY = ridges.at( 1 ).point.y() - ridges.at( 0 ).point.y();
  const double ridgeLength = std::hypot( ridgeX, ridgeY );
  if ( ridgeLength <= 1e-8 )
  {
    MeshResult result;
    result.error = QStringLiteral( "Hipped roof ridge points must have different positions." );
    return result;
  }

  const double parallelScore = std::fabs( ( ridgeX / ridgeLength ) * edgeX + ( ridgeY / ridgeLength ) * edgeY );
  if ( parallelScore < 0.35 )
  {
    MeshResult result;
    result.error = QStringLiteral( "Hipped roof ridge line should be roughly parallel to the selected boundary edge." );
    return result;
  }

  QVector<AnchorPoint> eaveAnchors;
  QVector<AnchorPoint> roofAnchors;
  appendAnchor( eaveAnchors, QgsPointXY( boundary.point.x(), boundary.point.y() ), boundary.point.z() );
  appendAnchor( roofAnchors, QgsPointXY( ridges.at( 0 ).point.x(), ridges.at( 0 ).point.y() ), ridges.at( 0 ).point.z() );
  appendAnchor( roofAnchors, QgsPointXY( ridges.at( 1 ).point.x(), ridges.at( 1 ).point.y() ), ridges.at( 1 ).point.z() );

  MeshResult result = buildAnchoredRoofPrismMesh( buildingGeometry, eaveAnchors, roofAnchors, QStringLiteral( "Hipped roof" ) );
  if ( result.success )
  {
    const QgsPoint firstRidge( ridges.at( 0 ).point.x(), ridges.at( 0 ).point.y(), ridges.at( 0 ).point.z() );
    const QgsPoint secondRidge( ridges.at( 1 ).point.x(), ridges.at( 1 ).point.y(), ridges.at( 1 ).point.z() );
    result.mesh.structureLines.append( qMakePair( firstRidge, secondRidge ) );

    auto appendNearestHipLines = [&]( const QgsPoint &ridgePoint ) {
      QVector<QPair<double, QgsPoint>> eaveCandidates;
      eaveCandidates.reserve( ring.size() );
      for ( const QgsPointXY &ringPoint : ring )
      {
        const double dx = ringPoint.x() - ridgePoint.x();
        const double dy = ringPoint.y() - ridgePoint.y();
        const double z = eaveHeightAt( eaveAnchors, ringPoint, boundary.point.z() );
        eaveCandidates.append( qMakePair( dx * dx + dy * dy, QgsPoint( ringPoint.x(), ringPoint.y(), z ) ) );
      }
      std::sort( eaveCandidates.begin(), eaveCandidates.end(), []( const QPair<double, QgsPoint> &left, const QPair<double, QgsPoint> &right ) {
        return left.first < right.first;
      } );

      int appended = 0;
      for ( const QPair<double, QgsPoint> &candidate : eaveCandidates )
      {
        if ( candidate.first <= 1e-10 )
          continue;
        result.mesh.structureLines.append( qMakePair( ridgePoint, candidate.second ) );
        if ( ++appended >= 2 )
          break;
      }
    };
    appendNearestHipLines( firstRidge );
    appendNearestHipLines( secondRidge );
  }
  return result;
}

BuildingRoof::MeshResult BuildingRoof::buildHippedRoofPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints, const QVector<RoofSample> &pointCloudSamples )
{
  MeshResult result;
  const QList<RoofPoint> boundaries = boundaryPoints( roofPoints );
  const QList<RoofPoint> ridges = ridgePoints( roofPoints );
  if ( boundaries.size() != 1 || ridges.size() != 2 )
  {
    result.error = QStringLiteral( "Hipped roof requires one boundary point and two ridge points as type markers." );
    return result;
  }
  const QVector<QgsPointXY> ring = exteriorRing( firstPolygon( buildingGeometry ) );
  const AutoHeightBand band = inferAutoGabledHeightBand( ring, pointCloudSamples );
  double bestScore = std::numeric_limits<double>::max();
  QList<RoofPoint> ridgeMarkers;
  for ( const RoofPoint &ridge : ridges )
    ridgeMarkers.append( RoofPoint{ QgsPoint(), ridge.type } );
  for ( const QgsPoint &candidateBoundary : automaticRoofBoundaryCandidates( ring, band.baseHeight ) )
  {
    const QList<RoofPoint> boundaryHints{ RoofPoint{ candidateBoundary, boundaries.first().type } };
    const QList<RoofPoint> automaticPoints = topHeightHippedRoofPoints( ring, boundaryHints, ridgeMarkers, pointCloudSamples );
    if ( automaticPoints.isEmpty() )
      continue;
    const MeshResult candidate = buildHippedRoofPrismMeshFromKeypoints( buildingGeometry, buildingHeight, automaticPoints );
    if ( !candidate.success )
      continue;
    const double score = automaticRoofMeshScore( candidate.mesh, pointCloudSamples, band );
    if ( score < bestScore )
    {
      bestScore = score;
      result = candidate;
    }
  }
  if ( !result.success )
    result.error = QStringLiteral( "The point cloud does not support an automatic hipped roof direction and ridge." );
  return result;
}
