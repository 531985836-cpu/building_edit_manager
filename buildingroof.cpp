#include "buildingroof.h"

#include <qgsgeometrycollection.h>
#include <qgslinestring.h>
#include <qgspolygon.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <queue>
#include <set>

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

  double pointOnSlopeZ( const QgsPointXY &point, const QgsPoint &lowPoint, const QgsPoint &highPoint, double baseHeight )
  {
    const double axisX = highPoint.x() - lowPoint.x();
    const double axisY = highPoint.y() - lowPoint.y();
    const double axisLength2 = axisX * axisX + axisY * axisY;
    if ( axisLength2 <= 1e-12 )
      return baseHeight;

    const double t = ( ( point.x() - lowPoint.x() ) * axisX + ( point.y() - lowPoint.y() ) * axisY ) / axisLength2;
    return baseHeight + t * std::fabs( highPoint.z() - lowPoint.z() );
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

  QVector<QgsPointXY> concaveReliefBoxFromHighCluster( const QVector<ClusterSample> &samples, int clusterCount, const QgsPointXY &targetPoint, double axisX, double axisY, double normalX, double normalY, double padding )
  {
    if ( clusterCount <= 0 )
      return {};

    const double targetU = targetPoint.x() * axisX + targetPoint.y() * axisY;
    const double targetV = targetPoint.x() * normalX + targetPoint.y() * normalY;

    struct ClusterBounds
    {
      int cluster = -1;
      int count = 0;
      double minU = std::numeric_limits<double>::max();
      double maxU = -std::numeric_limits<double>::max();
      double minV = std::numeric_limits<double>::max();
      double maxV = -std::numeric_limits<double>::max();
      double minDistance2 = std::numeric_limits<double>::max();
    };

    ClusterBounds best;
    for ( int cluster = 0; cluster < clusterCount; ++cluster )
    {
      ClusterBounds candidate;
      candidate.cluster = cluster;
      bool hasLeft = false;
      bool hasRight = false;
      bool hasBottom = false;
      bool hasTop = false;

      for ( const ClusterSample &sample : samples )
      {
        if ( sample.cluster != cluster )
          continue;

        ++candidate.count;
        candidate.minU = std::min( candidate.minU, sample.u );
        candidate.maxU = std::max( candidate.maxU, sample.u );
        candidate.minV = std::min( candidate.minV, sample.v );
        candidate.maxV = std::max( candidate.maxV, sample.v );
        hasLeft = hasLeft || sample.u < targetU;
        hasRight = hasRight || sample.u > targetU;
        hasBottom = hasBottom || sample.v < targetV;
        hasTop = hasTop || sample.v > targetV;

        const double du = sample.u - targetU;
        const double dv = sample.v - targetV;
        candidate.minDistance2 = std::min( candidate.minDistance2, du * du + dv * dv );
      }

      if ( candidate.count < 3 || !hasLeft || !hasRight || !hasBottom || !hasTop )
        continue;
      if ( targetU <= candidate.minU || targetU >= candidate.maxU || targetV <= candidate.minV || targetV >= candidate.maxV )
        continue;

      if ( best.cluster < 0 || candidate.count > best.count || ( candidate.count == best.count && candidate.minDistance2 < best.minDistance2 ) )
        best = candidate;
    }

    if ( best.cluster < 0 )
      return {};

    double cellSize = std::max( padding, 1e-6 );
    const double spanU = std::max( best.maxU - best.minU, cellSize );
    const double spanV = std::max( best.maxV - best.minV, cellSize );
    const double maxGridSide = 220.0;
    cellSize = std::max( cellSize, std::max( spanU, spanV ) / maxGridSide );

    const double originU = best.minU - cellSize;
    const double originV = best.minV - cellSize;
    const int columns = std::max( 3, static_cast<int>( std::ceil( ( spanU + cellSize * 2.0 ) / cellSize ) ) );
    const int rows = std::max( 3, static_cast<int>( std::ceil( ( spanV + cellSize * 2.0 ) / cellSize ) ) );
    const int total = columns * rows;
    QVector<unsigned char> occupied( total, 0 );

    auto cellIndex = [columns]( int col, int row ) {
      return row * columns + col;
    };
    auto colFromU = [&]( double u ) {
      return std::max( 0, std::min( columns - 1, static_cast<int>( std::floor( ( u - originU ) / cellSize ) ) ) );
    };
    auto rowFromV = [&]( double v ) {
      return std::max( 0, std::min( rows - 1, static_cast<int>( std::floor( ( v - originV ) / cellSize ) ) ) );
    };

    for ( const ClusterSample &sample : samples )
    {
      if ( sample.cluster != best.cluster )
        continue;
      occupied[cellIndex( colFromU( sample.u ), rowFromV( sample.v ) )] = 1;
    }

    QVector<unsigned char> closed = occupied;
    for ( int row = 0; row < rows; ++row )
    {
      for ( int col = 0; col < columns; ++col )
      {
        if ( !occupied.at( cellIndex( col, row ) ) )
          continue;
        for ( int dr = -1; dr <= 1; ++dr )
        {
          for ( int dc = -1; dc <= 1; ++dc )
          {
            const int nc = col + dc;
            const int nr = row + dr;
            if ( nc >= 0 && nc < columns && nr >= 0 && nr < rows )
              closed[cellIndex( nc, nr )] = 1;
          }
        }
      }
    }

    const int startCol = colFromU( targetU );
    const int startRow = rowFromV( targetV );
    closed[cellIndex( startCol, startRow )] = 0;

    QVector<unsigned char> visited( total, 0 );
    QVector<int> queue;
    queue.reserve( total );
    queue.append( cellIndex( startCol, startRow ) );
    visited[cellIndex( startCol, startRow )] = 1;

    int minCol = startCol;
    int maxCol = startCol;
    int minRow = startRow;
    int maxRow = startRow;
    bool touchesGridEdge = false;
    for ( int cursor = 0; cursor < queue.size(); ++cursor )
    {
      const int index = queue.at( cursor );
      const int col = index % columns;
      const int row = index / columns;
      minCol = std::min( minCol, col );
      maxCol = std::max( maxCol, col );
      minRow = std::min( minRow, row );
      maxRow = std::max( maxRow, row );
      touchesGridEdge = touchesGridEdge || col == 0 || row == 0 || col == columns - 1 || row == rows - 1;

      const int dCol[4] = { -1, 1, 0, 0 };
      const int dRow[4] = { 0, 0, -1, 1 };
      for ( int i = 0; i < 4; ++i )
      {
        const int nextCol = col + dCol[i];
        const int nextRow = row + dRow[i];
        if ( nextCol < 0 || nextCol >= columns || nextRow < 0 || nextRow >= rows )
          continue;
        const int nextIndex = cellIndex( nextCol, nextRow );
        if ( visited.at( nextIndex ) || closed.at( nextIndex ) )
          continue;
        visited[nextIndex] = 1;
        queue.append( nextIndex );
      }
    }

    if ( touchesGridEdge || queue.size() < 2 )
      return {};

    const double expand = cellSize * 0.75;
    const double minU = std::max( best.minU, originU + minCol * cellSize - expand );
    const double maxU = std::min( best.maxU, originU + ( maxCol + 1 ) * cellSize + expand );
    const double minV = std::max( best.minV, originV + minRow * cellSize - expand );
    const double maxV = std::min( best.maxV, originV + ( maxRow + 1 ) * cellSize + expand );
    if ( maxU - minU <= cellSize || maxV - minV <= cellSize )
      return {};

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
    for ( int i = 0; i < samples.size(); ++i )
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

  QgsGeometry roofSurfaceGeometry( const QVector<QgsPointXY> &ring, const QgsPoint &lowPoint, const QgsPoint &highPoint, double baseHeight )
  {
    QVector<QgsPoint> points;
    points.reserve( ring.size() + 1 );
    for ( const QgsPointXY &point : ring )
      points.append( QgsPoint( point.x(), point.y(), pointOnSlopeZ( point, lowPoint, highPoint, baseHeight ) ) );
    if ( !points.isEmpty() )
      points.append( points.first() );

    std::unique_ptr<QgsPolygon> polygon = std::make_unique<QgsPolygon>();
    polygon->setExteriorRing( new QgsLineString( points ) );
    return QgsGeometry( polygon.release() );
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

  double chooseEndpointOnRidgeSide( const LineInterval &interval, double bendT, double ridgeT )
  {
    const double side = ridgeT - bendT;
    double bestT = std::fabs( interval.start - bendT ) > std::fabs( interval.end - bendT ) ? interval.start : interval.end;
    double bestDistance = -1.0;
    for ( double candidate : { interval.start, interval.end } )
    {
      if ( ( candidate - bendT ) * side < -1e-7 )
        continue;

      const double distance = std::fabs( candidate - bendT );
      if ( distance > bestDistance )
      {
        bestDistance = distance;
        bestT = candidate;
      }
    }
    return bestT;
  }

  double chooseFarthestEndpoint( const LineInterval &interval, double originT )
  {
    return std::fabs( interval.start - originT ) > std::fabs( interval.end - originT ) ? interval.start : interval.end;
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

  LeastSquaresRoofPlane fitRoofPlaneAcrossRidgeDirection( const QVector<QgsPoint> &points, double normalX, double normalY )
  {
    LeastSquaresRoofPlane plane;
    if ( points.size() < 8 )
      return plane;

    const double normalLength = std::hypot( normalX, normalY );
    if ( normalLength <= 1e-10 )
      return plane;
    normalX /= normalLength;
    normalY /= normalLength;

    double meanS = 0.0;
    double meanZ = 0.0;
    for ( const QgsPoint &point : points )
    {
      meanS += point.x() * normalX + point.y() * normalY;
      meanZ += point.z();
    }
    meanS /= points.size();
    meanZ /= points.size();

    double sumSS = 0.0;
    double sumSZ = 0.0;
    for ( const QgsPoint &point : points )
    {
      const double centeredS = point.x() * normalX + point.y() * normalY - meanS;
      const double centeredZ = point.z() - meanZ;
      sumSS += centeredS * centeredS;
      sumSZ += centeredS * centeredZ;
    }
    if ( sumSS <= 1e-10 )
      return plane;

    const double slope = sumSZ / sumSS;
    const double intercept = meanZ - slope * meanS;
    double squaredErrorSum = 0.0;
    for ( const QgsPoint &point : points )
    {
      const double predictedZ = slope * ( point.x() * normalX + point.y() * normalY ) + intercept;
      const double error = point.z() - predictedZ;
      squaredErrorSum += error * error;
    }

    plane.success = std::isfinite( slope ) && std::isfinite( intercept );
    plane.a = slope * normalX;
    plane.b = slope * normalY;
    plane.c = intercept;
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

  double constrainedGabledPlaneRoofZ( const LeastSquaresRoofPlane &plane, const QgsPointXY &point, const QgsPoint &ridgePoint, double normalX, double normalY, double fallbackZ )
  {
    const double distance = signedDistanceToLine( point, ridgePoint, normalX, normalY );
    const double slopeAcrossRidge = plane.a * normalX + plane.b * normalY;
    double z = ridgePoint.z() + slopeAcrossRidge * distance;
    if ( !std::isfinite( z ) || z <= 0.0 )
      return fallbackZ;
    if ( z > ridgePoint.z() )
      z = ridgePoint.z();
    return z;
  }

  double leastSquaresGabledTopZForSide( const QgsPointXY &point, const GabledBoundarySlopePlanes &planes, const QgsPoint &ridgePoint, double normalX, double normalY, double fallbackZ )
  {
    const double distance = signedDistanceToLine( point, ridgePoint, normalX, normalY );
    if ( std::fabs( distance ) <= 1e-7 )
      return ridgePoint.z();

    const LeastSquaresRoofPlane *plane = planeForRidgeSide( planes, distance > 0.0 );
    if ( !plane )
      return fallbackZ;
    return constrainedGabledPlaneRoofZ( *plane, point, ridgePoint, normalX, normalY, fallbackZ );
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

    const double boundaryPlaneZ = constrainedGabledPlaneRoofZ( *boundaryPlane, boundaryXY, ridgePoint, normalX, normalY, boundaryPoint.z() );
    if ( !std::isfinite( boundaryPlaneZ ) || boundaryPlaneZ <= 0.0 || boundaryPlaneZ >= ridgePoint.z() - 0.10 )
      return false;

    return true;
  }

  void appendTriangulatedLeastSquaresRoofSurface( BuildingRoof::Mesh &mesh, const QVector<QgsPointXY> &polygon, const LeastSquaresRoofPlane &plane, const QgsPoint &ridgePoint, double normalX, double normalY, double fallbackZ )
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
      mesh.vertices.append( QgsPoint( point.x(), point.y(), constrainedGabledPlaneRoofZ( plane, point, ridgePoint, normalX, normalY, fallbackZ ) ) );

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

  bool extractDebugRansacPlane( const QVector<QgsPoint> &points, int minimumInliers, LeastSquaresRoofPlane &plane, QVector<QgsPoint> &inliers )
  {
    plane = LeastSquaresRoofPlane();
    inliers.clear();
    if ( points.size() < std::max( 3, minimumInliers ) )
      return false;

    const double threshold = robustPlaneThreshold( points );
    const int iterations = points.size() > 200 ? 520 : 320;
    quint32 state = 2166136261u ^ static_cast<quint32>( points.size() * 16777619u );
    auto nextIndex = [&state, &points]() -> int {
      state = state * 1664525u + 1013904223u;
      return static_cast<int>( state % static_cast<quint32>( points.size() ) );
    };

    QVector<QgsPoint> bestInliers;
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
      if ( !std::isfinite( slope ) || slope > 5.0 )
        continue;

      QVector<QgsPoint> candidateInliers;
      candidateInliers.reserve( points.size() );
      double squaredError = 0.0;
      for ( const QgsPoint &point : points )
      {
        const double residual = roofPlaneResidual( candidate, point );
        if ( residual <= threshold )
        {
          candidateInliers.append( point );
          squaredError += residual * residual;
        }
      }
      if ( candidateInliers.size() < minimumInliers )
        continue;

      const double meanError = squaredError / candidateInliers.size();
      if ( candidateInliers.size() > bestInliers.size()
           || ( candidateInliers.size() == bestInliers.size() && meanError < bestMeanError ) )
      {
        bestInliers = candidateInliers;
        bestMeanError = meanError;
      }
    }

    if ( bestInliers.size() < minimumInliers )
      return false;

    plane = fitLeastSquaresRoofPlane( bestInliers );
    if ( !plane.success )
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

  bool leastSquaresGabledRidgePointFromPointCloud( const QVector<QgsPointXY> &ring, const QVector<BuildingRoof::RoofSample> &pointCloudSamples, const QgsPoint &boundaryPoint, QgsPoint &ridgePoint, double *ridgeDirX = nullptr, double *ridgeDirY = nullptr, GabledBoundarySlopePlanes *boundarySlopePlanes = nullptr )
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

    const AutoHeightBand heightBand = inferAutoGabledHeightBand( ring, pointCloudSamples );
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

    const BuildingRoof::RoofPlaneSegmentation dominantSegmentation = BuildingRoof::segmentRoofPlanesForDebug( polygonGeometryFromRing( ring ), pointCloudSamples );
    double dominantPairScore = std::numeric_limits<double>::max();
    QgsPoint dominantRidgePoint;
    double dominantDirX = baseDirX;
    double dominantDirY = baseDirY;
    GabledBoundarySlopePlanes dominantSlopePlanes;
    bool dominantPairFound = false;
    for ( int firstIndex = 0; firstIndex < dominantSegmentation.segments.size(); ++firstIndex )
    {
      const BuildingRoof::RoofPlaneSegment &firstSegment = dominantSegmentation.segments.at( firstIndex );
      if ( !firstSegment.accepted || firstSegment.points.size() < 12 )
        continue;
      const LeastSquaresRoofPlane firstPlane = fitRoofPlaneAcrossRidgeDirection( firstSegment.points, normalX, normalY );
      if ( !firstPlane.success )
        continue;

      for ( int secondIndex = firstIndex + 1; secondIndex < dominantSegmentation.segments.size(); ++secondIndex )
      {
        const BuildingRoof::RoofPlaneSegment &secondSegment = dominantSegmentation.segments.at( secondIndex );
        if ( !secondSegment.accepted || secondSegment.points.size() < 12 )
          continue;
        const LeastSquaresRoofPlane secondPlane = fitRoofPlaneAcrossRidgeDirection( secondSegment.points, normalX, normalY );
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
        if ( combinedAreaRatio < 0.10 )
          continue;
        const double balance = static_cast<double>( std::min( firstSegment.points.size(), secondSegment.points.size() ) )
                               / std::max( firstSegment.points.size(), secondSegment.points.size() );
        const double ridgeHeightPenalty = std::fabs( ridgeHeight - heightBand.ridgeHeight )
                                          / std::max( 0.30, heightBand.highHeight - heightBand.lowHeight );
        const double score = ( firstPlane.rmse * firstSegment.points.size() + secondPlane.rmse * secondSegment.points.size() )
                             / std::max( 1, firstSegment.points.size() + secondSegment.points.size() )
                             + ( 1.0 - alignment ) * 0.45
                             + ( 1.0 - balance ) * 0.15
                             + ridgeHeightPenalty * 0.25
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
    if ( alignment < 0.35 )
      return result;

    const QgsPointXY preferredPoint( ( segment.start.x() + segment.end.x() ) * 0.5, ( segment.start.y() + segment.end.y() ) * 0.5 );
    const double denominator = lineA * lineA + lineB * lineB;
    QgsPointXY fittedPoint( preferredPoint.x() - lineA * ( lineA * preferredPoint.x() + lineB * preferredPoint.y() + lineC ) / denominator,
                            preferredPoint.y() - lineB * ( lineA * preferredPoint.x() + lineB * preferredPoint.y() + lineC ) / denominator );
    if ( !pointInRing( ring, fittedPoint ) )
      fittedPoint = pullPointInsideRing( ring, fittedPoint, preferredPoint );

    const double fittedAlong = lineParameter( segment.start, segment.dirX, segment.dirY, fittedPoint );
    const double clampedAlong = std::max( 0.0, std::min( segment.length, fittedAlong ) );
    const double crossOffset = signedDistanceToLine( fittedPoint, segment.ridgePoint, segment.normalX, segment.normalY );
    const double maxCrossOffset = std::max( 0.20, ringExtentSize( ring ) * 0.05 );
    const double clampedCrossOffset = std::max( -maxCrossOffset, std::min( maxCrossOffset, crossOffset ) );
    const QgsPointXY segmentPoint = pointOnLine( segment.start, segment.dirX, segment.dirY, clampedAlong );
    fittedPoint = QgsPointXY( segmentPoint.x() + segment.normalX * clampedCrossOffset,
                              segmentPoint.y() + segment.normalY * clampedCrossOffset );
    fittedPoint = pullPointInsideRing( ring, fittedPoint, preferredPoint );

    const double height = 0.5 * ( roofPlaneZ( firstPlane, fittedPoint ) + roofPlaneZ( secondPlane, fittedPoint ) );
    if ( !std::isfinite( height ) )
      return result;

    result.success = true;
    result.point = QgsPoint( fittedPoint.x(), fittedPoint.y(), height );
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

  bool leastSquaresBentGabledRidgePointFromPointCloud( const QVector<QgsPointXY> &inputRing, const QVector<BuildingRoof::RoofSample> &pointCloudSamples, const QgsPoint &boundaryPoint, QgsPoint &ridgePoint, double *ridgeDirX = nullptr, double *ridgeDirY = nullptr, double *boundaryHeightFromSlope = nullptr )
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

    const double weightedHeight = ( primaryRidge.point.z() * primaryRidge.supportCount + secondaryRidge.point.z() * secondaryRidge.supportCount )
                                  / std::max( 1, primaryRidge.supportCount + secondaryRidge.supportCount );
    if ( weightedHeight <= heightBand.baseHeight + std::max( 0.20, heightBand.binWidth ) )
      return false;

    ridgePoint = QgsPoint( primaryRidge.point.x(), primaryRidge.point.y(), weightedHeight );
    if ( ridgeDirX )
      *ridgeDirX = dirX;
    if ( ridgeDirY )
      *ridgeDirY = dirY;
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

  QList<BuildingRoof::RoofPoint> topHeightGabledRoofPoints( const QVector<QgsPointXY> &ring, const QList<BuildingRoof::RoofPoint> &boundaries, const QList<BuildingRoof::RoofPoint> &ridges, const QVector<BuildingRoof::RoofSample> &pointCloudSamples, GabledBoundarySlopePlanes *slopePlanesOut = nullptr, double *ridgeDirXOut = nullptr, double *ridgeDirYOut = nullptr )
  {
    QList<BuildingRoof::RoofPoint> points;
    if ( boundaries.size() != 1 || ( ridges.size() != 1 && ridges.size() != 3 ) )
      return points;

    QgsPoint ridgePoint;
    double ridgeDirX = 0.0;
    double ridgeDirY = 0.0;
    GabledBoundarySlopePlanes boundarySlopePlanes;
    double bentBoundaryHeightFromSlope = std::numeric_limits<double>::quiet_NaN();
    bool ridgePointFound = false;
    if ( ridges.size() == 3 )
    {
      ridgePointFound = leastSquaresBentGabledRidgePointFromPointCloud( ring, pointCloudSamples, boundaries.first().point, ridgePoint, &ridgeDirX, &ridgeDirY, &bentBoundaryHeightFromSlope );
      if ( !ridgePointFound )
        ridgePointFound = topHeightBentRidgePointFromPointCloud( ring, pointCloudSamples, boundaries.first().point, ridgePoint, &ridgeDirX, &ridgeDirY );
    }
    else
    {
      ridgePointFound = leastSquaresGabledRidgePointFromPointCloud( ring, pointCloudSamples, boundaries.first().point, ridgePoint, &ridgeDirX, &ridgeDirY, &boundarySlopePlanes );
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
    points.append( BuildingRoof::RoofPoint{ ridgePoint, ridges.first().type } );
    for ( int i = 1; i < ridges.size(); ++i )
      points.append( ridges.at( i ) );

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

    for ( int i = 0; i < roofPoints.size(); i += centerStride )
    {
      const QgsPoint &center = roofPoints.at( i );
      QVector<QgsPoint> neighbors;
      neighbors.reserve( 48 );
      for ( const QgsPoint &neighbor : roofPoints )
      {
        const double dx = neighbor.x() - center.x();
        const double dy = neighbor.y() - center.y();
        if ( dx * dx + dy * dy <= neighborRadius2 )
          neighbors.append( neighbor );
      }

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

  QList<BuildingRoof::RoofPoint> topHeightApexRoofPoints( const QVector<QgsPointXY> &ring, const QList<BuildingRoof::RoofPoint> &boundaries, const QList<BuildingRoof::RoofPoint> &vertices, const QVector<BuildingRoof::RoofSample> &pointCloudSamples )
  {
    QList<BuildingRoof::RoofPoint> points;
    if ( boundaries.size() != 1 || vertices.size() != 1 )
      return points;

    QgsPoint apexPoint = vertices.first().point;
    if ( !automaticApexPointFromPointCloud( ring, pointCloudSamples, vertices.first().point, apexPoint ) )
      return points;

    const QgsPointXY footprintCenter = ringCentroidPoint( ring );
    apexPoint.setX( footprintCenter.x() );
    apexPoint.setY( footprintCenter.y() );

    QgsPoint boundaryPoint = boundaries.first().point;
    double boundaryHeight = boundaryPoint.z();
    if ( automaticBoundaryHeightFromPerimeterBand( ring, pointCloudSamples, apexPoint, boundaryHeight ) )
      boundaryPoint.setZ( boundaryHeight );

    double fittedApexHeight = apexPoint.z();
    if ( fitApexHeightFromRoofSurface( ring, pointCloudSamples, footprintCenter, boundaryPoint.z(), fittedApexHeight ) )
      apexPoint.setZ( fittedApexHeight );

    if ( apexPoint.z() <= boundaryPoint.z() + 1e-6 )
      apexPoint.setZ( boundaryPoint.z() + std::max( 0.30, ringExtentSize( ring ) * 0.01 ) );

    points.append( BuildingRoof::RoofPoint{ boundaryPoint, boundaries.first().type } );
    points.append( BuildingRoof::RoofPoint{ apexPoint, vertices.first().type } );
    return points;
  }

  QList<BuildingRoof::RoofPoint> topHeightSingleSurfaceRoofPoints( const QVector<QgsPointXY> &ring, const QList<BuildingRoof::RoofPoint> &boundaries, const QList<BuildingRoof::RoofPoint> &surfaces, const QVector<BuildingRoof::RoofSample> &pointCloudSamples )
  {
    QList<BuildingRoof::RoofPoint> points;
    if ( boundaries.size() != 1 || surfaces.size() != 1 )
      return points;

    QgsPoint surfacePoint = surfaces.first().point;
    if ( !automaticApexPointFromPointCloud( ring, pointCloudSamples, surfaces.first().point, surfacePoint ) )
      return points;

    QgsPoint boundaryPoint = boundaries.first().point;
    double boundaryHeight = boundaryPoint.z();
    if ( automaticBoundaryHeightFromPerimeterBand( ring, pointCloudSamples, surfacePoint, boundaryHeight ) )
      boundaryPoint.setZ( boundaryHeight );

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
    return baseHeight + sphericalCapHeight( distance, supportRadius, rise );
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
    return baseHeight + sphericalCapHeight( distance, supportRadius, rise );
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
    centerS = 0.5 * ( profileDistance( a, normalX, normalY ) + profileDistance( b, normalX, normalY ) );

    minS = std::numeric_limits<double>::max();
    maxS = -std::numeric_limits<double>::max();
    for ( const QgsPointXY &ringPoint : ring )
    {
      const double s = profileDistance( ringPoint, normalX, normalY );
      minS = std::min( minS, s );
      maxS = std::max( maxS, s );
    }

    centerS = std::max( minS, std::min( maxS, centerS ) );
    return maxS - minS > 1e-8;
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

  double barrelRoofZ( const QgsPointXY &point, const QgsPoint &surfaceStart, const QgsPoint &surfaceEnd, const QVector<QgsPointXY> &ring, double baseHeight )
  {
    double normalX = 0.0;
    double normalY = 0.0;
    const QVector<ProfileAnchor> profileAnchors = barrelProfileAnchors( ring, surfaceStart, surfaceEnd, baseHeight, normalX, normalY );
    if ( profileAnchors.isEmpty() )
      return baseHeight;

    return profileRoofZ( profileAnchors, profileDistance( point, normalX, normalY ) );
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
    const int angularSegments = 96;
    const int radialSegments = 32;
    const QgsPointXY center( surfacePoint.x(), surfacePoint.y() );

    QVector<double> rayDistances;
    rayDistances.reserve( angularSegments );
    double supportRadius = 0.0;
    for ( int i = 0; i < angularSegments; ++i )
    {
      const double angle = twoPi * i / angularSegments;
      double distance = 0.0;
      if ( !rayDistanceToRingBoundary( ring, center, std::cos( angle ), std::sin( angle ), distance ) )
        distance = 0.0;
      rayDistances.append( distance );
      supportRadius = std::max( supportRadius, distance );
    }
    if ( supportRadius <= 1e-8 )
      return;

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
        const double angle = twoPi * i / angularSegments;
        const double distance = rayDistances.at( i ) * fraction;
        const QgsPointXY point( center.x() + std::cos( angle ) * distance, center.y() + std::sin( angle ) * distance );
        const double z = baseHeight + sphericalCapHeight( distance, supportRadius, rise );
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
  }

  void appendBarrelRoofSurface( BuildingRoof::Mesh &mesh, const QVector<QgsPointXY> &ring, const QgsPoint &surfaceStart, const QgsPoint &surfaceEnd, double baseHeight )
  {
    double normalX = 0.0;
    double normalY = 0.0;
    const QVector<ProfileAnchor> profileAnchors = barrelProfileAnchors( ring, surfaceStart, surfaceEnd, baseHeight, normalX, normalY );
    if ( profileAnchors.size() < 3 )
      return;

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
        mesh.vertices.append( QgsPoint( point.x(), point.y(), profileRoofZ( profileAnchors, profileDistance( point, normalX, normalY ) ) ) );

      const QVector<int> triangles = triangulateRing( strip );
      for ( int t = 0; t + 2 < triangles.size(); t += 3 )
        mesh.indices << offset + triangles[t] << offset + triangles[t + 1] << offset + triangles[t + 2];
    }
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

  void appendCurvedRoofSurface( BuildingRoof::Mesh &mesh, const QVector<QgsPointXY> &ring, const QList<BuildingRoof::RoofPoint> &surfaces, double baseHeight )
  {
    if ( ring.size() < 3 || surfaces.isEmpty() )
      return;

    if ( surfaces.size() == 1 )
      appendDomeRoofSurface( mesh, ring, surfaces.first().point, baseHeight );
    else
      appendBarrelRoofSurface( mesh, ring, surfaces.at( 0 ).point, surfaces.at( 1 ).point, baseHeight );
  }

  void appendCurvedRoofWall( BuildingRoof::Mesh &mesh, const QVector<QgsPointXY> &ring, const QList<BuildingRoof::RoofPoint> &surfaces, double baseHeight )
  {
    if ( ring.size() < 2 )
      return;

    const int count = ring.size();
    for ( int i = 0; i < count; ++i )
    {
      const QgsPointXY &a = ring.at( i );
      const QgsPointXY &b = ring.at( ( i + 1 ) % count );
      const double edgeLength = std::hypot( b.x() - a.x(), b.y() - a.y() );
      const double targetLength = std::max( ringExtentSize( ring ) / 80.0, 1e-8 );
      const int segments = std::max( 1, std::min( 32, static_cast<int>( std::ceil( edgeLength / targetLength ) ) ) );
      for ( int segment = 0; segment < segments; ++segment )
      {
        const double t0 = static_cast<double>( segment ) / segments;
        const double t1 = static_cast<double>( segment + 1 ) / segments;
        const QgsPointXY p0( a.x() + ( b.x() - a.x() ) * t0, a.y() + ( b.y() - a.y() ) * t0 );
        const QgsPointXY p1( a.x() + ( b.x() - a.x() ) * t1, a.y() + ( b.y() - a.y() ) * t1 );
        const double topA = curvedRoofZAt( p0, ring, surfaces, baseHeight );
        const double topB = curvedRoofZAt( p1, ring, surfaces, baseHeight );
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

  bool findTransitionChord( const QVector<QgsPointXY> &ring, int cornerIndex, const QgsPointXY &ridgeXY, double dirX, double dirY, const QVector<LineInterval> &primaryIntervals, QgsPointXY &bend, double &bendT, double &splitNormalX, double &splitNormalY )
  {
    const QgsPointXY &corner = ring[cornerIndex];
    const int count = ring.size();
    bool found = false;
    double bestScore = std::numeric_limits<double>::max();

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
      const double perpendicularScore = std::fabs( chordDirX * dirX + chordDirY * dirY );
      const double centeredScore = std::fabs( chordT - 0.5 );
      const double score = perpendicularScore + 0.05 * centeredScore;
      if ( score < bestScore )
      {
        bestScore = score;
        bend = candidateBend;
        bendT = candidateBendT;
        splitNormalX = -chordDirY;
        splitNormalY = chordDirX;
        found = true;
      }
    }

    return found;
  }

  void computeBentSegmentLimits( BentGableSegment &segment, const QVector<QgsPointXY> &ring, const QgsPoint &boundaryPoint )
  {
    const double boundaryDistance = signedDistanceToLine( QgsPointXY( boundaryPoint.x(), boundaryPoint.y() ), segment.ridgePoint, segment.normalX, segment.normalY );
    for ( const QgsPointXY &point : ring )
    {
      const double along = lineParameter( segment.start, segment.dirX, segment.dirY, point );
      if ( along < -1e-7 || along > segment.length + 1e-7 )
        continue;

      const double distance = signedDistanceToLine( point, segment.ridgePoint, segment.normalX, segment.normalY );
      if ( distance * boundaryDistance < 0.0 )
        segment.oppositeLimit = std::max( segment.oppositeLimit, std::fabs( distance ) );
      else
        segment.sameSideLimit = std::max( segment.sameSideLimit, std::fabs( distance ) );
    }

    if ( segment.sameSideLimit <= 1e-8 )
      segment.sameSideLimit = std::fabs( boundaryDistance );
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
    return found && bestScore < 0.95;
  }

  double bentGabledTopZ( const QgsPointXY &point, const QgsPoint &boundaryPoint, const QVector<BentGableSegment> &segments, double baseHeight )
  {
    double bestZ = baseHeight;
    bool hasSegment = false;
    for ( const BentGableSegment &segment : segments )
    {
      const double along = lineParameter( segment.start, segment.dirX, segment.dirY, point );
      if ( along < -1e-7 || along > segment.length + 1e-7 )
        continue;

      const double z = gabledTopZ( point, boundaryPoint, segment.ridgePoint, segment.normalX, segment.normalY, segment.sameSideLimit, segment.oppositeLimit, baseHeight );
      bestZ = hasSegment ? std::max( bestZ, z ) : z;
      hasSegment = true;
    }
    return hasSegment ? bestZ : baseHeight;
  }

  BentGableLayout findBentGabledLayout( const QVector<QgsPointXY> &inputRing, const QgsPointXY &ridgeXY, double dirX, double dirY )
  {
    BentGableLayout layout;
    if ( inputRing.size() < 5 )
      return layout;

    const QVector<LineInterval> primaryIntervals = lineInsideRingIntervals( inputRing, ridgeXY, dirX, dirY );
    if ( primaryIntervals.isEmpty() )
      return layout;

    bool found = false;
    double bestScore = std::numeric_limits<double>::max();
    for ( int pass = 0; pass < 2 && !found; ++pass )
    {
      bestScore = std::numeric_limits<double>::max();
      for ( int i = 0; i < inputRing.size(); ++i )
      {
        if ( pass == 0 && !isConcaveVertex( inputRing, i ) )
          continue;
        if ( pass == 1 && isConcaveVertex( inputRing, i ) )
          continue;

        QgsPointXY bend;
        double bendT = 0.0;
        double splitNormalX = 0.0;
        double splitNormalY = 0.0;
        if ( !findTransitionChord( inputRing, i, ridgeXY, dirX, dirY, primaryIntervals, bend, bendT, splitNormalX, splitNormalY ) )
          continue;

        LineInterval primaryInterval;
        if ( !findIntervalContaining( primaryIntervals, 0.0, bendT, primaryInterval ) )
          continue;

        double secondaryDirX = 0.0;
        double secondaryDirY = 0.0;
        if ( !bentSecondaryDirection( inputRing, i, dirX, dirY, secondaryDirX, secondaryDirY ) )
          continue;

        const QVector<LineInterval> secondaryIntervals = lineInsideRingIntervals( inputRing, bend, secondaryDirX, secondaryDirY );
        LineInterval secondaryInterval;
        if ( !findIntervalContaining( secondaryIntervals, 0.0, 0.0, secondaryInterval ) )
          continue;

        const double primaryEndT = chooseEndpointOnRidgeSide( primaryInterval, bendT, 0.0 );
        const double secondaryEndT = chooseFarthestEndpoint( secondaryInterval, 0.0 );
        const QgsPointXY primaryEnd = pointOnLine( ridgeXY, dirX, dirY, primaryEndT );
        const QgsPointXY secondaryEnd = pointOnLine( bend, secondaryDirX, secondaryDirY, secondaryEndT );

        if ( nearlySamePoint( primaryEnd, bend ) || nearlySamePoint( secondaryEnd, bend ) )
          continue;

        const double score = std::fabs( bendT );
        if ( score < bestScore )
        {
          bestScore = score;
          layout.primaryEnd = primaryEnd;
          layout.bend = bend;
          layout.secondaryEnd = secondaryEnd;
          layout.splitNormalX = splitNormalX;
          layout.splitNormalY = splitNormalY;
          layout.success = true;
          found = true;
        }
      }
    }

    return layout;
  }

  void appendTriangulatedBentRoofSurface( BuildingRoof::Mesh &mesh, const QVector<QgsPointXY> &polygon, const QgsPoint &boundaryPoint, const QVector<BentGableSegment> &segments, double baseHeight )
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
      mesh.vertices.append( QgsPoint( point.x(), point.y(), bentGabledTopZ( point, boundaryPoint, segments, baseHeight ) ) );

    const QVector<int> triangles = triangulateRing( localRing );
    for ( int i = 0; i + 2 < triangles.size(); i += 3 )
      mesh.indices << vertexOffset + triangles[i] << vertexOffset + triangles[i + 1] << vertexOffset + triangles[i + 2];
  }

  void appendBentSegmentRoofSurfaces( BuildingRoof::Mesh &mesh, const QgsGeometry &footprintGeometry, const QVector<QgsPointXY> &referenceRing, const BentGableSegment &segment, const QgsPoint &boundaryPoint, const QVector<BentGableSegment> &segments, double baseHeight )
  {
    const QgsPointXY ridgeXY( segment.ridgePoint.x(), segment.ridgePoint.y() );
    for ( bool keepPositive : { true, false } )
    {
      const QgsGeometry slopeGeometry = intersectWithHalfPlane( footprintGeometry, referenceRing, ridgeXY, segment.normalX, segment.normalY, keepPositive );
      for ( const QVector<QgsPointXY> &slopeRing : exteriorRingsFromGeometry( slopeGeometry ) )
        appendTriangulatedBentRoofSurface( mesh, slopeRing, boundaryPoint, segments, baseHeight );
    }
  }

  BuildingRoof::MeshResult buildBentGabledRoofPrismMesh( const QVector<QgsPointXY> &inputRing, const QgsPoint &boundaryPoint, const QgsPoint &ridgePoint, double dirX, double dirY )
  {
    BuildingRoof::MeshResult result;
    if ( inputRing.size() < 5 )
      return result;

    const QgsPointXY ridgeXY( ridgePoint.x(), ridgePoint.y() );
    const BentGableLayout layout = findBentGabledLayout( inputRing, ridgeXY, dirX, dirY );
    if ( !layout.success )
      return result;

    const double baseHeight = boundaryPoint.z();
    QVector<QgsPointXY> ring = ringWithInsertedBoundaryPoints( inputRing, QVector<QgsPointXY>{ layout.primaryEnd, layout.secondaryEnd } );

    QVector<BentGableSegment> segments;
    BentGableSegment primarySegment;
    BentGableSegment secondarySegment;
    if ( !makeBentSegment( layout.primaryEnd, layout.bend, ridgePoint.z(), ring, boundaryPoint, primarySegment ) )
      return result;
    if ( !makeBentSegment( layout.bend, layout.secondaryEnd, ridgePoint.z(), ring, boundaryPoint, secondarySegment ) )
      return result;
    segments << primarySegment << secondarySegment;

    const QgsPoint splitPoint( layout.bend.x(), layout.bend.y(), ridgePoint.z() );
    const double primarySide = signedDistanceToLine( layout.primaryEnd, splitPoint, layout.splitNormalX, layout.splitNormalY );
    if ( std::fabs( primarySide ) <= 1e-8 )
      return result;

    const QgsGeometry footprintGeometry = polygonGeometryFromRing( ring );
    const QgsPointXY splitXY( splitPoint.x(), splitPoint.y() );
    const QgsGeometry primaryFootprint = intersectWithHalfPlane( footprintGeometry, ring, splitXY, layout.splitNormalX, layout.splitNormalY, primarySide > 0.0 );
    const QgsGeometry secondaryFootprint = intersectWithHalfPlane( footprintGeometry, ring, splitXY, layout.splitNormalX, layout.splitNormalY, primarySide < 0.0 );
    if ( exteriorRingsFromGeometry( primaryFootprint ).isEmpty() || exteriorRingsFromGeometry( secondaryFootprint ).isEmpty() )
      return result;

    for ( const QgsPointXY &point : ring )
    {
      result.mesh.vertices.append( QgsPoint( point.x(), point.y(), 0.0 ) );
      result.mesh.vertices.append( QgsPoint( point.x(), point.y(), bentGabledTopZ( point, boundaryPoint, segments, baseHeight ) ) );
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

    appendBentSegmentRoofSurfaces( result.mesh, primaryFootprint, ring, primarySegment, boundaryPoint, segments, baseHeight );
    appendBentSegmentRoofSurfaces( result.mesh, secondaryFootprint, ring, secondarySegment, boundaryPoint, segments, baseHeight );

    result.success = !result.mesh.isEmpty();
    if ( !result.success )
      result.error = QStringLiteral( "Bent gabled roof mesh generation failed." );
    return result;
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

  void averageRidgeHeightsIfClose( QList<BuildingRoof::RoofPoint> &ridges )
  {
    if ( ridges.size() < 2 )
      return;

    double minZ = std::numeric_limits<double>::max();
    double maxZ = -std::numeric_limits<double>::max();
    double sumZ = 0.0;
    for ( const BuildingRoof::RoofPoint &ridge : ridges )
    {
      minZ = std::min( minZ, ridge.point.z() );
      maxZ = std::max( maxZ, ridge.point.z() );
      sumZ += ridge.point.z();
    }

    if ( maxZ - minZ > BuildingRoof::RIDGE_HEIGHT_AVERAGE_THRESHOLD )
      return;

    const double averageZ = sumZ / ridges.size();
    for ( BuildingRoof::RoofPoint &ridge : ridges )
      ridge.point.setZ( averageZ );
  }

  void averageMirroredRidgeProfileHeightsIfClose( QVector<ProfileAnchor> &ridgeAnchors )
  {
    std::sort( ridgeAnchors.begin(), ridgeAnchors.end(), []( const ProfileAnchor &lhs, const ProfileAnchor &rhs ) {
      return lhs.s < rhs.s;
    } );

    for ( int left = 0, right = ridgeAnchors.size() - 1; left < right; ++left, --right )
    {
      ProfileAnchor &leftAnchor = ridgeAnchors[left];
      ProfileAnchor &rightAnchor = ridgeAnchors[right];
      if ( std::fabs( leftAnchor.z - rightAnchor.z ) > BuildingRoof::RIDGE_HEIGHT_AVERAGE_THRESHOLD )
        continue;

      const double averageZ = 0.5 * ( leftAnchor.z + rightAnchor.z );
      leftAnchor.z = averageZ;
      rightAnchor.z = averageZ;
    }
  }
}

BuildingRoof::RoofPlaneSegmentation BuildingRoof::segmentRoofPlanesForDebug( const QgsGeometry &buildingGeometry, const QVector<RoofSample> &pointCloudSamples, bool preserveSmallEndPlanes )
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
    if ( !std::isfinite( point.z() ) || point.z() < 0.0 || !pointInRing( ring, QgsPointXY( point.x(), point.y() ) ) )
      continue;
    insidePoints.append( point );
    insideHeights.append( point.z() );
  }
  if ( insidePoints.size() < 24 )
    return result;

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

  QVector<QgsPoint> roofPoints;
  roofPoints.reserve( insidePoints.size() );
  for ( const QgsPoint &point : insidePoints )
  {
    if ( point.z() >= roofFloor && point.z() <= roofCeiling )
      roofPoints.append( point );
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
  const int minimumInliers = std::max( 12, static_cast<int>( std::ceil( roofPoints.size() * 0.035 ) ) );
  const double minimumAreaRatio = preserveSmallEndPlanes ? 0.015 : 0.06;

  QVector<QgsPoint> remainingPoints = roofPoints;
  for ( int segmentIndex = 0; segmentIndex < 8 && remainingPoints.size() >= minimumInliers; ++segmentIndex )
  {
    LeastSquaresRoofPlane plane;
    QVector<QgsPoint> planePoints;
    if ( !extractDebugRansacPlane( remainingPoints, minimumInliers, plane, planePoints ) )
      break;

    const QVector<QVector<QgsPoint>> connectedComponents = projectedPointComponents( planePoints, cellSize, bounds.xMinimum(), bounds.yMinimum() );
    if ( connectedComponents.isEmpty() )
      break;

    RoofPlaneSegment segment;
    segment.id = segmentIndex + 1;
    segment.points = connectedComponents.first();
    segment.projectedAreaRatio = std::min( 1.0, pointSetProjectedArea( segment.points, cellSize, bounds.xMinimum(), bounds.yMinimum() ) / footprintArea );
    segment.accepted = segment.projectedAreaRatio >= minimumAreaRatio;
    result.segments.append( segment );

    QVector<QgsPoint> disconnectedPoints;
    for ( int componentIndex = 1; componentIndex < connectedComponents.size(); ++componentIndex )
      disconnectedPoints += connectedComponents.at( componentIndex );
    if ( !disconnectedPoints.isEmpty() )
    {
      RoofPlaneSegment disconnectedSegment;
      disconnectedSegment.id = segment.id;
      disconnectedSegment.accepted = false;
      disconnectedSegment.points = disconnectedPoints;
      disconnectedSegment.projectedAreaRatio = std::min( 1.0, pointSetProjectedArea( disconnectedPoints, cellSize, bounds.xMinimum(), bounds.yMinimum() ) / footprintArea );
      result.segments.append( disconnectedSegment );
    }

    const double removalThreshold = robustPlaneThreshold( planePoints ) * 1.15;
    QVector<QgsPoint> nextPoints;
    nextPoints.reserve( remainingPoints.size() - planePoints.size() );
    for ( const QgsPoint &point : remainingPoints )
    {
      if ( roofPlaneResidual( plane, point ) > removalThreshold )
        nextPoints.append( point );
    }
    if ( nextPoints.size() >= remainingPoints.size() )
      break;
    remainingPoints = nextPoints;
  }

  result.unclassifiedPoints = remainingPoints;
  return result;
}

BuildingRoof::Result BuildingRoof::buildSingleSlopeRoof( const QgsGeometry &buildingGeometry, const QList<RoofPoint> &roofPoints )
{
  Result result;

  const QList<RoofPoint> boundary = boundaryPoints( roofPoints );
  if ( hasRidgePoint( roofPoints ) || boundary.size() != 2 )
  {
    result.error = QStringLiteral( "Single-slope roof requires exactly two boundary points and no ridge point." );
    return result;
  }

  const QgsPoint p1 = boundary.at( 0 ).point;
  const QgsPoint p2 = boundary.at( 1 ).point;
  if ( std::fabs( p1.z() - p2.z() ) <= 1e-6 )
  {
    result.error = QStringLiteral( "The two boundary points must have different heights." );
    return result;
  }

  const QgsPolygonXY polygon = firstPolygon( buildingGeometry );
  const QVector<QgsPointXY> ring = exteriorRing( polygon );
  if ( ring.size() < 3 )
  {
    result.error = QStringLiteral( "The building footprint cannot be used for roof generation." );
    return result;
  }

  const QgsPoint lowPoint = p1.z() <= p2.z() ? p1 : p2;
  const QgsPoint highPoint = p1.z() <= p2.z() ? p2 : p1;
  result.geometry = roofSurfaceGeometry( ring, lowPoint, highPoint, lowPoint.z() );
  result.success = !result.geometry.isNull() && !result.geometry.isEmpty();
  if ( !result.success )
    result.error = QStringLiteral( "Single-slope roof surface generation failed." );
  return result;
}

BuildingRoof::MeshResult BuildingRoof::buildSingleSlopePrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints )
{
  Q_UNUSED( buildingHeight )
  MeshResult result;

  const QList<RoofPoint> boundary = boundaryPoints( roofPoints );
  if ( hasRidgePoint( roofPoints ) )
  {
    result.error = QStringLiteral( "Single-slope roof requires no ridge point." );
    return result;
  }
  if ( boundary.size() != 2 )
  {
    result.error = QStringLiteral( "Single-slope roof requires exactly two boundary points." );
    return result;
  }

  const QgsPoint p1 = boundary.at( 0 ).point;
  const QgsPoint p2 = boundary.at( 1 ).point;
  if ( std::fabs( p1.z() - p2.z() ) <= 1e-6 )
  {
    result.error = QStringLiteral( "The two boundary points must have different heights." );
    return result;
  }
  if ( std::hypot( p2.x() - p1.x(), p2.y() - p1.y() ) <= 1e-6 )
  {
    result.error = QStringLiteral( "The two boundary points must have different positions." );
    return result;
  }

  const QgsPolygonXY polygon = firstPolygon( buildingGeometry );
  const QVector<QgsPointXY> ring = exteriorRing( polygon );
  if ( ring.size() < 3 )
  {
    result.error = QStringLiteral( "The building footprint cannot be used for roof mesh generation." );
    return result;
  }

  const QgsPoint lowPoint = p1.z() <= p2.z() ? p1 : p2;
  const QgsPoint highPoint = p1.z() <= p2.z() ? p2 : p1;
  const double baseHeight = lowPoint.z();

  for ( const QgsPointXY &point : ring )
  {
    result.mesh.vertices.append( QgsPoint( point.x(), point.y(), 0.0 ) );
    result.mesh.vertices.append( QgsPoint( point.x(), point.y(), pointOnSlopeZ( point, lowPoint, highPoint, baseHeight ) ) );
  }

  const int count = ring.size();
  for ( int i = 0; i < count; ++i )
  {
    const int next = ( i + 1 ) % count;
    result.mesh.indices << 2 * i << 2 * next << 2 * i + 1;
    result.mesh.indices << 2 * i + 1 << 2 * next << 2 * next + 1;
  }

  const QVector<int> triangles = triangulateRing( ring );
  for ( int i = 0; i + 2 < triangles.size(); i += 3 )
  {
    result.mesh.indices << 2 * triangles[i] + 1 << 2 * triangles[i + 1] + 1 << 2 * triangles[i + 2] + 1;
    result.mesh.indices << 2 * triangles[i] << 2 * triangles[i + 2] << 2 * triangles[i + 1];
  }

  result.success = !result.mesh.isEmpty();
  if ( !result.success )
    result.error = QStringLiteral( "Single-slope roof mesh generation failed." );
  return result;
}

BuildingRoof::MeshResult BuildingRoof::buildFlatReliefPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints, const QVector<RoofSample> &pointCloudSamples )
{
  Q_UNUSED( buildingHeight )
  MeshResult result;

  const QList<RoofPoint> boundary = boundaryPoints( roofPoints );
  if ( hasRidgePoint( roofPoints ) )
  {
    result.error = QStringLiteral( "Flat relief roof requires no ridge point." );
    return result;
  }
  if ( boundary.size() != 2 )
  {
    result.error = QStringLiteral( "Flat relief roof requires exactly two boundary points." );
    return result;
  }

  const QgsPoint p1 = boundary.at( 0 ).point;
  const QgsPoint p2 = boundary.at( 1 ).point;
  if ( std::hypot( p2.x() - p1.x(), p2.y() - p1.y() ) <= 1e-6 )
  {
    result.error = QStringLiteral( "The two flat relief points must have different positions." );
    return result;
  }
  if ( std::fabs( p1.z() - p2.z() ) <= 1e-6 )
  {
    result.error = QStringLiteral( "The two flat relief points must have different heights." );
    return result;
  }

  const QgsPolygonXY polygon = firstPolygon( buildingGeometry );
  const QVector<QgsPointXY> ring = exteriorRing( polygon );
  if ( ring.size() < 3 )
  {
    result.error = QStringLiteral( "The building footprint cannot be used for flat relief roof mesh generation." );
    return result;
  }

  const QgsPointXY p1xy( p1.x(), p1.y() );
  const QgsPointXY p2xy( p2.x(), p2.y() );
  const double d1 = distanceToRing2( ring, p1xy );
  const double d2 = distanceToRing2( ring, p2xy );
  const QgsPoint mainPoint = d1 <= d2 ? p1 : p2;
  const QgsPoint reliefPoint = d1 <= d2 ? p2 : p1;
  const double mainHeight = mainPoint.z();
  const double reliefHeight = reliefPoint.z();

  double axisX = 0.0;
  double axisY = 0.0;
  if ( !nearestEdgeDirection( ring, mainPoint, axisX, axisY ) )
  {
    axisX = p2.x() - p1.x();
    axisY = p2.y() - p1.y();
    const double length = std::hypot( axisX, axisY );
    if ( length <= 1e-8 )
    {
      result.error = QStringLiteral( "Cannot infer flat relief direction." );
      return result;
    }
    axisX /= length;
    axisY /= length;
  }
  const double normalX = -axisY;
  const double normalY = axisX;

  const double extent = ringExtentSize( ring );
  QVector<ClusterSample> samples;
  const double heightDifference = std::fabs( reliefHeight - mainHeight );
  const bool isRaisedRelief = reliefHeight > mainHeight;
  const double clusterHeight = isRaisedRelief ? reliefHeight : mainHeight;
  const double otherHeight = isRaisedRelief ? mainHeight : reliefHeight;
  const double preferredTolerance = std::max( 0.05, heightDifference * 0.20 );
  const double clusterTolerance = separatedHeightTolerance( heightDifference, preferredTolerance );
  const double otherTolerance = separatedHeightTolerance( heightDifference, preferredTolerance );
  for ( const RoofSample &sample : pointCloudSamples )
  {
    const QgsPoint &point = sample.point;
    const QgsPointXY samplePoint( point.x(), point.y() );
    if ( !pointInRing( ring, samplePoint ) )
      continue;

    if ( std::fabs( point.z() - clusterHeight ) > clusterTolerance )
      continue;
    if ( std::fabs( point.z() - otherHeight ) <= otherTolerance )
      continue;

    const double u = point.x() * axisX + point.y() * axisY;
    const double v = point.x() * normalX + point.y() * normalY;
    samples.append( ClusterSample{ samplePoint, u, v, -1, false } );
  }

  if ( samples.size() < 3 )
  {
    result.error = QStringLiteral( "DBSCAN could not find enough point-cloud samples near the flat relief height." );
    return result;
  }

  const double eps = estimateDbscanEps( samples, std::max( extent * 0.03, 0.5 ) );
  const int clusterCount = assignDbscanClusters( samples, eps, 3 );
  const double raisedPadding = eps * 0.20;
  const double concavePadding = eps * 0.50;
  QVector<QgsPointXY> reliefRing = isRaisedRelief
                                     ? largestClusterBox( samples, clusterCount, axisX, axisY, normalX, normalY, raisedPadding )
                                     : concaveReliefBoxFromHighCluster( samples, clusterCount, QgsPointXY( reliefPoint.x(), reliefPoint.y() ), axisX, axisY, normalX, normalY, concavePadding );
  if ( reliefRing.size() < 3 )
  {
    result.error = QStringLiteral( "DBSCAN could not form a flat relief cluster." );
    return result;
  }

  const QgsGeometry footprintGeometry = polygonGeometryFromRing( ring );
  reliefRing = regularizeReliefRingToFootprint( reliefRing, ring, axisX, axisY, normalX, normalY, std::max( eps * 0.35, extent * 0.01 ) );
  QgsGeometry reliefGeometry = polygonGeometryFromRing( reliefRing ).intersection( footprintGeometry );
  if ( reliefGeometry.isNull() || reliefGeometry.isEmpty() )
  {
    result.error = QStringLiteral( "Flat relief cluster does not intersect the footprint." );
    return result;
  }

  const QgsGeometry mainRoofGeometry = isRaisedRelief ? footprintGeometry : footprintGeometry.difference( reliefGeometry );
  appendVerticalWall( result.mesh, ring, 0.0, mainHeight );
  appendHorizontalRingSurface( result.mesh, ring, 0.0, true );
  appendHorizontalGeometrySurface( result.mesh, mainRoofGeometry, mainHeight );
  appendHorizontalGeometrySurface( result.mesh, reliefGeometry, reliefHeight );

  const double lowerZ = std::min( mainHeight, reliefHeight );
  const double upperZ = std::max( mainHeight, reliefHeight );
  for ( const QVector<QgsPointXY> &reliefPart : exteriorRingsFromGeometry( reliefGeometry ) )
    appendVerticalWall( result.mesh, reliefPart, lowerZ, upperZ );

  result.success = !result.mesh.isEmpty();
  if ( !result.success )
    result.error = QStringLiteral( "Flat relief roof mesh generation failed." );
  return result;
}

BuildingRoof::MeshResult BuildingRoof::buildClusteredFlatTopHippedRoofPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints, const QVector<RoofSample> &pointCloudSamples )
{
  Q_UNUSED( buildingHeight )
  MeshResult result;

  const QList<RoofPoint> boundary = boundaryPoints( roofPoints );
  const QList<RoofPoint> ridges = ridgePoints( roofPoints );
  if ( boundary.size() != 2 || ridges.size() != 1 )
  {
    result.error = QStringLiteral( "Clustered flat-top hipped roof requires exactly two boundary points and one ridge point." );
    return result;
  }

  const QgsPoint basePoint = boundary.at( 0 ).point;
  const QgsPoint topPoint = boundary.at( 1 ).point;
  const QgsPoint clusterPoint = ridges.first().point;
  const double baseHeight = basePoint.z();
  const double topHeight = topPoint.z();
  if ( topHeight <= baseHeight + 1e-6 )
  {
    result.error = QStringLiteral( "The second boundary point must be higher than the first boundary point." );
    return result;
  }

  const QgsPolygonXY polygon = firstPolygon( buildingGeometry );
  const QVector<QgsPointXY> ring = exteriorRing( polygon );
  if ( ring.size() < 3 )
  {
    result.error = QStringLiteral( "The building footprint cannot be used for clustered flat-top hipped roof mesh generation." );
    return result;
  }

  double axisX = 0.0;
  double axisY = 0.0;
  if ( !nearestEdgeDirection( ring, basePoint, axisX, axisY ) )
  {
    axisX = topPoint.x() - basePoint.x();
    axisY = topPoint.y() - basePoint.y();
    const double length = std::hypot( axisX, axisY );
    if ( length <= 1e-8 )
    {
      result.error = QStringLiteral( "Cannot infer clustered flat-top hipped roof direction." );
      return result;
    }
    axisX /= length;
    axisY /= length;
  }
  const double normalX = -axisY;
  const double normalY = axisX;

  const double extent = ringExtentSize( ring );
  const double heightDifference = std::fabs( topHeight - baseHeight );
  const double topTolerance = std::max( 0.10, heightDifference * 0.20 );
  const double baseTolerance = std::max( 0.10, heightDifference * 0.20 );
  QVector<ClusterSample> samples;
  for ( const RoofSample &sample : pointCloudSamples )
  {
    const QgsPoint &point = sample.point;
    const QgsPointXY samplePoint( point.x(), point.y() );
    if ( !pointInRing( ring, samplePoint ) )
      continue;

    if ( std::fabs( point.z() - topHeight ) > topTolerance )
      continue;
    if ( std::fabs( point.z() - baseHeight ) <= baseTolerance )
      continue;

    const double u = point.x() * axisX + point.y() * axisY;
    const double v = point.x() * normalX + point.y() * normalY;
    samples.append( ClusterSample{ samplePoint, u, v, -1, false } );
  }

  if ( samples.size() < 3 )
  {
    result.error = QStringLiteral( "DBSCAN could not find enough point-cloud samples near the upper flat-top height." );
    return result;
  }

  const double eps = estimateDbscanEps( samples, std::max( extent * 0.03, 0.5 ) );
  const int clusterCount = assignDbscanClusters( samples, eps, 3 );
  QVector<QgsPointXY> topRing = clusterBoxNearPoint( samples, clusterCount, QgsPointXY( clusterPoint.x(), clusterPoint.y() ), axisX, axisY, normalX, normalY, eps * 0.5, extent * 2.0 );
  if ( topRing.size() < 3 )
    topRing = clusterBoxNearPoint( samples, clusterCount, QgsPointXY( topPoint.x(), topPoint.y() ), axisX, axisY, normalX, normalY, eps * 0.5, extent * 2.0 );
  if ( topRing.size() < 3 )
  {
    result.error = QStringLiteral( "DBSCAN could not form an upper flat-top cluster." );
    return result;
  }

  const QgsGeometry footprintGeometry = polygonGeometryFromRing( ring );
  topRing = regularizeReliefRingToFootprint( topRing, ring, axisX, axisY, normalX, normalY, std::max( eps * 0.35, extent * 0.01 ) );
  topRing = regularizeRingAsFootprintInset( topRing, ring );
  QgsGeometry topGeometry = polygonGeometryFromRing( topRing ).intersection( footprintGeometry );
  if ( topGeometry.isNull() || topGeometry.isEmpty() )
  {
    result.error = QStringLiteral( "Upper flat-top cluster does not intersect the footprint." );
    return result;
  }

  QgsGeometry slopeGeometry = footprintGeometry.difference( topGeometry );
  if ( slopeGeometry.isNull() || slopeGeometry.isEmpty() )
  {
    result.error = QStringLiteral( "Upper flat-top cluster covers the full footprint." );
    return result;
  }

  const QVector<QVector<QgsPointXY>> topRings = exteriorRingsFromGeometry( topGeometry );
  if ( topRings.isEmpty() )
  {
    result.error = QStringLiteral( "Upper flat-top cluster has no usable boundary." );
    return result;
  }

  appendVerticalWall( result.mesh, ring, 0.0, baseHeight );
  appendHorizontalRingSurface( result.mesh, ring, 0.0, true );
  appendSlopedFlatTopHippedSurface( result.mesh, slopeGeometry, ring, topRings, baseHeight, topHeight );
  appendHorizontalGeometrySurface( result.mesh, topGeometry, topHeight );

  result.success = !result.mesh.isEmpty();
  if ( !result.success )
    result.error = QStringLiteral( "Clustered flat-top hipped roof mesh generation failed." );
  return result;
}

static BuildingRoof::MeshResult buildCurvedRoofPrismMeshFromKeypoints( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<BuildingRoof::RoofPoint> &roofPoints )
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

  appendCurvedRoofWall( result.mesh, ring, surfaces, boundaryPoint.z() );
  appendHorizontalRingSurface( result.mesh, ring, 0.0, true );
  appendCurvedRoofSurface( result.mesh, ring, surfaces, boundaryPoint.z() );
  if ( surfaces.size() == 1 )
    appendApexStructureLines( result.mesh, ring, boundaryPoint.z(), surfaces.first().point, 12 );

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
  const QList<RoofPoint> boundaries = boundaryPoints( roofPoints );
  const QList<RoofPoint> ridges = ridgePoints( roofPoints );
  const QList<RoofPoint> vertices = vertexPoints( roofPoints );
  const QList<RoofPoint> surfaces = surfacePoints( roofPoints );

  const QgsPolygonXY polygon = firstPolygon( buildingGeometry );
  const QVector<QgsPointXY> ring = exteriorRing( polygon );
  if ( !pointCloudSamples.isEmpty() && ring.size() >= 3 && boundaries.size() == 1 && ridges.isEmpty() && vertices.isEmpty() && surfaces.size() == 1 )
  {
    const QList<RoofPoint> automaticPoints = topHeightSingleSurfaceRoofPoints( ring, boundaries, surfaces, pointCloudSamples );
    if ( !automaticPoints.isEmpty() )
    {
      MeshResult automaticResult = buildCurvedRoofPrismMeshFromKeypoints( buildingGeometry, buildingHeight, automaticPoints );
      if ( automaticResult.success )
        return automaticResult;
    }
  }

  return buildCurvedRoofPrismMeshFromKeypoints( buildingGeometry, buildingHeight, roofPoints );
}

BuildingRoof::MeshResult BuildingRoof::buildApexRoofPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints, const QVector<RoofSample> &pointCloudSamples )
{
  const QList<RoofPoint> boundaries = boundaryPoints( roofPoints );
  const QList<RoofPoint> ridges = ridgePoints( roofPoints );
  const QList<RoofPoint> vertices = vertexPoints( roofPoints );

  const QgsPolygonXY polygon = firstPolygon( buildingGeometry );
  const QVector<QgsPointXY> ring = exteriorRing( polygon );
  if ( !pointCloudSamples.isEmpty() && ring.size() >= 3 && boundaries.size() == 1 && ridges.isEmpty() && vertices.size() == 1 )
  {
    const QList<RoofPoint> automaticPoints = topHeightApexRoofPoints( ring, boundaries, vertices, pointCloudSamples );
    if ( !automaticPoints.isEmpty() )
    {
      MeshResult automaticResult = buildApexRoofPrismMeshFromKeypoints( buildingGeometry, buildingHeight, automaticPoints );
      if ( automaticResult.success )
        return automaticResult;
    }
  }

  return buildApexRoofPrismMeshFromKeypoints( buildingGeometry, buildingHeight, roofPoints );
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

  double dirX = 0.0;
  double dirY = 0.0;
  if ( !nearestEdgeDirection( ring, boundaryPoint, dirX, dirY ) )
  {
    result.error = QStringLiteral( "Cannot infer ridge direction from the boundary point." );
    return result;
  }

  if ( ridges.size() == 3 )
  {
    const RoofPoint ridge = ridges.first();
    if ( ridge.point.z() <= boundaryPoint.z() + 1e-6 )
    {
      result.error = QStringLiteral( "The bent gabled ridge point must be higher than the boundary point." );
      return result;
    }

    const MeshResult bentResult = buildBentGabledRoofPrismMesh( ring, boundaryPoint, ridge.point, dirX, dirY );
    if ( bentResult.success )
      return bentResult;

    result.error = QStringLiteral( "Bent gabled roof mesh generation failed." );
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
  appendTriangulatedLeastSquaresRoofSurface( result.mesh, positiveRoofRing, *positivePlane, ridgePoint, normalX, normalY, fallbackZ );
  appendTriangulatedLeastSquaresRoofSurface( result.mesh, negativeRoofRing, *negativePlane, ridgePoint, normalX, normalY, fallbackZ );

  result.success = !result.mesh.isEmpty();
  if ( !result.success )
    result.error = QStringLiteral( "Least-squares clipped gabled roof mesh generation failed." );
  return result;
}

BuildingRoof::MeshResult BuildingRoof::buildGabledRoofPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints, const QVector<RoofSample> &pointCloudSamples )
{
  const QList<RoofPoint> boundary = boundaryPoints( roofPoints );
  const QList<RoofPoint> ridges = ridgePoints( roofPoints );
  if ( !pointCloudSamples.isEmpty() && boundary.size() == 1 && ( ridges.size() == 1 || ridges.size() == 3 ) )
  {
    const QgsPolygonXY polygon = firstPolygon( buildingGeometry );
    const QVector<QgsPointXY> ring = exteriorRing( polygon );
    if ( ring.size() >= 3 )
    {
      GabledBoundarySlopePlanes automaticSlopePlanes;
      double automaticRidgeDirX = 0.0;
      double automaticRidgeDirY = 0.0;
      const QList<RoofPoint> automaticRoofPoints = topHeightGabledRoofPoints( ring, boundary, ridges, pointCloudSamples, &automaticSlopePlanes, &automaticRidgeDirX, &automaticRidgeDirY );
      if ( !automaticRoofPoints.isEmpty() )
      {
        if ( ridges.size() == 1 && automaticSlopePlanes.success && automaticSlopePlanes.hasSideSigns )
        {
          const MeshResult leastSquaresClippedResult = buildLeastSquaresClippedGabledRoofPrismMesh( buildingGeometry, automaticRoofPoints, automaticSlopePlanes, automaticRidgeDirX, automaticRidgeDirY );
          if ( leastSquaresClippedResult.success )
            return leastSquaresClippedResult;
        }

        const MeshResult automaticResult = buildGabledRoofPrismMeshFromKeypoints( buildingGeometry, buildingHeight, automaticRoofPoints );
        if ( automaticResult.success )
          return automaticResult;
      }
    }
  }

  return buildGabledRoofPrismMeshFromKeypoints( buildingGeometry, buildingHeight, roofPoints );
}

BuildingRoof::MeshResult BuildingRoof::buildMultiRidgePrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints )
{
  Q_UNUSED( buildingHeight )

  const QList<RoofPoint> boundaries = boundaryPoints( roofPoints );
  const QList<RoofPoint> ridges = ridgePoints( roofPoints );
  if ( boundaries.isEmpty() || ridges.size() < 2 )
  {
    MeshResult result;
    result.error = QStringLiteral( "Multi-ridge roof requires at least one boundary point and at least two ridge points." );
    return result;
  }

  if ( boundaries.size() == 1 && ridges.size() == 2 )
  {
    MeshResult result;
    result.error = QStringLiteral( "One boundary point and two ridge points are reserved for hipped roofs." );
    return result;
  }

  if ( boundaries.size() == 1 && ridges.size() == 3 )
  {
    MeshResult result;
    result.error = QStringLiteral( "One boundary point and three ridge points are reserved for bent gabled roofs." );
    return result;
  }

  const QgsPolygonXY polygon = firstPolygon( buildingGeometry );
  QVector<QgsPointXY> ring = exteriorRing( polygon );
  if ( ring.size() < 3 )
  {
    MeshResult result;
    result.error = QStringLiteral( "Multi-ridge roof footprint is invalid." );
    return result;
  }

  double dirX = 0.0;
  double dirY = 0.0;
  if ( !nearestEdgeDirection( ring, boundaries.first().point, dirX, dirY ) )
  {
    MeshResult result;
    result.error = QStringLiteral( "Cannot infer multi-ridge direction from the boundary point." );
    return result;
  }

  const double normalX = -dirY;
  const double normalY = dirX;
  double minS = std::numeric_limits<double>::max();
  double maxS = -std::numeric_limits<double>::max();
  for ( const QgsPointXY &point : ring )
  {
    const double s = profileDistance( point, normalX, normalY );
    minS = std::min( minS, s );
    maxS = std::max( maxS, s );
  }

  QVector<ProfileAnchor> profileAnchors;
  QVector<ProfileAnchor> ridgeProfileAnchors;
  const double defaultEaveZ = boundaries.first().point.z();
  appendProfileAnchor( profileAnchors, minS, defaultEaveZ );
  appendProfileAnchor( profileAnchors, maxS, defaultEaveZ );
  for ( const RoofPoint &ridge : ridges )
    appendProfileAnchor( ridgeProfileAnchors, profileDistance( QgsPointXY( ridge.point.x(), ridge.point.y() ), normalX, normalY ), ridge.point.z() );

  averageMirroredRidgeProfileHeightsIfClose( ridgeProfileAnchors );
  for ( const ProfileAnchor &ridgeAnchor : ridgeProfileAnchors )
    appendProfileAnchor( profileAnchors, ridgeAnchor.s, ridgeAnchor.z );

  std::sort( profileAnchors.begin(), profileAnchors.end(), []( const ProfileAnchor &lhs, const ProfileAnchor &rhs ) {
    return lhs.s < rhs.s;
  } );
  if ( profileAnchors.size() < 3 )
  {
    MeshResult result;
    result.error = QStringLiteral( "Multi-ridge roof needs at least one interior ridge profile." );
    return result;
  }

  QVector<double> profileDistances;
  profileDistances.reserve( profileAnchors.size() );
  for ( const ProfileAnchor &anchor : profileAnchors )
    profileDistances.append( anchor.s );

  ring = ringWithProfileIntersections( ring, normalX, normalY, profileDistances );
  MeshResult result;
  for ( const QgsPointXY &point : ring )
  {
    const double z = profileRoofZ( profileAnchors, profileDistance( point, normalX, normalY ) );
    result.mesh.vertices.append( QgsPoint( point.x(), point.y(), 0.0 ) );
    result.mesh.vertices.append( QgsPoint( point.x(), point.y(), z ) );
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

  for ( int i = 0; i + 1 < profileAnchors.size(); ++i )
  {
    const double a = profileAnchors.at( i ).s;
    const double b = profileAnchors.at( i + 1 ).s;
    if ( b - a <= 1e-8 )
      continue;

    QVector<QgsPointXY> strip = clipPolygonByProfileRange( ring, normalX, normalY, a, b );
    if ( strip.size() < 3 )
      continue;

    const int offset = result.mesh.vertices.size();
    for ( const QgsPointXY &point : strip )
    {
      const double z = profileRoofZ( profileAnchors, profileDistance( point, normalX, normalY ) );
      result.mesh.vertices.append( QgsPoint( point.x(), point.y(), z ) );
    }

    const QVector<int> triangles = triangulateRing( strip );
    for ( int t = 0; t + 2 < triangles.size(); t += 3 )
      result.mesh.indices << offset + triangles[t] << offset + triangles[t + 1] << offset + triangles[t + 2];
  }

  result.success = !result.mesh.isEmpty();
  if ( !result.success )
    result.error = QStringLiteral( "Multi-ridge roof mesh generation failed." );
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
  const QList<RoofPoint> boundaries = boundaryPoints( roofPoints );
  const QList<RoofPoint> ridges = ridgePoints( roofPoints );
  if ( !pointCloudSamples.isEmpty() && boundaries.size() == 1 && ridges.size() == 2 )
  {
    const QgsPolygonXY polygon = firstPolygon( buildingGeometry );
    const QVector<QgsPointXY> ring = exteriorRing( polygon );
    if ( ring.size() >= 3 )
    {
      const QList<RoofPoint> automaticRoofPoints = topHeightHippedRoofPoints( ring, boundaries, ridges, pointCloudSamples );
      if ( !automaticRoofPoints.isEmpty() )
      {
        const MeshResult automaticResult = buildHippedRoofPrismMeshFromKeypoints( buildingGeometry, buildingHeight, automaticRoofPoints );
        if ( automaticResult.success )
          return automaticResult;
      }
    }
  }

  return buildHippedRoofPrismMeshFromKeypoints( buildingGeometry, buildingHeight, roofPoints );
}
