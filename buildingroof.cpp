#include "buildingroof.h"

#include <qgsgeometrycollection.h>
#include <qgslinestring.h>
#include <qgspolygon.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

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

    const double neighborRadius = std::max( 0.55, extent * 0.025 );
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
    const double edgeBandDistance = std::max( 0.50, extent * 0.03 );
    const double edgeBandDistance2 = edgeBandDistance * edgeBandDistance;

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
    const double highClip = sortedPercentile( allHeights, 0.995 );
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
      edgeHeights.reserve( pointCloudSamples.size() );
      edgePoints.reserve( pointCloudSamples.size() );
      for ( const BuildingRoof::RoofSample &sample : pointCloudSamples )
      {
        const QgsPoint &point = sample.point;
        const QgsPointXY pointXY( point.x(), point.y() );
        if ( !pointInRing( ring, pointXY ) )
          continue;
        if ( pointSegmentDistance2( pointXY, a, b ) > edgeBandDistance2 )
          continue;
        if ( point.z() < lowClip || point.z() > highClip )
          continue;
        edgeHeights.append( point.z() );
        edgePoints.append( point );
      }

      double edgeZ = globalEaveHeight;
      double supportScore = 0.10;
      double wallSupportScore = 0.0;
      const double edgeNormalX = -edgeDirY;
      const double edgeNormalY = edgeDirX;
      const QVector<double> wallHeights = verticalWallLikeHeightsNearEdge( edgePoints, edgeDirX, edgeDirY, edgeNormalX, edgeNormalY, extent );
      if ( wallHeights.size() >= 5 )
      {
        edgeZ = sortedPercentile( wallHeights, 0.85 );
        wallSupportScore = std::min( 1.0, static_cast<double>( wallHeights.size() ) / std::max( 6.0, edgeLength / std::max( edgeBandDistance, 1e-8 ) ) );
        supportScore = std::max( supportScore, wallSupportScore );
      }
      if ( edgeHeights.size() >= 5 )
      {
        std::sort( edgeHeights.begin(), edgeHeights.end() );
        if ( wallHeights.size() < 5 )
          edgeZ = sortedPercentile( edgeHeights, 0.25 );
        supportScore = std::min( 1.0, static_cast<double>( edgeHeights.size() ) / std::max( 8.0, edgeLength / std::max( edgeBandDistance, 1e-8 ) * 2.0 ) );
        if ( wallHeights.size() >= 5 )
          supportScore = std::max( supportScore, wallSupportScore );
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
        }
      }
      bestBoundaryZ = globalEaveHeight;
    }

    if ( bestEdge < 0 )
      return false;

    const QgsPointXY &edgeStart = ring.at( bestEdge );
    const QgsPointXY &edgeEnd = ring.at( ( bestEdge + 1 ) % ring.size() );
    const QgsPointXY boundaryXY( ( edgeStart.x() + edgeEnd.x() ) * 0.5, ( edgeStart.y() + edgeEnd.y() ) * 0.5 );
    double boundaryZ = bestBoundaryZ;

    if ( boundaryZ >= ridgePoint.z() - 1e-6 )
      boundaryZ = ridgePoint.z() - std::max( 0.30, ringExtentSize( ring ) * 0.01 );

    boundaryPoint = QgsPoint( boundaryXY.x(), boundaryXY.y(), boundaryZ );
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

  QList<BuildingRoof::RoofPoint> topHeightGabledRoofPoints( const QVector<QgsPointXY> &ring, const QList<BuildingRoof::RoofPoint> &boundaries, const QList<BuildingRoof::RoofPoint> &ridges, const QVector<BuildingRoof::RoofSample> &pointCloudSamples )
  {
    QList<BuildingRoof::RoofPoint> points;
    if ( boundaries.size() != 1 || ( ridges.size() != 1 && ridges.size() != 3 ) )
      return points;

    QgsPoint ridgePoint;
    double ridgeDirX = 0.0;
    double ridgeDirY = 0.0;
    const bool ridgePointFound = ridges.size() == 3
                                   ? topHeightBentRidgePointFromPointCloud( ring, pointCloudSamples, boundaries.first().point, ridgePoint, &ridgeDirX, &ridgeDirY )
                                   : topHeightRidgePointFromPointCloud( ring, pointCloudSamples, boundaries.first().point, ridgePoint, &ridgeDirX, &ridgeDirY );
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
    points.append( BuildingRoof::RoofPoint{ boundaryPoint, boundaries.first().type } );
    points.append( BuildingRoof::RoofPoint{ ridgePoint, ridges.first().type } );
    for ( int i = 1; i < ridges.size(); ++i )
      points.append( ridges.at( i ) );
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
    const QVector<LineInterval> primaryIntervals = lineInsideRingIntervals( inputRing, ridgeXY, dirX, dirY );
    if ( primaryIntervals.isEmpty() )
      return result;

    bool found = false;
    double bestScore = std::numeric_limits<double>::max();
    QgsPointXY bestPrimaryEnd;
    QgsPointXY bestBend;
    QgsPointXY bestSecondaryEnd;
    double bestSplitNormalX = 0.0;
    double bestSplitNormalY = 0.0;

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
          bestPrimaryEnd = primaryEnd;
          bestBend = bend;
          bestSecondaryEnd = secondaryEnd;
          bestSplitNormalX = splitNormalX;
          bestSplitNormalY = splitNormalY;
          found = true;
        }
      }
    }

    if ( !found )
      return result;

    const double baseHeight = boundaryPoint.z();
    QVector<QgsPointXY> ring = ringWithInsertedBoundaryPoints( inputRing, QVector<QgsPointXY>{ bestPrimaryEnd, bestSecondaryEnd } );

    QVector<BentGableSegment> segments;
    BentGableSegment primarySegment;
    BentGableSegment secondarySegment;
    if ( !makeBentSegment( bestPrimaryEnd, bestBend, ridgePoint.z(), ring, boundaryPoint, primarySegment ) )
      return result;
    if ( !makeBentSegment( bestBend, bestSecondaryEnd, ridgePoint.z(), ring, boundaryPoint, secondarySegment ) )
      return result;
    segments << primarySegment << secondarySegment;

    const QgsPoint splitPoint( bestBend.x(), bestBend.y(), ridgePoint.z() );
    const double primarySide = signedDistanceToLine( bestPrimaryEnd, splitPoint, bestSplitNormalX, bestSplitNormalY );
    if ( std::fabs( primarySide ) <= 1e-8 )
      return result;

    const QgsGeometry footprintGeometry = polygonGeometryFromRing( ring );
    const QgsPointXY splitXY( splitPoint.x(), splitPoint.y() );
    const QgsGeometry primaryFootprint = intersectWithHalfPlane( footprintGeometry, ring, splitXY, bestSplitNormalX, bestSplitNormalY, primarySide > 0.0 );
    const QgsGeometry secondaryFootprint = intersectWithHalfPlane( footprintGeometry, ring, splitXY, bestSplitNormalX, bestSplitNormalY, primarySide < 0.0 );
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

BuildingRoof::MeshResult BuildingRoof::buildCurvedRoofPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints )
{
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

  result.success = !result.mesh.isEmpty();
  if ( !result.success )
    result.error = QStringLiteral( "Curved roof mesh generation failed." );
  return result;
}

BuildingRoof::MeshResult BuildingRoof::buildApexRoofPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints )
{
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

  result.success = !result.mesh.isEmpty();
  if ( !result.success )
    result.error = QStringLiteral( "Apex roof mesh generation failed." );
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
      const QList<RoofPoint> automaticRoofPoints = topHeightGabledRoofPoints( ring, boundary, ridges, pointCloudSamples );
      if ( !automaticRoofPoints.isEmpty() )
      {
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

BuildingRoof::MeshResult BuildingRoof::buildHippedRoofPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints )
{
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

  return buildAnchoredRoofPrismMesh( buildingGeometry, eaveAnchors, roofAnchors, QStringLiteral( "Hipped roof" ) );
}
