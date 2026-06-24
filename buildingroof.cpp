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

  QList<BuildingRoof::RoofPoint> boundaryPoints( const QList<BuildingRoof::RoofPoint> &roofPoints )
  {
    QList<BuildingRoof::RoofPoint> points;
    for ( const BuildingRoof::RoofPoint &roofPoint : roofPoints )
    {
      if ( !isGroundPointType( roofPoint.type ) && !isRidgePointType( roofPoint.type ) )
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

  double gabledTopZ( const QgsPointXY &point, const QgsPoint &boundaryPoint, const QgsPoint &ridgePoint, double normalX, double normalY, double oppositeLimit, double baseHeight )
  {
    const double roofRise = ridgePoint.z() - boundaryPoint.z();
    const double ridgeHeight = baseHeight + roofRise;
    const double distance = signedDistanceToLine( point, ridgePoint, normalX, normalY );
    const double boundaryDistance = signedDistanceToLine( QgsPointXY( boundaryPoint.x(), boundaryPoint.y() ), ridgePoint, normalX, normalY );
    const double sameSideLimit = std::fabs( boundaryDistance );

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

  void appendTriangulatedRoofSurface( BuildingRoof::Mesh &mesh, const QVector<QgsPointXY> &polygon, const QgsPoint &boundaryPoint, const QgsPoint &ridgePoint, double normalX, double normalY, double oppositeLimit, double baseHeight )
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
      mesh.vertices.append( QgsPoint( point.x(), point.y(), gabledTopZ( point, boundaryPoint, ridgePoint, normalX, normalY, oppositeLimit, baseHeight ) ) );

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

BuildingRoof::MeshResult BuildingRoof::buildGabledRoofPrismMesh( const QgsGeometry &buildingGeometry, double buildingHeight, const QList<RoofPoint> &roofPoints )
{
  Q_UNUSED( buildingHeight )
  MeshResult result;

  const QList<RoofPoint> boundary = boundaryPoints( roofPoints );
  QList<RoofPoint> ridges;
  for ( const RoofPoint &roofPoint : roofPoints )
  {
    if ( isRidgePointType( roofPoint.type ) )
      ridges.append( roofPoint );
  }

  if ( boundary.size() != 1 || ridges.size() != 1 )
  {
    result.error = QStringLiteral( "Gabled roof requires exactly one boundary point and one ridge point." );
    return result;
  }

  const QgsPoint boundaryPoint = boundary.first().point;
  const QgsPoint ridgePoint = ridges.first().point;
  if ( ridgePoint.z() <= boundaryPoint.z() + 1e-6 )
  {
    result.error = QStringLiteral( "The ridge point must be higher than the boundary point." );
    return result;
  }

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

  const double normalX = -dirY;
  const double normalY = dirX;
  const double boundaryDistance = signedDistanceToLine( QgsPointXY( boundaryPoint.x(), boundaryPoint.y() ), ridgePoint, normalX, normalY );
  if ( std::fabs( boundaryDistance ) <= 1e-6 )
  {
    result.error = QStringLiteral( "The boundary point must not lie on the ridge line." );
    return result;
  }

  ring = ringWithRidgeIntersections( ring, ridgePoint, normalX, normalY );
  double oppositeLimit = 0.0;
  for ( const QgsPointXY &point : ring )
  {
    const double distance = signedDistanceToLine( point, ridgePoint, normalX, normalY );
    if ( distance * boundaryDistance < 0.0 )
      oppositeLimit = std::max( oppositeLimit, std::fabs( distance ) );
  }

  const double baseHeight = boundaryPoint.z();
  for ( const QgsPointXY &point : ring )
  {
    result.mesh.vertices.append( QgsPoint( point.x(), point.y(), 0.0 ) );
    result.mesh.vertices.append( QgsPoint( point.x(), point.y(), gabledTopZ( point, boundaryPoint, ridgePoint, normalX, normalY, oppositeLimit, baseHeight ) ) );
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

  appendTriangulatedRoofSurface( result.mesh, clipRingByRidgeSide( ring, ridgePoint, normalX, normalY, true ), boundaryPoint, ridgePoint, normalX, normalY, oppositeLimit, baseHeight );
  appendTriangulatedRoofSurface( result.mesh, clipRingByRidgeSide( ring, ridgePoint, normalX, normalY, false ), boundaryPoint, ridgePoint, normalX, normalY, oppositeLimit, baseHeight );

  result.success = !result.mesh.isEmpty();
  if ( !result.success )
    result.error = QStringLiteral( "Gabled roof mesh generation failed." );
  return result;
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
  const double defaultEaveZ = boundaries.first().point.z();
  appendProfileAnchor( profileAnchors, minS, defaultEaveZ );
  appendProfileAnchor( profileAnchors, maxS, defaultEaveZ );
  for ( const RoofPoint &boundary : boundaries )
    appendProfileAnchor( profileAnchors, profileDistance( QgsPointXY( boundary.point.x(), boundary.point.y() ), normalX, normalY ), boundary.point.z() );
  for ( const RoofPoint &ridge : ridges )
    appendProfileAnchor( profileAnchors, profileDistance( QgsPointXY( ridge.point.x(), ridge.point.y() ), normalX, normalY ), ridge.point.z() );

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
  const QList<RoofPoint> ridges = ridgePoints( roofPoints );
  if ( boundaries.size() != 1 || ridges.size() != 2 )
  {
    MeshResult result;
    result.error = QStringLiteral( "Hipped roof requires exactly one boundary point and two ridge points." );
    return result;
  }

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
  appendAnchor( roofAnchors, QgsPointXY( boundary.point.x(), boundary.point.y() ), boundary.point.z() );
  appendAnchor( roofAnchors, QgsPointXY( ridges.at( 0 ).point.x(), ridges.at( 0 ).point.y() ), ridges.at( 0 ).point.z() );
  appendAnchor( roofAnchors, QgsPointXY( ridges.at( 1 ).point.x(), ridges.at( 1 ).point.y() ), ridges.at( 1 ).point.z() );

  return buildAnchoredRoofPrismMesh( buildingGeometry, eaveAnchors, roofAnchors, QStringLiteral( "Hipped roof" ) );
}
