#include "roofinteraction.h"

#include "roofmodelpipeline.h"
#include "ui_roofedit.h"

#include <qgisinterface.h>
#include <qgscoordinatetransform.h>
#include <qgsexception.h>
#include <qgsgeometryengine.h>
#include <qgsmapcanvas.h>
#include <qgsmapmouseevent.h>
#include <qgsmaptopixel.h>
#include <qgsmessagebar.h>
#include <qgsrubberband.h>

#include <QApplication>
#include <QKeyEvent>
#include <QKeySequence>
#include <QMouseEvent>
#include <QSignalBlocker>
#include <QWheelEvent>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace
{
  using RoofType = RoofModelSettings::Type;
  using GableSideWidths = BuildingRoof::GableSideWidths;

  QgsGeometry polygonalPart( QgsGeometry geometry )
  {
    if ( geometry.type() == Qgis::GeometryType::Unknown )
      geometry.convertGeometryCollectionToSubclass( Qgis::GeometryType::Polygon );
    if ( geometry.type() != Qgis::GeometryType::Polygon || geometry.isEmpty() )
      return {};
    if ( !geometry.isMultipart() )
      return QgsGeometry::fromMultiPolygonXY( { geometry.asPolygon() } );
    return QgsGeometry::fromMultiPolygonXY( geometry.asMultiPolygon() );
  }

  QString geometryValue( const QgsGeometry &geometry )
  {
    return QString::fromLatin1( geometry.asWkb().toBase64() );
  }

  QVariantMap regionValue( const QgsGeometry &geometry, double altitude )
  {
    return { { QStringLiteral( "wkb" ), geometryValue( geometry ) },
             { QStringLiteral( "height" ), altitude } };
  }

  QgsGeometry storedGeometry( const QVariantMap &region, const QString &key )
  {
    QgsGeometry geometry;
    geometry.fromWkb( QByteArray::fromBase64( region.value( key ).toString().toLatin1() ) );
    return geometry;
  }

  QgsGeometry regionGeometry( const QVariantMap &region )
  {
    return storedGeometry( region, QStringLiteral( "wkb" ) );
  }

  std::optional<QgsGeometry> uncoveredRegion( QgsGeometry area, const QVariantList &regions, int first )
  {
    for ( int index = first; index < regions.size() && !area.isEmpty(); ++index )
    {
      const QgsGeometry covered = regionGeometry( regions[index].toMap() );
      if ( covered.isEmpty() || !area.boundingBox().intersects( covered.boundingBox() ) )
        continue;
      QgsGeometry difference = area.difference( covered );
      if ( difference.isNull() )
        return std::nullopt;
      area = polygonalPart( std::move( difference ) );
    }
    return area;
  }

  bool isPartition( const QVariant &value )
  {
    return value.toMap().value( QStringLiteral( "partition" ) ).toBool();
  }

  QVariantMap withRidge( QVariantMap region, const QgsPolylineXY &vertices, double lift )
  {
    region.remove( QStringLiteral( "ridge_support" ) );
    region.remove( QStringLiteral( "fitted_gable" ) );
    region.remove( QStringLiteral( "gable_widths" ) );
    region.insert( QStringLiteral( "ridge" ), geometryValue( QgsGeometry::fromPolylineXY( vertices ) ) );
    region.insert( QStringLiteral( "ridge_lift" ), lift );
    return region;
  }

  bool validGableWidths( const GableSideWidths &widths, double minimum = 0.0 )
  {
    return std::isfinite( widths[0] ) && std::isfinite( widths[1] ) && std::min( widths[0], widths[1] ) > minimum;
  }

  GableSideWidths storedGableWidths( const QVariantMap &region )
  {
    const QVariantList values = region.value( QStringLiteral( "gable_widths" ) ).toList();
    if ( values.size() != 2 )
      return {};
    return { values[0].toDouble(), values[1].toDouble() };
  }

  RoofType storedRidgeType( const QVariantMap &region )
  {
    const auto type = RoofModelSettings::typeFromId( region.value( QStringLiteral( "ridge_type" ) ).toString() );
    return type == RoofType::None && region.value( QStringLiteral( "fitted_gable" ) ).toBool() ? RoofType::Gabled : type;
  }

  QgsGeometry boundaryOf( const QgsGeometry &geometry )
  {
    return QgsGeometry( geometry.constGet()->boundary() );
  }

  double cross( const QgsPointXY &a, const QgsPointXY &b, const QgsPointXY &c )
  {
    return ( b.x() - a.x() ) * ( c.y() - a.y() ) - ( b.y() - a.y() ) * ( c.x() - a.x() );
  }

  double geometryTolerance( const QgsGeometry &geometry )
  {
    const QgsRectangle extent = geometry.boundingBox();
    return std::max( 1e-9, std::hypot( extent.width(), extent.height() ) * 1e-9 );
  }

  double areaTolerance( const QgsGeometry &geometry )
  {
    const double tolerance = geometryTolerance( geometry );
    return std::max( tolerance * tolerance, geometry.area() * 1e-9 );
  }

  RoofType ridgeTypeForGeometry( const QgsGeometry &outline, const QgsPolylineXY &ridge )
  {
    if ( ridge.size() == 3 )
      return RoofType::BrokenGabled;
    const QgsGeometry boundary = boundaryOf( outline );
    const double tolerance = geometryTolerance( outline );
    if ( boundary.distance( QgsGeometry::fromPointXY( ridge.first() ) ) <= tolerance
         && boundary.distance( QgsGeometry::fromPointXY( ridge.last() ) ) <= tolerance )
      return RoofType::Gabled;
    return RoofType::Hipped;
  }

  QgsGeometry ridgeComponent( const QgsGeometry &outline, const QgsPolylineXY &ridge )
  {
    if ( ridge.size() < 2 )
      return {};
    const double tolerance = geometryTolerance( outline );
    for ( const QgsGeometry &part : outline.asGeometryCollection() )
    {
      if ( part.distance( QgsGeometry::fromPointXY( ridge.first() ) ) <= tolerance
           && part.distance( QgsGeometry::fromPointXY( ridge.last() ) ) <= tolerance )
        return part;
    }
    return {};
  }

  struct RoofFacet
  {
    QgsGeometry geometry;
    QgsPointXY origin;
    double height = 0.0;
    double gradientX = 0.0;
    double gradientY = 0.0;

    double heightAt( const QgsPointXY &point ) const
    {
      return height + gradientX * ( point.x() - origin.x() ) + gradientY * ( point.y() - origin.y() );
    }
  };

  struct RoofPatch
  {
    QgsGeometry geometry;
    QgsGeometry boundary;
    double height;
    QVector<RoofFacet> facets;
    QgsGeometry slopedArea;
    QVector<QPair<QgsPoint, QgsPoint>> slopeLines;
  };

  void appendTriangle( BuildingRoof::Mesh &mesh, const QgsPoint &a, const QgsPoint &b, const QgsPoint &c )
  {
    const int offset = mesh.vertices.size();
    mesh.vertices << a << b << c;
    mesh.indices << offset << offset + 1 << offset + 2;
  }

  bool appendSurface( BuildingRoof::Mesh &mesh, const QgsGeometry &geometry, double height, bool up,
                      const RoofFacet *facet = nullptr )
  {
    const QgsGeometry triangles = geometry.constrainedDelaunayTriangulation();
    double triangulatedArea = 0.0;
    const double minimumAltitude = 2.0 * geometryTolerance( geometry );
    for ( const QgsGeometry &triangle : triangles.asGeometryCollection() )
    {
      const QgsPolygonXY polygon = triangle.asPolygon();
      if ( polygon.isEmpty() || polygon.first().size() != 4 )
        continue;
      const QgsPolylineXY &ring = polygon.first();
      const double signedArea = cross( ring[0], ring[1], ring[2] );
      const double longestEdge = std::max( { ring[0].distance( ring[1] ), ring[1].distance( ring[2] ), ring[2].distance( ring[0] ) } );
      // Drop near-collinear clipping artefacts without discarding real triangles by the area of the whole roof.
      if ( !std::isfinite( signedArea ) || std::abs( signedArea ) <= minimumAltitude * longestEdge )
        continue;
      triangulatedArea += 0.5 * std::abs( signedArea );
      const auto lift = [height, facet]( const QgsPointXY &p ) { return QgsPoint( p.x(), p.y(), facet ? facet->heightAt( p ) : height ); };
      const QgsPoint a = lift( ring[0] );
      const QgsPoint b = lift( ring[1] );
      const QgsPoint c = lift( ring[2] );
      if ( ( signedArea > 0.0 ) == up )
        appendTriangle( mesh, a, b, c );
      else
        appendTriangle( mesh, a, c, b );
    }
    return triangulatedArea > 0.0 && std::abs( triangulatedArea - geometry.area() ) <= areaTolerance( geometry );
  }

  double segmentParameter( const QgsPointXY &a, const QgsPointXY &b, const QgsPointXY &point )
  {
    const double dx = b.x() - a.x(), dy = b.y() - a.y();
    return std::clamp( ( ( point.x() - a.x() ) * dx + ( point.y() - a.y() ) * dy ) / ( dx * dx + dy * dy ), 0.0, 1.0 );
  }

  QgsPointXY interpolate( const QgsPointXY &a, const QgsPointXY &b, double t )
  {
    return QgsPointXY( a.x() + t * ( b.x() - a.x() ), a.y() + t * ( b.y() - a.y() ) );
  }

  struct GableEave
  {
    double width = 0.0;
    double score = 0.0;
    QgsPointXY direction;
  };

  std::array<GableEave, 2> gableEaves( const QgsGeometry &component, const QgsPolylineXY &ridge )
  {
    const QgsPointXY &a = ridge.first(), &b = ridge.last();
    const double length = a.distance( b );
    const QgsPolylineXY exterior = component.asPolygon().first();
    std::array<GableEave, 2> eaves;
    std::array<QVector<GableEave>, 2> candidates;
    std::array<double, 2> farthest{};
    for ( const QgsPointXY &point : exterior )
    {
      const double distance = cross( a, b, point ) / length;
      const int side = distance > 0.0;
      farthest[side] = std::max( farthest[side], std::abs( distance ) );
    }
    for ( int index = 1; index < exterior.size(); ++index )
    {
      const QgsPointXY &p = exterior[index - 1], &q = exterior[index];
      const double pDistance = cross( a, b, p ) / length;
      const double qDistance = cross( a, b, q ) / length;
      const double edgeLength = p.distance( q );
      const double width = std::abs( ( pDistance + qDistance ) * 0.5 );
      const int side = pDistance > 0.0;
      if ( edgeLength <= 0.0 || farthest[side] <= 0.0 || side != ( qDistance > 0.0 ) || width < 0.45 * farthest[side]
           || std::abs( pDistance - qDistance ) > 0.12 * edgeLength )
        continue;
      const double score = edgeLength * std::sqrt( width / farthest[side] );
      candidates[side].append( { width, score, QgsPointXY( ( q.x() - p.x() ) / edgeLength, ( q.y() - p.y() ) / edgeLength ) } );
    }
    for ( int side : { 0, 1 } )
    {
      eaves[side].width = farthest[side];
      const double offsetTolerance = farthest[side] * 0.005;
      for ( GableEave candidate : candidates[side] )
      {
        // A cutout can interrupt a single main eave. Count its collinear pieces together.
        double support = 0.0;
        for ( const GableEave &other : candidates[side] )
        {
          const double angle = candidate.direction.x() * other.direction.y() - candidate.direction.y() * other.direction.x();
          if ( std::abs( angle ) < 0.01 && std::abs( candidate.width - other.width ) <= offsetTolerance )
            support += other.score;
        }
        if ( support > eaves[side].score )
        {
          candidate.score = support;
          eaves[side] = candidate;
        }
      }
    }
    return eaves;
  }

  GableSideWidths gableWidths( const QgsGeometry &component, const QgsPolylineXY &ridge, const GableSideWidths &fitted )
  {
    if ( validGableWidths( fitted, geometryTolerance( component ) ) )
      return fitted;
    const auto eaves = gableEaves( component, ridge );
    return { eaves[0].width, eaves[1].width };
  }

  GableSideWidths movedGableWidths( const QgsPolylineXY &previous, const QgsPolylineXY &ridge, const GableSideWidths &widths )
  {
    if ( !validGableWidths( widths ) )
      return {};
    const QgsPointXY center = interpolate( previous.first(), previous.last(), 0.5 );
    const double length = previous.first().distance( previous.last() );
    const double nx = ( previous.first().y() - previous.last().y() ) / length;
    const double ny = ( previous.last().x() - previous.first().x() ) / length;
    const QgsPointXY left( center.x() - widths[0] * nx, center.y() - widths[0] * ny );
    const QgsPointXY right( center.x() + widths[1] * nx, center.y() + widths[1] * ny );
    const double newLength = ridge.first().distance( ridge.last() );
    const double leftDistance = cross( ridge.first(), ridge.last(), left ) / newLength;
    const double rightDistance = cross( ridge.first(), ridge.last(), right ) / newLength;
    const GableSideWidths moved = leftDistance < rightDistance ? GableSideWidths{ -leftDistance, rightDistance }
                                                               : GableSideWidths{ -rightDistance, leftDistance };
    return validGableWidths( moved ) ? moved : GableSideWidths{};
  }

  QgsPolylineXY clipRidgeAxis( const QgsGeometry &component, const QgsPolylineXY &ridge )
  {
    const QgsPointXY &a = ridge.first(), &b = ridge.last();
    const double length = a.distance( b );
    if ( length <= geometryTolerance( component ) )
      return {};
    const QgsRectangle bounds = component.boundingBox();
    const double reach = 2.0 * std::hypot( bounds.width(), bounds.height() ) / length;
    const QgsGeometry axis = QgsGeometry::fromPolylineXY( { interpolate( a, b, -reach ), interpolate( a, b, reach ) } );
    const QgsGeometry clipped = axis.intersection( component );
    double first = std::numeric_limits<double>::max(), last = -first;
    auto points = clipped.vertices();
    while ( points.hasNext() )
    {
      const QgsPointXY point( points.next() );
      const double t = ( ( point.x() - a.x() ) * ( b.x() - a.x() )
                         + ( point.y() - a.y() ) * ( b.y() - a.y() ) ) / ( length * length );
      first = std::min( first, t );
      last = std::max( last, t );
    }
    if ( ( last - first ) * length <= geometryTolerance( component ) )
      return {};
    return { interpolate( a, b, first ), interpolate( a, b, last ) };
  }

  std::optional<QgsPointXY> parallelEaveDirection( const std::array<GableEave, 2> &eaves )
  {
    const QgsPointXY &first = eaves[0].direction, &second = eaves[1].direction;
    if ( eaves[0].score == 0.0 || eaves[1].score == 0.0
         || std::abs( first.x() * second.y() - first.y() * second.x() ) > 1e-4 )
      return std::nullopt;
    return first;
  }

  QgsPolylineXY alignRidgeToEaves( const QgsGeometry &component, const QgsPolylineXY &ridge )
  {
    const auto direction = parallelEaveDirection( gableEaves( component, ridge ) );
    if ( !direction )
      return ridge;
    const QgsPointXY &axis = *direction;
    const QgsPointXY center = interpolate( ridge.first(), ridge.last(), 0.5 );
    const double dot = ( ridge.last().x() - ridge.first().x() ) * axis.x() + ( ridge.last().y() - ridge.first().y() ) * axis.y();
    const double length = std::copysign( ridge.first().distance( ridge.last() ) * 0.5, dot );
    const auto aligned = clipRidgeAxis( component, { QgsPointXY( center.x() - length * axis.x(), center.y() - length * axis.y() ),
                                                       QgsPointXY( center.x() + length * axis.x(), center.y() + length * axis.y() ) } );
    return aligned.isEmpty() ? ridge : aligned;
  }

  struct ProfileSample
  {
    double fraction;
    double z;
    int side;
  };

  bool fitRoofProfile( const QVector<ProfileSample> &profile, const std::array<int, 2> &count,
                       double &eave, double &rise, double *score = nullptr )
  {
    if ( std::min( count[0], count[1] ) < 6 )
      return false;
    double base = eave, lift = rise;
    const double robustLimit = std::max( 0.15, rise * 0.10 );
    for ( int iteration = 0; iteration < 5; ++iteration )
    {
      double w = 0.0, x = 0.0, y = 0.0, xx = 0.0, xy = 0.0;
      for ( const auto &[fraction, z, side] : std::as_const( profile ) )
      {
        const double residual = std::abs( z - base - lift * fraction );
        const double weight = robustLimit / ( std::max( robustLimit, residual ) * count[side] );
        w += weight;
        x += weight * fraction;
        y += weight * z;
        xx += weight * fraction * fraction;
        xy += weight * fraction * z;
      }
      const double spread = xx - x * x / w;
      if ( spread <= w * 0.005 )
        return false;
      lift = ( xy - x * y / w ) / spread;
      base = ( y - lift * x ) / w;
    }
    if ( !std::isfinite( base ) || !std::isfinite( lift ) || lift <= 0.05 )
      return false;
    if ( score )
    {
      *score = 0.0;
      for ( const auto &[fraction, z, side] : std::as_const( profile ) )
      {
        const double residual = std::abs( z - base - lift * fraction );
        const double loss = residual <= robustLimit ? residual * residual : robustLimit * ( 2.0 * residual - robustLimit );
        *score += loss / count[side];
      }
    }
    eave = base;
    rise = lift;
    return true;
  }

  // Fit one common eave height and one rise after choosing the two main eave positions.
  bool fitGableHeights( const QgsGeometry &component, const QgsPolylineXY &ridge,
                        const QVector<BuildingRoof::RoofSample> &samples, double &eave, double &rise,
                        double *score = nullptr, const GableSideWidths &fitted = {} )
  {
    const auto widths = gableWidths( component, ridge, fitted );
    if ( !validGableWidths( widths, geometryTolerance( component ) ) )
      return false;
    const auto &a = ridge.first();
    const auto &b = ridge.last();
    const double length = a.distance( b );
    QVector<ProfileSample> profile;
    profile.reserve( samples.size() );
    std::array<int, 2> count{};
    for ( const auto &sample : samples )
    {
      const double distance = cross( a, b, QgsPointXY( sample.point ) ) / length;
      const int side = distance > 0.0;
      profile.append( { 1.0 - std::abs( distance ) / widths[side], sample.point.z(), side } );
      ++count[side];
    }
    return fitRoofProfile( profile, count, eave, rise, score );
  }

  double ridgeDisplacement( const QgsPolylineXY &first, const QgsPolylineXY &second )
  {
    double distance = 0.0;
    for ( int i = 0; i < first.size(); ++i )
      distance = std::max( distance, first[i].distance( second[i] ) );
    return distance;
  }

  bool refineGable( const QgsGeometry &component, const QVector<BuildingRoof::RoofSample> &samples,
                    QgsPolylineXY &ridge, double &eave, double &rise, double movementLimit )
  {
    const QgsPolylineXY aligned = alignRidgeToEaves( component, ridge );
    const auto eaves = gableEaves( component, ridge );
    const bool fixedDirection = parallelEaveDirection( eaves ).has_value() && ridgeDisplacement( aligned, ridge ) <= movementLimit;
    const QgsPolylineXY &seed = fixedDirection ? aligned : ridge;
    const double offsetLimit = std::min( movementLimit, 0.35 * std::min( eaves[0].width, eaves[1].width ) );
    constexpr double angleLimit = 0.14;
    const double angle = std::atan2( seed.last().y() - seed.first().y(), seed.last().x() - seed.first().x() );
    const QgsPointXY center = interpolate( seed.first(), seed.last(), 0.5 );
    const double halfLength = 0.5 * seed.first().distance( seed.last() );

    // Score all candidates on the same sample subset, then fit the winning position on all samples.
    QVector<BuildingRoof::RoofSample> searchSamples;
    constexpr int sampleBudget = 8000;
    const int stride = std::max( 1, ( static_cast<int>( samples.size() ) + sampleBudget - 1 ) / sampleBudget );
    searchSamples.reserve( std::min( sampleBudget, static_cast<int>( samples.size() ) ) );
    for ( int i = 0; i < samples.size(); i += stride )
      searchSamples.append( samples[i] );

    double bestScore = std::numeric_limits<double>::max();
    double bestOffset = 0.0, bestAngle = 0.0;
    QgsPolylineXY bestRidge;
    const auto evaluate = [&]( double offset, double turn ) {
      if ( std::abs( offset ) > offsetLimit || std::abs( turn ) > angleLimit )
        return;
      const QgsPointXY midpoint( center.x() - offset * std::sin( angle ), center.y() + offset * std::cos( angle ) );
      const double dx = halfLength * std::cos( angle + turn ), dy = halfLength * std::sin( angle + turn );
      QgsPolylineXY candidate{ QgsPointXY( midpoint.x() - dx, midpoint.y() - dy ), QgsPointXY( midpoint.x() + dx, midpoint.y() + dy ) };
      double candidateEave = eave, candidateRise = rise, score = 0.0;
      if ( !fitGableHeights( component, candidate, searchSamples, candidateEave, candidateRise, &score ) || score >= bestScore )
        return;
      candidate = clipRidgeAxis( component, candidate );
      if ( candidate.isEmpty() || ridgeDisplacement( candidate, ridge ) > movementLimit )
        return;
      bestScore = score;
      bestOffset = offset;
      bestAngle = turn;
      bestRidge = std::move( candidate );
    };
    evaluate( 0.0, 0.0 );
    const int turns = fixedDirection ? 0 : 3;
    for ( int i = -3; i <= 3; ++i )
    {
      for ( int j = -turns; j <= turns; ++j )
        evaluate( offsetLimit * i / 3, angleLimit * j / 3 );
    }
    for ( double scale : { 1.0 / 6, 1.0 / 12, 1.0 / 24, 1.0 / 48 } )
    {
      const double offset = bestOffset, turn = bestAngle;
      for ( int i : { -1, 0, 1 } )
      {
        for ( int j = fixedDirection ? 0 : -1; j <= ( fixedDirection ? 0 : 1 ); ++j )
          evaluate( offset + i * scale * offsetLimit, turn + j * scale * angleLimit );
      }
    }
    double refinedEave = eave, refinedRise = rise, refinedScore = 0.0, startingScore = 0.0;
    const bool fitted = !bestRidge.isEmpty() && fitGableHeights( component, bestRidge, samples, refinedEave, refinedRise, &refinedScore );
    const bool startingFit = fitGableHeights( component, ridge, samples, eave, rise, &startingScore );
    if ( fitted && ( !startingFit || refinedScore < startingScore ) )
    {
      ridge = std::move( bestRidge );
      eave = refinedEave;
      rise = refinedRise;
    }
    return fitted || startingFit;
  }

  bool finishPlaneRoof( RoofPatch &patch, const QgsGeometry &component );

  bool addStraightGable( RoofPatch &patch, const QgsGeometry &component, const QgsPolylineXY &vertices, double lift,
                         const GableSideWidths &fitted )
  {
    const QgsPointXY &a = vertices.first();
    const QgsPointXY &b = vertices.last();
    const double length = a.distance( b );
    const double dx = ( b.x() - a.x() ) / length;
    const double dy = ( b.y() - a.y() ) / length;
    const QgsRectangle bounds = component.boundingBox();
    const double reach = 2.0 * std::hypot( bounds.width(), bounds.height() );
    const QgsPointXY start( a.x() - reach * dx, a.y() - reach * dy );
    const QgsPointXY end( a.x() + reach * dx, a.y() + reach * dy );
    const double ridgeHeight = patch.height + lift;
    const auto widths = gableWidths( component, vertices, fitted );

    for ( int side : { -1, 1 } )
    {
      const double width = widths[side > 0];
      if ( width <= geometryTolerance( component ) )
        return false;
      const double nx = -side * dy;
      const double ny = side * dx;
      // The eave width sets the slope, while the template extends beyond the footprint.
      const QgsGeometry halfPlane = QgsGeometry::fromPolygonXY( { { start, end,
        QgsPointXY( end.x() + reach * nx, end.y() + reach * ny ),
        QgsPointXY( start.x() + reach * nx, start.y() + reach * ny ), start } } );
      const QgsGeometry clipped = polygonalPart( component.intersection( halfPlane ) );
      if ( clipped.isEmpty() )
        return false;
      patch.facets.append( { clipped, a, ridgeHeight, -lift * nx / width, -lift * ny / width } );
    }
    if ( !finishPlaneRoof( patch, component ) )
      return false;

    const QgsGeometry roofAxis = QgsGeometry::fromPolylineXY( { start, end } ).intersection( component );
    if ( roofAxis.isNull() )
      return false;
    for ( const QgsGeometry &axisPart : roofAxis.asGeometryCollection() )
    {
      const QgsGeometry visible = axisPart.intersection( patch.geometry );
      for ( const QgsGeometry &part : visible.asGeometryCollection() )
      {
        const QgsPolylineXY line = part.asPolyline();
        for ( int index = 1; index < line.size(); ++index )
          patch.slopeLines.append( qMakePair( QgsPoint( line[index - 1].x(), line[index - 1].y(), ridgeHeight ),
                                             QgsPoint( line[index].x(), line[index].y(), ridgeHeight ) ) );
      }
    }
    return true;
  }

  QgsGeometry clipAtPlane( const QgsGeometry &geometry, const QgsRectangle &bounds, const QgsPointXY &origin,
                           double offset, double dx, double dy )
  {
    if ( geometry.isEmpty() )
      return {};
    const QgsPolylineXY rectangle{ { bounds.xMinimum(), bounds.yMinimum() }, { bounds.xMaximum(), bounds.yMinimum() },
                                   { bounds.xMaximum(), bounds.yMaximum() }, { bounds.xMinimum(), bounds.yMaximum() } };
    const auto valueAt = [&]( const QgsPointXY &point ) {
      return offset + dx * ( point.x() - origin.x() ) + dy * ( point.y() - origin.y() );
    };
    QgsPolylineXY ring;
    QgsPointXY previous = rectangle.last();
    double previousValue = valueAt( previous );
    int inside = 0;
    for ( const QgsPointXY &point : rectangle )
    {
      const double value = valueAt( point );
      inside += value >= 0.0;
      if ( ( previousValue < 0.0 && value > 0.0 ) || ( previousValue > 0.0 && value < 0.0 ) )
        ring.append( interpolate( previous, point, previousValue / ( previousValue - value ) ) );
      if ( value >= 0.0 )
        ring.append( point );
      previous = point;
      previousValue = value;
    }
    if ( inside == rectangle.size() )
      return geometry;
    if ( ring.size() < 3 )
      return {};
    ring.append( ring.first() );
    return polygonalPart( geometry.intersection( QgsGeometry::fromPolygonXY( { ring } ) ) );
  }

  bool finishPlaneRoof( RoofPatch &patch, const QgsGeometry &component )
  {
    const double minimumArea = areaTolerance( component );
    QVector<QgsGeometry> faces;
    double area = 0.0;
    for ( const RoofFacet &facet : std::as_const( patch.facets ) )
    {
      faces.append( facet.geometry );
      area += facet.geometry.area();
    }
    const QgsGeometry covered = QgsGeometry::unaryUnion( faces );
    if ( covered.isEmpty() || area - covered.area() > minimumArea )
      return false;
    const QgsGeometry missing = component.difference( covered );
    if ( missing.isNull() || missing.area() > minimumArea )
      return false;
    patch.slopedArea = component;
    return true;
  }

  std::optional<RoofFacet> hipEndFromCorners( const QgsPolylineXY &hull, const QVector<RoofFacet> &sides, const QgsPointXY &tip,
                                             double dx, double dy, double inward, double endWidth, double eave, double top )
  {
    struct Corner
    {
      QgsPointXY point;
      double across;
      double outward;
      double support;
    };
    const auto acrossAt = [&]( const QgsPointXY &point ) {
      return -( point.x() - tip.x() ) * dy + ( point.y() - tip.y() ) * dx;
    };
    const auto outwardAt = [&]( const QgsPointXY &point ) {
      return -inward * ( ( point.x() - tip.x() ) * dx + ( point.y() - tip.y() ) * dy ) / endWidth;
    };
    // Use the local width at this end to find its corners. A projecting bay
    // half way along the building can be wider than either real gable end.
    std::array<double, 2> cornerWidths{};
    for ( const QgsPointXY &point : hull )
    {
      if ( outwardAt( point ) < 0.35 )
        continue;
      const double across = acrossAt( point );
      const int side = across > 0.0;
      cornerWidths[side] = std::max( cornerWidths[side], std::abs( across ) );
    }
    if ( cornerWidths[0] <= 0.0 || cornerWidths[1] <= 0.0 )
      return std::nullopt;

    std::array<QVector<Corner>, 2> corners;
    for ( const QgsPointXY &point : hull )
    {
      const double across = acrossAt( point );
      const double outward = outwardAt( point );
      const int side = across > 0.0;
      const double sideReach = std::abs( across ) / cornerWidths[side];
      if ( sideReach >= 0.6 && outward >= -0.15 )
        corners[side].append( { point, across, outward, std::min( 1.0, sideReach ) + outward } );
    }

    std::optional<RoofFacet> selected;
    double bestSupport = -std::numeric_limits<double>::max();
    const double lift = top - eave;
    for ( const Corner &low : corners[0] )
    {
      for ( const Corner &high : corners[1] )
      {
        const double endAtAxis = ( high.across * low.outward - low.across * high.outward ) / ( high.across - low.across );
        if ( endAtAxis <= 0.15 )
          continue;

        const double lx = low.point.x() - tip.x(), ly = low.point.y() - tip.y();
        const double hx = high.point.x() - tip.x(), hy = high.point.y() - tip.y();
        const double determinant = lx * hy - ly * hx;
        const double lz = sides[0].heightAt( low.point ) - top, hz = sides[1].heightAt( high.point ) - top;
        RoofFacet cap{ {}, tip, top, ( lz * hy - hz * ly ) / determinant, ( lx * hz - hx * lz ) / determinant };
        if ( inward * ( cap.gradientX * dx + cap.gradientY * dy ) <= 0.0 )
          continue;

        // Corner anchors can span several short survey edges. A slightly convex
        // edge can cut the end plane below the reference eave at intermediate points.
        double belowEave = 0.0;
        for ( const QgsPointXY &point : hull )
          belowEave = std::max( belowEave, eave - cap.heightAt( point ) );
        if ( belowEave > lift * 0.12 )
          continue;
        const double support = low.support + high.support - 4.0 * belowEave / lift;
        if ( support > bestSupport )
        {
          bestSupport = support;
          selected = cap;
        }
      }
    }
    return selected;
  }

  bool addHippedRoof( RoofPatch &patch, const QgsGeometry &component, const QgsPolylineXY &ridge, double lift )
  {
    const auto &a = ridge.first(), &b = ridge.last();
    const double length = a.distance( b );
    const double dx = ( b.x() - a.x() ) / length, dy = ( b.y() - a.y() ) / length;
    const double top = patch.height + lift;
    const double tolerance = geometryTolerance( component );
    QgsRectangle bounds = component.boundingBox();
    bounds.grow( 0.02 * std::hypot( bounds.width(), bounds.height() ) );
    const QgsPolylineXY exterior = component.asPolygon().first();
    std::array<double, 2> sideWidths{}, endWidths{};
    // The full hip solid has an envelope in ridge-aligned coordinates. A notch may
    // cross the ridge axis before the outermost corner, so size all faces by projection.
    for ( const QgsPointXY &point : exterior )
    {
      const double x = point.x() - a.x(), y = point.y() - a.y();
      const double along = x * dx + y * dy;
      const double across = -x * dy + y * dx;
      sideWidths[0] = std::max( sideWidths[0], -across );
      sideWidths[1] = std::max( sideWidths[1], across );
      endWidths[0] = std::max( endWidths[0], -along );
      endWidths[1] = std::max( endWidths[1], along - length );
    }
    QVector<RoofFacet> planes;
    for ( int side : { -1, 1 } )
    {
      const double width = sideWidths[side > 0];
      if ( width <= tolerance )
        return false;
      planes.append( { {}, a, top, lift * side * dy / width, -lift * side * dx / width } );
    }
    const QgsPolygonXY hull = component.convexHull().asPolygon();
    for ( int end : { 0, 1 } )
    {
      const auto &tip = ridge[end];
      const double inward = end == 0 ? 1.0 : -1.0;
      const double width = endWidths[end];
      if ( width <= tolerance )
        return false;
      const RoofFacet farthest{ {}, tip, top, lift * inward * dx / width, lift * inward * dy / width };
      const auto cap = hull.isEmpty() ? std::optional<RoofFacet>()
                                     : hipEndFromCorners( hull.first(), planes, tip, dx, dy, inward, width, patch.height, top );
      planes.append( cap.value_or( farthest ) );
    }

    // Intersect the footprint column with the four roof half-spaces. A plane is exposed
    // only where it is lower than the other three.
    for ( int i = 0; i < planes.size(); ++i )
    {
      RoofFacet face = planes[i];
      QgsGeometry area = component;
      for ( int j = 0; j < planes.size() && !area.isEmpty(); ++j )
      {
        if ( i == j )
          continue;
        const RoofFacet &other = planes[j];
        const double offset = other.heightAt( face.origin ) - face.height;
        area = clipAtPlane( area, bounds, face.origin, offset, other.gradientX - face.gradientX, other.gradientY - face.gradientY );
      }
      if ( !area.isEmpty() && area.area() > areaTolerance( component ) )
      {
        face.geometry = std::move( area );
        patch.facets.append( std::move( face ) );
      }
    }
    return finishPlaneRoof( patch, component );
  }

  bool addBentGable( RoofPatch &patch, const QgsGeometry &component, const QgsPolylineXY &ridge, double lift )
  {
    const auto surfaces = BuildingRoof::bentGabledSurface( component, ridge );
    for ( const auto &surface : surfaces )
    {
      const QgsGeometry geometry = polygonalPart( surface.geometry );
      if ( !geometry.isEmpty() )
        patch.facets.append( { geometry, ridge[1], patch.height + lift * surface.fractionAtBend, lift * surface.gradientX, lift * surface.gradientY } );
    }
    return finishPlaneRoof( patch, component );
  }

  bool addRidgeFacets( RoofPatch &patch, const QgsGeometry &outline, const QgsPolylineXY &vertices, double lift,
                       RoofType type = RoofType::None, const GableSideWidths &widths = {} )
  {
    if ( vertices.size() < 2 || vertices.size() > 3 || !std::isfinite( lift ) || lift <= 0.0 )
      return false;

    const double tolerance = geometryTolerance( outline );
    const QgsGeometry ridge = QgsGeometry::fromPolylineXY( vertices );
    if ( !ridge.isSimple() || ridge.length() <= tolerance )
      return false;

    // A straight gable can be clipped along its axis by holes or recesses in a single polygon.
    const bool straightGable = type == RoofType::Gabled;
    if ( ( type == RoofType::Hipped && vertices.size() != 2 ) || ( type == RoofType::BrokenGabled && vertices.size() != 3 ) )
      return false;
    if ( vertices.size() == 2 && type != RoofType::Hipped )
    {
      for ( const QgsGeometry &part : outline.asGeometryCollection() )
      {
        const QgsGeometry boundary = boundaryOf( part );
        if ( straightGable && part.distance( QgsGeometry::fromPointXY( vertices.first() ) ) <= tolerance
             && part.distance( QgsGeometry::fromPointXY( vertices.last() ) ) <= tolerance )
          return addStraightGable( patch, part, vertices, lift, widths );
        if ( boundary.distance( QgsGeometry::fromPointXY( vertices.first() ) ) <= tolerance
             && boundary.distance( QgsGeometry::fromPointXY( vertices.last() ) ) <= tolerance )
          return addStraightGable( patch, part, vertices, lift, widths );
      }
    }
    if ( straightGable )
      return false;

    // Keep all roof parts without a ridge flat, including other parts of a multipart region.
    QgsGeometry component;
    for ( const QgsGeometry &part : outline.asGeometryCollection() )
    {
      const QgsGeometry outside = ridge.difference( part );
      if ( !outside.isNull() && outside.length() <= tolerance )
      {
        component = part;
        break;
      }
    }
    if ( component.isEmpty() )
      return false;

    const QgsGeometry boundary = boundaryOf( component );
    const auto onBoundary = [&boundary, tolerance]( const QgsPointXY &point ) {
      return boundary.distance( QgsGeometry::fromPointXY( point ) ) <= tolerance;
    };
    const bool startsAtEave = onBoundary( vertices.first() );
    const bool endsAtEave = onBoundary( vertices.last() );
    if ( startsAtEave != endsAtEave || ( !startsAtEave && vertices.size() != 2 )
         || ( type == RoofType::Hipped && startsAtEave )
         || ( vertices.size() == 3 && onBoundary( vertices[1] ) ) )
      return false;
    return startsAtEave ? addBentGable( patch, component, vertices, lift ) : addHippedRoof( patch, component, vertices, lift );
  }

  bool makeRidgePreview( RoofPatch &patch, const QgsGeometry &component, const QgsPolylineXY &ridge, RoofType type,
                         const GableSideWidths &widths = {} )
  {
    try
    {
      return addRidgeFacets( patch, component, ridge, 1.0, type, widths );
    }
    catch ( const QgsNotSupportedException & )
    {
      return false;
    }
  }

  bool fitFacetHeights( const RoofPatch &patch, const QVector<BuildingRoof::RoofSample> &samples,
                        double &eave, double &lift, double *score = nullptr )
  {
    if ( samples.isEmpty() )
      return false;
    std::vector<std::unique_ptr<QgsGeometryEngine>> prepared;
    prepared.reserve( patch.facets.size() );
    for ( const RoofFacet &facet : patch.facets )
    {
      auto &engine = prepared.emplace_back( QgsGeometry::createGeometryEngine( facet.geometry.constGet() ) );
      engine->prepareGeometry();
    }
    QVector<ProfileSample> profile;
    profile.reserve( samples.size() );
    std::array<int, 2> counts{};
    for ( const auto &sample : samples )
    {
      const QgsPointXY xy( sample.point );
      const QgsGeometry query = QgsGeometry::fromPointXY( xy );
      for ( int i = 0; i < patch.facets.size(); ++i )
      {
        const RoofFacet &facet = patch.facets[i];
        if ( !facet.geometry.boundingBox().contains( xy ) || !prepared[i]->intersects( query.constGet() ) )
          continue;
        const double fraction = std::min( facet.heightAt( xy ), 1.0 );
        const int side = fraction >= 0.5;
        profile.append( { fraction, sample.point.z(), side } );
        ++counts[side];
        break;
      }
    }
    return fitRoofProfile( profile, counts, eave, lift, score );
  }

  void appendStep( BuildingRoof::Mesh &mesh, const QgsPointXY &a, const QgsPointXY &b,
                   double leftA, double leftB, double rightA, double rightB )
  {
    constexpr double elevationTolerance = 1e-7;
    const double diffA = leftA - rightA;
    const double diffB = leftB - rightB;
    if ( std::abs( diffA ) <= elevationTolerance && std::abs( diffB ) <= elevationTolerance )
      return;
    if ( diffA * diffB < 0 && std::abs( diffA ) > elevationTolerance && std::abs( diffB ) > elevationTolerance )
    {
      const double t = diffA / ( diffA - diffB );
      const QgsPointXY middle = interpolate( a, b, t );
      const double height = leftA + t * ( leftB - leftA );
      appendStep( mesh, a, middle, leftA, height, rightA, height );
      appendStep( mesh, middle, b, height, leftB, height, rightB );
      return;
    }
    if ( diffA + diffB < 0 )
    {
      appendStep( mesh, b, a, rightB, rightA, leftB, leftA );
      return;
    }
    const QgsPoint lowerA( a.x(), a.y(), rightA ), lowerB( b.x(), b.y(), rightB );
    const QgsPoint upperA( a.x(), a.y(), leftA ), upperB( b.x(), b.y(), leftB );
    if ( std::abs( diffA ) > elevationTolerance )
      appendTriangle( mesh, lowerA, lowerB, upperA );
    if ( std::abs( diffB ) > elevationTolerance )
      appendTriangle( mesh, upperA, lowerB, upperB );
    mesh.structureLines.append( qMakePair( upperA, upperB ) );
  }

  void appendWallSegment( BuildingRoof::Mesh &mesh, const QgsPointXY &a, const QgsPointXY &b,
                          const RoofPatch &left, const RoofPatch &right )
  {
    if ( a == b )
      return;
    if ( left.facets.isEmpty() && right.facets.isEmpty() )
    {
      appendStep( mesh, a, b, left.height, left.height, right.height, right.height );
      return;
    }

    const QgsGeometry edge = QgsGeometry::fromPolylineXY( { a, b } );
    const double tolerance = std::max( geometryTolerance( left.geometry ), geometryTolerance( right.geometry ) );
    QgsRectangle edgeBounds = edge.boundingBox();
    edgeBounds.grow( tolerance );
    QVector<double> cuts{ 0.0, 1.0 };
    const auto appendCuts = [&]( const RoofPatch &patch ) {
      for ( const RoofFacet &facet : patch.facets )
      {
        if ( !facet.geometry.boundingBox().intersects( edgeBounds ) )
          continue;
        const QgsGeometry overlap = facet.geometry.intersection( edge );
        auto vertices = overlap.vertices();
        while ( vertices.hasNext() )
        {
          cuts.append( segmentParameter( a, b, QgsPointXY( vertices.next() ) ) );
        }
        auto facetVertices = facet.geometry.vertices();
        while ( facetVertices.hasNext() )
        {
          const QgsPointXY point( facetVertices.next() );
          const double t = segmentParameter( a, b, point );
          if ( point.distance( interpolate( a, b, t ) ) <= tolerance )
            cuts.append( t );
        }
      }
    };
    appendCuts( left );
    appendCuts( right );
    std::sort( cuts.begin(), cuts.end() );

    const auto surfaceAt = []( const RoofPatch &patch, const QgsGeometry &point ) -> const RoofFacet * {
      const double tolerance = geometryTolerance( patch.geometry );
      for ( const RoofFacet &facet : patch.facets )
      {
        const double distance = facet.geometry.distance( point );
        if ( distance >= 0.0 && distance <= tolerance )
          return &facet;
      }
      return nullptr;
    };
    for ( int i = 1; i < cuts.size(); ++i )
    {
      if ( cuts[i] - cuts[i - 1] <= 1e-10 )
        continue;
      const QgsPointXY start = interpolate( a, b, cuts[i - 1] );
      const QgsPointXY end = interpolate( a, b, cuts[i] );
      const QgsGeometry middle = QgsGeometry::fromPointXY( interpolate( start, end, 0.5 ) );
      const RoofFacet *leftFacet = surfaceAt( left, middle );
      const RoofFacet *rightFacet = surfaceAt( right, middle );
      appendStep( mesh, start, end,
                  leftFacet ? leftFacet->heightAt( start ) : left.height, leftFacet ? leftFacet->heightAt( end ) : left.height,
                  rightFacet ? rightFacet->heightAt( start ) : right.height, rightFacet ? rightFacet->heightAt( end ) : right.height );
    }
  }

  void appendExteriorWalls( BuildingRoof::Mesh &mesh, int firstRoofIndex, const QgsGeometryEngine &footprintEdge, double floor )
  {
    const int lastRoofIndex = mesh.indices.size();
    for ( int index = firstRoofIndex; index < lastRoofIndex; index += 3 )
    {
      const std::array<QgsPoint, 3> triangle{ mesh.vertices[mesh.indices[index]], mesh.vertices[mesh.indices[index + 1]],
                                              mesh.vertices[mesh.indices[index + 2]] };
      for ( int side : { 0, 1, 2 } )
      {
        const QgsPoint &a = triangle[side], &b = triangle[( side + 1 ) % 3];
        const QgsPointXY start( a ), end( b );
        if ( start == end )
          continue;
        const QgsGeometry edge = QgsGeometry::fromPolylineXY( { start, end } );
        if ( footprintEdge.contains( edge.constGet() ) )
          appendStep( mesh, start, end, a.z(), b.z(), floor, floor );
      }
    }
  }

  void appendSharedWalls( BuildingRoof::Mesh &mesh, const RoofPatch &left, const RoofPatch &right )
  {
    const QgsGeometry &edgeBoundary = right.boundary;
    if ( edgeBoundary.isEmpty() )
      return;
    const QgsMultiPolygonXY polygons = left.geometry.forcePolygonCounterClockwise().asMultiPolygon();
    auto engine = QgsGeometry::createGeometryEngine( edgeBoundary.constGet() );
    engine->prepareGeometry();
    for ( const QgsPolygonXY &polygon : polygons )
    {
      for ( const QgsPolylineXY &ring : polygon )
      {
        for ( int i = 1; i < ring.size(); ++i )
        {
          const QgsPointXY a = ring[i - 1], b = ring[i];
          const QgsGeometry edge = QgsGeometry::fromPolylineXY( { a, b } );
          if ( !engine->intersects( edge.constGet() ) )
            continue;
          const QgsGeometry shared = edge.intersection( edgeBoundary );
          for ( const QgsGeometry &part : shared.asGeometryCollection() )
          {
            const QgsPolylineXY line = part.asPolyline();
            for ( int j = 1; j < line.size(); ++j )
            {
              QgsPointXY start = line[j - 1], end = line[j];
              if ( ( end.x() - start.x() ) * ( b.x() - a.x() ) + ( end.y() - start.y() ) * ( b.y() - a.y() ) < 0 )
                std::swap( start, end );
              appendWallSegment( mesh, start, end, left, right );
            }
          }
        }
      }
    }
  }

  bool appendPatch( BuildingRoof::Mesh &mesh, const RoofPatch &patch )
  {
    if ( patch.facets.isEmpty() )
      return appendSurface( mesh, patch.geometry, patch.height, true );
    const double minimumArea = areaTolerance( patch.geometry );
    const bool wholeRoof = patch.geometry.isGeosEqual( patch.slopedArea );
    for ( const RoofFacet &facet : patch.facets )
    {
      if ( !facet.geometry.boundingBox().intersects( patch.geometry.boundingBox() ) )
        continue;
      const QgsGeometry clipped = wholeRoof ? facet.geometry : polygonalPart( facet.geometry.intersection( patch.geometry ) );
      if ( clipped.isEmpty() || clipped.area() <= minimumArea )
        continue;
      if ( !appendSurface( mesh, clipped, patch.height, true, &facet ) )
        return false;
    }
    if ( wholeRoof )
      return true;
    const QgsGeometry difference = patch.geometry.difference( patch.slopedArea );
    if ( difference.isNull() )
      return false;
    const QgsGeometry flat = polygonalPart( difference );
    return flat.isEmpty() || flat.area() <= minimumArea || appendSurface( mesh, flat, patch.height, true );
  }

  void appendPlaneCreases( BuildingRoof::Mesh &mesh, const RoofPatch &patch )
  {
    const double tolerance = geometryTolerance( patch.geometry );
    QVector<QgsGeometry> edges;
    for ( const RoofFacet &facet : patch.facets )
    {
      const QgsGeometry face = polygonalPart( patch.geometry.intersection( facet.geometry ) );
      edges.append( face.isEmpty() ? QgsGeometry() : boundaryOf( face ) );
    }
    for ( int i = 0; i < edges.size(); ++i )
    {
      if ( edges[i].isEmpty() )
        continue;
      const RoofFacet &first = patch.facets[i];
      for ( int j = 0; j < i; ++j )
      {
        const RoofFacet &second = patch.facets[j];
        if ( edges[j].isEmpty()
             || std::hypot( first.gradientX - second.gradientX, first.gradientY - second.gradientY ) <= 1e-10 )
          continue;
        QgsRectangle bounds = edges[i].boundingBox();
        bounds.grow( tolerance );
        if ( !bounds.intersects( edges[j].boundingBox() ) )
          continue;
        QgsGeometry seam = edges[i].intersection( edges[j] );
        if ( seam.length() <= tolerance )
        {
          // Independently clipped faces can have round-off on the same cut line.
          seam = edges[i].intersection( edges[j].buffer( tolerance, 1 ) );
        }
        for ( const QgsGeometry &part : seam.asGeometryCollection() )
        {
          const QgsPolylineXY line = part.asPolyline();
          for ( int k = 1; k < line.size(); ++k )
          {
            const auto &a = line[k - 1], &b = line[k];
            const double za = first.heightAt( a ), zb = first.heightAt( b );
            if ( a.distance( b ) <= 4.0 * tolerance || std::abs( za - second.heightAt( a ) ) > 1e-6
                 || std::abs( zb - second.heightAt( b ) ) > 1e-6 )
              continue;
            mesh.structureLines.append( { QgsPoint( a.x(), a.y(), za ), QgsPoint( b.x(), b.y(), zb ) } );
          }
        }
      }
    }
  }

  void appendFootprintLines( BuildingRoof::Mesh &mesh, const QgsGeometry &footprint, const QVector<RoofPatch> &patches, double floor )
  {
    const double tolerance = geometryTolerance( footprint );
    const auto polygons = footprint.asMultiPolygon();
    for ( const QgsPolygonXY &polygon : polygons )
    {
      for ( const QgsPolylineXY &ring : polygon )
      {
        const int count = ring.size() - 1;
        for ( int i = 0; i < count; ++i )
        {
          const auto &point = ring[i], &next = ring[i + 1], &previous = ring[( i + count - 1 ) % count];
          const QgsPoint foot( point.x(), point.y(), floor );
          mesh.structureLines.append( { foot, QgsPoint( next.x(), next.y(), floor ) } );
          if ( std::abs( cross( previous, point, next ) ) <= tolerance * std::max( point.distance( previous ), point.distance( next ) ) )
            continue;
          const QgsGeometry query = QgsGeometry::fromPointXY( point );
          double height = floor;
          for ( const RoofPatch &patch : patches )
          {
            if ( patch.geometry.distance( query ) > tolerance )
              continue;
            bool onSlope = false;
            for ( const RoofFacet &facet : patch.facets )
            {
              if ( facet.geometry.distance( query ) <= tolerance )
              {
                height = std::max( height, facet.heightAt( point ) );
                onSlope = true;
              }
            }
            if ( !onSlope )
              height = std::max( height, patch.height );
          }
          if ( height > floor + 1e-7 )
            mesh.structureLines.append( { foot, QgsPoint( point.x(), point.y(), height ) } );
        }
      }
    }
  }

  double estimateElevation( const QgsGeometry &region, QgsVectorLayer *layer, const QString &cloudId, double fallback )
  {
    auto *cloud = qobject_cast<QgsPointCloudLayer *>( QgsProject::instance()->mapLayer( cloudId ) );
    if ( !cloud )
      return fallback;
    QgsGeometry cloudRegion = region;
    try
    {
      const QgsCoordinateTransform transform( layer->crs(), cloud->crs(), QgsProject::instance() );
      if ( transform.isValid() && cloudRegion.transform( transform ) != Qgis::GeometryOperationResult::Success )
        return fallback;
    }
    catch ( const QgsCsException & )
    {
      return fallback;
    }

    bool complete = false;
    const auto samples = RoofModelPipeline::collectSamples( cloudRegion, cloudId, &complete );
    if ( !complete || samples.isEmpty() )
      return fallback;
    const QgsGeometry boundary = boundaryOf( cloudRegion );
    auto engine = QgsGeometry::createGeometryEngine( boundary.constGet() );
    QVector<QPair<double, double>> ranked;
    ranked.reserve( samples.size() );
    for ( const BuildingRoof::RoofSample &sample : samples )
    {
      const double distance = engine->distance( &sample.point );
      if ( distance >= 0 && std::isfinite( sample.point.z() ) )
        ranked.append( qMakePair( distance, sample.point.z() ) );
    }
    if ( ranked.isEmpty() )
      return fallback;
    // Use the median of samples closest to the perimeter to reduce the effect of isolated points.
    const int count = std::min( ranked.size(), std::max( 12, ranked.size() / 10 ) );
    std::partial_sort( ranked.begin(), ranked.begin() + count, ranked.end() );
    QVector<double> heights;
    heights.reserve( count );
    for ( int i = 0; i < count; ++i )
      heights.append( ranked[i].second );
    auto median = heights.begin() + count / 2;
    std::nth_element( heights.begin(), median, heights.end() );
    return *median;
  }
}

RoofInteraction::RoofInteraction( QgsMapCanvas *canvas, QgisInterface *iface, Ui::roofedit &ui,
                                  QObject *context, std::function<void()> changed, std::function<void()> previewChanged )
  : mCanvas( canvas )
  , mIface( iface )
  , mUi( ui )
  , mChanged( std::move( changed ) )
  , mPreviewChanged( std::move( previewChanged ) )
{
  QObject::connect( mUi.drawRegionCheckBox, &QCheckBox::toggled, context, [this]( bool enabled ) { setDrawing( enabled ); } );
  QObject::connect( mUi.eavesElevationSpinBox, qOverload<double>( &QDoubleSpinBox::valueChanged ), context, [this]( double height ) {
    const int index = mAdjustingRidge ? mRidgeRegion : mAdjustingRegionIndex;
    if ( index < 0 )
      return;
    QVariantMap region = mRegions[index].toMap();
    region.insert( QStringLiteral( "height" ), height );
    if ( setRegion( index, std::move( region ) ) )
      mChanged();
  } );
  QObject::connect( mUi.drawRidgeCheckBox, &QCheckBox::toggled, context, [this]( bool enabled ) { setRidgeDrawing( enabled ); } );
  QObject::connect( mUi.roofLiftHeightSpinBox, qOverload<double>( &QDoubleSpinBox::valueChanged ), context, [this]( double lift ) {
    if ( !mAdjustingRidge )
      return;
    if ( lift <= 0.0 )
    {
      const QSignalBlocker blocker( mUi.roofLiftHeightSpinBox );
      mUi.roofLiftHeightSpinBox->setValue( 0.01 );
    }
    QVariantMap region = mRegions[mRidgeRegion].toMap();
    region.insert( QStringLiteral( "ridge_lift" ), mUi.roofLiftHeightSpinBox->value() );
    if ( setRegion( mRidgeRegion, std::move( region ) ) )
      mChanged();
  } );
  mUi.eavesElevationSpinBox->setKeyboardTracking( false );
  mUi.roofLiftHeightSpinBox->setKeyboardTracking( false );
  QObject::connect( mUi.deleteRidgeButton, &QPushButton::clicked, context, [this] { deleteSelectedRidge(); } );
  mUi.drawRidgeCheckBox->setToolTip( QObject::tr( "勾选后显示绘制区域的边界。左键在区域内画屋脊，右键完成并用点云拟合。双坡的两端贴边，可加一个内部折点。四面坡画两个内部端点。Ctrl＋左键关闭吸附，Alt＋左键重画。" ) );
  mUi.drawRegionCheckBox->setToolTip( QObject::tr( "右键选区，点未覆盖处可选全部余区。按 Esc 或右键点轮廓外退出选区。左键落点，Alt＋左键可从选中边界起笔或沿边绘制。点回起点闭合，Enter 完成区域或分割线。选区后滚轮调檐口。" ) );
}

RoofInteraction::~RoofInteraction() = default;

void RoofInteraction::load( QgsVectorLayer *layer, const QgsFeature &building, const RoofModelSettings::Selection &saved )
{
  reset();
  mLayer = layer;
  mFootprint = polygonalPart( building.geometry() );
  mRegions = saved.regions;
  mHadSavedRegions = !mRegions.isEmpty();
  updateStoredRidges();
  const double height = building.fields().lookupField( QStringLiteral( "height" ) ) >= 0
                        ? building.attribute( QStringLiteral( "height" ) ).toDouble() : 0.0;
  mBaseHeight = mRegions.isEmpty() ? ( height > 0.0 && std::isfinite( height ) ? height : 3.0 ) : saved.regionBaseHeight;
  mUi.eavesElevationSpinBox->setValue( mRegions.isEmpty() ? mBaseHeight : mRegions.last().toMap().value( QStringLiteral( "height" ) ).toDouble() );
}

void RoofInteraction::reset()
{
  const QSignalBlocker blocker( mUi.drawRegionCheckBox );
  mUi.drawRegionCheckBox->setChecked( false );
  const QSignalBlocker ridgeBlocker( mUi.drawRidgeCheckBox );
  mUi.drawRidgeCheckBox->setChecked( false );
  cancelRidge();
  mStoredRidgeOutline.reset();
  mOutline.reset();
  mBoundaryOutline.reset();
  mBoundaryHighlight.reset();
  mVertexHandles.reset();
  mRegionBoundary = {};
  mMapRegionBoundary = {};
  mBoundaryPaths.clear();
  mFootprint = {};
  mRegions.clear();
  mUndoHistory.clear();
  mHadSavedRegions = false;
  cancelDraft();
  mLayer.clear();
}

void RoofInteraction::clearRegions()
{
  const QSignalBlocker blocker( mUi.drawRegionCheckBox );
  mUi.drawRegionCheckBox->setChecked( false );
  mUi.drawRidgeCheckBox->setChecked( false );
  mRegions.clear();
  mUndoHistory.clear();
  mHadSavedRegions = false;
  updateRegionBoundaries();
  updateStoredRidges();
  cancelDraft();
}

void RoofInteraction::addToSelection( RoofModelSettings::Selection &selection ) const
{
  selection.regions = mRegions;
  selection.regionBaseHeight = mBaseHeight;
}

bool RoofInteraction::addDragToSelection( RoofModelSettings::Selection &selection )
{
  if ( mRidgeDragStart && !mRidgeDragPreview.isEmpty() )
  {
    const auto widths = mRidgeType == RoofType::Gabled
                        ? movedGableWidths( mRidgeVertices, mRidgeDragPreview, mGableWidths ) : GableSideWidths{};
    selection.regions[mRidgeRegion] = ridgeRegionValue( mRidgeDragPreview, widths );
    return true;
  }
  return mVertexPress && mMovingVertex && mVertexPreview
         && modifyRegionVertex( VertexEdit::Move, *mMovingVertex, *mVertexPreview, &selection.regions );
}

void RoofInteraction::notifyPreviewChanged()
{
  if ( mPreviewChanged )
    mPreviewChanged();
}

bool RoofInteraction::hasBuilding() const
{
  return mLayer && !mFootprint.isEmpty();
}

bool RoofInteraction::setRegions( QVariantList regions )
{
  if ( regions == mRegions )
    return false;
  mUndoHistory.append( UndoState{ mRegions, selectedRegion(), mAdjustingRidge } );
  mRegions = std::move( regions );
  return true;
}

bool RoofInteraction::setRegion( int index, QVariantMap region )
{
  QVariantList regions = mRegions;
  regions[index] = std::move( region );
  return setRegions( std::move( regions ) );
}

int RoofInteraction::selectedRegion() const
{
  return mAdjustingRidge ? mRidgeRegion : mAdjustingRegionIndex;
}

void RoofInteraction::updateDeleteRidgeButton()
{
  const int index = selectedRegion();
  mUi.deleteRidgeButton->setEnabled( index >= 0 && index < mRegions.size()
                                   && mRegions[index].toMap().contains( QStringLiteral( "ridge" ) ) );
}

void RoofInteraction::deleteSelectedRidge()
{
  const int index = selectedRegion();
  if ( index < 0 || index >= mRegions.size() || mVertexPress || mRidgeDragStart )
    return;
  QVariantMap region = mRegions[index].toMap();
  if ( !region.contains( QStringLiteral( "ridge" ) ) )
    return;
  for ( const QString &key : { QStringLiteral( "ridge" ), QStringLiteral( "ridge_lift" ), QStringLiteral( "ridge_type" ),
                               QStringLiteral( "ridge_support" ), QStringLiteral( "fitted_gable" ), QStringLiteral( "gable_widths" ) } )
    region.remove( key );
  setRegion( index, std::move( region ) );
  // Keep the original region selected so its eaves can still be adjusted.
  mUi.drawRegionCheckBox->setChecked( true );
  selectRegion( index );
  const QSignalBlocker liftBlocker( mUi.roofLiftHeightSpinBox );
  mUi.roofLiftHeightSpinBox->setValue( 0.0 );
  updateDeleteRidgeButton();
  updateStoredRidges();
  mChanged();
  message( QObject::tr( "已删除所选区域的屋脊。Ctrl+Z 可撤销。" ) );
}

void RoofInteraction::undo()
{
  if ( !hasBuilding() || cancelDrag() )
    return;
  if ( !mVertices.isEmpty() || ( !mAdjustingRidge && !mRidgeVertices.isEmpty() ) )
  {
    removeLastSketchPoint();
    return;
  }
  if ( mUndoHistory.isEmpty() )
    return;

  UndoState state = mUndoHistory.takeLast();
  cancelDraft();
  cancelRidge();
  mRegions = std::move( state.regions );
  if ( mRegions.isEmpty() )
    mUi.drawRidgeCheckBox->setChecked( false );
  if ( state.selectedRegion >= 0 && state.selectedRegion < mRegions.size() )
  {
    if ( state.ridgeSelected && mRegions[state.selectedRegion].toMap().contains( QStringLiteral( "ridge" ) ) )
    {
      mUi.drawRidgeCheckBox->setChecked( true );
      if ( !selectRidgeRegion( state.selectedRegion ) || !restoreRidge() )
      {
        mUi.drawRegionCheckBox->setChecked( true );
        selectRegion( state.selectedRegion );
      }
    }
    else
    {
      mUi.drawRegionCheckBox->setChecked( true );
      selectRegion( state.selectedRegion );
    }
  }
  updateRegionBoundaries();
  updateStoredRidges();
  updateDeleteRidgeButton();
  mChanged();
  message( QObject::tr( "已撤销上一次屋顶编辑。" ) );
}

bool RoofInteraction::hasUnfinishedEdit() const
{
  return !mVertices.isEmpty() || ( !mAdjustingRidge && !mRidgeVertices.isEmpty() ) || mVertexPress || mRidgeDragStart;
}

bool RoofInteraction::hasRemovedSavedRegions() const
{
  return mHadSavedRegions && mRegions.isEmpty();
}

void RoofInteraction::setDrawing( bool enabled )
{
  if ( enabled )
  {
    mUi.drawRidgeCheckBox->setChecked( false );
    if ( mCanvas )
      mCanvas->setFocus();
    message( QObject::tr( "右键选区，左键落点。点回起点闭合，Enter 完成区域或分割线。" ) );
  }
  else
    cancelDraft();
  updateRegionBoundaries();
  updateStoredRidges();
}

void RoofInteraction::cancelDraft()
{
  const bool wasPreviewing = mVertexPreview.has_value();
  mVertices.clear();
  mMiddlePress.reset();
  mAdjustingRegion = {};
  mAdjustingRegionIndex = -1;
  mMovingVertex.reset();
  mVertexPreview.reset();
  mVertexPress.reset();
  highlightBoundary( -1 );
  updateOutline();
  updateDeleteRidgeButton();
  if ( wasPreviewing )
    notifyPreviewChanged();
}

bool RoofInteraction::cancelDrag()
{
  mMiddlePress.reset();
  if ( mVertexPress )
  {
    mMovingVertex.reset();
    mVertexPress.reset();
    mVertexPreview.reset();
    updateOutline();
    notifyPreviewChanged();
    return true;
  }
  if ( mRidgeDragStart )
  {
    mRidgeDragStart.reset();
    mDraggedRidgeVertex = -1;
    mRidgeDragPreview.clear();
    updateRidgeOutline();
    notifyPreviewChanged();
    return true;
  }
  return false;
}

bool RoofInteraction::press( QgsMapMouseEvent *event )
{
  if ( !hasBuilding() || !mCanvas )
    return false;
  const QgsPointXY mapPoint = event->originalMapPoint();
  const bool drawing = mUi.drawRegionCheckBox->isChecked();
  if ( event->button() == Qt::RightButton && mUi.drawRidgeCheckBox->isChecked()
       && !mAdjustingRidge && !mRidgeVertices.isEmpty() )
  {
    startRidgeAdjustment();
    return true;
  }
  if ( event->button() == Qt::RightButton && drawing )
  {
    if ( !switchRegion( mapPoint ) )
      deselectRegion();
    return true;
  }
  if ( event->button() != Qt::LeftButton )
    return false;
  cancelDrag();
  if ( mVertices.isEmpty() && ( mAdjustingRidge || mRidgeVertices.isEmpty() ) )
  {
    const auto &settings = mCanvas->mapSettings();
    const QgsPointXY point = settings.mapToLayerCoordinates( mLayer, mapPoint );
    if ( !mFootprint.intersects( QgsGeometry::fromPointXY( point ) ) )
    {
      QgsPointXY nearest;
      int next = -1;
      if ( boundaryOf( mFootprint ).closestSegmentWithContext( point, nearest, next ) < 0.0
           || settings.layerToMapCoordinates( mLayer, nearest ).distance( mapPoint ) > 8.0 * settings.mapUnitsPerPixel() )
        return false;
    }
  }
  if ( mUi.drawRidgeCheckBox->isChecked() )
    return pressRidge( event );

  if ( mVertices.isEmpty() && event->modifiers() == Qt::NoModifier )
  {
    if ( editRegionVertex( event ) )
      return true;
    if ( drawing )
    {
      // Keep the selection through the first press of an edge double-click.
      if ( canMoveRegionVertices() && regionEdgeAt( activeRegionShape(), mapPoint ) )
        return true;
    }
    else
    {
      const int ridge = ridgeAt( mapPoint );
      if ( ridge >= 0 )
      {
        pickRidge( ridge, event );
        return true;
      }
      const int region = regionAt( mapPoint );
      if ( region >= 0 )
      {
        mUi.drawRegionCheckBox->setChecked( true );
        selectRegion( region );
        editRegionVertex( event );
        return true;
      }
    }
  }
  if ( !drawing )
    return false;
  if ( event->modifiers().testFlag( Qt::ShiftModifier ) )
  {
    switchRegion( mapPoint );
    return true;
  }
  if ( mVertices.isEmpty() )
    cancelDraft();

  RegionPoint vertex = regionPoint( event );
  const auto &settings = mCanvas->mapSettings();
  const bool closesRegion = mVertices.size() >= 3
                            && settings.layerToMapCoordinates( mLayer, mVertices.first().point ).distance( mapPoint )
                               <= 8.0 * settings.mapUnitsPerPixel();
  if ( closesRegion )
    vertex = mVertices.first();
  if ( event->modifiers().testFlag( Qt::AltModifier ) && !mVertices.isEmpty() )
  {
    const auto trace = traceBoundary( mVertices.last(), vertex );
    if ( trace.isEmpty() )
    {
      message( QObject::tr( "沿边绘制时，请在同一条边界上落下起点和终点。" ), Qgis::MessageLevel::Warning );
      return true;
    }
    mVertices += trace;
  }
  else if ( mVertices.isEmpty() || mVertices.last().point != vertex.point )
    mVertices.append( vertex );

  if ( closesRegion )
    finishRegionSketch( false );
  else
  {
    highlightBoundary( vertex.boundaryVertex );
    updateOutline();
  }
  return true;
}

void RoofInteraction::finishRegionSketch( bool allowSplit )
{
  QgsPolylineXY vertices;
  vertices.reserve( mVertices.size() );
  for ( const RegionPoint &vertex : std::as_const( mVertices ) )
    vertices.append( vertex.point );

  if ( allowSplit && vertices.size() >= 2 && vertices.first() != vertices.last()
       && mVertices.first().boundaryVertex >= 0 && mVertices.last().boundaryVertex >= 0 )
  {
    if ( splitRegions( vertices ) )
    {
      cancelDraft();
      updateRegionBoundaries();
      updateStoredRidges();
      mChanged();
      message( QObject::tr( "区域已分割。右键选择一块调高程，左键可继续画分割线。" ) );
    }
    else
      message( QObject::tr( "分割线未切出新区域。请让线穿过区域内部，可按退格撤回顶点。" ), Qgis::MessageLevel::Warning );
    return;
  }
  if ( vertices.size() < 3 )
  {
    message( QObject::tr( "分割线的两端需要吸附边界。绘制闭合区域需要至少三个不同的顶点。" ), Qgis::MessageLevel::Warning );
    return;
  }
  if ( vertices.first() != vertices.last() )
    vertices.append( vertices.first() );
  const QgsGeometry region = QgsGeometry::fromPolygonXY( { vertices } );
  if ( region.area() <= 0.0 || !region.isGeosValid() )
  {
    message( QObject::tr( "绘制区域无效。可按退格撤回顶点，再重新闭合。" ), Qgis::MessageLevel::Warning );
    return;
  }
  const QgsGeometry clipped = polygonalPart( region.intersection( mFootprint ) );
  if ( clipped.isEmpty() || clipped.area() <= 0.0 )
  {
    message( QObject::tr( "绘制区域与所选建筑没有重叠。" ), Qgis::MessageLevel::Warning );
    return;
  }
  addRegion( clipped );
}

int RoofInteraction::regionAt( const QgsPointXY &mapPoint ) const
{
  const QgsGeometry point = QgsGeometry::fromPointXY( mCanvas->mapSettings().mapToLayerCoordinates( mLayer, mapPoint ) );
  if ( !mFootprint.intersects( point ) )
    return -1;
  for ( int index = mRegions.size() - 1; index >= 0; --index )
  {
    const QgsGeometry region = regionGeometry( mRegions[index].toMap() );
    if ( region.intersects( point ) )
      return index;
  }
  return -1;
}

bool RoofInteraction::switchRegion( const QgsPointXY &mapPoint )
{
  if ( !mVertices.isEmpty() || mVertexPress )
  {
    message( QObject::tr( "请先完成当前草图，或按 Esc 取消。" ) );
    return true;
  }
  const int index = regionAt( mapPoint );
  if ( index >= 0 )
  {
    selectRegion( index );
    return true;
  }
  if ( !mFootprint.intersects( QgsGeometry::fromPointXY( mCanvas->mapSettings().mapToLayerCoordinates( mLayer, mapPoint ) ) ) )
    return false;
  const auto remaining = uncoveredRegion( mFootprint, mRegions, 0 );
  if ( remaining && !remaining->isEmpty() )
    addRegion( *remaining, true );
  return true;
}

void RoofInteraction::selectRegion( int index )
{
  if ( index == mAdjustingRegionIndex )
    return;
  const QgsGeometry region = regionGeometry( mRegions[index].toMap() );
  const auto visible = uncoveredRegion( polygonalPart( mFootprint.intersection( region ) ), mRegions, index + 1 );
  if ( !visible || visible->isEmpty() )
    return;
  cancelDraft();
  mAdjustingRegionIndex = index;
  mAdjustingRegion = *visible;
  const QVariantMap values = mRegions[index].toMap();
  const QSignalBlocker eaveBlocker( mUi.eavesElevationSpinBox );
  const QSignalBlocker liftBlocker( mUi.roofLiftHeightSpinBox );
  mUi.eavesElevationSpinBox->setValue( values.value( QStringLiteral( "height" ) ).toDouble() );
  mUi.roofLiftHeightSpinBox->setValue( values.value( QStringLiteral( "ridge_lift" ) ).toDouble() );
  updateOutline();
  updateDeleteRidgeButton();
  message( QObject::tr( "右键切区，Esc 或右键点轮廓外退出选区。滚轮调檐口，可拖动顶点或双击改点。" ) );
}

void RoofInteraction::deselectRegion()
{
  if ( !hasSelectedRegion() )
    return;
  cancelDraft();
  message( QObject::tr( "已退出选区。滚轮缩放视图，可左键绘制新区域或右键重新选区。" ) );
}

void RoofInteraction::addRegion( const QgsGeometry &region, bool remainder )
{
  if ( !remainder )
  {
    for ( int index = mRegions.size() - 1; index >= 0; --index )
    {
      if ( regionGeometry( mRegions[index].toMap() ).isGeosEqual( region ) )
      {
        cancelDraft();
        selectRegion( index );
        return;
      }
    }
  }

  const double height = remainder && !mRegions.isEmpty()
                        ? mBaseHeight : estimateElevation( region, mLayer, mUi.pointCloudComboBox->currentData().toString(), mBaseHeight );
  // The base stays below the drawn patches so its visible shape follows their remainder.
  QVariantList regions = mRegions;
  if ( remainder )
    regions.prepend( regionValue( mFootprint, height ) );
  else
    regions.append( regionValue( region, height ) );
  setRegions( std::move( regions ) );
  cancelDraft();
  updateRegionBoundaries();
  updateStoredRidges();
  selectRegion( remainder ? 0 : mRegions.size() - 1 );
  mChanged();
}

bool RoofInteraction::canMoveRegionVertices() const
{
  if ( mAdjustingRegionIndex < 0 )
    return false;
  return !activeRegionShape().isGeosEqual( mFootprint );
}

QgsGeometry RoofInteraction::activeRegionShape() const
{
  return regionGeometry( mRegions[mAdjustingRegionIndex].toMap() );
}

std::optional<QgsPointXY> RoofInteraction::regionVertexAt( const QgsGeometry &shape, const QgsPointXY &mapPoint ) const
{
  const QgsMapSettings &settings = mCanvas->mapSettings();
  double nearest = 8.0 * settings.mapUnitsPerPixel();
  std::optional<QgsPointXY> found;
  auto vertices = shape.vertices();
  while ( vertices.hasNext() )
  {
    const QgsPointXY point( vertices.next() );
    const double distance = settings.layerToMapCoordinates( mLayer, point ).distance( mapPoint );
    if ( distance < nearest )
    {
      nearest = distance;
      found = point;
    }
  }
  return found;
}

std::optional<QgsPointXY> RoofInteraction::regionEdgeAt( const QgsGeometry &shape, const QgsPointXY &mapPoint ) const
{
  const QgsMapSettings &settings = mCanvas->mapSettings();
  const QgsPointXY point = settings.mapToLayerCoordinates( mLayer, mapPoint );
  QgsPointXY closest;
  int next = -1;
  if ( boundaryOf( shape ).closestSegmentWithContext( point, closest, next, nullptr, 0.0 ) >= 0.0
       && settings.layerToMapCoordinates( mLayer, closest ).distance( mapPoint ) <= 8.0 * settings.mapUnitsPerPixel() )
    return closest;
  return std::nullopt;
}

bool RoofInteraction::editRegionVertex( QgsMapMouseEvent *event )
{
  if ( !canMoveRegionVertices() )
    return false;
  mMovingVertex = regionVertexAt( activeRegionShape(), event->originalMapPoint() );
  mVertexPress.reset();
  mVertexPreview.reset();
  if ( mMovingVertex )
    mVertexPress = event->originalPixelPoint();
  updateOutline();
  return mMovingVertex.has_value();
}

bool RoofInteraction::doubleClick( QgsMapMouseEvent *event )
{
  if ( !mLayer || !mCanvas || event->button() != Qt::LeftButton || event->modifiers() != Qt::NoModifier )
    return false;
  if ( !mUi.drawRegionCheckBox->isChecked() || !canMoveRegionVertices() )
    return false;
  mVertexPress.reset();
  mVertexPreview.reset();
  mMovingVertex.reset();
  const QgsGeometry shape = activeRegionShape();
  const auto vertex = regionVertexAt( shape, event->originalMapPoint() );
  bool changed = false;
  if ( vertex )
  {
    changed = modifyRegionVertex( VertexEdit::Remove, *vertex );
    if ( !changed )
      message( QObject::tr( "无法删除此顶点。请保留原始轮廓、有效多边形和相邻分界。" ), Qgis::MessageLevel::Warning );
  }
  else if ( const auto edge = regionEdgeAt( shape, event->originalMapPoint() ) )
  {
    changed = modifyRegionVertex( VertexEdit::Insert, *edge );
    if ( changed )
      mMovingVertex = *edge;
    else
      message( QObject::tr( "无法在这条边上添加顶点。" ), Qgis::MessageLevel::Warning );
  }
  updateOutline();
  if ( changed )
    mChanged();
  return true;
}

void RoofInteraction::canvasEvent( QEvent *event )
{
  if ( !mCanvas || !mLayer || mFootprint.isEmpty() || !mUi.drawRegionCheckBox->isChecked() )
    return;
  switch ( event->type() )
  {
    case QEvent::MouseButtonPress:
    {
      const auto *mouse = static_cast<QMouseEvent *>( event );
      if ( mouse->button() == Qt::MiddleButton && mouse->modifiers() == Qt::NoModifier )
        mMiddlePress = mouse->pos();
      break;
    }
    case QEvent::MouseMove:
    {
      const auto *mouse = static_cast<QMouseEvent *>( event );
      if ( mMiddlePress && ( !mouse->buttons().testFlag( Qt::MiddleButton )
                            || ( mouse->pos() - *mMiddlePress ).manhattanLength() >= QApplication::startDragDistance() ) )
        mMiddlePress.reset();
      break;
    }
    case QEvent::MouseButtonRelease:
    {
      const auto *mouse = static_cast<QMouseEvent *>( event );
      if ( mouse->button() != Qt::MiddleButton || !mMiddlePress )
        break;
      const QPoint start = *mMiddlePress;
      mMiddlePress.reset();
      if ( mouse->modifiers() == Qt::NoModifier && ( mouse->pos() - start ).manhattanLength() < QApplication::startDragDistance() )
        switchRegion( mCanvas->getCoordinateTransform()->toMapCoordinates( start ) );
      break;
    }
    default:
      break;
  }
}

bool RoofInteraction::modifyRegionVertex( VertexEdit operation, const QgsPointXY &from, QgsPointXY to, QVariantList *preview )
{
  const QVariantList &current = mRegions;
  const bool shared = isPartition( current[mAdjustingRegionIndex] );
  const double tolerance = geometryTolerance( mFootprint );
  const QgsGeometry fromPoint = QgsGeometry::fromPointXY( from );
  if ( shared && operation != VertexEdit::Insert )
  {
    const QgsGeometry boundary = boundaryOf( mFootprint );
    QgsPointXY closest;
    int next = -1;
    const double distance = boundary.closestSegmentWithContext( from, closest, next, nullptr, 0.0 );
    if ( distance >= 0.0 && distance <= tolerance * tolerance && next > 0 )
    {
      const QgsPointXY a( boundary.vertexAt( next - 1 ) ), b( boundary.vertexAt( next ) );
      if ( from.distance( a ) <= tolerance || from.distance( b ) <= tolerance )
        return false;
      if ( operation == VertexEdit::Move )
      {
        const double t = segmentParameter( a, b, to );
        to = interpolate( a, b, t );
        if ( to.distance( a ) <= tolerance || to.distance( b ) <= tolerance )
          return false;
      }
    }
  }
  if ( operation == VertexEdit::Move && from.distance( to ) <= tolerance )
    return false;

  const double minimumArea = areaTolerance( mFootprint );
  const QgsGeometry selected = activeRegionShape();
  const QgsGeometry selectedBoundary = boundaryOf( selected );
  QVariantList updated = current;
  QVector<QgsGeometry> oldPartition;
  QVector<QgsGeometry> newPartition;
  QVector<QgsGeometry> oldAdjacent;
  QVector<QgsGeometry> newAdjacent;
  double partitionArea = 0.0;
  bool changed = false;
  for ( int index = 0; index < current.size(); ++index )
  {
    if ( shared != isPartition( current[index] ) )
      continue;
    QVariantMap entry = current[index].toMap();
    QgsGeometry shape = regionGeometry( entry );
    if ( shape.isEmpty() )
      continue;
    const QgsGeometry boundary = boundaryOf( shape );
    const bool touches = boundary.distance( fromPoint ) <= tolerance;
    if ( shared )
      oldPartition.append( shape );
    else if ( index != mAdjustingRegionIndex )
    {
      if ( !touches )
        continue;
      const QgsGeometry seam = selectedBoundary.intersection( boundary );
      const QgsGeometry overlap = selected.intersection( shape );
      if ( seam.isEmpty() || seam.length() <= tolerance || seam.distance( fromPoint ) > tolerance
           || overlap.isNull() || overlap.area() > minimumArea )
        continue;
    }
    if ( !shared )
      oldAdjacent.append( shape );
    bool changedShape = false;
    if ( touches )
    {
      int vertex, previous, next;
      double distance;
      shape.closestVertex( from, vertex, previous, next, distance );
      const bool hasVertex = vertex >= 0 && distance <= tolerance * tolerance;
      if ( operation == VertexEdit::Insert && !hasVertex )
      {
        if ( !shape.addTopologicalPoint( QgsPoint( from ), tolerance ) )
          return false;
        changedShape = true;
      }
      else if ( operation == VertexEdit::Remove && hasVertex )
      {
        QgsVertexId id;
        if ( !shape.vertexIdFromVertexNr( vertex, id ) || shape.constGet()->vertexCount( id.part, id.ring ) <= 4 || !shape.deleteVertex( vertex ) )
          return false;
        changedShape = true;
      }
      else if ( operation == VertexEdit::Move )
      {
        // A T-junction may be a vertex only on the smaller neighboring region.
        if ( !hasVertex )
        {
          if ( !shape.addTopologicalPoint( QgsPoint( from ), tolerance ) )
            return false;
          shape.closestVertex( from, vertex, previous, next, distance );
        }
        if ( vertex < 0 || !shape.moveVertex( to.x(), to.y(), vertex ) )
          return false;
        changedShape = true;
      }
    }
    if ( changedShape )
    {
      if ( !shape.isGeosValid() || shape.area() <= 0.0 )
        return false;
      const QgsGeometry outside = shape.difference( mFootprint );
      if ( outside.isNull() || outside.area() > minimumArea )
        return false;

      const QgsGeometry ridge = storedGeometry( entry, QStringLiteral( "ridge" ) );
      if ( !ridge.isEmpty() && !entry.contains( QStringLiteral( "ridge_support" ) ) )
      {
        RoofPatch candidate{ shape, {}, 0.0, {}, {}, {} };
        try
        {
          if ( !addRidgeFacets( candidate, shape, ridge.asPolyline(), entry.value( QStringLiteral( "ridge_lift" ) ).toDouble(),
                                storedRidgeType( entry ), storedGableWidths( entry ) ) )
            return false;
        }
        catch ( const QgsNotSupportedException & )
        {
          return false;
        }
      }
      entry.insert( QStringLiteral( "wkb" ), geometryValue( shape ) );
      updated[index] = entry;
      changed = true;
    }
    if ( shared )
    {
      newPartition.append( shape );
      partitionArea += shape.area();
    }
    else
      newAdjacent.append( shape );
  }
  if ( !changed )
    return false;
  if ( shared )
  {
    const QgsGeometry original = QgsGeometry::unaryUnion( oldPartition );
    const QgsGeometry revised = QgsGeometry::unaryUnion( newPartition );
    const QgsGeometry change = original.symDifference( revised );
    if ( change.isNull() || change.area() > minimumArea || partitionArea - revised.area() > minimumArea )
      return false;
  }
  else if ( newAdjacent.size() > 1 )
  {
    const QgsGeometry original = QgsGeometry::unaryUnion( oldAdjacent );
    const QgsGeometry revised = QgsGeometry::unaryUnion( newAdjacent );
    const QgsGeometry change = original.symDifference( revised );
    if ( change.isNull() || change.area() > minimumArea )
      return false;
    double area = 0.0;
    for ( const QgsGeometry &part : std::as_const( newAdjacent ) )
      area += part.area();
    if ( area - revised.area() > minimumArea )
      return false;
  }

  const QgsGeometry shape = regionGeometry( updated[mAdjustingRegionIndex].toMap() );
  const auto visible = uncoveredRegion( polygonalPart( mFootprint.intersection( shape ) ), updated, mAdjustingRegionIndex + 1 );
  if ( !visible || visible->isEmpty() )
    return false;
  if ( preview )
  {
    *preview = std::move( updated );
    return true;
  }
  mAdjustingRegion = *visible;
  setRegions( std::move( updated ) );
  updateRegionBoundaries();
  updateStoredRidges();
  return true;
}

RoofInteraction::RegionPoint RoofInteraction::regionPoint( QgsMapMouseEvent *event ) const
{
  const QgsMapSettings &settings = mCanvas->mapSettings();
  if ( !mMapRegionBoundary.isEmpty() && !event->modifiers().testFlag( Qt::ControlModifier ) )
  {
    QgsPointXY closest;
    int vertex = -1;
    const double tolerance = 8.0 * settings.mapUnitsPerPixel();
    const double distance = mMapRegionBoundary.closestSegmentWithContext( event->originalMapPoint(), closest, vertex, nullptr, 0.0 );
    if ( vertex > 0 && distance >= 0.0 && distance <= tolerance * tolerance )
    {
      const QgsPointXY candidate = settings.mapToLayerCoordinates( mLayer, closest );
      const QgsPointXY a( mRegionBoundary.vertexAt( vertex - 1 ) );
      const QgsPointXY b( mRegionBoundary.vertexAt( vertex ) );
      const double startDistance = QgsPointXY( mMapRegionBoundary.vertexAt( vertex - 1 ) ).distance( event->originalMapPoint() );
      const double endDistance = QgsPointXY( mMapRegionBoundary.vertexAt( vertex ) ).distance( event->originalMapPoint() );
      if ( std::min( startDistance, endDistance ) <= tolerance )
        return { startDistance <= endDistance ? a : b, vertex };
      return { a == b ? a : interpolate( a, b, segmentParameter( a, b, candidate ) ), vertex };
    }
  }
  return { settings.mapToLayerCoordinates( mLayer, event->snapPoint() ) };
}

QVector<RoofInteraction::RegionPoint> RoofInteraction::traceBoundary( const RegionPoint &from, const RegionPoint &to ) const
{
  if ( from.boundaryVertex < 0 || to.boundaryVertex < 0 || from.point == to.point )
    return {};
  const double epsilon = geometryTolerance( mFootprint );
  double shortest = std::numeric_limits<double>::max();
  QVector<RegionPoint> trace;
  for ( int part = 0; part < mBoundaryPaths.size(); ++part )
  {
    const QgsPolylineXY &path = mBoundaryPaths[part];
    if ( path.size() < 2 )
      continue;
    QVector<double> offsets{ 0.0 };
    std::array<double, 2> distances{ epsilon, epsilon }, positions{ -1.0, -1.0 };
    const std::array<QgsPointXY, 2> endpoints{ from.point, to.point };
    for ( int i = 1; i < path.size(); ++i )
    {
      const double length = path[i - 1].distance( path[i] );
      offsets.append( offsets.last() + length );
      if ( length <= 0.0 )
        continue;
      for ( int end : { 0, 1 } )
      {
        const double t = segmentParameter( path[i - 1], path[i], endpoints[end] );
        const double distance = interpolate( path[i - 1], path[i], t ).distance( endpoints[end] );
        if ( distance <= distances[end] )
        {
          distances[end] = distance;
          positions[end] = offsets[i - 1] + t * length;
        }
      }
    }
    if ( positions[0] < 0.0 || positions[1] < 0.0 )
      continue;
    const bool closed = path.first() == path.last();
    const double span = positions[1] - positions[0];
    double distance = std::abs( span );
    int direction = span < 0.0 ? -1 : 1;
    if ( closed && distance > offsets.last() * 0.5 )
    {
      distance = offsets.last() - distance;
      direction *= -1;
    }
    if ( distance <= epsilon || distance >= shortest )
      continue;
    shortest = distance;
    QVector<QPair<double, int>> between;
    for ( int i = 0; i < path.size() - ( closed ? 1 : 0 ); ++i )
    {
      double walked = direction * ( offsets[i] - positions[0] );
      if ( closed && walked < 0.0 )
        walked += offsets.last();
      if ( walked > epsilon && walked < distance - epsilon )
        between.append( { walked, i } );
    }
    std::sort( between.begin(), between.end() );
    trace.clear();
    for ( const auto &entry : std::as_const( between ) )
    {
      const int vertex = mRegionBoundary.vertexNrFromVertexId( QgsVertexId( part, 0, entry.second ) );
      trace.append( { path[entry.second], vertex, true } );
    }
    trace.append( to );
  }
  return trace;
}

bool RoofInteraction::splitRegions( const QgsPolylineXY &vertices )
{
  const QgsGeometry line = QgsGeometry::fromPolylineXY( vertices );
  if ( !line.isSimple() )
    return false;
  QgsPointSequence cut;
  cut.reserve( vertices.size() );
  for ( const QgsPointXY &vertex : vertices )
    cut.append( QgsPoint( vertex ) );

  // Keep closed roof patches above the footprint partition.
  QVariantList current = mRegions;
  QVariantList partitions;
  int baseIndex = -1;
  for ( int index = 0; index < current.size(); ++index )
  {
    if ( isPartition( current[index] ) )
      partitions.append( current[index] );
    else if ( regionGeometry( current[index].toMap() ).isGeosEqual( mFootprint ) )
      baseIndex = index;
  }
  const auto remaining = uncoveredRegion( mFootprint, partitions, 0 );
  if ( !remaining )
    return false;
  const double minimumArea = areaTolerance( mFootprint );
  if ( remaining->area() > minimumArea )
  {
    QVariantMap region = baseIndex >= 0 ? current[baseIndex].toMap() : regionValue( mFootprint, mBaseHeight );
    region.insert( QStringLiteral( "partition" ), true );
    if ( partitions.isEmpty() && baseIndex >= 0 )
      current[baseIndex] = region;
    else
    {
      if ( region.contains( QStringLiteral( "ridge" ) ) && !region.contains( QStringLiteral( "ridge_support" ) ) )
        region.insert( QStringLiteral( "ridge_support" ), region.value( QStringLiteral( "wkb" ) ) );
      QVector<QgsGeometry> gaps;
      for ( const QgsGeometry &part : remaining->asGeometryCollection() )
      {
        if ( part.area() > minimumArea )
          gaps.append( part );
      }
      if ( !gaps.isEmpty() )
      {
        region.insert( QStringLiteral( "wkb" ), geometryValue( QgsGeometry::collectGeometry( gaps ) ) );
        current.insert( baseIndex + 1, region );
      }
    }
  }

  QVariantList result;
  bool split = false;
  for ( const QVariant &value : std::as_const( current ) )
  {
    QVariantMap region = value.toMap();
    const QgsGeometry outline = regionGeometry( region );
    if ( !isPartition( value ) )
    {
      const QgsGeometry boundary = outline.isEmpty() ? QgsGeometry() : boundaryOf( outline );
      const double tolerance = geometryTolerance( mFootprint );
      if ( boundary.isEmpty() || boundary.distance( QgsGeometry::fromPointXY( vertices.first() ) ) > tolerance
           || boundary.distance( QgsGeometry::fromPointXY( vertices.last() ) ) > tolerance )
      {
        result.append( value );
        continue;
      }
    }
    // The existing slopes are clipped to each child of the cut.
    if ( region.contains( QStringLiteral( "ridge" ) ) && !region.contains( QStringLiteral( "ridge_support" ) ) )
      region.insert( QStringLiteral( "ridge_support" ), region.value( QStringLiteral( "wkb" ) ) );

    QVariantList children;
    bool regionSplit = false;
    for ( QgsGeometry part : outline.asGeometryCollection() )
    {
      QVector<QgsGeometry> pieces;
      if ( line.boundingBox().intersects( part.boundingBox() ) )
      {
        QgsPointSequence topologyPoints;
        const Qgis::GeometryOperationResult status = part.splitGeometry( cut, pieces, false, topologyPoints );
        if ( status != Qgis::GeometryOperationResult::Success && status != Qgis::GeometryOperationResult::NothingHappened )
          return false;
        regionSplit |= status == Qgis::GeometryOperationResult::Success;
      }
      pieces.prepend( part );
      for ( const QgsGeometry &piece : std::as_const( pieces ) )
      {
        QVariantMap child = region;
        child.insert( QStringLiteral( "wkb" ), geometryValue( piece ) );
        children.append( child );
      }
    }
    split |= regionSplit;
    if ( regionSplit )
      result += children;
    else
      result.append( value );
  }
  return split && setRegions( std::move( result ) );
}

void RoofInteraction::updateRegionBoundaries()
{
  mBoundaryOutline.reset();
  mBoundaryHighlight.reset();
  mRegionBoundary = {};
  mMapRegionBoundary = {};
  mBoundaryPaths.clear();
  const bool drawingRegion = mUi.drawRegionCheckBox->isChecked();
  if ( ( !drawingRegion && !mUi.drawRidgeCheckBox->isChecked() ) || !mCanvas || !mLayer || mFootprint.isEmpty() )
    return;

  QVector<QgsGeometry> lines;
  if ( drawingRegion )
    lines = boundaryOf( mFootprint ).asGeometryCollection();
  for ( const QVariant &value : std::as_const( mRegions ) )
  {
    const QgsGeometry region = regionGeometry( value.toMap() );
    if ( !region.isEmpty() && ( !drawingRegion || !region.isGeosEqual( mFootprint ) ) )
      lines += boundaryOf( region ).asGeometryCollection();
  }
  mRegionBoundary = QgsGeometry::collectGeometry( lines );
  mBoundaryPaths = mRegionBoundary.asMultiPolyline();
  mMapRegionBoundary = mRegionBoundary;
  const QgsCoordinateTransform transform = mCanvas->mapSettings().layerTransform( mLayer );
  try
  {
    if ( transform.isValid() && mMapRegionBoundary.transform( transform ) != Qgis::GeometryOperationResult::Success )
      mMapRegionBoundary = {};
  }
  catch ( const QgsCsException & )
  {
    mMapRegionBoundary = {};
  }
  if ( mMapRegionBoundary.isEmpty() )
    return;
  mBoundaryOutline = std::make_unique<QgsRubberBand>( mCanvas, Qgis::GeometryType::Line );
  mBoundaryOutline->setStrokeColor( QColor( 0, 139, 190, 190 ) );
  mBoundaryOutline->setWidth( drawingRegion ? 1 : 2 );
  mBoundaryOutline->setLineStyle( Qt::DashLine );
  mBoundaryOutline->setToGeometry( mMapRegionBoundary, nullptr );
}

void RoofInteraction::highlightBoundary( int vertex )
{
  if ( vertex < 0 )
  {
    if ( mBoundaryHighlight )
      mBoundaryHighlight->hide();
    return;
  }
  if ( !mBoundaryHighlight )
  {
    mBoundaryHighlight = std::make_unique<QgsRubberBand>( mCanvas, Qgis::GeometryType::Line );
    mBoundaryHighlight->setStrokeColor( QColor( 0, 139, 190 ) );
    mBoundaryHighlight->setWidth( 3 );
  }
  const QgsPointXY a( mMapRegionBoundary.vertexAt( vertex - 1 ) );
  const QgsPointXY b( mMapRegionBoundary.vertexAt( vertex ) );
  mBoundaryHighlight->setToGeometry( QgsGeometry::fromPolylineXY( { a, b } ), nullptr );
}

void RoofInteraction::setRidgeDrawing( bool enabled )
{
  if ( !enabled )
  {
    cancelRidge();
    updateRegionBoundaries();
    return;
  }
  if ( mRegions.isEmpty() )
  {
    mUi.drawRidgeCheckBox->setChecked( false );
    message( QObject::tr( "请先绘制区域，或在区域绘制模式下右键选择建筑轮廓。" ), Qgis::MessageLevel::Warning );
    return;
  }
  if ( mUi.drawRegionCheckBox->isChecked() )
    mUi.drawRegionCheckBox->setChecked( false );
  else
    updateRegionBoundaries();
  if ( mCanvas )
    mCanvas->setFocus();
  message( QObject::tr( "左键画屋脊，右键完成并根据点云微调。两端贴边画双坡，内部两点画四面坡，可画一个折点。" ) );
}

void RoofInteraction::cancelRidge()
{
  const bool wasPreviewing = !mRidgeDragPreview.isEmpty();
  mRidgeDragPreview.clear();
  if ( mRidgeRegion >= 0 )
  {
    mOutline.reset();
    mVertexHandles.reset();
  }
  mRidgeVertices.clear();
  mAdjustingRidge = false;
  mRidgeType = RoofType::None;
  mRidgeSamples.clear();
  mGableWidths = {};
  mRidgeComponent = {};
  mRidgeDragStart.reset();
  mDraggedRidgeVertex = -1;
  mRidgeRegion = -1;
  mRidgeBoundary = {};
  mMapRidgeBoundary = {};
  mRidgeOutline.reset();
  updateDeleteRidgeButton();
  if ( wasPreviewing )
    notifyPreviewChanged();
}

QgsPointXY RoofInteraction::ridgePoint( QgsMapMouseEvent *event ) const
{
  const QgsMapSettings &settings = mCanvas->mapSettings();
  const QgsPointXY mapPoint = event->originalMapPoint();
  QgsPointXY closest;
  int nextVertex = -1;
  const double tolerance = 8.0 * settings.mapUnitsPerPixel();
  if ( !event->modifiers().testFlag( Qt::ControlModifier ) && !mMapRidgeBoundary.isEmpty() )
  {
    const double distance = mMapRidgeBoundary.closestSegmentWithContext( mapPoint, closest, nextVertex );
    if ( distance >= 0.0 && distance <= tolerance * tolerance )
    {
      const QgsPointXY candidate = settings.mapToLayerCoordinates( mLayer, closest );
      // Project again in layer coordinates so a snapped point lies on the stored outline.
      mRidgeBoundary.closestSegmentWithContext( candidate, closest, nextVertex, nullptr, 0.0 );
      return closest;
    }
  }
  return settings.mapToLayerCoordinates( mLayer, mapPoint );
}

bool RoofInteraction::pressRidge( QgsMapMouseEvent *event )
{
  if ( event->button() != Qt::LeftButton )
    return false;

  const QgsMapSettings &settings = mCanvas->mapSettings();
  const QgsPointXY mapPoint = event->originalMapPoint();
  if ( mAdjustingRidge || mRidgeVertices.isEmpty() )
  {
    const bool refit = event->modifiers().testFlag( Qt::ShiftModifier );
    const bool manual = event->modifiers().testFlag( Qt::AltModifier ) || event->modifiers().testFlag( Qt::ControlModifier );
    const int hit = !refit && !manual ? ridgeAt( mapPoint ) : -1;
    if ( hit >= 0 )
    {
      pickRidge( hit, event );
      return true;
    }

    int index = regionAt( mapPoint );
    if ( index >= 0 && mAdjustingRidge && !refit && !manual )
    {
      mUi.drawRegionCheckBox->setChecked( true );
      selectRegion( index );
      editRegionVertex( event );
      return true;
    }
    if ( index < 0 && !event->modifiers().testFlag( Qt::ControlModifier ) )
    {
      const QgsGeometry point = QgsGeometry::fromPointXY( settings.mapToLayerCoordinates( mLayer, mapPoint ) );
      double nearest = 8.0 * settings.mapUnitsPerPixel();
      for ( int regionIndex = mRegions.size() - 1; regionIndex >= 0; --regionIndex )
      {
        const QgsGeometry region = regionGeometry( mRegions[regionIndex].toMap() );
        if ( region.isEmpty() )
          continue;
        QgsPointXY closest;
        int next = -1;
        if ( boundaryOf( region ).closestSegmentWithContext( point.asPoint(), closest, next ) < 0.0 )
          continue;
        const double distance = settings.layerToMapCoordinates( mLayer, closest ).distance( mapPoint );
        if ( distance < nearest )
        {
          index = regionIndex;
          nearest = distance;
        }
      }
    }
    if ( index == mRidgeRegion && mAdjustingRidge && !refit && !manual )
      return true;
    if ( !selectRidgeRegion( index ) )
    {
      message( QObject::tr( "请在绘制区域内或边界附近落下第一点。" ), Qgis::MessageLevel::Warning );
      return true;
    }

    const QVariantMap region = mRegions[mRidgeRegion].toMap();
    if ( !manual && restoreRidge() )
    {
      if ( refit && !region.contains( QStringLiteral( "ridge_support" ) ) )
      {
        QgsPolylineXY ridge = mRidgeVertices;
        double eave = mUi.eavesElevationSpinBox->value(), lift = mUi.roofLiftHeightSpinBox->value();
        QVector<BuildingRoof::RoofSample> samples;
        GableSideWidths widths{};
        if ( fitDrawnRidge( mRidgeComponent, mRidgeType, ridge, eave, lift, samples, widths ) )
          previewRidge( std::move( ridge ), mRidgeComponent, mRidgeType, eave, lift, std::move( samples ), widths );
        else
          message( QObject::tr( "当前点云无法拟合这条屋脊。" ), Qgis::MessageLevel::Warning );
      }
      return true;
    }
    if ( refit && !manual )
    {
      cancelRidge();
      message( QObject::tr( "请先用左键画屋脊，完成后会根据点云拟合。" ) );
      return true;
    }
    mUi.eavesElevationSpinBox->setValue( region.value( QStringLiteral( "height" ) ).toDouble() );
    mUi.roofLiftHeightSpinBox->setValue( region.value( QStringLiteral( "ridge_lift" ), 1.0 ).toDouble() );
  }

  if ( mRidgeVertices.size() >= 3 )
  {
    message( QObject::tr( "屋脊最多三个点。右键完成，退格可撤回上一点。" ) );
    return true;
  }

  const QgsPointXY point = ridgePoint( event );
  const QgsGeometry region = regionGeometry( mRegions[mRidgeRegion].toMap() );
  if ( region.distance( QgsGeometry::fromPointXY( point ) ) > geometryTolerance( region ) )
  {
    if ( mRidgeVertices.isEmpty() )
      cancelRidge();
    message( QObject::tr( "屋脊顶点必须位于所选区域内。" ), Qgis::MessageLevel::Warning );
    return true;
  }
  if ( mRidgeVertices.isEmpty() || point.distance( mRidgeVertices.last() ) > geometryTolerance( region ) )
    mRidgeVertices.append( point );
  updateRidgeOutline();
  return true;
}

void RoofInteraction::pickRidge( int index, QgsMapMouseEvent *event )
{
  mUi.drawRidgeCheckBox->setChecked( true );
  if ( ( index == mRidgeRegion && mAdjustingRidge ) || ( selectRidgeRegion( index ) && restoreRidge() ) )
  {
    startRidgeDrag( event );
    return;
  }
  // A stored ridge may no longer fit a modified outline. Select the region so it can still be removed.
  mUi.drawRegionCheckBox->setChecked( true );
  selectRegion( index );
}

bool RoofInteraction::selectRidgeRegion( int index )
{
  cancelRidge();
  if ( index < 0 || index >= mRegions.size() )
    return false;
  const QgsGeometry region = regionGeometry( mRegions[index].toMap() );
  if ( region.isEmpty() )
    return false;
  QgsGeometry boundary = boundaryOf( region );
  QgsGeometry mapBoundary = boundary;
  const QgsCoordinateTransform transform = mCanvas->mapSettings().layerTransform( mLayer );
  try
  {
    if ( transform.isValid() && mapBoundary.transform( transform ) != Qgis::GeometryOperationResult::Success )
      return false;
  }
  catch ( const QgsCsException & )
  {
    return false;
  }
  mRidgeRegion = index;
  mRidgeBoundary = std::move( boundary );
  mMapRidgeBoundary = std::move( mapBoundary );
  return true;
}

QgsGeometry RoofInteraction::visibleRidge( int index ) const
{
  const QVariantMap region = mRegions[index].toMap();
  const QgsGeometry line = storedGeometry( region, QStringLiteral( "ridge" ) );
  if ( line.isEmpty() )
    return {};
  const auto visible = uncoveredRegion( regionGeometry( region ), mRegions, index + 1 );
  return visible && !visible->isEmpty() ? line.intersection( *visible ) : QgsGeometry();
}

int RoofInteraction::ridgeAt( const QgsPointXY &mapPoint ) const
{
  const auto &settings = mCanvas->mapSettings();
  const QgsCoordinateTransform transform = settings.layerTransform( mLayer );
  const double tolerance = 8.0 * settings.mapUnitsPerPixel();
  double nearest = tolerance * tolerance;
  int found = -1;
  for ( int index = mRegions.size() - 1; index >= 0; --index )
  {
    QgsGeometry ridge = visibleRidge( index );
    if ( ridge.isEmpty() )
      continue;
    try
    {
      if ( transform.isValid() && ridge.transform( transform ) != Qgis::GeometryOperationResult::Success )
        continue;
    }
    catch ( const QgsCsException & )
    {
      continue;
    }
    QgsPointXY closest;
    int next = -1;
    const double distance = ridge.closestSegmentWithContext( mapPoint, closest, next );
    if ( distance >= 0.0 && distance < nearest )
    {
      nearest = distance;
      found = index;
    }
  }
  return found;
}

void RoofInteraction::updateStoredRidges()
{
  mStoredRidgeOutline.reset();
  if ( !mCanvas || !mLayer )
    return;
  for ( int index = 0; index < mRegions.size(); ++index )
  {
    const QgsGeometry ridge = visibleRidge( index );
    if ( ridge.type() != Qgis::GeometryType::Line || ridge.isEmpty() )
      continue;
    if ( !mStoredRidgeOutline )
    {
      mStoredRidgeOutline = std::make_unique<QgsRubberBand>( mCanvas, Qgis::GeometryType::Line );
      mStoredRidgeOutline->setWidth( 2 );
      mStoredRidgeOutline->setStrokeColor( QColor( 155, 94, 62 ) );
      mStoredRidgeOutline->setZValue( mStoredRidgeOutline->zValue() - 1 );
    }
    mStoredRidgeOutline->addGeometry( ridge, mLayer );
  }
}

bool RoofInteraction::restoreRidge()
{
  const QVariantMap region = mRegions[mRidgeRegion].toMap();
  const QgsPolylineXY ridge = storedGeometry( region, QStringLiteral( "ridge" ) ).asPolyline();
  if ( ridge.size() < 2 )
    return false;
  const QgsGeometry support = storedGeometry( region, QStringLiteral( "ridge_support" ) );
  const QgsGeometry component = ridgeComponent( support.isEmpty() ? regionGeometry( region ) : support, ridge );
  if ( component.isEmpty() )
    return false;
  RoofType type = storedRidgeType( region );
  if ( type == RoofType::None )
    type = ridgeTypeForGeometry( component, ridge );
  RoofPatch patch{ component, {}, 0.0, {}, {}, {} };
  const double lift = region.value( QStringLiteral( "ridge_lift" ) ).toDouble();
  const auto widths = storedGableWidths( region );
  if ( lift <= 0.0 || !makeRidgePreview( patch, component, ridge, type, widths ) )
    return false;
  mRidgeVertices = ridge;
  mRidgeComponent = component;
  mRidgeType = type;
  mGableWidths = widths;
  mAdjustingRidge = true;
  setRidgeHeights( region.value( QStringLiteral( "height" ) ).toDouble(), lift );
  updateRidgeOutline();
  updateDeleteRidgeButton();
  if ( !support.isEmpty() )
    message( QObject::tr( "这条屋脊来自分区前的坡面。可用滚轮调抬升、Delete 删除，或按 Alt＋左键为当前分区重画。" ) );
  return true;
}

QVector<BuildingRoof::RoofSample> RoofInteraction::collectRidgeSamples( const QgsGeometry &component ) const
{
  const QString cloudId = mUi.pointCloudComboBox->currentData().toString();
  auto *cloud = qobject_cast<QgsPointCloudLayer *>( QgsProject::instance()->mapLayer( cloudId ) );
  if ( !cloud )
    return {};
  auto visible = uncoveredRegion( component, mRegions, mRidgeRegion + 1 );
  if ( !visible || visible->isEmpty() )
    return {};

  try
  {
    const QgsCoordinateTransform toCloud( mLayer->crs(), cloud->crs(), QgsProject::instance() );
    if ( toCloud.isValid() && visible->transform( toCloud ) != Qgis::GeometryOperationResult::Success )
      return {};
    bool complete = false;
    auto samples = RoofModelPipeline::collectSamples( *visible, cloudId, &complete );
    if ( !complete )
      return {};
    if ( toCloud.isValid() && !toCloud.isShortCircuited() )
    {
      for ( auto &sample : samples )
      {
        const QgsPointXY xy = toCloud.transform( QgsPointXY( sample.point ), Qgis::TransformDirection::Reverse );
        sample.point = QgsPoint( xy.x(), xy.y(), sample.point.z() );
      }
    }
    return samples;
  }
  catch ( const QgsCsException & )
  {
    return {};
  }
}

bool RoofInteraction::fitDrawnRidge( const QgsGeometry &component, RoofType type, QgsPolylineXY &ridge,
                                     double &eave, double &lift, QVector<BuildingRoof::RoofSample> &samples,
                                     GableSideWidths &widths ) const
{
  samples = collectRidgeSamples( component );
  if ( samples.size() < 12 )
    return false;

  BuildingRoof::RidgePrediction prediction;
  try
  {
    const auto markers = RoofModelSettings::builderMarkers( type );
    if ( type == RoofType::Hipped )
      BuildingRoof::buildHippedRoofPrismMesh( component, 1.0, markers, samples, &prediction );
    else
      BuildingRoof::buildGabledRoofPrismMesh( component, 1.0, markers, samples, &prediction );
  }
  catch ( const QgsNotSupportedException & )
  {
    prediction = {};
  }

  const QgsRectangle bounds = component.boundingBox();
  // Keep each control point near the user's sketch, even if the automatic fit finds another ridge.
  const double movementLimit = 0.15 * std::min( bounds.width(), bounds.height() );
  QgsPolylineXY proposal = std::move( prediction.ridge );
  auto automaticWidths = prediction.gableWidths;
  if ( type == RoofType::Gabled && proposal.size() == 2 )
    proposal = clipRidgeAxis( component, proposal );
  if ( proposal.size() == ridge.size() )
  {
    auto reversed = proposal;
    std::reverse( reversed.begin(), reversed.end() );
    if ( ridgeDisplacement( reversed, ridge ) < ridgeDisplacement( proposal, ridge ) )
    {
      proposal = std::move( reversed );
      std::swap( automaticWidths[0], automaticWidths[1] );
    }
  }
  const bool nearby = proposal.size() == ridge.size() && ridgeDisplacement( proposal, ridge ) <= movementLimit;
  if ( nearby && prediction.samples.size() >= 12 )
    samples = std::move( prediction.samples );

  double seedEave = eave, seedRise = lift;
  if ( nearby && std::isfinite( prediction.eaveHeight ) && std::isfinite( prediction.ridgeHeight )
       && prediction.ridgeHeight > prediction.eaveHeight )
  {
    seedEave = prediction.eaveHeight;
    seedRise = prediction.ridgeHeight - prediction.eaveHeight;
  }
  if ( type == RoofType::Gabled )
  {
    if ( nearby && validGableWidths( automaticWidths, geometryTolerance( component ) ) )
    {
      RoofPatch patch{ component, {}, 0.0, {}, {}, {} };
      double fittedEave = seedEave, fittedLift = seedRise;
      // Use the runs of the automatic least-squares planes and one shared ridge/eave datum.
      // Reusing the automatic direction also keeps the plane intersection continuous after clipping.
      if ( makeRidgePreview( patch, component, proposal, type, automaticWidths )
           && fitGableHeights( component, proposal, samples, fittedEave, fittedLift, nullptr, automaticWidths ) )
      {
        ridge = std::move( proposal );
        eave = fittedEave;
        lift = fittedLift;
        widths = automaticWidths;
        return true;
      }
    }
    QgsPolylineXY fitted = ridge;
    RoofPatch patch{ component, {}, 0.0, {}, {}, {} };
    if ( !refineGable( component, samples, fitted, seedEave, seedRise, movementLimit )
         || !makeRidgePreview( patch, component, fitted, type ) )
      return false;
    ridge = std::move( fitted );
    eave = seedEave;
    lift = seedRise;
    return true;
  }

  double bestScore = std::numeric_limits<double>::max();
  QgsPolylineXY bestRidge;
  const auto evaluate = [&]( const QgsPolylineXY &candidate ) {
    RoofPatch patch{ component, {}, 0.0, {}, {}, {} };
    double fittedEave = seedEave, fittedRise = seedRise, score = 0.0;
    if ( !makeRidgePreview( patch, component, candidate, type ) || !fitFacetHeights( patch, samples, fittedEave, fittedRise, &score )
         || score >= bestScore )
      return;
    bestScore = score;
    bestRidge = candidate;
    eave = fittedEave;
    lift = fittedRise;
  };
  evaluate( ridge );
  if ( nearby )
    evaluate( proposal );
  if ( bestRidge.isEmpty() )
    return false;
  ridge = std::move( bestRidge );
  return true;
}

void RoofInteraction::previewRidge( QgsPolylineXY ridge, const QgsGeometry &component, RoofType type,
                                    double eave, double lift, QVector<BuildingRoof::RoofSample> samples,
                                    GableSideWidths widths )
{
  mRidgeVertices = std::move( ridge );
  mRidgeComponent = component;
  mRidgeType = type;
  mRidgeSamples = std::move( samples );
  mGableWidths = widths;
  mAdjustingRidge = true;
  setRidgeHeights( eave, lift );
  applyRidge();
  message( QObject::tr( "拖动屋脊、端点或折点调整位置，滚轮调抬升。Delete 删除屋脊，左键可切换。" ) );
}

void RoofInteraction::setRidgeHeights( double eave, double lift )
{
  const QSignalBlocker eaveBlocker( mUi.eavesElevationSpinBox );
  const QSignalBlocker liftBlocker( mUi.roofLiftHeightSpinBox );
  mUi.eavesElevationSpinBox->setValue( eave );
  mUi.roofLiftHeightSpinBox->setValue( std::max( 0.01, lift ) );
}

QVariantMap RoofInteraction::ridgeRegionValue( const QgsPolylineXY &ridge, const GableSideWidths &widths ) const
{
  QVariantMap region = withRidge( mRegions[mRidgeRegion].toMap(), ridge, mUi.roofLiftHeightSpinBox->value() );
  region.insert( QStringLiteral( "height" ), mUi.eavesElevationSpinBox->value() );
  region.insert( QStringLiteral( "ridge_type" ), RoofModelSettings::typeId( mRidgeType ) );
  if ( mRidgeType == RoofType::Gabled )
  {
    region.insert( QStringLiteral( "fitted_gable" ), true );
    if ( validGableWidths( widths, geometryTolerance( mRidgeComponent ) ) )
      region.insert( QStringLiteral( "gable_widths" ), QVariantList{ widths[0], widths[1] } );
  }
  return region;
}

void RoofInteraction::applyRidge()
{
  const bool updated = setRegion( mRidgeRegion, ridgeRegionValue( mRidgeVertices, mGableWidths ) );
  updateStoredRidges();
  updateRidgeOutline();
  updateDeleteRidgeButton();
  if ( updated )
    mChanged();
}

void RoofInteraction::startRidgeDrag( QgsMapMouseEvent *event )
{
  if ( mRegions[mRidgeRegion].toMap().contains( QStringLiteral( "ridge_support" ) ) )
    return;
  const auto &settings = mCanvas->mapSettings();
  const QgsPointXY point = event->originalMapPoint();
  const double tolerance = 8.0 * settings.mapUnitsPerPixel();
  mDraggedRidgeVertex = -1;
  QgsPolylineXY vertices;
  double closest = tolerance;
  bool overSegment = false;
  for ( int index = 0; index < mRidgeVertices.size(); ++index )
  {
    const QgsPointXY vertex = settings.layerToMapCoordinates( mLayer, mRidgeVertices[index] );
    const double distance = vertex.distance( point );
    if ( distance <= closest )
    {
      closest = distance;
      mDraggedRidgeVertex = index;
    }
    if ( !vertices.isEmpty() && vertices.last() != vertex )
      overSegment |= interpolate( vertices.last(), vertex, segmentParameter( vertices.last(), vertex, point ) ).distance( point ) <= tolerance;
    vertices.append( vertex );
  }
  if ( mDraggedRidgeVertex >= 0 || overSegment )
    mRidgeDragStart = settings.mapToLayerCoordinates( mLayer, point );
}

QgsPolylineXY RoofInteraction::draggedRidge( QgsMapMouseEvent *event ) const
{
  const QgsPointXY target = mCanvas->mapSettings().mapToLayerCoordinates( mLayer, event->originalMapPoint() );
  const double dx = target.x() - mRidgeDragStart->x(), dy = target.y() - mRidgeDragStart->y();
  QgsPolylineXY ridge = mRidgeVertices;
  for ( int index = 0; index < ridge.size(); ++index )
  {
    if ( mDraggedRidgeVertex < 0 || mDraggedRidgeVertex == index )
      ridge[index] = QgsPointXY( ridge[index].x() + dx, ridge[index].y() + dy );
  }
  if ( mRidgeType == RoofType::Gabled )
  {
    ridge = clipRidgeAxis( mRidgeComponent, ridge );
    if ( !ridge.isEmpty() && !validGableWidths( mGableWidths ) )
      ridge = alignRidgeToEaves( mRidgeComponent, ridge );
    return ridge;
  }
  if ( mRidgeType == RoofType::BrokenGabled )
  {
    const QgsGeometry boundary = boundaryOf( mRidgeComponent );
    for ( int index : { 0, static_cast<int>( ridge.size() ) - 1 } )
    {
      if ( mDraggedRidgeVertex >= 0 && mDraggedRidgeVertex != index )
        continue;
      QgsPointXY closest;
      int next = -1;
      if ( boundary.closestSegmentWithContext( ridge[index], closest, next, nullptr, 0.0 ) >= 0.0 )
        ridge[index] = closest;
    }
  }
  return ridge;
}

bool RoofInteraction::release( QgsMapMouseEvent *event )
{
  if ( mVertexPress && event->button() == Qt::LeftButton )
  {
    const bool dragged = ( event->originalPixelPoint() - *mVertexPress ).manhattanLength() >= QApplication::startDragDistance();
    mVertexPress.reset();
    const bool wasPreviewing = mVertexPreview.has_value();
    mVertexPreview.reset();
    if ( dragged && mMovingVertex )
    {
      const QgsPointXY to = mCanvas->mapSettings().mapToLayerCoordinates( mLayer, event->snapPoint() );
      if ( modifyRegionVertex( VertexEdit::Move, *mMovingVertex, to ) )
      {
        mMovingVertex.reset();
        mChanged();
      }
      else
        message( QObject::tr( "无法移动到此处。请检查轮廓、相邻区域和屋脊的位置。" ), Qgis::MessageLevel::Warning );
    }
    updateOutline();
    if ( wasPreviewing )
      notifyPreviewChanged();
    return true;
  }
  if ( !mRidgeDragStart || event->button() != Qt::LeftButton )
    return false;
  const bool moved = mCanvas->mapSettings().mapToLayerCoordinates( mLayer, event->originalMapPoint() ).distance( *mRidgeDragStart )
                     > geometryTolerance( mRidgeComponent );
  QgsPolylineXY ridge = moved ? draggedRidge( event ) : QgsPolylineXY();
  const bool wasPreviewing = !mRidgeDragPreview.isEmpty();
  mRidgeDragPreview.clear();
  mRidgeDragStart.reset();
  if ( moved )
  {
    double eave = mUi.eavesElevationSpinBox->value(), lift = mUi.roofLiftHeightSpinBox->value();
    const auto widths = mRidgeType == RoofType::Gabled && !ridge.isEmpty()
                        ? movedGableWidths( mRidgeVertices, ridge, mGableWidths ) : GableSideWidths{};
    RoofPatch patch{ mRidgeComponent, {}, 0.0, {}, {}, {} };
    if ( makeRidgePreview( patch, mRidgeComponent, ridge, mRidgeType, widths ) )
    {
      if ( mRidgeType == RoofType::Gabled )
        fitGableHeights( mRidgeComponent, ridge, mRidgeSamples, eave, lift, nullptr, widths );
      else
        fitFacetHeights( patch, mRidgeSamples, eave, lift );
      mRidgeVertices = ridge;
      mGableWidths = widths;
      setRidgeHeights( eave, lift );
      applyRidge();
    }
    else
      message( QObject::tr( "这个位置无法成坡，已保留原屋脊。四面坡端点应留在区域内，折线双坡的折点应留在区域内。" ), Qgis::MessageLevel::Warning );
  }
  updateRidgeOutline();
  if ( wasPreviewing )
    notifyPreviewChanged();
  return true;
}

bool RoofInteraction::startRidgeAdjustment()
{
  if ( mRidgeVertices.size() < 2 )
  {
    message( QObject::tr( "请至少绘制两个屋脊点。" ), Qgis::MessageLevel::Warning );
    return false;
  }
  const QgsGeometry region = regionGeometry( mRegions[mRidgeRegion].toMap() );
  const QgsGeometry component = ridgeComponent( region, mRidgeVertices );
  const RoofType type = component.isEmpty() ? RoofType::None : ridgeTypeForGeometry( component, mRidgeVertices );
  RoofPatch candidate{ component, {}, 0.0, {}, {}, {} };
  if ( component.isEmpty() || !makeRidgePreview( candidate, component, mRidgeVertices, type ) )
  {
    message( QObject::tr( "直双坡画两个贴边点，折线双坡画两个贴边端点和一个内部折点，四面坡画两个内部端点。" ), Qgis::MessageLevel::Warning );
    return false;
  }

  QgsPolylineXY ridge = mRidgeVertices;
  double eave = mUi.eavesElevationSpinBox->value(), lift = mUi.roofLiftHeightSpinBox->value();
  QVector<BuildingRoof::RoofSample> samples;
  GableSideWidths widths{};
  const bool fitted = fitDrawnRidge( component, type, ridge, eave, lift, samples, widths );
  previewRidge( std::move( ridge ), component, type, eave, lift, std::move( samples ), widths );
  if ( !fitted && !mUi.pointCloudComboBox->currentData().toString().isEmpty() )
    message( QObject::tr( "当前点云未能可靠拟合，已按手绘屋脊建模，可拖动或用滚轮微调。" ) );
  return true;
}

void RoofInteraction::updateRidgeOutline( const QgsPointXY *cursor, const QgsPolylineXY &preview )
{
  if ( !mCanvas || !mLayer || mRidgeRegion < 0 )
    return;
  if ( !mOutline )
  {
    mOutline = std::make_unique<QgsRubberBand>( mCanvas, Qgis::GeometryType::Polygon );
    mOutline->setStrokeColor( QColor( 0, 139, 190 ) );
    mOutline->setFillColor( QColor( 0, 139, 190, 35 ) );
    mOutline->setWidth( 2 );
    const QgsGeometry region = regionGeometry( mRegions[mRidgeRegion].toMap() );
    const auto visible = uncoveredRegion( region, mRegions, mRidgeRegion + 1 );
    mOutline->setToGeometry( visible.value_or( region ), mLayer );
  }
  if ( !mRidgeOutline )
  {
    mRidgeOutline = std::make_unique<QgsRubberBand>( mCanvas, Qgis::GeometryType::Line );
    mRidgeOutline->setStrokeColor( QColor( 215, 96, 20 ) );
    mRidgeOutline->setWidth( 3 );
    mRidgeOutline->setIcon( QgsRubberBand::ICON_CIRCLE );
  }
  mRidgeOutline->reset( Qgis::GeometryType::Line );
  if ( mAdjustingRidge && mRegions[mRidgeRegion].toMap().contains( QStringLiteral( "ridge_support" ) ) )
  {
    const QgsGeometry ridge = visibleRidge( mRidgeRegion );
    if ( ridge.type() == Qgis::GeometryType::Line )
      mRidgeOutline->setToGeometry( ridge, mLayer );
    return;
  }
  const QgsPolylineXY &vertices = preview.isEmpty() ? mRidgeVertices : preview;
  for ( int i = 0; i < vertices.size(); ++i )
    mRidgeOutline->addPoint( mCanvas->mapSettings().layerToMapCoordinates( mLayer, vertices[i] ), !cursor && i == vertices.size() - 1 );
  if ( cursor )
    mRidgeOutline->addPoint( mCanvas->mapSettings().layerToMapCoordinates( mLayer, *cursor ) );

  if ( mAdjustingRidge )
  {
    if ( !mVertexHandles )
    {
      mVertexHandles = std::make_unique<QgsRubberBand>( mCanvas, Qgis::GeometryType::Point );
      mVertexHandles->setIcon( QgsRubberBand::ICON_CIRCLE );
      mVertexHandles->setIconSize( 8 );
      mVertexHandles->setStrokeColor( QColor( 215, 96, 20 ) );
      mVertexHandles->setFillColor( Qt::white );
    }
    mVertexHandles->setToGeometry( QgsGeometry::fromMultiPointXY( vertices ), mLayer );
  }
}

void RoofInteraction::move( QgsMapMouseEvent *event )
{
  if ( ( mVertexPress || mRidgeDragStart ) && !event->buttons().testFlag( Qt::LeftButton ) )
    cancelDrag();
  if ( mVertexPress && mMovingVertex && event->buttons().testFlag( Qt::LeftButton )
       && ( mVertexPreview || ( event->originalPixelPoint() - *mVertexPress ).manhattanLength() >= QApplication::startDragDistance() ) )
  {
    mVertexPreview = mCanvas->mapSettings().mapToLayerCoordinates( mLayer, event->snapPoint() );
    updateOutline();
    notifyPreviewChanged();
    return;
  }
  if ( mRidgeDragStart )
  {
    mRidgeDragPreview = draggedRidge( event );
    updateRidgeOutline( nullptr, mRidgeDragPreview );
    notifyPreviewChanged();
    return;
  }
  if ( !mRidgeVertices.isEmpty() && !mAdjustingRidge && mCanvas && mLayer )
  {
    const QgsPointXY cursor = ridgePoint( event );
    updateRidgeOutline( &cursor );
  }
  if ( mUi.drawRegionCheckBox->isChecked() && mAdjustingRegion.isEmpty() && mCanvas && mLayer )
  {
    const RegionPoint cursor = regionPoint( event );
    highlightBoundary( cursor.boundaryVertex );
    if ( !mVertices.isEmpty() )
      updateOutline( &cursor.point, event->modifiers().testFlag( Qt::AltModifier ) ? traceBoundary( mVertices.last(), cursor ) : QVector<RegionPoint>() );
  }
}

bool RoofInteraction::wheel( QWheelEvent *event )
{
  if ( event->angleDelta().y() == 0 )
    return false;
  QDoubleSpinBox *elevation = nullptr;
  if ( !mAdjustingRegion.isEmpty() )
    elevation = mUi.eavesElevationSpinBox;
  else if ( mAdjustingRidge )
    elevation = mUi.roofLiftHeightSpinBox;
  if ( !elevation )
    return false;
  elevation->setValue( elevation->value() + elevation->singleStep() * event->angleDelta().y() / 120.0 );
  return true;
}

void RoofInteraction::removeLastSketchPoint()
{
  if ( !mVertices.isEmpty() )
  {
    mVertices.removeLast();
    while ( !mVertices.isEmpty() && mVertices.last().intermediate )
      mVertices.removeLast();
    highlightBoundary( mVertices.isEmpty() ? -1 : mVertices.last().boundaryVertex );
    updateOutline();
  }
  else if ( !mAdjustingRidge && !mRidgeVertices.isEmpty() )
  {
    mRidgeVertices.removeLast();
    if ( mRidgeVertices.isEmpty() )
      cancelRidge();
    else
      updateRidgeOutline();
  }
}

bool RoofInteraction::keyPress( QKeyEvent *event )
{
  if ( !hasBuilding() )
    return false;
  if ( event->matches( QKeySequence::Undo ) )
  {
    undo();
    return true;
  }

  const int key = event->key();
  // QGIS otherwise uses these keys to delete selected vector features.
  const bool deleteKey = key == Qt::Key_Delete || key == Qt::Key_Backspace;
  if ( deleteKey && ( mVertexPress || mRidgeDragStart || ( key == Qt::Key_Delete && event->isAutoRepeat() ) ) )
    return true;
  if ( key == Qt::Key_Backspace )
  {
    removeLastSketchPoint();
    return true;
  }
  const bool finish = key == Qt::Key_Return || key == Qt::Key_Enter;
  if ( mUi.drawRidgeCheckBox->isChecked() )
  {
    if ( key == Qt::Key_Delete && mAdjustingRidge )
    {
      deleteSelectedRidge();
      return true;
    }
    if ( key == Qt::Key_Escape )
    {
      if ( mRidgeVertices.isEmpty() )
        mUi.drawRidgeCheckBox->setChecked( false );
      else
        cancelRidge();
      return true;
    }
    return deleteKey;
  }
  if ( !mUi.drawRegionCheckBox->isChecked() )
    return deleteKey;
  if ( finish && !mVertices.isEmpty() )
  {
    finishRegionSketch( !event->modifiers().testFlag( Qt::ControlModifier ) );
    return true;
  }
  if ( key == Qt::Key_Delete && mAdjustingRegionIndex >= 0 )
  {
    QVariantList regions = mRegions;
    regions.removeAt( mAdjustingRegionIndex );
    setRegions( std::move( regions ) );
    cancelDraft();
    updateRegionBoundaries();
    updateStoredRidges();
    mChanged();
    message( QObject::tr( "已删除所选区域。Ctrl+Z 可撤销。" ) );
    return true;
  }
  if ( key == Qt::Key_Escape )
  {
    if ( event->isAutoRepeat() )
      return true;
    if ( hasSelectedRegion() )
      deselectRegion();
    else if ( !mVertices.isEmpty() )
      cancelDraft();
    else
      mUi.drawRegionCheckBox->setChecked( false );
    return true;
  }
  return deleteKey;
}

void RoofInteraction::updateOutline( const QgsPointXY *cursor, const QVector<RegionPoint> &trace )
{
  mVertexHandles.reset();
  if ( !mUi.drawRegionCheckBox->isChecked() || !mCanvas || !mLayer )
  {
    mOutline.reset();
    return;
  }
  if ( !mOutline )
  {
    mOutline = std::make_unique<QgsRubberBand>( mCanvas, Qgis::GeometryType::Line );
    mOutline->setWidth( 2 );
    mOutline->setStrokeColor( QColor( 0, 139, 190 ) );
    mOutline->setFillColor( QColor( 0, 139, 190, 35 ) );
    mOutline->setIcon( QgsRubberBand::ICON_CIRCLE );
  }
  if ( !mVertices.isEmpty() )
  {
    mOutline->reset( Qgis::GeometryType::Line );
    for ( const RegionPoint &vertex : std::as_const( mVertices ) )
      mOutline->addPoint( mCanvas->mapSettings().layerToMapCoordinates( mLayer, vertex.point ), false );
    for ( const RegionPoint &vertex : trace )
      mOutline->addPoint( mCanvas->mapSettings().layerToMapCoordinates( mLayer, vertex.point ), false );
    mOutline->addPoint( mCanvas->mapSettings().layerToMapCoordinates( mLayer, cursor ? *cursor : mVertices.last().point ) );
    return;
  }
  const QgsGeometry &outline = mAdjustingRegion;
  if ( outline.isEmpty() )
    mOutline->reset( Qgis::GeometryType::Polygon );
  else
    mOutline->setToGeometry( outline, mLayer );
  if ( canMoveRegionVertices() )
  {
    const QgsGeometry shape = activeRegionShape();
    QgsMultiPointXY points;
    auto vertices = shape.vertices();
    while ( vertices.hasNext() )
      points.append( QgsPointXY( vertices.next() ) );
    mVertexHandles = std::make_unique<QgsRubberBand>( mCanvas, Qgis::GeometryType::Point );
    mVertexHandles->setIcon( QgsRubberBand::ICON_CIRCLE );
    mVertexHandles->setIconSize( mMovingVertex ? 10 : 7 );
    mVertexHandles->setStrokeColor( QColor( 0, 105, 150 ) );
    mVertexHandles->setFillColor( QColor( 255, 255, 255 ) );
    if ( mMovingVertex )
      points = { mVertexPreview.value_or( *mMovingVertex ) };
    mVertexHandles->setToGeometry( QgsGeometry::fromMultiPointXY( points ), mLayer );
  }
}

void RoofInteraction::message( const QString &text, Qgis::MessageLevel level ) const
{
  if ( mIface )
    mIface->messageBar()->pushMessage( QObject::tr( "屋顶编辑" ), text, level, 5 );
}

BuildingRoof::MeshResult RoofInteraction::build( const QgsGeometry &geometry, const RoofModelSettings::Selection &selection )
{
  BuildingRoof::MeshResult result;
  result.error = QObject::tr( "无法根据绘制区域生成屋顶网格，请检查建筑轮廓和区域范围。" );
  const QgsGeometry footprint = polygonalPart( geometry );
  if ( footprint.isEmpty() || !footprint.isGeosValid() || !std::isfinite( selection.regionBaseHeight ) )
    return result;
  QgsGeometry remaining = footprint;
  QVector<RoofPatch> patches;
  for ( auto it = selection.regions.crbegin(); it != selection.regions.crend() && !remaining.isEmpty(); ++it )
  {
    const QVariantMap region = it->toMap();
    const QgsGeometry outline = regionGeometry( region );
    const double height = region.value( QStringLiteral( "height" ) ).toDouble();
    if ( outline.isEmpty() || !outline.isGeosValid() || !std::isfinite( height ) )
      return result;
    const QgsGeometry intersection = remaining.equals( outline ) ? remaining : remaining.intersection( outline );
    if ( intersection.isNull() )
      return result;
    const QgsGeometry patch = polygonalPart( intersection );
    if ( patch.isEmpty() )
      continue;
    RoofPatch roofPatch{ patch, boundaryOf( patch ), height, {}, {}, {} };
    const QgsGeometry ridge = storedGeometry( region, QStringLiteral( "ridge" ) );
    if ( !ridge.isEmpty() )
    {
      try
      {
        const QgsGeometry support = storedGeometry( region, QStringLiteral( "ridge_support" ) );
        if ( !addRidgeFacets( roofPatch, support.isEmpty() ? outline : support, ridge.asPolyline(), region.value( QStringLiteral( "ridge_lift" ) ).toDouble(),
                              storedRidgeType( region ), storedGableWidths( region ) ) )
        {
          result.error = QObject::tr( "无法根据保存的屋脊线生成坡面，请检查屋脊线位置。" );
          return result;
        }
      }
      catch ( const QgsNotSupportedException & )
      {
        result.error = QObject::tr( "当前 GEOS 版本不支持屋脊建模所需的约束三角剖分。" );
        return result;
      }
    }
    patches.append( roofPatch );
    const QgsGeometry difference = remaining.difference( patch );
    if ( difference.isNull() )
      return result;
    remaining = polygonalPart( difference );
  }
  if ( patches.isEmpty() )
    return result;
  if ( !remaining.isEmpty() )
    patches.append( { remaining, boundaryOf( remaining ), selection.regionBaseHeight, {}, {}, {} } );

  // Fill the footprint as a set of nonoverlapping plates. The latest region owns overlaps.
  // Extrude each exterior wall from the roof mesh's own edge to keep the join exact.
  const QgsGeometry footprintEdgeBand = boundaryOf( footprint ).buffer( geometryTolerance( footprint ) * 2.0, 4 );
  if ( footprintEdgeBand.isEmpty() )
    return result;
  auto footprintEdge = QgsGeometry::createGeometryEngine( footprintEdgeBand.constGet() );
  footprintEdge->prepareGeometry();
  double floor = 0.0;
  for ( const RoofPatch &patch : std::as_const( patches ) )
  {
    floor = std::min( floor, patch.height );
    for ( const RoofFacet &facet : patch.facets )
    {
      auto vertices = facet.geometry.vertices();
      while ( vertices.hasNext() )
        floor = std::min( floor, facet.heightAt( QgsPointXY( vertices.next() ) ) );
    }
  }
  if ( floor < 0.0 )
    floor -= 0.1;
  try
  {
    if ( !appendSurface( result.mesh, footprint, floor, false ) )
      return result;
    for ( int i = 0; i < patches.size(); ++i )
    {
      const RoofPatch &patch = patches[i];
      const int firstRoofIndex = result.mesh.indices.size();
      if ( !appendPatch( result.mesh, patch ) )
        return result;
      appendExteriorWalls( result.mesh, firstRoofIndex, *footprintEdge, floor );
      for ( int j = 0; j < i; ++j )
      {
        const RoofPatch &neighbor = patches[j];
        if ( patch.geometry.boundingBox().intersects( neighbor.geometry.boundingBox() ) )
          appendSharedWalls( result.mesh, patch, neighbor );
      }
      result.mesh.structureLines += patch.slopeLines;
      if ( patch.slopeLines.isEmpty() )
        appendPlaneCreases( result.mesh, patch );
    }
    appendFootprintLines( result.mesh, footprint, patches, floor );
  }
  catch ( const QgsNotSupportedException & )
  {
    result.error = QObject::tr( "当前 GEOS 版本不支持屋顶区域所需的约束三角剖分。" );
    return result;
  }
  result.success = true;
  result.error.clear();
  return result;
}
