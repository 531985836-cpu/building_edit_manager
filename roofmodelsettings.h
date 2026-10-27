#pragma once

#include "buildingroof.h"

#include <qgsfeatureiterator.h>
#include <qgspointcloudlayer.h>
#include <qgsproject.h>
#include <qgsvectorlayer.h>
#include <QVariantMap>

namespace RoofModelSettings
{
  enum class Type { None, Gabled, Hipped, BrokenGabled, Apex, Barrel, Dome, Frustum, Relief };

  struct Selection
  {
    Type type = Type::None;
    QString pointCloudLayerId;
  };

  inline QString typeId( Type type )
  {
    switch ( type )
    {
      case Type::Gabled: return QStringLiteral( "gabled" );
      case Type::Hipped: return QStringLiteral( "hipped" );
      case Type::BrokenGabled: return QStringLiteral( "broken_gabled" );
      case Type::Apex: return QStringLiteral( "apex" );
      case Type::Barrel: return QStringLiteral( "barrel" );
      case Type::Dome: return QStringLiteral( "dome" );
      case Type::Frustum: return QStringLiteral( "frustum" );
      case Type::Relief: return QStringLiteral( "relief" );
      case Type::None: return QString();
    }
    return QString();
  }

  inline Type typeFromId( const QString &id )
  {
    for ( Type type : { Type::Gabled, Type::Hipped, Type::BrokenGabled, Type::Apex,
                       Type::Barrel, Type::Dome, Type::Frustum, Type::Relief } )
    {
      if ( typeId( type ) == id )
        return type;
    }
    return Type::None;
  }

  inline QVariantMap toMap( const Selection &selection )
  {
    return { { QStringLiteral( "type" ), typeId( selection.type ) },
             { QStringLiteral( "point_cloud" ), selection.pointCloudLayerId } };
  }

  inline Selection fromMap( const QVariantMap &values )
  {
    return { typeFromId( values.value( QStringLiteral( "type" ) ).toString() ),
             values.value( QStringLiteral( "point_cloud" ) ).toString() };
  }

  inline QString propertyKey( QgsFeatureId fid )
  {
    return QStringLiteral( "building_edit_manager/roof/%1" ).arg( fid );
  }

  inline const char *previewKey()
  {
    return "buildingEditManagerRoofPreview";
  }

  inline bool isLegacyPointLayer( const QString &name )
  {
    return name == QLatin1String( "Roof_Edit_Points" )
           || name == QLatin1String( "Roof_Saved_Points" )
           || name == QLatin1String( "Roof_Edit_3D_Preview_Point" );
  }

  inline Type typeFromLegacyCounts( int boundaries, int ridges, int vertices, int surfaces )
  {
    if ( boundaries == 1 && ridges == 0 && vertices == 0 )
    {
      if ( surfaces == 1 ) return Type::Dome;
      if ( surfaces == 2 ) return Type::Barrel;
    }
    if ( boundaries == 1 && ridges == 0 && vertices == 1 && surfaces == 0 )
      return Type::Apex;
    if ( surfaces == 0 && vertices == 0 )
    {
      if ( boundaries == 2 && ridges == 0 ) return Type::Relief;
      if ( boundaries == 2 && ridges == 1 ) return Type::Frustum;
      if ( boundaries == 1 && ridges == 1 ) return Type::Gabled;
      if ( boundaries == 1 && ridges == 2 ) return Type::Hipped;
      if ( boundaries == 1 && ridges == 3 ) return Type::BrokenGabled;
    }
    return Type::None;
  }

  inline Selection legacySelection( QgsFeatureId fid )
  {
    // Read old roof types for compatibility, never their point coordinates.
    for ( const QString &name : { QStringLiteral( "Roof_Saved_Points" ), QStringLiteral( "Roof_Edit_Points" ) } )
    {
      for ( QgsMapLayer *mapLayer : QgsProject::instance()->mapLayersByName( name ) )
      {
        QgsVectorLayer *layer = qobject_cast<QgsVectorLayer *>( mapLayer );
        if ( !layer || layer->fields().indexOf( QStringLiteral( "building_fid" ) ) < 0
             || layer->fields().indexOf( QStringLiteral( "type" ) ) < 0 )
          continue;

        QgsFeatureRequest request;
        request.setFilterExpression( QStringLiteral( "building_fid = %1" ).arg( fid ) );
        request.setFlags( Qgis::FeatureRequestFlag::NoGeometry );
        QgsFeatureIterator it = layer->getFeatures( request );
        QgsFeature feature;
        int boundaries = 0, ridges = 0, vertices = 0, surfaces = 0;
        QString source;
        while ( it.nextFeature( feature ) )
        {
          const QString type = feature.attribute( QStringLiteral( "type" ) ).toString();
          if ( type.contains( QStringLiteral( "ridge" ), Qt::CaseInsensitive )
               || type.contains( QStringLiteral( "\u5c4b\u810a" ) )
               || type.contains( QStringLiteral( "\u705e\u5b2d\u5268" ) ) )
            ++ridges;
          else if ( type.contains( QStringLiteral( "vertex" ), Qt::CaseInsensitive ) || type.contains( QStringLiteral( "\u9876\u70b9" ) ) )
            ++vertices;
          else if ( type.contains( QStringLiteral( "surface" ), Qt::CaseInsensitive ) || type.contains( QStringLiteral( "\u66f2\u9762" ) ) )
            ++surfaces;
          else if ( !type.contains( QStringLiteral( "ground" ), Qt::CaseInsensitive )
                    && !type.contains( QStringLiteral( "\u5730\u9762" ) )
                    && !type.contains( QStringLiteral( "\u9366\u4f34\u6f70" ) ) )
            ++boundaries;
          if ( layer->fields().indexOf( QStringLiteral( "source" ) ) >= 0 )
            source = feature.attribute( QStringLiteral( "source" ) ).toString();
        }

        Selection selection;
        selection.type = typeFromLegacyCounts( boundaries, ridges, vertices, surfaces );
        if ( selection.type == Type::None )
          continue;
        for ( QgsMapLayer *candidate : QgsProject::instance()->mapLayersByName( source ) )
        {
          if ( qobject_cast<QgsPointCloudLayer *>( candidate ) )
          {
            selection.pointCloudLayerId = candidate->id();
            break;
          }
        }
        return selection;
      }
    }
    return {};
  }

  inline Selection read( QgsVectorLayer *layer, QgsFeatureId fid, bool includePreview = true )
  {
    if ( !layer || fid == FID_NULL )
      return {};
    const QVariantMap preview = layer->property( previewKey() ).toMap();
    if ( includePreview && !preview.isEmpty() && preview.value( QStringLiteral( "building_fid" ) ).toLongLong() == fid )
      return fromMap( preview );
    const QVariant saved = layer->customProperty( propertyKey( fid ) );
    if ( saved.isValid() )
      return fromMap( saved.toMap() );
    return legacySelection( fid );
  }

  inline void save( QgsVectorLayer *layer, QgsFeatureId fid, const Selection &selection )
  {
    // An explicit None masks legacy markers after deleting a roof.
    layer->setCustomProperty( propertyKey( fid ), toMap( selection ) );
    QgsProject::instance()->setDirty( true );
  }

  inline void setPreview( QgsVectorLayer *layer, QgsFeatureId fid, const Selection &selection )
  {
    QVariantMap values = toMap( selection );
    values.insert( QStringLiteral( "building_fid" ), fid );
    layer->setProperty( previewKey(), values );
  }

  inline bool clearPreview( QgsVectorLayer *layer, QgsFeatureId fid )
  {
    if ( !layer )
      return false;
    const QVariantMap preview = layer->property( previewKey() ).toMap();
    if ( preview.isEmpty() || preview.value( QStringLiteral( "building_fid" ) ).toLongLong() != fid )
      return false;
    layer->setProperty( previewKey(), QVariant() );
    return true;
  }

  inline QList<BuildingRoof::RoofPoint> builderMarkers( Type type )
  {
    // Type-only compatibility tokens keep the existing builders unchanged.
    int boundaries = 1, ridges = 0, vertices = 0, surfaces = 0;
    switch ( type )
    {
      case Type::Gabled: ridges = 1; break;
      case Type::Hipped: ridges = 2; break;
      case Type::BrokenGabled: ridges = 3; break;
      case Type::Apex: vertices = 1; break;
      case Type::Barrel: surfaces = 2; break;
      case Type::Dome: surfaces = 1; break;
      case Type::Frustum: boundaries = 2; ridges = 1; break;
      case Type::Relief: boundaries = 2; break;
      case Type::None: return {};
    }
    QList<BuildingRoof::RoofPoint> markers;
    for ( int i = 0; i < boundaries; ++i )
      markers.append( BuildingRoof::RoofPoint{ QgsPoint( 0, 0, 0 ), QStringLiteral( "boundary" ) } );
    for ( int i = 0; i < ridges; ++i )
      markers.append( BuildingRoof::RoofPoint{ QgsPoint( 0, 0, 0 ), QStringLiteral( "ridge" ) } );
    for ( int i = 0; i < vertices; ++i )
      markers.append( BuildingRoof::RoofPoint{ QgsPoint( 0, 0, 0 ), QStringLiteral( "vertex" ) } );
    for ( int i = 0; i < surfaces; ++i )
      markers.append( BuildingRoof::RoofPoint{ QgsPoint( 0, 0, 0 ), QStringLiteral( "surface" ) } );
    return markers;
  }
}
