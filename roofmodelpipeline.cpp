#include "roofmodelpipeline.h"

#include <qgsgeos.h>
#include <qgspointcloudattribute.h>
#include <qgspointcloudblock.h>
#include <qgspointclouddataprovider.h>
#include <qgspointcloudindex.h>
#include <qgspointcloudlayer.h>
#include <qgspointcloudrequest.h>
#include <qgsproject.h>

#include <QCache>
#include <QDataStream>
#include <QObject>
#include <QPointer>
#include <algorithm>
#include <cmath>
#include <memory>

namespace
{
  QgsPointCloudLayer *pointCloudLayer( const QString &id )
  {
    if ( !id.isEmpty() )
      return qobject_cast<QgsPointCloudLayer *>( QgsProject::instance()->mapLayer( id ) );
    for ( QgsMapLayer *layer : QgsProject::instance()->mapLayers().values() )
    {
      auto *cloud = qobject_cast<QgsPointCloudLayer *>( layer );
      if ( cloud && cloud->dataProvider() )
        return cloud;
    }
    return nullptr;
  }

  void collectNodes( const QgsPointCloudIndex &index, const QgsPointCloudNodeId &id,
                     const QgsRectangle &extent, QList<QgsPointCloudNodeId> &nodes )
  {
    const QgsRectangle bounds = QgsPointCloudNode::bounds( index.rootNodeBounds(), id ).toRectangle();
    if ( !bounds.isNull() && !extent.intersects( bounds ) )
      return;
    const QgsPointCloudNode node = index.getNode( id );
    if ( node.pointCount() != 0 )
      nodes.append( id );
    for ( const QgsPointCloudNodeId &child : node.children() )
      collectNodes( index, child, extent, nodes );
  }

  // Keep one cloud crop, but retain completed meshes across footprint changes within a memory budget.
  class RoofCache : public QObject
  {
    public:
      RoofCache()
        : models( 64 * 1024 )
      {
      }

      void clearSamples()
      {
        ++revision;
        samples.clear();
        loaded = false;
      }

      void invalidate()
      {
        clearSamples();
        models.clear();
      }

      QByteArray modelKey( RoofModelSettings::Type type ) const
      {
        QByteArray key;
        QDataStream stream( &key, QIODevice::WriteOnly );
        stream << geometryKey << static_cast<qint32>( type );
        return key;
      }

      void storeModel( const QByteArray &key, const BuildingRoof::MeshResult &result )
      {
        const qint64 bytes = static_cast<qint64>( result.mesh.vertices.size() ) * sizeof( QgsPoint )
                             + static_cast<qint64>( result.mesh.indices.size() ) * sizeof( int )
                             + static_cast<qint64>( result.mesh.structureLines.size() ) * sizeof( QPair<QgsPoint, QgsPoint> )
                             + static_cast<qint64>( result.error.size() ) * sizeof( QChar ) + key.size() + sizeof( result );
        const qint64 cost = std::max<qint64>( 1, ( bytes + 1023 ) / 1024 );
        if ( cost <= models.maxCost() )
          models.insert( key, new BuildingRoof::MeshResult( result ), static_cast<int>( cost ) );
      }

      bool prepare( const QgsGeometry &geometry, const QString &cloudId, bool loadSamples = true )
      {
        QgsPointCloudLayer *nextCloud = pointCloudLayer( cloudId );
        const QByteArray nextGeometry = geometry.asWkb();
        if ( cloud != nextCloud )
        {
          for ( const QMetaObject::Connection &connection : connections )
            QObject::disconnect( connection );
          connections.clear();
          cloud = nextCloud;
          invalidate();
          if ( cloud )
          {
            connections.append( connect( cloud, &QgsMapLayer::dataChanged, this, [this]() { invalidate(); } ) );
            connections.append( connect( cloud, &QgsMapLayer::dataSourceChanged, this, [this]() { invalidate(); } ) );
            connections.append( connect( cloud, &QgsPointCloudLayer::subsetStringChanged, this, [this]() { invalidate(); } ) );
            connections.append( connect( cloud, &QObject::destroyed, this, [this]() { invalidate(); } ) );
          }
        }
        if ( geometryKey != nextGeometry )
        {
          geometryKey = nextGeometry;
          clearSamples();
        }
        if ( !cloud || !cloud->dataProvider() || geometry.isNull() || geometry.isEmpty() )
          return false;
        if ( provider != cloud->dataProvider() )
        {
          QObject::disconnect( indexChangedConnection );
          provider = cloud->dataProvider();
          indexChangedConnection = connect( provider.data(), &QgsPointCloudDataProvider::indexGenerationStateChanged,
                                            this, [this]() { invalidate(); } );
          invalidate();
        }
        if ( !loadSamples || loaded )
          return true;

        const quint64 readRevision = revision;
        QgsPointCloudIndex index = cloud->dataProvider()->index();
        if ( !index.isValid() || !index.root().isValid() || !index.hasNode( index.root() ) )
          return false;

        QgsPointCloudAttributeCollection requestedAttributes;
        const QgsPointCloudAttributeCollection availableAttributes = index.attributes();
        for ( const QString &name : { QStringLiteral( "X" ), QStringLiteral( "Y" ), QStringLiteral( "Z" ) } )
        {
          int offset = 0;
          const QgsPointCloudAttribute *attribute = availableAttributes.find( name, offset );
          if ( !attribute )
            return false;
          requestedAttributes.push_back( *attribute );
        }

        const QgsRectangle extent = geometry.boundingBox();
        QList<QgsPointCloudNodeId> nodes;
        collectNodes( index, index.root(), extent, nodes );
        QgsPointCloudRequest request;
        request.setFilterRect( extent );
        request.setAttributes( requestedAttributes );
        QgsGeos footprint( geometry.constGet() );
        footprint.prepareGeometry();
        QVector<BuildingRoof::RoofSample> collected;
        bool complete = true;
        for ( const QgsPointCloudNodeId &node : nodes )
        {
          std::unique_ptr<QgsPointCloudBlock> block = index.nodeData( node, request );
          if ( !block )
          {
            complete = false;
            continue;
          }
          const QgsPointCloudAttributeCollection attributes = block->attributes();
          int xOffset = 0, yOffset = 0, zOffset = 0;
          const QgsPointCloudAttribute *xAttribute = attributes.find( QStringLiteral( "X" ), xOffset );
          const QgsPointCloudAttribute *yAttribute = attributes.find( QStringLiteral( "Y" ), yOffset );
          const QgsPointCloudAttribute *zAttribute = attributes.find( QStringLiteral( "Z" ), zOffset );
          if ( !xAttribute || !yAttribute || !zAttribute )
          {
            complete = false;
            continue;
          }
          const int recordSize = attributes.pointRecordSize();
          const QgsVector3D scale = block->scale();
          const QgsVector3D offset = block->offset();
          for ( int i = 0; i < block->pointCount(); ++i )
          {
            double x = 0.0, y = 0.0, z = 0.0;
            QgsPointCloudAttribute::getPointXYZ( block->data(), i, recordSize,
                                                 xOffset, xAttribute->type(), yOffset, yAttribute->type(), zOffset, zAttribute->type(),
                                                 scale, offset, x, y, z );
            if ( !std::isfinite( x ) || !std::isfinite( y ) || !std::isfinite( z ) || z < 0.0
                 || !extent.contains( x, y ) || !footprint.contains( x, y ) )
              continue;
            collected.append( BuildingRoof::RoofSample{ QgsPoint( x, y, z ) } );
          }
        }
        if ( readRevision != revision )
          return false;
        samples = collected;
        loaded = complete;
        return true;
      }

      QPointer<QgsPointCloudLayer> cloud;
      QPointer<QgsPointCloudDataProvider> provider;
      QByteArray geometryKey;
      QVector<BuildingRoof::RoofSample> samples;
      QCache<QByteArray, BuildingRoof::MeshResult> models;
      bool loaded = false;
      quint64 revision = 0;
      QList<QMetaObject::Connection> connections;
      QMetaObject::Connection indexChangedConnection;
  };

  RoofCache &cache()
  {
    static RoofCache instance;
    return instance;
  }
}

QVector<BuildingRoof::RoofSample> RoofModelPipeline::collectSamples( const QgsGeometry &geometry, const QString &pointCloudLayerId )
{
  RoofCache &entry = cache();
  return entry.prepare( geometry, pointCloudLayerId ) ? entry.samples : QVector<BuildingRoof::RoofSample>();
}

BuildingRoof::MeshResult RoofModelPipeline::build( const QgsGeometry &geometry, const RoofModelSettings::Selection &selection )
{
  RoofCache &entry = cache();
  const bool sourceReady = entry.prepare( geometry, selection.pointCloudLayerId, false );
  const QByteArray key = entry.modelKey( selection.type );
  if ( sourceReady )
  {
    if ( const BuildingRoof::MeshResult *found = entry.models.object( key ) )
      return *found;
  }
  const bool prepared = sourceReady && entry.prepare( geometry, selection.pointCloudLayerId );
  // Automatic roofs infer all elevations from the cloud; the prism height is not an input.
  const BuildingRoof::MeshResult result = BuildingRoof::buildRoofPrismMesh(
    geometry, 1.0, RoofModelSettings::builderMarkers( selection.type ),
    prepared ? entry.samples : QVector<BuildingRoof::RoofSample>() );
  // A failed node read must be retried, not frozen as an incomplete model in the cache.
  if ( prepared && entry.loaded )
    entry.storeModel( key, result );
  return result;
}
