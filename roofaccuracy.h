#pragma once

#include "roofmodelsettings.h"

#include <QObject>
#include <functional>
#include <memory>

class QgisInterface;
class QWidget;
class QgsVectorLayer;
namespace Ui
{
class roofedit;
}

namespace RoofAccuracy
{
struct PointError
{
  QgsPoint point;
  double distance = 0.0;
};

struct Result
{
  QVector<PointError> points;
  double mae = 0.0;
  double rmse = 0.0;
  bool canceled = false;
  QString error;
  double inlierRate ( double tolerance ) const;
};

// Inputs use metres. Optional display points preserve the original layer coordinates.
Result evaluate ( const QgsGeometry &footprint, const BuildingRoof::Mesh &mesh,
                  const QVector<BuildingRoof::RoofSample> &samples,
                  const QVector<QgsPoint> &displayPoints = {}, const std::function<bool ()> &canceled = {},
                  const std::function<void ( double )> &progress = {} );

// All evaluation, UI state, cancellation and 3D display live in this component.
class Controller final : public QObject
{
public:
  Controller ( QWidget *panel, Ui::roofedit &ui, QgisInterface *iface, QObject *parent );
  ~Controller () override;
  void setContext ( QgsVectorLayer *layer, QgsFeatureId fid, const RoofModelSettings::Selection &selection );
  void clear ();

private:
  struct Private;
  std::unique_ptr<Private> d;
};
}
