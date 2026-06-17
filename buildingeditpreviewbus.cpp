#include "buildingeditpreviewbus.h"

BuildingEditPreviewBus *BuildingEditPreviewBus::instance()
{
  static BuildingEditPreviewBus bus;
  return &bus;
}
