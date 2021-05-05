#pragma once

#include "modules/common/proto/drive_state.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning_btree/reference_line/reference_line.h"

namespace apollo {
namespace planning_btree {

class DynamicReferenceLine
{
  public:
    DynamicReferenceLine() = default;

    DynamicReferenceLine(const common::VehicleState& vehicle_state,
                         const common::TrajectoryPoint& adc_planning_point,
                         const ReferenceLine& reference_line,
                         const hdmap::RouteSegments& route_segments);

  private:
    const common::VehicleState vehicle_state_;
    const common::TrajectoryPoint adc_planning_point_;
    ReferenceLine reference_line_;
    hdmap::RouteSegments route_segments_;
};

}  // namespace planning_btree
}  // namespace apollo