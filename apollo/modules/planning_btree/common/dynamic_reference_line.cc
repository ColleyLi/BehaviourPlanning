#include "modules/planning_btree/common/dynamic_reference_line.h"

namespace apollo {
namespace planning_btree {

using apollo::common::TrajectoryPoint;

DynamicReferenceLine::DynamicReferenceLine(
                                     const common::VehicleState& vehicle_state,
                                     const TrajectoryPoint& adc_planning_point,
                                     const ReferenceLine& reference_line,
                                     const hdmap::RouteSegments& route_segments)
    : vehicle_state_(vehicle_state),
      adc_planning_point_(adc_planning_point),
      reference_line_(reference_line),
      route_segments_(route_segments) {}

}  // namespace planning_btree
}  // namespace apollo