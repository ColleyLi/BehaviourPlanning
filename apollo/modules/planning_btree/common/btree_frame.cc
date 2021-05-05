#include "modules/planning_btree/common/btree_frame.h"

namespace apollo {
namespace planning_btree {

BTreeFrame::BTreeFrame(uint32_t sequence_num)
    : sequence_num_(sequence_num) {}

BTreeFrame::BTreeFrame(uint32_t sequence_num, const PlanningInput &planning_input,
             const common::TrajectoryPoint &planning_start_point,
             const common::VehicleState &vehicle_state,
             ReferenceLineProvider *reference_line_provider)
    : sequence_num_(sequence_num),
      planning_input_(planning_input),
      planning_start_point_(planning_start_point),
      vehicle_state_(vehicle_state),
      reference_line_provider_(reference_line_provider) {}

BTreeFrame::BTreeFrame(uint32_t sequence_num, const PlanningInput &planning_input,
             const common::TrajectoryPoint &planning_start_point,
             const common::VehicleState &vehicle_state)
    : BTreeFrame(sequence_num, planning_input, planning_start_point, vehicle_state,
            nullptr) {}


common::Status BTreeFrame::Init(
      const common::VehicleStateProvider *vehicle_state_provider,
      const std::list<ReferenceLine> &reference_lines,
      const std::list<hdmap::RouteSegments> &segments,
      const std::vector<routing::LaneWaypoint> &future_route_waypoints)
{
  AERROR << "Size of ref lines: " << reference_lines.size();

  for (auto& segment: segments)
  {
    AERROR << "Segment: " << segment.Id() << " " << segment.NextAction();
  }

  // future_route_waypoints_ = future_route_waypoints;
  
  
  // for (auto &ptr : Obstacle::CreateObstacles(*local_view_.prediction_obstacles)) 
  // {
  //   AddObstacle(*ptr);
  // }

  dynamic_reference_lines_.clear();
  auto ref_line_iter = reference_lines.begin();
  auto segments_iter = segments.begin();
  while (ref_line_iter != reference_lines.end()) 
  {
    if (segments_iter->StopForDestination()) 
    {
      is_near_destination_ = true;
    }
    dynamic_reference_lines_.emplace_back(vehicle_state_, planning_start_point_,
                                      *ref_line_iter, *segments_iter);
    ++ref_line_iter;
    ++segments_iter;
  }

  // for (auto &ref_line : dynamic_reference_lines_) 
  // {
    // if (!ref_line.Init(obstacles())) 
    // {
      // AERROR << "Failed to init reference line";
      // }
  // }

  return common::Status::OK();
}

}  // namespace planning_btree
}  // namespace apollo