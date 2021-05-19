#include "modules/planning_btree/common/btree_frame.h"

namespace apollo {
namespace planning_btree {

using apollo::common::math::Box2d;
using apollo::common::math::Polygon2d;

// TODO: move to config
namespace {
constexpr double kVirtualStopWallLength = 0.1;
constexpr double kStopWallWidth = 4.0;
}  // namespace

BTreeFrame::BTreeFrame(uint32_t sequence_num) : sequence_num_(sequence_num) {}

BTreeFrame::BTreeFrame(uint32_t sequence_num,
                       const PlanningInput &planning_input,
                       const common::TrajectoryPoint &planning_start_point,
                       const common::VehicleState &vehicle_state,
                       ReferenceLineProvider *reference_line_provider)
    : sequence_num_(sequence_num),
      planning_input_(planning_input),
      planning_start_point_(planning_start_point),
      vehicle_state_(vehicle_state),
      reference_line_provider_(reference_line_provider) {}

BTreeFrame::BTreeFrame(uint32_t sequence_num,
                       const PlanningInput &planning_input,
                       const common::TrajectoryPoint &planning_start_point,
                       const common::VehicleState &vehicle_state)
    : BTreeFrame(sequence_num, planning_input, planning_start_point,
                 vehicle_state, nullptr) {}

common::Status BTreeFrame::Init(
    const common::VehicleStateProvider *vehicle_state_provider,
    const std::list<ReferenceLine> &reference_lines,
    const std::list<hdmap::RouteSegments> &segments,
    const std::vector<routing::LaneWaypoint> &future_route_waypoints) {
  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  // AERROR << "Size of ref lines: " << reference_lines.size();

  // for (auto& segment: segments)
  // {
  //   AERROR << "Segment: " << segment.Id() << " " << segment.NextAction();
  // }

  // future_route_waypoints_ = future_route_waypoints;

  for (auto &ptr :
       Obstacle::CreateObstacles(*planning_input_.prediction_obstacles)) {
    AddObstacle(*ptr);
  }

  dynamic_reference_lines_.clear();
  auto ref_line_iter = reference_lines.begin();
  auto segments_iter = segments.begin();
  while (ref_line_iter != reference_lines.end()) {
    if (segments_iter->StopForDestination()) {
      is_near_destination_ = true;
    }
    dynamic_reference_lines_.emplace_back(vehicle_state_, planning_start_point_,
                                          *ref_line_iter, *segments_iter);
    ++ref_line_iter;
    ++segments_iter;
  }

  for (auto &ref_line : dynamic_reference_lines_) {
    if (!ref_line.Init(GetObstacles())) {
      AERROR << "Failed to init reference line";
    }
  }

  return common::Status::OK();
}

const std::vector<const Obstacle *> BTreeFrame::GetObstacles() const {
  return obstacles_.Items();
}

const Obstacle *BTreeFrame::CreateStopObstacle(
    DynamicReferenceLine *const dynamic_reference_line,
    const std::string &obstacle_id, const double obstacle_s) {
  if (dynamic_reference_line == nullptr) {
    AERROR << "dynamic_reference_line nullptr";
    return nullptr;
  }

  const auto &reference_line = dynamic_reference_line->GetReferenceLine();
  const double box_center_s = obstacle_s + kVirtualStopWallLength / 2.0;
  auto box_center = reference_line.GetReferencePoint(box_center_s);
  double heading = reference_line.GetReferencePoint(obstacle_s).heading();
  Box2d stop_wall_box{box_center, heading, kVirtualStopWallLength,
                      kStopWallWidth};

  return CreateStaticVirtualObstacle(obstacle_id, stop_wall_box);
}

/**
 * @brief: create static virtual object with lane width,
 *         mainly used for virtual stop wall
 */
const Obstacle *BTreeFrame::CreateStopObstacle(const std::string &obstacle_id,
                                               const std::string &lane_id,
                                               const double lane_s) {
  if (!hdmap_) {
    AERROR << "Invalid HD Map.";
    return nullptr;
  }
  const auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(lane_id));
  if (!lane) {
    AERROR << "Failed to find lane[" << lane_id << "]";
    return nullptr;
  }

  double dest_lane_s = std::max(0.0, lane_s);
  auto dest_point = lane->GetSmoothPoint(dest_lane_s);

  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  lane->GetWidth(dest_lane_s, &lane_left_width, &lane_right_width);

  Box2d stop_wall_box{{dest_point.x(), dest_point.y()},
                      lane->Heading(dest_lane_s),
                      kVirtualStopWallLength,
                      lane_left_width + lane_right_width};

  return CreateStaticVirtualObstacle(obstacle_id, stop_wall_box);
}

const Obstacle *BTreeFrame::CreateStaticObstacle(
    DynamicReferenceLine *const dynamic_reference_line,
    const std::string &obstacle_id, const double obstacle_start_s,
    const double obstacle_end_s) {
  if (dynamic_reference_line == nullptr) {
    AERROR << "dynamic_reference_line nullptr";
    return nullptr;
  }

  const auto &reference_line = dynamic_reference_line->GetReferenceLine();

  // start_xy
  common::SLPoint sl_point;
  sl_point.set_s(obstacle_start_s);
  sl_point.set_l(0.0);
  common::math::Vec2d obstacle_start_xy;
  if (!reference_line.SLToXY(sl_point, &obstacle_start_xy)) {
    AERROR << "Failed to get start_xy from sl: " << sl_point.DebugString();
    return nullptr;
  }

  // end_xy
  sl_point.set_s(obstacle_end_s);
  sl_point.set_l(0.0);
  common::math::Vec2d obstacle_end_xy;
  if (!reference_line.SLToXY(sl_point, &obstacle_end_xy)) {
    AERROR << "Failed to get end_xy from sl: " << sl_point.DebugString();
    return nullptr;
  }

  double left_lane_width = 0.0;
  double right_lane_width = 0.0;
  if (!reference_line.GetLaneWidth(obstacle_start_s, &left_lane_width,
                                   &right_lane_width)) {
    AERROR << "Failed to get lane width at s[" << obstacle_start_s << "]";
    return nullptr;
  }

  common::math::Box2d obstacle_box{
      common::math::LineSegment2d(obstacle_start_xy, obstacle_end_xy),
      left_lane_width + right_lane_width};

  return CreateStaticVirtualObstacle(obstacle_id, obstacle_box);
}

const Obstacle *BTreeFrame::CreateStaticVirtualObstacle(const std::string &id,
                                                        const Box2d &box) {
  const auto *object = obstacles_.Find(id);
  if (object) {
    AWARN << "obstacle " << id << " already exist.";
    return object;
  }
  auto *ptr =
      obstacles_.Add(id, *Obstacle::CreateStaticVirtualObstacles(id, box));
  if (!ptr) {
    AERROR << "Failed to create virtual obstacle " << id;
  }
  return ptr;
}

void BTreeFrame::AddObstacle(const Obstacle &obstacle) {
  obstacles_.Add(obstacle.Id(), obstacle);
}

const std::list<DynamicReferenceLine> &BTreeFrame::GetDynamicReferenceLines()
    const {
  return dynamic_reference_lines_;
}

std::list<DynamicReferenceLine> *BTreeFrame::GetMutableDynamicReferenceLines() {
  return &dynamic_reference_lines_;
}

void BTreeFrame::SetCurrentDynamicReferenceLine(
    DynamicReferenceLine *dynamic_reference_line) {
  current_dynamic_reference_line_ = dynamic_reference_line;
}

const DynamicReferenceLine &BTreeFrame::GetCurrentDynamicReferenceLine() const {
  return *current_dynamic_reference_line_;
}

DynamicReferenceLine *BTreeFrame::GetMutableCurrentDynamicReferenceLine() {
  return current_dynamic_reference_line_;
}

const common::TrajectoryPoint &BTreeFrame::GetPlanningStartPoint() const {
  return planning_start_point_;
}

}  // namespace planning_btree
}  // namespace apollo