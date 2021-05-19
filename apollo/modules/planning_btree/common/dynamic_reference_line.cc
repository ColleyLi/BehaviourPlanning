#include "modules/planning_btree/common/dynamic_reference_line.h"

namespace apollo {
namespace planning_btree {

using apollo::common::TrajectoryPoint;
using apollo::common::VehicleConfigHelper;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;

DynamicReferenceLine::DynamicReferenceLine(
    const common::VehicleState& vehicle_state,
    const TrajectoryPoint& adc_planning_point,
    const ReferenceLine& reference_line,
    const hdmap::RouteSegments& route_segments)
    : vehicle_state_(vehicle_state),
      adc_planning_point_(adc_planning_point),
      reference_line_(reference_line),
      route_segments_(route_segments) {}

bool DynamicReferenceLine::Init(const std::vector<const Obstacle*>& obstacles) {
  const auto& param = VehicleConfigHelper::GetConfig().vehicle_param();
  // stitching point
  const auto& path_point = adc_planning_point_.path_point();
  Vec2d position(path_point.x(), path_point.y());
  Vec2d vec_to_center(
      (param.front_edge_to_center() - param.back_edge_to_center()) / 2.0,
      (param.left_edge_to_center() - param.right_edge_to_center()) / 2.0);
  Vec2d center(position + vec_to_center.rotate(path_point.theta()));
  Box2d box(center, path_point.theta(), param.length(), param.width());
  // realtime vehicle position
  Vec2d vehicle_position(vehicle_state_.x(), vehicle_state_.y());
  Vec2d vehicle_center(vehicle_position +
                       vec_to_center.rotate(vehicle_state_.heading()));
  Box2d vehicle_box(vehicle_center, vehicle_state_.heading(), param.length(),
                    param.width());

  if (!reference_line_.GetSLBoundary(box, &adc_sl_boundary_)) {
    AERROR << "Failed to get ADC boundary from box: " << box.DebugString();
    return false;
  }

  // InitFirstOverlaps();

  if (adc_sl_boundary_.end_s() < 0 ||
      adc_sl_boundary_.start_s() > reference_line_.Length()) {
    AWARN << "Vehicle SL " << adc_sl_boundary_.ShortDebugString()
          << " is not on reference line:[0, " << reference_line_.Length()
          << "]";
  }

  static constexpr double kOutOfReferenceLineL = 10.0;  // in meters
  if (adc_sl_boundary_.start_l() > kOutOfReferenceLineL ||
      adc_sl_boundary_.end_l() < -kOutOfReferenceLineL) {
    AERROR << "Ego vehicle is too far away from reference line.";
    return false;
  }

  is_on_reference_line_ = reference_line_.IsOnLane(adc_sl_boundary_);

  if (!AddObstacles(obstacles)) {
    AERROR << "Failed to add obstacles to reference line";
    return false;
  }

  // const auto& map_path = reference_line_.map_path();
  // for (const auto& speed_bump : map_path.speed_bump_overlaps()) {
  //   // -1 and + 1.0 are added to make sure it can be sampled.
  //   reference_line_.AddSpeedLimit(speed_bump.start_s - 1.0,
  //                                 speed_bump.end_s + 1.0,
  //                                 FLAGS_speed_bump_speed_limit);
  // }

  // SetCruiseSpeed(FLAGS_default_cruise_speed);

  // vehicle_signal_.Clear();

  is_drivable_ = false;
  cost_ = -1.0;

  return true;
}

const SLBoundary& DynamicReferenceLine::GetADCSLBoundary() const {
  return adc_sl_boundary_;
}

ObstacleDecisions* DynamicReferenceLine::GetMutableObstacleDecisions() {
  return &obstacle_decisions_;
}

const ObstacleDecisions& DynamicReferenceLine::GetObstacleDecisions() const {
  return obstacle_decisions_;
}

Obstacle* DynamicReferenceLine::GetBlockingObstacle() const {
  return blocking_obstacle_;
}

void DynamicReferenceLine::SetBlockingObstacle(
    const std::string& blocking_obstacle_id) {
  blocking_obstacle_ = obstacle_decisions_.Find(blocking_obstacle_id);
}

bool DynamicReferenceLine::AddObstacles(
    const std::vector<const Obstacle*>& obstacles) {
  bool use_multi_thread_to_add_obstacles = false;
  if (use_multi_thread_to_add_obstacles) {
    // std::vector<std::future<Obstacle*>> results;
    // for (const auto* obstacle : obstacles) {
    //   results.push_back(
    //       cyber::Async(&ReferenceLineInfo::AddObstacle, this, obstacle));
    // }
    // for (auto& result : results) {
    //   if (!result.get()) {
    //     AERROR << "Fail to add obstacles.";
    //     return false;
    //   }
    // }
  } else {
    for (const auto* obstacle : obstacles) {
      if (!AddObstacle(obstacle)) {
        AERROR << "Failed to add obstacle " << obstacle->Id();
        return false;
      }
    }
  }

  return true;
}

Obstacle* DynamicReferenceLine::AddObstacle(const Obstacle* obstacle) {
  if (!obstacle) {
    AERROR << "The provided obstacle is empty";
    return nullptr;
  }
  auto* mutable_obstacle = obstacle_decisions_.AddObstacle(*obstacle);
  if (!mutable_obstacle) {
    AERROR << "failed to add obstacle " << obstacle->Id();
    return nullptr;
  }

  SLBoundary perception_sl;
  if (!reference_line_.GetSLBoundary(obstacle->PerceptionBoundingBox(),
                                     &perception_sl)) {
    AERROR << "Failed to get sl boundary for obstacle: " << obstacle->Id();
    return mutable_obstacle;
  }
  mutable_obstacle->SetPerceptionSlBoundary(perception_sl);
  mutable_obstacle->CheckLaneBlocking(reference_line_);
  if (mutable_obstacle->IsLaneBlocking()) {
    ADEBUG << "obstacle [" << obstacle->Id() << "] is lane blocking.";
  } else {
    ADEBUG << "obstacle [" << obstacle->Id() << "] is NOT lane blocking.";
  }

  if (IsIrrelevantObstacle(*mutable_obstacle)) {
    ObjectDecisionType ignore;
    ignore.mutable_ignore();
    obstacle_decisions_.AddLateralDecision("reference_line_filter",
                                           obstacle->Id(), ignore);
    obstacle_decisions_.AddLongitudinalDecision("reference_line_filter",
                                                obstacle->Id(), ignore);
  } else {
    mutable_obstacle->BuildReferenceLineStBoundary(reference_line_,
                                                   adc_sl_boundary_.start_s());
  }
  return mutable_obstacle;
}

bool DynamicReferenceLine::IsIrrelevantObstacle(const Obstacle& obstacle) {
  if (obstacle.IsCautionLevelObstacle()) {
    return false;
  }
  // if adc is on the road, and obstacle behind adc, ignore
  const auto& obstacle_boundary = obstacle.PerceptionSLBoundary();
  if (obstacle_boundary.end_s() > reference_line_.Length()) {
    return true;
  }
  if (is_on_reference_line_ && !IsLaneChangePath() &&
      obstacle_boundary.end_s() < adc_sl_boundary_.end_s() &&
      (reference_line_.IsOnLane(obstacle_boundary) ||
       obstacle_boundary.end_s() < 0.0)) {  // if obstacle is far backward
    return true;
  }
  return false;
}

const ReferenceLine& DynamicReferenceLine::GetReferenceLine() const {
  return reference_line_;
}

ReferenceLine* DynamicReferenceLine::GetMutableReferenceLine() {
  return &reference_line_;
}

const std::vector<PathBoundary>&
DynamicReferenceLine::GetCandidatePathBoundaries() const {
  return candidate_path_boundaries_;
}

void DynamicReferenceLine::SetCandidatePathBoundaries(
    std::vector<PathBoundary>&& path_boundaries) {
  candidate_path_boundaries_ = std::move(path_boundaries);
}

const std::vector<PathData>& DynamicReferenceLine::GetCandidatePathData()
    const {
  return candidate_path_data_;
}

void DynamicReferenceLine::SetCandidatePathData(
    std::vector<PathData>&& candidate_path_data) {
  candidate_path_data_ = std::move(candidate_path_data);
}

const PathData& DynamicReferenceLine::GetPathData() const { return path_data_; }

PathData* DynamicReferenceLine::GetMutablePathData() { return &path_data_; }

const SpeedData& DynamicReferenceLine::GetSpeedData() const {
  return speed_data_;
}

SpeedData* DynamicReferenceLine::GetMutableSpeedData() { return &speed_data_; }

const double DynamicReferenceLine::GetCost() const { return cost_; }

void DynamicReferenceLine::SetCost(double cost) { cost_ = cost; }

void DynamicReferenceLine::AddCost(double cost) { cost_ += cost; }

bool DynamicReferenceLine::IsDrivable() const { return is_drivable_; }

void DynamicReferenceLine::SetDrivable(bool drivable) {
  is_drivable_ = drivable;
}

bool DynamicReferenceLine::IsLaneChangePath() const {
  return !GetRouteSegments().IsOnSegment();
}

const hdmap::RouteSegments& DynamicReferenceLine::GetRouteSegments() const {
  return route_segments_;
}

const DiscretizedTrajectory& DynamicReferenceLine::GetDiscretizedTrajectory()
    const {
  return discretized_trajectory_;
}

void DynamicReferenceLine::SetDiscretizedTrajectory(
    const DiscretizedTrajectory& trajectory) {
  discretized_trajectory_ = trajectory;
}

bool DynamicReferenceLine::CombinePathAndSpeedProfiles(
    const double relative_time, const double start_s,
    DiscretizedTrajectory* ptr_discretized_trajectory) {
  ACHECK(ptr_discretized_trajectory != nullptr);
  // use varied resolution to reduce data load but also provide enough data
  // point for control module

  // TODO: move this to config
  const double kDenseTimeResoltuion = 0.02;
  const double kSparseTimeResolution = 0.1;
  const double kDenseTimeSec = 1.0;

  if (path_data_.discretized_path().empty()) {
    AERROR << GetRouteSegments().Id() << " path data is empty";
    return false;
  }

  if (speed_data_.empty()) {
    AERROR << GetRouteSegments().Id() << "speed profile is empty";
    return false;
  }

  for (double cur_rel_time = 0.0; cur_rel_time < speed_data_.TotalTime();
       cur_rel_time += (cur_rel_time < kDenseTimeSec ? kDenseTimeResoltuion
                                                     : kSparseTimeResolution)) {
    common::SpeedPoint speed_point;
    if (!speed_data_.EvaluateByTime(cur_rel_time, &speed_point)) {
      AERROR << "Fail to get speed point with relative time " << cur_rel_time;
      return false;
    }

    if (speed_point.s() > path_data_.discretized_path().Length()) {
      break;
    }
    common::PathPoint path_point =
        path_data_.GetPathPointWithPathS(speed_point.s());
    path_point.set_s(path_point.s() + start_s);

    common::TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_point.set_v(speed_point.v());
    trajectory_point.set_a(speed_point.a());
    trajectory_point.set_relative_time(speed_point.t() + relative_time);
    ptr_discretized_trajectory->AppendTrajectoryPoint(trajectory_point);
  }
  return true;
}


bool DynamicReferenceLine::GetNeighborLaneInfo(
    const DynamicReferenceLine::LaneType lane_type, const double s,
    hdmap::Id* ptr_lane_id, double* ptr_lane_width) const {
  auto ptr_lane_info = LocateLaneInfo(s);
  if (ptr_lane_info == nullptr) {
    return false;
  }

  switch (lane_type) {
    case LaneType::LeftForward: {
      if (ptr_lane_info->lane().left_neighbor_forward_lane_id().empty()) {
        return false;
      }
      *ptr_lane_id = ptr_lane_info->lane().left_neighbor_forward_lane_id(0);
      break;
    }
    case LaneType::LeftReverse: {
      if (ptr_lane_info->lane().left_neighbor_reverse_lane_id().empty()) {
        return false;
      }
      *ptr_lane_id = ptr_lane_info->lane().left_neighbor_reverse_lane_id(0);
      break;
    }
    case LaneType::RightForward: {
      if (ptr_lane_info->lane().right_neighbor_forward_lane_id().empty()) {
        return false;
      }
      *ptr_lane_id = ptr_lane_info->lane().right_neighbor_forward_lane_id(0);
      break;
    }
    case LaneType::RightReverse: {
      if (ptr_lane_info->lane().right_neighbor_reverse_lane_id().empty()) {
        return false;
      }
      *ptr_lane_id = ptr_lane_info->lane().right_neighbor_reverse_lane_id(0);
      break;
    }
    default:
      ACHECK(false);
  }
  auto ptr_neighbor_lane =
      hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(*ptr_lane_id);
  if (ptr_neighbor_lane == nullptr) {
    return false;
  }

  auto ref_point = reference_line_.GetReferencePoint(s);

  double neighbor_s = 0.0;
  double neighbor_l = 0.0;
  if (!ptr_neighbor_lane->GetProjection({ref_point.x(), ref_point.y()},
                                        &neighbor_s, &neighbor_l)) {
    return false;
  }

  *ptr_lane_width = ptr_neighbor_lane->GetWidth(neighbor_s);
  return true;
}

hdmap::LaneInfoConstPtr DynamicReferenceLine::LocateLaneInfo(
    const double s) const {
  std::vector<hdmap::LaneInfoConstPtr> lanes;
  reference_line_.GetLaneFromS(s, &lanes);
  if (lanes.empty()) {
    AWARN << "cannot get any lane using s";
    return nullptr;
  }

  return lanes.front();
}


bool DynamicReferenceLine::ReachedDestination() const {
  static constexpr double kDestinationDeltaS = 0.6;
  const double distance_destination = SDistanceToDestination();
  return distance_destination <= kDestinationDeltaS;
}

double DynamicReferenceLine::SDistanceToDestination() const {
  double res = std::numeric_limits<double>::max();
  // TODO: move "DEST" to config
  const auto* dest_ptr = obstacle_decisions_.Find("DEST");
  if (!dest_ptr) {
    return res;
  }
  if (!dest_ptr->LongitudinalDecision().has_stop()) {
    return res;
  }
  if (!reference_line_.IsOnLane(dest_ptr->PerceptionBoundingBox().center())) {
    return res;
  }
  const double stop_s = dest_ptr->PerceptionSLBoundary().start_s() +
                        dest_ptr->LongitudinalDecision().stop().distance_s();
  return stop_s - adc_sl_boundary_.end_s();
}

}  // namespace planning_btree
}  // namespace apollo