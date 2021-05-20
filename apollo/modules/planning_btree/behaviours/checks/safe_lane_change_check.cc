#include "modules/planning_btree/behaviours/checks/safe_lane_change_check.h"
#include "modules/planning/tasks/deciders/lane_change_decider/lane_change_decider.h"

namespace apollo {
namespace planning_btree {

using common::SLPoint;

BTreeNodeState SafeLaneChangeCheck::Init(const BTreeNodeConfig& config) {
  config_ = config;
  state_ = BTreeNodeState::NODE_INITIALIZED;
  return state_;
}

BTreeNodeState SafeLaneChangeCheck::Execute(BTreeFrame* frame) {
  auto dynamic_reference_line = frame->GetMutableCurrentDynamicReferenceLine();
  if (!dynamic_reference_line->IsLaneChangePath() ||
      IsClearToChangeLane(dynamic_reference_line)) {
    if (dynamic_reference_line->IsLaneChangePath()) {
     ADEBUG << "Is clear to change line!";
    } else {
      ADEBUG << "This line does not want to change lane!";
    }
    state_ = BTreeNodeState::NODE_DONE;
    return state_;
  }
  ADEBUG << "Lane change is not safe!";
  state_ = BTreeNodeState::NODE_FAILED;
  return state_;
}

bool SafeLaneChangeCheck::IsClearToChangeLane(
    DynamicReferenceLine* dynamic_reference_line) {
  double ego_start_s = dynamic_reference_line->GetADCSLBoundary().start_s();
  double ego_end_s = dynamic_reference_line->GetADCSLBoundary().end_s();
  double ego_v =
      std::abs(dynamic_reference_line->GetADCState().linear_velocity());

  for (const auto* obstacle :
       dynamic_reference_line->GetMutableObstacleDecisions()
           ->obstacles()
           .Items()) {
    if (obstacle->IsVirtual() || obstacle->IsStatic()) {
      ADEBUG << "skip one virtual or static obstacle";
      continue;
    }

    double start_s = std::numeric_limits<double>::max();
    double end_s = -std::numeric_limits<double>::max();
    double start_l = std::numeric_limits<double>::max();
    double end_l = -std::numeric_limits<double>::max();

    for (const auto& p : obstacle->PerceptionPolygon().points()) {
      SLPoint sl_point;
      dynamic_reference_line->GetReferenceLine().XYToSL(p, &sl_point);
      start_s = std::fmin(start_s, sl_point.s());
      end_s = std::fmax(end_s, sl_point.s());

      start_l = std::fmin(start_l, sl_point.l());
      end_l = std::fmax(end_l, sl_point.l());
    }

    if (dynamic_reference_line->IsLaneChangePath()) {
      static constexpr double kLateralShift = 2.5;
      if (end_l < -kLateralShift || start_l > kLateralShift) {
        continue;
      }
    }

    // Raw estimation on whether same direction with ADC or not based on
    // prediction trajectory
    bool same_direction = true;
    if (obstacle->HasTrajectory()) {
      double obstacle_moving_direction =
          obstacle->Trajectory().trajectory_point(0).path_point().theta();
      const auto& vehicle_state = dynamic_reference_line->GetADCState();
      double vehicle_moving_direction = vehicle_state.heading();
      if (vehicle_state.gear() == canbus::Chassis::GEAR_REVERSE) {
        vehicle_moving_direction =
            common::math::NormalizeAngle(vehicle_moving_direction + M_PI);
      }
      double heading_difference = std::abs(common::math::NormalizeAngle(
          obstacle_moving_direction - vehicle_moving_direction));
      same_direction = heading_difference < (M_PI / 2.0);
    }

    // TODO(All) move to confs
    static constexpr double kSafeTimeOnSameDirection = 3.0;
    static constexpr double kSafeTimeOnOppositeDirection = 5.0;
    static constexpr double kForwardMinSafeDistanceOnSameDirection = 10.0;
    static constexpr double kBackwardMinSafeDistanceOnSameDirection = 10.0;
    static constexpr double kForwardMinSafeDistanceOnOppositeDirection = 50.0;
    static constexpr double kBackwardMinSafeDistanceOnOppositeDirection = 1.0;
    static constexpr double kDistanceBuffer = 0.5;

    double kForwardSafeDistance = 0.0;
    double kBackwardSafeDistance = 0.0;
    if (same_direction) {
      kForwardSafeDistance =
          std::fmax(kForwardMinSafeDistanceOnSameDirection,
                    (ego_v - obstacle->speed()) * kSafeTimeOnSameDirection);
      kBackwardSafeDistance =
          std::fmax(kBackwardMinSafeDistanceOnSameDirection,
                    (obstacle->speed() - ego_v) * kSafeTimeOnSameDirection);
    } else {
      kForwardSafeDistance =
          std::fmax(kForwardMinSafeDistanceOnOppositeDirection,
                    (ego_v + obstacle->speed()) * kSafeTimeOnOppositeDirection);
      kBackwardSafeDistance = kBackwardMinSafeDistanceOnOppositeDirection;
    }

    if (HysteresisFilter(ego_start_s - end_s, kBackwardSafeDistance,
                         kDistanceBuffer, obstacle->IsLaneChangeBlocking()) &&
        HysteresisFilter(start_s - ego_end_s, kForwardSafeDistance,
                         kDistanceBuffer, obstacle->IsLaneChangeBlocking())) {
      dynamic_reference_line->GetMutableObstacleDecisions()
          ->Find(obstacle->Id())
          ->SetLaneChangeBlocking(true);
      ADEBUG << "Lane Change is blocked by obstacle" << obstacle->Id();
      return false;
    } else {
      dynamic_reference_line->GetMutableObstacleDecisions()
          ->Find(obstacle->Id())
          ->SetLaneChangeBlocking(false);
    }
  }
  return true;
}

bool SafeLaneChangeCheck::HysteresisFilter(const double obstacle_distance,
                                           const double safe_distance,
                                           const double distance_buffer,
                                           const bool is_obstacle_blocking) {
  if (is_obstacle_blocking) {
    return obstacle_distance < safe_distance + distance_buffer;
  } else {
    return obstacle_distance < safe_distance - distance_buffer;
  }
}

}  // namespace planning_btree
}  // namespace apollo