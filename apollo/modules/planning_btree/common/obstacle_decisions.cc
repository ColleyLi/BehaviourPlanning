#include "modules/planning_btree/common/obstacle_decisions.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/util.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

namespace apollo {
namespace planning_btree {

Obstacle *ObstacleDecisions::AddObstacle(const Obstacle &obstacle) {
  return obstacles_.Add(obstacle.Id(), obstacle);
}

const IndexedObstacles &ObstacleDecisions::obstacles() const { return obstacles_; }

Obstacle *ObstacleDecisions::Find(const std::string &object_id) {
  return obstacles_.Find(object_id);
}

const Obstacle *ObstacleDecisions::Find(const std::string &object_id) const {
  return obstacles_.Find(object_id);
}

const perception::PerceptionObstacle *ObstacleDecisions::FindPerceptionObstacle(
    const std::string &perception_obstacle_id) const {
  for (const auto *obstacle : obstacles_.Items()) {
    if (std::to_string(obstacle->Perception().id()) == perception_obstacle_id) {
      return &(obstacle->Perception());
    }
  }

  return nullptr;
}

void ObstacleDecisions::SetSTBoundary(const std::string &id,
                                 const STBoundary &boundary) {
  auto *obstacle = obstacles_.Find(id);

  if (!obstacle) {
    AERROR << "Failed to find obstacle : " << id;
    return;
  } else {
    obstacle->set_path_st_boundary(boundary);
  }
}

bool ObstacleDecisions::AddLateralDecision(const std::string &tag,
                                      const std::string &object_id,
                                      const ObjectDecisionType &decision) {
  auto *obstacle = obstacles_.Find(object_id);
  if (!obstacle) {
    AERROR << "failed to find obstacle";
    return false;
  }
  obstacle->AddLateralDecision(tag, decision);
  return true;
}

void ObstacleDecisions::EraseStBoundaries() {
  for (const auto *obstacle : obstacles_.Items()) {
    auto *obstacle_ptr = obstacles_.Find(obstacle->Id());
    obstacle_ptr->EraseStBoundary();
  }
}

bool ObstacleDecisions::AddLongitudinalDecision(const std::string &tag,
                                           const std::string &object_id,
                                           const ObjectDecisionType &decision) {
  auto *obstacle = obstacles_.Find(object_id);
  if (!obstacle) {
    AERROR << "failed to find obstacle";
    return false;
  }
  obstacle->AddLongitudinalDecision(tag, decision);
  return true;
}

bool ObstacleDecisions::MergeWithMainStop(const ObjectStop &obj_stop,
                                     const std::string &obj_id,
                                     const ReferenceLine &reference_line,
                                     const SLBoundary &adc_sl_boundary) {
  common::PointENU stop_point = obj_stop.stop_point();
  common::SLPoint stop_line_sl;
  reference_line.XYToSL(stop_point, &stop_line_sl);

  double stop_line_s = stop_line_sl.s();
  if (stop_line_s < 0.0 || stop_line_s > reference_line.Length()) {
    AERROR << "Ignore object:" << obj_id << " fence route_s[" << stop_line_s
           << "] not in range[0, " << reference_line.Length() << "]";
    return false;
  }

  // check stop_line_s vs adc_s, ignore if it is further way than main stop
  const auto &vehicle_config = common::VehicleConfigHelper::GetConfig();
  stop_line_s = std::fmax(
      stop_line_s, adc_sl_boundary.end_s() -
                       vehicle_config.vehicle_param().front_edge_to_center());

  if (stop_line_s >= stop_reference_line_s_) {
    ADEBUG << "stop point is farther than current main stop point.";
    return false;
  }

  main_stop_.Clear();
  main_stop_.set_reason_code(obj_stop.reason_code());
  main_stop_.set_reason("stop by " + obj_id);
  main_stop_.mutable_stop_point()->set_x(obj_stop.stop_point().x());
  main_stop_.mutable_stop_point()->set_y(obj_stop.stop_point().y());
  main_stop_.set_stop_heading(obj_stop.stop_heading());
  stop_reference_line_s_ = stop_line_s;

  ADEBUG << " main stop obstacle id:" << obj_id
         << " stop_line_s:" << stop_line_s << " stop_point: ("
         << obj_stop.stop_point().x() << obj_stop.stop_point().y()
         << " ) stop_heading: " << obj_stop.stop_heading();
  return true;
}

}  // namespace planning_btree
}  // namespace apollo
