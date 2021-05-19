#include "modules/planning_btree/traffic_rules/traffic_rule.h"

#include <memory>
#include <vector>

#include "modules/map/proto/map_lane.pb.h"

namespace apollo {
namespace planning_btree {

// TODO: move this to config
namespace
{

}

using apollo::common::util::WithinBound;

/*
 * @brief: build virtual obstacle of stop wall, and add STOP decision
 */
bool TrafficRule::BuildStopDecision(const std::string& stop_wall_id, const double stop_line_s,
                      const double stop_distance,
                      const StopReasonCode& stop_reason_code,
                      const std::vector<std::string>& wait_for_obstacles,
                      const std::string& decision_tag, BTreeFrame* const frame,
                      DynamicReferenceLine* const dynamic_reference_line) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(dynamic_reference_line);

  // check
  const auto& reference_line = dynamic_reference_line->GetReferenceLine();
  if (!WithinBound(0.0, reference_line.Length(), stop_line_s)) {
    AERROR << "stop_line_s[" << stop_line_s << "] is not on reference line";
    return true;
  }

  // create virtual stop wall
  const auto* obstacle =
      frame->CreateStopObstacle(dynamic_reference_line, stop_wall_id, stop_line_s);
  if (!obstacle) {
    AERROR << "Failed to create obstacle [" << stop_wall_id << "]";
    return false;
  }
  const Obstacle* stop_wall = dynamic_reference_line->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to add obstacle[" << stop_wall_id << "]";
    return false;
  }

  // build stop decision
  const double stop_s = stop_line_s - stop_distance;
  const auto& stop_point = reference_line.GetReferencePoint(stop_s);
  const double stop_heading =
      reference_line.GetReferencePoint(stop_s).heading();

  ObjectDecisionType stop;
  auto* stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(stop_reason_code);
  stop_decision->set_distance_s(-stop_distance);
  stop_decision->set_stop_heading(stop_heading);
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  for (size_t i = 0; i < wait_for_obstacles.size(); ++i) {
    stop_decision->add_wait_for_obstacle(wait_for_obstacles[i]);
  }

  auto* obstacle_decisions = dynamic_reference_line->GetMutableObstacleDecisions();
  obstacle_decisions->AddLongitudinalDecision(decision_tag, stop_wall->Id(), stop);

  return true;
}

bool TrafficRule::BuildStopDecision(const std::string& stop_wall_id,
                      const std::string& lane_id, const double lane_s,
                      const double stop_distance,
                      const StopReasonCode& stop_reason_code,
                      const std::vector<std::string>& wait_for_obstacles,
                      const std::string& decision_tag, BTreeFrame* const frame,
                      DynamicReferenceLine* const dynamic_reference_line) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(dynamic_reference_line);

  const auto& reference_line = dynamic_reference_line->GetReferenceLine();

  // create virtual stop wall
  const auto* obstacle =
      frame->CreateStopObstacle(stop_wall_id, lane_id, lane_s);
  if (!obstacle) {
    AERROR << "Failed to create obstacle [" << stop_wall_id << "]";
    return false;
  }

  const Obstacle* stop_wall = dynamic_reference_line->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to create obstacle for: " << stop_wall_id;
    return false;
  }

  const auto& stop_wall_box = stop_wall->PerceptionBoundingBox();
  if (!reference_line.IsOnLane(stop_wall_box.center())) {
    ADEBUG << "stop point is not on lane. SKIP STOP decision";
    return true;
  }

  // build stop decision
  auto stop_point = reference_line.GetReferencePoint(
      stop_wall->PerceptionSLBoundary().start_s() - stop_distance);

  ObjectDecisionType stop;
  auto* stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(stop_reason_code);
  stop_decision->set_distance_s(-stop_distance);
  stop_decision->set_stop_heading(stop_point.heading());
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  auto* obstacle_decisions = dynamic_reference_line->GetMutableObstacleDecisions();
  obstacle_decisions->AddLongitudinalDecision(decision_tag, stop_wall->Id(), stop);

  return true;
}

}  // namespace planning
}  // namespace apollo
