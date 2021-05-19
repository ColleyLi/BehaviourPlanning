#include "modules/planning_btree/traffic_rules/rules/destination.h"

#include <memory>
#include <vector>

#include "modules/map/proto/map_lane.pb.h"

namespace apollo {
namespace planning_btree {

// TODO: move this to config
namespace
{
  const std::string kDestinationObstacleID = "DEST";
  constexpr double kVirtualStopWallLength = 0.1;
  // constexpr bool kEnableScenarioPullOver = false;
}

using apollo::common::VehicleConfigHelper;

Destination::Destination(const TrafficRuleConfig& config,
                         const std::shared_ptr<DependencyInjector>& injector)
    : TrafficRule(config, injector) {}

bool Destination::ApplyRule(BTreeFrame* frame,
                              DynamicReferenceLine* const dynamic_reference_line) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(dynamic_reference_line);

  MakeDecisions(frame, dynamic_reference_line);

  return true;
}

/**
 * @brief: build stop decision
 */
bool Destination::MakeDecisions(BTreeFrame* frame,
                               DynamicReferenceLine* const dynamic_reference_line) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(dynamic_reference_line);

  if (!frame->IsNearDestination()) {
    return true;
  }

  const auto& routing = frame->GetPlanningInput().routing;
  if (routing->routing_request().waypoint_size() < 2) {
    AERROR << "routing_request has no end";
    return false;
  }

  common::SLPoint dest_sl;
  const auto& reference_line = dynamic_reference_line->GetReferenceLine();
  const auto& routing_end = *(routing->routing_request().waypoint().rbegin());
  reference_line.XYToSL(routing_end.pose(), &dest_sl);
  const auto& adc_sl = dynamic_reference_line->GetADCSLBoundary();
  const auto& dest = injector_->planning_state()->destination_state();
  if (adc_sl.start_s() > dest_sl.s() && !dest.has_passed_destination()) {
    ADEBUG << "Destination at back, but we have not reached destination yet";
    return true;
  }

  const std::string stop_wall_id = kDestinationObstacleID;
  const std::vector<std::string> wait_for_obstacle_ids;

  // if (kEnableScenarioPullOver) {
  //   const auto& pull_over_bool =
  //       injector_->planning_context()->planning_status().pull_over();
  //   if (pull_over_bool.has_position() &&
  //       pull_over_bool.position().has_x() &&
  //       pull_over_bool.position().has_y()) {
  //     // build stop decision based on pull-over position
  //     ADEBUG << "BuildStopDecision: pull-over position";
  //     common::SLPoint pull_over_sl;
  //     reference_line.XYToSL(pull_over_bool.position(), &pull_over_sl);

  //     const double stop_line_s = pull_over_sl.s() +
  //                                VehicleConfigHelper::GetConfig()
  //                                    .vehicle_param()
  //                                    .front_edge_to_center() +
  //                                config_.destination().stop_distance();
  //     BuildStopDecision(
  //         stop_wall_id, stop_line_s, config_.destination().stop_distance(),
  //         StopReasonCode::STOP_REASON_PULL_OVER, wait_for_obstacle_ids,
  //         TrafficRuleConfig::RuleId_Name(config_.rule_id()), frame,
  //         dynamic_reference_line);
  //     return true;
  //   }
  // }

  // build stop decision
  ADEBUG << "BuildStopDecision: destination";
  const double dest_lane_s =
      std::fmax(0.0, routing_end.s() - kVirtualStopWallLength -
                         config_.destination().stop_distance());
  BuildStopDecision(stop_wall_id, routing_end.id(), dest_lane_s,
                          config_.destination().stop_distance(),
                          StopReasonCode::STOP_REASON_DESTINATION,
                          wait_for_obstacle_ids,
                          TrafficRuleConfig::RuleId_Name(config_.rule_id()),
                          frame, dynamic_reference_line);

  return true;
}

}  // namespace planning
}  // namespace apollo
