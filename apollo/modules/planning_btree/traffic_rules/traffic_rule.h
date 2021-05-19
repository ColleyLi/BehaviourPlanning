#pragma once

#include <memory>

#include "modules/planning_btree/common/dependency_injector.h"
#include "modules/planning_btree/common/btree_frame.h"
#include "modules/planning_btree/common/dynamic_reference_line.h"
#include "modules/planning_btree/common/obstacle.h"
#include "modules/planning_btree/common/obstacle_decisions.h"
#include "modules/planning_btree/proto/traffic_rule_config.pb.h"

namespace apollo {
namespace planning_btree {

class TrafficRule {
 public:
  explicit TrafficRule(const TrafficRuleConfig& config) : config_(config) {}
  TrafficRule(const TrafficRuleConfig& config, const std::shared_ptr<DependencyInjector>& injector)
      : config_(config),
        injector_(injector) {}
  virtual ~TrafficRule() = default;
  virtual bool ApplyRule(
      BTreeFrame* const frame, DynamicReferenceLine* const dynamic_reference_line) = 0;
  
  protected:
  bool BuildStopDecision(const std::string& stop_wall_id, const double stop_line_s,
                      const double stop_distance,
                      const StopReasonCode& stop_reason_code,
                      const std::vector<std::string>& wait_for_obstacles,
                      const std::string& decision_tag, BTreeFrame* const frame,
                      DynamicReferenceLine* const dynamic_reference_line);

  bool BuildStopDecision(const std::string& stop_wall_id,
                      const std::string& lane_id, const double lane_s,
                      const double stop_distance,
                      const StopReasonCode& stop_reason_code,
                      const std::vector<std::string>& wait_for_obstacles,
                      const std::string& decision_tag, BTreeFrame* const frame,
                      DynamicReferenceLine* const dynamic_reference_line);

 protected:
  TrafficRuleConfig config_;
  std::shared_ptr<DependencyInjector> injector_;
};

}  // namespace planning_btree
}  // namespace apollo
