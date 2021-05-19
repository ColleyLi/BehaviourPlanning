#pragma once

#include <memory>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/planning_btree/proto/traffic_rule_config.pb.h"
#include "modules/planning_btree/reference_line/reference_line.h"
#include "modules/planning_btree/traffic_rules/traffic_rule.h"

namespace apollo {
namespace planning_btree {

class TrafficRuleDispatcher {
 public:
  TrafficRuleDispatcher() = default;
  bool Init(const TrafficRuleConfigs &config);
  virtual ~TrafficRuleDispatcher() = default;
  bool Execute(
      BTreeFrame *frame, DynamicReferenceLine *dynamic_reference_line,
      const std::shared_ptr<DependencyInjector> &injector);

 private:
  static apollo::common::util::Factory<
      TrafficRuleConfig::RuleId, TrafficRule,
      TrafficRule *(*)(const TrafficRuleConfig &config,
                       const std::shared_ptr<DependencyInjector> &injector)>
      s_rule_factory;

  void RegisterRules();
  void BuildPlanningTarget(DynamicReferenceLine *dynamic_reference_line);

  TrafficRuleConfigs rule_configs_;
};

}  // namespace planning_btree
}  // namespace apollo
