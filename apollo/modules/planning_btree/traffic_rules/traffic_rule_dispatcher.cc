#include "modules/planning_btree/traffic_rules/traffic_rule_dispatcher.h"

#include <limits>
#include <memory>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning_btree/traffic_rules/rules/destination.h"

namespace apollo {
namespace planning_btree {

apollo::common::util::Factory<
    TrafficRuleConfig::RuleId, TrafficRule,
    TrafficRule *(*)(const TrafficRuleConfig &config,
                     const std::shared_ptr<DependencyInjector> &injector)>
    TrafficRuleDispatcher::s_rule_factory;

void TrafficRuleDispatcher::RegisterRules() {
  s_rule_factory.Register(
      TrafficRuleConfig::DESTINATION,
      [](const TrafficRuleConfig &config,
         const std::shared_ptr<DependencyInjector> &injector) -> TrafficRule * {
        return new Destination(config, injector);
      });
}

bool TrafficRuleDispatcher::Init(const TrafficRuleConfigs &config) {
  if (s_rule_factory.Empty()) {
    RegisterRules();
  }
  rule_configs_ = config;
  return true;
}

bool TrafficRuleDispatcher::Execute(
    BTreeFrame *frame, DynamicReferenceLine *dynamic_reference_line,
    const std::shared_ptr<DependencyInjector> &injector) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(dynamic_reference_line);

  for (const auto &rule_config : rule_configs_.config()) {
    if (!rule_config.enabled()) {
      ADEBUG << "Rule " << rule_config.rule_id() << " not enabled";
      continue;
    }
    auto rule = s_rule_factory.CreateObject(rule_config.rule_id(), rule_config,
                                            injector);
    if (!rule) {
      AERROR << "Could not find rule " << rule_config.DebugString();
      continue;
    }
    rule->ApplyRule(frame, dynamic_reference_line);
    ADEBUG << "Applied rule "
           << TrafficRuleConfig::RuleId_Name(rule_config.rule_id());
  }

  return true;
}

}  // namespace planning_btree
}  // namespace apollo
