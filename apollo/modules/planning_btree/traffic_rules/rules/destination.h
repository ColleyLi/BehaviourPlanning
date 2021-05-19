#pragma once

#include <memory>
#include <string>

#include "modules/common/proto/geometry.pb.h"
#include "modules/planning_btree/traffic_rules/traffic_rule.h"

namespace apollo {
namespace planning_btree {


class Destination : public TrafficRule {
 public:
  Destination(const TrafficRuleConfig& config,
              const std::shared_ptr<DependencyInjector>& injector);
  virtual ~Destination() = default;

  bool ApplyRule(BTreeFrame* const frame,
                           DynamicReferenceLine* const dynamic_reference_line);

 private:
  bool MakeDecisions(BTreeFrame* const frame,
                    DynamicReferenceLine* const dynamic_reference_line);
};

}  // namespace planning_btree
}  // namespace apollo
