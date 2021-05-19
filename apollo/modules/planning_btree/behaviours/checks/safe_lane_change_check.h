#pragma once

#include "modules/planning_btree/behaviours/btree_check.h"

namespace apollo {
namespace planning_btree {

class SafeLaneChangeCheck : public BTreeCheck {
 public:
  BTreeNodeState Init(const BTreeNodeConfig& config);
  BTreeNodeState Execute(BTreeFrame* frame);

 private:
  bool IsClearToChangeLane(DynamicReferenceLine* dynamic_reference_line);

  bool HysteresisFilter(const double obstacle_distance,
                        const double safe_distance,
                        const double distance_buffer,
                        const bool is_obstacle_blocking);
};

}  // namespace planning_btree
}  // namespace apollo