#pragma once

#include "modules/planning/behaviour_tree/b_tree_task.h"
#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/proto/b_tree_node_config.pb.h"

namespace apollo {
namespace planning {
namespace behaviour_tree {

class SpeedGenerator: public BTreeTask
{
  public:
    BTreeNodeState Init(const BTreeNodeConfig& config);
    BTreeNodeState Execute(Frame* frame);
    BTreeNodeState Execute(Frame* frame, ReferenceLineInfo* reference_line_info);

  private:
    BTreeNodeState ConstantSpeed(Frame* frame, ReferenceLineInfo* reference_line_info);
    BTreeNodeState GenerateBoundaries(Frame* frame, ReferenceLineInfo* reference_line_info);
    double SetSpeedFallbackDistance(PathDecision *const path_decision);
    BTreeNodeState OptimizeSTGraph(Frame* frame, ReferenceLineInfo* reference_line_info);

  private:
    std::shared_ptr<DependencyInjector> injector_;
    SpeedBoundsDeciderConfig speed_bounds_config_;
    DpStSpeedOptimizerConfig dp_st_speed_optimizer_config_;
};

} // namespace behaviour_tree
} // namespace planning
} // namespace apollo