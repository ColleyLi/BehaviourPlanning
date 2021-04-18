#include "modules/planning/behaviour_tree/tasks/fallback_path_generator/fallback_path_generator.h" 

namespace apollo {
namespace planning {
namespace behaviour_tree {
  BTreeNodeState FallbackPathGenerator::Init(const BTreeNodeConfig& config)
  {
    config_ = config;
    state_ = BTreeNodeState::NODE_INITIALIZED;
    return state_;
  }

  BTreeNodeState FallbackPathGenerator::Execute(Frame* frame)
  {
    state_ = BTreeNodeState::NODE_DONE;
    return state_;
  }

  BTreeNodeState FallbackPathGenerator::Execute(Frame* frame, ReferenceLineInfo* reference_line_info)
  {
    state_ = BTreeNodeState::NODE_DONE;
    return state_;
  }

} // namespace behaviour_tree
} // namespace planning
} // namespace apollo