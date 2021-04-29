#include "modules/planning_btree/behaviours/tasks/fallback_speed_generator/fallback_speed_generator.h" 

namespace apollo {
namespace planning_btree {

  BTreeNodeState FallbackSpeedGenerator::Init(const BTreeNodeConfig& config)
  {
    config_ = config;
    state_ = BTreeNodeState::NODE_INITIALIZED;
    return state_;
  }

  BTreeNodeState FallbackSpeedGenerator::Execute(BTreeFrame* frame)
  {
    state_ = BTreeNodeState::NODE_DONE;
    return state_;
  }

  // BTreeNodeState FallbackSpeedGenerator::Execute(BTreeFrame* frame, ReferenceLineInfo* reference_line_info)
  // {
  //   state_ = BTreeNodeState::NODE_DONE;
  //   return state_;
  // }

} // namespace planning_btree
} // namespace apollo