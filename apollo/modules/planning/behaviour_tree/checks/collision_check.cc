#include "modules/planning/behaviour_tree/checks/collision_check.h"

namespace apollo {
namespace planning {
namespace behaviour_tree {
BTreeNodeState CollisionCheck::Init(const BTreeNodeConfig& config)
{
  config_ = config;
  state_ = BTreeNodeState::NODE_INITIALIZED;
  return state_;
}

BTreeNodeState CollisionCheck::Execute(Frame* frame, ReferenceLineInfo* reference_line_info)
{
    return Execute(frame);
}

BTreeNodeState CollisionCheck::Execute(Frame* frame)
{
    state_ = BTreeNodeState::NODE_FAILED;
    return state_;
}

} // namespace behaviour_tree
} // namespace planning
} // namespace apollo