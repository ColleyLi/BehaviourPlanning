#include "modules/planning_btree/behaviours/checks/collision_check.h"

namespace apollo {
namespace planning_btree {
BTreeNodeState CollisionCheck::Init(const BTreeNodeConfig& config)
{
  config_ = config;
  state_ = BTreeNodeState::NODE_INITIALIZED;
  return state_;
}

BTreeNodeState CollisionCheck::Execute(BTreeFrame* frame)
{
    state_ = BTreeNodeState::NODE_FAILED;
    return state_;
}

} // namespace planning_btree
} // namespace apollo