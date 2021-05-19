#pragma once

#include "modules/planning_btree/behaviours/btree_check.h"

namespace apollo {
namespace planning_btree {

class CollisionCheck: public BTreeCheck
{
    BTreeNodeState Init(const BTreeNodeConfig& config);
    BTreeNodeState Execute(BTreeFrame* frame);
};

} // namespace planning_btree
} // namespace apollo