#pragma once

#include "modules/planning_btree/behaviours/btree_task.h"

namespace apollo {
namespace planning_btree{

class ObstacleProcessor: public BTreeTask
{
    BTreeNodeState Init(const BTreeNodeConfig& config);
    BTreeNodeState Execute(BTreeFrame* frame);
};

} // namespace planning_btree
} // namespace apollo