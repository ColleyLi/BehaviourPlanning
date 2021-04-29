#pragma once

#include "modules/planning_btree/behaviours/btree_task.h"

namespace apollo {
namespace planning_btree {

class FallbackSpeedGenerator: public BTreeTask
{
    BTreeNodeState Init(const BTreeNodeConfig& config);
    BTreeNodeState Execute(BTreeFrame* frame);
    // BTreeNodeState Execute(BTreeFrame* frame, ReferenceLineInfo* reference_line_info);
};

} // namespace planning_btree
} // namespace apollo