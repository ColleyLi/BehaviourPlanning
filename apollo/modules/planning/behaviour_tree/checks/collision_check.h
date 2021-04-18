#pragma once

#include "modules/planning/behaviour_tree/b_tree_check.h"

namespace apollo {
namespace planning {
namespace behaviour_tree {

class CollisionCheck: public BTreeCheck
{
    BTreeNodeState Init(const BTreeNodeConfig& config);
    BTreeNodeState Execute(Frame* frame);
    BTreeNodeState Execute(Frame* frame, ReferenceLineInfo* reference_line_info);
};

} // namespace behaviour_tree
} // namespace planning
} // namespace apollo