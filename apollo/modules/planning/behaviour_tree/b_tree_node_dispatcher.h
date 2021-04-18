#pragma once

#include <memory>
#include "modules/planning/behaviour_tree/b_tree_node.h"
#include "modules/common/util/factory.h"

namespace apollo {
namespace planning {
namespace behaviour_tree {

class BTreeNodeDispatcher
{
    public:
        BTreeNodeDispatcher() = default;
        ~BTreeNodeDispatcher() = default;

        bool Init();
  	    std::shared_ptr<BTreeNode> Dispatch(const BTreeNodeType& node_type);
    private:
        void RegisterNodes();

        common::util::Factory<BTreeNodeType, BTreeNode> node_factory_;
};

} // namespace behaviour_tree
} // namespace planning
} // namespace apollo