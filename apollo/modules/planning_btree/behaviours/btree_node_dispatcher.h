#pragma once

#include <memory>
#include "modules/planning_btree/behaviours/btree_node.h"
#include "modules/common/util/factory.h"

namespace apollo {
namespace planning_btree {

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

} // namespace planning_btree
} // namespace apollo