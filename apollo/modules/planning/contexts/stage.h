#pragma once

#include <memory>
#include "modules/planning/common/frame.h"
#include "modules/planning/behaviour_tree/b_tree_node_dispatcher.h"
#include "modules/planning/behaviour_tree/b_tree_node.h"
#include "modules/common/proto/pnc_point.pb.h"

namespace apollo {
namespace planning {
namespace context {

using behaviour_tree::BTreeNode;
using behaviour_tree::BTreeNodeDispatcher;
using common::TrajectoryPoint;

class Stage
{
    public:
        BTreeStageState Init(const BTreeStageConfig& config);
        BTreeStageState GetState();

        virtual BTreeStageState Execute(const TrajectoryPoint& planning_start_point, Frame* const frame) = 0;

    protected:
        BTreeNodeDispatcher node_dispatcher_;
        BTreeStageParameters parameters_;
        BTreeStageState state_ = BTreeStageState::STAGE_NOT_INITIALIZED;
        std::shared_ptr<BTreeNode> behaviour_tree_;
        std::unordered_map<std::string, std::shared_ptr<BTreeNode>> nodes_;
};

} // namespace context
} // namespace planning
} // namespace apollo