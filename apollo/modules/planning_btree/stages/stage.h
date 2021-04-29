#pragma once

#include <memory>
#include "modules/planning_btree/common/btree_frame.h"
#include "modules/planning_btree/behaviours/btree_node_dispatcher.h"
#include "modules/planning_btree/behaviours/btree_node.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning_btree/proto/btree_stage_config.pb.h"

namespace apollo {
namespace planning_btree {

using common::TrajectoryPoint;

class Stage
{
    public:
        BTreeStageState Init(const BTreeStageConfig& config);
        BTreeStageState GetState();

        virtual BTreeStageState Execute(const TrajectoryPoint& planning_btree_start_point, BTreeFrame* const frame) = 0;

    protected:
        BTreeNodeDispatcher node_dispatcher_;
        BTreeStageParameters parameters_;
        BTreeStageState state_ = BTreeStageState::STAGE_NOT_INITIALIZED;
        std::shared_ptr<BTreeNode> behaviour_tree_;
        std::unordered_map<std::string, std::shared_ptr<BTreeNode>> nodes_;
};

} // namespace planning_btree
} // namespace apollo