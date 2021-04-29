#pragma once

#include "modules/planning_btree/common/btree_frame.h"
#include "modules/planning_btree/stages/stage_selector.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning_btree/proto/btree_context_config.pb.h"

namespace apollo {
namespace planning_btree {

using common::TrajectoryPoint;

class Context
{
    public:
        BTreeContextState Init(const BTreeContextConfig& config);

        virtual BTreeContextState Execute(const TrajectoryPoint& planning_start_point, BTreeFrame* const frame) = 0;

    protected:
        StageSelector stage_selector_;
        BTreeContextParameters parameters_;
        BTreeContextState state_ = BTreeContextState::CONTEXT_NOT_INITIALIZED;
};

} // namespace planning_btree
} // namespace apollo