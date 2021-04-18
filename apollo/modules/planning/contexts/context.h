#pragma once

#include "modules/planning/common/frame.h"
#include "modules/planning/contexts/stage_selector.h"
#include "modules/common/proto/pnc_point.pb.h"

namespace apollo {
namespace planning {
namespace context {

using common::TrajectoryPoint;

class Context
{
    public:
        BTreeContextState Init(const BTreeContextConfig& config);

        virtual BTreeContextState Execute(const TrajectoryPoint& planning_start_point, Frame* const frame) = 0;

    protected:
        StageSelector stage_selector_;
        BTreeContextParameters parameters_;
        BTreeContextState state_ = BTreeContextState::CONTEXT_NOT_INITIALIZED;
};

} // namespace context
} // namespace planning
} // namespace apollo