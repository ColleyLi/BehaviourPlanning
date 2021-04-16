#pragma once

#include "modules/common/status/status.h"
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
        virtual common::Status Init(const BTreeContextConfig& config) = 0;

        virtual common::Status Execute(const TrajectoryPoint& planning_start_point, Frame* const frame) = 0;

    protected:
        StageSelector stage_selector_;
        BTreeContextParameters parameters_;
};
} // namespace apollo
} // namespace planning
} // namespace context