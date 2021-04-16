#pragma once

#include "modules/planning/contexts/context.h"

namespace apollo {
namespace planning {
namespace context {

class LaneFollowContext : public Context
{
    public:
        LaneFollowContext() = default;

        common::Status Init(const BTreeContextConfig& config) override;

        common::Status Execute(const TrajectoryPoint& planning_start_point, Frame* const frame) override;
};

} // namespace apollo
} // namespace planning
} // namespace context