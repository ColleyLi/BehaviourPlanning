#pragma once

#include "modules/planning/contexts/context.h"

namespace apollo {
namespace planning {
namespace context {

class LaneFollowContext : public Context
{
    public:
        BTreeContextState Init(const BTreeContextConfig& config);

        BTreeContextState Execute(const TrajectoryPoint& planning_start_point, Frame* const frame);
};

} // namespace context
} // namespace planning
} // namespace apollo