#pragma once

#include "modules/planning/contexts/stage.h"

namespace apollo {
namespace planning {
namespace context {

class LaneFollowStage : public Stage
{
    public:
        LaneFollowStage() = default;

        BTreeStageState Init(const BTreeStageConfig& config);

        BTreeStageState Execute(const TrajectoryPoint& planning_start_point, Frame* const frame);
};

} // namespace context
} // namespace planning
} // namespace apollo