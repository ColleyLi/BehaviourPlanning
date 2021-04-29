#pragma once

#include "modules/planning_btree/stages/stage.h"

namespace apollo {
namespace planning_btree {

class LaneFollowStage : public Stage
{
    public:
        LaneFollowStage() = default;

        BTreeStageState Init(const BTreeStageConfig& config);

        BTreeStageState Execute(const TrajectoryPoint& planning_start_point, BTreeFrame* const frame);
};

} // namespace planning_btree
} // namespace apollo