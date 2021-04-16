#pragma once

#include "modules/planning/contexts/stage.h"
#include "modules/planning/proto/b_tree_task_config.pb.h"

namespace apollo {
namespace planning {
namespace context {

class LaneFollowStage : public Stage
{
    public:
        LaneFollowStage() = default;

        common::Status Init(const BTreeStageConfig& config) override;

        common::Status Execute(const TrajectoryPoint& planning_start_point, Frame* const frame) override;
};

} // namespace apollo
} // namespace planning
} // namespace context