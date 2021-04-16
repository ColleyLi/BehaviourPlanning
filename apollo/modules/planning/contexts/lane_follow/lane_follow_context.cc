#include "modules/planning/contexts/lane_follow/lane_follow_context.h"

namespace apollo {
namespace planning {
namespace context {

common::Status LaneFollowContext::Init(const BTreeContextConfig& config)
{
    stage_selector_.Init(config.stage_fsm());
    parameters_ = config.parameters();
    return common::Status::OK();
}

common::Status LaneFollowContext::Execute(const TrajectoryPoint& planning_start_point, Frame* const frame)
{
    AERROR << "Executed LaneFollow context";

    auto current_stage = stage_selector_.GetCurrentStage(planning_start_point, frame);

    current_stage->Execute(planning_start_point, frame);

    return common::Status::OK();
}

} // namespace apollo
} // namespace planning
} // namespace context