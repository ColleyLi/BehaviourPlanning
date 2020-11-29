#include "modules/planning/behaviour_tree/checks/safe_lane_change_check.h"
#include "modules/planning/tasks/deciders/lane_change_decider/lane_change_decider.h"

namespace apollo
{
namespace planning
{

using apollo::common::Status;
using apollo::common::ErrorCode;

Status SafeLaneChangeCheck::Process(Frame* frame, ReferenceLineInfo* reference_line_info)
{
    if (!reference_line_info->IsChangeLanePath() ||
        LaneChangeDecider::IsClearToChangeLane(reference_line_info))
    {
        return Status::OK();
    }

    return Status(ErrorCode::PLANNING_ERROR, "Lane change is not safe");
}

Status SafeLaneChangeCheck::Process(Frame* frame)
{
    return Status(ErrorCode::PLANNING_ERROR, "Lane change is not safe");
}

}
}