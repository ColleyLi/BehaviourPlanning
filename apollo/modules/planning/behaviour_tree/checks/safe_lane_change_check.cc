#include "modules/planning/behaviour_tree/checks/safe_lane_change_check.h"
#include "modules/planning/tasks/deciders/lane_change_decider/lane_change_decider.h"

namespace apollo {
namespace planning {
namespace behaviour_tree { 
BTreeNodeState SafeLaneChangeCheck::Init(const BTreeNodeConfig& config)
{
  config_ = config;
  state_ = BTreeNodeState::NODE_INITIALIZED;
  return state_;
}

BTreeNodeState SafeLaneChangeCheck::Execute(Frame* frame, ReferenceLineInfo* reference_line_info)
{
    ADEBUG << "Checking if lane change is possible for: " << reference_line_info->Lanes().Id();
    if (!reference_line_info->IsChangeLanePath() ||
        LaneChangeDecider::IsClearToChangeLane(reference_line_info))
    {
        if (reference_line_info->IsChangeLanePath())
        {
          ADEBUG << "Is clear to change line!";
        }
        else
        {
          ADEBUG << "This line does not want to change lane!";
        }
        state_ = BTreeNodeState::NODE_DONE;
        return state_;
    }

    ADEBUG << "Lane change is not safe!"; 
    state_ = BTreeNodeState::NODE_FAILED;
    return state_;
}

BTreeNodeState SafeLaneChangeCheck::Execute(Frame* frame)
{
  state_ = BTreeNodeState::NODE_FAILED;
  return state_;
}

} // namespace behaviour_tree
} // namespace planning
} // namespace apollo