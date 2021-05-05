#include "modules/planning_btree/contexts/lane_follow/lane_follow_context.h"

namespace apollo {
namespace planning_btree {

LaneFollowContext::LaneFollowContext(const std::shared_ptr<DependencyInjector>& injector)
    :Context(injector)
{

}

BTreeContextState LaneFollowContext::Init(const BTreeContextConfig& config)
{
    state_ = Context::Init(config);

    return state_;
}

BTreeContextState LaneFollowContext::Execute(const TrajectoryPoint& planning_start_point, BTreeFrame* const frame)
{
    AERROR << "Executed LaneFollow context";

    auto current_stage = stage_selector_->GetCurrentStage(planning_start_point, frame);

    current_stage->Execute(planning_start_point, frame);

    state_ = BTreeContextState::CONTEXT_DONE;

    return state_;
}

} // namespace planning_btree
} // namespace apollo