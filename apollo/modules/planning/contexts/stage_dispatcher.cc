#include <memory>
#include "modules/planning/contexts/stage_dispatcher.h"
#include "modules/planning/contexts/lane_follow/stages/lane_follow_stage.h"

namespace apollo {
namespace planning {
namespace context {


common::Status StageDispatcher::Init()
{
    RegisterStages();

    return common::Status::OK();
}

void StageDispatcher::RegisterStages() 
{
    stage_factory_.Register(BTreeStageName::LANE_FOLLOW_STAGE, []() -> Stage* { return new LaneFollowStage();});
}

std::shared_ptr<Stage> StageDispatcher::Dispatch(const BTreeStageName& stage_name)
{
    return stage_factory_.CreateObject(stage_name);
}

} // namespace apollo
} // namespace planning
} // namespace context