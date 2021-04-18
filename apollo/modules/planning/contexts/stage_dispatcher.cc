#include <memory>
#include "modules/planning/contexts/stage_dispatcher.h"
#include "modules/planning/contexts/lane_follow/stages/lane_follow_stage.h"

namespace apollo {
namespace planning {
namespace context {


bool StageDispatcher::Init()
{
    RegisterStages();

    return true;
}

void StageDispatcher::RegisterStages() 
{
    stage_factory_.Register(BTreeStageType::LANE_FOLLOW_STAGE, []() -> Stage* { return new LaneFollowStage();});
}

std::shared_ptr<Stage> StageDispatcher::Dispatch(const BTreeStageType& stage_type)
{
    return stage_factory_.CreateObject(stage_type);
}

} // namespace context
} // namespace planning
} // namespace apollo