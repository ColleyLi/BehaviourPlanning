#include <memory>
#include "modules/planning_btree/stages/stage_dispatcher.h"
#include "modules/planning_btree/stages/lane_follow/lane_follow_stage.h"

namespace apollo {
namespace planning_btree {


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

} // namespace planning_btree
} // namespace apollo