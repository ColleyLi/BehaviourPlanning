#include "cyber/common/log.h"

#include "modules/planning/scenarios/my_lane_follow/my_lane_follow_scenario.h"
#include "modules/planning/scenarios/my_lane_follow/my_lane_follow_stage.h"

namespace apollo 
{
namespace planning 
{
namespace scenario 
{
namespace lane_follow 
{

std::unique_ptr<Stage> MyLaneFollowScenario::CreateStage(
    const ScenarioConfig::StageConfig& stage_config) 
{
  if (stage_config.stage_type() != ScenarioConfig::MY_LANE_FOLLOW_DEFAULT_STAGE) 
  {
    AERROR << "Follow lane does not support stage type: "
           << ScenarioConfig::StageType_Name(stage_config.stage_type());
    return nullptr;
  }
  return std::unique_ptr<Stage>(new MyLaneFollowStage(stage_config));
}

}  // namespace lane_follow
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
