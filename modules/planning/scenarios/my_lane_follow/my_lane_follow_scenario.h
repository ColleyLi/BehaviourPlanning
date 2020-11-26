#pragma once

#include <memory>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning.pb.h"

#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/speed_profile_generator.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_point.h"
#include "modules/planning/scenarios/scenario.h"
#include "modules/planning/scenarios/stage.h"
#include "modules/planning/tasks/task.h"

namespace apollo 
{
namespace planning 
{
namespace scenario 
{
namespace lane_follow 
{

class MyLaneFollowScenario : public Scenario 
{
	public:
  	MyLaneFollowScenario(const ScenarioConfig& config,
      const ScenarioContext* context)
      : Scenario(config, context) 
      {
      	
      }

  std::unique_ptr<Stage> CreateStage(
      const ScenarioConfig::StageConfig& stage_config) override;
};

}  // namespace lane_follow
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
