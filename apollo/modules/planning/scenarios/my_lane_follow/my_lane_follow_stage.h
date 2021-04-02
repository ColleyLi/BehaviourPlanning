#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning.pb.h"

#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/speed_profile_generator.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_point.h"
#include "modules/planning/scenarios/scenario.h"
#include "modules/planning/scenarios/stage.h"
#include "modules/planning/behaviour_tree/bt_node.h"

namespace apollo 
{
namespace planning 
{
namespace scenario 
{
namespace lane_follow 
{

class MyLaneFollowStage : public Stage 
{
  public:
    explicit MyLaneFollowStage(const ScenarioConfig::StageConfig& config);

    StageStatus Process(const common::TrajectoryPoint& planning_init_point,
                        Frame* frame) override;

 private:
    ScenarioConfig config_;
    std::unique_ptr<Stage> stage_;
    std::unique_ptr<BTNode> behaviour_tree_;
};

}  // namespace lane_follow
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
