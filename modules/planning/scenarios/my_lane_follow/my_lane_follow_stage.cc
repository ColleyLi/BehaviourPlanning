#include "modules/planning/scenarios/my_lane_follow/my_lane_follow_stage.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/common/util/string_util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/behaviour_tree/bt_sequence.h"
#include "modules/planning/behaviour_tree/bt_selector.h"
#include "modules/planning/behaviour_tree/tasks/path_generator/path_generator.h"
#include "modules/planning/behaviour_tree/tasks/speed_generator/speed_generator.h"
#include "modules/planning/behaviour_tree/tasks/fallback_speed_generator/fallback_speed_generator.h"
#include "modules/planning/behaviour_tree/tasks/fallback_path_generator/fallback_path_generator.h"
#include "modules/planning/behaviour_tree/tasks/obstacle_processor/obstacle_processor.h"
#include "modules/planning/behaviour_tree/tasks/lane_prioritizer/lane_prioritizer.h"
#include "modules/planning/behaviour_tree/checks/safe_lane_change_check.h"
#include "modules/planning/behaviour_tree/checks/collision_check.h"
#include "modules/planning/behaviour_tree/priority_selectors/lane_priority_selector.h"

namespace apollo 
{
namespace planning 
{
namespace scenario 
{
namespace lane_follow 
{

using common::ErrorCode;
using common::SLPoint;
using common::Status;
using common::TrajectoryPoint;
using common::time::Clock;

MyLaneFollowStage::MyLaneFollowStage(const ScenarioConfig::StageConfig& config)
    : Stage(config) 
    {

      PlanningConfig planning_config;
      apollo::cyber::common::GetProtoFromFile(FLAGS_behaviour_tree_config_file, &planning_config);

      BTSelector* action_selector = new BTSelector();
      action_selector->SetName("ActionSelector");

      BTSequence* lane_follow_sequence = new BTSequence();
      lane_follow_sequence->SetName("LaneFollowSequence");
      BTSequence* emergency_sequence = new BTSequence();
      emergency_sequence->SetName("EmergencySequence");
      BTSequence* maneuver_sequence = new BTSequence();
      maneuver_sequence->SetName("ManeuverSequence");
      BTSequence* lane_process_sequence = new BTSequence();
      lane_process_sequence->SetName("LaneProcessSequence");

      CollisionCheck* collision_check = new CollisionCheck();
      collision_check->SetName("CollisionCheck");
      SafeLaneChangeCheck* safe_lane_change_check = new SafeLaneChangeCheck();
      safe_lane_change_check->SetName("SafeLaneChangeCheck");

      LanePrioritySelector* lane_priority_selector = new LanePrioritySelector();
      lane_priority_selector->SetName("LinePrioritySelector");

      PathGenerator* path_generator_node = new PathGenerator();
      path_generator_node->SetName("PathGenerator");
      SpeedGenerator* speed_generator_node = new SpeedGenerator();
      speed_generator_node->SetName("SpeedGenerator");
      speed_generator_node->Init(planning_config);
      FallbackPathGenerator* fallback_path_generator_node = new FallbackPathGenerator();
      fallback_path_generator_node->SetName("FallbackPathGenerator");
      FallbackSpeedGenerator* fallback_speed_generator_node = new FallbackSpeedGenerator();
      fallback_speed_generator_node->SetName("FallbackSpeedGenerator");
      ObstacleProcessor* obstacle_processor = new ObstacleProcessor();
      obstacle_processor->SetName("ObstacleProcessor");
      LanePrioritizer* lane_prioritizer = new LanePrioritizer();
      lane_prioritizer->SetName("LanePrioritizer");

      emergency_sequence->AddChild(collision_check);
      emergency_sequence->AddChild(fallback_path_generator_node);
      emergency_sequence->AddChild(fallback_speed_generator_node);

      lane_process_sequence->AddChild(safe_lane_change_check);
      lane_process_sequence->AddChild(path_generator_node);
      lane_process_sequence->AddChild(speed_generator_node);

      lane_priority_selector->AddChild(lane_process_sequence);

      maneuver_sequence->AddChild(lane_prioritizer);
      maneuver_sequence->AddChild(lane_priority_selector);

      action_selector->AddChild(emergency_sequence);
      action_selector->AddChild(maneuver_sequence);

      lane_follow_sequence->AddChild(obstacle_processor);
      lane_follow_sequence->AddChild(action_selector);

      behaviour_tree_ = std::unique_ptr<BTNode>(lane_follow_sequence);
    }

Stage::StageStatus MyLaneFollowStage::Process(const TrajectoryPoint& planning_start_point, Frame* frame) 
{
  ADEBUG << "Number of reference lines:\t" << frame->mutable_reference_line_info()->size();
  bool has_drivable_lane = false;
  
  behaviour_tree_->Process(frame);

  ADEBUG << "Reference lines after planning: ";
  for (auto& ref_line : *frame->mutable_reference_line_info()) 
  {
    ADEBUG << "Id: " << ref_line.Lanes().Id() << " Cost: " << ref_line.Cost() << " Drivable: " << ref_line.IsDrivable();
    if (ref_line.IsDrivable())
    {
      ref_line.set_trajectory_type(ADCTrajectory::NORMAL);

      DiscretizedTrajectory trajectory;
      if (ref_line.CombinePathAndSpeedProfile(planning_start_point.relative_time(),
                                              planning_start_point.path_point().s(), 
                                              &trajectory)) 
      {
        ref_line.SetTrajectory(trajectory);
        has_drivable_lane = true;
      }
    }
  }

  return has_drivable_lane ? StageStatus::RUNNING : StageStatus::ERROR;
}

}  // namespace lane_follow
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
