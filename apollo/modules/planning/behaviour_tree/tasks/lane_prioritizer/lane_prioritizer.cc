#include "modules/planning/behaviour_tree/tasks/lane_prioritizer/lane_prioritizer.h" 

namespace apollo {
namespace planning {
namespace behaviour_tree {

namespace
{
  const double kNearOffset = 8.0;
  const double kLaneWidth = 2.0;
  const double kPlanningHorizon = 60.0;
  const int kLaneChangeCost = 10;
  const int kLaneStayCost = 0;
  const int kBlockingObstacleCost = 100;
  const int kObstacleCost = 5;
  const int kFrontObstacleCost = 50;
}  
  BTreeNodeState LanePrioritizer::Init(const BTreeNodeConfig& config)
  {
    config_ = config;
    state_ = BTreeNodeState::NODE_INITIALIZED;
    return state_;
  }

  BTreeNodeState LanePrioritizer::Execute(Frame* frame)
  {
    for (auto& ref_line : *frame->mutable_reference_line_info())
    {
      if (ref_line.IsChangeLanePath())
      {
        ref_line.AddCost(kLaneChangeCost);
      }
      else
      {
        ref_line.AddCost(kLaneStayCost);
      }

      std::string blocking_obstacle_id = ref_line.GetBlockingObstacleId();
      if (blocking_obstacle_id.size() != 0)
      {
        ref_line.AddCost(kBlockingObstacleCost);
      }

      const auto& ego_sl_boundary = ref_line.AdcSlBoundary();
      PathDecision *const path_decision = ref_line.path_decision();
      for (const auto *obstacle : path_decision->obstacles().Items()) 
      {
        if (obstacle->IsVirtual())
        {
          continue;
        }
        const auto &obstacle_sl_boundary = obstacle->PerceptionSLBoundary();
        double obstacle_l = (obstacle_sl_boundary.start_l() + obstacle_sl_boundary.end_l()) / 2.0;
        AERROR << "Lane: " << ref_line.Lanes().Id() << " Obstacle: " << obstacle->Id() << " L: " << obstacle_l; 

        // Check if obstacle is not within the lane
        if(std::abs(obstacle_l) > kLaneWidth / 2.0)
        {
          continue;
        }

        // Check if obstacle is too far 
        if (std::abs(obstacle_sl_boundary.start_s() - ego_sl_boundary.start_s()) > kPlanningHorizon)
        {
          continue;
        }

        if (obstacle_sl_boundary.start_s() > ego_sl_boundary.start_s() - kNearOffset)
        {
          ref_line.AddCost(kFrontObstacleCost);
        }
        else
        {
          ref_line.AddCost(kObstacleCost);
        }
      } 
    }

    state_ = BTreeNodeState::NODE_DONE;
    return state_;
  }

  BTreeNodeState LanePrioritizer::Execute(Frame* frame, ReferenceLineInfo* reference_line_info)
  {
    return Execute(frame);
  }


} // namespace behaviour_tree
} // namespace planning
} // namespace apollo