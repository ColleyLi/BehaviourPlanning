#include "modules/planning_btree/behaviours/tasks/lane_prioritizer/lane_prioritizer.h" 

#include <string>

namespace apollo {
namespace planning_btree {

namespace
{
  const double kNearOffset = 8.0;
  const double kLaneWidth = 4.0;
  const double kPlanningHorizon = 100.0;
  const int kLaneChangeCost = 10;
  const int kLaneStayCost = 0;
  const int kBlockingObstacleCost = 100;
  const int kObstacleCost = 5;
  const int kFrontObstacleCost = 50;
  const int kIsAddedPassage = 30;
  const int kNeedChangeLaneCost = 20;
}  

  BTreeNodeState LanePrioritizer::Init(const BTreeNodeConfig& config)
  {
    config_ = config;
    state_ = BTreeNodeState::NODE_INITIALIZED;
    return state_;
  }

  BTreeNodeState LanePrioritizer::Execute(BTreeFrame* frame)
  {
    auto planning_input = frame->GetPlanningInput();
    auto routing = planning_input.routing;

    ADEBUG << routing->DebugString();
    
    for (auto& ref_line : *frame->GetMutableDynamicReferenceLines())
    {
      bool is_added_passage = false;
      auto route_segments = ref_line.GetRouteSegments();

      auto id = route_segments.Id();
      auto road_segment_index = std::stoi(id.substr(0, id.find("_")));
      auto passage_index = std::stoi(id.substr(id.find("_") + 1, id.length()));

      auto road = routing->road(road_segment_index);
      if (passage_index > road.passage_size() - 1)
      {
        is_added_passage = true;
      }
      ADEBUG << route_segments.NextAction();
      ADEBUG << route_segments.CanExit();

      ref_line.SetCost(0.0);
      if (ref_line.IsLaneChangePath())
      {
        ref_line.AddCost(kLaneChangeCost);
      }
      else
      {
        ref_line.AddCost(kLaneStayCost);
      }

      if (is_added_passage)
      {
        ref_line.AddCost(kIsAddedPassage);
      }
      else if (!route_segments.CanExit())
      {
       ref_line.AddCost(kNeedChangeLaneCost); 
      }

      Obstacle* blocking_obstacle = ref_line.GetBlockingObstacle();
      if (blocking_obstacle)
      {
        ref_line.AddCost(kBlockingObstacleCost);
      }

      const auto& ego_sl_boundary = ref_line.GetADCSLBoundary();
      ObstacleDecisions *const obstacle_decisions = ref_line.GetMutableObstacleDecisions();
      for (const auto *obstacle : obstacle_decisions->obstacles().Items()) 
      {
        if (obstacle->IsVirtual())
        {
          continue;
        }
        const auto &obstacle_sl_boundary = obstacle->PerceptionSLBoundary();
        double obstacle_l = (obstacle_sl_boundary.start_l() + obstacle_sl_boundary.end_l()) / 2.0;
        // ADEBUG << "Route segment: " << ref_line.GetRouteSegments().Id() << " Obstacle: " << obstacle->Id() << " L: " << obstacle_l; 

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

} // namespace planning
} // namespace apollo