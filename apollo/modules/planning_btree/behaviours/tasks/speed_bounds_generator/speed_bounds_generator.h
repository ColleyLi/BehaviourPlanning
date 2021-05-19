#pragma once

#include "modules/planning_btree/behaviours/btree_task.h"
#include "modules/planning_btree/behaviours/tasks/speed_bounds_generator/st_driving_limits.h"
#include "modules/planning_btree/behaviours/tasks/speed_bounds_generator/st_guide_line.h"
#include "modules/planning_btree/behaviours/tasks/speed_bounds_generator/st_obstacles_processor.h"
#include "modules/planning_btree/behaviours/tasks/speed_bounds_generator/speed_limit_decider.h"
#include "modules/planning_btree/behaviours/tasks/speed_bounds_generator/st_boundary_mapper.h"

namespace apollo {
namespace planning_btree {

class SpeedBoundsGenerator : public BTreeTask {
 public:
  BTreeNodeState Init(const BTreeNodeConfig& config);
  BTreeNodeState Execute(BTreeFrame* frame);

 private:
  bool SpeedBoundsDecider(BTreeFrame* const frame,
                          DynamicReferenceLine* const dynamic_reference_line);

  bool STBoundsDecider(BTreeFrame* const frame,
                       DynamicReferenceLine* const dynamic_reference_line);
  void InitSTBoundsDecider(const BTreeFrame& frame,
                           DynamicReferenceLine* const dynamic_reference_line);

  bool GenerateRegularSTBound(
      std::vector<std::tuple<double, double, double>>* const st_bound,
      std::vector<std::tuple<double, double, double>>* const vt_bound,
      std::vector<std::pair<double, double>>* const st_guide_line);

  void RemoveInvalidDecisions(
      std::pair<double, double> driving_limit,
      std::vector<
          std::pair<std::tuple<double, double, double>,
                    std::vector<std::pair<std::string, ObjectDecisionType>>>>*
          available_choices);

  void RankDecisions(
      double s_guide_line, std::pair<double, double> driving_limit,
      std::vector<
          std::pair<std::tuple<double, double, double>,
                    std::vector<std::pair<std::string, ObjectDecisionType>>>>*
          available_choices);

  double SetSpeedFallbackDistance(ObstacleDecisions* const obstacle_decisions);

 private:
  STGuideLine st_guide_line_;
  STDrivingLimits st_driving_limits_;
  STObstaclesProcessor st_obstacles_processor_;
};

}  // namespace planning_btree
}  // namespace apollo