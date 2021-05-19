#pragma once

#include <memory>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning_btree/behaviours/btree_task.h"
#include "modules/planning_btree/common/path/path_boundary.h"

namespace apollo {
namespace planning_btree {

class PathBoundsGenerator : public BTreeTask {
 public:
  enum class LaneBorrowType {
    LEFT_BORROW,
    NO_BORROW,
    RIGHT_BORROW,
  };

  BTreeNodeState Init(const BTreeNodeConfig& config) override;
  BTreeNodeState Execute(BTreeFrame* frame) override;

 private:
  bool Init(const BTreeFrame& frame,
            const DynamicReferenceLine& dynamic_reference_line);

  bool GenerateRegularPathBound(
      const DynamicReferenceLine& dynamic_reference_line,
      const LaneBorrowType& lane_borrow_type,
      std::vector<std::tuple<double, double, double>>* const path_bound,
      std::string* const blocking_obstacle_id,
      std::string* const lane_borrow_direction);

  //   bool GenerateFallbackPathBound(
  //       const DynamicReferenceLine& dynamic_reference_line,
  //       std::vector<std::tuple<double, double, double>>* const path_bound);

  bool GenerateLaneChangePathBound(
      const DynamicReferenceLine& dynamic_reference_line,
      std::vector<std::tuple<double, double, double>>* const path_bound);

  bool InitPathBound(
      const DynamicReferenceLine& dynamic_reference_line,
      std::vector<std::tuple<double, double, double>>* const path_bound);

  //   bool GetBoundaryFromRoads(
  //       const DynamicReferenceLine& dynamic_reference_line,
  //       std::vector<std::tuple<double, double, double>>* const path_bound);

  //   bool GetBoundaryFromLanes(
  //       const DynamicReferenceLine& dynamic_reference_line,
  //       const LaneBorrowType& lane_borrow_type,
  //       std::vector<std::tuple<double, double, double>>* const path_bound,
  //       std::string* const lane_borrow_direction);

  //   bool GetBoundaryFromADC(
  //       const DynamicReferenceLine& dynamic_reference_line, double
  //       ADC_extra_buffer, std::vector<std::tuple<double, double, double>>*
  //       const path_bound);

  bool GetBoundaryFromLanesAndADC(
      const DynamicReferenceLine& dynamic_reference_line,
      const LaneBorrowType& lane_borrow_type, double ADC_buffer,
      std::vector<std::tuple<double, double, double>>* const path_bound,
      std::string* const lane_borrow_direction,
      bool is_fallback_lane_change = false);

  bool GetBoundaryFromStaticObstacles(
      const ObstacleDecisions& obstacle_decisions,
      std::vector<std::tuple<double, double, double>>* const path_boundaries,
      std::string* const blocking_obstacle_id);

  std::vector<std::tuple<int, double, double, double, std::string>>
  SortObstaclesForSweepLine(
      const IndexedList<std::string, Obstacle>& indexed_obstacles);

  bool IsWithinPathDeciderScopeObstacle(const Obstacle& obstacle);

  bool CheckLaneBoundaryType(const DynamicReferenceLine& dynamic_reference_line,
                             const double check_s,
                             const LaneBorrowType& lane_borrow_type);

  bool UpdatePathBoundaryAndCenterLineWithBuffer(
      size_t idx, double left_bound, double right_bound,
      std::vector<std::tuple<double, double, double>>* const path_boundaries,
      double* const center_line);

  bool UpdatePathBoundaryWithBuffer(
      size_t idx, double left_bound, double right_bound,
      std::vector<std::tuple<double, double, double>>* const path_boundaries,
      bool is_left_lane_bound = false, bool is_right_lane_bound = false);

  double GetBufferBetweenADCCenterAndEdge();

  void TrimPathBounds(
      const int path_blocked_idx,
      std::vector<std::tuple<double, double, double>>* const path_boundaries);

 private:
  double adc_s_ = 0.0;
  double adc_ds_ = 0.0;
  double adc_l_ = 0.0;
  double adc_dl_ = 0.0;
  double adc_lane_width_ = 0.0;
  double adc_l_to_lane_center_ = 0.0;
};

}  // namespace planning_btree
}  // namespace apollo