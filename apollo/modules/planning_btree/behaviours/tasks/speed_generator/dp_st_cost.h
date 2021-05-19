#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning_btree/common/obstacle.h"
#include "modules/planning_btree/common/speed/st_boundary.h"
#include "modules/planning_btree/common/speed/st_point.h"
#include "modules/planning_btree/proto/st_drivable_boundary.pb.h"
#include "modules/planning_btree/behaviours/tasks/speed_generator/st_graph_point.h"

namespace apollo {
namespace planning_btree {

class DpStCost {
 public:
  DpStCost(const double total_t,
           const double total_s, 
           const double unit_t,
           const std::vector<const Obstacle*>& obstacles,
           const STDrivableBoundary& st_drivable_boundary,
           const common::TrajectoryPoint& init_point);

  double GetObstacleCost(const StGraphPoint& point);

  double GetSpatialPotentialCost(const StGraphPoint& point);

  double GetReferenceCost(const STPoint& point,
                          const STPoint& reference_point) const;

  double GetSpeedCost(const STPoint& first, const STPoint& second,
                      const double speed_limit,
                      const double cruise_speed) const;

  double GetAccelCostByTwoPoints(const double pre_speed, const STPoint& first,
                                 const STPoint& second);
  double GetAccelCostByThreePoints(const STPoint& first, const STPoint& second,
                                   const STPoint& third);

  double GetJerkCostByTwoPoints(const double pre_speed, const double pre_acc,
                                const STPoint& pre_point,
                                const STPoint& curr_point);
  double GetJerkCostByThreePoints(const double first_speed,
                                  const STPoint& first_point,
                                  const STPoint& second_point,
                                  const STPoint& third_point);

  double GetJerkCostByFourPoints(const STPoint& first, const STPoint& second,
                                 const STPoint& third, const STPoint& fourth);

 private:
  double GetAccelCost(const double accel);
  double JerkCost(const double jerk);

  void AddToKeepClearRange(const std::vector<const Obstacle*>& obstacles);
  static void SortAndMergeRange(
      std::vector<std::pair<double, double>>* keep_clear_range_);
  bool InKeepClearRange(double s) const;

  const std::vector<const Obstacle*>& obstacles_;

  STDrivableBoundary st_drivable_boundary_;

  const common::TrajectoryPoint& init_point_;

  double unit_t_ = 0.0;
  double total_s_ = 0.0;

  std::unordered_map<std::string, int> boundary_map_;
  std::vector<std::vector<std::pair<double, double>>> boundary_cost_;

  std::vector<std::pair<double, double>> keep_clear_range_;

  std::array<double, 200> accel_cost_;
  std::array<double, 400> jerk_cost_;
};

}  // namespace planning_btree
}  // namespace apollo
