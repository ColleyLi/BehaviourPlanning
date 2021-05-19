#pragma once

#include <memory>
#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning_btree/common/obstacle.h"
#include "modules/planning_btree/common/obstacle_decisions.h"
#include "modules/planning_btree/common/speed/speed_data.h"
#include "modules/planning_btree/common/speed/st_point.h"
#include "modules/planning_btree/common/speed/st_graph_data.h"
#include "modules/planning_btree/behaviours/tasks/speed_generator/dp_st_cost.h"
#include "modules/planning_btree/behaviours/tasks/speed_generator/st_graph_point.h"

namespace apollo {
namespace planning_btree {

class GriddedPathTimeGraph {
 public:
  GriddedPathTimeGraph(const StGraphData& st_graph_data,
                       const std::vector<const Obstacle*>& obstacles,
                       const common::TrajectoryPoint& init_point);

  bool Search(SpeedData* const speed_data);

 private:
  bool InitCostTable();

  bool InitSpeedLimitLookUp();

  bool RetrieveSpeedProfile(SpeedData* const speed_data);

  bool CalculateTotalCost();

  // defined for cyber task
  struct StGraphMessage {
    StGraphMessage(const uint32_t c_, const int32_t r_) : c(c_), r(r_) {}
    uint32_t c;
    uint32_t r;
  };
  void CalculateCostAt(const std::shared_ptr<StGraphMessage>& msg);

  double CalculateEdgeCost(const STPoint& first, const STPoint& second,
                           const STPoint& third, const STPoint& forth,
                           const double speed_limit, const double cruise_speed);
  double CalculateEdgeCostForSecondCol(const uint32_t row,
                                       const double speed_limit,
                                       const double cruise_speed);
  double CalculateEdgeCostForThirdCol(const uint32_t curr_row,
                                      const uint32_t pre_row,
                                      const double speed_limit,
                                      const double cruise_speed);

  // get the row-range of next time step
  void GetRowRange(const StGraphPoint& point, size_t* next_highest_row,
                   size_t* next_lowest_row);

 private:
  const StGraphData& st_graph_data_;

  std::vector<double> speed_limit_by_index_;

  std::vector<double> spatial_distance_by_index_;

  // obstacles based on the current reference line
  const std::vector<const Obstacle*>& obstacles_;

  // vehicle configuration parameter
  const common::VehicleParam& vehicle_param_ =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  // initial status
  common::TrajectoryPoint init_point_;

  // cost utility with configuration;
  DpStCost dp_st_cost_;

  double total_length_t_ = 0.0;
  double unit_t_ = 0.0;
  uint32_t dimension_t_ = 0;

  double total_length_s_ = 0.0;
  double dense_unit_s_ = 0.0;
  double sparse_unit_s_ = 0.0;
  uint32_t dense_dimension_s_ = 0;
  uint32_t sparse_dimension_s_ = 0;
  uint32_t dimension_s_ = 0;

  double max_acceleration_ = 0.0;
  double max_deceleration_ = 0.0;

  // cost_table_[t][s]
  // row: s, col: t --- NOTICE: Please do NOT change.
  std::vector<std::vector<StGraphPoint>> cost_table_;
};

}  // namespace planning_btree
}  // namespace apollo
