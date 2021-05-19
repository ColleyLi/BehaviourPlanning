#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/point_factory.h"
#include "modules/planning_btree/behaviours/btree_task.h"
#include "modules/planning_btree/common/path/path_data.h"
#include "modules/planning_btree/common/trajectory1d/piecewise_jerk_trajectory1d.h"
#include "modules/planning_btree/math/piecewise_jerk/piecewise_jerk_path_problem.h"

namespace apollo {
namespace planning_btree {

class PathGenerator : public BTreeTask {
  BTreeNodeState Init(const BTreeNodeConfig& config);
  BTreeNodeState Execute(BTreeFrame* frame);

  bool OptimizePath(
      const std::array<double, 3>& init_state,
      const std::array<double, 3>& end_state,
      std::vector<double> path_reference_l_ref,
      const size_t path_reference_size, const double delta_s,
      const bool is_valid_path_reference,
      const std::vector<std::pair<double, double>>& lat_boundaries,
      const std::vector<std::pair<double, double>>& ddl_bounds,
      const std::array<double, 5>& w, const int max_iter,
      std::vector<double>* ptr_x, std::vector<double>* ptr_dx,
      std::vector<double>* ptr_ddx);

  FrenetFramePath ToPiecewiseJerkPath(const std::vector<double>& l,
                                      const std::vector<double>& dl,
                                      const std::vector<double>& ddl,
                                      const double delta_s,
                                      const double start_s) const;

  double EstimateJerkBoundary(const double vehicle_speed,
                              const double axis_distance,
                              const double max_steering_rate) const;

  double GaussianWeighting(const double x, const double peak_weighting,
                           const double peak_weighting_x) const;
};

}  // namespace planning_btree
}  // namespace apollo