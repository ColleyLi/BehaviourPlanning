#pragma once

#include <tuple>
#include <utility>
#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/status/status.h"
#include "modules/planning_btree/common/obstacle.h"
#include "modules/planning_btree/common/path/path_data.h"
#include "modules/planning_btree/common/obstacle_decisions.h"
#include "modules/planning_btree/common/speed/st_boundary.h"
#include "modules/planning_btree/common/speed/speed_limit.h"
#include "modules/planning_btree/reference_line/reference_line.h"

namespace apollo {
namespace planning_btree {

class STDrivingLimits {
 public:
  STDrivingLimits() {}

  void Init(const double max_acc, const double max_dec, const double max_v,
            double curr_v);

  virtual ~STDrivingLimits() = default;

  /** @brief Given time t, calculate the driving limits in s due to
   * vehicle's dynamics.
   * @param Timestamp t.
   * @return The lower and upper bounds.
   */
  std::pair<double, double> GetVehicleDynamicsLimits(const double t) const;

  /** @brief Update the anchoring of the vehicle dynamics limits.
   * For example, when ADC is blocked by some obstacle, its max.
   * drivable area, max. speed, etc. are also limited subsequently.
   * @param Time t
   * @param lower bound in s
   * @param lower bound's corresponding speed.
   * @param upper bound in s
   * @param upper bound's corresponding speed.
   */
  void UpdateBlockingInfo(const double t, const double lower_s,
                          const double lower_v, const double upper_s,
                          const double upper_v);

 private:
  // Private variables for calculating vehicle dynamic limits:
  double max_acc_;
  double max_dec_;
  double max_v_;

  double upper_t0_;
  double upper_v0_;
  double upper_s0_;

  double lower_t0_;
  double lower_v0_;
  double lower_s0_;

  // The limits expressed as v vs. s, which contains the following parts:
  //  1. speed limits at path segments with big curvatures.
  std::vector<std::tuple<double, double, double>> curvature_speed_limits_s_v_;
  //  2. speed limits from traffic limits (speed bumps, etc.).
  std::vector<std::tuple<double, double, double>> traffic_speed_limits_s_v_;
  //  3. speed limits for safety considerations when other obstacles are nearby
  std::vector<std::tuple<double, double, double>> obstacles_speed_limits_s_v_;
};

}  // namespace planning_btree
}  // namespace apollo
