#pragma once

#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/status/status.h"
#include "modules/planning_btree/common/obstacle.h"
#include "modules/planning_btree/common/path/path_data.h"
#include "modules/planning_btree/common/obstacle_decisions.h"
#include "modules/planning_btree/common/speed/speed_data.h"
#include "modules/planning_btree/common/speed/st_boundary.h"
#include "modules/planning_btree/common/speed/speed_limit.h"
#include "modules/planning_btree/common/trajectory/discretized_trajectory.h"
#include "modules/planning_btree/reference_line/reference_line.h"

namespace apollo {
namespace planning_btree {

// TODO(jiacheng): currently implemented a constant velocity model for
// guide-line. Upgrade it to a constant acceleration model.
class STGuideLine {
 public:
  STGuideLine() {}

  void Init(double desired_v);

  void Init(double desired_v,
            const std::vector<common::TrajectoryPoint> &speed_reference);

  virtual ~STGuideLine() = default;

  double GetGuideSFromT(double t);

  void UpdateBlockingInfo(const double t, const double s_block,
                          const bool is_lower_block);

 private:
  // Variables for simple guide-line calculation.
  double t0_;
  double s0_;
  double v0_;
  // St guideline from upstream modules
  SpeedData guideline_speed_data_;
};

}  // namespace planning_btree
}  // namespace apollo
