#pragma once

#include <string>
#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/planning_btree/common/obstacle.h"
#include "modules/planning_btree/common/path/path_data.h"
#include "modules/planning_btree/common/speed/speed_limit.h"
#include "modules/planning_btree/reference_line/reference_line.h"

namespace apollo {
namespace planning_btree {

class SpeedLimitDecider {
 public:
  SpeedLimitDecider(const ReferenceLine& reference_line,
                    const PathData& path_data);

  virtual ~SpeedLimitDecider() = default;

  virtual bool GetSpeedLimits(
      const IndexedList<std::string, Obstacle>& obstacles,
      SpeedLimit* const speed_limit_data) const;

 private:
  double GetCentricAccLimit(const double kappa) const;

  void GetAvgKappa(const std::vector<common::PathPoint>& path_points,
                   std::vector<double>* kappa) const;

 private:
  const ReferenceLine& reference_line_;
  const PathData& path_data_;
  const apollo::common::VehicleParam& vehicle_param_;
};

}  // namespace planning_btree
}  // namespace apollo
