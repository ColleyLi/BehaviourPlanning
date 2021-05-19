#pragma once

#include "modules/planning_btree/common/trajectory/discretized_trajectory.h"
#include "modules/planning/proto/planning.pb.h"

namespace apollo {
namespace planning_btree {

using planning::ADCTrajectory;

class PublishableTrajectory : public DiscretizedTrajectory {
 public:
  PublishableTrajectory() = default;

  PublishableTrajectory(const double header_time,
                        const DiscretizedTrajectory& discretized_trajectory);
  /**
   * Create a publishable trajectory based on a trajectory protobuf
   */
  explicit PublishableTrajectory(const ADCTrajectory& trajectory_pb);

  double header_time() const;

  void PopulateTrajectoryProtobuf(ADCTrajectory* trajectory_pb) const;

 private:
  double header_time_ = 0.0;
};

}  // namespace planning_btree
}  // namespace apollo
