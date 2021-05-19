#include "modules/planning_btree/common/trajectory/publishable_trajectory.h"

#include "cyber/common/log.h"

namespace apollo {
namespace planning_btree {

PublishableTrajectory::PublishableTrajectory(
    const double header_time,
    const DiscretizedTrajectory& discretized_trajectory)
    : DiscretizedTrajectory(discretized_trajectory),
      header_time_(header_time) {}

PublishableTrajectory::PublishableTrajectory(const ADCTrajectory& trajectory_pb)
    : DiscretizedTrajectory(trajectory_pb),
      header_time_(trajectory_pb.header().timestamp_sec()) {}

double PublishableTrajectory::header_time() const { return header_time_; }

void PublishableTrajectory::PopulateTrajectoryProtobuf(
    ADCTrajectory* trajectory_pb) const {
  CHECK_NOTNULL(trajectory_pb);
  trajectory_pb->mutable_header()->set_timestamp_sec(header_time_);
  trajectory_pb->mutable_trajectory_point()->CopyFrom({begin(), end()});
  if (!empty()) {
    const auto& last_tp = back();
    trajectory_pb->set_total_path_length(last_tp.path_point().s());
    trajectory_pb->set_total_path_time(last_tp.relative_time());
  }
}

}  // namespace planning_btree
}  // namespace apollo
