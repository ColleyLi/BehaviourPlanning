#include "modules/planning_btree/btree_planning_base.h"

#include "cyber/time/clock.h"
// #include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
namespace planning_btree {

using apollo::common::Status;

BTreePlanningBase::BTreePlanningBase()
{

}

BTreePlanningBase::~BTreePlanningBase()
{

}

Status BTreePlanningBase::Init(const BTreePlanningConfig& config) 
{   
    config_ = config;
    planner_ = std::make_unique<BTreePlanner>();
    planner_->Init(config_);


    return Status::OK();
}


void BTreePlanningBase::Execute(const PlanningInput& planning_input, 
                                ADCTrajectory* const result_trajectory)
{
  planning_input_ = planning_input;
  frame_.reset(new BTreeFrame(0));
  TrajectoryPoint tp;

  planner_->Execute(tp, frame_.get(), result_trajectory);
}

apollo::common::Status BTreePlanningBase::Plan(
      const double current_time_stamp,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* const trajectory)
{
    // planner_.Execute();
    return Status::OK();
}

void BTreePlanningBase::FillPlanningPb(const double timestamp,
                                  ADCTrajectory* const trajectory_pb) {
  trajectory_pb->mutable_header()->set_timestamp_sec(timestamp);
  if (planning_input_.prediction_obstacles->has_header()) {
    trajectory_pb->mutable_header()->set_lidar_timestamp(
        planning_input_.prediction_obstacles->header().lidar_timestamp());
    trajectory_pb->mutable_header()->set_camera_timestamp(
        planning_input_.prediction_obstacles->header().camera_timestamp());
    trajectory_pb->mutable_header()->set_radar_timestamp(
        planning_input_.prediction_obstacles->header().radar_timestamp());
  }
  trajectory_pb->mutable_routing_header()->CopyFrom(
      planning_input_.routing->header());
}
}  // namespace planning_btree
}  // namespace apollo
