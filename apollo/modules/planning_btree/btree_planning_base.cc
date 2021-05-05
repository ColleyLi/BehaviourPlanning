#include "modules/planning_btree/btree_planning_base.h"

#include "cyber/time/clock.h"
#include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
namespace planning_btree {

using apollo::common::math::Vec2d;
using apollo::common::ErrorCode;
using apollo::cyber::Clock;
using apollo::hdmap::HDMapUtil;
using apollo::common::Status;
using apollo::common::VehicleState;
using apollo::routing::RoutingResponse;

bool IsVehicleStateValid(const VehicleState& vehicle_state) 
{
  if (std::isnan(vehicle_state.x()) || std::isnan(vehicle_state.y()) ||
      std::isnan(vehicle_state.z()) || std::isnan(vehicle_state.heading()) ||
      std::isnan(vehicle_state.kappa()) ||
      std::isnan(vehicle_state.linear_velocity()) ||
      std::isnan(vehicle_state.linear_acceleration())) 
  {
    return false;
  }
  return true;
}

bool IsDifferentRouting(const RoutingResponse& first, const RoutingResponse& second) 
{
  if (first.has_header() && second.has_header()) 
  {
    return first.header().sequence_num() != second.header().sequence_num();
  }
  return true;
}

BTreePlanningBase::BTreePlanningBase(const std::shared_ptr<DependencyInjector>& injector) : injector_(injector)
{

}

BTreePlanningBase::~BTreePlanningBase()
{

}

Status BTreePlanningBase::Init(const BTreePlanningConfig& config)
{ 
    config_ = config;
    
    BTPlan btplan;
    cyber::common::GetProtoFromFile(FLAGS_btplan_file, &btplan);
    injector_->planning_state()->Clear();
    injector_->planning_state()->set_allocated_btplan(&btplan);
    
    hdmap_ = HDMapUtil::BaseMapPtr();
    ACHECK(hdmap_) << "Failed to load map";

    reference_line_provider_ = std::make_unique<ReferenceLineProvider>(injector_->vehicle_state_provider(), hdmap_);
    reference_line_provider_->Start();

    planner_ = std::make_unique<BTreePlanner>(injector_);
    planner_->Init(config_);
    
    start_time_ = Clock::NowInSeconds();

    return Status::OK();
}

Status BTreePlanningBase::InitFrame(const uint32_t sequence_num,
                                 const TrajectoryPoint& planning_start_point,
                                 const VehicleState& vehicle_state) 
{
  frame_.reset(new BTreeFrame(sequence_num, planning_input_, planning_start_point,
                              vehicle_state, reference_line_provider_.get()));

  if (frame_ == nullptr) 
  {
    return Status(ErrorCode::PLANNING_ERROR, "Failed to init frame: nullptr.");
  }

  std::list<ReferenceLine> reference_lines;
  std::list<hdmap::RouteSegments> segments;
  if (!reference_line_provider_->GetReferenceLines(&reference_lines, &segments)) 
  {
    const std::string msg = "Reference line provider failed to create reference lines";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  DCHECK_EQ(reference_lines.size(), segments.size());

  auto forward_limit = hdmap::PncMap::LookForwardDistance(vehicle_state.linear_velocity());

  for (auto& ref_line : reference_lines) 
  {
    if (!ref_line.Segment(Vec2d(vehicle_state.x(), vehicle_state.y()), FLAGS_btree_look_backward_distance, forward_limit)) 
    {
      const std::string msg = "Failed to shrink reference lines";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  for (auto& seg : segments) 
  {
    if (!seg.Shrink(Vec2d(vehicle_state.x(), vehicle_state.y()), FLAGS_btree_look_backward_distance, forward_limit)) 
    {
      const std::string msg = "Failed to shrink routing segments";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  auto status = frame_->Init(injector_->vehicle_state_provider(), reference_lines, segments,
                             reference_line_provider_->FutureRouteWaypoints());
  if (!status.ok()) 
  {
    AERROR << "failed to init frame:" << status.ToString();
    return status;
  }

  return Status::OK();
}

void BTreePlanningBase::Execute(const PlanningInput& planning_input, 
                                ADCTrajectory* const result_trajectory)
{
  planning_input_ = planning_input;
  
  Status status = injector_->vehicle_state_provider()->Update(*planning_input_.localization_estimate, *planning_input_.chassis);
  VehicleState vehicle_state = injector_->vehicle_state_provider()->vehicle_state();

  if (!status.ok() || !IsVehicleStateValid(vehicle_state)) 
  {
    const std::string msg = "Update VehicleStateProvider failed or the vehicle state is outdated";
    AERROR << msg;
    return;
  }
   if (IsDifferentRouting(last_routing_, *planning_input_.routing)) 
  {
    AERROR << "New routing!";
    last_routing_ = *planning_input_.routing;
    reference_line_provider_->UpdateRoutingResponse(*planning_input_.routing);
    // injector_->planning_context()->mutable_planning_status()->Clear();
    // planner_->Init(config_);
  }
   
  auto failed_to_update_reference_line = !reference_line_provider_->UpdatedReferenceLine();
  if (failed_to_update_reference_line) 
  {
    const std::string msg = "Reference line provider failed to update reference line after rerouting";
    AERROR << msg;
    return;
  }

  reference_line_provider_->UpdateVehicleState(vehicle_state);

  const double planning_cycle_time = 1.0 / FLAGS_btree_planning_loop_rate;

//   std::string replan_reason;
//   std::vector<TrajectoryPoint> stitching_trajectory =
//       TrajectoryStitcher::ComputeStitchingTrajectory(
//           vehicle_state, start_timestamp, planning_cycle_time,
//           FLAGS_btree_trajectory_stitching_preserved_length, true,
//           last_publishable_trajectory_.get(), &replan_reason);

  TrajectoryPoint point;
  point.mutable_path_point()->set_s(0.0);
  point.mutable_path_point()->set_x(vehicle_state.x());
  point.mutable_path_point()->set_y(vehicle_state.y());
  point.mutable_path_point()->set_z(vehicle_state.z());
  point.mutable_path_point()->set_theta(vehicle_state.heading());
  point.mutable_path_point()->set_kappa(vehicle_state.kappa());
  point.set_v(vehicle_state.linear_velocity());
  point.set_a(vehicle_state.linear_acceleration());
  point.set_relative_time(planning_cycle_time);

  const uint32_t frame_num = static_cast<uint32_t>(seq_num_++);
//   status = InitFrame(frame_num, stitching_trajectory.back(), vehicle_state);
  status = InitFrame(frame_num, point, vehicle_state);

  planner_->Execute(point, frame_.get(), result_trajectory);
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
