#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/status/status.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
// #include "modules/dreamview/proto/chart.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/map/hdmap/hdmap.h"
// #include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning_btree/common/btree_frame.h"
// #include "modules/planning/common/trajectory/publishable_trajectory.h"
// #include "modules/planning/proto/planning.pb.h"
// #include "modules/planning/proto/planning_config.pb.h"
// #include "modules/planning/proto/traffic_rule_config.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"
#include "modules/planning_btree/common/planning_input.h"
#include "modules/planning_btree/proto/btree_planning_config.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning_btree/common/dependency_injector.h"
#include "modules/planning_btree/common/btree_planning_gflags.h"
#include "modules/planning_btree/btree_planner.h"
#include "modules/planning_btree/common/btree_planning_gflags.h"

namespace apollo {
namespace planning_btree {

using apollo::planning::ADCTrajectory;

class BTreePlanningBase {
 public:
  BTreePlanningBase(const std::shared_ptr<DependencyInjector>& injector);

  ~BTreePlanningBase();

  apollo::common::Status Init(const BTreePlanningConfig& config);

  void Execute(const PlanningInput& planning_input, 
               ADCTrajectory* const result_trajectory);

 private:
  apollo::common::Status Plan(
      const double current_time_stamp,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* const trajectory);

  void FillPlanningPb(const double timestamp, 
                      ADCTrajectory* const trajectory_pb);

  common::Status InitFrame(const uint32_t sequence_num,
                           const common::TrajectoryPoint& planning_start_point,
                           const common::VehicleState& vehicle_state); 
                           
  // bool CheckPlanningConfig(const PlanningConfig& config);
  // void GenerateStopTrajectory(ADCTrajectory* ptr_trajectory_pb);
 
 private:
  PlanningInput planning_input_;
  const hdmap::HDMap* hdmap_ = nullptr;

  double start_time_ = 0.0;
  size_t seq_num_ = 0;

  BTreePlanningConfig config_;
  // TrafficRuleConfigs traffic_rule_configs_;

  std::unique_ptr<BTreeFrame> frame_;
  std::unique_ptr<BTreePlanner> planner_;
  // std::unique_ptr<PublishableTrajectory> last_publishable_trajectory_;
  std::shared_ptr<DependencyInjector> injector_;
  
  routing::RoutingResponse last_routing_;
  std::unique_ptr<ReferenceLineProvider> reference_line_provider_;
  // Smoother planning_smoother_
};

}  // namespace planning_btree
}  // namespace apollo
