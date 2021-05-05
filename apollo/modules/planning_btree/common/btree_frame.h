#pragma once

#include <list>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

// #include "modules/common/math/vec2d.h"
// #include "modules/common/monitor_log/monitor_log_buffer.h"
// #include "modules/common/proto/geometry.pb.h"
// #include "modules/common/status/status.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
// #include "modules/localization/proto/pose.pb.h"
// #include "modules/planning/common/ego_info.h"
#include "modules/planning_btree/common/indexed_queue.h"
#include "modules/planning_btree/common/planning_input.h"
// #include "modules/planning/common/obstacle.h"
// #include "modules/planning/common/open_space_info.h"
#include "modules/planning_btree/common/dynamic_reference_line.h"
// #include "modules/planning/common/trajectory/publishable_trajectory.h"
// #include "modules/planning/proto/pad_msg.pb.h"
// #include "modules/planning/proto/planning.pb.h"
// #include "modules/planning/proto/planning_config.pb.h"
// #include "modules/planning/proto/planning_internal.pb.h"
#include "modules/planning_btree/reference_line/reference_line_provider.h"
// #include "modules/prediction/proto/prediction_obstacle.pb.h"
// #include "modules/routing/proto/routing.pb.h"

namespace apollo {
namespace planning_btree {

class BTreeFrame {
 public:
  explicit BTreeFrame(uint32_t sequence_num);

  BTreeFrame(uint32_t sequence_num, const PlanningInput &planning_input,
        const common::TrajectoryPoint &planning_start_point,
        const common::VehicleState &vehicle_state,
        ReferenceLineProvider *reference_line_provider);

  BTreeFrame(uint32_t sequence_num, const PlanningInput &planning_input,
        const common::TrajectoryPoint &planning_start_point,
        const common::VehicleState &vehicle_state);

  ~BTreeFrame() = default;

  // const common::TrajectoryPoint &PlanningStartPoint() const;

  common::Status Init(
      const common::VehicleStateProvider *vehicle_state_provider,
      const std::list<ReferenceLine> &reference_lines,
      const std::list<hdmap::RouteSegments> &segments,
      const std::vector<routing::LaneWaypoint> &future_route_waypoints);

//   uint32_t SequenceNum() const;

//   const PublishableTrajectory &ComputedTrajectory() const;

//   const std::list<ReferenceLineInfo> &reference_line_info() const;
//   std::list<ReferenceLineInfo> *mutable_reference_line_info();

//   Obstacle *Find(const std::string &id);

//   const ReferenceLineInfo *FindDriveReferenceLineInfo();

//   const ReferenceLineInfo *FindTargetReferenceLineInfo();

//   const ReferenceLineInfo *FindFailedReferenceLineInfo();

//   const ReferenceLineInfo *DriveReferenceLineInfo() const;

//   const std::vector<const Obstacle *> obstacles() const;

//   const Obstacle *CreateStopObstacle(
//       ReferenceLineInfo *const reference_line_info,
//       const std::string &obstacle_id, const double obstacle_s);

//   const Obstacle *CreateStopObstacle(const std::string &obstacle_id,
//                                      const std::string &lane_id,
//                                      const double lane_s);

//   const Obstacle *CreateStaticObstacle(
//       ReferenceLineInfo *const reference_line_info,
//       const std::string &obstacle_id, const double obstacle_start_s,
//       const double obstacle_end_s);

//   bool Rerouting(PlanningContext *planning_context);

//   const common::VehicleState &vehicle_state() const;

//   static void AlignPredictionTime(
//       const double planning_start_time,
//       prediction::PredictionObstacles *prediction_obstacles);

//   void set_current_frame_planned_trajectory(
//       ADCTrajectory current_frame_planned_trajectory) {
//     current_frame_planned_trajectory_ =
//         std::move(current_frame_planned_trajectory);
//   }

//   const ADCTrajectory &current_frame_planned_trajectory() const {
//     return current_frame_planned_trajectory_;
//   }

//   void set_current_frame_planned_path(
//       DiscretizedPath current_frame_planned_path) {
//     current_frame_planned_path_ = std::move(current_frame_planned_path);
//   }

//   const DiscretizedPath &current_frame_planned_path() const {
//     return current_frame_planned_path_;
//   }

//   const bool is_near_destination() const {
//     return is_near_destination_;
//   }

//   void UpdateReferenceLinePriority(
    //   const std::map<std::string, uint32_t> &id_to_priority);

  // const PlanningInput &planning_input() const {
    // return planning_input_;
  // }

//   ThreadSafeIndexedObstacles *GetObstacleList() {
//     return &obstacles_;
//   }

//   const OpenSpaceInfo &open_space_info() const {
//     return open_space_info_;
//   }

//   OpenSpaceInfo *mutable_open_space_info() {
//     return &open_space_info_;
//   }

//   perception::TrafficLight GetSignal(const std::string &traffic_light_id) const;

//   const DrivingAction &GetPadMsgDrivingAction() const {
//     return pad_msg_driving_action_;
//   }

 // private:
  // common::Status InitFrameData(
      // const common::VehicleStateProvider *vehicle_state_provider,
      // const EgoInfo *ego_info);

//   bool CreateReferenceLineInfo(const std::list<ReferenceLine> &reference_lines,
//                                const std::list<hdmap::RouteSegments> &segments);

//   const Obstacle *FindCollisionObstacle(const EgoInfo *ego_info) const;

//   /**
//    * @brief create a static virtual obstacle
//    */
//   const Obstacle *CreateStaticVirtualObstacle(const std::string &id,
//                                               const common::math::Box2d &box);

//   void AddObstacle(const Obstacle &obstacle);

//   void ReadTrafficLights();

//   void ReadPadMsgDrivingAction();
//   void ResetPadMsgDrivingAction();

 private:
//   static DrivingAction pad_msg_driving_action_;
  uint32_t sequence_num_ = 0;
  PlanningInput planning_input_;
//   const hdmap::HDMap *hdmap_ = nullptr;
  common::TrajectoryPoint planning_start_point_;
  common::VehicleState vehicle_state_;
  std::list<DynamicReferenceLine> dynamic_reference_lines_;

  bool is_near_destination_ = false;

//   const ReferenceLineInfo *drive_reference_line_info_ = nullptr;

//   ThreadSafeIndexedObstacles obstacles_;

//   std::unordered_map<std::string, const perception::TrafficLight *>
    //   traffic_lights_;

//   current frame published trajectory
//   ADCTrajectory current_frame_planned_trajectory_;

  // current frame path for future possible speed fallback
//   DiscretizedPath current_frame_planned_path_;

  const ReferenceLineProvider *reference_line_provider_ = nullptr;
// 
//   OpenSpaceInfo open_space_info_;

//   std::vector<routing::LaneWaypoint> future_route_waypoints_;

//   common::monitor::MonitorLogBuffer monitor_logger_buffer_;
};

class BTreeFrameHistory : public IndexedQueue<uint32_t, BTreeFrame> 
{
 public:
  BTreeFrameHistory();
};

}  // namespace planning_btree
}  // namespace apollo