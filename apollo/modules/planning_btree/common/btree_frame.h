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
#include "modules/planning_btree/common/dependency_injector.h"
#include "modules/planning_btree/common/obstacle.h"
#include "modules/planning_btree/common/planning_input.h"
#include "modules/planning_btree/common/utils/indexed_queue.h"
// #include "modules/planning/common/open_space_info.h"
#include "modules/planning_btree/common/dynamic_reference_line.h"
// #include "modules/planning/common/trajectory/publishable_trajectory.h"
// #include "modules/planning/proto/pad_msg.pb.h"
// #include "modules/planning/proto/planning.pb.h"
// #include "modules/planning/proto/planning_config.pb.h"
// #include "modules/planning/proto/planning_internal.pb.h"
#include "modules/map/hdmap/hdmap_util.h"
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
             ReferenceLineProvider *reference_line_provider,
             const std::shared_ptr<DependencyInjector> &injector);

  BTreeFrame(uint32_t sequence_num, const PlanningInput &planning_input,
             const common::TrajectoryPoint &planning_start_point,
             const common::VehicleState &vehicle_state,
             const std::shared_ptr<DependencyInjector> &injector);

  ~BTreeFrame() = default;

  common::Status Init(
      const common::VehicleStateProvider *vehicle_state_provider,
      const std::list<ReferenceLine> &reference_lines,
      const std::list<hdmap::RouteSegments> &segments,
      const std::vector<routing::LaneWaypoint> &future_route_waypoints);

  const DependencyInjector &GetDependencyInjector() { return *injector_; }

  const PlanningInput &GetPlanningINput() {return planning_input_; }

  const bool IsNearDestination() const { return is_near_destination_; }

  const PlanningInput &GetPlanningInput() const { return planning_input_; }

  const std::list<DynamicReferenceLine> &GetDynamicReferenceLines() const;
  std::list<DynamicReferenceLine> *GetMutableDynamicReferenceLines();

  void SetCurrentDynamicReferenceLine(
      DynamicReferenceLine *dynamic_reference_line);
  const DynamicReferenceLine &GetCurrentDynamicReferenceLine() const;
  DynamicReferenceLine *GetMutableCurrentDynamicReferenceLine();

  const common::TrajectoryPoint &GetPlanningStartPoint() const;

  const std::vector<const Obstacle *> GetObstacles() const;

  const Obstacle *CreateStopObstacle(
      DynamicReferenceLine *const dynamic_reference_line,
      const std::string &obstacle_id, const double obstacle_s);

  const Obstacle *CreateStopObstacle(const std::string &obstacle_id,
                                     const std::string &lane_id,
                                     const double lane_s);

  const Obstacle *CreateStaticObstacle(
      DynamicReferenceLine *const dynamic_reference_line,
      const std::string &obstacle_id, const double obstacle_start_s,
      const double obstacle_end_s);

 private:
  const Obstacle *CreateStaticVirtualObstacle(const std::string &id,
                                              const common::math::Box2d &box);
  void AddObstacle(const Obstacle &obstacle);

 private:
  uint32_t sequence_num_ = 0;
  PlanningInput planning_input_;

  common::TrajectoryPoint planning_start_point_;
  common::VehicleState vehicle_state_;
  bool is_near_destination_ = false;

  const hdmap::HDMap *hdmap_ = nullptr;

  ThreadSafeIndexedObstacles obstacles_;

  std::list<DynamicReferenceLine> dynamic_reference_lines_;
  DynamicReferenceLine *current_dynamic_reference_line_ = nullptr;
  const ReferenceLineProvider *reference_line_provider_ = nullptr;

  std::shared_ptr<DependencyInjector> injector_;
};

class BTreeFrameHistory : public IndexedQueue<uint32_t, BTreeFrame> {
 public:
  BTreeFrameHistory();
};

}  // namespace planning_btree
}  // namespace apollo