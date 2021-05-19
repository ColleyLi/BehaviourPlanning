#pragma once

#include "modules/common/proto/drive_state.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/common/configs/vehicle_config_helper.h"

#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning_btree/common/path/path_boundary.h"
#include "modules/planning_btree/common/path/path_data.h"
#include "modules/planning_btree/common/speed/speed_data.h"
#include "modules/planning_btree/common/speed/st_graph_data.h"
#include "modules/planning_btree/common/obstacle.h"
#include "modules/planning_btree/common/obstacle_decisions.h"
#include "modules/planning_btree/common/trajectory/discretized_trajectory.h"
#include "modules/planning_btree/reference_line/reference_line.h"

namespace apollo {
namespace planning_btree {

class DynamicReferenceLine {
 public:
  enum class LaneType { LeftForward, LeftReverse, RightForward, RightReverse };
  
  DynamicReferenceLine() = default;

  DynamicReferenceLine(const common::VehicleState& vehicle_state,
                       const common::TrajectoryPoint& adc_planning_point,
                       const ReferenceLine& reference_line,
                       const hdmap::RouteSegments& route_segments);

  bool Init(const std::vector<const Obstacle*>& obstacles);
  
  const common::VehicleState& GetADCState() const { return vehicle_state_; }
  const SLBoundary& GetADCSLBoundary() const;
  
  ObstacleDecisions* GetMutableObstacleDecisions();
  const ObstacleDecisions& GetObstacleDecisions() const;

  Obstacle* GetBlockingObstacle() const;
  void SetBlockingObstacle(const std::string& blocking_obstacle_id);
  
  bool AddObstacles(const std::vector<const Obstacle*>& obstacles);
  Obstacle* AddObstacle(const Obstacle* obstacle);
  
  const ReferenceLine& GetReferenceLine() const;
  ReferenceLine* GetMutableReferenceLine();

  const std::vector<PathBoundary>& GetCandidatePathBoundaries() const;
  void SetCandidatePathBoundaries(std::vector<PathBoundary>&& path_boundaries);

  const std::vector<PathData>& GetCandidatePathData() const;
  void SetCandidatePathData(std::vector<PathData>&& path_data);

  const PathData& GetPathData() const;
  PathData* GetMutablePathData();

  const SpeedData& GetSpeedData() const;
  SpeedData* GetMutableSpeedData();
  
  StGraphData* GetMutableSTGraphData() { return &st_graph_data_; }

  const StGraphData& GetSTGraphData() { return st_graph_data_; }

  const double GetCost() const;
  void SetCost(double cost);
  void AddCost(double cost);

  bool IsDrivable() const;
  void SetDrivable(bool drivable);

  bool IsLaneChangePath() const;

  const hdmap::RouteSegments& GetRouteSegments() const;

  const DiscretizedTrajectory& GetDiscretizedTrajectory() const;
  void SetDiscretizedTrajectory(const DiscretizedTrajectory& trajectory);

  bool CombinePathAndSpeedProfiles(
      const double relative_time, const double start_s,
      DiscretizedTrajectory* discretized_trajectory);

  hdmap::LaneInfoConstPtr LocateLaneInfo(const double s) const;

  bool GetNeighborLaneInfo(const DynamicReferenceLine::LaneType lane_type,
                           const double s, hdmap::Id* ptr_lane_id,
                           double* ptr_lane_width) const;

  double SDistanceToDestination() const;
  bool ReachedDestination() const;
 private:
  bool IsIrrelevantObstacle(const Obstacle& obstacle);

 private:
  const common::VehicleState vehicle_state_;
  const common::TrajectoryPoint adc_planning_point_;
  SLBoundary adc_sl_boundary_;
  ReferenceLine reference_line_;
  hdmap::RouteSegments route_segments_;

  std::vector<PathBoundary> candidate_path_boundaries_;
  std::vector<PathData> candidate_path_data_;

  PathData path_data_;

  StGraphData st_graph_data_;
  SpeedData speed_data_;

  DiscretizedTrajectory discretized_trajectory_;

  ObstacleDecisions obstacle_decisions_;
  Obstacle* blocking_obstacle_ = nullptr;

  double cost_;
  bool is_drivable_;
  bool is_on_reference_line_;
};

}  // namespace planning_btree
}  // namespace apollo