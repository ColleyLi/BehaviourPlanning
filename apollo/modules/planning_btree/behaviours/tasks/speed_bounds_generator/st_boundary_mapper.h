#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/planning_btree/common/obstacle.h"
#include "modules/planning_btree/common/path/path_data.h"
#include "modules/planning_btree/common/obstacle_decisions.h"
#include "modules/planning_btree/common/speed/st_boundary.h"
#include "modules/planning_btree/common/speed/speed_limit.h"
#include "modules/planning_btree/reference_line/reference_line.h"

namespace apollo {
namespace planning_btree {

class STBoundaryMapper {
 public:
  STBoundaryMapper(const ReferenceLine& reference_line,
                   const PathData& path_data, const double planning_distance,
                   const double planning_time);

  virtual ~STBoundaryMapper() = default;

  bool ComputeSTBoundary(ObstacleDecisions* obstacle_decisions) const;

 private:
  /** @brief Calls GetOverlapBoundaryPoints to get upper and lower points
   * for a given obstacle, and then formulate STBoundary based on that.
   * It also labels boundary type based on previously documented decisions.
   */
  void ComputeSTBoundary(Obstacle* obstacle) const;

  /** @brief Map the given obstacle onto the ST-Graph. The boundary is
   * represented as upper and lower points for every s of interests.
   * Note that upper_points.size() = lower_points.size()
   */
  bool GetOverlapBoundaryPoints(
      const std::vector<common::PathPoint>& path_points,
      const Obstacle& obstacle, std::vector<STPoint>* upper_points,
      std::vector<STPoint>* lower_points) const;

  /** @brief Given a path-point and an obstacle bounding box, check if the
   *        ADC, when at that path-point, will collide with the obstacle.
   * @param The path-point of the center of rear-axis for ADC.
   * @param The bounding box of the obstacle.
   * @param The extra lateral buffer for our ADC.
   */
  bool CheckOverlap(const common::PathPoint& path_point,
                    const common::math::Box2d& obs_box,
                    const double l_buffer) const;

  /** @brief Maps the closest STOP decision onto the ST-graph. This STOP
   * decision can be stopping for blocking obstacles, or can be due to
   * traffic rules, etc.
   */
  bool MapStopDecision(Obstacle* stop_obstacle,
                       const ObjectDecisionType& decision) const;

  /** @brief Fine-tune the boundary for yielding or overtaking obstacles.
   * Increase boundary on the s-dimension or set the boundary type, etc.,
   * when necessary.
   */
  void ComputeSTBoundaryWithDecision(Obstacle* obstacle,
                                     const ObjectDecisionType& decision) const;

 private:
  const ReferenceLine& reference_line_;
  const PathData& path_data_;
  const common::VehicleParam& vehicle_param_;
  const double planning_max_distance_;
  const double planning_max_time_;
};

}  // namespace planning
}  // namespace apollo
