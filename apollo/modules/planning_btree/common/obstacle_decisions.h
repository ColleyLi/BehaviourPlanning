#pragma once

#include <limits>
#include <string>

#include "modules/planning_btree/common/utils/indexed_list.h"
#include "modules/planning_btree/common/obstacle.h"
#include "modules/planning_btree/proto/decision.pb.h"

namespace apollo {
namespace planning_btree {

/**
 * @class ObstacleDecisions
 *
 * @brief ObstacleDecisions represents all obstacle decisions on one path.
 */
class ObstacleDecisions {
 public:
  ObstacleDecisions() = default;

  Obstacle *AddObstacle(const Obstacle &obstacle);

  const IndexedList<std::string, Obstacle> &obstacles() const;

  bool AddLateralDecision(const std::string &tag, const std::string &object_id,
                          const ObjectDecisionType &decision);
  bool AddLongitudinalDecision(const std::string &tag,
                               const std::string &object_id,
                               const ObjectDecisionType &decision);

  const Obstacle *Find(const std::string &object_id) const;

  const perception::PerceptionObstacle *FindPerceptionObstacle(
      const std::string &perception_obstacle_id) const;

  Obstacle *Find(const std::string &object_id);

  void SetSTBoundary(const std::string &id, const STBoundary &boundary);
  void EraseStBoundaries();
  MainStop main_stop() const { return main_stop_; }
  double stop_reference_line_s() const { return stop_reference_line_s_; }
  bool MergeWithMainStop(const ObjectStop &obj_stop, const std::string &obj_id,
                         const ReferenceLine &ref_line,
                         const SLBoundary &adc_sl_boundary);

 private:
  IndexedList<std::string, Obstacle> obstacles_;
  MainStop main_stop_;
  double stop_reference_line_s_ = std::numeric_limits<double>::max();
};

}  // namespace planning_btree
}  // namespace apollo
