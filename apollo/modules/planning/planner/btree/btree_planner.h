#pragma once

#include <memory>
#include <string>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"
#include "modules/planning/planner/planner.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_point.h"

#include "modules/planning/contexts/context_selector.h"
#include "modules/planning/contexts/context.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class BTreePlanner
 * @brief BTreePlanner is a behaviour tree planner
 */

class BTreePlanner : public PlannerWithReferenceLine {
 public:
  /**
   * @brief Constructor
   */
  BTreePlanner() = delete;

  explicit BTreePlanner(
      const std::shared_ptr<DependencyInjector>& injector)
      : PlannerWithReferenceLine(injector) {}

  /**
   * @brief Destructor
   */
  virtual ~BTreePlanner() = default;

  void Stop() override {}

  std::string Name() override { return "BTREE"; }

  common::Status Init(const PlanningConfig& config) override;

  /**
   * @brief Override function Plan in parent class Planner.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @return OK if planning succeeds; error otherwise.
   */
  common::Status Plan(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame,
                      ADCTrajectory* ptr_computed_trajectory) override;
  private:
    PlanningConfig config_;
    context::ContextSelector context_selector_;
};

}  // namespace planning
}  // namespace apollo
