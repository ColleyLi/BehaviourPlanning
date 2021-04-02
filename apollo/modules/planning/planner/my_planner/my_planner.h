#pragma once

#include <string>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"
#include "modules/planning/planner/planner.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_point.h"

namespace apollo 
{
namespace planning 
{

/**
 * @class MyPlanner
 * @brief MyPlanner implementation
 */

class MyPlanner : public PlannerWithReferenceLine 
{
  public:
    MyPlanner() = default;
    virtual ~MyPlanner() = default;

    virtual void Stop() 
    {
    }

    std::string Name() override 
    { 
      return "MY_PLANNER";
    }

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
};

}  // namespace planning
}  // namespace apollo
