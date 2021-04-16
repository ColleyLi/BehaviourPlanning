#pragma once

#include <string>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/planner/planner.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_point.h"

#include "modules/planning/contexts/context_selector.h"
#include "modules/planning/contexts/context.h"

namespace apollo 
{
namespace planning 
{

class BTreePlanner : public Planner 
{
  public:
    BTreePlanner() = default;
    virtual ~BTreePlanner() = default;

    virtual void Stop() 
    {
    }

    std::string Name() override 
    { 
      return "BTREE_PLANNER";
    }

    common::Status Init(const PlanningConfig& config) override;

    common::Status Plan(const common::TrajectoryPoint& planning_init_point,
                        Frame* frame,
                        ADCTrajectory* ptr_computed_trajectory) override;

  private:
    context::ContextSelector context_selector_;
};

}  // namespace planning
}  // namespace apollo
