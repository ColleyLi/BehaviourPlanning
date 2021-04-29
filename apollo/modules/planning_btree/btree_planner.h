#pragma once

#include <memory>
#include <string>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/status/status.h"
// #include "modules/common/util/factory.h"
// #include "modules/planning/common/reference_line_info.h"
// #include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"
// #include "modules/planning/proto/planning.pb.h"
// #include "modules/planning/reference_line/reference_line.h"
// #include "modules/planning/reference_line/reference_point.h"

#include "modules/planning_btree/common/btree_frame.h"
#include "modules/planning_btree/proto/btree_planning_config.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning_btree/contexts/context_selector.h"
#include "modules/planning_btree/contexts/context.h"
// #include "modules/common/util/message_util.h"

namespace apollo {
namespace planning_btree {

using apollo::planning::ADCTrajectory;

class BTreePlanner
{
 public:
  BTreePlanner();

  ~BTreePlanner();

  void Stop();

  common::Status Init(const BTreePlanningConfig& config);

  common::Status Execute(const common::TrajectoryPoint& planning_init_point,
                      BTreeFrame* frame,
                      ADCTrajectory* ptr_computed_trajectory);
  private:
    BTreePlanningConfig config_;
    ContextSelector context_selector_;
};

}  // namespace planning_btree
}  // namespace apollo