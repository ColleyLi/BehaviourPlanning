#include <set>

#include "modules/planning/planner/btree_planner/btree_planner.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo
{
namespace planning 
{

using common::Status;
using common::TrajectoryPoint;

Status BTreePlanner::Init(const PlanningConfig& config) 
{
  config_ = config;

  std::set<BTreeContextName> config_contexts;

  for (int i = 0; i < config_.btree_planning_config().context_to_use_size(); ++i) 
  {
    const BTreeContextName context = config_.btree_planning_config().context_to_use(i); 
    config_contexts.insert(context);
  }

  context_selector_.Init(config_contexts);

  return Status::OK();
}

Status BTreePlanner::Plan(const TrajectoryPoint& planning_start_point,
                               Frame* frame,
                               ADCTrajectory* ptr_computed_trajectory) 
{
  DCHECK_NOTNULL(frame);

  AERROR << "Running BTree planner";

  auto current_context = context_selector_.GetCurrentContext(planning_start_point, frame);

  current_context->Execute(planning_start_point, frame);

  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
