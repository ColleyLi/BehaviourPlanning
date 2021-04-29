// #include <set>

#include "modules/planning_btree/btree_planner.h"
// #include "modules/planning/common/planning_gflags.h"

namespace apollo
{
namespace planning_btree
{

BTreePlanner::BTreePlanner()
{

}

BTreePlanner::~BTreePlanner()
{

}

using common::Status;
using common::TrajectoryPoint;

Status BTreePlanner::Init(const BTreePlanningConfig& config) 
{
  config_ = config;

  std::set<BTreeContextType> config_contexts;
  for (int i = 0; i < config_.context_to_use_size(); ++i) 
  {
    const BTreeContextType context = config_.context_to_use(i); 
    config_contexts.insert(context);
  }

  context_selector_.Init(config_contexts);

  return Status::OK();
}

Status BTreePlanner::Execute(const TrajectoryPoint& planning_start_point,
                               BTreeFrame* frame,
                               ADCTrajectory* ptr_computed_trajectory) 
{
  DCHECK_NOTNULL(frame);

  AERROR << "Running BTree planner";

  auto current_context = context_selector_.GetCurrentContext(planning_start_point, frame);

  current_context->Execute(planning_start_point, frame);

  return Status::OK();
}

}  // namespace planning_btree
}  // namespace apollo