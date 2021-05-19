// #include <set>

#include "modules/planning_btree/btree_planner.h"
// #include "modules/planning/common/planning_gflags.h"

namespace apollo
{
namespace planning_btree
{

BTreePlanner::BTreePlanner(const std::shared_ptr<DependencyInjector>& injector)
  : injector_(injector)
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
  context_selector_ = std::make_unique<ContextSelector>(injector_);
  context_selector_->Init();

  return Status::OK();
}

Status BTreePlanner::Execute(const TrajectoryPoint& planning_start_point,
                               BTreeFrame* frame,
                               ADCTrajectory* ptr_computed_trajectory) 
{
  DCHECK_NOTNULL(frame);

  auto current_context = context_selector_->GetCurrentContext(planning_start_point, frame);
  current_context->Execute(planning_start_point, frame);

  return Status::OK();
}

}  // namespace planning_btree
}  // namespace apollo