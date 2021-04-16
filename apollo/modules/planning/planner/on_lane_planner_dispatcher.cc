#include "modules/planning/planner/on_lane_planner_dispatcher.h"
#include "cyber/common/file.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/planning_config.pb.h"

namespace apollo
{
namespace planning 
{

std::unique_ptr<Planner> OnLanePlannerDispatcher::DispatchPlanner() 
{
  PlanningConfig planning_config;
  bool res_load_config = apollo::cyber::common::GetProtoFromFile(FLAGS_planning_config_file, &planning_config);
  if (!res_load_config) 
  {
    return nullptr;
  }

  // BTree planner dispatch
  if (planning_config.has_btree_planning_config())
  {
    return planner_factory_.CreateObject(planning_config.btree_planning_config().planner_type());
  }

  // Standard planning config dispatch
  return planner_factory_.CreateObject(planning_config.standard_planning_config().planner_type(0));
}

}  // namespace planning
}  // namespace apollo
