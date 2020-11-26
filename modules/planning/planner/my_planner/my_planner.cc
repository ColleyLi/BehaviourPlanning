#include "modules/planning/planner/my_planner/my_planner.h"

#include <set>

#include "modules/planning/common/planning_gflags.h"

namespace apollo
{
namespace planning 
{

using common::Status;
using common::TrajectoryPoint;

Status MyPlanner::Init(const PlanningConfig& config) 
{
  config_ = config;

  // Extract scenarios from config
  std::set<ScenarioConfig::ScenarioType> supported_scenarios;
  const auto& config_scenarios = 
    config_.standard_planning_config().planner_scenarios();

  // Load scenarios
  for (int i = 0; i < config_scenarios.scenario_type_size(); ++i) 
  {
    const ScenarioConfig::ScenarioType scenario = config_scenarios.scenario_type(i);
    supported_scenarios.insert(scenario);
  }

  scenario_manager_.Init(supported_scenarios);

  return Status::OK();
}

Status MyPlanner::Plan(const TrajectoryPoint& planning_start_point,
                               Frame* frame,
                               ADCTrajectory* ptr_computed_trajectory) 
{
  DCHECK_NOTNULL(frame);

  // Update scenario manager with recent data. It will update mutable_scenario
  scenario_manager_.Update(planning_start_point, *frame);

  // Extract current scenario and run it
  scenario_ = scenario_manager_.mutable_scenario();
  auto result = scenario_->Process(planning_start_point, frame);

  /*
  // Debug info logging
  if (FLAGS_enable_record_debug) 
  {
    auto scenario_debug = ptr_computed_trajectory->mutable_debug()
                              ->mutable_planning_data()
                              ->mutable_scenario();
    scenario_debug->set_scenario_type(scenario_->scenario_type());
    scenario_debug->set_stage_type(scenario_->GetStage());
    scenario_debug->set_msg(scenario_->GetMsg());
  }
  */

  // Check if scenario was executed sucessfully
  if (result == scenario::Scenario::STATUS_DONE) 
  {
    // Only updates scenario manager when previous scenario's status is
    // STATUS_DONE
    scenario_manager_.Update(planning_start_point, *frame);
  } 
  else if (result == scenario::Scenario::STATUS_UNKNOWN) 
  {
    return Status(common::PLANNING_ERROR, "scenario returned unknown");
  }

  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
