#include "modules/planning/tasks/task_factory.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/common/status/status.h"
#include "modules/planning/tasks/my_tasks/my_task/my_task.h"
#include "modules/planning/tasks/task.h"

namespace apollo 
{
namespace planning 
{

apollo::common::util::Factory<
    TaskConfig::TaskType, Task, Task* (*)(const TaskConfig& config),
    std::unordered_map<TaskConfig::TaskType,
                       Task* (*)(const TaskConfig& config), std::hash<int>>>
    TaskFactory::task_factory_;

std::unordered_map<TaskConfig::TaskType, TaskConfig, std::hash<int>>
    TaskFactory::default_task_configs_;

void TaskFactory::Init(const PlanningConfig& config) 
{
  task_factory_.Register(TaskConfig::MY_TASK, [](const TaskConfig& config) -> Task*
                       {
                         return new MyTask(config);
                       });

  for (const auto& default_task_config : config.default_task_config()) 
  {
    default_task_configs_[default_task_config.task_type()] = default_task_config;
  }
}

std::unique_ptr<Task> TaskFactory::CreateTask(const TaskConfig& task_config) 
{
  TaskConfig merged_config;
  if (default_task_configs_.find(task_config.task_type()) != default_task_configs_.end()) 
  {
    merged_config = default_task_configs_[task_config.task_type()];
  }
  merged_config.MergeFrom(task_config);
  return task_factory_.CreateObject(task_config.task_type(), merged_config);
}

}  // namespace planning
}  // namespace apollo
