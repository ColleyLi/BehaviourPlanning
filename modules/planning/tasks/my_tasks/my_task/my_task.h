#pragma once

#include "modules/planning/tasks/task.h"

namespace apollo 
{
namespace planning 
{

class MyTask : public Task 
{
 public:
  explicit MyTask(const TaskConfig &config);

  apollo::common::Status Execute(Frame *frame, ReferenceLineInfo *reference_line_info) override;

 private:
  apollo::common::Status Process(Frame *frame, ReferenceLineInfo *reference_line_info);
};

}  // namespace planning:
}  // namespace apollo
