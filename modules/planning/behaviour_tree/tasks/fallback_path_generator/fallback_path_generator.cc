#include "modules/planning/behaviour_tree/tasks/fallback_path_generator/fallback_path_generator.h" 

namespace apollo
{
namespace planning
{ 
  Status FallbackPathGenerator::Process(Frame* frame)
  {
    return Status::OK();
  }

  Status FallbackPathGenerator::Process(Frame* frame, ReferenceLineInfo* reference_line_info)
  {
    return Status::OK();
  }

}
}