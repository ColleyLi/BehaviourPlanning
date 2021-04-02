#include "modules/planning/behaviour_tree/tasks/fallback_speed_generator/fallback_speed_generator.h" 

namespace apollo
{
namespace planning
{ 
  Status FallbackSpeedGenerator::Process(Frame* frame)
  {
    return Status::OK();
  }

  Status FallbackSpeedGenerator::Process(Frame* frame, ReferenceLineInfo* reference_line_info)
  {
    return Status::OK();
  }

}
}