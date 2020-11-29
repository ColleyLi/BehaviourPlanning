#include "modules/planning/behaviour_tree/tasks/obstacle_processor/obstacle_processor.h" 

namespace apollo
{
namespace planning
{ 
  Status ObstacleProcessor::Process(Frame* frame)
  {
    Status status;
    for (auto& ref_line : *frame->mutable_reference_line_info())
    {
      status = Process(frame, &ref_line);
      if(!status.ok())
      {
        return status;
      } 
    }

    return Status::OK();
  }

  Status ObstacleProcessor::Process(Frame* frame, ReferenceLineInfo* reference_line_info)
  {
    

    return Status::OK();
  }

}
}