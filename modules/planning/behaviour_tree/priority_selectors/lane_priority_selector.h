#include "modules/planning/behaviour_tree/bt_composite_node.h"

namespace apollo
{    
namespace planning
{

using common::Status;
using common::ErrorCode;

class LanePrioritySelector: public BTCompositeNode
{
  public:
    Status Process(Frame* frame, ReferenceLineInfo* reference_line_info)
    {
      return Process(frame);
    }

    Status Process(Frame* frame)
    {
        std::list<ReferenceLineInfo>* ref_lines = frame->mutable_reference_line_info();
        ref_lines->sort([](const ReferenceLineInfo& l1,
                     const ReferenceLineInfo& l2) 
                  { return l1.Cost() < l2.Cost();});

      for(auto& ref_line : *frame->mutable_reference_line_info())
      {
        for(BTNode* child : GetChildren())
        {
          if(child->Process(frame, &ref_line).ok())
          {
            return Status::OK();
          }
        }
      }

      std::string msg("All tasks failed");
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
};

}
}