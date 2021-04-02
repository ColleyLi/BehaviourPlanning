#pragma once

#include "modules/planning/behaviour_tree/bt_composite_node.h"

namespace apollo
{    
namespace planning
{

using common::Status;
using common::ErrorCode;

class BTSelector: public BTCompositeNode
{
  public:
    virtual Status Process(Frame* frame) override
    {
      for(BTNode* child : GetChildren())
      {
        if(child->Process(frame).ok())
        {   
          std::string msg("Selected task: ");
          msg += child->GetName();
          ADEBUG << msg;
          return Status::OK();
        }
      }
      std::string msg("All tasks failed");
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    virtual Status Process(Frame* frame, ReferenceLineInfo* reference_line_info) override
    {
      for(BTNode* child : GetChildren())
      {
        if(!child->Process(frame, reference_line_info).ok())
        {
          std::string msg("Selected task: ");
          msg += child->GetName();
          ADEBUG << msg;
          return Status::OK();
        }
      }

      std::string msg("All tasks failed");
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

};

}
}