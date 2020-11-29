#pragma once

#include "modules/common/status/status.h"
#include "modules/planning/common/frame.h"

namespace apollo
{    
namespace planning
{

class BTNode
{
  public:
    BTNode() {}

    BTNode(int id):
      id_(id), name_("Unknown")
    {}

    BTNode (std::string name):
      id_(0), name_(name)  
    {}

    BTNode(int id, std::string name):
      id_(id), name_(name)
    {}

    void SetId(int id)
    {
      id_ = id;
    }

    void SetName(std::string name)
    {
      name_ = name;
    }

    int GetId()
    {
      return id_;
    }

    std::string GetName()
    {
      return name_;
    }

    virtual apollo::common::Status Process(Frame* frame) = 0;
    virtual apollo::common::Status Process(Frame* frame, ReferenceLineInfo* reference_line_info) = 0;

  private:
    int id_;
    std::string name_;
};
}
}