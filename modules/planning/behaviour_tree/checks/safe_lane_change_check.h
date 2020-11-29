#pragma once

#include "modules/planning/behaviour_tree/bt_node.h"

namespace apollo
{
namespace planning
{

using apollo::common::Status;

class SafeLaneChangeCheck: public BTNode
{
  public:
    Status Process(Frame* frame);
    Status Process(Frame* frame, ReferenceLineInfo* reference_line_info);
};
}
}