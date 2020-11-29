#pragma once

#include "modules/planning/behaviour_tree/bt_task.h"

namespace apollo
{
namespace planning
{

using apollo::common::Status;

class LanePrioritizer: public BTTask
{
    Status Process(Frame* frame);
    Status Process(Frame* frame, ReferenceLineInfo* reference_line_info);
};

}
}