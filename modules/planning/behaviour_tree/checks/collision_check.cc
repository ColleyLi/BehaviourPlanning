#include "modules/planning/behaviour_tree/checks/collision_check.h"

namespace apollo
{
namespace planning
{

using apollo::common::Status;
using apollo::common::ErrorCode;

Status CollisionCheck::Process(Frame* frame, ReferenceLineInfo* reference_line_info)
{
    return Process(frame);
}

Status CollisionCheck::Process(Frame* frame)
{
    return Status(ErrorCode::PLANNING_ERROR, "No collision");
}

}
}
