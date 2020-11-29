#include "modules/planning/behaviour_tree/tasks/lane_prioritizer/lane_prioritizer.h" 

namespace apollo
{
namespace planning
{ 

namespace
{
  const int kLaneChangeCost = 10;
  const int kLaneStayCost = 0;
  const int kBlockingObstacleCost = 100;
}  

  Status LanePrioritizer::Process(Frame* frame)
  {
    for (auto& ref_line : *frame->mutable_reference_line_info())
    {
      if (ref_line.IsChangeLanePath())
      {
        ref_line.AddCost(kLaneChangeCost);
      }
      else
      {
        ref_line.AddCost(kLaneStayCost);
      }

      std::string blocking_obstacle_id = ref_line.GetBlockingObstacleId();
      if (blocking_obstacle_id.size() != 0)
      {
        ref_line.AddCost(kBlockingObstacleCost);
      }
    } 

    return Status::OK();
  }

  Status LanePrioritizer::Process(Frame* frame, ReferenceLineInfo* reference_line_info)
  {
    return Process(frame);
  }

}
}