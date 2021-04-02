#pragma once

#include "modules/planning/behaviour_tree/bt_task.h"

namespace apollo
{
namespace planning
{

using apollo::common::Status;

class SpeedGenerator: public BTTask
{
  public:
    void Init(const PlanningConfig& planning_config);
    Status Process(Frame* frame);
    Status Process(Frame* frame, ReferenceLineInfo* reference_line_info);

  private:
    Status ConstantSpeed(Frame* frame, ReferenceLineInfo* reference_line_info);
    Status GenerateBoundaries(Frame* frame, ReferenceLineInfo* reference_line_info);
    double SetSpeedFallbackDistance(PathDecision *const path_decision);
    Status OptimizeSTGraph(Frame* frame, ReferenceLineInfo* reference_line_info);

  private:
    SpeedBoundsDeciderConfig speed_bounds_config_;
    DpStSpeedConfig dp_st_speed_config_;
};

}
}