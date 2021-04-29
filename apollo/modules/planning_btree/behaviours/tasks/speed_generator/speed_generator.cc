#include "modules/planning_btree/behaviours/tasks/speed_generator/speed_generator.h"

namespace apollo {
namespace planning_btree {

BTreeNodeState SpeedGenerator::Init(const BTreeNodeConfig& config)
{
  config_ = config;

  state_ = BTreeNodeState::NODE_INITIALIZED;
  return state_;
}

BTreeNodeState SpeedGenerator::Execute(BTreeFrame *frame)
{
  state_ = BTreeNodeState::NODE_DONE;
  return state_;
}

// BTreeNodeState SpeedGenerator::Execute(BTreeFrame* frame, ReferenceLineInfo* reference_line_info)
// {
//   state_ = ConstantSpeed(frame, reference_line_info);

//   if (state_ == BTreeNodeState::NODE_DONE)
//   {
//     reference_line_info->SetDrivable(true);
//   }

//   return state_;
// }

// BTreeNodeState SpeedGenerator::ConstantSpeed(BTreeFrame* frame, ReferenceLineInfo* reference_line_info)
// {
//   SpeedData speed_data;

//   double dt = 0.1;
//   double v = 6.0;
//   double planning_time_horizon = 3.0;
//   for (double t = 0; t < planning_time_horizon; t+=dt)
//   {
//     speed_data.AppendSpeedPoint(t * v, t, v, 0.0, 0.0);
//   }

//   *(reference_line_info->mutable_speed_data()) = speed_data;

//   state_ = BTreeNodeState::NODE_DONE;
//   return state_;
// }

} // namespace planning_btree
} // namespace apollo