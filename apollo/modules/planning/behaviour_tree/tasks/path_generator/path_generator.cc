#include "modules/planning/behaviour_tree/tasks/path_generator/path_generator.h"

namespace apollo {
namespace planning {
namespace behaviour_tree {

  BTreeNodeState PathGenerator::Init(const BTreeNodeConfig& config)
  {
    config_ = config;
    state_ = BTreeNodeState::NODE_INITIALIZED;
    return state_;
  }

  BTreeNodeState PathGenerator::Execute(Frame* frame)
  {
    state_ = BTreeNodeState::NODE_DONE;
    return state_;
  }

  BTreeNodeState PathGenerator::Execute(Frame* frame, ReferenceLineInfo* reference_line_info)
  {
    const ReferenceLine& reference_line = reference_line_info->reference_line();
    const common::TrajectoryPoint& cur_ego_position = frame->PlanningStartPoint();
  
    std::vector<common::FrenetFramePoint> frenet_frame_path;

    double cur_s = reference_line.GetFrenetPoint(cur_ego_position.path_point()).s();
    double dt = 0.1;
    double ds = 10;
    double planning_time_horizon = 3.0;
    for (double t = 0; t < planning_time_horizon; t+=dt)
    {
      common::FrenetFramePoint frenet_frame_point;
      frenet_frame_point.set_s(t * ds + cur_s);
      frenet_frame_path.push_back(std::move(frenet_frame_point));
    }

    PathData path_data = *(reference_line_info->mutable_path_data());
    path_data.SetReferenceLine(&reference_line);
    path_data.SetFrenetPath(FrenetFramePath(frenet_frame_path));
    *(reference_line_info->mutable_path_data()) = path_data;
    
    state_ = BTreeNodeState::NODE_DONE;
    return state_;
  }

} // namespace behaviour_tree
} // namespace planning
} // namespace apollo