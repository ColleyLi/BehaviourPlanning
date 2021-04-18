#include "modules/planning/behaviour_tree/tasks/obstacle_processor/obstacle_processor.h" 

namespace apollo {
namespace planning {
namespace behaviour_tree {
  
  BTreeNodeState ObstacleProcessor::Init(const BTreeNodeConfig& config)
  {
    config_ = config;
    state_ = BTreeNodeState::NODE_INITIALIZED;
    return state_;
  }

  BTreeNodeState ObstacleProcessor::Execute(Frame* frame)
  {
    for (auto& ref_line : *frame->mutable_reference_line_info())
    {
      BTreeNodeState state = Execute(frame, &ref_line);
      if(!(state = BTreeNodeState::NODE_DONE))
      {
        return state;
      } 
    }

    state_ = BTreeNodeState::NODE_DONE;
    return state_;
  }

  BTreeNodeState ObstacleProcessor::Execute(Frame* frame, ReferenceLineInfo* reference_line_info)
  {
    state_ = BTreeNodeState::NODE_DONE;
    return state_;
  }

} // namespace behaviour_tree
} // namespace planning
} // namespace apollo