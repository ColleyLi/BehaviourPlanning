#include "modules/planning_btree/behaviours/tasks/obstacle_processor/obstacle_processor.h" 

namespace apollo {
namespace planning_btree {
  
  BTreeNodeState ObstacleProcessor::Init(const BTreeNodeConfig& config)
  {
    config_ = config;
    state_ = BTreeNodeState::NODE_INITIALIZED;
    return state_;
  }

  BTreeNodeState ObstacleProcessor::Execute(BTreeFrame* frame)
  {
    // for (auto& ref_line : *frame->mutable_reference_line_info())
    // {
    //   BTreeNodeState state = Execute(frame, &ref_line);
    //   if(!(state = BTreeNodeState::NODE_DONE))
    //   {
    //     return state;
    //   } 
    // }

    state_ = BTreeNodeState::NODE_DONE;
    return state_;
  }

  // BTreeNodeState ObstacleProcessor::Execute(BTreeFrame* frame, ReferenceLineInfo* reference_line_info)
  // {
  //   state_ = BTreeNodeState::NODE_DONE;
  //   return state_;
  // }

} // namespace planning_btree
} // namespace apollo