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

    state_ = BTreeNodeState::NODE_DONE;
    return state_;
  }

} // namespace planning_btree
} // namespace apollo