#include <memory>
#include "modules/planning/behaviour_tree/b_tree_node_dispatcher.h"
#include "modules/planning/behaviour_tree/b_tree_sequence.h"
#include "modules/planning/behaviour_tree/b_tree_selector.h"
#include "modules/planning/behaviour_tree/tasks/obstacle_processor/obstacle_processor.h"
#include "modules/planning/behaviour_tree/tasks/path_generator/path_generator.h"
#include "modules/planning/behaviour_tree/tasks/speed_generator/speed_generator.h"
#include "modules/planning/behaviour_tree/tasks/lane_prioritizer/lane_prioritizer.h"
#include "modules/planning/behaviour_tree/tasks/fallback_path_generator/fallback_path_generator.h"
#include "modules/planning/behaviour_tree/tasks/fallback_speed_generator/fallback_speed_generator.h"
#include "modules/planning/behaviour_tree/priority_selectors/lane_priority_selector.h"
#include "modules/planning/behaviour_tree/checks/collision_check.h"
#include "modules/planning/behaviour_tree/checks/safe_lane_change_check.h"

namespace apollo {
namespace planning {
namespace behaviour_tree {


bool BTreeNodeDispatcher::Init()
{
    RegisterNodes();

    return true;
}

void BTreeNodeDispatcher::RegisterNodes() 
{
    node_factory_.Register(BTreeNodeType::SEQUENCE, []() -> BTreeNode* { return new BTreeSequence();});
    
    node_factory_.Register(BTreeNodeType::SELECTOR, []() -> BTreeNode* { return new BTreeSelector();});
    
    node_factory_.Register(BTreeNodeType::OBSTACLE_PROCESSOR_TASK, []() -> BTreeNode* { return new ObstacleProcessor();});
    
    node_factory_.Register(BTreeNodeType::PATH_GENERATOR_TASK, []() -> BTreeNode* { return new PathGenerator();});
    
    node_factory_.Register(BTreeNodeType::SPEED_GENERATOR_TASK, []() -> BTreeNode* { return new SpeedGenerator();});
    
    node_factory_.Register(BTreeNodeType::LANE_PRIORITIZER_TASK, []() -> BTreeNode* { return new LanePrioritizer();});
    
    node_factory_.Register(BTreeNodeType::FALLBACK_PATH_GENERATOR_TASK, []() -> BTreeNode* { return new FallbackPathGenerator();});

    node_factory_.Register(BTreeNodeType::FALLBACK_SPEED_GENERATOR_TASK, []() -> BTreeNode* { return new FallbackSpeedGenerator();});
    
    node_factory_.Register(BTreeNodeType::LANE_PRIORITY_SELECTOR, []() -> BTreeNode* { return new LanePrioritySelector();});
    
    node_factory_.Register(BTreeNodeType::COLLISION_CHECK, []() -> BTreeNode* { return new CollisionCheck();});
    
    node_factory_.Register(BTreeNodeType::SAFE_LANE_CHANGE_CHECK, []() -> BTreeNode* { return new SafeLaneChangeCheck();});
}

std::shared_ptr<BTreeNode> BTreeNodeDispatcher::Dispatch(const BTreeNodeType& node_type)
{
    return node_factory_.CreateObject(node_type);
}

} // namespace behaviour_tree
} // namespace planning
} // namespace apollo