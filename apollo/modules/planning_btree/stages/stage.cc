#include "modules/planning_btree/stages/stage.h"

namespace apollo {
namespace planning_btree {

BTreeStageState Stage::Init(const BTreeStageConfig& config)
{
    node_dispatcher_.Init(); 

    parameters_ = config.parameters();

    auto tree_config = config.tree();
    for(int i = 0; i < tree_config.node_size(); ++i)
    {
        auto node = tree_config.node(i);
        nodes_[node.id()] = node_dispatcher_.Dispatch(node.type());
        nodes_[node.id()]->SetId(node.id());
        nodes_[node.id()]->SetName(node.name());
        nodes_[node.id()]->Init(node.config());
    }

    for(int i = 0; i < tree_config.node_size(); ++i)
    {
        auto node = tree_config.node(i);
        for(int j = 0; j < node.child_id_size(); ++j)
        {
            nodes_[node.id()]->AddChild(nodes_[node.child_id(j)]);
        }
    }

    behaviour_tree_ = nodes_[tree_config.root_node_id()];

    state_ = BTreeStageState::STAGE_INITIALIZED;
    return state_;
}


BTreeStageState Stage::GetState()
{
    return state_;
}

} // namespace planning_btree
} // namespace apollo