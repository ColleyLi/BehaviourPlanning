
#include "modules/planning/contexts/context_selector.h"
#include "modules/planning/proto/b_tree_context_config.pb.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {
namespace context {

bool ContextSelector::Init(const std::set<BTreeContextName>& contexts_to_use)
{
    context_dispatcher_.Init();

    BTreeContextConfigs context_configs;
    apollo::cyber::common::GetProtoFromFile(FLAGS_b_tree_context_config_file, &context_configs);

    for (auto it = contexts_to_use.begin(); it != contexts_to_use.end(); ++it) 
    {
        auto context_name = *it;
        auto context = context_dispatcher_.Dispatch(context_name);
        
        // TODO: Speed-up config lookup
        for (int i = 0; i < context_configs.b_tree_context_config_size(); ++i)
        {
            auto context_config = context_configs.b_tree_context_config(i);
            if (context_config.name() == context_name)
            {
                context->Init(context_config);
                break;
            } 
        }

        contexts_[context_name] = context;
    }

    return true;
}

std::shared_ptr<Context> ContextSelector::GetCurrentContext(const TrajectoryPoint& planning_start_point, Frame* const frame)
{
    // TODO: Context selection logic
    auto current_context = contexts_[BTreeContextName::BTREE_LANE_FOLLOW_CONTEXT]; 
    return current_context;
}

}
}
}