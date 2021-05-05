#include "modules/planning_btree/contexts/context_selector.h"
#include "modules/planning_btree/proto/btree_context_config.pb.h"
#include "modules/planning_btree/common/btree_planning_gflags.h"

namespace apollo {
namespace planning_btree {

ContextSelector::ContextSelector(const std::shared_ptr<DependencyInjector>& injector)
    : injector_(injector)
{

}

bool ContextSelector::Init(const std::set<BTreeContextType>& contexts_to_use)
{
    context_dispatcher_ = std::make_unique<ContextDispatcher>(injector_);
    context_dispatcher_->Init();
    auto context_configs = injector_->planning_state()->btplan().context_configs();

    for (auto it = contexts_to_use.begin(); it != contexts_to_use.end(); ++it) 
    {
        auto context_type = *it;
        auto context = context_dispatcher_->Dispatch(context_type);
        
        // TODO: Speed-up config lookup
        for (int i = 0; i < context_configs.context_config_size(); ++i)
        {
            auto context_config = context_configs.context_config(i);
            if (context_config.type() == context_type)
            {
                context->Init(context_config);
                break;
            } 
        }

        contexts_[context_type] = context;
    }

    return true;
}

std::shared_ptr<Context> ContextSelector::GetCurrentContext(const TrajectoryPoint& planning_start_point, BTreeFrame* const frame)
{
    // TODO: Context selection logic
    auto current_context = contexts_[BTreeContextType::LANE_FOLLOW_CONTEXT]; 
    return current_context;
}

} // namespace planning_btree
} // namespace apollo