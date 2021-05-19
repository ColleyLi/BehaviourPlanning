#include <memory>
#include "modules/planning_btree/contexts/context_dispatcher.h"
#include "modules/planning_btree/contexts/lane_follow/lane_follow_context.h"

namespace apollo {
namespace planning_btree {

ContextDispatcher::ContextDispatcher(const std::shared_ptr<DependencyInjector>& injector)
    :injector_(injector)
{

}

bool ContextDispatcher::Init()
{
    context_factory_.Clear();
    RegisterContexts();

    return true;
}

void ContextDispatcher::RegisterContexts() 
{
    context_factory_.Register(BTreeContextType::LANE_FOLLOW_CONTEXT, [](const std::shared_ptr<DependencyInjector>& injector) -> Context* { return new LaneFollowContext(injector);});
}

std::shared_ptr<Context> ContextDispatcher::Dispatch(const BTreeContextType& context_type)
{
    return context_factory_.CreateObject(context_type, injector_);
}

} // namespace planning_btree
} // namespace apollo