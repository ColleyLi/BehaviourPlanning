#include <memory>
#include "modules/planning_btree/contexts/context_dispatcher.h"
#include "modules/planning_btree/contexts/lane_follow/lane_follow_context.h"

namespace apollo {
namespace planning_btree {

bool ContextDispatcher::Init()
{
    RegisterContexts();

    return true;
}

void ContextDispatcher::RegisterContexts() 
{
    context_factory_.Register(BTreeContextType::LANE_FOLLOW_CONTEXT, []() -> Context* { return new LaneFollowContext();});
}

std::shared_ptr<Context> ContextDispatcher::Dispatch(const BTreeContextType& context_type)
{
    return context_factory_.CreateObject(context_type);
}

} // namespace planning_btree
} // namespace apollo