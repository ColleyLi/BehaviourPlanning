#include <memory>
#include "modules/planning/contexts/context_dispatcher.h"
#include "modules/planning/contexts/lane_follow/lane_follow_context.h"

namespace apollo {
namespace planning {
namespace context {

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

} // namespace context
} // namespace planning
} // namespace apollo