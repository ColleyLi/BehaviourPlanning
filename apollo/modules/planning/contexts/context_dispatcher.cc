#include <memory>
#include "modules/planning/contexts/context_dispatcher.h"
#include "modules/planning/contexts/lane_follow/lane_follow_context.h"

namespace apollo {
namespace planning {
namespace context {

common::Status ContextDispatcher::Init()
{
    RegisterContexts();

    return common::Status::OK();
}

void ContextDispatcher::RegisterContexts() 
{
    context_factory_.Register(BTreeContextName::BTREE_LANE_FOLLOW_CONTEXT, []() -> Context* { return new LaneFollowContext();});
}

std::shared_ptr<Context> ContextDispatcher::Dispatch(const BTreeContextName& context_name)
{
    return context_factory_.CreateObject(context_name);
}

} // namespace apollo
} // namespace planning
} // namespace context