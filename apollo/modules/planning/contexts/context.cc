#include "modules/planning/contexts/context.h"

namespace apollo {
namespace planning {
namespace context {

BTreeContextState Context::Init(const BTreeContextConfig& config)
{
    stage_selector_.Init(config.stage_fsm());
    parameters_ = config.parameters();
    state_ = BTreeContextState::CONTEXT_INITIALIZED;

    return state_;
}

} // namespace context
} // namespace planning
} // namespace apollo