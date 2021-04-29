#include "modules/planning_btree/contexts/context.h"

namespace apollo {
namespace planning_btree {

BTreeContextState Context::Init(const BTreeContextConfig& config)
{
    stage_selector_.Init(config.stage_fsm());
    parameters_ = config.parameters();
    state_ = BTreeContextState::CONTEXT_INITIALIZED;

    return state_;
}

} // namespace planning_btree
} // namespace apollo