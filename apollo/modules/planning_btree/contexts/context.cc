#include "modules/planning_btree/contexts/context.h"

namespace apollo {
namespace planning_btree {

Context::Context(const std::shared_ptr<DependencyInjector>& injector)
    :injector_(injector)
{

}

BTreeContextState Context::Init(const BTreeContextConfig& config)
{
    stage_selector_ = std::make_unique<StageSelector>(injector_);
    stage_selector_->Init(config.stage_fsm());
    parameters_ = config.parameters();
    state_ = BTreeContextState::CONTEXT_INITIALIZED;

    return state_;
}

} // namespace planning_btree
} // namespace apollo