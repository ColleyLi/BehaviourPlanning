#pragma once

#include "modules/planning_btree/common/btree_frame.h"
#include "modules/planning_btree/stages/stage_selector.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning_btree/proto/btree_context_config.pb.h"
#include "modules/planning_btree/common/dependency_injector.h"

namespace apollo {
namespace planning_btree {

using common::TrajectoryPoint;

class Context
{
    public:
        Context(const std::shared_ptr<DependencyInjector>& injector);
        
        BTreeContextState Init(const BTreeContextConfig& config);

        virtual BTreeContextState Execute(const TrajectoryPoint& planning_start_point, BTreeFrame* const frame) = 0;

    protected:
        std::unique_ptr<StageSelector> stage_selector_ = nullptr;
        BTreeContextParameters parameters_;
        BTreeContextState state_ = BTreeContextState::CONTEXT_NOT_INITIALIZED;
        
        std::shared_ptr<DependencyInjector> injector_;
};

} // namespace planning_btree
} // namespace apollo