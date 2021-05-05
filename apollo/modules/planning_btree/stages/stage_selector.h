#pragma once

#include <unordered_map>
#include <memory>
#include "modules/planning_btree/stages/stage.h"
#include "modules/planning_btree/stages/stage_dispatcher.h"
#include "modules/planning_btree/proto/btree_context_config.pb.h"
#include "modules/planning_btree/common/dependency_injector.h"

namespace apollo {
namespace planning_btree {

using common::TrajectoryPoint;

class StageSelector
{
    public:
        StageSelector(const std::shared_ptr<DependencyInjector>& injector);

        bool Init(const StageFSM& fsm_config);
        
        std::shared_ptr<Stage> GetCurrentStage(const TrajectoryPoint& planning_start_point, BTreeFrame* const frame);

    private:
        std::unique_ptr<StageDispatcher> stage_dispatcher_ = nullptr;
        std::unordered_map<BTreeStageType, std::shared_ptr<Stage>, std::hash<int>> stages_;
        std::unordered_map<BTreeStageState, std::unordered_map<BTreeStageType, BTreeStageType, std::hash<int>>, std::hash<int>> transitions_;
        std::shared_ptr<Stage> current_stage_ = nullptr;
        BTreeStageType current_stage_type_;
        std::shared_ptr<DependencyInjector> injector_;
};

} // namespace planning_btree
} // namespace apollo