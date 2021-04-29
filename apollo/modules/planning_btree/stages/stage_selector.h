#pragma once

#include <unordered_map>
#include <memory>
#include "modules/planning_btree/stages/stage.h"
#include "modules/planning_btree/stages/stage_dispatcher.h"
#include "modules/planning_btree/proto/btree_context_config.pb.h"
#include "cyber/common/file.h"

namespace apollo {
namespace planning_btree {

using common::TrajectoryPoint;

class StageSelector
{
    public:
        StageSelector() = default;

        bool Init(const StageFSM& fsm_config);
        
        std::shared_ptr<Stage> GetCurrentStage(const TrajectoryPoint& planning_start_point, BTreeFrame* const frame);

    private:
        StageDispatcher stage_dispatcher_;
        std::unordered_map<BTreeStageType, std::shared_ptr<Stage>, std::hash<int>> stages_;
        std::unordered_map<BTreeStageState, std::unordered_map<BTreeStageType, BTreeStageType, std::hash<int>>, std::hash<int>> transitions_;
        std::shared_ptr<Stage> current_stage_ = nullptr;
        BTreeStageType current_stage_type_;
};

} // namespace planning_btree
} // namespace apollo