#pragma once

#include <unordered_map>
#include <memory>
#include "modules/planning/contexts/stage.h"
#include "modules/planning/contexts/stage_dispatcher.h"
#include "modules/common/proto/pnc_point.pb.h"


namespace apollo {
namespace planning {
namespace context {

using common::TrajectoryPoint;

class StageSelector
{
    public:
        StageSelector() = default;

        bool Init(const StageFSM& fsm_config);
        
        std::shared_ptr<Stage> GetCurrentStage(const TrajectoryPoint& planning_start_point, Frame* const frame);

    private:
        StageDispatcher stage_dispatcher_;
        std::unordered_map<BTreeStageName, std::shared_ptr<Stage>, std::hash<int>> stages_;
        std::unordered_map<BTreeStageState, std::unordered_map<BTreeStageName, BTreeStageName, std::hash<int>>, std::hash<int>> transitions_;
        std::shared_ptr<Stage> current_stage_ = nullptr;
        BTreeStageName current_stage_name_;
};

} // namespace apollo
} // namespace planning
} // namespace context