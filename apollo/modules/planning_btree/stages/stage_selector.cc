#include "modules/planning_btree/stages/stage_selector.h"
#include "modules/planning_btree/common/btree_planning_gflags.h"

namespace apollo {
namespace planning_btree {

bool StageSelector::Init(const StageFSM& fsm_config)
{
    stage_dispatcher_.Init();
    stages_.clear();
    transitions_.clear();

    BTreeStageConfigs stage_configs;
    cyber::common::GetProtoFromFile(FLAGS_btree_stage_config_file, &stage_configs);

    for(int i = 0; i < fsm_config.stage_size(); ++i)
    {
        auto stage_type = fsm_config.stage(i);
        auto stage = stage_dispatcher_.Dispatch(stage_type);

        // TODO: Speed-up config lookup
        for (int i = 0; i < stage_configs.stage_config_size(); ++i)
        {
            auto stage_config = stage_configs.stage_config(i);
            if (stage_config.type() == stage_type)
            {
                stage->Init(stage_config);
                break;
            } 
        }

        stages_[stage_type] = stage;
    }

    for(int i = 0; i < fsm_config.transition_size(); ++i)
    {
        auto transition = fsm_config.transition(i);
        transitions_[transition.on_state()][transition.from_stage()] = transition.to_stage();
    }

    current_stage_type_ = fsm_config.initial_stage();
    current_stage_ = stages_[current_stage_type_];
    return true;
}

std::shared_ptr<Stage> StageSelector::GetCurrentStage(const TrajectoryPoint& planning_start_point, BTreeFrame* const frame)
{
    auto current_state = current_stage_->GetState();

    current_stage_type_ = transitions_[current_state][current_stage_type_];
    current_stage_ = stages_[current_stage_type_];

    return current_stage_;
}

} // namespace planning_btree
} // namespace apollo