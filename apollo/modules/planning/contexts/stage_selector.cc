#include "modules/planning/contexts/stage_selector.h"

namespace apollo {
namespace planning {
namespace context {

bool StageSelector::Init(const StageFSM& fsm_config)
{
    stage_dispatcher_.Init();
    stages_.clear();
    transitions_.clear();

    BTreeStageConfigs stage_configs;
    apollo::cyber::common::GetProtoFromFile(FLAGS_b_tree_stage_config_file, &stage_configs);

    for(int i = 0; i < fsm_config.stage_size(); ++i)
    {
        auto stage_name = fsm_config.stage(i);
        auto stage = stage_dispatcher_.Dispatch(stage_name);

        // TODO: Speed-up config lookup
        for (int i = 0; i < stage_configs.b_tree_stage_config_size(); ++i)
        {
            auto stage_config = stage_configs.b_tree_stage_config(i);
            if (stage_config.name() == stage_name)
            {
                stage->Init(stage_config);
                break;
            } 
        }

        stages_[stage_name] = stage;
    }

    for(int i = 0; i < fsm_config.transition_size(); ++i)
    {
        auto transition = fsm_config.transition(i);

        transitions_[transition.on_state()][transition.from_stage()] = transition.to_stage();
    }

    current_stage_name_ = fsm_config.initial_stage();
    current_stage_ = stages_[current_stage_name_];
    return true;
}

std::shared_ptr<Stage> StageSelector::GetCurrentStage(const TrajectoryPoint& planning_start_point, Frame* const frame)
{
    auto current_state = current_stage_->GetState();

    current_stage_name_ = transitions_[current_state][current_stage_name_];
    current_stage_ = stages_[current_stage_name_];

    return current_stage_;
}

} // namespace apollo
} // namespace planning
} // namespace context