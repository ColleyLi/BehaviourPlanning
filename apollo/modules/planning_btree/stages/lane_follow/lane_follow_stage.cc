#include "modules/planning_btree/stages/lane_follow/lane_follow_stage.h"
#include <unordered_map>

namespace apollo {
namespace planning_btree {

BTreeStageState LaneFollowStage::Init(const BTreeStageConfig& config)
{
    state_ = Stage::Init(config);

    return state_;
}

BTreeStageState LaneFollowStage::Execute(const TrajectoryPoint& planning_start_point, BTreeFrame* const frame)
{
    AERROR << "Executed LaneFollow stage";

    // AERROR << "Number of reference lines:\t" << frame->mutable_reference_line_info()->size();
    // bool has_drivable_lane = false;

    // // Refresh states
    // for (auto& ref_line : *frame->mutable_reference_line_info()) 
    // {
    //     ref_line.SetDrivable(false);
    //     ref_line.set_is_path_lane_borrow(true);
    //     ref_line.SetCost(0.0);
    // } 

    behaviour_tree_->Execute(frame);

    // AERROR << "Reference lines after planning: ";
    // for (auto& ref_line : *frame->mutable_reference_line_info()) 
    // {
    //     AERROR << "Id: " << ref_line.Lanes().Id() << " Cost: " << ref_line.Cost() << " IsDrivable: " << ref_line.IsDrivable() << " IsChangeLane: " << ref_line.IsChangeLanePath();
    //     if (ref_line.IsDrivable())
    //     {
    //     ref_line.set_trajectory_type(ADCTrajectory::NORMAL);

    //     DiscretizedTrajectory trajectory;
    //     if (ref_line.CombinePathAndSpeedProfile(planning_start_point.relative_time(),
    //                                             planning_start_point.path_point().s(), 
    //                                             &trajectory)) 
    //     {
    //         ref_line.SetTrajectory(trajectory);
    //         // has_drivable_lane = true;
    //     }
    //     }
    // }

    state_ = BTreeStageState::STAGE_DONE;
    return state_;
}

} // namespace planning_btree
} // namespace apollo