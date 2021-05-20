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
    ADEBUG << "Number of dynamic reference lines:\t" << frame->GetDynamicReferenceLines().size(); 

    auto status = behaviour_tree_->Execute(frame);

    if (status == BTreeNodeState::NODE_FAILED)
    {
        ADEBUG << "Btree execution status: FAILED";
    }
    else if (status == BTreeNodeState::NODE_DONE)
    {
        ADEBUG << "Btree execution status: DONE";
    }
    
    for (auto& ref_line : *frame->GetMutableDynamicReferenceLines()) 
    {
        ADEBUG << "Id: " << ref_line.GetRouteSegments().Id() << " Cost: " << ref_line.GetCost() << " Is Drivable: " << ref_line.IsDrivable() << " Is Lane Change: " << ref_line.IsLaneChangePath();
        if (ref_line.IsDrivable())
        {
    //     ref_line.set_trajectory_type(ADCTrajectory::NORMAL);

        DiscretizedTrajectory trajectory;
        if (ref_line.CombinePathAndSpeedProfiles(planning_start_point.relative_time(),
                                                planning_start_point.path_point().s(), 
                                                &trajectory)) 
        {
            ref_line.SetDiscretizedTrajectory(trajectory);
            // has_drivable_lane = true;
        }
        }
    }

    state_ = BTreeStageState::STAGE_DONE;
    return state_;
}

} // namespace planning_btree
} // namespace apollo