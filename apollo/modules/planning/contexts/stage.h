#pragma once

#include <memory>
#include "modules/common/status/status.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/behaviour_tree/bt_node.h"
#include "modules/common/proto/pnc_point.pb.h"


namespace apollo {
namespace planning {
namespace context {

using common::TrajectoryPoint;

class Stage
{
    public:
        virtual common::Status Init(const BTreeStageConfig& config) = 0;

        virtual common::Status Execute(const TrajectoryPoint& planning_start_point, Frame* const frame) = 0;

        BTreeStageState GetState()
        {
            return state_;
        };

    protected:
        BTreeStageParameters parameters_;
        BTreeStageState state_;
        std::unique_ptr<BTNode> behaviour_tree_;
};

} // namespace apollo
} // namespace planning
} // namespace context