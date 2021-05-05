#pragma once

#include "modules/planning_btree/contexts/context.h"

namespace apollo {
namespace planning_btree {

class LaneFollowContext : public Context
{
    public:
        LaneFollowContext(const std::shared_ptr<DependencyInjector>& injector);
        
        BTreeContextState Init(const BTreeContextConfig& config);

        BTreeContextState Execute(const TrajectoryPoint& planning_start_point, BTreeFrame* const frame);
};

} // namespace planning_btree
} // namespace apollo