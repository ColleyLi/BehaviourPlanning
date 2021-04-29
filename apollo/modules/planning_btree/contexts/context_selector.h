#pragma once
#include <set>
#include <unordered_map>
#include <memory>
#include "modules/planning_btree/contexts/context.h"
#include "modules/planning_btree/contexts/context_dispatcher.h"
#include "modules/common/proto/pnc_point.pb.h"


namespace apollo {
namespace planning_btree {

using common::TrajectoryPoint;

class ContextSelector
{
    public:
        ContextSelector() = default;

        bool Init(const std::set<BTreeContextType>& contexts_to_use);

        std::shared_ptr<Context> GetCurrentContext(const TrajectoryPoint& planning_start_point, BTreeFrame* const frame);

    private:
        ContextDispatcher context_dispatcher_;
        std::unordered_map<BTreeContextType, std::shared_ptr<Context>, std::hash<int>> contexts_;
};

} // namespace planning_btree
} // namespace apollo