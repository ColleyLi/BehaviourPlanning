#pragma once
#include <set>
#include <unordered_map>
#include <memory>
#include "modules/planning/contexts/context.h"
#include "modules/planning/contexts/context_dispatcher.h"
#include "modules/common/proto/pnc_point.pb.h"


namespace apollo {
namespace planning {
namespace context {

using common::TrajectoryPoint;

class ContextSelector
{
    public:
        ContextSelector() = default;

        bool Init(const std::set<BTreeContextName>& contexts_to_use);

        std::shared_ptr<Context> GetCurrentContext(const TrajectoryPoint& planning_start_point, Frame* const frame);

    private:
        ContextDispatcher context_dispatcher_;
        std::unordered_map<BTreeContextName, std::shared_ptr<Context>, std::hash<int>> contexts_;
};

} // namespace apollo
} // namespace planning
} // namespace context