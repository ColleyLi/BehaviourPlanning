#pragma once
#include <set>
#include <unordered_map>
#include <memory>
#include "modules/planning_btree/contexts/context.h"
#include "modules/planning_btree/contexts/context_dispatcher.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning_btree/common/dependency_injector.h"

namespace apollo {
namespace planning_btree {

using common::TrajectoryPoint;

class ContextSelector
{
    public:
        ContextSelector(const std::shared_ptr<DependencyInjector>& injector);

        bool Init();

        std::shared_ptr<Context> GetCurrentContext(const TrajectoryPoint& planning_start_point, BTreeFrame* const frame);

    private:
        std::shared_ptr<DependencyInjector> injector_;
        std::unique_ptr<ContextDispatcher> context_dispatcher_ = nullptr;
        std::unordered_map<BTreeContextType, std::shared_ptr<Context>, std::hash<int>> contexts_;
};

} // namespace planning_btree
} // namespace apollo