#pragma once

#include <unordered_map>
#include <memory>
#include "modules/planning_btree/contexts/context.h"
#include "modules/common/util/factory.h"
#include "modules/planning_btree/proto/btree_context_config.pb.h"

namespace apollo {
namespace planning_btree {

class ContextDispatcher
{
    public:
        ContextDispatcher(const std::shared_ptr<DependencyInjector>& injector);
        ~ContextDispatcher() = default;

        bool Init();

  	    std::shared_ptr<Context> Dispatch(const BTreeContextType& context_type);
    private:
        void RegisterContexts();

        common::util::Factory<BTreeContextType, Context, 
                              Context *(*)(const std::shared_ptr<DependencyInjector> &injector),
                              std::unordered_map<BTreeContextType, Context *(*)(const std::shared_ptr<DependencyInjector> &injector), std::hash<int>>> context_factory_;
        
        std::shared_ptr<DependencyInjector> injector_;
};

} // namespace planning_btree
} // namespace apollo