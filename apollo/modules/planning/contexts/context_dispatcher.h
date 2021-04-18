#pragma once

#include <unordered_map>
#include <memory>
#include "modules/planning/contexts/context.h"
#include "modules/common/util/factory.h"
#include "modules/planning/proto/b_tree_context_config.pb.h"

namespace apollo {
namespace planning {
namespace context {

class ContextDispatcher
{
    public:
        ContextDispatcher() = default;
        ~ContextDispatcher() = default;

        bool Init();

  	    std::shared_ptr<Context> Dispatch(const BTreeContextType& context_type);
    private:
        void RegisterContexts();

        common::util::Factory<BTreeContextType, Context> context_factory_;
};

} // namespace context
} // namespace planning
} // namespace apollo