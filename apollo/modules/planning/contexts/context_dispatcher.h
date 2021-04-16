#pragma once

#include <unordered_map>
#include <memory>
#include "modules/common/status/status.h"
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

        common::Status Init();

  	    std::shared_ptr<Context> Dispatch(const BTreeContextName& context_name);
    private:
        void RegisterContexts();

        common::util::Factory<BTreeContextName, Context> context_factory_;
};

} // namespace apollo
} // namespace planning
} // namespace context