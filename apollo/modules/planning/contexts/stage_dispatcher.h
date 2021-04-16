#pragma once

#include <memory>
#include "modules/common/status/status.h"
#include "modules/planning/contexts/stage.h"
#include "modules/common/util/factory.h"
#include "modules/planning/proto/b_tree_stage_config.pb.h"

namespace apollo {
namespace planning {
namespace context {

class StageDispatcher
{
    public:
        StageDispatcher() = default;
        ~StageDispatcher() = default;

        common::Status Init();

  	    std::shared_ptr<Stage> Dispatch(const BTreeStageName& stage_name);
    private:
        void RegisterStages();

        common::util::Factory<BTreeStageName, Stage> stage_factory_;
};

} // namespace apollo
} // namespace planning
} // namespace context