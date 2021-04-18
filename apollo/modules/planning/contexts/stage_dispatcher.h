#pragma once

#include <memory>
#include "modules/planning/contexts/stage.h"
#include "modules/common/util/factory.h"

namespace apollo {
namespace planning {
namespace context {

class StageDispatcher
{
    public:
        StageDispatcher() = default;
        ~StageDispatcher() = default;

        bool Init();

  	    std::shared_ptr<Stage> Dispatch(const BTreeStageType& stage_type);
    private:
        void RegisterStages();

        common::util::Factory<BTreeStageType, Stage> stage_factory_;
};

} // namespace context
} // namespace planning
} // namespace apollo