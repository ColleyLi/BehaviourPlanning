#pragma once

#include <memory>
#include "modules/planning_btree/stages/stage.h"
#include "modules/common/util/factory.h"

namespace apollo {
namespace planning_btree {

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

} // namespace planning_btree
} // namespace apollo