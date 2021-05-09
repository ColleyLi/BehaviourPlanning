#pragma once

#include <FlowScene.h>
#include <DataModelRegistry.h>

#include "modules/tools/btviz/models/stage_data_model.h"
#include "modules/tools/btviz/btviz_base.h"

class BTVizStageFlowScene : public QtNodes::FlowScene
{
    public:
        BTVizStageFlowScene(std::shared_ptr<QtNodes::DataModelRegistry> registry,
                    QObject* parent = Q_NULLPTR);
};