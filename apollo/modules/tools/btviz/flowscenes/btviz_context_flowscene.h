#pragma once

#include <FlowScene.h>
#include <DataModelRegistry.h>

#include "modules/tools/btviz/models/context_data_model.h"
#include "modules/tools/btviz/btviz_base.h"

class BTVizContextFlowScene : public QtNodes::FlowScene
{
    public:
        BTVizContextFlowScene(std::shared_ptr<QtNodes::DataModelRegistry> registry,
                    QObject* parent = Q_NULLPTR);

        void generateProtobuf(BTPlan& btplan);
};
