#pragma once

#include <FlowScene.h>
#include <DataModelRegistry.h>

#include "modules/tools/btviz/models/btree_data_model.h"
#include "modules/tools/btviz/btviz_base.h"

class BTVizBTreeFlowScene : public QtNodes::FlowScene
{
    public:
        BTVizBTreeFlowScene(std::shared_ptr<QtNodes::DataModelRegistry> registry,
                    QObject* parent = Q_NULLPTR);

        // QtNodes::Node& createNodeAtPosition(const QString& node_type, const QPointF& scene_pos) override;
        
        void generateProtobuf(BTreeConfig* tree_config);
    
};