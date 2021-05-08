#ifndef BTVIZ_BTREE_CANVAS_H
#define BTVIZ_BTREE_CANVAS_H

#include <memory>
#include <unordered_map>

#include <QString>
#include <QObject>
#include <QWidget>
#include <QLineEdit>

#include "modules/tools/btviz/btviz_base.h"
#include "modules/tools/btviz/btviz_context_flowscene.h"
#include "modules/tools/btviz/btviz_stage_flowscene.h"
#include "modules/tools/btviz/btviz_btree_flowscene.h"

#include "modules/tools/btviz/models/context_data_model.h"
#include "modules/tools/btviz/models/stage_data_model.h"
#include "modules/tools/btviz/models/btree_data_model.h"

#include <QStringStdHash.h>
#include <Node.h>
#include <NodeData.h>
#include <DataModelRegistry.h>
#include <FlowScene.h>
#include <FlowView.h>

class BTvizCanvas : public QObject
{
    Q_OBJECT
public:
    BTvizCanvas(QWidget *parent = nullptr);

    QtNodes::FlowScene* scene() { return flow_scene_.get(); }
    QtNodes::FlowView*  view() { return flow_view_.get(); } 

    void fitToScreen() const;
    
    enum class FlowSceneType
    {
        NO_SCENE,
        CONTEXT_SCENE,
        STAGE_SCENE,
        BTREE_SCENE
    };

private:
    // TODO: refactor to make generic
    void registerContextNode(const BTvizNode& node);
    void registerStageNode(const BTvizNode& node);
    void registerBTreeNode(const BTvizNode& node);
    
    void onNodeDoubleClicked(QtNodes::Node& node);
    
    void createScene(const FlowSceneType& scene_type, const QString& scene_id, const QString& parent_scene_id, const QString& parent_name);
    void setScene(const FlowSceneType& scene_type, const QString& scene_id);

private:
    std::unordered_map<QString, std::shared_ptr<BTvizContextFlowScene>> context_flow_scenes_;
    std::unordered_map<QString, std::shared_ptr<BTvizStageFlowScene>> stage_flow_scenes_;
    std::unordered_map<QString, std::shared_ptr<BTvizBTreeFlowScene>> btree_flow_scenes_;

    FlowSceneType current_scene_type_ = FlowSceneType::NO_SCENE;
    QString current_scene_id_ = ""; 
    std::shared_ptr<QtNodes::FlowScene> flow_scene_ = nullptr;
    std::unique_ptr<QtNodes::FlowView> flow_view_ = nullptr;
    
    std::shared_ptr<QtNodes::DataModelRegistry> context_model_registry_;
    std::shared_ptr<QtNodes::DataModelRegistry> stage_model_registry_;
    std::shared_ptr<QtNodes::DataModelRegistry> btree_model_registry_;

    std::vector<BTvizNode> context_nodes_;
    std::vector<BTvizNode> stage_nodes_;
    std::vector<BTvizNode> btree_nodes_;
};

#endif // BTVIZ_BTREE_CANVAS_H