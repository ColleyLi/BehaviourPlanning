#pragma once

#include <memory>
#include <unordered_map>

#include <QString>
#include <QObject>
#include <QTabWidget>
#include <QLineEdit>
#include <QtCore/QFile>
#include <QtWidgets/QFileDialog>

#include "modules/tools/btviz/btviz_base.h"
#include "modules/tools/btviz/flowscenes/btviz_context_flowscene.h"
#include "modules/tools/btviz/flowscenes/btviz_stage_flowscene.h"
#include "modules/tools/btviz/flowscenes/btviz_btree_flowscene.h"

#include "modules/tools/btviz/models/context_data_model.h"
#include "modules/tools/btviz/models/stage_data_model.h"
#include "modules/tools/btviz/models/btree_data_model.h"

#include <QStringStdHash.h>
#include <Node.h>
#include <NodeData.h>
#include <DataModelRegistry.h>
#include <FlowScene.h>
#include <FlowView.h>

class BTVizCanvas : public QTabWidget
{
    Q_OBJECT
public:
    BTVizCanvas(QWidget *parent = nullptr);

    QtNodes::FlowScene* getScene() { return current_scene_.get(); }
    QtNodes::FlowView*  getView() { return flow_view_.get(); } 

    void fitToScreen() const;
    
    enum class FlowSceneType
    {
        NO_SCENE,
        CONTEXT_SCENE,
        STAGE_SCENE,
        BTREE_SCENE
    };

    void loadBTPlan();
    void saveBTPlan();

    void saveBTPlanProtobuf();

private:
    // TODO: refactor to make generic
    void registerContextNode(const BTVizNode& node);
    void registerStageNode(const BTVizNode& node);
    void registerBTreeNode(const BTVizNode& node);
    
    void onNodeDoubleClicked(QtNodes::Node& node);
    
    void createScene(const FlowSceneType& scene_type, const QString& scene_id, const QString& parent_scene_id, const QString& parent_name);
    void setScene(const FlowSceneType& scene_type, const QString& scene_id);

    void saveProtobuf(const QString& file_name);

private:
    std::unordered_map<QString, std::shared_ptr<BTVizContextFlowScene>> context_scenes_;
    std::unordered_map<QString, std::shared_ptr<BTVizStageFlowScene>> stage_scenes_;
    std::unordered_map<QString, std::shared_ptr<BTVizBTreeFlowScene>> btree_scenes_;

    QString current_tab_id_ = "";
    std::unordered_map<QString, int> tabs_;

    FlowSceneType current_scene_type_ = FlowSceneType::NO_SCENE;
    QString current_scene_id_ = "";
    std::shared_ptr<QtNodes::FlowScene> current_scene_ = nullptr;
    std::unique_ptr<QtNodes::FlowView> flow_view_ = nullptr;
    
    std::shared_ptr<QtNodes::DataModelRegistry> context_model_registry_;
    std::shared_ptr<QtNodes::DataModelRegistry> stage_model_registry_;
    std::shared_ptr<QtNodes::DataModelRegistry> btree_model_registry_;
};