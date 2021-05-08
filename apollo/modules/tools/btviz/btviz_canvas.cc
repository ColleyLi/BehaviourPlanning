#include "btviz_canvas.h"

#include <typeinfo>

BTvizCanvas::BTvizCanvas(QWidget *parent): QObject(parent)
{
    flow_view_ = std::make_unique<QtNodes::FlowView>();
    
    context_model_registry_ = std::make_shared<QtNodes::DataModelRegistry>();
    for (const auto& node: getExistingContextNodes())
    {
    //   qDebug() << "Adding node of type " << node.type << " to " << node.category;
      registerContextNode(node);
      context_nodes_.push_back(node);
    }

    stage_model_registry_ = std::make_shared<QtNodes::DataModelRegistry>();
    for (const auto& node: getExistingStageNodes())
    {
    //   qDebug() << "Adding node of type " << node.type << " to " << node.category;
      registerStageNode(node);
      stage_nodes_.push_back(node);
    }

    btree_model_registry_ = std::make_shared<QtNodes::DataModelRegistry>();
    for (const auto& node: getExistingBTreeNodes())
    {
    //   qDebug() << "Adding node of type " << node.type << " to " << node.category;
      registerBTreeNode(node);
      btree_nodes_.push_back(node);
    }

    QString btplan_id = "btplan_1";
    QString btplan_name = "My Plan";
    QString context_scene_id = "context_1";
    createScene(FlowSceneType::CONTEXT_SCENE, context_scene_id, btplan_id, btplan_name);
    setScene(FlowSceneType::CONTEXT_SCENE, context_scene_id);
}

void BTvizCanvas::registerContextNode(const BTvizNode& node)
{
    QtNodes::DataModelRegistry::RegistryItemCreator creator;
    creator = [node]() -> QtNodes::DataModelRegistry::RegistryItemPtr
    {
        return std::make_unique<ContextDataModel>(node);
    };
    context_model_registry_->registerModel(node.category, creator, node.type);
}

void BTvizCanvas::registerStageNode(const BTvizNode& node)
{
    QtNodes::DataModelRegistry::RegistryItemCreator creator;
    creator = [node]() -> QtNodes::DataModelRegistry::RegistryItemPtr
    {
        return std::make_unique<StageDataModel>(node);
    };
    stage_model_registry_->registerModel(node.category, creator, node.type);
}

void BTvizCanvas::registerBTreeNode(const BTvizNode& node)
{
    QtNodes::DataModelRegistry::RegistryItemCreator creator;
    creator = [node]() -> QtNodes::DataModelRegistry::RegistryItemPtr
    {
        return std::make_unique<BTreeDataModel>(node);
    };
    btree_model_registry_->registerModel(node.category, creator, node.type);
}

void BTvizCanvas::createScene(const FlowSceneType& scene_type, const QString& scene_id, const QString& parent_scene_id, const QString& parent_name)
{
    std::shared_ptr<QtNodes::FlowScene> scene;
    if (scene_type == FlowSceneType::CONTEXT_SCENE)
    {
        context_flow_scenes_[scene_id] = std::make_shared<BTvizContextFlowScene>(context_model_registry_, this);
        context_flow_scenes_[scene_id]->createNodeAtPosition(CONTEXT_ROOT_TYPE, scene_id, parent_scene_id, parent_name, QPointF(0, 0));
        scene = context_flow_scenes_[scene_id];
    }
    else if (scene_type == FlowSceneType::STAGE_SCENE)
    {
        stage_flow_scenes_[scene_id] = std::make_shared<BTvizStageFlowScene>(stage_model_registry_, this);
        stage_flow_scenes_[scene_id]->createNodeAtPosition(STAGE_ROOT_TYPE, scene_id, parent_scene_id, parent_name, QPointF(0, 0));
        scene = stage_flow_scenes_[scene_id];
    }
    else if (scene_type == FlowSceneType::BTREE_SCENE)
    {
        btree_flow_scenes_[scene_id] = std::make_shared<BTvizBTreeFlowScene>(btree_model_registry_, this);
        btree_flow_scenes_[scene_id]->createNodeAtPosition(BTREE_ROOT_TYPE, scene_id, parent_scene_id, parent_name, QPointF(0, 0));
        scene = btree_flow_scenes_[scene_id];
    }
    
    scene->setLayout(QtNodes::PortLayout::Vertical);
    connect(scene.get(), &QtNodes::FlowScene::nodeDoubleClicked, this, &BTvizCanvas::onNodeDoubleClicked);
}

void BTvizCanvas::setScene(const FlowSceneType& scene_type, const QString& scene_id)
{
    std::shared_ptr<QtNodes::FlowScene> scene;
    if (scene_type == FlowSceneType::CONTEXT_SCENE)
    {
        scene = context_flow_scenes_[scene_id];
    }
    else if (scene_type == FlowSceneType::STAGE_SCENE)
    {
        scene = stage_flow_scenes_[scene_id];
    }
    else if (scene_type == FlowSceneType::BTREE_SCENE)
    {
        scene = btree_flow_scenes_[scene_id];
    }

    // qDebug() << "Setting scene to: " << scene.get();
    // qDebug() << "Scene nodes size: " << scene->allNodes().size();

    current_scene_id_ = scene_id;
    current_scene_type_ = scene_type;
    flow_scene_ = scene;

    flow_view_->setScene(flow_scene_.get());

    // fitToScreen();

    // qDebug() << "Connected double click";
    // qDebug() << "----------NEW_SCENE-----------";
}

void BTvizCanvas::fitToScreen() const
{
    QRectF rect = flow_scene_->itemsBoundingRect();
    rect.setBottom(rect.top() + rect.height()* 1.2);

    const int min_height = 300;
    if(rect.height() < min_height)
    {
        rect.setBottom(rect.top() + min_height);
    }

    flow_view_->setSceneRect(rect);
    flow_view_->fitInView(rect, Qt::KeepAspectRatio);
    flow_view_->scale(0.9, 0.9);
}

void BTvizCanvas::onNodeDoubleClicked(QtNodes::Node& node)
{  
    // qDebug() << "Current node data model name:" << node.nodeDataModel()->name();
    // qDebug() << "Current scene type: " << int(current_scene_type_);
    if(current_scene_type_ == FlowSceneType::CONTEXT_SCENE)
    {
        // qDebug() << "Current scene is context";
        auto* node_data = dynamic_cast<ContextDataModel*>(node.nodeDataModel());
        if (node_data->getNodeType() != CONTEXT_ROOT_TYPE)
        {
            auto scene_id = node_data->getStageSceneId();
            // qDebug() << "Current Stage Scene ID: " << scene_id;
            if(scene_id.length() && stage_flow_scenes_.find(scene_id) != stage_flow_scenes_.end())
            {
                // qDebug() << "Current Stage ID is present in stage flow scenes. Opening STAGE_SCENE = 2";
                setScene(FlowSceneType::STAGE_SCENE, scene_id);
            }
            else
            {
                scene_id = "Stage Scene " + node_data->getNodeId();
                node_data->setStageSceneId(scene_id);
                createScene(FlowSceneType::STAGE_SCENE, scene_id, current_scene_id_, node_data->getNodeName());

                // qDebug() << "Created new stage scene with ID: " << scene_id << " Starting STAGE_SCENE = 2";

                setScene(FlowSceneType::STAGE_SCENE, scene_id);
            }
        }
        else
        {
            qDebug() << "Will open BTplan info in future";
        }
    }
    else if (current_scene_type_ == FlowSceneType::STAGE_SCENE)
    {
        // qDebug() << "Current scene is stage";
        auto* node_data = dynamic_cast<StageDataModel*>(node.nodeDataModel());
        if (node_data->getNodeType() == STAGE_ROOT_TYPE)
        {
            auto scene_id = node_data->getContextSceneId();
            if(scene_id.length() && context_flow_scenes_.find(scene_id) != context_flow_scenes_.end())
            {
                // qDebug() << "Returning back to context flow scene with ID: " << scene_id << "Opening CONTEXT_SCENE = 1";
                setScene(FlowSceneType::CONTEXT_SCENE, scene_id);
            }
            else
            {
                qDebug() << "This node does not have context scene id or scene id is not in context flow scenes list!";
            }
        }
        else
        {
            auto scene_id = node_data->getBTreeSceneId();
            if(scene_id.length() && btree_flow_scenes_.find(scene_id) != btree_flow_scenes_.end())
            {
                // qDebug() << "Current BTree scene ID is present in btree flow scenes. Opening BTREE_SCENE = 3";
                setScene(FlowSceneType::BTREE_SCENE, scene_id);
            }
            else
            {
                scene_id = "Btree Scene " + node_data->getNodeId();
                node_data->setBTreeSceneId(scene_id);
                createScene(FlowSceneType::BTREE_SCENE, scene_id, current_scene_id_, node_data->getNodeName());

                // qDebug() << "Created new btree scene with ID: " << scene_id << " Starting BTREE_SCENE = 3";

                setScene(FlowSceneType::BTREE_SCENE, scene_id);
            }
        }
    }
    else if (current_scene_type_ == FlowSceneType::BTREE_SCENE)
    {
        // qDebug() << "Current scene is btree scene";
        auto* node_data = dynamic_cast<BTreeDataModel*>(node.nodeDataModel());
        if (node_data->getNodeType() == BTREE_ROOT_TYPE)
        {
            auto scene_id = node_data->getStageSceneId();
            if(scene_id.length() && stage_flow_scenes_.find(scene_id) != stage_flow_scenes_.end())
            {
                // qDebug() << "Returning back to stage flow scene with ID: " << scene_id << "Opening STAGE_SCENE = 2";
                setScene(FlowSceneType::STAGE_SCENE, scene_id);
            }
            else
            {
                qDebug() << "This node does not have stage scene id or scene id is not in stage flow scenes list!";
            }
        }
        else
        {
            qDebug() << "Will open node config in future";
        }
    }
    else
    {
        qDebug() << "No scene";
    }
}