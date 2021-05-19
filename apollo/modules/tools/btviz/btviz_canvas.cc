#include "btviz_canvas.h"

BTVizCanvas::BTVizCanvas(QWidget *parent): QTabWidget(parent)
{
    flow_view_ = std::make_unique<QtNodes::FlowView>();
    
    context_model_registry_ = std::make_shared<QtNodes::DataModelRegistry>();
    for (const auto& node: getExistingContextNodes())
    {
    //   qDebug() << "Adding node of type " << node.type << " to " << node.category;
      registerContextNode(node);
    }

    stage_model_registry_ = std::make_shared<QtNodes::DataModelRegistry>();
    for (const auto& node: getExistingStageNodes())
    {
    //   qDebug() << "Adding node of type " << node.type << " to " << node.category;
      registerStageNode(node);
    }

    btree_model_registry_ = std::make_shared<QtNodes::DataModelRegistry>();
    for (const auto& node: getExistingBTreeNodes())
    {
    //   qDebug() << "Adding node of type " << node.type << " to " << node.category;
      registerBTreeNode(node);
    }

    QString btplan_id = "btplan_1";
    QString btplan_name = "My Plan";
    QString context_scene_id = btplan_id;
    
    current_tab_id_ = btplan_id;
    tabs_[btplan_id] = addTab(getView(), btplan_name);

    createScene(FlowSceneType::CONTEXT_SCENE, context_scene_id, btplan_id, btplan_name);
    setScene(FlowSceneType::CONTEXT_SCENE, context_scene_id);
}

void BTVizCanvas::registerContextNode(const BTVizNode& node)
{
    QtNodes::DataModelRegistry::RegistryItemCreator creator;
    creator = [node]() -> QtNodes::DataModelRegistry::RegistryItemPtr
    {
        return std::make_unique<ContextDataModel>(node);
    };
    context_model_registry_->registerModel(node.category, creator, node.type);
}

void BTVizCanvas::registerStageNode(const BTVizNode& node)
{
    QtNodes::DataModelRegistry::RegistryItemCreator creator;
    creator = [node]() -> QtNodes::DataModelRegistry::RegistryItemPtr
    {
        return std::make_unique<StageDataModel>(node);
    };
    stage_model_registry_->registerModel(node.category, creator, node.type);
}

void BTVizCanvas::registerBTreeNode(const BTVizNode& node)
{
    QtNodes::DataModelRegistry::RegistryItemCreator creator;
    creator = [node]() -> QtNodes::DataModelRegistry::RegistryItemPtr
    {
        return std::make_unique<BTreeDataModel>(node);
    };
    btree_model_registry_->registerModel(node.category, creator, node.type);
}

void BTVizCanvas::createScene(const FlowSceneType& scene_type, const QString& scene_id, const QString& parent_scene_id, const QString& parent_name)
{
    QString root_node_type;
    std::shared_ptr<QtNodes::FlowScene> scene;
    if (scene_type == FlowSceneType::CONTEXT_SCENE)
    {
        context_scenes_[scene_id] = std::make_shared<BTVizContextFlowScene>(context_model_registry_, this);
        scene = context_scenes_[scene_id];
        root_node_type = CONTEXT_ROOT_TYPE;
    }
    else if (scene_type == FlowSceneType::STAGE_SCENE)
    {
        stage_scenes_[scene_id] = std::make_shared<BTVizStageFlowScene>(stage_model_registry_, this);
        scene = stage_scenes_[scene_id];
        root_node_type = STAGE_ROOT_TYPE;
    }
    else if (scene_type == FlowSceneType::BTREE_SCENE)
    {
        btree_scenes_[scene_id] = std::make_shared<BTVizBTreeFlowScene>(btree_model_registry_, this);
        scene = btree_scenes_[scene_id];
        root_node_type = BTREE_ROOT_TYPE;
    }

    scene->setParentSceneId(parent_scene_id); 
    scene->setParentName(parent_name); 
    scene->createRootNode(root_node_type);
    scene->setLayout(QtNodes::PortLayout::Vertical);
    connect(scene.get(), &QtNodes::FlowScene::nodeDoubleClicked, this, &BTVizCanvas::onNodeDoubleClicked);
}

void BTVizCanvas::setScene(const FlowSceneType& scene_type, const QString& scene_id)
{
    std::shared_ptr<QtNodes::FlowScene> scene;
    if (scene_type == FlowSceneType::CONTEXT_SCENE)
    {
        scene = context_scenes_[scene_id];
    }
    else if (scene_type == FlowSceneType::STAGE_SCENE)
    {
        scene = stage_scenes_[scene_id];
    }
    else if (scene_type == FlowSceneType::BTREE_SCENE)
    {
        scene = btree_scenes_[scene_id];
    }

    // qDebug() << "Setting scene to: " << scene.get();
    // qDebug() << "Scene nodes size: " << scene->allNodes().size();

    current_scene_id_ = scene_id;
    current_scene_type_ = scene_type;
    current_scene_ = scene;

    flow_view_->setScene(getScene());
    // fitToScreen();

    // qDebug() << "Connected double click";
    // qDebug() << "----------NEW_SCENE-----------";
}

void BTVizCanvas::fitToScreen() const
{
    QRectF rect = current_scene_->itemsBoundingRect();
    rect.setBottom(rect.top() + rect.height() * 1.1);

    const int min_height = 200;
    if(rect.height() < min_height)
    {
        rect.setBottom(rect.top() + min_height);
    }

    flow_view_->setSceneRect(rect);
    flow_view_->fitInView(rect, Qt::KeepAspectRatio);
    flow_view_->scale(0.9, 0.9);
}

void BTVizCanvas::onNodeDoubleClicked(QtNodes::Node& node)
{  
    // qDebug() << "Current node data model id:" << node.nodeDataModel()->getNodeId();
    // qDebug() << "Current scene type: " << int(current_scene_type_);
    if(current_scene_type_ == FlowSceneType::CONTEXT_SCENE)
    {
        // qDebug() << "Current scene is context";
        auto* node_data = dynamic_cast<ContextDataModel*>(node.nodeDataModel());
        if (node_data->getNodeType() != CONTEXT_ROOT_TYPE)
        {   
            auto node_id = node_data->getNodeId();
            auto scene_id = current_scene_->getChildSceneId(node_id);
            // qDebug() << "Current Stage Scene ID: " << scene_id;
            if(scene_id.length() && stage_scenes_.find(scene_id) != stage_scenes_.end())
            {
                // qDebug() << "Current Stage ID is present in stage flow scenes. Opening STAGE_SCENE = 2";
                setScene(FlowSceneType::STAGE_SCENE, scene_id);
            }
            else
            {
                scene_id = "Stage Scene " + node_data->getNodeId();
                current_scene_->setChildSceneId(node_id, scene_id);
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
            auto scene_id = current_scene_->getParentSceneId();
            if(scene_id.length() && context_scenes_.find(scene_id) != context_scenes_.end())
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
            auto node_id = node_data->getNodeId();
            auto scene_id = current_scene_->getChildSceneId(node_id);
            if(scene_id.length() && btree_scenes_.find(scene_id) != btree_scenes_.end())
            {
                // qDebug() << "Current BTree scene ID is present in btree flow scenes. Opening BTREE_SCENE = 3";
                setScene(FlowSceneType::BTREE_SCENE, scene_id);
            }
            else
            {
                scene_id = "Btree Scene " + node_data->getNodeId();
                current_scene_->setChildSceneId(node_id, scene_id);
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
            auto scene_id = current_scene_->getParentSceneId();
            if(scene_id.length() && stage_scenes_.find(scene_id) != stage_scenes_.end())
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

void BTVizCanvas::clearCanvas()
{
    // TODO: cleanup
    for (auto& element: context_scenes_)
    {
        element.second->clearScene();
        disconnect(element.second.get(), &QtNodes::FlowScene::nodeDoubleClicked, this, &BTVizCanvas::onNodeDoubleClicked);
    }
    context_scenes_.clear();

    for (auto& element: stage_scenes_)
    {
        element.second->clearScene();
        disconnect(element.second.get(), &QtNodes::FlowScene::nodeDoubleClicked, this, &BTVizCanvas::onNodeDoubleClicked);
    }
    stage_scenes_.clear();

    for (auto& element: btree_scenes_)
    {
        element.second->clearScene();
        disconnect(element.second.get(), &QtNodes::FlowScene::nodeDoubleClicked, this, &BTVizCanvas::onNodeDoubleClicked);
    }
    btree_scenes_.clear();
}

void BTVizCanvas::loadBTPlan()
{
  QString file_name = QFileDialog::getOpenFileName(nullptr,
                                 tr("Open BTPlan"),
                                 QDir::homePath(),
                                 tr("BTPlan Files (*.btplan)"));

  if (!QFileInfo::exists(file_name))
  {
    return;
  }
  
  QFile file(file_name);

  if (file.open(QIODevice::ReadOnly))
  {
    loadBTPlanFromJSON(QJsonDocument::fromJson(file.readAll()).object());
  }
}

void BTVizCanvas::loadBTPlanFromJSON(const QJsonObject& btplan_json)
{
    clearCanvas();

    current_tab_id_ = btplan_json["current_tab_id"].toString();
    current_scene_id_ = btplan_json["current_scene_id"].toString();
    current_scene_type_ = static_cast<FlowSceneType>(btplan_json["current_scene_type"].toInt());

    QJsonObject tabs_json = btplan_json["tabs"].toObject();
    for(auto key: tabs_json.keys())
    {
        tabs_[key] = tabs_json.value(key).toInt();
    }

    // qDebug() << "Loaded tabs";

    QJsonObject context_scenes_json = btplan_json["context_scenes"].toObject();
    for(auto scene_id: context_scenes_json.keys())
    {
        // qDebug() << "Current context scene: " << scene_id;
        context_scenes_[scene_id] = std::make_shared<BTVizContextFlowScene>(context_model_registry_, this);
        // qDebug() << "Created scene";
        context_scenes_[scene_id]->loadFromMemory(context_scenes_json.value(scene_id).toObject());
        // qDebug() << "Loaded scene from json";
        connect(context_scenes_[scene_id].get(), &QtNodes::FlowScene::nodeDoubleClicked, this, &BTVizCanvas::onNodeDoubleClicked);
    }

    // qDebug() << "Loaded context scenes";

    QJsonObject stage_scenes_json = btplan_json["stage_scenes"].toObject();
    for(auto scene_id: stage_scenes_json.keys())
    {
        stage_scenes_[scene_id] = std::make_shared<BTVizStageFlowScene>(stage_model_registry_, this);
        stage_scenes_[scene_id]->loadFromMemory(stage_scenes_json.value(scene_id).toObject());
        connect(stage_scenes_[scene_id].get(), &QtNodes::FlowScene::nodeDoubleClicked, this, &BTVizCanvas::onNodeDoubleClicked);
    }
    
    // qDebug() << "Loaded stage scenes";
    
    QJsonObject btree_scenes_json = btplan_json["btree_scenes"].toObject();
    for(auto scene_id: btree_scenes_json.keys())
    {
        btree_scenes_[scene_id] = std::make_shared<BTVizBTreeFlowScene>(btree_model_registry_, this);
        btree_scenes_[scene_id]->loadFromMemory(btree_scenes_json.value(scene_id).toObject());
        connect(btree_scenes_[scene_id].get(), &QtNodes::FlowScene::nodeDoubleClicked, this, &BTVizCanvas::onNodeDoubleClicked);
    }

    // qDebug() << "Loaded btree scenes";

    setScene(current_scene_type_, current_scene_id_);
}

void BTVizCanvas::saveBTPlan()
{
    QString file_name = QFileDialog::getSaveFileName(nullptr,
                                 tr("Open BTPlan"),
                                 QDir::homePath(),
                                 tr("BTPlan Files (*.btplan)"));

    if (!file_name.isEmpty())
    {
        if (!file_name.endsWith(".btplan", Qt::CaseInsensitive))
            file_name += ".btplan";

        saveBTPlanToFile(file_name);
    }
}

void BTVizCanvas::saveBTPlanToFile(const QString& file_name)
{   
    QJsonObject btplan_json;
    btplan_json["current_tab_id"] = current_tab_id_;
    btplan_json["current_scene_type"] = static_cast<int>(current_scene_type_);
    btplan_json["current_scene_id"] = current_scene_id_;
    
    QJsonObject tabs_json;
    for (auto& element: tabs_)
    {
        tabs_json[element.first] = element.second;
    }
    btplan_json["tabs"] = tabs_json;

    QJsonObject context_scenes_json;
    for (auto& element: context_scenes_)
    {
        context_scenes_json[element.first] = element.second->getSceneJson();
    }
    btplan_json["context_scenes"] = context_scenes_json;

    QJsonObject stage_scenes_json;
    for (auto& element: stage_scenes_)
    {
        stage_scenes_json[element.first] = element.second->getSceneJson();
    }
    btplan_json["stage_scenes"] = stage_scenes_json;

    QJsonObject btree_scenes_json;
    for (auto& element: btree_scenes_)
    {
        btree_scenes_json[element.first]= element.second->getSceneJson();
    }
    btplan_json["btree_scenes"] = btree_scenes_json;

    QJsonDocument document(btplan_json);

    QFile file(file_name);
    if (file.open(QIODevice::WriteOnly))
    {
      file.write(document.toJson(QJsonDocument::Indented));
    }
}

void BTVizCanvas::saveBTPlanProtobuf()
{
    QString file_name = QFileDialog::getSaveFileName(nullptr,
                                 tr("Open BTPlan Protobuf"),
                                 QDir::homePath(),
                                 tr("BTPlan Protobuf Files (*.pb.btplan)"));

    if (!file_name.isEmpty())
    {
        if (!file_name.endsWith("pb.btplan", Qt::CaseInsensitive))
            file_name += ".pb.btplan";

        saveBTPlanProtobufToFile(file_name);
    }
}

// TODO: This function has two for loops since stage flowscene does not have access to btree_scenes_
// Maybe will need to refactor
void BTVizCanvas::saveBTPlanProtobufToFile(const QString& file_name)
{
    BTPlan btplan;
    auto parameters = btplan.mutable_parameters();

    auto context_configs = btplan.mutable_context_configs();
    auto stage_configs = btplan.mutable_stage_configs();

    auto current_context_scene = context_scenes_[current_tab_id_];
    for (auto* context_node : current_context_scene->allNodes())
    {
        auto* context_node_data = dynamic_cast<ContextDataModel*>(context_node->nodeDataModel());
        auto context_node_id = context_node_data->getNodeId();
        auto context_node_type = context_node_data->getNodeType();

        // qDebug() << "Processing context node with id: " << context_node_id;

        if (context_node_type == CONTEXT_ROOT_TYPE)
        {
            parameters->set_name(current_context_scene->getParentName().toStdString());
            parameters->set_description("Description of my plan");
            continue;
        }
        
        auto stage_scene_id = current_context_scene->getChildSceneId(context_node_id);
        auto current_stage_scene = stage_scenes_[stage_scene_id];
        // qDebug() << "Processing stage scene with id: " << stage_scene_id;
        
        auto context_config = context_configs->add_context_config();
        // TODO: fill parameters
        context_config->mutable_parameters();
        context_config->set_type(QStringToBTreeContextType(context_node_type));

        auto stage_config = stage_configs->add_stage_config();
        auto stage_fsm = context_config->mutable_stage_fsm();

        // TODO: remove root node search 
        // search is done to make sure root node is always processed first

        for (auto* stage_node : current_stage_scene->allNodes())
        {   
            auto* stage_node_data = dynamic_cast<StageDataModel*>(stage_node->nodeDataModel());
            auto stage_node_type = stage_node_data->getNodeType();

            if (stage_node_type == STAGE_ROOT_TYPE)
            {
                auto root_out_list = stage_node_data->getOutList();
                if (root_out_list.size())
                {
                    // TODO: fix string concat for initial stage
                    QStringList parts = root_out_list.at(0).toLower().split(' ', QString::SkipEmptyParts);
                    QString stage_string;
                    for (int i = 0; i < parts.size() - 1; ++i) stage_string += parts[i] + " ";
                    auto stage_type = QStringToBTreeStageType(stage_string);
                    stage_fsm->set_initial_stage(stage_type);
                }
                break;
            }
        }

        for (auto* stage_node : current_stage_scene->allNodes())
        {
            auto* stage_node_data = dynamic_cast<StageDataModel*>(stage_node->nodeDataModel());
            auto stage_node_id = stage_node_data->getNodeId();
            auto stage_node_type = stage_node_data->getNodeType();

            // qDebug() << "Processing stage node with id: " << stage_node_id;

            if (stage_node_type == STAGE_ROOT_TYPE)
            {
                continue;
            }
            
            auto btree_scene_id = current_stage_scene->getChildSceneId(stage_node_id);
            auto current_btree_scene = btree_scenes_[btree_scene_id];
            auto stage_type = QStringToBTreeStageType(stage_node_type);

            // TODO: fill transitions according to real FSM
            stage_fsm->add_stage(stage_type);

            std::vector<BTreeStageState> self_transition_states = {
                BTreeStageState::STAGE_NOT_INITIALIZED,
                BTreeStageState::STAGE_INITIALIZED,
                BTreeStageState::STAGE_FAILED,
                BTreeStageState::STAGE_RUNNING,
            };

            for (auto self_state: self_transition_states)
            {
                auto transition = stage_fsm->add_transition(); 
                transition->set_on_state(self_state);
                transition->set_from_stage(stage_type);
                transition->set_to_stage(stage_type);
            }
            
            auto transition = stage_fsm->add_transition(); 
            transition->set_on_state(BTreeStageState::STAGE_DONE);
            transition->set_from_stage(stage_type);
            auto node_out_list = stage_node_data->getOutList();
            if (node_out_list.size())
            {
                // TODO: fix string concat for stage type
                QStringList parts = node_out_list.at(0).toLower().split(' ', QString::SkipEmptyParts);
                QString to_stage_string;
                for (int i = 0; i < parts.size() - 1; ++i) to_stage_string += parts[i] + " ";
                auto to_stage_type = QStringToBTreeStageType(to_stage_string);
               
                transition->set_to_stage(to_stage_type);
            }
            else
            {
                // Set self on success (to cycle in the last stage)
                // transition->set_to_stage(stage_type);

                // Set initial stage on success (to go back to the first stage)
                transition->set_to_stage(stage_fsm->initial_stage());
            }

            // TODO: fill parameters
            stage_config->mutable_parameters();
            stage_config->set_type(stage_type);

            current_btree_scene->generateProtobuf(stage_config->mutable_tree());
        }
    }
    
    SetProtoToASCIIFile(btplan, file_name.toStdString());
}