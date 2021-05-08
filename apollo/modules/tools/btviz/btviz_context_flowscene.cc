#include <QMimeData>
#include <QStandardItemModel>
#include <QVariant>
#include <QGraphicsSceneDragDropEvent>
#include <QKeyEvent>
#include <QCursor>
#include <QApplication>
#include <QGraphicsView>
#include <QtCore/QFile>
#include <QtWidgets/QFileDialog>
#include <Node.h>

#include "modules/tools/btviz/models/context_data_model.h"
#include "modules/tools/btviz/btviz_context_flowscene.h"

static int getUid()
{
    static int uid = 0;
    return uid++;
}

BTvizContextFlowScene::BTvizContextFlowScene(std::shared_ptr<QtNodes::DataModelRegistry> registry,
                                 QObject * parent):
    FlowScene(registry, parent),
    locked_(false)
{
}

QtNodes::Node& BTvizContextFlowScene::createNodeAtPosition(const QString& node_type, const QPointF& scene_pos) 
{
    QString node_id = node_type + " ";
    node_id.append(QString::number(getUid()));
    auto node = registry().create(node_type);
    if(!node)
    {
        char buffer[250];
        sprintf(buffer, "No registered node with type: [%s]", node_type.toStdString().c_str());
        throw std::runtime_error(buffer);
    }
    auto node_ptr = dynamic_cast<ContextDataModel*>(node.get());
    node_ptr->setNodeId(node_id);
    node_ptr->Init();
    auto& node_qt = createNode(std::move(node));
    setNodePosition(node_qt, scene_pos);

    return node_qt;
}

QtNodes::Node& BTvizContextFlowScene::createNodeAtPosition(const QString& node_type, const QString& this_scene_id, const QString& btplan_id, const QString& btplan_name, const QPointF& scene_pos)    
{
    QString node_id = node_type + " ";
    node_id.append(QString::number(getUid()));
    auto node = registry().create(node_type);
    if(!node)
    {
        char buffer[250];
        sprintf(buffer, "No registered node with type: [%s]", node_type.toStdString().c_str());
        throw std::runtime_error(buffer);
    }
    auto node_ptr = dynamic_cast<ContextDataModel*>(node.get());
    node_ptr->setNodeId(node_id);
    node_ptr->Init();
    node_ptr->setNodeName(btplan_name);
    // node_ptr->setBTplanName(btplan_name);
    // node_ptr->setBTplanId(btplan_id);
    auto& node_qt = createNode(std::move(node));
    setNodePosition(node_qt, scene_pos);

    return node_qt;
}

void BTvizContextFlowScene::saveProtobuf() const
{
    QString file_name =
    QFileDialog::getSaveFileName(nullptr,
                                 tr("Open Context Protobuf"),
                                 QDir::homePath(),
                                 tr("Context Protobuf Files (*.pb.txt)"));

    if (!file_name.isEmpty())
    {
        if (!file_name.endsWith("pb.txt", Qt::CaseInsensitive))
            file_name += ".pb.txt";

        qDebug() << "Will save to: " << file_name;

        generateProtobuf(file_name);
    }
}

void BTvizContextFlowScene::generateProtobuf(const QString& file_name) const
{
    BTreeConfig tree;
    // for (auto* node : allNodes())
    // {
    //     auto* node_data = dynamic_cast<BTreeDataModel*>(node->nodeDataModel());
    //     if (node_data->getNodeType() == "Root")
    //     {
    //         std::vector<QString> children_ids = node_data->getChildren();
    //         if (children_ids.size())
    //         {
    //             tree.set_root_node_id(children_ids.at(0).toStdString());
    //         }
    //         continue;
    //     }

    //     auto pb_node = tree.add_node();
    //     pb_node->set_id(node_data->getNodeId().toStdString());
    //     pb_node->set_name(node_data->getNodeName().toStdString());
    //     pb_node->set_type(QStringToBTreeNodeType(node_data->getNodeType()));

    //     std::vector<QString> children_ids = node_data->getChildren();
    //     for (auto const& child_id: children_ids)
    //     {
    //         pb_node->add_child_id(child_id.toStdString());
    //     } 
    // }
    SetProtoToASCIIFile(tree, file_name.toStdString());
}