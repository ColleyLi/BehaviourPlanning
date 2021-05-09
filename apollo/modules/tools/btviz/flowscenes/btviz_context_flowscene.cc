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

#include "modules/tools/btviz/flowscenes/btviz_context_flowscene.h"

BTVizContextFlowScene::BTVizContextFlowScene(std::shared_ptr<QtNodes::DataModelRegistry> registry,
                                 QObject * parent): FlowScene(registry, parent)
{
}

void BTVizContextFlowScene::generateProtobuf(BTPlan& btplan)
{
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
}