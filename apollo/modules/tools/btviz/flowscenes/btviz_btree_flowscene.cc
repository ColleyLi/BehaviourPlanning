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

#include "modules/tools/btviz/flowscenes/btviz_btree_flowscene.h"

BTVizBTreeFlowScene::BTVizBTreeFlowScene(std::shared_ptr<QtNodes::DataModelRegistry> registry,
                                 QObject * parent): FlowScene(registry, parent)
{
}

void BTVizBTreeFlowScene::generateProtobuf(BTreeConfig* tree_config)
{
    for (auto* node : allNodes())
    {
        auto* node_data = dynamic_cast<BTreeDataModel*>(node->nodeDataModel());
        if (node_data->getNodeType() == BTREE_ROOT_TYPE)
        {
            std::vector<QString> children_ids = node_data->getChildren();
            if (children_ids.size())
            {
                tree_config->set_root_node_id(children_ids.at(0).toStdString());
            }
            continue;
        }

        auto pb_node = tree_config->add_node();
        pb_node->set_id(node_data->getNodeId().toStdString());
        pb_node->set_name(node_data->getNodeName().toStdString());
        pb_node->set_type(QStringToBTreeNodeType(node_data->getNodeType()));

        // TODO: add config
        auto config = pb_node->mutable_config();
        config->mutable_default_config();


        std::vector<QString> children_ids = node_data->getChildren();
        for (auto const& child_id: children_ids)
        {
            pb_node->add_child_id(child_id.toStdString());
        } 
    }
}