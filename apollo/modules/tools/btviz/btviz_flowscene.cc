#include <QMimeData>
#include <QStandardItemModel>
#include <QVariant>
#include <QGraphicsSceneDragDropEvent>
#include <QKeyEvent>
#include <QCursor>
#include <QApplication>
#include <QGraphicsView>
#include <Node.hpp>

#include "modules/tools/btviz/models/btree_data_model.h"
#include "modules/tools/btviz/btviz_flowscene.h"

BTvizFlowScene::BTvizFlowScene(std::shared_ptr<QtNodes::DataModelRegistry> registry,
                                 QObject * parent):
    FlowScene(registry, parent),
    locked_(false)
{
}

QtNodes::Node& BTvizFlowScene::createNodeAtPosition(const QString& node_type, const QPointF& scene_pos) 
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
    auto node_ptr = dynamic_cast<BTreeDataModel*>(node.get());
    node_ptr->setNodeId(node_id);
    node_ptr->Init();
    auto& node_qt = createNode(std::move(node));
    setNodePosition(node_qt, scene_pos);

    return node_qt;
}

// void BTvizFlowScene::dragEnterEvent(QGraphicsSceneDragDropEvent *event)
// {
//     if(event->mimeData()->hasFormat("application/x-qabstractitemmodeldatalist"))
//     {
//         event->setAccepted(true);
//     }
// }

// void BTvizFlowScene::dragLeaveEvent(QGraphicsSceneDragDropEvent *event)
// {
    // event->acceptProposedAction();
// }

// void BTvizFlowScene::dragMoveEvent(QGraphicsSceneDragDropEvent* event)
// {
    // event->acceptProposedAction();
// }

// void BTvizFlowScene::keyPressEvent(QKeyEvent *event)
// {
    // for( const auto& it: nodes())
    // {
        // const auto& node = it.second;
        // auto line_edits = node->nodeDataModel()->embeddedWidget()->findChildren<QLineEdit*>();
        // for(auto line_edit: line_edits )
        // {
            // if( line_edit->hasFocus() )
            // {
                // Do not swallow the keyPressEvent, you are editing a QLineEdit
                // QGraphicsScene::keyPressEvent(event);
                // return;
            // }
        // }
    // }

    // const QString& registration_ID = _clipboard_node.model.registration_ID;

    // auto selected_items = selectedItems();
    // if( selected_items.size() == 1 &&
    //     event->key() == Qt::Key_C &&
    //     event->modifiers() == Qt::ControlModifier)
    // {
    //     auto node_item = dynamic_cast<QtNodes::NodeGraphicsObject*>( selected_items.front() );
    //     if( !node_item ) return;

    //     QtNodes::Node& selected_node = node_item->node();
    //     auto node_model = dynamic_cast<BTreeDataModel*>( selected_node.nodeDataModel() );
    //     if( !node_model ) return;

    //     _clipboard_node.model = node_model->model();
    //     _clipboard_node.instance_name  = node_model->instanceName();
    // }
    // else if( event->key() == Qt::Key_V &&
    //          event->modifiers() == Qt::ControlModifier &&
    //          registry().isRegistered( registration_ID  ) )
    // {
    //     auto views_ = views();
    //     QGraphicsView* view = views_.front();
    //     auto mouse_pos = view->viewport()->mapFromGlobal( QCursor::pos() );
    //     auto scene_pos = view->mapToScene( mouse_pos );

    //     createNodeAtPos( registration_ID,
    //                     _clipboard_node.instance_name,
    //                     scene_pos );
    // }
    // else{
    //     QGraphicsScene::keyPressEvent(event);
    // }
// }

// void BTvizFlowScene::dropEvent(QGraphicsSceneDragDropEvent *event)
// {
    // if(!locked_ && event->mimeData()->hasFormat("application/x-qabstractitemmodeldatalist"))
    // {
        // QByteArray encoded = event->mimeData()->data("application/x-qabstractitemmodeldatalist");
        // QDataStream stream(&encoded, QIODevice::ReadOnly);
        // QPointF scene_pos = event->scenePos();

        // while (!stream.atEnd())
        // {
            // int row, col;
            // QMap<int, QVariant> roleDataMap;
            // stream >> row >> col >> roleDataMap;

            // auto it = roleDataMap.find(0);
            // if (it != roleDataMap.end() )
            // {
                // const auto& id = it.value().toString();
                // createNodeAtPos(id, id, scene_pos);
            // }
        // }
    // }
    // event->acceptProposedAction();
// }