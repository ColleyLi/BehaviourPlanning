#include "Node.h"

#include <QtCore/QObject>

#include <utility>
#include <iostream>

#include "FlowScene.h"

#include "NodeGraphicsObject.h"
#include "NodeDataModel.h"

#include "ConnectionGraphicsObject.h"
#include "ConnectionState.h"

using QtNodes::Node;
using QtNodes::NodeGeometry;
using QtNodes::NodeState;
using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDataModel;
using QtNodes::NodeGraphicsObject;
using QtNodes::PortIndex;
using QtNodes::PortType;

Node::
Node(std::unique_ptr<NodeDataModel> && dataModel)
  : Node(std::move(dataModel), QUuid::createUuid())
{}

Node::
Node(std::unique_ptr<NodeDataModel> && dataModel, QUuid&& uuid)
  : _uid(std::move(uuid))
  , _nodeDataModel(std::move(dataModel))
  , _nodeState(_nodeDataModel)
  , _nodeGeometry(_nodeDataModel)
  , _nodeGraphicsObject(nullptr)
{
  _nodeGeometry.recalculateSize();

  // propagate data: model => node
  connect(_nodeDataModel.get(), &NodeDataModel::dataUpdated,
          this, &Node::onDataUpdated);

  connect(_nodeDataModel.get(), &NodeDataModel::embeddedWidgetSizeUpdated,
          this, &Node::onNodeSizeUpdated );

  connect(_nodeDataModel.get(), &NodeDataModel::portAdded,
          this, &Node::onPortAdded);

  connect(_nodeDataModel.get(), &NodeDataModel::portMoved,
          this, &Node::onPortMoved);

  connect(_nodeDataModel.get(), &NodeDataModel::portRemoved,
          this, &Node::onPortRemoved);
}


Node::
~Node() = default;

QJsonObject
Node::
save() const
{
  QJsonObject nodeJson;

  nodeJson["id"] = _uid.toString();

  nodeJson["model"] = _nodeDataModel->save();

  QJsonObject obj;
  obj["x"] = _nodeGraphicsObject->pos().x();
  obj["y"] = _nodeGraphicsObject->pos().y();
  nodeJson["position"] = obj;

  return nodeJson;
}


void
Node::
restore(QJsonObject const& json)
{
  _uid = QUuid(json["id"].toString());

  QJsonObject positionJson = json["position"].toObject();
  QPointF     point(positionJson["x"].toDouble(),
                    positionJson["y"].toDouble());
  _nodeGraphicsObject->setPos(point);

  _nodeDataModel->restore(json["model"].toObject());
}


QUuid
Node::
id() const
{
  return _uid;
}


void
Node::
reactToPossibleConnection(PortType reactingPortType,
                          NodeDataType const &reactingDataType,
                          QPointF const &scenePoint)
{
  QTransform const t = _nodeGraphicsObject->sceneTransform();

  QPointF nodePoint = t.inverted().map(scenePoint);

  _nodeGeometry.setDraggingPosition(nodePoint);

  _nodeGraphicsObject->update();

  _nodeState.setReaction(NodeState::REACTING,
                         reactingPortType,
                         reactingDataType);
}


void
Node::
resetReactionToConnection()
{
  _nodeState.setReaction(NodeState::NOT_REACTING);
  _nodeGraphicsObject->update();
}


NodeGraphicsObject const &
Node::
nodeGraphicsObject() const
{
  return *_nodeGraphicsObject.get();
}


NodeGraphicsObject &
Node::
nodeGraphicsObject()
{
  return *_nodeGraphicsObject.get();
}


void
Node::
setGraphicsObject(std::unique_ptr<NodeGraphicsObject>&& graphics)
{
  _nodeGraphicsObject = std::move(graphics);

  _nodeGeometry.recalculateSize();
}


NodeGeometry&
Node::
nodeGeometry()
{
  return _nodeGeometry;
}


NodeGeometry const&
Node::
nodeGeometry() const
{
  return _nodeGeometry;
}


NodeState const &
Node::
nodeState() const
{
  return _nodeState;
}


NodeState &
Node::
nodeState()
{
  return _nodeState;
}


NodeDataModel*
Node::
nodeDataModel() const
{
  return _nodeDataModel.get();
}


void
Node::
propagateData(PortIndex inPortIndex) const
{
  NodeState const& state = nodeState();
  NodeState::ConnectionPtrSet connections = state.connections(PortType::In, inPortIndex);

  std::vector<std::shared_ptr<NodeData>> nodeData;
  nodeData.reserve(connections.size());
  for (const auto& connection : connections)
  {
    if (Connection* c = connection.second)
    {
      Node* outNode = c->getNode(PortType::Out);
      PortIndex outNodeIndex = c->getPortIndex(PortType::Out);
      std::shared_ptr<NodeData> outData = outNode->nodeDataModel()->outData(outNodeIndex);
      auto& converter = c->typeConverter();
      if (converter != nullptr)
      {
        outData = converter(outData);
      }
      nodeData.push_back(outData);
    }
  }

  _nodeDataModel->setInData(std::move(nodeData), inPortIndex);

  //Recalculate the nodes visuals. A data change can result in the node taking more space than before, so this forces a recalculate+repaint on the affected node
  updateGraphics();
}


void
Node::
onDataUpdated(PortIndex index)
{
  auto connections = _nodeState.connections(PortType::Out, index);
  for (auto const & c : connections)
  {
    if (c.second)
      c.second->propagateData();
  }
}

void
Node::
onNodeSizeUpdated()
{
  if (nodeDataModel()->embeddedWidget())
  {
    nodeDataModel()->embeddedWidget()->adjustSize();
  }
  nodeGeometry().recalculateSize();
  _nodeGraphicsObject->moveConnections();
}

void
Node::
updateGraphics() const
{
  _nodeGraphicsObject->setGeometryChanged();
  _nodeGeometry.recalculateSize();
  _nodeGraphicsObject->update();
  _nodeGraphicsObject->moveConnections();
}

void
Node::
insertEntry(PortType portType, PortIndex index)
{
  // Insert new port
  auto& entries = _nodeState.getEntries(portType);
  entries.insert(entries.begin() + index, NodeState::ConnectionPtrSet());

  // Move subsequent port indices up by one
  for (int i = index + 1; i < static_cast<int>(entries.size()); ++i)
  {
    for (const auto& value : entries[i])
    {
      if (Connection* connection = value.second)
      {
        Node* node = connection->getNode(portType);
        if (node)
        {
           PortIndex newIndex = connection->getPortIndex(portType) + 1;
           connection->setNodeToPort(*node, portType, newIndex);
        }
      }
    }
  }
}

void
Node::
eraseEntry(PortType portType, PortIndex index)
{
  auto& entries = _nodeState.getEntries(portType);
  entries.erase(entries.begin() + index);

  // Move subsequent port indices down by one
  for (int i = index; i < static_cast<int>(entries.size()); ++i)
  {
    for (const auto& value : entries[i])
    {
       if (Connection* connection = value.second)
       {
          Node* node = connection->getNode(portType);
          if (node)
          {
            PortIndex newIndex = connection->getPortIndex(portType) - 1;
            connection->setNodeToPort(*node, portType, newIndex);
          }
       }
    }
  }
}

void
Node::
onPortAdded(PortType portType, PortIndex index)
{
  insertEntry(portType, index);

  updateGraphics();
}

void
Node::
onPortMoved(PortType portType, PortIndex oldIndex, PortIndex newIndex)
{
  auto& entries = _nodeState.getEntries(portType);
  auto connections = entries[oldIndex];

  eraseEntry(portType, oldIndex);
  insertEntry(portType, newIndex);

  updateGraphics();
}

void
Node::
onPortRemoved(PortType portType, PortIndex index)
{

  // Remove connections to this port
  auto& entries = _nodeState.getEntries(portType);
  int nPorts = _nodeDataModel->nPorts(portType);
  for (int i = nPorts; i < static_cast<int>(entries.size()); ++i)
  {
     std::vector<Connection*> connections;
     for (const auto& value : entries[index]){
        if(Connection* connection = value.second){
           connections.push_back(connection);
        }
     }

     // connections may be removed from entries in connectionRemoved()
     for (Connection* connection : connections)
       Q_EMIT connectionRemoved(*connection);
  }

  // // Remove connections to this port
  // auto& entries = _nodeState.getEntries(portType);
  // int nPorts = _nodeDataModel->nPorts(portType);
  // for (int i = nPorts; i < static_cast<int>(entries.size()); ++i)
  // {
  //   std::vector<Connection*> connections;
  //   for (const auto& value : entries[index])
  //     connections.push_back(value.second);

    // connections may be removed from entries in connectionRemoved()
    // for (Connection* connection : connections)
    // {
    //   int out_port_index = connection->getPortIndex(QtNodes::PortType::Out);
    //   int in_port_index = connection->getPortIndex(QtNodes::PortType::In);
    //   if ((portType == PortType::In) && (in_port_index == index))
    //   {
    //     continue;
    //   }
    //   if ((portType == PortType::Out) && (out_port_index == index))
    //   {
    //     continue;
    //   }
    //   Q_EMIT connectionRemoved(*connection);
    // }
  // }

  eraseEntry(portType, index);

  updateGraphics();
}
