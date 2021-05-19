#pragma once

#include <QtCore/QUuid>
#include <QtCore/QJsonDocument>
#include <QtWidgets/QGraphicsScene>

#include <unordered_map>
#include <tuple>
#include <functional>

#include "QUuidStdHash.h"
#include "QStringStdHash.h"
#include "Export.h"
#include "DataModelRegistry.h"
#include "TypeConverter.h"
#include "memory.h"

namespace QtNodes
{

class NodeDataModel;
class FlowItemInterface;
class Node;
class NodeGraphicsObject;
class Connection;
class ConnectionGraphicsObject;
class NodeStyle;

/// Scene holds connections and nodes.
class NODE_EDITOR_PUBLIC FlowScene
  : public QGraphicsScene
{
  Q_OBJECT
public:

  FlowScene(std::shared_ptr<DataModelRegistry> registry,
            QObject * parent = Q_NULLPTR);

  FlowScene(QObject * parent = Q_NULLPTR);

  ~FlowScene() override;

  static int getUid()
  {
    static int uid = 0;
    return uid++;
  }

public:

  std::shared_ptr<Connection>
  createConnection(PortType connectedPort,
                   Node& node,
                   PortIndex portIndex);

  std::shared_ptr<Connection>
  createConnection(Node& nodeIn,
                   PortIndex portIndexIn,
                   Node& nodeOut,
                   PortIndex portIndexOut,
                   TypeConverter const & converter = TypeConverter{});

  std::shared_ptr<Connection> restoreConnection(QJsonObject const &connectionJson);

  void deleteConnection(Connection& connection);

  Node&createNode(std::unique_ptr<NodeDataModel> && dataModel);

  virtual Node& createRootNode(const QString& root_node_type); 
  
  virtual Node& createNodeAtPosition(const QString& node_type, const QPointF& scene_pos);
  
  Node&restoreNode(QJsonObject const& nodeJson);

  void removeNode(Node& node);

  DataModelRegistry&registry() const;

  void setRegistry(std::shared_ptr<DataModelRegistry> registry);

  void iterateOverNodes(std::function<void(Node*)> const & visitor);

  void iterateOverNodeData(std::function<void(NodeDataModel*)> const & visitor);

  void iterateOverNodeDataDependentOrder(std::function<void(NodeDataModel*)> const & visitor);

  QPointF getNodePosition(Node const& node) const;

  void setNodePosition(Node& node, QPointF const& pos) const;

  QSizeF getNodeSize(Node const& node) const;

public:

  std::unordered_map<QUuid, std::unique_ptr<Node> > const & nodes() const;

  std::unordered_map<QUuid, std::shared_ptr<Connection> > const & connections() const;

  std::vector<Node*> allNodes() const;

  std::vector<Node*> selectedNodes() const;

public:

  void clearScene();

  /// Open a FileDialog to save the scene in a .flow file
  void save() const;

  /// Load a FileDialog to open a scene from a .flow file
  void load();
  
  // Return scene JSON object
  QJsonObject getSceneJson();

  /// Dump the scene on a JSON QByteArray
  QByteArray saveToMemory(QJsonDocument::JsonFormat format = QJsonDocument::Indented) const;

  /// Load a scene from a JSON QByteArray
  void loadFromMemory(const QByteArray& data);

  /// Load a scene from a JSON Object
  void loadFromMemory(const QJsonObject& data);

  /// Save only a subset of the nodes to memory, as well as the connections that link two nodes lying within this subset.
  QByteArray copyNodes(const std::vector<Node*> & nodes) const;

  //! Paste selected nodes to the scene replacing uuids with new ones with a certain offset from the original position.
  void pasteNodes(const QByteArray& data, const QPointF& pointOffset = QPointF(0,0));

  void setLayout( QtNodes::PortLayout layout);

  QtNodes::PortLayout layout() const;

  bool isLocked() const {return _locked;}
  void lock() {_locked = true;}
  void unlock() {_locked = false;}

  const QString& getParentSceneId() {return _parent_scene_id;}
  void setParentSceneId(const QString& scene_id) {_parent_scene_id = scene_id;}

  const QString& getParentName() {return _parent_name;}
  void setParentName(const QString& parent_name) {_parent_name = parent_name;}

  std::vector<QString> getChildSceneIds() const;
  const QString& getChildSceneId(const QString& node_id) {return _child_scene_ids[node_id];}
  void setChildSceneId(const QString& node_id, const QString scene_id) {_child_scene_ids[node_id] = scene_id;}

Q_SIGNALS:

  /**
   * @brief Node has been created but not on the scene yet.
   * @see nodePlaced()
   */
  void nodeCreated(Node &n);

  /**
   * @brief Node has been added to the scene.
   * @details Connect to this signal if need a correct position of node.
   * @see nodeCreated()
   */
  void nodePlaced(Node &n);

  void nodeDeleted(Node &n);

  void connectionCreated(Connection const &c);
  void connectionDeleted(Connection const &c);

  void nodeMoved(Node& n, const QPointF& newLocation);

  void nodeDoubleClicked(Node& n);

  void connectionHovered(Connection& c, QPoint screenPos);

  void nodeHovered(Node& n, QPoint screenPos);

  void connectionHoverLeft(Connection& c);

  void nodeHoverLeft(Node& n);

  void nodeContextMenu(Node& n, const QPointF& pos);

private:

  using SharedConnection = std::shared_ptr<Connection>;
  using UniqueNode       = std::unique_ptr<Node>;

  // DO NOT reorder this member to go after the others.
  // This should outlive all the connections and nodes of
  // the graph, so that nodes can potentially have pointers into it,
  // which is why it comes first in the class.
  std::shared_ptr<DataModelRegistry> _registry;

  std::unordered_map<QUuid, SharedConnection> _connections;
  std::unordered_map<QUuid, UniqueNode>       _nodes;

  QtNodes::PortLayout _layout;

  bool _locked;
  QString _parent_scene_id;
  QString _parent_name;

  // Mapping from node id to node's internal scene id
  // For root node -> upper-level scene
  // For child node -> lower-level scene
  std::unordered_map<QString, QString> _child_scene_ids;

private Q_SLOTS:

  void setupConnectionSignals(Connection const& c);

  void sendConnectionCreatedToNodes(Connection const& c);
  void sendConnectionDeletedToNodes(Connection const& c);

};

Node*
locateNodeAt(QPointF scenePoint, FlowScene &scene,
             QTransform const & viewTransform);
}
