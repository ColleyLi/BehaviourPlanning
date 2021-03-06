#pragma once

#include <QtWidgets/QWidget>
#include <QDebug>

#include "PortType.h"
#include "NodeData.h"
#include "Serializable.h"
#include "NodeGeometry.h"
#include "NodeStyle.h"
#include "NodePainterDelegate.h"
#include "Export.h"
#include "memory.h"

#include "modules/tools/btviz/btviz_base.h"

namespace QtNodes
{

enum class NodeValidationState
{
  Valid,
  Warning,
  Error
};

class Connection;

class StyleCollection;

class NODE_EDITOR_PUBLIC NodeDataModel
  : public QObject
  , public Serializable
{
  Q_OBJECT

public:

  NodeDataModel(const BTVizNode& node);

  virtual
  ~NodeDataModel() = default;

  virtual void
  Init() {};

  /// Caption is used in GUI
  virtual QString
  caption() const { return QString(); }

  /// It is possible to hide caption in GUI
  virtual bool
  captionVisible() const { return false; }

  /// Port caption is used in GUI to label individual ports
  virtual QString
  portCaption(PortType, PortIndex) const { return QString(); }

  /// It is possible to hide port caption in GUI
  virtual bool
  portCaptionVisible(PortType, PortIndex) const { return false; }

  /// Name makes this model unique
  virtual QString
  name() const  {return node_.id; }

public:

  QJsonObject
  save() const override;

public:

  virtual
  unsigned int
  nPorts(PortType portType) const = 0;

  /// Return if ports are dynamics
  virtual
  bool
  hasDynamicPorts(PortType) const { return false; }

  virtual
  NodeDataType
  dataType(PortType portType, PortIndex portIndex) const = 0;

public:

  enum class ConnectionPolicy
  {
    One,
    Many
  };

  ConnectionPolicy
  portConnectionPolicy(PortType portType, PortIndex portIndex) const;

  virtual
  ConnectionPolicy
  portOutConnectionPolicy(PortIndex) const
  {
    return ConnectionPolicy::One;
  }

  virtual
  ConnectionPolicy
  portInConnectionPolicy(PortIndex) const
  {
    return ConnectionPolicy::One;
  }

  NodeStyle const&
  nodeStyle() const;

  void
  setNodeStyle(NodeStyle const& style);
  
  void setNodeId(const QString& id);

  const QString& getNodeId();

  virtual void setNodeName(const QString& name);

  const QString& getNodeName();
  
  const QString& getNodeType();
  
  const QString& getNodeCategory();


public:

  /// Triggers the algorithm
  virtual
  void
  setInData(std::shared_ptr<NodeData> nodeData,
            PortIndex port) = 0;

  virtual
  void
  setInData(std::vector<std::shared_ptr<NodeData> > nodeData,
            PortIndex port);

  virtual
  std::shared_ptr<NodeData>
  outData(PortIndex port) = 0;

  virtual
  QWidget *
  embeddedWidget() = 0;

  virtual
  bool
  resizable() const { return false; }

  virtual
  NodeValidationState
  validationState() const { return NodeValidationState::Valid; }

  virtual
  QString
  validationMessage() const { return QString(""); }

  virtual
  NodePainterDelegate*
  painterDelegate() const { return nullptr; }

public Q_SLOTS:

  virtual void
  inputConnectionCreated(Connection const&) {}

  virtual void
  inputConnectionDeleted(Connection const&) {}

  virtual void
  outputConnectionCreated(Connection const&) {}

  virtual void
  outputConnectionDeleted(Connection const&) {}

Q_SIGNALS:

  void
  dataUpdated(PortIndex index);

  void
  dataInvalidated(PortIndex index);

  void
  computingStarted();

  void
  computingFinished();

  void
  embeddedWidgetSizeUpdated();

  void
  portAdded(PortType type, PortIndex index);

  void
  portMoved(PortType type, PortIndex oldIndex, PortIndex newIndex);

  void
  portRemoved(PortType type, PortIndex index);

protected:
  BTVizNode node_;

private:

  NodeStyle _nodeStyle;
};
}
