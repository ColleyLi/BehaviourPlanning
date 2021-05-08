#include "NodeDataModel.h"

#include "StyleCollection.h"

using QtNodes::NodeDataModel;
using QtNodes::NodeStyle;
using QtNodes::NodeData;
using QtNodes::PortIndex;
using QtNodes::PortType;

NodeDataModel::
NodeDataModel()
  : _nodeStyle(StyleCollection::nodeStyle())
{
  // Derived classes can initialize specific style here
}


QJsonObject
NodeDataModel::
save() const
{
  QJsonObject modelJson;

  modelJson["name"] = name();

  // if ports are dynamics, write their value when saved.
  // when restored, model need to update the dynamic value.

  if(hasDynamicPorts(PortType::In))
  {
     modelJson["dynamic_inputs"]  = static_cast<int>(nPorts(PortType::In));
  }

  if(hasDynamicPorts(PortType::Out))
  {
    modelJson["dynamic_outputs"]  = static_cast<int>(nPorts(PortType::Out));
  }

  return modelJson;
}


NodeDataModel::ConnectionPolicy
NodeDataModel::
portConnectionPolicy(PortType portType, PortIndex portIndex) const
{
  ConnectionPolicy result = ConnectionPolicy::One;

  switch (portType)
  {
    case PortType::In:
      result = portInConnectionPolicy(portIndex);
      break;

    case PortType::Out:
      result = portOutConnectionPolicy(portIndex);
      break;

    case PortType::None:
    default:
      break;
  }

  return result;
}


NodeStyle const&
NodeDataModel::
nodeStyle() const
{
  return _nodeStyle;
}


void
NodeDataModel::
setNodeStyle(NodeStyle const& style)
{
  _nodeStyle = style;
}


void
NodeDataModel::
setInData(std::vector<std::shared_ptr<NodeData> > nodeData, PortIndex port)
{
  if (portInConnectionPolicy(port) == QtNodes::NodeDataModel::ConnectionPolicy::One)
  {
    if (nodeData.empty())
      setInData(nullptr, port);
    else
      setInData(nodeData[0], port);
  }
  else
  {
    Q_ASSERT(false);
  }
}
