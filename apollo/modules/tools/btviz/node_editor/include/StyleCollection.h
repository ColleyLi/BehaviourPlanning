#pragma once

#include "NodeStyle.h"
#include "ConnectionStyle.h"
#include "FlowViewStyle.h"
#include "Export.h"

namespace QtNodes
{

class NODE_EDITOR_PUBLIC StyleCollection
{
public:

  static
  NodeStyle const&
  nodeStyle();

  static
  ConnectionStyle const&
  connectionStyle();

  static
  FlowViewStyle const&
  flowViewStyle();

public:

  static
  void
  setNodeStyle(NodeStyle);

  static
  void
  setConnectionStyle(ConnectionStyle);

  static
  void
  setFlowViewStyle(FlowViewStyle);

private:

  StyleCollection() = default;

  StyleCollection(StyleCollection const&) = delete;

  StyleCollection&
  operator=(StyleCollection const&) = delete;

  static
  StyleCollection&
  instance();

private:

  NodeStyle _nodeStyle;

  ConnectionStyle _connectionStyle;

  FlowViewStyle _flowViewStyle;
};
}
