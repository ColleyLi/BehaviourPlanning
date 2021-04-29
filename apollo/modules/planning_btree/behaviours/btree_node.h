#pragma once

#include "modules/planning_btree/common/btree_frame.h"
#include "modules/planning_btree/proto/btree_config.pb.h"

#include <memory>
#include <vector>

namespace apollo {    
namespace planning_btree {

class BTreeNode
{
  public:
    BTreeNode():
      id_("NoId"), name_("NoName")
    {}

    BTreeNode(std::string id, std::string name):
      id_(id), name_(name)
    {}

    void SetId(std::string id)
    {
      id_ = id;
    }

    void SetName(std::string name)
    {
      name_ = name;
    }

    std::string GetId()
    {
      return id_;
    }

    std::string GetName()
    {
      return name_;
    }
    
    void AddChild(std::shared_ptr<BTreeNode> child)
    {
      children_.push_back(child);
    }

    std::vector<std::shared_ptr<BTreeNode>>& GetChildren()
    {
     return children_;
    }

    virtual BTreeNodeState Init(const BTreeNodeConfig& config) = 0;
    virtual BTreeNodeState Execute(BTreeFrame* frame) = 0;
    // virtual BTreeNodeState Execute(BTreeFrame* frame, ReferenceLineInfo* reference_line_info) = 0;

  protected:
    BTreeNodeConfig config_;
    BTreeNodeState state_ = BTreeNodeState::NODE_NOT_INITIALIZED;
    std::string id_;
    std::string name_;
    std::vector<std::shared_ptr<BTreeNode>> children_;
};

} // namespace planning_btree
} // namespace apollo