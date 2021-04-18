#include "modules/planning/behaviour_tree/b_tree_node.h"

namespace apollo {    
namespace planning {
namespace behaviour_tree {

class BTreeSequence: public BTreeNode
{
  public:
    BTreeNodeState Init(const BTreeNodeConfig& config)
    {
      config_ = config;
      state_ = BTreeNodeState::NODE_INITIALIZED;

      return state_;
    }

    BTreeNodeState Execute(Frame* frame)
    {
      for(auto child : GetChildren())
      {
        if(!(child->Execute(frame) == BTreeNodeState::NODE_DONE))
        {
          std::string msg("Task failed: ");
          msg += child->GetName();
          ADEBUG << msg;
          state_ = BTreeNodeState::NODE_FAILED;
          return state_;
        }
      }
      state_ = BTreeNodeState::NODE_DONE;
      return state_;
    }

    BTreeNodeState Execute(Frame* frame, ReferenceLineInfo* reference_line_info)
    {
      for(auto child : GetChildren())
      {
        if(!(child->Execute(frame, reference_line_info) == BTreeNodeState::NODE_DONE))
        {
          std::string msg("Task failed: ");
          msg += child->GetName();
          ADEBUG << msg;
          state_ = BTreeNodeState::NODE_FAILED;
          return state_;
        }
      }
      state_ = BTreeNodeState::NODE_DONE;
      return state_;
    }
};

} // namespace behaviour_tree
} // namespace planning
} // namespace apollo