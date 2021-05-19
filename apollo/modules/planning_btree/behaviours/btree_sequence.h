#pragma once

#include "modules/planning_btree/behaviours/btree_node.h"

namespace apollo {    
namespace planning_btree {

class BTreeSequence: public BTreeNode
{
  public:
    BTreeNodeState Init(const BTreeNodeConfig& config)
    {
      config_ = config;
      state_ = BTreeNodeState::NODE_INITIALIZED;

      return state_;
    }

    BTreeNodeState Execute(BTreeFrame* frame)
    {
      for(auto child : GetChildren())
      {
        if(!(child->Execute(frame) == BTreeNodeState::NODE_DONE))
        {
          std::string msg("Child of sequence failed: ");
          msg += child->GetName();
          ADEBUG << msg;
          state_ = BTreeNodeState::NODE_FAILED;
          return state_;
        }
      }

      ADEBUG << "All children of sequence '" << GetName() << "' are done";
      state_ = BTreeNodeState::NODE_DONE;
      return state_;
    }

    // BTreeNodeState Execute(BTreeFrame* frame, ReferenceLineInfo* reference_line_info)
    // {
    //   for(auto child : GetChildren())
    //   {
    //     if(!(child->Execute(frame, reference_line_info) == BTreeNodeState::NODE_DONE))
    //     {
    //       std::string msg("Task failed: ");
    //       msg += child->GetName();
    //       ADEBUG << msg;
    //       state_ = BTreeNodeState::NODE_FAILED;
    //       return state_;
    //     }
    //   }
    //   state_ = BTreeNodeState::NODE_DONE;
    //   return state_;
    // }
};

} // namespace planning_btree
} // namespace apollo