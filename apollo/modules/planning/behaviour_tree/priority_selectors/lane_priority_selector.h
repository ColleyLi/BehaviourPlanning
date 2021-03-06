#include "modules/planning/behaviour_tree/b_tree_node.h"

namespace apollo {    
namespace planning {
namespace behaviour_tree {

class LanePrioritySelector: public BTreeNode
{
  public:
    BTreeNodeState Init(const BTreeNodeConfig& config)
    {
      config_ = config;
      state_ = BTreeNodeState::NODE_INITIALIZED;
      return state_;
    }

    BTreeNodeState Execute(Frame* frame, ReferenceLineInfo* reference_line_info)
    {
      return Execute(frame);
    }

    BTreeNodeState Execute(Frame* frame)
    {
      std::list<ReferenceLineInfo>* ref_lines = frame->mutable_reference_line_info();
      ref_lines->sort([](const ReferenceLineInfo& l1, const ReferenceLineInfo& l2){return l1.Cost() < l2.Cost();});

      for(auto& ref_line : *frame->mutable_reference_line_info())
      {
        for(auto child : GetChildren())
        {
          if(child->Execute(frame, &ref_line) == BTreeNodeState::NODE_DONE)
          {
            state_ = BTreeNodeState::NODE_DONE;
            return state_;
          }
        }
      }

      std::string msg("All tasks failed");
      ADEBUG << msg;
      state_ = BTreeNodeState::NODE_FAILED;
      return state_;
    }
};

} // namespace behaviour_tree
} // namespace planning
} // namespace apollo