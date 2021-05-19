#include "modules/planning_btree/behaviours/btree_node.h"

namespace apollo {
namespace planning_btree {

class LanePrioritySelector : public BTreeNode {
 public:
  BTreeNodeState Init(const BTreeNodeConfig& config) {
    config_ = config;
    state_ = BTreeNodeState::NODE_INITIALIZED;
    return state_;
  }

  BTreeNodeState Execute(BTreeFrame* frame) {
    std::list<DynamicReferenceLine>* ref_lines =
        frame->GetMutableDynamicReferenceLines();
    ref_lines->sort(
        [](const DynamicReferenceLine& l1, const DynamicReferenceLine& l2) {
          return l1.GetCost() < l2.GetCost();
        });

    for (auto& ref_line : *ref_lines) {
      frame->SetCurrentDynamicReferenceLine(&ref_line);
      for (auto child : GetChildren()) {
        if (child->Execute(frame) == BTreeNodeState::NODE_DONE) {
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

}  // namespace planning_btree
}  // namespace apollo