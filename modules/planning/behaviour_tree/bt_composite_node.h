#pragma once

#include "modules/planning/behaviour_tree/bt_node.h"

namespace apollo
{
namespace planning
{

class BTCompositeNode: public BTNode
{
  public: 
    void AddChild(BTNode *child)
    {
      children_.push_back(child);
    }

   std::vector<BTNode*>& GetChildren()
   {
     return children_;
   }

  private:
    std::vector<BTNode*> children_;

};

}
}