#pragma once

#include "modules/planning_btree/behaviours/btree_task.h"
#include "modules/planning_btree/behaviours/tasks/speed_generator/gridded_path_time_graph.h"

namespace apollo {
namespace planning_btree {

class SpeedGenerator : public BTreeTask {
 public:
  BTreeNodeState Init(const BTreeNodeConfig& config);
  BTreeNodeState Execute(BTreeFrame* frame);

 private:
  bool GenerateConstantSpeed(BTreeFrame* frame,
                             DynamicReferenceLine* dynamic_reference_line);

  bool DPSTGraphOptimizer(BTreeFrame* frame,
                          DynamicReferenceLine* dynamic_reference_line);
  bool SearchPathTimeGraph(DynamicReferenceLine* dynamic_reference_line,
                           SpeedData* speed_data,
                           const common::TrajectoryPoint& init_point) const;
};

}  // namespace planning_btree
}  // namespace apollo