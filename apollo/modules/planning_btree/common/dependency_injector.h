#pragma once

#include "modules/common/vehicle_state/vehicle_state_provider.h"
// #include "modules/planning/common/ego_info.h"
// #include "modules/planning/common/frame.h"
// #include "modules/planning/common/history.h"
// #include "modules/planning/common/learning_based_data.h"
#include "modules/planning_btree/proto/btree_planning_state.pb.h"

namespace apollo {
namespace planning_btree {

class DependencyInjector {
 public:
  DependencyInjector() = default;
  ~DependencyInjector() = default;

  BTreePlanningState* planning_state() {
    return &planning_state_;
  }
//   FrameHistory* frame_history() {
//     return &frame_history_;
//   }
//   History* history() {
//     return &history_;
//   }
//   EgoInfo* ego_info() {
//     return &ego_info_;
//   }
  apollo::common::VehicleStateProvider* vehicle_state_provider() {
    return &vehicle_state_provider_;
  }
//   LearningBasedData* learning_based_data() {
//     return &learning_based_data_;
//   }

 private:
  BTreePlanningState planning_state_;
//   FrameHistory frame_history_;
//   History history_;
//   EgoInfo ego_info_;
  apollo::common::VehicleStateProvider vehicle_state_provider_;
//   LearningBasedData learning_based_data_;
};

}  // namespace planning_btree
}  // namespace apollo
