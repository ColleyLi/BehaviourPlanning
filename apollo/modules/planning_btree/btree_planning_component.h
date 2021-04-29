#pragma once

#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/message/raw_message.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/localization.pb.h"
// #include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"
// #include "modules/planning/common/message_process.h"
// #include "modules/planning/common/planning_gflags.h"
#include "modules/planning_btree/btree_planning_base.h"
// #include "modules/planning/proto/learning_data.pb.h"
// #include "modules/planning/proto/pad_msg.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning_btree/proto/btree_planning_config.pb.h"


namespace apollo {
namespace planning_btree {

class BTreePlanningComponent final
    : public cyber::Component<prediction::PredictionObstacles, 
    						  canbus::Chassis,
                              localization::LocalizationEstimate> {
 public:
  BTreePlanningComponent() = default;

  ~BTreePlanningComponent() = default;

 public:
  bool Init() override;

  bool Proc(const std::shared_ptr<prediction::PredictionObstacles>&
                prediction_obstacles,
            const std::shared_ptr<canbus::Chassis>& chassis,
            const std::shared_ptr<localization::LocalizationEstimate>&
                localization_estimate) override;

 private:
  void CheckRerouting();
  bool CheckInput();

 private:
  std::shared_ptr<cyber::Reader<perception::TrafficLightDetection>>
      traffic_light_reader_;
  std::shared_ptr<cyber::Reader<routing::RoutingResponse>> routing_reader_;
//   std::shared_ptr<cyber::Reader<planning::PadMessage>> pad_msg_reader_;
//   std::shared_ptr<cyber::Reader<relative_map::MapMsg>> relative_map_reader_;

  std::shared_ptr<cyber::Writer<planning::ADCTrajectory>> planning_writer_;
  std::shared_ptr<cyber::Writer<routing::RoutingRequest>> rerouting_writer_;
  // std::shared_ptr<cyber::Writer<PlanningLearningData>>
      // planning_learning_data_writer_;

  std::mutex mutex_;
//   perception::TrafficLightDetection traffic_light_;
  routing::RoutingResponse routing_;
  // planning::PadMessage pad_msg_;
//   relative_map::MapMsg relative_map_;

  PlanningInput planning_input_;

  std::unique_ptr<BTreePlanningBase> planning_base_;

  BTreePlanningConfig config_;
  // MessageProcess message_process_;
};

CYBER_REGISTER_COMPONENT(BTreePlanningComponent)

}  // namespace planning_btree
}  // namespace apollo
