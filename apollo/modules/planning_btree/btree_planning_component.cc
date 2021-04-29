#include "modules/planning_btree/btree_planning_component.h"

#include "cyber/common/file.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/pnc_map.h"
// #include "modules/planning/common/history.h"
// #include "modules/planning/common/planning_context.h"
// #include "modules/planning/navi_planning.h"
// #include "modules/planning/on_lane_planning.h"

namespace apollo {
namespace planning_btree {

using apollo::cyber::ComponentBase;
using apollo::hdmap::HDMapUtil;
// using apollo::perception::TrafficLightDetection;
// using apollo::relative_map::MapMsg;
using apollo::routing::RoutingRequest;
using apollo::routing::RoutingResponse;
using apollo::planning::ADCTrajectory;

bool BTreePlanningComponent::Init() {
  planning_base_ = std::make_unique<BTreePlanningBase>();

  AERROR << "Init btree planning component";

  ACHECK(ComponentBase::GetProtoConfig(&config_))
      << "failed to load planning config file "
      << ComponentBase::ConfigFilePath();

  // if (FLAGS_planning_offline_learning ||
  //     config_.learning_mode() != PlanningConfig::NO_LEARNING) {
  //   if (!message_process_.Init(config_, injector_)) {
  //     AERROR << "failed to init MessageProcess";
  //     return false;
  //   }
  // }

  planning_base_->Init(config_);

  routing_reader_ = node_->CreateReader<RoutingResponse>(
      config_.topic_config().routing_response_topic(),
      [this](const std::shared_ptr<RoutingResponse>& routing) {
        AINFO << "Received routing data: run routing callback."
              << routing->header().DebugString();
        std::lock_guard<std::mutex> lock(mutex_);
        routing_.CopyFrom(*routing);
      });

  // traffic_light_reader_ = node_->CreateReader<TrafficLightDetection>(
  //     config_.topic_config().traffic_light_detection_topic(),
  //     [this](const std::shared_ptr<TrafficLightDetection>& traffic_light) {
  //       ADEBUG << "Received traffic light data: run traffic light callback.";
  //       std::lock_guard<std::mutex> lock(mutex_);
  //       traffic_light_.CopyFrom(*traffic_light);
  //     });

  // pad_msg_reader_ = node_->CreateReader<PadMessage>(
  //     config_.topic_config().planning_pad_topic(),
  //     [this](const std::shared_ptr<PadMessage>& pad_msg) {
  //       ADEBUG << "Received pad data: run pad callback.";
  //       std::lock_guard<std::mutex> lock(mutex_);
  //       pad_msg_.CopyFrom(*pad_msg);
  //     });

  // story_telling_reader_ = node_->CreateReader<Stories>(
  //     config_.topic_config().story_telling_topic(),
  //     [this](const std::shared_ptr<Stories>& stories) {
  //       ADEBUG << "Received story_telling data: run story_telling callback.";
  //       std::lock_guard<std::mutex> lock(mutex_);
  //       stories_.CopyFrom(*stories);
  //     });

  // if (FLAGS_use_navigation_mode) {
  //   relative_map_reader_ = node_->CreateReader<MapMsg>(
  //       config_.topic_config().relative_map_topic(),
  //       [this](const std::shared_ptr<MapMsg>& map_message) {
  //         ADEBUG << "Received relative map data: run relative map callback.";
  //         std::lock_guard<std::mutex> lock(mutex_);
  //         relative_map_.CopyFrom(*map_message);
  //       });
  // }
  planning_writer_ = node_->CreateWriter<ADCTrajectory>(
      config_.topic_config().planning_trajectory_topic());

  rerouting_writer_ = node_->CreateWriter<RoutingRequest>(
      config_.topic_config().routing_request_topic());

  // planning_learning_data_writer_ = node_->CreateWriter<PlanningLearningData>(
  //     config_.topic_config().planning_learning_data_topic());

  return true;
}

bool BTreePlanningComponent::Proc(
    const std::shared_ptr<prediction::PredictionObstacles>&
        prediction_obstacles,
    const std::shared_ptr<canbus::Chassis>& chassis,
    const std::shared_ptr<localization::LocalizationEstimate>&
        localization_estimate) {
  ACHECK(prediction_obstacles != nullptr);

  // check and process possible rerouting request
  // CheckRerouting();

  AERROR << "Proc";

  // process fused input data
  planning_input_.prediction_obstacles = prediction_obstacles;
  planning_input_.chassis = chassis;
  planning_input_.localization_estimate = localization_estimate;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!planning_input_.routing ||
        hdmap::PncMap::IsNewRouting(*planning_input_.routing, routing_)) {
      planning_input_.routing =
          std::make_shared<routing::RoutingResponse>(routing_);
    }
  }
  // {
  //   std::lock_guard<std::mutex> lock(mutex_);
  //   planning_input_.traffic_light =
  //       std::make_shared<TrafficLightDetection>(traffic_light_);
  //   planning_input_.relative_map = std::make_shared<MapMsg>(relative_map_);
  // }
  // {
  //   std::lock_guard<std::mutex> lock(mutex_);
  //   planning_input_.pad_msg = std::make_shared<PadMessage>(pad_msg_);
  // }
  // {
  //   std::lock_guard<std::mutex> lock(mutex_);
  //   planning_input_.stories = std::make_shared<Stories>(stories_);
  // }

  if (!CheckInput()) {
    AERROR << "Input check failed";
    return false;
  }

  // if (config_.learning_mode() != PlanningConfig::NO_LEARNING) {
  //   // data process for online training
  //   message_process_.OnChassis(*planning_input_.chassis);
  //   message_process_.OnPrediction(*planning_input_.prediction_obstacles);
  //   message_process_.OnRoutingResponse(*planning_input_.routing);
  //   message_process_.OnStoryTelling(*planning_input_.stories);
  //   message_process_.OnTrafficLightDetection(*planning_input_.traffic_light);
  //   message_process_.OnLocalization(*planning_input_.localization_estimate);
  // }

  // // publish learning data frame for RL test
  // if (config_.learning_mode() == PlanningConfig::RL_TEST) {
  //   PlanningLearningData planning_learning_data;
  //   LearningDataFrame* learning_data_frame =
  //       injector_->learning_based_data()->GetLatestLearningDataFrame();
  //   if (learning_data_frame) {
  //     planning_learning_data.mutable_learning_data_frame()
  //                           ->CopyFrom(*learning_data_frame);
  //     common::util::FillHeader(node_->Name(), &planning_learning_data);
  //     planning_learning_data_writer_->Write(planning_learning_data);
  //   } else {
  //     AERROR << "fail to generate learning data frame";
  //     return false;
  //   }
  //   return true;
  // }

  ADCTrajectory adc_trajectory_pb;
  planning_base_->Execute(planning_input_, &adc_trajectory_pb);
  common::util::FillHeader(node_->Name(), &adc_trajectory_pb);

  // modify trajectory relative time due to the timestamp change in header
  auto start_time = adc_trajectory_pb.header().timestamp_sec();
  const double dt = start_time - adc_trajectory_pb.header().timestamp_sec();
  for (auto& p : *adc_trajectory_pb.mutable_trajectory_point()) {
    p.set_relative_time(p.relative_time() + dt);
  }
  planning_writer_->Write(adc_trajectory_pb);

  // // record in history
  // auto* history = injector_->history();
  // history->Add(adc_trajectory_pb);

  return true;
}

void BTreePlanningComponent::CheckRerouting() {
  // auto* rerouting = injector_->planning_context()
  //                       ->mutable_planning_status()
  //                       ->mutable_rerouting();
  // if (!rerouting->need_rerouting()) {
    // return;
  // }
  // common::util::FillHeader(node_->Name(), rerouting->mutable_routing_request());
  // rerouting->set_need_rerouting(false);
  // rerouting_writer_->Write(rerouting->routing_request());
}

bool BTreePlanningComponent::CheckInput() {
  ADCTrajectory trajectory_pb;
  auto* not_ready = trajectory_pb.mutable_decision()
                        ->mutable_main_decision()
                        ->mutable_not_ready();

  if (planning_input_.localization_estimate == nullptr) {
    not_ready->set_reason("localization not ready");
  } 
  else if (planning_input_.chassis == nullptr) 
  {
    not_ready->set_reason("chassis not ready");
  } 
  else if (HDMapUtil::BaseMapPtr() == nullptr) {
    not_ready->set_reason("map not ready");
  } 
  else {
    // nothing
  }

  // if (FLAGS_use_navigation_mode) {
    // if (!planning_input_.relative_map->has_header()) {
      // not_ready->set_reason("relative map not ready");
    // }
  // } else {
    if (!planning_input_.routing->has_header()) {
      not_ready->set_reason("routing not ready");
    }
  // }

  if (not_ready->has_reason()) {
    AERROR << not_ready->reason() << "; skip the planning cycle.";
    common::util::FillHeader(node_->Name(), &trajectory_pb);
    planning_writer_->Write(trajectory_pb);
    return false;
  }
  return true;
}

}  // namespace planning_btree
}  // namespace apollo
