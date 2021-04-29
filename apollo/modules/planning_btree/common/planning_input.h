#pragma once

#include <memory>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/map/relative_map/proto/navigation.pb.h"
// #include "modules/perception/proto/traffic_light_detection.pb.h"
// #include "modules/planning/proto/pad_msg.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"
// #include "modules/storytelling/proto/story.pb.h"

namespace apollo {
namespace planning_btree {

struct PlanningInput 
{
  std::shared_ptr<prediction::PredictionObstacles> prediction_obstacles;
  std::shared_ptr<canbus::Chassis> chassis;
  std::shared_ptr<localization::LocalizationEstimate> localization_estimate;
  // std::shared_ptr<perception::TrafficLightDetection> traffic_light;
  std::shared_ptr<routing::RoutingResponse> routing;
  // std::shared_ptr<relative_map::MapMsg> relative_map;
  // std::shared_ptr<PadMessage> pad_msg;
  // std::shared_ptr<storytelling::Stories> stories;
};

}  // namespace planning
}  // namespace apollo
