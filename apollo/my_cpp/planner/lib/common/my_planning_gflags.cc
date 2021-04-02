#include "my_cpp/planner/lib/common/my_planning_gflags.h"

#include <limits>

DEFINE_string(routing_response_topic, "/apollo/routing_response",
              "routing response topic name");
DEFINE_string(routing_request_topic, "/apollo/routing_request", "routing request topic name");
DEFINE_string(localization_topic, "/apollo/localization/pose",
                  "localization topic name");
DEFINE_string(chassis_topic, "/apollo/canbus/chassis", "chassis topic name");
DEFINE_string(prediction_topic, "/apollo/prediction", "prediction topic name");
DEFINE_string(planning_trajectory_topic, "/apollo/planning", "planning trajectory topic name");
