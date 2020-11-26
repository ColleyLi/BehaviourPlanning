#ifndef MY_PLANNER_H
#define MY_PLANNER_H

#include <iostream>
#include "cyber/cyber.h"
#include "my_cpp/planner/lib/common/my_planning_gflags.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/routing/proto/routing.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h" 

namespace apollo
{
namespace my_planner
{

class MyPlanner
{
  public:
    MyPlanner() = default;
    ~MyPlanner() = default;

    void Init();
    void Plan();

  private:
  	std::unique_ptr<apollo::cyber::Node> node_;
    
    std::shared_ptr<cyber::Reader<routing::RoutingResponse>> routing_reader_;
    std::shared_ptr<cyber::Reader<localization::LocalizationEstimate>> localization_reader_;
    std::shared_ptr<cyber::Reader<canbus::Chassis>> chassis_reader_;
    std::shared_ptr<cyber::Reader<prediction::PredictionObstacles>> obstacles_reader_;
    
    std::shared_ptr<cyber::Writer<planning::ADCTrajectory>> planning_writer_;
    std::shared_ptr<cyber::Writer<routing::RoutingRequest>> rerouting_writer_;

    std::mutex mutex_;

    routing::RoutingResponse routing_;
    localization::LocalizationEstimate localization_estimate_;
    canbus::Chassis chassis_;
    prediction::PredictionObstacle obstacles_;

    
};

} // namespace my_planner
} // namespace apollo


#endif
