#include "my_cpp/planner/lib/my_planner.h"

namespace apollo
{
namespace my_planner
{

using apollo::routing::RoutingResponse;
using apollo::routing::RoutingRequest;
using apollo::localization::LocalizationEstimate;
using apollo::canbus::Chassis;
using apollo::prediction::PredictionObstacles;
using apollo::planning::ADCTrajectory;

void MyPlanner::Init()
{
	std::cout << "Init" << std::endl;

	node_ = apollo::cyber::CreateNode("my_planner");

	routing_reader_ = node_->CreateReader<RoutingResponse>(
      FLAGS_routing_response_topic,
      [this](const std::shared_ptr<RoutingResponse>& routing)
      {
        AINFO << "Received routing data. Running RoutingResponse callback";
        {
          std::lock_guard<std::mutex> lock(mutex_);
          routing_.CopyFrom(*routing);
        }
        this->Plan();
      });

	localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      FLAGS_localization_topic,
      [this](const std::shared_ptr<LocalizationEstimate>& localization_estimate)
      {
        AINFO << "Received localization data. Running LocalizationEstimate callback";
        std::lock_guard<std::mutex> lock(mutex_);
        localization_estimate_.CopyFrom(*localization_estimate);
      });

	chassis_reader_ = node_->CreateReader<Chassis>(
      FLAGS_chassis_topic,
      [this](const std::shared_ptr<Chassis>& chassis)
      {
        AINFO << "Received chassis data. Running Chassis callback";
        std::lock_guard<std::mutex> lock(mutex_);
        chassis_.CopyFrom(*chassis);
      });

  planning_writer_ = 
    node_->CreateWriter<ADCTrajectory>(FLAGS_planning_trajectory_topic);

  rerouting_writer_ = 
    node_->CreateWriter<RoutingRequest>(FLAGS_routing_request_topic);



	obstacles_reader_ = node_->CreateReader<PredictionObstacles>(
      FLAGS_prediction_topic,
      [this](const std::shared_ptr<PredictionObstacles>& obstacles)
      {
        AINFO << "Received obstacles data. Running PredictionObstacles callback";
        std::lock_guard<std::mutex> lock(mutex_);
        obstacles_.CopyFrom(*obstacles);
      });
}

void MyPlanner::Plan()
{
  std::cout << "Planning" << std::endl;
}


} // namespace my_planner
} // namespace apollo
