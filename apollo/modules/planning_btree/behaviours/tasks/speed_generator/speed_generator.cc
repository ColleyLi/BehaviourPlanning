#include "modules/planning_btree/behaviours/tasks/speed_generator/speed_generator.h"

namespace apollo {
namespace planning_btree {

BTreeNodeState SpeedGenerator::Init(const BTreeNodeConfig& config) {
  config_ = config;

  state_ = BTreeNodeState::NODE_INITIALIZED;
  return state_;
}

BTreeNodeState SpeedGenerator::Execute(BTreeFrame* frame) {
  auto dynamic_reference_line = frame->GetMutableCurrentDynamicReferenceLine();

  // if (!GenerateConstantSpeed(frame, dynamic_reference_line)) {
  //   state_ = BTreeNodeState::NODE_FAILED;
  //   return state_;
  // }

  if (!DPSTGraphOptimizer(frame, dynamic_reference_line)) {
    AERROR << "DPSTGraph optimization failed";
    state_ = BTreeNodeState::NODE_FAILED;
    return state_;
  }

  state_ = BTreeNodeState::NODE_DONE;
  return state_;
}

bool SpeedGenerator::DPSTGraphOptimizer(
    BTreeFrame* frame, DynamicReferenceLine* dynamic_reference_line) {
  auto init_point = frame->GetPlanningStartPoint();
  auto path_data = dynamic_reference_line->GetPathData();
  SpeedData speed_data;

  if (path_data.discretized_path().empty()) {
    const std::string msg = "Empty path data";
    AERROR << msg;
    return false;
  }

  if (!SearchPathTimeGraph(dynamic_reference_line, &speed_data, init_point)) {
    const std::string msg = "Failed to search graph with dynamic programming";
    AERROR << msg;
    return false;
  }

  ADEBUG << "Speed data size: " << speed_data.size();

  *(dynamic_reference_line->GetMutableSpeedData()) = speed_data;

  ADEBUG << "Set speed data for lane: "
         << dynamic_reference_line->GetRouteSegments().Id();
  dynamic_reference_line->SetDrivable(true);

  return true;
}

bool SpeedGenerator::SearchPathTimeGraph(
    DynamicReferenceLine* dynamic_reference_line, SpeedData* speed_data,
    const common::TrajectoryPoint& init_point) const {
  GriddedPathTimeGraph st_graph(
      dynamic_reference_line->GetSTGraphData(),
      dynamic_reference_line->GetMutableObstacleDecisions()->obstacles().Items(),
      init_point);

  // auto st_graph_data = dynamic_reference_line->GetSTGraphData();
  // AERROR << "Size of st graph st boundaries: " << st_graph_data.st_boundaries().size();
  // AERROR << "Size of st graph speed limit: " << st_graph_data.speed_limit().speed_limit_points().size();
  // AERROR << "St graph total time: " << st_graph_data.total_time_by_conf() << " St graph path length: "
  // << st_graph_data.path_length();
  // AERROR << " INitial point: " << init_point.DebugString();
  // AERROR << "Obstacles: " <<  dynamic_reference_line->GetMutableObstacleDecisions()->obstacles().Items().size();
  // AERROR << "St drivable boundary: " <<  st_graph_data.st_drivable_boundary().DebugString();


  if (!st_graph.Search(speed_data)) {
    AERROR << "failed to search graph with dynamic programming.";
    return false;
  }

  return true;
}

bool SpeedGenerator::GenerateConstantSpeed(
    BTreeFrame* frame, DynamicReferenceLine* dynamic_reference_line) {
  SpeedData speed_data;

  double dt = 0.1;
  double v = 6.0;
  double planning_time_horizon = 10.0;
  for (double t = 0; t < planning_time_horizon; t += dt) {
    speed_data.AppendSpeedPoint(t * v, t, v, 0.0, 0.0);
  }

  *(dynamic_reference_line->GetMutableSpeedData()) = speed_data;

  AERROR << "Set speed data for lane: "
         << dynamic_reference_line->GetRouteSegments().Id();
  dynamic_reference_line->SetDrivable(true);

  return true;
}

}  // namespace planning_btree
}  // namespace apollo