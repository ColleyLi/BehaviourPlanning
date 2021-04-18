#include "modules/planning/behaviour_tree/tasks/speed_generator/speed_generator.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/tasks/deciders/speed_bounds_decider/speed_limit_decider.h"
#include "modules/planning/tasks/deciders/speed_bounds_decider/st_boundary_mapper.h"
#include "modules/planning/tasks/optimizers/path_time_heuristic/gridded_path_time_graph.h"
#include "modules/common/status/status.h"

namespace apollo {
namespace planning {
namespace behaviour_tree {

  using common::ErrorCode;
  using common::Status;
  using apollo::common::TrajectoryPoint;
  using apollo::planning_internal::StGraphBoundaryDebug;
  using apollo::planning_internal::STGraphDebug;

  BTreeNodeState SpeedGenerator::Init(const BTreeNodeConfig& config)
  {
    config_ = config;

    auto speed_generator_config = config.speed_generator_task_config();
    speed_bounds_config_ = speed_generator_config.speed_bounds_decider_config();
    dp_st_speed_config_ = speed_generator_config.dp_st_speed_config();
    
    state_ = BTreeNodeState::NODE_INITIALIZED;
    return state_;
  }

  BTreeNodeState SpeedGenerator::Execute(Frame *frame)
  {
    state_ = BTreeNodeState::NODE_DONE;
    return state_;
  }

  BTreeNodeState SpeedGenerator::Execute(Frame* frame, ReferenceLineInfo* reference_line_info)
  {
    // state_ = ConstantSpeed(frame, reference_line_info);

    GenerateBoundaries(frame, reference_line_info);
    state_ = OptimizeSTGraph(frame, reference_line_info);

    if (state_ == BTreeNodeState::NODE_DONE)
    {
      reference_line_info->SetDrivable(true);
    }

    return state_;
  }

  BTreeNodeState SpeedGenerator::GenerateBoundaries(Frame* frame, ReferenceLineInfo* reference_line_info)
  {
    // Retrieve data from frame and reference_line_info
    const PathData &path_data = reference_line_info->path_data();
    const TrajectoryPoint &init_point = frame->PlanningStartPoint();
    const ReferenceLine &reference_line = reference_line_info->reference_line();
    PathDecision *const path_decision = reference_line_info->path_decision();

    // Map obstacles into st graph
    STBoundaryMapper boundary_mapper(
      speed_bounds_config_, reference_line, path_data,
      path_data.discretized_path().Length(), speed_bounds_config_.total_time());

    path_decision->EraseStBoundaries();
    if (boundary_mapper.ComputeSTBoundary(path_decision).code() == ErrorCode::PLANNING_ERROR) 
    {
      const std::string msg = "Mapping obstacle failed.";
      AERROR << msg;
      state_ = BTreeNodeState::NODE_FAILED;
      return state_;
    }

    std::vector<const STBoundary *> boundaries;
    for (auto *obstacle : path_decision->obstacles().Items()) 
    {
      const auto &id = obstacle->Id();
      const auto &st_boundary = obstacle->path_st_boundary();
      if (!st_boundary.IsEmpty()) 
      {
        if (st_boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) 
        {
          path_decision->Find(id)->SetBlockingObstacle(false);
        } 
        else 
        {
          path_decision->Find(id)->SetBlockingObstacle(true);
        }
        boundaries.push_back(&st_boundary);
      }
    }

    const double min_s_on_st_boundaries = SetSpeedFallbackDistance(path_decision);

    // 2. Create speed limit along path
    SpeedLimitDecider speed_limit_decider(speed_bounds_config_, reference_line, path_data);

    SpeedLimit speed_limit;
    if (!speed_limit_decider.GetSpeedLimits(path_decision->obstacles(), &speed_limit).ok())
    {
      std::string msg("Getting speed limits failed!");
      AERROR << msg;
      state_ = BTreeNodeState::NODE_FAILED;
      return state_;
    }

    // 3. Get path_length as s axis search bound in st graph
    const double path_data_length = path_data.discretized_path().Length();

    // 4. Get time duration as t axis search bound in st graph
    const double total_time_by_conf = speed_bounds_config_.total_time();

    // Load generated st graph data back to frame
    StGraphData *st_graph_data = reference_line_info->mutable_st_graph_data();

    // Add a st_graph debug info and save the pointer to st_graph_data for
    // optimizer logging
    auto *debug = reference_line_info->mutable_debug();
    STGraphDebug *st_graph_debug = debug->mutable_planning_data()->add_st_graph();

    st_graph_data->LoadData(boundaries, min_s_on_st_boundaries, init_point,
                            speed_limit, path_data_length, total_time_by_conf,
                            st_graph_debug);

    state_ = BTreeNodeState::NODE_DONE;
    return state_;
  }

double SpeedGenerator::SetSpeedFallbackDistance(PathDecision *const path_decision) 
{
  // Set min_s_on_st_boundaries to guide speed fallback. Different stop distance
  // is taken when there is an obstacle moving in opposite direction of ADV
  constexpr double kEpsilon = 1.0e-6;
  double min_s_non_reverse = std::numeric_limits<double>::infinity();
  double min_s_reverse = std::numeric_limits<double>::infinity();
  // TODO(Jinyun): tmp workaround for side pass capability because of doomed
  // speed planning failure when side pass creeping
  double side_pass_stop_s = std::numeric_limits<double>::infinity();

  for (auto *obstacle : path_decision->obstacles().Items()) 
  {
    const auto &st_boundary = obstacle->path_st_boundary();

    if (st_boundary.IsEmpty()) 
    {
      continue;
    }

    const auto left_bottom_point_s = st_boundary.bottom_left_point().s();
    const auto right_bottom_point_s = st_boundary.bottom_right_point().s();
    const auto lowest_s = std::min(left_bottom_point_s, right_bottom_point_s);

    if (left_bottom_point_s - right_bottom_point_s > kEpsilon) 
    {
      if (min_s_reverse > lowest_s) 
      {
        min_s_reverse = lowest_s;
      }
    } 
    else if (min_s_non_reverse > lowest_s) 
    {
      min_s_non_reverse = lowest_s;
    }

    if (obstacle->LongitudinalDecision().stop().reason_code() ==
            StopReasonCode::STOP_REASON_SIDEPASS_SAFETY && side_pass_stop_s > lowest_s) 
    {
      side_pass_stop_s = lowest_s;
    }
  }

  min_s_reverse = std::max(min_s_reverse, 0.0);
  min_s_non_reverse = std::max(min_s_non_reverse, 0.0);
  side_pass_stop_s = std::max(side_pass_stop_s, 0.0);

  if (!std::isinf(side_pass_stop_s)) 
  {
    return side_pass_stop_s;
  }

  return min_s_non_reverse > min_s_reverse ? 0.0 : min_s_non_reverse;
}
  
  BTreeNodeState SpeedGenerator::OptimizeSTGraph(Frame* frame, ReferenceLineInfo* reference_line_info)
  {
    SpeedData speed_data;
    const TrajectoryPoint &init_point = frame->PlanningStartPoint();
    const PathData &path_data = reference_line_info->path_data();
    
    if (path_data.discretized_path().empty()) 
    {
      std::string msg("Empty path data");
      AERROR << msg;
      state_ = BTreeNodeState::NODE_FAILED;
      return state_;
    }

    GriddedPathTimeGraph st_graph(
      reference_line_info->st_graph_data(), dp_st_speed_config_,
      reference_line_info->path_decision()->obstacles().Items(), init_point);

    if (!st_graph.Search(&speed_data).ok()) 
    {
      std::string msg("Failed to search graph with dynamic programming");
      AERROR << msg;
      state_ = BTreeNodeState::NODE_FAILED;
      return state_;
    }

    *(reference_line_info->mutable_speed_data()) = speed_data;

    state_ = BTreeNodeState::NODE_DONE;
    return state_;
  } 
  
  BTreeNodeState SpeedGenerator::ConstantSpeed(Frame* frame, ReferenceLineInfo* reference_line_info)
  {
    SpeedData speed_data;

    double dt = 0.1;
    double v = 6.0;
    double planning_time_horizon = 3.0;
    for (double t = 0; t < planning_time_horizon; t+=dt)
    {
      speed_data.AppendSpeedPoint(t * v, t, v, 0.0, 0.0);
    }

    *(reference_line_info->mutable_speed_data()) = speed_data;

    state_ = BTreeNodeState::NODE_DONE;
    return state_;
  }

} // namespace behaviour_tree
} // namespace planning
} // namespace apollo