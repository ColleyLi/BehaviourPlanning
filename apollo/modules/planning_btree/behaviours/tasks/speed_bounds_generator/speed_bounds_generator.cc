#include "modules/planning_btree/behaviours/tasks/speed_bounds_generator/speed_bounds_generator.h"

namespace apollo {
namespace planning_btree {

namespace {
// STBoundPoint contains (t, s_min, s_max)
using STBoundPoint = std::tuple<double, double, double>;
// STBound is a vector of STBoundPoints
using STBound = std::vector<STBoundPoint>;
// ObsDecSet is a set of decision for new obstacles.
using ObsDecSet = std::vector<std::pair<std::string, ObjectDecisionType>>;

constexpr double kTotalTime = 7.0;
constexpr double kSTBoundsDeciderResolution = 0.1;
constexpr double kSTPassableThreshold = 3.0;
constexpr bool kUseSTDrivableBoundary = true;
constexpr double kCruiseSped = 11.0;
}  // namespace

BTreeNodeState SpeedBoundsGenerator::Init(const BTreeNodeConfig& config) {
  config_ = config;
  state_ = BTreeNodeState::NODE_INITIALIZED;
  return state_;
}

BTreeNodeState SpeedBoundsGenerator::Execute(BTreeFrame* frame) {
  auto dynamic_reference_line = frame->GetMutableCurrentDynamicReferenceLine();
  if (!STBoundsDecider(frame, dynamic_reference_line)) {
    AERROR << "STBoundsDecider failed!";
    state_ = BTreeNodeState::NODE_FAILED;
    return state_;
  }

  if (!SpeedBoundsDecider(frame, dynamic_reference_line)) {
    AERROR << "SpeedBoundDecider failed!";
    state_ = BTreeNodeState::NODE_FAILED;
    return state_;
  }

  state_ = BTreeNodeState::NODE_DONE;
  return state_;
}

bool SpeedBoundsGenerator::STBoundsDecider(
    BTreeFrame* const frame,
    DynamicReferenceLine* const dynamic_reference_line) {
  // Initialize the related helper classes.
  InitSTBoundsDecider(*frame, dynamic_reference_line);

  // Sweep the t-axis, and determine the s-boundaries step by step.
  STBound regular_st_bound;
  STBound regular_vt_bound;
  std::vector<std::pair<double, double>> st_guide_line;
  bool ret = GenerateRegularSTBound(&regular_st_bound, &regular_vt_bound,
                                    &st_guide_line);

  if (!ret || regular_st_bound.empty()) {
    return false;
  }

  StGraphData* st_graph_data = dynamic_reference_line->GetMutableSTGraphData();
  st_graph_data->SetSTDrivableBoundary(regular_st_bound, regular_vt_bound);

  // auto all_st_boundaries = st_obstacles_processor_.GetAllSTBoundaries();
  // std::vector<STBoundary> st_boundaries;
  // for (const auto& st_boundary : all_st_boundaries) {
  //   st_boundaries.push_back(st_boundary.second);
  // }
  // ADEBUG << "Total ST boundaries = " << st_boundaries.size();

  // for (const auto& st_bound_pt : regular_st_bound) {
  //   double t = 0.0;
  //   double s_lower = 0.0;
  //   std::tie(t, s_lower, std::ignore) = st_bound_pt;
  //   ADEBUG << "(" << t << ", " << s_lower << ")";
  // }

  // for (int i = static_cast<int>(regular_st_bound.size()) - 1; i >= 0; --i) {
  //   double t = 0.0;
  //   double s_upper = 0.0;
  //   std::tie(t, std::ignore, s_upper) = regular_st_bound[i];
  //   ADEBUG << "(" << t << ", " << s_upper << ")";
  // }
  
  return true;
}

bool SpeedBoundsGenerator::SpeedBoundsDecider(
    BTreeFrame* const frame,
    DynamicReferenceLine* const dynamic_reference_line) {
  const PathData& path_data = dynamic_reference_line->GetPathData();
  const ReferenceLine& reference_line =
      dynamic_reference_line->GetReferenceLine();
  const common::TrajectoryPoint& init_point = frame->GetPlanningStartPoint();
  ObstacleDecisions* const obstacle_decisions =
      dynamic_reference_line->GetMutableObstacleDecisions();

  // 1. Map obstacles into st graph
  auto time1 = std::chrono::system_clock::now();
  STBoundaryMapper boundary_mapper(reference_line, path_data,
                                   path_data.discretized_path().Length(),
                                   kTotalTime);

  if (!kUseSTDrivableBoundary) {
    obstacle_decisions->EraseStBoundaries();
  }

  if (!boundary_mapper.ComputeSTBoundary(obstacle_decisions)) {
    const std::string msg = "Mapping obstacle failed.";
    AERROR << msg;
    return false;
  }

  auto time2 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = time2 - time1;
  ADEBUG << "Time for ST Boundary Mapping = " << diff.count() * 1000
         << " msec.";

  std::vector<const STBoundary*> boundaries;
  for (auto* obstacle : obstacle_decisions->obstacles().Items()) {
    const auto& id = obstacle->Id();
    const auto& st_boundary = obstacle->path_st_boundary();
    if (!st_boundary.IsEmpty()) {
      if (st_boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
        obstacle_decisions->Find(id)->SetBlockingObstacle(false);
      } else {
        obstacle_decisions->Find(id)->SetBlockingObstacle(true);
      }
      boundaries.push_back(&st_boundary);
    }
  }

  const double min_s_on_st_boundaries =
      SetSpeedFallbackDistance(obstacle_decisions);

  // 2. Create speed limit along path
  SpeedLimitDecider speed_limit_decider(reference_line, path_data);

  SpeedLimit speed_limit;
  if (!speed_limit_decider.GetSpeedLimits(obstacle_decisions->obstacles(),
                                          &speed_limit)) {
    const std::string msg = "Getting speed limits failed!";
    AERROR << msg;
    return false;
  }

  // 3. Get path_length as s axis search bound in st graph
  const double path_data_length = path_data.discretized_path().Length();

  // 4. Get time duration as t axis search bound in st graph
  const double total_time_by_conf = kTotalTime;

  // Load generated st graph data back to frame
  StGraphData* st_graph_data = dynamic_reference_line->GetMutableSTGraphData();

  st_graph_data->LoadData(boundaries, min_s_on_st_boundaries, init_point,
                          speed_limit, kCruiseSped, path_data_length,
                          total_time_by_conf);

  return true;
}

double SpeedBoundsGenerator::SetSpeedFallbackDistance(
    ObstacleDecisions* const obstacle_decisions) {
  // Set min_s_on_st_boundaries to guide speed fallback.
  static constexpr double kEpsilon = 1.0e-6;
  double min_s_non_reverse = std::numeric_limits<double>::infinity();
  double min_s_reverse = std::numeric_limits<double>::infinity();

  for (auto* obstacle : obstacle_decisions->obstacles().Items()) {
    const auto& st_boundary = obstacle->path_st_boundary();

    if (st_boundary.IsEmpty()) {
      continue;
    }

    const auto left_bottom_point_s = st_boundary.bottom_left_point().s();
    const auto right_bottom_point_s = st_boundary.bottom_right_point().s();
    const auto lowest_s = std::min(left_bottom_point_s, right_bottom_point_s);

    if (left_bottom_point_s - right_bottom_point_s > kEpsilon) {
      if (min_s_reverse > lowest_s) {
        min_s_reverse = lowest_s;
      }
    } else if (min_s_non_reverse > lowest_s) {
      min_s_non_reverse = lowest_s;
    }
  }

  min_s_reverse = std::max(min_s_reverse, 0.0);
  min_s_non_reverse = std::max(min_s_non_reverse, 0.0);

  return min_s_non_reverse > min_s_reverse ? 0.0 : min_s_non_reverse;
}

void SpeedBoundsGenerator::InitSTBoundsDecider(
    const BTreeFrame& frame,
    DynamicReferenceLine* const dynamic_reference_line) {
  const PathData& path_data = dynamic_reference_line->GetPathData();
  ObstacleDecisions* obstacle_decisions =
      dynamic_reference_line->GetMutableObstacleDecisions();

  // Map all related obstacles onto ST-Graph.
  auto time1 = std::chrono::system_clock::now();
  st_obstacles_processor_.Init(path_data.discretized_path().Length(),
                               kTotalTime, path_data, obstacle_decisions);
  st_obstacles_processor_.MapObstaclesToSTBoundaries(obstacle_decisions);
  auto time2 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = time2 - time1;
  ADEBUG << "Time for ST Obstacles Processing = " << diff.count() * 1000
         << " msec.";

  // Initialize Guide-Line and Driving-Limits.
  static constexpr double desired_speed = 15.0;
  st_guide_line_.Init(desired_speed);

  static constexpr double max_acc = 2.5;
  static constexpr double max_dec = 5.0;
  static constexpr double max_v = desired_speed * 1.5;
  st_driving_limits_.Init(max_acc, max_dec, max_v,
                          frame.GetPlanningStartPoint().v());
}

bool SpeedBoundsGenerator::GenerateRegularSTBound(
    STBound* const st_bound, STBound* const vt_bound,
    std::vector<std::pair<double, double>>* const st_guide_line) {
  // Initialize st-boundary.
  for (double curr_t = 0.0; curr_t <= kTotalTime;
       curr_t += kSTBoundsDeciderResolution) {
    st_bound->emplace_back(curr_t, std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::max());
    vt_bound->emplace_back(curr_t, std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::max());
  }

  // Sweep-line to get detailed ST-boundary.
  for (size_t i = 0; i < st_bound->size(); ++i) {
    double t, s_lower, s_upper, lower_obs_v, upper_obs_v;
    std::tie(t, s_lower, s_upper) = st_bound->at(i);
    std::tie(t, lower_obs_v, upper_obs_v) = vt_bound->at(i);
    ADEBUG << "Processing st-boundary at t = " << t;

    // Get Boundary due to driving limits
    auto driving_limits_bound = st_driving_limits_.GetVehicleDynamicsLimits(t);
    s_lower = std::fmax(s_lower, driving_limits_bound.first);
    s_upper = std::fmin(s_upper, driving_limits_bound.second);
    ADEBUG << "Bounds for s due to driving limits are "
           << "s_upper = " << s_upper << ", s_lower = " << s_lower;

    // Get Boundary due to obstacles
    std::vector<std::pair<double, double>> available_s_bounds;
    std::vector<ObsDecSet> available_obs_decisions;
    if (!st_obstacles_processor_.GetSBoundsFromDecisions(
            t, &available_s_bounds, &available_obs_decisions)) {
      const std::string msg =
          "Failed to find a proper boundary due to obstacles.";
      AERROR << msg;
      return false;
    }
    std::vector<std::pair<STBoundPoint, ObsDecSet>> available_choices;
    ADEBUG << "Available choices are:";
    for (int j = 0; j < static_cast<int>(available_s_bounds.size()); ++j) {
      ADEBUG << "  (" << available_s_bounds[j].first << ", "
             << available_s_bounds[j].second << ")";
      available_choices.emplace_back(
          std::make_tuple(0.0, available_s_bounds[j].first,
                          available_s_bounds[j].second),
          available_obs_decisions[j]);
    }
    RemoveInvalidDecisions(driving_limits_bound, &available_choices);

    if (!available_choices.empty()) {
      ADEBUG << "One decision needs to be made among "
             << available_choices.size() << " choices.";
      double guide_line_s = st_guide_line_.GetGuideSFromT(t);
      st_guide_line->emplace_back(t, guide_line_s);
      RankDecisions(guide_line_s, driving_limits_bound, &available_choices);
      // Select the top decision.
      auto top_choice_s_range = available_choices.front().first;
      bool is_limited_by_upper_obs = false;
      bool is_limited_by_lower_obs = false;
      if (s_lower < std::get<1>(top_choice_s_range)) {
        s_lower = std::get<1>(top_choice_s_range);
        is_limited_by_lower_obs = true;
      }
      if (s_upper > std::get<2>(top_choice_s_range)) {
        s_upper = std::get<2>(top_choice_s_range);
        is_limited_by_upper_obs = true;
      }

      // Set decision for obstacles without decisions.
      auto top_choice_decision = available_choices.front().second;
      st_obstacles_processor_.SetObstacleDecision(top_choice_decision);

      // Update st-guide-line, st-driving-limit info, and v-limits.
      std::pair<double, double> limiting_speed_info;
      if (st_obstacles_processor_.GetLimitingSpeedInfo(t,
                                                       &limiting_speed_info)) {
        st_driving_limits_.UpdateBlockingInfo(
            t, s_lower, limiting_speed_info.first, s_upper,
            limiting_speed_info.second);
        st_guide_line_.UpdateBlockingInfo(t, s_lower, true);
        st_guide_line_.UpdateBlockingInfo(t, s_upper, false);
        if (is_limited_by_lower_obs) {
          lower_obs_v = limiting_speed_info.first;
        }
        if (is_limited_by_upper_obs) {
          upper_obs_v = limiting_speed_info.second;
        }
      }
    } else {
      const std::string msg = "No valid st-boundary exists.";
      AERROR << msg;
      return false;
    }

    // Update into st_bound
    st_bound->at(i) = std::make_tuple(t, s_lower, s_upper);
    vt_bound->at(i) = std::make_tuple(t, lower_obs_v, upper_obs_v);
  }

  return true;
}

void SpeedBoundsGenerator::RemoveInvalidDecisions(
    std::pair<double, double> driving_limit,
    std::vector<std::pair<STBoundPoint, ObsDecSet>>* available_choices) {
  // Remove those choices that don't even fall within driving-limits.
  size_t i = 0;
  while (i < available_choices->size()) {
    double s_lower = 0.0;
    double s_upper = 0.0;
    std::tie(std::ignore, s_lower, s_upper) = available_choices->at(i).first;
    if (s_lower > driving_limit.second || s_upper < driving_limit.first) {
      // Invalid bound, should be removed.
      if (i != available_choices->size() - 1) {
        swap(available_choices->at(i),
             available_choices->at(available_choices->size() - 1));
      }
      available_choices->pop_back();
    } else {
      // Valid bound, proceed to the next one.
      ++i;
    }
  }
}

void SpeedBoundsGenerator::RankDecisions(
    double s_guide_line, std::pair<double, double> driving_limit,
    std::vector<std::pair<STBoundPoint, ObsDecSet>>* available_choices) {
  // Perform sorting of the existing decisions.
  bool has_swaps = true;
  while (has_swaps) {
    has_swaps = false;
    for (int i = 0; i < static_cast<int>(available_choices->size()) - 1; ++i) {
      double A_s_lower = 0.0;
      double A_s_upper = 0.0;
      std::tie(std::ignore, A_s_lower, A_s_upper) =
          available_choices->at(i).first;
      double B_s_lower = 0.0;
      double B_s_upper = 0.0;
      std::tie(std::ignore, B_s_lower, B_s_upper) =
          available_choices->at(i + 1).first;

      ADEBUG << "    Range ranking: A has s_upper = " << A_s_upper
             << ", s_lower = " << A_s_lower;
      ADEBUG << "    Range ranking: B has s_upper = " << B_s_upper
             << ", s_lower = " << B_s_lower;

      // If not both are larger than passable-threshold, should select
      // the one with larger room.
      double A_room = std::fmin(driving_limit.second, A_s_upper) -
                      std::fmax(driving_limit.first, A_s_lower);
      double B_room = std::fmin(driving_limit.second, B_s_upper) -
                      std::fmax(driving_limit.first, B_s_lower);
      if (A_room < kSTPassableThreshold || B_room < kSTPassableThreshold) {
        if (A_room < B_room) {
          swap(available_choices->at(i + 1), available_choices->at(i));
          has_swaps = true;
          ADEBUG << "Swapping to favor larger room.";
        }
        continue;
      }

      // Should select the one with overlap to guide-line
      bool A_contains_guideline =
          A_s_upper >= s_guide_line && A_s_lower <= s_guide_line;
      bool B_contains_guideline =
          B_s_upper >= s_guide_line && B_s_lower <= s_guide_line;
      if (A_contains_guideline != B_contains_guideline) {
        if (!A_contains_guideline) {
          swap(available_choices->at(i + 1), available_choices->at(i));
          has_swaps = true;
          ADEBUG << "Swapping to favor overlapping with guide-line.";
        }
        continue;
      }
    }
  }
}

}  // namespace planning_btree
}  // namespace apollo