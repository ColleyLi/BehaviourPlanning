#include "modules/planning_btree/behaviours/tasks/path_bounds_generator/path_bounds_generator.h"

namespace apollo {
namespace planning_btree {

namespace {
// PathBoundPoint contains: (s, l_min, l_max)
using PathBoundPoint = std::tuple<double, double, double>;
// PathBound contains a vector of PathBoundPoints
using PathBound = std::vector<PathBoundPoint>;
// ObstacleEdge contains: (is_start_s, s, l_min, l_max, obstacle_id)
using ObstacleEdge = std::tuple<int, double, double, double, std::string>;

using apollo::common::VehicleConfigHelper;

// TODO: move this to config
constexpr double kPathBoundsStep = 0.5;
constexpr double kDefaultLaneWidth = 4.0;
constexpr double kPathBoundsHorizon = 100.0;
constexpr bool kIsExtendLaneBoundsToIncludeADC = true;
constexpr double kADCBufferCoef = 1.0;
constexpr double kStaticObstacleSpeedThreshold = 0.5;
constexpr double kObstacleLonStartBuffer = 3.0;
constexpr double kObstacleLonEndBuffer = 2.0;
constexpr double kObstacleLatBuffer = 0.4;
constexpr int kNumExtraTailBoundPoint = 20;
}  // namespace

BTreeNodeState PathBoundsGenerator::Init(const BTreeNodeConfig& config) {
  config_ = config;
  state_ = BTreeNodeState::NODE_INITIALIZED;
  return state_;
}

BTreeNodeState PathBoundsGenerator::Execute(BTreeFrame* frame) {
  std::vector<PathBoundary> candidate_path_boundaries;
  auto dynamic_reference_line = frame->GetMutableCurrentDynamicReferenceLine();

  if (!dynamic_reference_line) {
    state_ = BTreeNodeState::NODE_FAILED;
    return state_;
  }

  Init(*frame, *dynamic_reference_line);

  std::vector<LaneBorrowType> lane_borrow_type_list;
  lane_borrow_type_list.push_back(LaneBorrowType::NO_BORROW);

  if (dynamic_reference_line->IsLaneChangePath()) {
    PathBound lane_change_path_bound;
    GenerateLaneChangePathBound(*dynamic_reference_line,
                                &lane_change_path_bound);

    // Update the fallback path boundary into the dynamic_reference_line.
    std::vector<std::pair<double, double>> lane_change_path_bound_pair;
    for (size_t i = 0; i < lane_change_path_bound.size(); ++i) {
      lane_change_path_bound_pair.emplace_back(
          std::get<1>(lane_change_path_bound[i]),
          std::get<2>(lane_change_path_bound[i]));
    }
    candidate_path_boundaries.emplace_back(
        std::get<0>(lane_change_path_bound[0]), kPathBoundsStep,
        lane_change_path_bound_pair);
    candidate_path_boundaries.back().set_label("regular/lanechange");
    ADEBUG << "Generated lane change bound";

    // for (size_t i = 0; i < lane_change_path_bound.size(); ++i) {
    //   AERROR << "idx " << i
    //          << "; s = " << std::get<0>(lane_change_path_bound[i])
    //          << "; l_min = " << std::get<1>(lane_change_path_bound[i])
    //   << "; l_max = " << std::get<2>(lane_change_path_bound[i]);
    // }
  } else {
    for (const auto& lane_borrow_type : lane_borrow_type_list) {
      PathBound regular_path_bound;
      std::string blocking_obstacle_id = "";
      std::string lane_borrow_direction = "";
      GenerateRegularPathBound(*dynamic_reference_line, lane_borrow_type,
                               &regular_path_bound, &blocking_obstacle_id,
                               &lane_borrow_direction);

      if (regular_path_bound.empty()) {
        continue;
      }

      std::vector<std::pair<double, double>> regular_path_bound_pair;
      for (size_t i = 0; i < regular_path_bound.size(); ++i) {
        regular_path_bound_pair.emplace_back(
            std::get<1>(regular_path_bound[i]),
            std::get<2>(regular_path_bound[i]));
      }
      candidate_path_boundaries.emplace_back(std::get<0>(regular_path_bound[0]),
                                             kPathBoundsStep,
                                             regular_path_bound_pair);
      candidate_path_boundaries.back().set_label("regular");
      ADEBUG << "Generated regular bound";
    }
  }

  dynamic_reference_line->SetCandidatePathBoundaries(
      std::move(candidate_path_boundaries));

  state_ = BTreeNodeState::NODE_DONE;
  return state_;
}

bool PathBoundsGenerator::Init(
    const BTreeFrame& frame,
    const DynamicReferenceLine& dynamic_reference_line) {
  const ReferenceLine& reference_line =
      dynamic_reference_line.GetReferenceLine();
  common::TrajectoryPoint planning_start_point = frame.GetPlanningStartPoint();

  auto adc_sl_data = reference_line.ToFrenetFrame(planning_start_point);
  adc_s_ = adc_sl_data.first[0];
  adc_l_ = adc_sl_data.second[0];
  adc_ds_ = adc_sl_data.first[1];
  adc_dl_ = adc_sl_data.second[1] * adc_ds_;
  double offset_to_map = 0.0;
  reference_line.GetOffsetToMap(adc_s_, &offset_to_map);
  adc_l_to_lane_center_ = adc_l_ + offset_to_map;

  // ADC's lane width.
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  if (!reference_line.GetLaneWidth(adc_s_, &lane_left_width,
                                   &lane_right_width)) {
    AWARN << "Failed to get lane width at planning start point";
    adc_lane_width_ = kDefaultLaneWidth;
  } else {
    adc_lane_width_ = lane_left_width + lane_right_width;
  }

  return true;
}

bool PathBoundsGenerator::GenerateRegularPathBound(
    const DynamicReferenceLine& dynamic_reference_line,
    const LaneBorrowType& lane_borrow_type, PathBound* const path_bound,
    std::string* const blocking_obstacle_id,
    std::string* const lane_borrow_direction) {
  if (!InitPathBound(dynamic_reference_line, path_bound)) {
    return false;
  }

  if (!GetBoundaryFromLanesAndADC(dynamic_reference_line, lane_borrow_type, 0.1,
                                  path_bound, lane_borrow_direction)) {
    return false;
  }

  PathBound temp_path_bound = *path_bound;
  if (!GetBoundaryFromStaticObstacles(
          dynamic_reference_line.GetObstacleDecisions(), path_bound,
          blocking_obstacle_id)) {
    return false;
  }

  int counter = 0;
  while (!blocking_obstacle_id->empty() &&
         path_bound->size() < temp_path_bound.size() &&
         counter < kNumExtraTailBoundPoint) {
    path_bound->push_back(temp_path_bound[path_bound->size()]);
    counter++;
  }

  return true;
}

bool PathBoundsGenerator::GenerateLaneChangePathBound(
    const DynamicReferenceLine& dynamic_reference_line,
    PathBound* const path_bound) {
  if (!InitPathBound(dynamic_reference_line, path_bound)) {
    return false;
  }

  std::string dummy_lane_borrow_direction;
  if (!GetBoundaryFromLanesAndADC(dynamic_reference_line,
                                  LaneBorrowType::NO_BORROW, 0.1, path_bound,
                                  &dummy_lane_borrow_direction, true)) {
    return false;
  }

  // GetBoundaryFromLaneChangeForbiddenZone(dynamic_reference_line, path_bound);

  PathBound temp_path_bound = *path_bound;
  std::string blocking_obstacle_id;
  if (!GetBoundaryFromStaticObstacles(dynamic_reference_line.GetObstacleDecisions(),
                                      path_bound, &blocking_obstacle_id)) {
    return false;
  }

  // Append some extra path bound points to avoid zero-length path data.
  int counter = 0;
  while (!blocking_obstacle_id.empty() &&
         path_bound->size() < temp_path_bound.size() &&
         counter < kNumExtraTailBoundPoint) {
    path_bound->push_back(temp_path_bound[path_bound->size()]);
    counter++;
  }

  return true;
}

bool PathBoundsGenerator::InitPathBound(
    const DynamicReferenceLine& dynamic_reference_line,
    std::vector<std::tuple<double, double, double>>* const path_bound) {
  path_bound->clear();
  const auto& reference_line = dynamic_reference_line.GetReferenceLine();

  for (double curr_s = adc_s_;
       curr_s < std::fmin(adc_s_ + kPathBoundsHorizon, reference_line.Length());
       curr_s += kPathBoundsStep) {
    path_bound->emplace_back(curr_s, std::numeric_limits<double>::lowest(),
                             std::numeric_limits<double>::max());
  }

  if (path_bound->empty()) {
    return false;
  }
  return true;
}

bool PathBoundsGenerator::GetBoundaryFromLanesAndADC(
    const DynamicReferenceLine& dynamic_reference_line,
    const LaneBorrowType& lane_borrow_type, double ADC_buffer,
    PathBound* const path_bound, std::string* const borrow_lane_type,
    bool is_fallback_lanechange) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  ACHECK(!path_bound->empty());
  const ReferenceLine& reference_line =
      dynamic_reference_line.GetReferenceLine();
  bool is_left_lane_boundary = true;
  bool is_right_lane_boundary = true;
  const double boundary_buffer = 0.05;  // meter

  // Go through every point, update the boundary based on lane info and
  // ADC's position.
  double past_lane_left_width = adc_lane_width_ / 2.0;
  double past_lane_right_width = adc_lane_width_ / 2.0;
  int path_blocked_idx = -1;
  bool borrowing_reverse_lane = false;
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = std::get<0>((*path_bound)[i]);
    // 1. Get the current lane width at current point.
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    double offset_to_lane_center = 0.0;
    if (!reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                     &curr_lane_right_width)) {
      AWARN << "Failed to get lane width at s = " << curr_s;
      curr_lane_left_width = past_lane_left_width;
      curr_lane_right_width = past_lane_right_width;
    } else {
      // check if lane boundary is also road boundary
      double curr_road_left_width = 0.0;
      double curr_road_right_width = 0.0;
      if (reference_line.GetRoadWidth(curr_s, &curr_road_left_width,
                                      &curr_road_right_width)) {
        is_left_lane_boundary =
            (std::abs(curr_road_left_width - curr_lane_left_width) >
             boundary_buffer);
        is_right_lane_boundary =
            (std::abs(curr_road_right_width - curr_lane_right_width) >
             boundary_buffer);
      }
      reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      curr_lane_left_width += offset_to_lane_center;
      curr_lane_right_width -= offset_to_lane_center;
      past_lane_left_width = curr_lane_left_width;
      past_lane_right_width = curr_lane_right_width;
    }

    // 2. Get the neighbor lane widths at the current point.
    double curr_neighbor_lane_width = 0.0;
    if (CheckLaneBoundaryType(dynamic_reference_line, curr_s,
                              lane_borrow_type)) {
      hdmap::Id neighbor_lane_id;
      if (lane_borrow_type == LaneBorrowType::LEFT_BORROW) {
        // Borrowing left neighbor lane.
        if (dynamic_reference_line.GetNeighborLaneInfo(
                DynamicReferenceLine::LaneType::LeftForward, curr_s,
                &neighbor_lane_id, &curr_neighbor_lane_width)) {
          ADEBUG << "Borrow left forward neighbor lane.";
        } else if (dynamic_reference_line.GetNeighborLaneInfo(
                       DynamicReferenceLine::LaneType::LeftReverse, curr_s,
                       &neighbor_lane_id, &curr_neighbor_lane_width)) {
          borrowing_reverse_lane = true;
          ADEBUG << "Borrow left reverse neighbor lane.";
        } else {
          ADEBUG << "There is no left neighbor lane.";
        }
      } else if (lane_borrow_type == LaneBorrowType::RIGHT_BORROW) {
        // Borrowing right neighbor lane.
        if (dynamic_reference_line.GetNeighborLaneInfo(
                DynamicReferenceLine::LaneType::RightForward, curr_s,
                &neighbor_lane_id, &curr_neighbor_lane_width)) {
          ADEBUG << "Borrow right forward neighbor lane.";
        } else if (dynamic_reference_line.GetNeighborLaneInfo(
                       DynamicReferenceLine::LaneType::RightReverse, curr_s,
                       &neighbor_lane_id, &curr_neighbor_lane_width)) {
          borrowing_reverse_lane = true;
          ADEBUG << "Borrow right reverse neighbor lane.";
        } else {
          ADEBUG << "There is no right neighbor lane.";
        }
      }
    }

    // 3. Calculate the proper boundary based on lane-width, ADC's position,
    //    and ADC's velocity.
    static constexpr double kMaxLateralAccelerations = 1.5;
    double offset_to_map = 0.0;
    reference_line.GetOffsetToMap(curr_s, &offset_to_map);

    double ADC_speed_buffer = (adc_dl_ > 0 ? 1.0 : -1.0) * adc_dl_ * adc_dl_ /
                              kMaxLateralAccelerations / 2.0;

    double curr_left_bound_lane =
        curr_lane_left_width + (lane_borrow_type == LaneBorrowType::LEFT_BORROW
                                    ? curr_neighbor_lane_width
                                    : 0.0);

    double curr_right_bound_lane =
        -curr_lane_right_width -
        (lane_borrow_type == LaneBorrowType::RIGHT_BORROW
             ? curr_neighbor_lane_width
             : 0.0);

    double curr_left_bound = 0.0;
    double curr_right_bound = 0.0;

    if (kIsExtendLaneBoundsToIncludeADC || is_fallback_lanechange) {
      // extend path bounds to include ADC in fallback or change lane path
      // bounds.
      double curr_left_bound_adc =
          std::fmax(adc_l_to_lane_center_,
                    adc_l_to_lane_center_ + ADC_speed_buffer) +
          GetBufferBetweenADCCenterAndEdge() + ADC_buffer;
      curr_left_bound =
          std::fmax(curr_left_bound_lane, curr_left_bound_adc) - offset_to_map;

      double curr_right_bound_adc =
          std::fmin(adc_l_to_lane_center_,
                    adc_l_to_lane_center_ + ADC_speed_buffer) -
          GetBufferBetweenADCCenterAndEdge() - ADC_buffer;
      curr_right_bound =
          std::fmin(curr_right_bound_lane, curr_right_bound_adc) -
          offset_to_map;
    } else {
      curr_left_bound = curr_left_bound_lane - offset_to_map;
      curr_right_bound = curr_right_bound_lane - offset_to_map;
    }

    // 4. Update the boundary.
    if (!UpdatePathBoundaryWithBuffer(i, curr_left_bound, curr_right_bound,
                                      path_bound, is_left_lane_boundary,
                                      is_right_lane_boundary)) {
      path_blocked_idx = static_cast<int>(i);
    }
    if (path_blocked_idx != -1) {
      break;
    }
  }

  TrimPathBounds(path_blocked_idx, path_bound);

  if (lane_borrow_type == LaneBorrowType::NO_BORROW) {
    *borrow_lane_type = "";
  } else {
    *borrow_lane_type = borrowing_reverse_lane ? "reverse" : "forward";
  }

  return true;
}

bool PathBoundsGenerator::CheckLaneBoundaryType(
    const DynamicReferenceLine& dynamic_reference_line, const double check_s,
    const LaneBorrowType& lane_borrow_type) {
  if (lane_borrow_type == LaneBorrowType::NO_BORROW) {
    return false;
  }

  const ReferenceLine& reference_line =
      dynamic_reference_line.GetReferenceLine();
  auto ref_point = reference_line.GetNearestReferencePoint(check_s);
  if (ref_point.lane_waypoints().empty()) {
    return false;
  }

  const auto waypoint = ref_point.lane_waypoints().front();
  hdmap::LaneBoundaryType::Type lane_boundary_type =
      hdmap::LaneBoundaryType::UNKNOWN;
  if (lane_borrow_type == LaneBorrowType::LEFT_BORROW) {
    lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
  } else if (lane_borrow_type == LaneBorrowType::RIGHT_BORROW) {
    lane_boundary_type = hdmap::RightBoundaryType(waypoint);
  }
  if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW ||
      lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
    return false;
  }
  return true;
}

bool PathBoundsGenerator::UpdatePathBoundaryWithBuffer(
    size_t idx, double left_bound, double right_bound,
    PathBound* const path_boundaries, bool is_left_lane_bound,
    bool is_right_lane_bound) {
  // substract vehicle width when bound does not come from the lane boundary
  const double default_adc_buffer_coeff = 1.0;
  double left_adc_buffer_coeff =
      (is_left_lane_bound ? kADCBufferCoef : default_adc_buffer_coeff);
  double right_adc_buffer_coeff =
      (is_right_lane_bound ? kADCBufferCoef : default_adc_buffer_coeff);

  // Update the right bound (l_min):
  double new_l_min =
      std::fmax(std::get<1>((*path_boundaries)[idx]),
                right_bound + right_adc_buffer_coeff *
                                  GetBufferBetweenADCCenterAndEdge());
  // Update the left bound (l_max):
  double new_l_max = std::fmin(
      std::get<2>((*path_boundaries)[idx]),
      left_bound - left_adc_buffer_coeff * GetBufferBetweenADCCenterAndEdge());

  // Check if ADC is blocked.
  // If blocked, don't update anything, return false.
  if (new_l_min > new_l_max) {
    ADEBUG << "Path is blocked at idx = " << idx;
    return false;
  }
  // Otherwise, update path_boundaries and center_line; then return true.
  std::get<1>((*path_boundaries)[idx]) = new_l_min;
  std::get<2>((*path_boundaries)[idx]) = new_l_max;
  return true;
}

double PathBoundsGenerator::GetBufferBetweenADCCenterAndEdge() {
  double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
  // TODO(all): currently it's a fixed number. But it can take into account many
  // factors such as: ADC length, possible turning angle, speed, etc.
  static constexpr double kAdcEdgeBuffer = 0.0;

  return (adc_half_width + kAdcEdgeBuffer);
}

void PathBoundsGenerator::TrimPathBounds(const int path_blocked_idx,
                                         PathBound* const path_boundaries) {
  if (path_blocked_idx != -1) {
    if (path_blocked_idx == 0) {
      ADEBUG << "Completely blocked. Cannot move at all.";
    }
    int range = static_cast<int>(path_boundaries->size()) - path_blocked_idx;
    for (int i = 0; i < range; ++i) {
      path_boundaries->pop_back();
    }
  }
}

bool PathBoundsGenerator::GetBoundaryFromStaticObstacles(
    const ObstacleDecisions& obstacle_decisions,
    PathBound* const path_boundaries, std::string* const blocking_obstacle_id) {
  // Preprocessing.
  auto indexed_obstacles = obstacle_decisions.obstacles();
  auto sorted_obstacles = SortObstaclesForSweepLine(indexed_obstacles);
  ADEBUG << "There are " << sorted_obstacles.size() << " obstacles.";
  double center_line = adc_l_;
  size_t obs_idx = 0;
  int path_blocked_idx = -1;
  std::multiset<double, std::greater<double>> right_bounds;
  right_bounds.insert(std::numeric_limits<double>::lowest());
  std::multiset<double> left_bounds;
  left_bounds.insert(std::numeric_limits<double>::max());
  // Maps obstacle ID's to the decided ADC pass direction, if ADC should
  // pass from left, then true; otherwise, false.
  std::unordered_map<std::string, bool> obs_id_to_direction;
  // Maps obstacle ID's to the decision of whether side-pass on this obstacle
  // is allowed. If allowed, then true; otherwise, false.
  std::unordered_map<std::string, bool> obs_id_to_sidepass_decision;

  // Step through every path point.
  for (size_t i = 1; i < path_boundaries->size(); ++i) {
    double curr_s = std::get<0>((*path_boundaries)[i]);
    // Check and see if there is any obstacle change:
    if (obs_idx < sorted_obstacles.size() &&
        std::get<1>(sorted_obstacles[obs_idx]) < curr_s) {
      while (obs_idx < sorted_obstacles.size() &&
             std::get<1>(sorted_obstacles[obs_idx]) < curr_s) {
        const auto& curr_obstacle = sorted_obstacles[obs_idx];
        const double curr_obstacle_s = std::get<1>(curr_obstacle);
        const double curr_obstacle_l_min = std::get<2>(curr_obstacle);
        const double curr_obstacle_l_max = std::get<3>(curr_obstacle);
        const std::string curr_obstacle_id = std::get<4>(curr_obstacle);
        ADEBUG << "id[" << curr_obstacle_id << "] s[" << curr_obstacle_s
               << "] curr_obstacle_l_min[" << curr_obstacle_l_min
               << "] curr_obstacle_l_max[" << curr_obstacle_l_max
               << "] center_line[" << center_line << "]";
        if (std::get<0>(curr_obstacle) == 1) {
          // A new obstacle enters into our scope:
          //   - Decide which direction for the ADC to pass.
          //   - Update the left/right bound accordingly.
          //   - If boundaries blocked, then decide whether can side-pass.
          //   - If yes, then borrow neighbor lane to side-pass.
          if (curr_obstacle_l_min + curr_obstacle_l_max < center_line * 2) {
            // Obstacle is to the right of center-line, should pass from left.
            obs_id_to_direction[curr_obstacle_id] = true;
            right_bounds.insert(curr_obstacle_l_max);
          } else {
            // Obstacle is to the left of center-line, should pass from right.
            obs_id_to_direction[curr_obstacle_id] = false;
            left_bounds.insert(curr_obstacle_l_min);
          }
          if (!UpdatePathBoundaryAndCenterLineWithBuffer(
                  i, *left_bounds.begin(), *right_bounds.begin(),
                  path_boundaries, &center_line)) {
            path_blocked_idx = static_cast<int>(i);
            *blocking_obstacle_id = curr_obstacle_id;
            break;
          }
        } else {
          // An existing obstacle exits our scope.
          if (obs_id_to_direction[curr_obstacle_id]) {
            right_bounds.erase(right_bounds.find(curr_obstacle_l_max));
          } else {
            left_bounds.erase(left_bounds.find(curr_obstacle_l_min));
          }
          obs_id_to_direction.erase(curr_obstacle_id);
        }
        // Update the bounds and center_line.
        std::get<1>((*path_boundaries)[i]) = std::fmax(
            std::get<1>((*path_boundaries)[i]),
            *right_bounds.begin() + GetBufferBetweenADCCenterAndEdge());
        std::get<2>((*path_boundaries)[i]) = std::fmin(
            std::get<2>((*path_boundaries)[i]),
            *left_bounds.begin() - GetBufferBetweenADCCenterAndEdge());
        if (std::get<1>((*path_boundaries)[i]) >
            std::get<2>((*path_boundaries)[i])) {
          ADEBUG << "Path is blocked at s = " << curr_s;
          path_blocked_idx = static_cast<int>(i);
          if (!obs_id_to_direction.empty()) {
            *blocking_obstacle_id = obs_id_to_direction.begin()->first;
          }
          break;
        } else {
          center_line = (std::get<1>((*path_boundaries)[i]) +
                         std::get<2>((*path_boundaries)[i])) /
                        2.0;
        }

        ++obs_idx;
      }
    } else {
      // If no obstacle change, update the bounds and center_line.
      std::get<1>((*path_boundaries)[i]) =
          std::fmax(std::get<1>((*path_boundaries)[i]),
                    *right_bounds.begin() + GetBufferBetweenADCCenterAndEdge());
      std::get<2>((*path_boundaries)[i]) =
          std::fmin(std::get<2>((*path_boundaries)[i]),
                    *left_bounds.begin() - GetBufferBetweenADCCenterAndEdge());
      if (std::get<1>((*path_boundaries)[i]) >
          std::get<2>((*path_boundaries)[i])) {
        ADEBUG << "Path is blocked at s = " << curr_s;
        path_blocked_idx = static_cast<int>(i);
        if (!obs_id_to_direction.empty()) {
          *blocking_obstacle_id = obs_id_to_direction.begin()->first;
        }
      } else {
        center_line = (std::get<1>((*path_boundaries)[i]) +
                       std::get<2>((*path_boundaries)[i])) /
                      2.0;
      }
    }

    // Early exit if path is blocked.
    if (path_blocked_idx != -1) {
      break;
    }
  }

  TrimPathBounds(path_blocked_idx, path_boundaries);

  return true;
}

// The tuple contains (is_start_s, s, l_min, l_max, obstacle_id)
std::vector<ObstacleEdge> PathBoundsGenerator::SortObstaclesForSweepLine(
    const IndexedList<std::string, Obstacle>& indexed_obstacles) {
  std::vector<ObstacleEdge> sorted_obstacles;

  // Go through every obstacle and preprocess it.
  for (const auto* obstacle : indexed_obstacles.Items()) {
    // Only focus on those within-scope obstacles.
    if (!IsWithinPathDeciderScopeObstacle(*obstacle)) {
      continue;
    }
    // Only focus on obstacles that are ahead of ADC.
    if (obstacle->PerceptionSLBoundary().end_s() < adc_s_) {
      continue;
    }
    // Decompose each obstacle's rectangle into two edges: one at
    // start_s; the other at end_s.
    const auto obstacle_sl = obstacle->PerceptionSLBoundary();
    sorted_obstacles.emplace_back(
        1, obstacle_sl.start_s() - kObstacleLonStartBuffer,
        obstacle_sl.start_l() - kObstacleLatBuffer,
        obstacle_sl.end_l() + kObstacleLatBuffer, obstacle->Id());
    sorted_obstacles.emplace_back(
        0, obstacle_sl.end_s() + kObstacleLonEndBuffer,
        obstacle_sl.start_l() - kObstacleLatBuffer,
        obstacle_sl.end_l() + kObstacleLatBuffer, obstacle->Id());
  }

  // Sort.
  std::sort(sorted_obstacles.begin(), sorted_obstacles.end(),
            [](const ObstacleEdge& lhs, const ObstacleEdge& rhs) {
              if (std::get<1>(lhs) != std::get<1>(rhs)) {
                return std::get<1>(lhs) < std::get<1>(rhs);
              } else {
                return std::get<0>(lhs) > std::get<0>(rhs);
              }
            });

  return sorted_obstacles;
}

bool PathBoundsGenerator::UpdatePathBoundaryAndCenterLineWithBuffer(
    size_t idx, double left_bound, double right_bound,
    PathBound* const path_boundaries, double* const center_line) {
  UpdatePathBoundaryWithBuffer(idx, left_bound, right_bound, path_boundaries);
  *center_line = (std::get<1>((*path_boundaries)[idx]) +
                  std::get<2>((*path_boundaries)[idx])) /
                 2.0;
  return true;
}

bool PathBoundsGenerator::IsWithinPathDeciderScopeObstacle(
    const Obstacle& obstacle) {
  // Obstacle should be non-virtual.
  if (obstacle.IsVirtual()) {
    return false;
  }
  // Obstacle should not have ignore decision.
  if (obstacle.HasLongitudinalDecision() && obstacle.HasLateralDecision() &&
      obstacle.IsIgnore()) {
    return false;
  }
  // Obstacle should not be moving obstacle.
  if (!obstacle.IsStatic() ||
      obstacle.speed() > kStaticObstacleSpeedThreshold) {
    return false;
  }
  // TODO(jiacheng):
  // Some obstacles are not moving, but only because they are waiting for
  // red light (traffic rule) or because they are blocked by others (social).
  // These obstacles will almost certainly move in the near future and we
  // should not side-pass such obstacles.

  return true;
}

// void PathBoundsGenerator::GetBoundaryFromLaneChangeForbiddenZone(
//     const DynamicReferenceLine& dynamic_reference_line, PathBound* const
//     path_bound) {
//   // Sanity checks.
//   CHECK_NOTNULL(path_bound);
//   const ReferenceLine& reference_line =
//   dynamic_reference_line.GetReferenceLine();

//   // If there is a pre-determined lane-change starting position, then use it;
//   // otherwise, decide one.
//   auto* lane_change_status = injector_->planning_context()
//                                  ->mutable_planning_status()
//                                  ->mutable_change_lane();
//   if (lane_change_status->is_clear_to_change_lane()) {
//     ADEBUG << "Current position is clear to change lane. No need prep s.";
//     lane_change_status->set_exist_lane_change_start_position(false);
//     return;
//   }
//   double lane_change_start_s = 0.0;
//   if (lane_change_status->exist_lane_change_start_position()) {
//     common::SLPoint point_sl;
//     reference_line.XYToSL(lane_change_status->lane_change_start_position(),
//                           &point_sl);
//     lane_change_start_s = point_sl.s();
//   } else {
//     // TODO(jiacheng): train ML model to learn this.
//     lane_change_start_s = FLAGS_lane_change_prepare_length + adc_frenet_s_;

//     // Update the decided lane_change_start_s into planning-context.
//     common::SLPoint lane_change_start_sl;
//     lane_change_start_sl.set_s(lane_change_start_s);
//     lane_change_start_sl.set_l(0.0);
//     common::math::Vec2d lane_change_start_xy;
//     reference_line.SLToXY(lane_change_start_sl, &lane_change_start_xy);
//     lane_change_status->set_exist_lane_change_start_position(true);
//     lane_change_status->mutable_lane_change_start_position()->set_x(
//         lane_change_start_xy.x());
//     lane_change_status->mutable_lane_change_start_position()->set_y(
//         lane_change_start_xy.y());
//   }

//   // Remove the target lane out of the path-boundary, up to the decided S.
//   if (lane_change_start_s < adc_frenet_s_) {
//     // If already passed the decided S, then return.
//     // lane_change_status->set_exist_lane_change_start_position(false);
//     return;
//   }
//   for (size_t i = 0; i < path_bound->size(); ++i) {
//     double curr_s = std::get<0>((*path_bound)[i]);
//     if (curr_s > lane_change_start_s) {
//       break;
//     }
//     double curr_lane_left_width = 0.0;
//     double curr_lane_right_width = 0.0;
//     double offset_to_map = 0.0;
//     reference_line.GetOffsetToMap(curr_s, &offset_to_map);
//     if (reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
//                                     &curr_lane_right_width)) {
//       double offset_to_lane_center = 0.0;
//       reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
//       curr_lane_left_width += offset_to_lane_center;
//       curr_lane_right_width -= offset_to_lane_center;
//     }
//     curr_lane_left_width -= offset_to_map;
//     curr_lane_right_width += offset_to_map;

//     std::get<1>((*path_bound)[i]) =
//         adc_frenet_l_ > curr_lane_left_width
//             ? curr_lane_left_width + GetBufferBetweenADCCenterAndEdge()
//             : std::get<1>((*path_bound)[i]);
//     std::get<1>((*path_bound)[i]) =
//         std::fmin(std::get<1>((*path_bound)[i]), adc_frenet_l_ - 0.1);
//     std::get<2>((*path_bound)[i]) =
//         adc_frenet_l_ < -curr_lane_right_width
//             ? -curr_lane_right_width - GetBufferBetweenADCCenterAndEdge()
//             : std::get<2>((*path_bound)[i]);
//     std::get<2>((*path_bound)[i]) =
//         std::fmax(std::get<2>((*path_bound)[i]), adc_frenet_l_ + 0.1);
//   }
// }

// // Currently, it processes each obstacle based on its frenet-frame
// // projection. Therefore, it might be overly conservative when processing
// // obstacles whose headings differ from road-headings a lot.
// // TODO(all): (future work) this can be improved in the future.
// bool PathBoundsDecider::GetBoundaryFromStaticObstacles(
//     const PathDecision& path_decision, PathBound* const path_boundaries,
//     std::string* const blocking_obstacle_id) {
//   // Preprocessing.
//   auto indexed_obstacles = path_decision.obstacles();
//   auto sorted_obstacles = SortObstaclesForSweepLine(indexed_obstacles);
//   ADEBUG << "There are " << sorted_obstacles.size() << " obstacles.";
//   double center_line = adc_frenet_l_;
//   size_t obs_idx = 0;
//   int path_blocked_idx = -1;
//   std::multiset<double, std::greater<double>> right_bounds;
//   right_bounds.insert(std::numeric_limits<double>::lowest());
//   std::multiset<double> left_bounds;
//   left_bounds.insert(std::numeric_limits<double>::max());
//   // Maps obstacle ID's to the decided ADC pass direction, if ADC should
//   // pass from left, then true; otherwise, false.
//   std::unordered_map<std::string, bool> obs_id_to_direction;
//   // Maps obstacle ID's to the decision of whether side-pass on this obstacle
//   // is allowed. If allowed, then true; otherwise, false.
//   std::unordered_map<std::string, bool> obs_id_to_sidepass_decision;

//   // Step through every path point.
//   for (size_t i = 1; i < path_boundaries->size(); ++i) {
//     double curr_s = std::get<0>((*path_boundaries)[i]);
//     // Check and see if there is any obstacle change:
//     if (obs_idx < sorted_obstacles.size() &&
//         std::get<1>(sorted_obstacles[obs_idx]) < curr_s) {
//       while (obs_idx < sorted_obstacles.size() &&
//              std::get<1>(sorted_obstacles[obs_idx]) < curr_s) {
//         const auto& curr_obstacle = sorted_obstacles[obs_idx];
//         const double curr_obstacle_s = std::get<1>(curr_obstacle);
//         const double curr_obstacle_l_min = std::get<2>(curr_obstacle);
//         const double curr_obstacle_l_max = std::get<3>(curr_obstacle);
//         const std::string curr_obstacle_id = std::get<4>(curr_obstacle);
//         ADEBUG << "id[" << curr_obstacle_id << "] s[" << curr_obstacle_s
//                << "] curr_obstacle_l_min[" << curr_obstacle_l_min
//                << "] curr_obstacle_l_max[" << curr_obstacle_l_max
//                << "] center_line[" << center_line << "]";
//         if (std::get<0>(curr_obstacle) == 1) {
//           // A new obstacle enters into our scope:
//           //   - Decide which direction for the ADC to pass.
//           //   - Update the left/right bound accordingly.
//           //   - If boundaries blocked, then decide whether can side-pass.
//           //   - If yes, then borrow neighbor lane to side-pass.
//           if (curr_obstacle_l_min + curr_obstacle_l_max < center_line * 2) {
//             // Obstacle is to the right of center-line, should pass from
//             left. obs_id_to_direction[curr_obstacle_id] = true;
//             right_bounds.insert(curr_obstacle_l_max);
//           } else {
//             // Obstacle is to the left of center-line, should pass from
//             right. obs_id_to_direction[curr_obstacle_id] = false;
//             left_bounds.insert(curr_obstacle_l_min);
//           }
//           if (!UpdatePathBoundaryAndCenterLineWithBuffer(
//                   i, *left_bounds.begin(), *right_bounds.begin(),
//                   path_boundaries, &center_line)) {
//             path_blocked_idx = static_cast<int>(i);
//             *blocking_obstacle_id = curr_obstacle_id;
//             break;
//           }
//         } else {
//           // An existing obstacle exits our scope.
//           if (obs_id_to_direction[curr_obstacle_id]) {
//             right_bounds.erase(right_bounds.find(curr_obstacle_l_max));
//           } else {
//             left_bounds.erase(left_bounds.find(curr_obstacle_l_min));
//           }
//           obs_id_to_direction.erase(curr_obstacle_id);
//         }
//         // Update the bounds and center_line.
//         std::get<1>((*path_boundaries)[i]) = std::fmax(
//             std::get<1>((*path_boundaries)[i]),
//             *right_bounds.begin() + GetBufferBetweenADCCenterAndEdge());
//         std::get<2>((*path_boundaries)[i]) = std::fmin(
//             std::get<2>((*path_boundaries)[i]),
//             *left_bounds.begin() - GetBufferBetweenADCCenterAndEdge());
//         if (std::get<1>((*path_boundaries)[i]) >
//             std::get<2>((*path_boundaries)[i])) {
//           ADEBUG << "Path is blocked at s = " << curr_s;
//           path_blocked_idx = static_cast<int>(i);
//           if (!obs_id_to_direction.empty()) {
//             *blocking_obstacle_id = obs_id_to_direction.begin()->first;
//           }
//           break;
//         } else {
//           center_line = (std::get<1>((*path_boundaries)[i]) +
//                          std::get<2>((*path_boundaries)[i])) /
//                         2.0;
//         }

//         ++obs_idx;
//       }
//     } else {
//       // If no obstacle change, update the bounds and center_line.
//       std::get<1>((*path_boundaries)[i]) =
//           std::fmax(std::get<1>((*path_boundaries)[i]),
//                     *right_bounds.begin() +
//                     GetBufferBetweenADCCenterAndEdge());
//       std::get<2>((*path_boundaries)[i]) =
//           std::fmin(std::get<2>((*path_boundaries)[i]),
//                     *left_bounds.begin() -
//                     GetBufferBetweenADCCenterAndEdge());
//       if (std::get<1>((*path_boundaries)[i]) >
//           std::get<2>((*path_boundaries)[i])) {
//         ADEBUG << "Path is blocked at s = " << curr_s;
//         path_blocked_idx = static_cast<int>(i);
//         if (!obs_id_to_direction.empty()) {
//           *blocking_obstacle_id = obs_id_to_direction.begin()->first;
//         }
//       } else {
//         center_line = (std::get<1>((*path_boundaries)[i]) +
//                        std::get<2>((*path_boundaries)[i])) /
//                       2.0;
//       }
//     }

//     // Early exit if path is blocked.
//     if (path_blocked_idx != -1) {
//       break;
//     }
//   }

//   TrimPathBounds(path_blocked_idx, path_boundaries);

//   return true;
// }

}  // namespace planning_btree
}  // namespace apollo