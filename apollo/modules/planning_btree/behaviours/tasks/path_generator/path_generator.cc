#include "modules/planning_btree/behaviours/tasks/path_generator/path_generator.h"

namespace apollo {
namespace planning_btree {
using apollo::common::VehicleConfigHelper;
using apollo::common::math::Gaussian;

// default_path_config {
//   l_weight: 1.0
//   dl_weight: 20.0
//   ddl_weight: 1000.0
//   dddl_weight: 50000.0
// }
// lane_change_path_config {
//   l_weight: 1.0
//   dl_weight: 5.0
//   ddl_weight: 800.0
//   dddl_weight: 30000.0
// }

// TODO: move to config
namespace {
constexpr int kMaxIter = 4000;
constexpr double kPathReferenceLWeight = 0.0;
constexpr double kConfigLWeight = 20.0;
constexpr double kConfigDLWeight = 20.0;
constexpr double kConfigDDLWeight = 1000.0;
constexpr double kConfigDDDLWeight = 30000.0;
constexpr double kConfigLWeightCL = 20.0;
constexpr double kConfigDLWeightCL = 5.0;
constexpr double kConfigDDLWeightCL = 500.0;
constexpr double kConfigDDDLWeightCL = 5000.0;
constexpr double kTrajectorySpaceResolution = 1.0;
constexpr double kLateralDerivativeBoundDefault = 2.0;
}  // namespace

BTreeNodeState PathGenerator::Init(const BTreeNodeConfig& config) {
  config_ = config;
  state_ = BTreeNodeState::NODE_INITIALIZED;
  return state_;
}

BTreeNodeState PathGenerator::Execute(BTreeFrame* frame) {
  auto dynamic_reference_line = frame->GetMutableCurrentDynamicReferenceLine();
  const ReferenceLine& reference_line =
      dynamic_reference_line->GetReferenceLine();
  common::TrajectoryPoint planning_start_point = frame->GetPlanningStartPoint();

  const auto init_frenet_state =
      reference_line.ToFrenetFrame(planning_start_point);

double lw, dlw, ddlw, dddlw;
if (dynamic_reference_line->IsLaneChangePath())
{
  lw = kConfigLWeightCL;
  dlw = kConfigDLWeightCL;
  ddlw = kConfigDDLWeightCL;
  dddlw = kConfigDDDLWeightCL;
}
else
{
  lw = kConfigLWeight;
  dlw = kConfigDLWeight;
  ddlw = kConfigDDLWeight;
  dddlw = kConfigDDDLWeight;  
}

std::array<double, 5> w = {
      lw,
      dlw * std::fmax(init_frenet_state.first[1] * init_frenet_state.first[1], 5.0),
      ddlw,
      dddlw,
       0.0};

  const auto& path_boundaries =
      dynamic_reference_line->GetCandidatePathBoundaries();
  const auto& reference_path_data = dynamic_reference_line->GetPathData();

  std::vector<PathData> candidate_path_data;
  for (const auto& path_boundary : path_boundaries) {
    size_t path_boundary_size = path_boundary.boundary().size();

    std::vector<double> opt_l;
    std::vector<double> opt_dl;
    std::vector<double> opt_ddl;
    std::array<double, 3> end_state = {0.0, 0.0, 0.0};

    // TODO: check the necessity of this assignment
    // Maybe it is enough to initialize new path data
    PathData* const final_path_data =
        dynamic_reference_line->GetMutablePathData();
    PathData path_data = *final_path_data;

    std::vector<double> path_reference_l(path_boundary_size, 0.0);
    bool is_valid_path_reference = false;
    size_t path_reference_size = reference_path_data.path_reference().size();

    if (path_boundary.label().find("regular") != std::string::npos &&
        reference_path_data.is_valid_path_reference()) {
      ADEBUG << "path label is: " << path_boundary.label();
      // when path reference is ready
      for (size_t i = 0; i < path_reference_size; ++i) {
        common::SLPoint path_reference_sl;
        reference_line.XYToSL(
            common::util::PointFactory::ToPointENU(
                reference_path_data.path_reference().at(i).x(),
                reference_path_data.path_reference().at(i).y()),
            &path_reference_sl);
        path_reference_l[i] = path_reference_sl.l();
      }
      end_state[0] = path_reference_l.back();
      path_data.set_is_optimized_towards_trajectory_reference(true);
      is_valid_path_reference = true;
    }

    const auto& veh_param =
        common::VehicleConfigHelper::GetConfig().vehicle_param();
    const double lat_acc_bound =
        std::tan(veh_param.max_steer_angle() / veh_param.steer_ratio()) /
        veh_param.wheel_base();
    std::vector<std::pair<double, double>> ddl_bounds;
    for (size_t i = 0; i < path_boundary_size; ++i) {
      double s = static_cast<double>(i) * path_boundary.delta_s() +
                 path_boundary.start_s();
      double kappa = reference_line.GetNearestReferencePoint(s).kappa();
      ddl_bounds.emplace_back(-lat_acc_bound - kappa, lat_acc_bound - kappa);
    }

    bool optimization_result = OptimizePath(
        init_frenet_state.second, end_state, std::move(path_reference_l),
        path_reference_size, path_boundary.delta_s(), is_valid_path_reference,
        path_boundary.boundary(), ddl_bounds, w, kMaxIter, &opt_l, &opt_dl,
        &opt_ddl);

    if (optimization_result) {
      auto frenet_frame_path =
          ToPiecewiseJerkPath(opt_l, opt_dl, opt_ddl, path_boundary.delta_s(),
                              path_boundary.start_s());

      path_data.SetReferenceLine(&reference_line);
      path_data.SetFrenetPath(std::move(frenet_frame_path));
      path_data.set_path_label(path_boundary.label());
      path_data.set_blocking_obstacle_id(path_boundary.blocking_obstacle_id());
      candidate_path_data.push_back(std::move(path_data));
    }
  }

  if (candidate_path_data.empty()) {
    state_ = BTreeNodeState::NODE_FAILED;
    return state_;
  }
  // TODO: make path data selection
  *(dynamic_reference_line->GetMutablePathData()) = candidate_path_data.front();

  dynamic_reference_line->SetCandidatePathData(std::move(candidate_path_data));

  state_ = BTreeNodeState::NODE_DONE;
  return state_;
}

bool PathGenerator::OptimizePath(
    const std::array<double, 3>& init_state,
    const std::array<double, 3>& end_state,
    std::vector<double> path_reference_l_ref, const size_t path_reference_size,
    const double delta_s, const bool is_valid_path_reference,
    const std::vector<std::pair<double, double>>& lat_boundaries,
    const std::vector<std::pair<double, double>>& ddl_bounds,
    const std::array<double, 5>& w, const int max_iter, std::vector<double>* x,
    std::vector<double>* dx, std::vector<double>* ddx) {
  // num of knots
  const size_t kNumKnots = lat_boundaries.size();
  PiecewiseJerkPathProblem piecewise_jerk_problem(kNumKnots, delta_s,
                                                  init_state);

  // TODO(Hongyi): update end_state settings
  piecewise_jerk_problem.set_end_state_ref({1000.0, 0.0, 0.0}, end_state);
  // pull over scenarios
  // Because path reference might also make the end_state != 0
  // we have to exclude this condition here
  if (end_state[0] != 0 && !is_valid_path_reference) {
    std::vector<double> x_ref(kNumKnots, end_state[0]);
    // const auto& pull_over_type = injector_->planning_context()
    //                                  ->planning_status()
    //                                  .pull_over()
    //                                  .pull_over_type();
    // const double weight_x_ref =
    //     pull_over_type == PullOverStatus::EMERGENCY_PULL_OVER ? 200.0 : 10.0;
    const double weight_x_ref = 10.0;
    piecewise_jerk_problem.set_x_ref(weight_x_ref, std::move(x_ref));
  }
  // use path reference as a optimization cost function
  if (is_valid_path_reference) {
    // for non-path-reference part
    // weight_x_ref is set to default value, where
    // l weight = weight_x_ + weight_x_ref_ = (1.0 + 0.0)
    std::vector<double> weight_x_ref_vec(kNumKnots, 0.0);
    // increase l weight for path reference part only

    const double peak_value = kPathReferenceLWeight;
    const double peak_value_x =
        0.5 * static_cast<double>(path_reference_size) * delta_s;
    for (size_t i = 0; i < path_reference_size; ++i) {
      // Gaussian weighting
      const double x = static_cast<double>(i) * delta_s;
      weight_x_ref_vec.at(i) = GaussianWeighting(x, peak_value, peak_value_x);
    }
    piecewise_jerk_problem.set_x_ref(std::move(weight_x_ref_vec),
                                     std::move(path_reference_l_ref));
  }

  piecewise_jerk_problem.set_weight_x(w[0]);
  piecewise_jerk_problem.set_weight_dx(w[1]);
  piecewise_jerk_problem.set_weight_ddx(w[2]);
  piecewise_jerk_problem.set_weight_dddx(w[3]);

  piecewise_jerk_problem.set_scale_factor({1.0, 10.0, 100.0});

  auto start_time = std::chrono::system_clock::now();

  piecewise_jerk_problem.set_x_bounds(lat_boundaries);
  piecewise_jerk_problem.set_dx_bounds(-kLateralDerivativeBoundDefault,
                                       kLateralDerivativeBoundDefault);
  piecewise_jerk_problem.set_ddx_bounds(ddl_bounds);

  // Estimate lat_acc and jerk boundary from vehicle_params
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const double axis_distance = veh_param.wheel_base();
  const double max_yaw_rate =
      veh_param.max_steer_angle_rate() / veh_param.steer_ratio() / 2.0;
  const double jerk_bound = EstimateJerkBoundary(std::fmax(init_state[1], 1.0),
                                                 axis_distance, max_yaw_rate);
  piecewise_jerk_problem.set_dddx_bound(jerk_bound);

  bool success = piecewise_jerk_problem.Optimize(max_iter);

  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  ADEBUG << "Path Optimizer used time: " << diff.count() * 1000 << " ms.";

  if (!success) {
    AERROR << "piecewise jerk path optimizer failed";
    return false;
  }

  *x = piecewise_jerk_problem.opt_x();
  *dx = piecewise_jerk_problem.opt_dx();
  *ddx = piecewise_jerk_problem.opt_ddx();

  return true;
}

FrenetFramePath PathGenerator::ToPiecewiseJerkPath(
    const std::vector<double>& x, const std::vector<double>& dx,
    const std::vector<double>& ddx, const double delta_s,
    const double start_s) const {
  ACHECK(!x.empty());
  ACHECK(!dx.empty());
  ACHECK(!ddx.empty());

  PiecewiseJerkTrajectory1d piecewise_jerk_traj(x.front(), dx.front(),
                                                ddx.front());

  for (std::size_t i = 1; i < x.size(); ++i) {
    const auto dddl = (ddx[i] - ddx[i - 1]) / delta_s;
    piecewise_jerk_traj.AppendSegment(dddl, delta_s);
  }

  std::vector<common::FrenetFramePoint> frenet_frame_path;
  double accumulated_s = 0.0;
  while (accumulated_s < piecewise_jerk_traj.ParamLength()) {
    double l = piecewise_jerk_traj.Evaluate(0, accumulated_s);
    double dl = piecewise_jerk_traj.Evaluate(1, accumulated_s);
    double ddl = piecewise_jerk_traj.Evaluate(2, accumulated_s);

    common::FrenetFramePoint frenet_frame_point;
    frenet_frame_point.set_s(accumulated_s + start_s);
    frenet_frame_point.set_l(l);
    frenet_frame_point.set_dl(dl);
    frenet_frame_point.set_ddl(ddl);
    frenet_frame_path.push_back(std::move(frenet_frame_point));

    accumulated_s += kTrajectorySpaceResolution;
  }

  return FrenetFramePath(std::move(frenet_frame_path));
}

double PathGenerator::EstimateJerkBoundary(const double vehicle_speed,
                                           const double axis_distance,
                                           const double max_yaw_rate) const {
  return max_yaw_rate / axis_distance / vehicle_speed;
}

double PathGenerator::GaussianWeighting(const double x,
                                        const double peak_weighting,
                                        const double peak_weighting_x) const {
  double std = 1 / (std::sqrt(2 * M_PI) * peak_weighting);
  double u = peak_weighting_x * std;
  double x_updated = x * std;
  return Gaussian(u, std, x_updated);
}

}  // namespace planning_btree
}  // namespace apollo