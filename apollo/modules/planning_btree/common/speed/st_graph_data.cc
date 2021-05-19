#include "modules/planning_btree/common/speed/st_graph_data.h"

namespace apollo {
namespace planning_btree {

// TODO: move this to config
namespace
{
  constexpr double kDefaultCruiseSpeed = 11.0;
}

using apollo::common::TrajectoryPoint;

void StGraphData::LoadData(const std::vector<const STBoundary*>& st_boundaries,
                           const double min_s_on_st_boundaries,
                           const apollo::common::TrajectoryPoint& init_point,
                           const SpeedLimit& speed_limit,
                           const double cruise_speed,
                           const double path_data_length,
                           const double total_time_by_conf) {
  init_ = true;
  st_boundaries_ = st_boundaries;
  min_s_on_st_boundaries_ = min_s_on_st_boundaries;
  init_point_ = init_point;
  speed_limit_ = speed_limit;
  cruise_speed_ = cruise_speed;
  path_data_length_ = path_data_length;
  total_time_by_conf_ = total_time_by_conf;
}

const std::vector<const STBoundary*>& StGraphData::st_boundaries() const {
  return st_boundaries_;
}

double StGraphData::min_s_on_st_boundaries() const {
  return min_s_on_st_boundaries_;
}

const TrajectoryPoint& StGraphData::init_point() const { return init_point_; }

const SpeedLimit& StGraphData::speed_limit() const { return speed_limit_; }

double StGraphData::cruise_speed() const {
  return cruise_speed_ > 0.0 ? cruise_speed_ : kDefaultCruiseSpeed;
}

double StGraphData::path_length() const { return path_data_length_; }

double StGraphData::total_time_by_conf() const { return total_time_by_conf_; }

bool StGraphData::SetSTDrivableBoundary(
    const std::vector<std::tuple<double, double, double>>& s_boundary,
    const std::vector<std::tuple<double, double, double>>& v_obs_info) {
  if (s_boundary.size() != v_obs_info.size()) {
    return false;
  }
  for (size_t i = 0; i < s_boundary.size(); ++i) {
    auto st_bound_instance = st_drivable_boundary_.add_st_boundary();
    st_bound_instance->set_t(std::get<0>(s_boundary[i]));
    st_bound_instance->set_s_lower(std::get<1>(s_boundary[i]));
    st_bound_instance->set_s_upper(std::get<2>(s_boundary[i]));
    if (std::get<1>(v_obs_info[i]) > -kObsSpeedIgnoreThreshold) {
      st_bound_instance->set_v_obs_lower(std::get<1>(v_obs_info[i]));
    }
    if (std::get<2>(v_obs_info[i]) < kObsSpeedIgnoreThreshold) {
      st_bound_instance->set_v_obs_upper(std::get<2>(v_obs_info[i]));
    }
  }
  return true;
}

const STDrivableBoundary& StGraphData::st_drivable_boundary() const {
  return st_drivable_boundary_;
}

}  // namespace planning_btree
}  // namespace apollo
