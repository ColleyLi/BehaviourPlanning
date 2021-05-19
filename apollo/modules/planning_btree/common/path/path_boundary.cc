#include "modules/planning_btree/common/path/path_boundary.h"

namespace apollo {
namespace planning_btree {

PathBoundary::PathBoundary(const double start_s, const double delta_s,
                           std::vector<std::pair<double, double>> path_boundary)
    : start_s_(start_s),
      delta_s_(delta_s),
      boundary_(std::move(path_boundary)) {}

double PathBoundary::start_s() const { return start_s_; }

double PathBoundary::delta_s() const { return delta_s_; }

void PathBoundary::set_boundary(
    const std::vector<std::pair<double, double>>& boundary) {
  boundary_ = boundary;
}

const std::vector<std::pair<double, double>>& PathBoundary::boundary() const {
  return boundary_;
}

void PathBoundary::set_label(const std::string& label) { label_ = label; }

const std::string& PathBoundary::label() const { return label_; }

void PathBoundary::set_blocking_obstacle_id(const std::string& obs_id) {
  blocking_obstacle_id_ = obs_id;
}

const std::string& PathBoundary::blocking_obstacle_id() const {
  return blocking_obstacle_id_;
}

}  // namespace planning_btree
}  // namespace apollo
