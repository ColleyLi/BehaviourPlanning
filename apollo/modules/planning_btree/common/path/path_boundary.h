#pragma once

#include <string>
#include <utility>
#include <vector>

namespace apollo {
namespace planning_btree {

class PathBoundary {
 public:
  PathBoundary(const double start_s, const double delta_s,
               std::vector<std::pair<double, double>> path_boundary);

  virtual ~PathBoundary() = default;

  double start_s() const;

  double delta_s() const;

  void set_boundary(const std::vector<std::pair<double, double>>& boundary);
  const std::vector<std::pair<double, double>>& boundary() const;

  void set_label(const std::string& label);
  const std::string& label() const;

  void set_blocking_obstacle_id(const std::string& obs_id);
  const std::string& blocking_obstacle_id() const;

 private:
  double start_s_ = 0.0;
  double delta_s_ = 0.0;
  std::vector<std::pair<double, double>> boundary_;
  std::string label_ = "regular";
  std::string blocking_obstacle_id_ = "";
};

}  // namespace planning_btree
}  // namespace apollo
