#pragma once

#include <utility>
#include <vector>

namespace apollo {
namespace planning_btree {

class SpeedLimit {
 public:
  SpeedLimit() = default;

  void AppendSpeedLimit(const double s, const double v);

  const std::vector<std::pair<double, double>>& speed_limit_points() const;

  double GetSpeedLimitByS(const double s) const;

  void Clear();

 private:
  // use a vector to represent speed limit
  // the first number is s, the second number is v
  // It means at distance s from the start point, the speed limit is v.
  std::vector<std::pair<double, double>> speed_limit_points_;
};

}  // namespace planning_btree
}  // namespace apollo
