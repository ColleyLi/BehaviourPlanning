#pragma once

#include <string>
#include <vector>

#include "modules/planning_btree/common/trajectory1d/constant_jerk_trajectory1d.h"
#include "modules/planning_btree/math/curve1d/curve1d.h"

namespace apollo {
namespace planning_btree {

class PiecewiseJerkTrajectory1d : public Curve1d {
 public:
  PiecewiseJerkTrajectory1d(const double p, const double v, const double a);

  virtual ~PiecewiseJerkTrajectory1d() = default;

  double Evaluate(const std::uint32_t order, const double param) const;

  double ParamLength() const;

  std::string ToString() const;

  void AppendSegment(const double jerk, const double param);

 private:
  std::vector<ConstantJerkTrajectory1d> segments_;

  double last_p_;

  double last_v_;

  double last_a_;

  std::vector<double> param_;
};

}  // namespace planning_btree
}  // namespace apollo
