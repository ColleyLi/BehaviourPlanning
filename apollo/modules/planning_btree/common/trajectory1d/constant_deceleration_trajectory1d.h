#pragma once

#include <string>

#include "modules/planning_btree/math/curve1d/curve1d.h"

namespace apollo {
namespace planning_btree {

class ConstantDecelerationTrajectory1d : public Curve1d {
 public:
  ConstantDecelerationTrajectory1d(const double init_s, const double init_v,
                                   const double a);

  virtual ~ConstantDecelerationTrajectory1d() = default;

  double ParamLength() const override;

  std::string ToString() const override;

  // handles extrapolation internally
  double Evaluate(const std::uint32_t order, const double param) const override;

 private:
  double Evaluate_s(const double t) const;

  double Evaluate_v(const double t) const;

  double Evaluate_a(const double t) const;

  double Evaluate_j(const double t) const;

  double init_s_;

  double init_v_;

  double deceleration_;

  double end_t_;

  double end_s_;
};

}  // namespace planning_btree
}  // namespace apollo
