#pragma once

#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning_btree/proto/sl_boundary.pb.h"

namespace apollo {
namespace planning_btree {

class FrenetFramePath : public std::vector<common::FrenetFramePoint> {
 public:
  FrenetFramePath() = default;
  explicit FrenetFramePath(std::vector<common::FrenetFramePoint> points);

  double Length() const;
  common::FrenetFramePoint EvaluateByS(const double s) const;

  /**
   * @brief Get the FrenetFramePoint that is within SLBoundary, or the one with
   * smallest l() in SLBoundary's s range [start_s(), end_s()]
   */
  common::FrenetFramePoint GetNearestPoint(const SLBoundary &sl) const;

 private:
  static bool LowerBoundComparator(const common::FrenetFramePoint &p,
                                   const double s) {
    return p.s() < s;
  }
  static bool UpperBoundComparator(const double s,
                                   const common::FrenetFramePoint &p) {
    return s < p.s();
  }
};

}  // namespace planning_btree
}  // namespace apollo
