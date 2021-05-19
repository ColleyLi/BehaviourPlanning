#pragma once

#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

namespace apollo {
namespace planning_btree {

class DiscretizedPath : public std::vector<common::PathPoint> {
 public:
  DiscretizedPath() = default;

  explicit DiscretizedPath(std::vector<common::PathPoint> path_points);

  double Length() const;

  common::PathPoint Evaluate(const double path_s) const;

  common::PathPoint EvaluateReverse(const double path_s) const;

 protected:
  std::vector<common::PathPoint>::const_iterator QueryLowerBound(
      const double path_s) const;
  std::vector<common::PathPoint>::const_iterator QueryUpperBound(
      const double path_s) const;
};

}  // namespace planning_btree
}  // namespace apollo
