#include "modules/planning_btree/behaviours/tasks/speed_generator/st_gap_estimator.h"

#include <algorithm>
#include <cmath>

namespace apollo {
namespace planning_btree {

// TODO(Jinyun): move to configs
static constexpr double kOvertakeTimeBuffer = 3.0;    // in seconds
static constexpr double kMinOvertakeDistance = 10.0;  // in meters
static constexpr double kDpSafetyDistance = 20.0;     // in meters
static constexpr double kDpSafetyTimeBuffer = 3.0;    // in meters
static constexpr double kYieldDistance = 5.0;
static constexpr double kFollowMinDistance = 3.0;
static constexpr double kFollowTimeBuffer = 2.5;

// TODO(Jinyun): unite gap calculation in dp st and speed decider
double StGapEstimator::EstimateSafeOvertakingGap() { return kDpSafetyDistance; }

double StGapEstimator::EstimateSafeFollowingGap(const double target_obs_speed) {
  return target_obs_speed * kDpSafetyTimeBuffer;
}

double StGapEstimator::EstimateSafeYieldingGap() {
  return kYieldDistance;
}

// TODO(Jinyun): add more variables to overtaking gap calculation
double StGapEstimator::EstimateProperOvertakingGap(
    const double target_obs_speed, const double adc_speed) {
  const double overtake_distance_s =
      std::fmax(std::fmax(adc_speed, target_obs_speed) * kOvertakeTimeBuffer,
                kMinOvertakeDistance);
  return overtake_distance_s;
}

// TODO(Jinyun): add more variables to follow gap calculation
double StGapEstimator::EstimateProperFollowingGap(const double adc_speed) {
  return std::fmax(adc_speed * kFollowTimeBuffer,
                   kFollowMinDistance);
}

// TODO(Jinyun): add more variables to yielding gap calculation
double StGapEstimator::EstimateProperYieldingGap() {
  return kYieldDistance;
}

}  // namespace planning_btree
}  // namespace apollo
