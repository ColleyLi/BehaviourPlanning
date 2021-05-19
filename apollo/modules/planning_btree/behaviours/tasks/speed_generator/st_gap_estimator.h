namespace apollo {
namespace planning_btree {
class StGapEstimator {
 public:
  StGapEstimator() = delete;

  virtual ~StGapEstimator() = delete;

  static double EstimateSafeOvertakingGap();

  static double EstimateSafeFollowingGap(const double target_obs_speed);

  static double EstimateSafeYieldingGap();

  static double EstimateProperOvertakingGap(const double target_obs_speed,
                                            const double adc_speed);

  static double EstimateProperFollowingGap(const double adc_speed);

  static double EstimateProperYieldingGap();
};
}  // namespace planning_btree
}  // namespace apollo
