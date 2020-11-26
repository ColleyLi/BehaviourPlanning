#define protected public
#define private public
#include "modules/planning/scenarios/my_lane_follow/my_lane_follow_scenario.h"

#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo 
{
namespace planning 
{
namespace scenario 
{
namespace lane_follow 
{

class MyLaneFollowScenarioTest : public ::testing::Test 
{
 public:
  virtual void SetUp() {}

 protected:
  std::unique_ptr<MyLaneFollowScenario> scenario_;
};

TEST_F(MyLaneFollowScenarioTest, VerifyConf) 
{
  FLAGS_scenario_lane_follow_config_file =
      "/apollo/modules/planning/conf/scenario/my_lane_follow_config.pb.txt";

  ScenarioConfig config;
  EXPECT_TRUE(apollo::cyber::common::GetProtoFromFile(
      FLAGS_scenario_lane_follow_config_file, &config));
}

TEST_F(MyLaneFollowScenarioTest, Init) 
{
  FLAGS_scenario_lane_follow_config_file =
      "/apollo/modules/planning/testdata/conf/"
      "scenario/my_lane_follow_config.pb.txt";

  ScenarioConfig config;
  EXPECT_TRUE(apollo::cyber::common::GetProtoFromFile(
      FLAGS_scenario_lane_follow_config_file, &config));
  ScenarioContext context;
  scenario_.reset(new MyLaneFollowScenario(config, &context));
  EXPECT_EQ(scenario_->scenario_type(), ScenarioConfig::LANE_FOLLOW);
}

}  // namespace lane_follow
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
