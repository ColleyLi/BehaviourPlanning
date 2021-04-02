#include "modules/planning/planner/my_planner/my_planner.h"

#include "gtest/gtest.h"

#include "modules/common/proto/drive_state.pb.h"
#include "modules/common/proto/pnc_point.pb.h"

#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo 
{
namespace planning 
{

TEST(MyPlannerTest, Simple) 
{
  MyPlanner my_planner;
  PlanningConfig config;
  EXPECT_EQ(my_planner.Name(), "MY_PLANNER");
  EXPECT_EQ(my_planner.Init(config), common::Status::OK());
}

}  // namespace planning
}  // namespace apollo
