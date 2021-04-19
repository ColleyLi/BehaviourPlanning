#include "modules/planning/planner/btree/btree_planner.h"

#include "gtest/gtest.h"
#include "modules/common/proto/drive_state.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

TEST(BTreePlannerTest, Simple) {
  auto injector = std::make_shared<DependencyInjector>();
  BTreePlanner btree_planner(injector);
  PlanningConfig config;
  EXPECT_EQ(btree_planner.Name(), "BTREE");
  EXPECT_EQ(btree_planner.Init(config), common::Status::OK());
}

}  // namespace planning
}  // namespace apollo