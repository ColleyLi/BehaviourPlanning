#include "modules/planning/tasks/my_tasks/my_task/my_task.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/planning.pb.h"
// #include "modules/planning/common/path_boundary.h"
// #include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/speed_profile_generator.h"

namespace apollo
{
namespace planning 
{

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::SpeedPoint;
using apollo::common::VehicleConfigHelper;
using apollo::common::util::StrCat;

namespace   
{
using PathBoundPoint = std::tuple<double, double, double>;
using PathBound = std::vector<PathBoundPoint>;
const double kPathBoundsDeciderResolution = 0.5;
const double kPathBoundsDeciderHorizon = 100.0;
const double kDefaultLaneWidth = 5.0;
const double kDefaultRoadWidth = 20.0;
}

MyTask::MyTask(const TaskConfig &config) : Task(config) 
{
  SetName("MyTask");
}

apollo::common::Status MyTask::Execute(Frame *frame, ReferenceLineInfo *reference_line_info) 
{
  return Process(frame, reference_line_info);
}

Status MyTask::Process(Frame *frame, ReferenceLineInfo *reference_line_info) 
{
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  const ReferenceLine& reference_line = reference_line_info->reference_line();
  const common::TrajectoryPoint& cur_ego_position = frame->PlanningStartPoint();
  
  std::vector<common::FrenetFramePoint> frenet_frame_path;
  SpeedData speed_data;

  double cur_s = reference_line.GetFrenetPoint(cur_ego_position.path_point()).s();
  double dt = 0.1;
  double v = 6.0;
  double planning_time_horizon = 3.0;
  for (double t = 0; t < planning_time_horizon; t+=dt)
  {
    common::FrenetFramePoint frenet_frame_point;
    frenet_frame_point.set_s(t * v + cur_s);
    frenet_frame_path.push_back(std::move(frenet_frame_point));
    speed_data.AppendSpeedPoint(t * v, t, v, 0.0, 0.0);
  }

  PathData path_data = *(reference_line_info->mutable_path_data());
  path_data.SetReferenceLine(&reference_line);
  path_data.SetFrenetPath(FrenetFramePath(frenet_frame_path));
  *(reference_line_info->mutable_path_data()) = path_data;
  *(reference_line_info->mutable_speed_data()) = speed_data;

  ADEBUG << "Path after my task";
  ADEBUG << reference_line_info->PathSpeedDebugString();

  return Status::OK();
}

}  //  namespace planning
}  //  namespace apollo
