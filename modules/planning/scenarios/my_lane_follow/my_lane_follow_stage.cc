#include "modules/planning/scenarios/my_lane_follow/my_lane_follow_stage.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/common/util/string_util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/constraint_checker/constraint_checker.h"
#include "modules/planning/tasks/deciders/lane_change_decider/lane_change_decider.h"
#include "modules/planning/tasks/deciders/path_decider/path_decider.h"
#include "modules/planning/tasks/deciders/speed_decider/speed_decider.h"
#include "modules/planning/tasks/optimizers/path_time_heuristic/path_time_heuristic_optimizer.h"

namespace apollo 
{
namespace planning 
{
namespace scenario 
{
namespace lane_follow 
{

using common::ErrorCode;
using common::SLPoint;
using common::Status;
using common::TrajectoryPoint;
using common::time::Clock;

namespace 
{
constexpr double kPathOptimizationFallbackCost = 2e4;
constexpr double kSpeedOptimizationFallbackCost = 2e4;
constexpr double kStraightForwardLineCost = 10.0;
}  // namespace

MyLaneFollowStage::MyLaneFollowStage(const ScenarioConfig::StageConfig& config)
    : Stage(config) {}

Stage::StageStatus MyLaneFollowStage::Process(const TrajectoryPoint& planning_start_point, Frame* frame) 
{
  ADEBUG << "Number of reference lines:\t" << frame->mutable_reference_line_info()->size();

  bool has_drivable_lane = false;
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) 
  {
    auto cur_status = PlanOnReferenceLine(planning_start_point, frame, &reference_line_info);

    if (cur_status.ok()) 
    {
      if (reference_line_info.IsChangeLanePath()) 
      {
        if (LaneChangeDecider::IsClearToChangeLane(&reference_line_info)) 
        {
          has_drivable_lane = true;
          reference_line_info.SetDrivable(true);
        } 
        else 
        {
          reference_line_info.SetDrivable(false);
        }
      } 
      else 
      {
        has_drivable_lane = true;
      }
    } 
    else 
    {
      reference_line_info.SetDrivable(false);
    }
  }

  ADEBUG << "Reference lines after planning: ";
  for (const auto &reference_line_info : frame->reference_line_info()) 
  {
    ADEBUG << "Id: " << reference_line_info.Lanes().Id() << "Cost: " << reference_line_info.Cost() << " Drivable: " << reference_line_info.IsDrivable();
  }

  return has_drivable_lane ? StageStatus::RUNNING : StageStatus::ERROR;
}

Status MyLaneFollowStage::PlanOnReferenceLine(
    const TrajectoryPoint& planning_start_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) 
{
  //if (!reference_line_info->IsChangeLanePath()) 
  //{
  //   reference_line_info->AddCost(kStraightForwardLineCost);
  //}

  auto ret = Status::OK();
  // Propose to integrate behaviour tree here
  for (auto* task : task_list_) 
  {
    // const double start_timestamp = Clock::NowInSeconds();

    ret = task->Execute(frame, reference_line_info);
    
    if (!ret.ok()) 
    {
      AERROR << "Failed to run task [" << task->Name() << "], Error message: " << ret.error_message();
      break;
    }
  }

  // Check path and speed results for path or speed fallback
  reference_line_info->set_trajectory_type(ADCTrajectory::NORMAL);
  
  //if (!ret.ok()) 
  // {
  //   PlanFallbackTrajectory(planning_start_point, frame, reference_line_info);
  // }

  DiscretizedTrajectory trajectory;
  if (!reference_line_info->CombinePathAndSpeedProfile(
          planning_start_point.relative_time(),
          planning_start_point.path_point().s(), &trajectory)) 
  {
    std::string msg("Failed to aggregate path and speed into trajectory");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  /*
  // determine if there is a destination on reference line.
  double dest_stop_s = -1.0;
  for (const auto* obstacle : reference_line_info->path_decision()->obstacles().Items()) 
  {
    if (obstacle->LongitudinalDecision().has_stop() &&
        obstacle->LongitudinalDecision().stop().reason_code() == STOP_REASON_DESTINATION)
    {
      SLPoint dest_sl = GetStopSL(obstacle->LongitudinalDecision().stop(),
                                  reference_line_info->reference_line());
      dest_stop_s = dest_sl.s();
    }
  }

  for (const auto* obstacle : reference_line_info->path_decision()->obstacles().Items())
  {
    if (obstacle->IsVirtual()) 
    {
      continue;
    }

    if (!obstacle->IsStatic()) 
    {
      continue;
    }

    if (obstacle->LongitudinalDecision().has_stop()) 
    {
      bool add_stop_obstacle_cost = false;
      if (dest_stop_s < 0.0) 
      {
        add_stop_obstacle_cost = true;
      } 
      else 
      {
        SLPoint stop_sl = GetStopSL(obstacle->LongitudinalDecision().stop(),
                                    reference_line_info->reference_line());
        if (stop_sl.s() < dest_stop_s) 
        {
          add_stop_obstacle_cost = true;
        }
      }
      if (add_stop_obstacle_cost) 
      {
        constexpr double kRefrenceLineStaticObsCost = 1e3;
        reference_line_info->AddCost(kRefrenceLineStaticObsCost);
      }
    }
  }

  if (FLAGS_enable_trajectory_check) 
  {
    if (ConstraintChecker::ValidTrajectory(trajectory) !=
        ConstraintChecker::Result::VALID) 
    {
      std::string msg("Current planning trajectory is not valid.");
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }
  */

  reference_line_info->SetTrajectory(trajectory);
  reference_line_info->SetDrivable(true);
  return Status::OK();
}

void MyLaneFollowStage::PlanFallbackTrajectory(
    const TrajectoryPoint& planning_start_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) 
{
  // Path and speed fallback
  if (reference_line_info->path_data().Empty()) 
  {
    AERROR << "Path fallback due to algorithm failure";
    GenerateFallbackPathProfile(reference_line_info,
                                reference_line_info->mutable_path_data());
    reference_line_info->AddCost(kPathOptimizationFallbackCost);
    reference_line_info->set_trajectory_type(ADCTrajectory::PATH_FALLBACK);
  }

  if (reference_line_info->trajectory_type() != ADCTrajectory::PATH_FALLBACK) 
  {
    if (!RetrieveLastFramePathProfile(reference_line_info, frame, reference_line_info->mutable_path_data()))
    {
      const auto& candidate_path_data = reference_line_info->GetCandidatePathData();
      for (const auto& path_data : candidate_path_data) 
      {
        if (path_data.path_label().find("self") != std::string::npos) 
        {
          *reference_line_info->mutable_path_data() = path_data;
          AERROR << "Use current frame self lane path as fallback ";
          break;
        }
      }
    }
  }

  AERROR << "Speed fallback due to algorithm failure";
  // TODO(Hongyi): refine the fall-back handling here.
  // To use piecewise jerk speed fallback, stop distance here
  // is an upper bound of s, not a target.
  // TODO(Jiacheng): move this stop_path_threshold to a gflag
  const double path_stop_distance =
      reference_line_info->path_data().discretized_path().Length();

  const double obstacle_stop_distance =
      reference_line_info->st_graph_data().is_initialized()
          ? reference_line_info->st_graph_data().min_s_on_st_boundaries()
          : std::numeric_limits<double>::infinity();

  const double curr_speed_distance =
      FLAGS_fallback_total_time *
      std::min({FLAGS_default_cruise_speed,
                reference_line_info->vehicle_state().linear_velocity()});

  *reference_line_info->mutable_speed_data() =
      SpeedProfileGenerator::GenerateFallbackSpeed(std::min(
          {path_stop_distance, obstacle_stop_distance, curr_speed_distance}));

  if (reference_line_info->trajectory_type() != ADCTrajectory::PATH_FALLBACK) 
  {
    reference_line_info->AddCost(kSpeedOptimizationFallbackCost);
    reference_line_info->set_trajectory_type(ADCTrajectory::SPEED_FALLBACK);
  }
}

void MyLaneFollowStage::GenerateFallbackPathProfile(
    const ReferenceLineInfo* reference_line_info, PathData* path_data) {
  const double unit_s = 1.0;
  const auto& reference_line = reference_line_info->reference_line();

  auto adc_point = EgoInfo::Instance()->start_point();
  DCHECK(adc_point.has_path_point());
  const auto adc_point_x = adc_point.path_point().x();
  const auto adc_point_y = adc_point.path_point().y();

  common::SLPoint adc_point_s_l;
  if (!reference_line.XYToSL({adc_point_x, adc_point_y}, &adc_point_s_l)) {
    AERROR << "Fail to project ADC to reference line when calculating path "
              "fallback. Straight forward path is generated";
    const auto adc_point_heading = adc_point.path_point().theta();
    const auto adc_point_kappa = adc_point.path_point().kappa();
    const auto adc_point_dkappa = adc_point.path_point().dkappa();
    std::vector<common::PathPoint> path_points;
    double adc_traversed_x = adc_point_x;
    double adc_traversed_y = adc_point_y;

    const double max_s = 100.0;
    for (double s = 0; s < max_s; s += unit_s) {
      common::PathPoint path_point = common::util::MakePathPoint(
          adc_traversed_x, adc_traversed_y, 0.0, adc_point_heading,
          adc_point_kappa, adc_point_dkappa, 0.0);
      path_point.set_s(s);
      path_points.push_back(std::move(path_point));
      adc_traversed_x += unit_s * std::cos(adc_point_heading);
      adc_traversed_y += unit_s * std::sin(adc_point_heading);
    }
    path_data->SetDiscretizedPath(DiscretizedPath(std::move(path_points)));
    return;
  }

  // Generate a fallback path along the reference line direction
  const auto adc_s = adc_point_s_l.s();
  const auto& adc_ref_point =
      reference_line.GetReferencePoint(adc_point_x, adc_point_y);
  const double dx = adc_point_x - adc_ref_point.x();
  const double dy = adc_point_y - adc_ref_point.y();

  std::vector<common::PathPoint> path_points;
  const double max_s = reference_line.Length();
  for (double s = adc_s; s < max_s; s += unit_s) {
    const auto& ref_point = reference_line.GetReferencePoint(s);
    common::PathPoint path_point = common::util::MakePathPoint(
        ref_point.x() + dx, ref_point.y() + dy, 0.0, ref_point.heading(),
        ref_point.kappa(), ref_point.dkappa(), 0.0);
    path_point.set_s(s - adc_s);
    path_points.push_back(std::move(path_point));
  }
  path_data->SetDiscretizedPath(DiscretizedPath(std::move(path_points)));
}

bool MyLaneFollowStage::RetrieveLastFramePathProfile(
    const ReferenceLineInfo* reference_line_info, const Frame* frame,
    PathData* path_data) 
{
  const auto* ptr_last_frame = FrameHistory::Instance()->Latest();
  if (ptr_last_frame == nullptr) 
  {
    AERROR << "Last frame doesn't succeed, fail to retrieve last frame path data";
    return false;
  }

  const auto& last_frame_discretized_path =
      ptr_last_frame->current_frame_planned_path();

  path_data->SetDiscretizedPath(last_frame_discretized_path);
  const auto adc_frenet_frame_point_ =
      reference_line_info->reference_line().GetFrenetPoint(
          frame->PlanningStartPoint().path_point());

  bool trim_success = path_data->LeftTrimWithRefS(adc_frenet_frame_point_);
  if (!trim_success) 
  {
    AERROR << "Fail to trim path_data. adc_frenet_frame_point: "
           << adc_frenet_frame_point_.ShortDebugString();
    return false;
  }

  AERROR << "Use last frame good path to do speed fallback";
  return true;
}

SLPoint MyLaneFollowStage::GetStopSL(const ObjectStop& stop_decision,
                                   const ReferenceLine& reference_line) const 
{
  SLPoint sl_point;
  reference_line.XYToSL(
      {stop_decision.stop_point().x(), stop_decision.stop_point().y()}, &sl_point);
  return sl_point;
}

void MyLaneFollowStage::RecordObstacleDebugInfo(
    ReferenceLineInfo* reference_line_info) 
{
  if (!FLAGS_enable_record_debug) 
  {
    ADEBUG << "Skip record debug info";
    return;
  }

  auto ptr_debug = reference_line_info->mutable_debug();
  const auto path_decision = reference_line_info->path_decision();
  for (const auto obstacle : path_decision->obstacles().Items()) 
  {
    auto obstacle_debug = ptr_debug->mutable_planning_data()->add_obstacle();
    obstacle_debug->set_id(obstacle->Id());
    obstacle_debug->mutable_sl_boundary()->CopyFrom(obstacle->PerceptionSLBoundary());

    const auto& decider_tags = obstacle->decider_tags();
    const auto& decisions = obstacle->decisions();
    if (decider_tags.size() != decisions.size()) 
    {
      AERROR << "decider_tags size: " << decider_tags.size()
             << " different from decisions size:" << decisions.size();
    }

    for (size_t i = 0; i < decider_tags.size(); ++i) 
    {
      auto decision_tag = obstacle_debug->add_decision_tag();
      decision_tag->set_decider_tag(decider_tags[i]);
      decision_tag->mutable_decision()->CopyFrom(decisions[i]);
    }
  }
}


void MyLaneFollowStage::RecordDebugInfo(ReferenceLineInfo* reference_line_info,
                                      const std::string& name,
                                      const double time_diff_ms) 
{
  if (!FLAGS_enable_record_debug) 
  {
    ADEBUG << "Skip record debug info";
    return;
  }

  if (reference_line_info == nullptr) 
  {
    AERROR << "Reference line info is null.";
    return;
  }

  auto ptr_latency_stats = reference_line_info->mutable_latency_stats();

  auto ptr_stats = ptr_latency_stats->add_task_stats();
  ptr_stats->set_name(name);
  ptr_stats->set_time_ms(time_diff_ms);
}

}  // namespace lane_follow
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
