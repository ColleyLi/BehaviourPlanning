#pragma once

#include "gflags/gflags.h"


DECLARE_string(btree_context_config_file);
DECLARE_string(btree_stage_config_file);
DECLARE_string(btplan_file);

// General planning flags
DECLARE_double(btree_planning_loop_rate);

// Reference Line flags
DECLARE_double(btree_default_city_road_speed_limit);
DECLARE_double(btree_default_highway_speed_limit);
DECLARE_double(btree_planning_upper_speed_limit);
DECLARE_int32(btree_trajectory_point_num_for_debug);

// Reference line provider flags
DECLARE_bool(btree_enable_reference_line_provider_thread);
DECLARE_double(btree_default_reference_line_width);
DECLARE_double(btree_smoothed_reference_line_max_diff);
DECLARE_bool(btree_prioritize_change_lane);
DECLARE_double(btree_change_lane_min_length);
DECLARE_double(btree_look_forward_extend_distance);
DECLARE_double(btree_look_backward_distance);
DECLARE_double(btree_reference_line_stitch_overlap_distance);
DECLARE_bool(btree_generate_neighbors);

// Smoother flags
DECLARE_string(btree_smoother_config_filename);
DECLARE_bool(btree_enable_smooth_reference_line);

// QP solver flags
DECLARE_bool(btree_enable_osqp_debug);
DECLARE_bool(btree_enable_reference_line_stitching);