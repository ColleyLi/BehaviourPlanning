#include "modules/planning_btree/common/btree_planning_gflags.h"

#include <limits>
DEFINE_string(btree_traffic_rule_config_filename,
             "/apollo/modules/planning_btree/conf/traffic_rule_config.pb.txt",
             "behaviour tree planner traffic rule config file");
DEFINE_string(btplan_file,
             "/apollo/modules/planning_btree/conf/btplan.pb.txt",
             "behaviour tree plan file");


// General planning flags
DEFINE_double(btree_planning_loop_rate, 10, "Planning rate");

// Reference Line flags
DEFINE_double(btree_default_city_road_speed_limit, 11.1111,
              "default speed limit (m/s) for city road. 40 km/h");
DEFINE_double(btree_default_highway_speed_limit, 25,
              "default speed limit (m/s) for highway. 90 km/h");
DEFINE_double(btree_planning_upper_speed_limit, 36.1111,
              "Maximum speed (m/s) in planning. 130 km/h");
DEFINE_int32(btree_trajectory_point_num_for_debug, 10,
             "number of output trajectory points for debugging");

// Reference line provider flags
DEFINE_bool(btree_enable_reference_line_provider_thread, true,
            "Enable reference line provider thread");
DEFINE_double(btree_default_reference_line_width, 4.0,
              "Default reference line width");
DEFINE_double(btree_smoothed_reference_line_max_diff, 5.0,
              "Maximum position difference between the smoothed and the raw "
              "reference lines");
DEFINE_bool(btree_prioritize_change_lane, false,
            "change lane strategy has higher priority, always use a valid "
            "change lane path if such path exists");
DEFINE_double(btree_change_lane_min_length, 30.0,
              "meters. If the change lane target has longer length than this "
              "threshold, it can shortcut the default lane");
DEFINE_double(btree_look_forward_extend_distance, 50,
              "The step size when extending reference line");
DEFINE_double(btree_look_backward_distance, 50,
              "The step size when extending reference line");
DEFINE_double(btree_reference_line_stitch_overlap_distance, 20,
              "The overlap distance with the existing reference line when "
              "stitching the existing reference line");
DEFINE_bool(btree_generate_neighbors, false,
            "whether to generate all neighbors to the current lane");
// Smoother flags
DEFINE_string(btree_smoother_config_filename,
              "/apollo/modules/planning_btree/conf/smoother_configs/qp_spline_smoother_config.pb.txt",
              "The configuration file for qp_spline smoother");
DEFINE_bool(btree_enable_smooth_reference_line, true,
            "enable smooth the map reference line");

// QP solver flags
DEFINE_bool(btree_enable_osqp_debug, false,
            "True to turn on OSQP verbose debug output in log");
DEFINE_bool(btree_enable_reference_line_stitching, true,
            "Enable stitching reference line, which can reducing computing "
            "time and improve stability");