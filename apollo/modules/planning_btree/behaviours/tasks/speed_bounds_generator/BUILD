load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning_btree\\\""]

cc_library(
    name = "speed_bounds_generator",
    srcs = [
        "speed_bounds_generator.cc",
    ],
    hdrs = [
        "speed_bounds_generator.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning_btree/common:btree_frame",
        "//modules/planning_btree/behaviours:btree_task",
        ":st_boundary_mapper",
        ":st_driving_limits",
        ":st_guide_line",
        ":st_obstacles_processor",
    ],
)

cc_library(
    name = "st_boundary_mapper",
    srcs = [
        "speed_limit_decider.cc",
        "st_boundary_mapper.cc",
    ],
    hdrs = [
        "speed_limit_decider.h",
        "st_boundary_mapper.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/map/pnc_map",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/configs/proto:vehicle_config_cc_proto",
        "//modules/planning_btree/common:obstacle",
        "//modules/planning_btree/common:obstacle_decisions",
        "//modules/planning_btree/common/speed:speed_limit",
        "//modules/planning_btree/common/path:discretized_path",
        "//modules/planning_btree/common/path:path_data",
        "//modules/planning_btree/common/speed:st_boundary",
        "//modules/planning_btree/reference_line",
    ],
)


cc_library(
    name = "st_driving_limits",
    srcs = ["st_driving_limits.cc"],
    hdrs = ["st_driving_limits.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/configs/proto:vehicle_config_cc_proto",
        "//modules/common/status",
        "//modules/map/pnc_map",
        "//modules/planning_btree/common:obstacle",
        "//modules/planning_btree/common:obstacle_decisions",
        "//modules/planning_btree/common/speed:speed_limit",
        "//modules/planning_btree/common/path:path_data",
        "//modules/planning_btree/common/speed:st_boundary",
        "//modules/planning_btree/reference_line",
    ],
)

cc_library(
    name = "st_guide_line",
    srcs = ["st_guide_line.cc"],
    hdrs = ["st_guide_line.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/configs/proto:vehicle_config_cc_proto",
        "//modules/common/proto:pnc_point_cc_proto",
        "//modules/common/status",
        "//modules/map/pnc_map",
        "//modules/map/proto:map_cc_proto",
        "//modules/planning_btree/common:obstacle",
        "//modules/planning_btree/common:obstacle_decisions",
        "//modules/planning_btree/common/speed:speed_limit",
        "//modules/planning_btree/common/path:discretized_path",
        "//modules/planning_btree/common/path:frenet_frame_path",
        "//modules/planning_btree/common/path:path_data",
        "//modules/planning_btree/common/speed:speed_data",
        "//modules/planning_btree/common/speed:st_boundary",
        "//modules/planning_btree/common/trajectory:discretized_trajectory",
        "//modules/planning_btree/reference_line",
    ],
)


cc_library(
    name = "st_obstacles_processor",
    srcs = ["st_obstacles_processor.cc"],
    hdrs = ["st_obstacles_processor.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/configs/proto:vehicle_config_cc_proto",
        "//modules/common/proto:pnc_point_cc_proto",
        "//modules/common/status",
        "//modules/map/pnc_map",
         "//modules/planning_btree/common:obstacle",
        "//modules/planning_btree/common:obstacle_decisions",
        "//modules/planning_btree/common/speed:speed_limit",
        "//modules/planning_btree/common/path:discretized_path",
        "//modules/planning_btree/common/path:frenet_frame_path",
        "//modules/planning_btree/common/path:path_data",
        "//modules/planning_btree/common/speed:speed_data",
        "//modules/planning_btree/common/speed:st_boundary",
        "//modules/planning_btree/common/trajectory:discretized_trajectory",
        "//modules/planning_btree/reference_line",
    ],
)

cpplint()