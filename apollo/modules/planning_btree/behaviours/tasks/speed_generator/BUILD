load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning_btree\\\""]

cc_library(
    name = "speed_generator",
    srcs = [
        "speed_generator.cc",
    ],
    hdrs = [
        "speed_generator.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning_btree/common:btree_frame",
        "//modules/planning_btree/behaviours:btree_task",
        ":gridded_path_time_graph",
    ],
)


cc_library(
    name = "gridded_path_time_graph",
    srcs = ["gridded_path_time_graph.cc"],
    hdrs = ["gridded_path_time_graph.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":dp_st_cost",
        ":st_graph_point",
        "//cyber/common:log",
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/configs/proto:vehicle_config_cc_proto",
        "//modules/common/proto:geometry_cc_proto",
        "//modules/common/proto:pnc_point_cc_proto",
        "//modules/common/status",
        "//modules/planning_btree/common:obstacle",
        "//modules/planning_btree/common:obstacle_decisions",
        "//modules/planning_btree/common/speed:st_graph_data",
        "//modules/planning_btree/common/speed:speed_data",
    ],
)

cc_library(
    name = "st_gap_estimator",
    srcs = ["st_gap_estimator.cc"],
    hdrs = ["st_gap_estimator.h"],
    copts = PLANNING_COPTS,
)

cc_library(
    name = "st_graph_point",
    srcs = ["st_graph_point.cc"],
    hdrs = ["st_graph_point.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning_btree/common/speed:st_point",
    ],
)

cc_library(
    name = "dp_st_cost",
    srcs = ["dp_st_cost.cc"],
    hdrs = ["dp_st_cost.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":st_graph_point",
        ":st_gap_estimator",
        "//modules/common/proto:pnc_point_cc_proto",
        "//modules/planning_btree/common:obstacle",
        "//modules/planning_btree/common/speed:st_boundary",
        "//modules/planning_btree/proto:st_drivable_boundary_cc_proto",
    ],
)

cpplint()