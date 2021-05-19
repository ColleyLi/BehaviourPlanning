load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning_btree\\\""]

cc_library(
    name = "planning_input",
    hdrs = ["planning_input.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        # "//modules/map/relative_map/proto:navigation_cc_proto",
        # "//modules/perception/proto:traffic_light_detection_cc_proto",
        # "//modules/planning/proto:pad_msg_cc_proto",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/routing/proto:routing_cc_proto",
        # "//modules/storytelling/proto:story_cc_proto",
    ],
)

cc_library(
    name = "btree_frame",
    srcs = ["btree_frame.cc"],
    hdrs = ["btree_frame.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning_btree/common/utils:indexed_queue",
        ":planning_input",
        ":dynamic_reference_line",
        "//cyber/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/monitor_log",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/pnc_map",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/planning_btree/reference_line:reference_line_provider",
        ":obstacle",
        "@eigen",
    ],
)

cc_library(
    name = "btree_planning_gflags",
    srcs = ["btree_planning_gflags.cc"],
    hdrs = ["btree_planning_gflags.h"],
    copts = PLANNING_COPTS,
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "dependency_injector",
    hdrs = ["dependency_injector.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning_btree/proto:btree_planning_state_cc_proto",
    ],
)

cc_library(
    name = "dynamic_reference_line",
    srcs = ["dynamic_reference_line.cc"],
    hdrs = ["dynamic_reference_line.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber/common:log",
        "//modules/common/math",
        "//modules/common/proto:pnc_point_cc_proto",
        "//modules/common/util:point_factory",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/pnc_map",
        "//modules/map/proto:map_cc_proto",
        "//modules/planning_btree/reference_line",
        "//modules/planning_btree/common/path:path_data",
        "//modules/planning_btree/common/speed:speed_data",
        "//modules/planning_btree/common/speed:st_graph_data",
        "//modules/planning_btree/common/path:path_boundary",
        "//modules/planning_btree/common/trajectory:discretized_trajectory",
        ":obstacle",
        ":obstacle_decisions",
        "@eigen",
    ],
)

cc_library(
    name = "trajectory_stitcher",
    srcs = ["trajectory_stitcher.cc"],
    hdrs = ["trajectory_stitcher.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber/common:log",
        "//modules/common/proto:pnc_point_cc_proto",
        "//modules/common/vehicle_model",
        "//modules/planning_btree/common/trajectory:publishable_trajectory",
        "//modules/planning_btree/reference_line",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "obstacle",
    srcs = ["obstacle.cc"],
    hdrs = ["obstacle.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/util:map_util",
        "//modules/planning_btree/common/utils:indexed_list",
        "//modules/planning_btree/common/speed:st_boundary",
        "//modules/planning_btree/reference_line",
        "//modules/planning_btree/proto:decision_cc_proto",
        "//modules/prediction/proto:feature_cc_proto",
        "//modules/prediction/proto:lane_graph_cc_proto",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/prediction/proto:prediction_point_cc_proto",
        "//modules/prediction/proto:scenario_cc_proto",
    ],
)

cc_library(
    name = "obstacle_decisions",
    srcs = ["obstacle_decisions.cc"],
    hdrs = ["obstacle_decisions.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":obstacle",
        "//modules/planning_btree/reference_line",
        "//modules/planning_btree/proto:decision_cc_proto",
    ],
)

cpplint()