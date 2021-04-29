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
        ":planning_input",
        "//cyber/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/monitor_log",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/pnc_map",
        "//modules/planning/proto:planning_cc_proto",
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
cpplint()