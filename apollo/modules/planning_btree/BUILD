load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "lib_planning_btree_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":btree_planning_component_lib"],
)

cc_library(
    name = "btree_planning_component_lib",
    srcs = ["btree_planning_component.cc"],
    hdrs = ["btree_planning_component.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning_btree\\\"",
    ],
    deps = [
        "//cyber",
        ":btree_planning_base",
        ":btree_planner",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
        "//modules/localization/proto:localization_cc_proto",
        # "//modules/map/relative_map/proto:navigation_cc_proto",
        # "//modules/perception/proto:traffic_light_detection_cc_proto",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/planning_btree/proto:btree_planning_config_cc_proto",
        "//modules/map/pnc_map:pnc_map",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "btree_planning_base",
    srcs = ["btree_planning_base.cc"],
    hdrs = ["btree_planning_base.h"],
    copts = [
    ],
    deps = [
        ":btree_planner",
        "//cyber",
        "//modules/common/configs:config_gflags",
        "//modules/common/math:quaternion",
        "//modules/common/proto:pnc_point_cc_proto",
        "//modules/common/util:future",
        "//modules/common/util:message_util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/perception/proto:traffic_light_detection_cc_proto",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/planning_btree/proto:btree_planning_config_cc_proto",
        "//modules/planning_btree/common:planning_input",
    ],
)

cc_library(
    name = "btree_planner",
    srcs = ["btree_planner.cc"],
    hdrs = ["btree_planner.h"],
    copts = [
    ],
    deps = [
        "//modules/common/util:message_util",
        "//modules/planning_btree/proto:btree_planning_config_cc_proto",
        "//modules/planning_btree/common:planning_input",
        "//modules/planning_btree/common:btree_frame",
        "//modules/planning/proto:planning_cc_proto",       
        "//modules/planning_btree/contexts:context_selector",
    ],
)

filegroup(
    name = "btree_planning_conf",
    srcs = glob([
        "conf/**",
    ]),
)

cpplint()
