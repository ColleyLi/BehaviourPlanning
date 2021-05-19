load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning_btree\\\""]

cc_library(
    name = "btree_check",
    srcs = [
        "btree_check.cc",
    ],
    hdrs = [
        "btree_check.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning_btree/common:btree_frame",
        "//modules/planning_btree/behaviours:btree_node",
    ],
)

cc_library(
    name = "btree_task",
    srcs = [
        "btree_task.cc",
    ],
    hdrs = [
        "btree_task.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning_btree/common:btree_frame",
        "//modules/planning_btree/behaviours:btree_node",
    ],
)

cc_library(
    name = "btree_sequence",
    srcs = [
        "btree_sequence.cc",
    ],
    hdrs = [
        "btree_sequence.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning_btree/common:btree_frame",
        "//modules/planning_btree/behaviours:btree_node",
    ],
)

cc_library(
    name = "btree_selector",
    srcs = [
        "btree_selector.cc",
    ],
    hdrs = [
        "btree_selector.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning_btree/common:btree_frame",
        "//modules/planning_btree/behaviours:btree_node",
    ],
)

cc_library(
    name = "btree_node",
    srcs = [
        "btree_node.cc",
    ],
    hdrs = [
        "btree_node.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning_btree/common:btree_frame",
        "//modules/planning_btree/proto:btree_config_cc_proto",
    ],
)

cc_library(
    name = "btree_node_dispatcher",
    srcs = [
        "btree_node_dispatcher.cc",
    ],
    hdrs = [
        "btree_node_dispatcher.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        ":btree_node",
        ":btree_selector",
        ":btree_sequence",
        "//modules/planning_btree/behaviours/checks:collision_check",
        "//modules/planning_btree/behaviours/checks:safe_lane_change_check",
        "//modules/planning_btree/behaviours/priority_selectors:lane_priority_selector",
        "//modules/planning_btree/behaviours/tasks/fallback_path_generator:fallback_path_generator",
        "//modules/planning_btree/behaviours/tasks/fallback_speed_generator:fallback_speed_generator",
        "//modules/planning_btree/behaviours/tasks/lane_prioritizer:lane_prioritizer",
        "//modules/planning_btree/behaviours/tasks/obstacle_processor:obstacle_processor",
        "//modules/planning_btree/behaviours/tasks/path_bounds_generator:path_bounds_generator",
        "//modules/planning_btree/behaviours/tasks/path_generator:path_generator",
        "//modules/planning_btree/behaviours/tasks/speed_bounds_generator:speed_bounds_generator",
        "//modules/planning_btree/behaviours/tasks/speed_generator:speed_generator",
    ],
)
cpplint()
