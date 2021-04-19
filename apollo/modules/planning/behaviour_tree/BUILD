load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "b_tree_check",
    srcs = [
        "b_tree_check.cc",
    ],
    hdrs = [
        "b_tree_check.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:frame",
        "//modules/planning/behaviour_tree:b_tree_node",
    ],
)

cc_library(
    name = "b_tree_task",
    srcs = [
        "b_tree_task.cc",
    ],
    hdrs = [
        "b_tree_task.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:frame",
        "//modules/planning/behaviour_tree:b_tree_node",
    ],
)

cc_library(
    name = "b_tree_sequence",
    srcs = [
        "b_tree_sequence.cc",
    ],
    hdrs = [
        "b_tree_sequence.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:frame",
        "//modules/planning/behaviour_tree:b_tree_node",
    ],
)

cc_library(
    name = "b_tree_selector",
    srcs = [
        "b_tree_selector.cc",
    ],
    hdrs = [
        "b_tree_selector.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:frame",
        "//modules/planning/behaviour_tree:b_tree_node",
    ],
)

cc_library(
    name = "b_tree_node",
    srcs = [
        "b_tree_node.cc",
    ],
    hdrs = [
        "b_tree_node.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:frame",
    ],
)

cc_library(
    name = "b_tree_node_dispatcher",
    srcs = [
        "b_tree_node_dispatcher.cc",
    ],
    hdrs = [
        "b_tree_node_dispatcher.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":b_tree_node",
        ":b_tree_selector",
        ":b_tree_sequence",
        "//modules/planning/common:planning_common",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/behaviour_tree/checks:collision_check",
        "//modules/planning/behaviour_tree/checks:safe_lane_change_check",
        "//modules/planning/behaviour_tree/priority_selectors:lane_priority_selector",
        "//modules/planning/behaviour_tree/tasks/fallback_path_generator:fallback_path_generator",
        "//modules/planning/behaviour_tree/tasks/fallback_speed_generator:fallback_speed_generator",
        "//modules/planning/behaviour_tree/tasks/lane_prioritizer:lane_prioritizer",
        "//modules/planning/behaviour_tree/tasks/obstacle_processor:obstacle_processor",
        "//modules/planning/behaviour_tree/tasks/path_generator:path_generator",
        "//modules/planning/behaviour_tree/tasks/speed_generator:speed_generator",
    ],
)
cpplint()
