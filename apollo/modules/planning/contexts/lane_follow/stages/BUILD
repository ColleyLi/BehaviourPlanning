load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lane_follow_stage",
    srcs = [
        "lane_follow_stage.cc",
    ],
    hdrs = [
        "lane_follow_stage.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common",
        "//modules/planning/common:planning_common",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/contexts:stage",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/behaviour_tree:b_tree_node",
        "//modules/planning/behaviour_tree:b_tree_sequence",
        "//modules/planning/behaviour_tree:b_tree_selector",
        "//modules/planning/behaviour_tree/priority_selectors:lane_priority_selector",
        "//modules/planning/behaviour_tree/checks:collision_check",
        "//modules/planning/behaviour_tree/checks:safe_lane_change_check",
        "//modules/planning/behaviour_tree/tasks/path_generator:path_generator",
        "//modules/planning/behaviour_tree/tasks/speed_generator:speed_generator",
        "//modules/planning/behaviour_tree/tasks/fallback_speed_generator:fallback_speed_generator",
        "//modules/planning/behaviour_tree/tasks/fallback_path_generator:fallback_path_generator",
        "//modules/planning/behaviour_tree/tasks/lane_prioritizer:lane_prioritizer",
        "//modules/planning/behaviour_tree/tasks/obstacle_processor:obstacle_processor",
    ],
)