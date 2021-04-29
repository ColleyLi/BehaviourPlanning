load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning_btree\\\""]

cc_library(
    name = "lane_follow_stage",
    srcs = [
        "lane_follow_stage.cc",
    ],
    hdrs = [
        "lane_follow_stage.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning_btree/common:btree_frame",
        "//modules/planning_btree/stages:stage",
        "//modules/planning_btree/proto:btree_config_cc_proto",
        "//modules/planning_btree/behaviours:btree_node",
        "//modules/planning_btree/behaviours:btree_sequence",
        "//modules/planning_btree/behaviours:btree_selector",
        "//modules/planning_btree/behaviours/priority_selectors:lane_priority_selector",
        "//modules/planning_btree/behaviours/checks:collision_check",
        "//modules/planning_btree/behaviours/checks:safe_lane_change_check",
        "//modules/planning_btree/behaviours/tasks/path_generator:path_generator",
        "//modules/planning_btree/behaviours/tasks/speed_generator:speed_generator",
        "//modules/planning_btree/behaviours/tasks/fallback_speed_generator:fallback_speed_generator",
        "//modules/planning_btree/behaviours/tasks/fallback_path_generator:fallback_path_generator",
        "//modules/planning_btree/behaviours/tasks/lane_prioritizer:lane_prioritizer",
        "//modules/planning_btree/behaviours/tasks/obstacle_processor:obstacle_processor",
    ],
)