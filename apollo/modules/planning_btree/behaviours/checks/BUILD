load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning_btree_btree\\\""]

cc_library(
    name = "collision_check",
    srcs = [
        "collision_check.cc",
    ],
    hdrs = [
        "collision_check.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning_btree/common:btree_frame",
        "//modules/planning_btree/behaviours:btree_check",
    ],
)

cc_library(
    name = "safe_lane_change_check",
    srcs = [
        "safe_lane_change_check.cc",
    ],
    hdrs = [
        "safe_lane_change_check.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning_btree/common:btree_frame",
        "//modules/planning/tasks/deciders/lane_change_decider",
        "//modules/planning_btree/behaviours:btree_check",
    ],
)

cpplint()