load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "collision_check",
    srcs = [
        "collision_check.cc",
    ],
    hdrs = [
        "collision_check.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/behaviour_tree:bt_node",
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
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/tasks/deciders/lane_change_decider",
        "//modules/planning/behaviour_tree:bt_node",
    ],
)

cpplint()