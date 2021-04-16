load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "speed_generator",
    srcs = [
        "speed_generator.cc",
    ],
    hdrs = [
        "speed_generator.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common",
        "//modules/planning/common:planning_common",
        "//modules/planning/common/util:util_lib",
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/behaviour_tree:bt_task",
        "//modules/planning/tasks/deciders/speed_bounds_decider",
        "//modules/planning/tasks/optimizers/path_time_heuristic:path_time_heuristic_optimizer",
    ],
)

cpplint()