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
        "//modules/common/status",
        "//modules/planning/common:planning_common",
        "//modules/planning/common:dependency_injector",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/common:frame",
        "//modules/planning/behaviour_tree:b_tree_task",
        "//modules/planning/tasks/deciders/speed_bounds_decider:st_boundary_mapper",
        "//modules/planning/tasks/optimizers/path_time_heuristic:gridded_path_time_graph",
    ],
)

cpplint()