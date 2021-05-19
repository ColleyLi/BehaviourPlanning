load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning_btree\\\""]

cc_library(
    name = "path_generator",
    srcs = [
        "path_generator.cc",
    ],
    hdrs = [
        "path_generator.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning_btree/common:btree_frame",
        "//modules/planning_btree/behaviours:btree_task",
        "//modules/planning_btree/common/path:path_data",
        "//modules/planning_btree/math/piecewise_jerk:piecewise_jerk_path_problem",
        "//modules/planning_btree/common/trajectory1d:piecewise_jerk_trajectory1d",
        "//modules/planning_btree/common/trajectory1d:constant_jerk_trajectory1d",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/util",
    ],
)

cpplint()