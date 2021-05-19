load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning_btree\\\""]

cc_library(
    name = "path_bounds_generator",
    srcs = [
        "path_bounds_generator.cc",
    ],
    hdrs = [
        "path_bounds_generator.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning_btree/common:btree_frame",
        "//modules/planning_btree/behaviours:btree_task",
        "//modules/planning_btree/common/path:path_boundary",
    ],
)

cpplint()