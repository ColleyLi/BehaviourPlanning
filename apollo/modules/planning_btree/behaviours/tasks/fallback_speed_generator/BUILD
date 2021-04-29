load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning_btree\\\""]

cc_library(
    name = "fallback_speed_generator",
    srcs = [
        "fallback_speed_generator.cc",
    ],
    hdrs = [
        "fallback_speed_generator.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning_btree/common:btree_frame",
        "//modules/planning_btree/behaviours:btree_task",
    ],
)

cpplint()