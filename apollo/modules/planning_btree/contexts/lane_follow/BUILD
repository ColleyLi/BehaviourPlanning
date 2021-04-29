load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning_btree\\\""]

cc_library(
    name = "lane_follow_context",
    srcs = [
        "lane_follow_context.cc",
    ],
    hdrs = [
        "lane_follow_context.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning_btree/contexts:context",
    ],
)