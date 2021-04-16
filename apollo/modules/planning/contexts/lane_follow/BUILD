load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lane_follow_context",
    srcs = [
        "lane_follow_context.cc",
    ],
    hdrs = [
        "lane_follow_context.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common",
        "//modules/planning/common:planning_common",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/contexts:context",
    ],
)