load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lane_prioritizer",
    srcs = [
        "lane_prioritizer.cc",
    ],
    hdrs = [
        "lane_prioritizer.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:frame",
        "//modules/planning/behaviour_tree:b_tree_task",
    ],
)

cpplint()