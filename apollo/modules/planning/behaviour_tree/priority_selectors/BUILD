load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lane_priority_selector",
    srcs = [
        "lane_priority_selector.cc",
    ],
    hdrs = [
        "lane_priority_selector.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:frame",
        "//modules/planning/behaviour_tree:b_tree_node",
    ],
)

cpplint()