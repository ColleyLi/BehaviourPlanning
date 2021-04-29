load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning_btree\\\""]

cc_library(
    name = "lane_priority_selector",
    srcs = [
        "lane_priority_selector.cc",
    ],
    hdrs = [
        "lane_priority_selector.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning_btree/common:btree_frame",
        "//modules/planning_btree/behaviours:btree_node",
    ],
)

cpplint()