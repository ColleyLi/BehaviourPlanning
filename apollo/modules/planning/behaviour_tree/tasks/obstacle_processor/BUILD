load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "obstacle_processor",
    srcs = [
        "obstacle_processor.cc",
    ],
    hdrs = [
        "obstacle_processor.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:frame",
        "//modules/planning/behaviour_tree:b_tree_task",
    ],
)

cpplint()