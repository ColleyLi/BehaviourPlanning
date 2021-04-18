load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "path_generator",
    srcs = [
        "path_generator.cc",
    ],
    hdrs = [
        "path_generator.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:frame",
        "//modules/planning/behaviour_tree:b_tree_task",
    ],
)

cpplint()