load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "bt_task",
    srcs = [
        "bt_task.cc",
    ],
    hdrs = [
        "bt_task.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/behaviour_tree:bt_node",
    ],
)

cc_library(
    name = "bt_sequence",
    srcs = [
        "bt_sequence.cc",
    ],
    hdrs = [
        "bt_sequence.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/behaviour_tree:bt_composite_node",
    ],
)

cc_library(
    name = "bt_selector",
    srcs = [
        "bt_selector.cc",
    ],
    hdrs = [
        "bt_selector.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/behaviour_tree:bt_composite_node",
    ],
)


cc_library(
    name = "bt_composite_node",
    srcs = [
        "bt_composite_node.cc",
    ],
    hdrs = [
        "bt_composite_node.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/behaviour_tree:bt_node",
    ],
)


cc_library(
    name = "bt_node",
    srcs = [
        "bt_node.cc",
    ],
    hdrs = [
        "bt_node.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:frame",
    ],
)

cpplint()
