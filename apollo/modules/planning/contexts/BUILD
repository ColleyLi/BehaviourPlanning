load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stage",
    srcs = [
        "stage.cc",
    ],
    hdrs = [
        "stage.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:planning_common",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/behaviour_tree:b_tree_node",
        "//modules/planning/behaviour_tree:b_tree_node_dispatcher",
    ],
)

cc_library(
    name = "stage_dispatcher",
    srcs = [
        "stage_dispatcher.cc",
    ],
    hdrs = [
        "stage_dispatcher.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":stage",
        "//modules/planning/common:planning_common",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/contexts/lane_follow/stages:lane_follow_stage",
    ],
)

cc_library(
    name = "stage_selector",
    srcs = [
        "stage_selector.cc",
    ],
    hdrs = [
        "stage_selector.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":stage",
        ":stage_dispatcher",
        "//modules/planning/common:planning_common",
        "//modules/planning/common/util:util_lib",
    ],
)

cc_library(
    name = "context",
    srcs = [
        "context.cc",
    ],
    hdrs = [
        "context.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":stage_selector",
        "//modules/planning/common:planning_common",
        "//modules/planning/common/util:util_lib",
    ],
)

cc_library(
    name = "context_dispatcher",
    srcs = [
        "context_dispatcher.cc",
    ],
    hdrs = [
        "context_dispatcher.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":context",
        "//modules/planning/common:planning_common",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/contexts/lane_follow:lane_follow_context",
    ],
)

cc_library(
    name = "context_selector",
    srcs = [
        "context_selector.cc",
    ],
    hdrs = [
        "context_selector.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":context",
        ":context_dispatcher",
        "//modules/planning/common:planning_common",
        "//modules/planning/common/util:util_lib",
    ],
)