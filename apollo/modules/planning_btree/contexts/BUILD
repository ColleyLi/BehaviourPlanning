load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning_btree\\\""]

cc_library(
    name = "context",
    srcs = [
        "context.cc",
    ],
    hdrs = [
        "context.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning_btree/stages:stage_selector",
        "//modules/planning_btree/common:btree_frame",
        "//modules/planning_btree/common:dependency_injector",
        "//modules/planning_btree/proto:btree_context_config_cc_proto",
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
    copts = PLANNING_COPTS,
    deps = [
        ":context",
        "//modules/planning_btree/common:btree_frame",
        "//modules/planning_btree/common:dependency_injector",
        "//modules/planning_btree/proto:btree_context_config_cc_proto",
        "//modules/planning_btree/contexts/lane_follow:lane_follow_context",
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
    copts = PLANNING_COPTS,
    deps = [
        ":context",
        ":context_dispatcher",
        "//modules/planning_btree/common:btree_frame",
        "//modules/planning_btree/common:dependency_injector",
        "//modules/planning_btree/proto:btree_context_config_cc_proto",
    ],
)

cpplint()