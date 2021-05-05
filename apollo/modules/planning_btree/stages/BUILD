load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning_btree\\\""]

cc_library(
    name = "stage",
    srcs = [
        "stage.cc",
    ],
    hdrs = [
        "stage.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning_btree/common:btree_frame",
        "//modules/planning_btree/behaviours:btree_node",
        "//modules/planning_btree/behaviours:btree_node_dispatcher",
        "//modules/planning_btree/proto:btree_stage_config_cc_proto",
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
    copts = PLANNING_COPTS,
    deps = [
        ":stage",
        "//modules/planning_btree/common:btree_frame",
        "//modules/planning_btree/stages/lane_follow:lane_follow_stage",
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
    copts = PLANNING_COPTS,
    deps = [
        ":stage",
        ":stage_dispatcher",
        "//modules/planning_btree/common:btree_frame",
        "//cyber/common:file",
        "//modules/planning_btree/proto:btree_context_config_cc_proto",
        "//modules/planning_btree/common:btree_planning_gflags",
        "//modules/planning_btree/common:dependency_injector",
    ],
)

cpplint()