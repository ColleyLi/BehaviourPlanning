load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning_btree\\\""]


cc_library(
    name = "traffic_rule_dispatcher",
    srcs = ["traffic_rule_dispatcher.cc"],
    hdrs = ["traffic_rule_dispatcher.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":traffic_rule",
        "//modules/common/status",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning_btree/common:btree_frame",
        "//modules/planning_btree/common:dynamic_reference_line",
        "//modules/planning_btree/proto:traffic_rule_config_cc_proto",
        "//modules/planning_btree/traffic_rules/rules:destination",
    ],
)

cc_library(
    name = "traffic_rule",
    srcs = [
        "traffic_rule.cc",
    ],
    hdrs = [
        "traffic_rule.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/util:factory",
        "//modules/common/util:map_util",
        "//modules/perception/proto:perception_obstacle_cc_proto",
        "//modules/planning_btree/common:dependency_injector",
        "//modules/planning_btree/common:btree_frame",
        "//modules/planning_btree/common:dynamic_reference_line",
        "//modules/planning_btree/common:obstacle",
        "//modules/planning_btree/common:obstacle_decisions",
        "//modules/planning_btree/proto:traffic_rule_config_cc_proto",
    ],
)

cpplint()
