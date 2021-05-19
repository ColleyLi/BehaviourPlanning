load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning_btree\\\""]

cc_library(
    name = "destination",
    srcs = [
        "destination.cc",
    ],
    hdrs = [
        "destination.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning_btree/traffic_rules:traffic_rule",
    ],
)

cpplint()
