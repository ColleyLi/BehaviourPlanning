load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning_btree\\\""]

cc_library(
    name = "st_point",
    srcs = ["st_point.cc"],
    hdrs = ["st_point.h"],
    deps = [
        "//modules/common/math",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "st_boundary",
    srcs = ["st_boundary.cc"],
    hdrs = ["st_boundary.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":st_point",
        "//cyber/common:log",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "speed_limit",
    srcs = ["speed_limit.cc"],
    hdrs = ["speed_limit.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/math",
    ],
)

cc_library(
    name = "st_graph_data",
    srcs = ["st_graph_data.cc"],
    hdrs = ["st_graph_data.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":speed_limit",
        "//modules/planning_btree/common/speed:st_boundary",
        "//modules/planning_btree/proto:st_drivable_boundary_cc_proto",
    ],
)

cc_test(
    name = "st_boundary_test",
    size = "small",
    srcs = ["st_boundary_test.cc"],
    copts = PLANNING_COPTS,
    deps = [
        ":st_boundary",
        "//cyber/common:log",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "speed_data",
    srcs = ["speed_data.cc"],
    hdrs = ["speed_data.h"],
    deps = [
        ":st_point",
        "//modules/common/math",
        "//modules/common/proto:pnc_point_cc_proto",
        "//modules/common/util",
        "//modules/common/util:point_factory",
        "//modules/common/util:string_util",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
