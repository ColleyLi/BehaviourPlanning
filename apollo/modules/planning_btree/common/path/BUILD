load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning_btree\\\""]

cc_library(
    name = "discretized_path",
    srcs = ["discretized_path.cc"],
    hdrs = ["discretized_path.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/math:linear_interpolation",
        "//modules/common/proto:pnc_point_cc_proto",
    ],
)

cc_library(
    name = "path_boundary",
    srcs = ["path_boundary.cc"],
    hdrs = ["path_boundary.h"],
    copts = PLANNING_COPTS,
)

cc_test(
    name = "discretized_path_test",
    size = "small",
    srcs = ["discretized_path_test.cc"],
    deps = [
        ":discretized_path",
        "//modules/common/util",
        "//modules/common/util:point_factory",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "frenet_frame_path",
    srcs = ["frenet_frame_path.cc"],
    hdrs = ["frenet_frame_path.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/math:linear_interpolation",
        "//modules/planning_btree/proto:sl_boundary_cc_proto",
    ],
)

cc_test(
    name = "frenet_frame_path_test",
    size = "small",
    srcs = ["frenet_frame_path_test.cc"],
    deps = [
        ":frenet_frame_path",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "path_data",
    srcs = ["path_data.cc"],
    hdrs = ["path_data.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":discretized_path",
        ":frenet_frame_path",
        "//modules/common/math:cartesian_frenet_conversion",
        "//modules/planning_btree/reference_line",
    ],
)

cpplint()
