load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "discretized_trajectory",
    srcs = ["discretized_trajectory.cc"],
    hdrs = ["discretized_trajectory.h"],
    deps = [
        "//modules/common/math:linear_interpolation",
        "//modules/common/proto:pnc_point_cc_proto",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "//modules/planning/proto:planning_cc_proto",
        "@eigen",
    ],
)

cc_library(
    name = "publishable_trajectory",
    srcs = ["publishable_trajectory.cc"],
    hdrs = ["publishable_trajectory.h"],
    deps = [
        ":discretized_trajectory",
        "//modules/planning/proto:planning_cc_proto",
    ],
)

cpplint()
