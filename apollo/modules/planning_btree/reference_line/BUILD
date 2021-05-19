load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning_btree\\\""]

cc_library(
    name = "reference_line",
    srcs = [
        "reference_line.cc",
        "reference_point.cc",
    ],
    hdrs = [
        "reference_line.h",
        "reference_point.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber/common:log",
        "//modules/common/math",
        "//modules/common/proto:pnc_point_cc_proto",
        "//modules/planning_btree/proto:sl_boundary_cc_proto",
        "//modules/routing/proto:routing_cc_proto",
        "//modules/common/util:map_util",
        "//modules/common/util:point_factory",
        "//modules/map/pnc_map:path",
        "//modules/planning_btree/common:btree_planning_gflags",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "reference_line_smoother",
    srcs = [],
    hdrs = ["reference_line_smoother.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning_btree/proto:reference_line_smoother_config_cc_proto",
    ],
)

cc_library(
    name = "qp_spline_reference_line_smoother",
    srcs = ["qp_spline_reference_line_smoother.cc"],
    hdrs = ["qp_spline_reference_line_smoother.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":reference_line",
        ":reference_line_smoother",
        "//modules/planning_btree/math:curve_math",
        "//modules/planning_btree/math:polynomial_xd",
        "//modules/planning_btree/math/smoothing_spline:osqp_spline_2d_solver",
        "//modules/planning_btree/proto:reference_line_smoother_config_cc_proto",
        "//modules/planning_btree/proto:sl_boundary_cc_proto",
        "//modules/routing/proto:routing_cc_proto",
        "@eigen",
    ],
)

cc_library(
    name = "spiral_reference_line_smoother",
    srcs = [
        "spiral_problem_interface.cc",
        "spiral_reference_line_smoother.cc",
    ],
    hdrs = [
        "spiral_problem_interface.h",
        "spiral_reference_line_smoother.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        ":reference_line",
        ":reference_line_smoother",
        "//modules/common/math",
        "//modules/planning_btree/common:btree_planning_gflags",
        "//modules/planning_btree/math/curve1d:quintic_spiral_path",
        "//modules/planning_btree/math/smoothing_spline:spline_2d_solver",
        "//modules/planning_btree/proto:sl_boundary_cc_proto",
        "//modules/routing/proto:routing_cc_proto",
        "@eigen",
        "@ipopt",
    ],
)

cc_test(
    name = "spiral_reference_line_smoother_test",
    size = "small",
    srcs = ["spiral_reference_line_smoother_test.cc"],
    data = [
    ],
    deps = [
        ":spiral_reference_line_smoother",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "discrete_points_reference_line_smoother",
    srcs = ["discrete_points_reference_line_smoother.cc"],
    hdrs = ["discrete_points_reference_line_smoother.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":reference_line",
        ":reference_line_smoother",
        "//modules/common/math",
        "//modules/planning_btree/common:btree_planning_gflags",
        "//modules/planning_btree/math:discrete_points_math",
        "//modules/planning_btree/math/discretized_points_smoothing:cos_theta_smoother",
        "//modules/planning_btree/math/discretized_points_smoothing:fem_pos_deviation_smoother",
        "//modules/planning_btree/proto:reference_line_smoother_config_cc_proto",
        "//modules/planning_btree/proto:sl_boundary_cc_proto",
        "//modules/routing/proto:routing_cc_proto",
    ],
)

cc_test(
    name = "qp_spline_reference_line_smoother_test",
    size = "small",
    srcs = ["qp_spline_reference_line_smoother_test.cc"],
    data = [
    ],
    deps = [
        ":qp_spline_reference_line_smoother",
        "//modules/map/hdmap",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "discrete_points_reference_line_smoother_test",
    size = "small",
    srcs = ["discrete_points_reference_line_smoother_test.cc"],
    data = [
    ],
    deps = [
        ":discrete_points_reference_line_smoother",
        "//modules/map/hdmap",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "reference_line_provider",
    srcs = ["reference_line_provider.cc"],
    hdrs = ["reference_line_provider.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":discrete_points_reference_line_smoother",
        ":qp_spline_reference_line_smoother",
        ":reference_line",
        ":spiral_reference_line_smoother",
        "//cyber/time:clock",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/util:factory",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/pnc_map",
        "//modules/planning_btree/common/utils:indexed_queue",
        "//modules/planning_btree/proto:reference_line_smoother_config_cc_proto",
        "//modules/planning_btree/proto:sl_boundary_cc_proto",
        "@eigen",
    ],
)

cc_binary(
    name = "smoother_util",
    srcs = ["smoother_util.cc"],
    deps = [
        ":discrete_points_reference_line_smoother",
        ":qp_spline_reference_line_smoother",
        ":reference_line",
        ":spiral_reference_line_smoother",
    ],
)

cc_binary(
    name = "spiral_smoother_util",
    srcs = ["spiral_smoother_util.cc"],
    deps = [
        ":spiral_reference_line_smoother",
    ],
)

cpplint()
