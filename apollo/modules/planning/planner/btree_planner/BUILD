load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "btree_planner",
    srcs = [
        "btree_planner.cc",
    ],
    hdrs = [
        "btree_planner.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//external:gflags",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/util:factory",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap",
        "//modules/planning/common:planning_common",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/planner",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/reference_line",
        "//modules/planning/contexts:context_selector",
        "@eigen",
    ],
)

cc_test(
    name = "btree_planner_test",
    size = "small",
    srcs = [
        "btree_planner_test.cc",
    ],
    deps = [
        ":btree_planner",
        "@gtest//:main",
    ],
)

cpplint()

