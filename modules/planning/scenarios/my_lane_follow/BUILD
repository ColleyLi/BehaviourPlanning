load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "my_lane_follow",
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":my_lane_follow_scenario",
        ":my_lane_follow_stage",
    ],
)

cc_library(
    name = "my_lane_follow_stage",
    srcs = [
        "my_lane_follow_stage.cc",
    ],
    hdrs = [
        "my_lane_follow_stage.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//external:gflags",
        "//modules/common",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/util:factory",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap",
        "//modules/planning/common:planning_common",
        "//modules/planning/common:speed_profile_generator",
        "//modules/planning/constraint_checker",
        "//modules/planning/math/curve1d:quartic_polynomial_curve1d",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/reference_line",
        "//modules/planning/reference_line:qp_spline_reference_line_smoother",
        "//modules/planning/scenarios:scenario",
        "//modules/planning/behaviour_tree:bt_node",
        "//modules/planning/behaviour_tree:bt_sequence",
        "//modules/planning/behaviour_tree:bt_selector",
        "//modules/planning/behaviour_tree/priority_selectors:lane_priority_selector",
        "//modules/planning/behaviour_tree/checks:collision_check",
        "//modules/planning/behaviour_tree/checks:safe_lane_change_check",
        "//modules/planning/behaviour_tree/tasks/path_generator:path_generator",
        "//modules/planning/behaviour_tree/tasks/speed_generator:speed_generator",
        "//modules/planning/behaviour_tree/tasks/fallback_speed_generator:fallback_speed_generator",
        "//modules/planning/behaviour_tree/tasks/fallback_path_generator:fallback_path_generator",
        "//modules/planning/behaviour_tree/tasks/lane_prioritizer:lane_prioritizer",
        "//modules/planning/behaviour_tree/tasks/obstacle_processor:obstacle_processor",
        "@eigen",
    ],
)

cc_library(
    name = "my_lane_follow_scenario",
    srcs = [
        "my_lane_follow_scenario.cc",
    ],
    hdrs = [
        "my_lane_follow_scenario.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":my_lane_follow_stage",
        "//cyber/common:log",
        "//external:gflags",
        "//modules/common",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/util:factory",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap",
        "//modules/planning/common:planning_common",
        "//modules/planning/common:speed_profile_generator",
        "//modules/planning/constraint_checker",
        "//modules/planning/math/curve1d:quartic_polynomial_curve1d",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/reference_line",
        "//modules/planning/reference_line:qp_spline_reference_line_smoother",
        "//modules/planning/scenarios:scenario",
        "//modules/planning/tasks/deciders/path_decider",
        "//modules/planning/tasks/deciders/speed_decider",
        "//modules/planning/tasks/optimizers:path_optimizer",
        "//modules/planning/tasks/optimizers:speed_optimizer",
        "//modules/planning/tasks/optimizers/path_time_heuristic:path_time_heuristic_optimizer",
        "@eigen",
    ],
)

cc_test(
    name = "my_lane_follow_scenario_test",
    size = "small",
    srcs = [
        "my_lane_follow_scenario_test.cc",
    ],
    data = [
        "//modules/planning:planning_conf",
    ],
    deps = [
        ":my_lane_follow",
        "@gtest//:main",
    ],
)

cpplint()
