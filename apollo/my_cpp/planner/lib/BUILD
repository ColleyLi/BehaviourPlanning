package(default_visibility = ["//visibility:public"])

cc_library(
    name = "my_planner_lib",
    srcs = ["my_planner.cc"],
    hdrs = ["my_planner.h"],
    deps = [
        "//cyber",
        "//modules/common/util:message_util",
        "//modules/localization/proto:localization_proto",
        "//modules/map/relative_map/proto:navigation_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/prediction/proto:prediction_proto",
        "//modules/common/adapters:adapter_gflags",
        "//my_cpp/planner/lib/common:my_planning_gflags",
    ],
)
