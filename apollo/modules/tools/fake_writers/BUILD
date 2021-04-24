load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "fake_localization_writer",
    srcs = ["fake_localization_writer.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/perception/proto:perception_obstacle_py_pb2",
        "//modules/localization/proto:imu_py_pb2",
        "//modules/localization/proto:gps_py_pb2",
        "//modules/localization/proto:localization_py_pb2",
        "//modules/drivers/gnss/proto:ins_py_pb2",
        "//modules/canbus/proto:chassis_py_pb2",
    ],
)