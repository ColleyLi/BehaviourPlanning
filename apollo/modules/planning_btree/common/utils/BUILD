load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning_btree\\\""]


cc_library(
    name = "indexed_list",
    hdrs = ["indexed_list.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/util:map_util",
        "@boost",
    ],
)

cc_test(
    name = "indexed_list_test",
    size = "small",
    srcs = ["indexed_list_test.cc"],
    deps = [
        ":indexed_list",
        "//modules/common/util",
        "@boost",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "indexed_queue",
    hdrs = ["indexed_queue.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/util:map_util",
    ],
)

cc_test(
    name = "indexed_queue_test",
    size = "small",
    srcs = ["indexed_queue_test.cc"],
    deps = [
        ":indexed_queue",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
    ],
)