load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//third_party/qt5:qt.bzl", "qt_cc_library")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "btviz",
    copts = [
        "-Iexternal/qt",
    ],
    includes = [
        ".",
    ],
    linkopts = [
        "-pthread",
    ],
    deps = [
        ":btviz_lib",
        "@fastrtps",
        "@qt//:qt_core",
        "@qt//:qt_gui",
        "@qt//:qt_opengl",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "btviz_lib",
    srcs = [
        "main.cc",
        "mainwindow.cc",
        "btviz_canvas.cc",
    ],
    hdrs = [
        "mainwindow.h",
        "btviz_canvas.h",
    ],
    copts = [
        "-Iexternal/qt",
    ],
    includes = [
        ".",
    ],
    linkstatic = False,
    uis = glob([
        "uis/*.ui",
    ]),
    deps = [
        ":btviz_base_lib",
        "//cyber",
        "//modules/tools/btviz/node_editor:node_editor_lib",
        "//modules/tools/btviz/flowscenes:btviz_flowscenes_lib",
        "//modules/tools/btviz/models:btviz_models_lib",
        "//modules/tools/btviz/resources:btviz_resource_lib",
        "@qt//:qt_core",
        "@qt//:qt_gui",
        "@qt//:qt_opengl",
        "@qt//:qt_widgets",
        "@qt//:qt_svg",
    ],
)


qt_cc_library(
    name = "btviz_base_lib",
    srcs = [
        "btviz_base.cc"
    ],
    hdrs = [
        "btviz_base.h",
    ],
    copts = [
        "-Iexternal/qt",
    ],
    includes = [
        ".",
    ],
    linkstatic = False,
    deps = [
        "//cyber",
        "//modules/planning_btree/proto:btree_planning_state_cc_proto",
        "//modules/planning_btree/proto:btree_context_config_cc_proto",
        "//modules/planning_btree/proto:btree_stage_config_cc_proto",
        "//modules/planning_btree/proto:btree_config_cc_proto",
        "@qt//:qt_core",
        "@qt//:qt_gui",
        "@qt//:qt_opengl",
        "@qt//:qt_widgets",
    ],
)

# cpplint()
