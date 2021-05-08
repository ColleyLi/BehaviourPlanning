load("//third_party/qt5:qt.bzl", "qt_cc_library")

package(default_visibility = ["//visibility:public"])

qt_cc_library(
    name = "btviz_models_lib",
    srcs = glob(
        ["*.cc"],
    ),
    hdrs = glob([
        "*.h",
    ]),
    copts = [
        "-Iexternal/qt",
    ],
    includes = [
    ],
    linkstatic = False,
    res = [
    ],
    uis = glob([
    ]),
    deps = [
        "//modules/tools/btviz:btviz_base_lib",
        "//modules/tools/btviz/node_editor:node_editor_lib",
        "//modules/planning_btree/proto:btree_context_config_cc_proto",
        "//modules/planning_btree/proto:btree_stage_config_cc_proto",
        "//modules/planning_btree/proto:btree_config_cc_proto",
        "@qt//:qt_core",
        "@qt//:qt_gui",
        "@qt//:qt_opengl",
        "@qt//:qt_widgets",
    ],
)