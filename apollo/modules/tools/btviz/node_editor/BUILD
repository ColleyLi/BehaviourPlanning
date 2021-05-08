load("//third_party/qt5:qt.bzl", "qt_cc_library")

package(default_visibility = ["//visibility:public"])

qt_cc_library(
    name = "node_editor_lib",
    srcs = glob(
        ["src/*.cc"],
    ),
    hdrs = glob([
        "src/*.h",
        "include/*.h",
    ]),
    copts = [
        "-Iexternal/qt",
    ],
    includes = [
        "./include",
    ],
    linkstatic = False,
    uis = glob([
        "*.ui",
    ]),
    deps = [
        "//modules/tools/btviz/resources:btviz_resource_lib",
        "@qt//:qt_core",
        "@qt//:qt_gui",
        "@qt//:qt_opengl",
        "@qt//:qt_widgets",
        "@qt//:qt_svg",
    ],
)