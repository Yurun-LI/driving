cc_library(
    name = "constant",
    hdrs = ["constant.h"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "vec",
    hdrs = ["vec.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":constant",
    ],
)

cc_library(
    name = "pose",
    hdrs = ["pose.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":utils",
        ":vec",
    ],
)

cc_library(
    name = "utils",
    hdrs = ["utils.h"],
    visibility = ["//visibility:public"],
    deps = [
    ],
)
