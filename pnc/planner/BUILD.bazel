cc_library(
    name = "state",
    srcs = ["state.cc"],
    hdrs = ["state.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//common/math:vec",
    ],
)

cc_library(
    name = "multi_hybrid_a_star",
    srcs = ["multi_hybrid_a_star.cc"],
    hdrs = ["multi_hybrid_a_star.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":state",
    ],
)

