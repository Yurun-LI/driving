EIGEN_FILES = glob([
    "eigen-3.4-rc1/Eigen/**",
    "eigen-3.4-rc1/unsupported/Eigen/CXX11/**",
    "eigen-3.4-rc1/unsupported/Eigen/FFT",
    "eigen-3.4-rc1/unsupported/Eigen/KroneckerProduct",
    "eigen-3.4-rc1/unsupported/Eigen/src/FFT/**",
    "eigen-3.4-rc1/unsupported/Eigen/src/KroneckerProduct/**",
    "eigen-3.4-rc1/unsupported/Eigen/MatrixFunctions",
    "eigen-3.4-rc1/unsupported/Eigen/SpecialFunctions",
    "eigen-3.4-rc1/unsupported/Eigen/src/MatrixFunctions/**",
    "eigen-3.4-rc1/unsupported/Eigen/src/SpecialFunctions/**",
])

cc_library(
    name = "eigen3",
    hdrs = EIGEN_FILES,
    includes = ["."],
    visibility = ["//visibility:public"],
)