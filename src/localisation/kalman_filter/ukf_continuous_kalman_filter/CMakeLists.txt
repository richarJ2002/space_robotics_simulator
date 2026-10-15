cmake_minimum_required(VERSION 3.16)

# Reusable, model-agnostic continuous-discrete UKF math engine: takes
# caller-propagated and caller-observed sigma points for a runtime-sized
# state, performs only the weighted recombination algebra. See
# ContinuousUnscentedKalmanFilterClass.h.
add_library(ukf_continuous_kalman_filter STATIC
  methods/initialize.cc
  methods/calculateSigmaWeights.cc
  methods/generateSigmaPoints.cc
  methods/predictFromSigmaPoints.cc
  methods/calculateUpdateNis.cc
  methods/updateFromSigmaPoints.cc
  methods/terminate.cc
  methods/getState.cc
  methods/getCovariance.cc
  methods/setState.cc
  methods/restore.cc
  methods/isCovarianceValid.cc
)
target_compile_features(ukf_continuous_kalman_filter PUBLIC cxx_std_17)
target_include_directories(ukf_continuous_kalman_filter
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ukf_continuous_kalman_filter PUBLIC Eigen3::Eigen)

install(TARGETS ukf_continuous_kalman_filter ARCHIVE DESTINATION lib)
