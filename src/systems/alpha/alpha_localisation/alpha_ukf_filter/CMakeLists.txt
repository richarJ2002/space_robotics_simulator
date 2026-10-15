cmake_minimum_required(VERSION 3.16)

# Alpha's comparison-only UKF model: roll/pitch/yaw state, per-source
# nonlinear observation mappings, NIS gating and rollback/replay. Wraps the
# model-agnostic ukf_continuous_kalman_filter engine
# (src/localisation/kalman_filter/) and exposes the fused estimate as
# odometry plus a retained path; it never broadcasts TF and never gates
# READY. See AlphaUkfFilterNodeClass.h.
add_library(alpha_ukf_filter STATIC
  methods/setNoiseTriplet.cc
  methods/rotationFromRpy.cc
  methods/rpyFromRotation.cc
  methods/wrapAngle.cc
  methods/selectNisThreshold.cc
  methods/measurementVariances.cc
  methods/isPoseValid.cc
  methods/stampToNanoseconds.cc
  methods/propagateSigmaPoint.cc
  methods/initializeFilter.cc
  methods/predictTo.cc
  methods/saveFilterCheckpoint.cc
  methods/restoreFilterCheckpointAtOrBefore.cc
  methods/discardFilterCheckpointsAfter.cc
  methods/clearFilterCheckpoints.cc
  methods/buildMeasurementRecord.cc
  methods/fuseMeasurementRecord.cc
  methods/replayMeasurementsAfter.cc
  methods/handleImuCallBack.cc
  methods/handleMeasurementCallBack.cc
  methods/logStepFailure.cc
  methods/publishEstimate.cc
  methods/prepareOdometry.cc
  methods/appendPathPose.cc
  methods/publishEstimatedPath.cc
  methods/publishDiagnosticsCallBack.cc
  private_functions/predictVisualMeasurement.cc
  private_functions/predictWheelMeasurement.cc
  private_functions/makeSourceStatus.cc
  private_functions/countRejections.cc
)
target_compile_features(alpha_ukf_filter PUBLIC cxx_std_17)
target_include_directories(alpha_ukf_filter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(alpha_ukf_filter
  common_console
  common_diagnostics
  ukf_continuous_kalman_filter
  Eigen3::Eigen
)
ament_target_dependencies(alpha_ukf_filter
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS alpha_ukf_filter ARCHIVE DESTINATION lib)
