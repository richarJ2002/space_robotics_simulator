cmake_minimum_required(VERSION 3.16)

# Sliding-window keyframe Full bundle adjustment (poses + landmarks),
# Eigen-only and comparison-only. Owns the source-agnostic KeyframeWindow,
# the Schur-complement BundleAdjustmentOptimizer, and the
# KeyframeLocalOptimizationNode ROS wrapper; see
# objects/KeyframeLocalOptimizationNodeClass.h.
add_library(keyframe_local_optimization STATIC
  methods/KeyframeWindow/initialize.cc
  methods/KeyframeWindow/terminate.cc
  methods/KeyframeWindow/pushKeyframe.cc
  methods/KeyframeWindow/keyframeCount.cc
  methods/KeyframeWindow/landmarkCount.cc
  methods/KeyframeWindow/computeTotalError.cc
  methods/KeyframeWindow/newestKeyframe.cc
  methods/KeyframeWindow/exportLandmarks.cc
  methods/BundleAdjustmentOptimizer/initialize.cc
  methods/BundleAdjustmentOptimizer/terminate.cc
  methods/BundleAdjustmentOptimizer/optimize.cc
  methods/KeyframeLocalOptimizationNode/buildKeyframeSample.cc
  methods/KeyframeLocalOptimizationNode/handlePairedInputCallBack.cc
  methods/KeyframeLocalOptimizationNode/handlePoseOnlyCallBack.cc
  methods/KeyframeLocalOptimizationNode/runOptimization.cc
  methods/KeyframeLocalOptimizationNode/publishOdometry.cc
  methods/KeyframeLocalOptimizationNode/publishPointCloud.cc
  methods/KeyframeLocalOptimizationNode/publishFeatureImage.cc
  methods/KeyframeLocalOptimizationNode/publishDiagnosticsCallBack.cc
  private_functions/skewOfVector.cc
  private_functions/rotationVectorToQuaternion.cc
  private_functions/quaternionToRotationVector.cc
  private_functions/invertDampedSymmetric3x3.cc
  private_functions/factorizeCholesky.cc
  private_functions/solveCholesky.cc
)
target_compile_features(keyframe_local_optimization PUBLIC cxx_std_17)
target_include_directories(keyframe_local_optimization PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(keyframe_local_optimization
  common_console
  common_diagnostics
  Eigen3::Eigen
)
ament_target_dependencies(keyframe_local_optimization
  builtin_interfaces
  diagnostic_msgs
  message_filters
  nav_msgs
  rclcpp
  sensor_msgs
)
install(TARGETS keyframe_local_optimization ARCHIVE DESTINATION lib)
