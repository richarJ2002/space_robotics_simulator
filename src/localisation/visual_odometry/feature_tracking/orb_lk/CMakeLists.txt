cmake_minimum_required(VERSION 3.16)

# OpenCV ORB corner detector front-end. Shares the GFTT engine's
# Point2D/Status/Limits vocabulary so the ROS wrapper can select either
# detector without changing its downstream pipeline. Excluded from the
# JSF AV applicability profile (see
# docs/compliance/feature_tracking/JSF_AV_APPLICABILITY_PROFILE.md):
# OpenCV-backed, general robotics profile only, no -Werror.
add_library(alpha_orb_feature_tracking STATIC
  methods/OrbCornerDetector/initialize.cc
  methods/OrbCornerDetector/detect.cc
  methods/OrbCornerDetector/terminate.cc
)

target_compile_features(alpha_orb_feature_tracking PUBLIC cxx_std_17)
target_include_directories(alpha_orb_feature_tracking PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../..
)
target_include_directories(alpha_orb_feature_tracking SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(alpha_orb_feature_tracking
  alpha_feature_tracking
  ${OpenCV_LIBRARIES}
)
