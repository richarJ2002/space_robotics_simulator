cmake_minimum_required(VERSION 3.16)

add_library(inertial_averaging STATIC
  methods/initialize.cc
  methods/push.cc
  methods/sampleCount.cc
  private_functions/isSampleFinite.cc
)
target_compile_features(inertial_averaging PUBLIC cxx_std_17)
target_include_directories(inertial_averaging PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
install(TARGETS inertial_averaging ARCHIVE DESTINATION lib)
