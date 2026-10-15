cmake_minimum_required(VERSION 3.16)

# Dispatcher for Alpha's localisation-consuming packages, mirroring
# src/localisation/kalman_filter/'s dispatcher pattern -- alpha_kalman_filter
# (which owns both the EKF fusion and the estimated path/TF) beside its
# comparison-only UKF sibling alpha_ukf_filter (odometry plus retained
# path, never TF, never gates READY).
add_subdirectory(alpha_kalman_filter)
add_subdirectory(alpha_ukf_filter)
