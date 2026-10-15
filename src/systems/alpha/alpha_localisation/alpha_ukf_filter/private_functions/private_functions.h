/*!
 * @file            private_functions.h
 *
 * @brief           Declares the Alpha UKF filter module's internal
 *                  helpers.
 *
 * @date            07/10/2026
 */

#ifndef SRS_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_UKF_FILTER_PRIVATE_H
#define SRS_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_UKF_FILTER_PRIVATE_H

/* C++ Standard Library Includes */
#include <cstdint>
#include <string>

/* External Library Includes */
#include <Eigen/Dense>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

/* Object Includes */
#include "objects/AlphaUkfFilterNodeClass.h"
#include "objects/UkfSourceDiagnosticsStruct.h"

namespace systems::alpha::alpha_localisation::alpha_ukf_filter
{

/*!
 * @brief       Maps one state sigma point to the visual pose observation.
 *
 *              Startup-fixed position plus roll/pitch/yaw, both already
 *              Euclidean in the UKF state.
 *
 * @param[in]   state_in
 *              Sigma-point state with startup-fixed position in metres
 *              and roll/pitch/yaw in radians.
 *
 * @return      Six-element pose prediction in metres and radians.
 */
Eigen::Matrix<double, 6, 1> predictVisualMeasurement(
    const AlphaUkfFilterNode::StateVector &state_in);

/*!
 * @brief       Maps one state sigma point to the wheel velocity observation.
 *
 *              Startup-fixed velocity rotated into the body frame.
 *
 * @param[in]   state_in
 *              Sigma-point state with roll/pitch/yaw in radians and
 *              startup-fixed velocity in metres per second.
 *
 * @return      Body-frame velocity prediction in metres per second.
 */
Eigen::Vector3d predictWheelMeasurement(
    const AlphaUkfFilterNode::StateVector &state_in);

/*!
 * @brief       Builds one per-source status.
 *
 *              Keys match the EKF source-status contract parsed by
 *              post_processing/python_tools/diagnostics/bag_diagnostics.py.
 *
 * @param[in]   sourceName_in
 *              Source name appended to the status name.
 *
 * @param[in]   source_in
 *              Source counters and latest timing values to report.
 *
 * @return      Diagnostic status with the per-source keys and values.
 *
 * @note        UKF statuses are a recorded documented-ignore there until
 *              a parser extension lands; comparison-only, never gates
 *              READY.
 */
diagnostic_msgs::msg::DiagnosticStatus
    makeSourceStatus(const std::string       &sourceName_in,
                     const SourceDiagnostics &source_in);

/*!
 * @brief       Counts one source's rejections for the console line.
 *
 * @param[in]   source_in
 *              Source counters to total.
 *
 * @return      Sum of age, NIS, and numerical rejection counts.
 */
std::uint64_t countRejections(const SourceDiagnostics &source_in);

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */

#endif /* SRS_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_UKF_FILTER_PRIVATE_H */
