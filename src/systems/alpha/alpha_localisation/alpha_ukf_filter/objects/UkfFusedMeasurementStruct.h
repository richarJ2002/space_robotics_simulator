/*!
 * @File:         FusedMeasurementStruct.h
 *
 * @Brief:        Declares one replayable odometry measurement for the UKF.
 *
 * @Date:         07/10/2026
 *
 */

/* Header Guard */
#ifndef SRS_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_UKF_FILTER_FUSED_MEAS_H
#define SRS_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_UKF_FILTER_FUSED_MEAS_H

/* Function Includes */
/* None */

/* Object Include */
#include "objects/UkfMeasurementKindEnum.h"

/* Data include */
#include <Eigen/Dense>

/* Generic Libraries */
/* None */

namespace systems::alpha::alpha_localisation::alpha_ukf_filter
{

/*!
 * @brief           Stores one admitted odometry measurement in
 *                  state-independent form so it can be re-fused after a
 *                  rollback: the measured quantities and their noise only.
 *                  The observation mapping is re-evaluated against the
 *                  rolled-back state at replay time.
 */
struct FusedMeasurement
{
    /*!
     * @brief           Source that produced this measurement.
     *
     * @frame           N/A
     * @units           N/A
     */
    MeasurementKind kind{MeasurementKind::MEASUREMENT_KIND_VISUAL};

    /*!
     * @brief           Header stamp of the source message.
     *
     * @frame           N/A
     * @units           ROS seconds
     */
    double timestamp_s{0.0};

    /*!
     * @brief           Measured quantities: visual pose is
     *                  [position xyz startup-fixed, roll/pitch/yaw]; wheel
     *                  is body velocity xyz.
     *
     * @frame           Mixed; see kind
     * @units           Mixed; see kind
     */
    Eigen::VectorXd measurement{Eigen::VectorXd()};

    /*!
     * @brief           Measurement-noise covariance, square in the
     *                  measurement dimension.
     *
     * @frame           Mixed; see kind
     * @units           Squared measurement units
     */
    Eigen::MatrixXd noise{Eigen::MatrixXd()};
};

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */

#endif /* SRS_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_UKF_FILTER_FUSED_MEAS_H */
