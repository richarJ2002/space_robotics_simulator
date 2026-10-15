/*!
 * @File:         ImuSampleStruct.h
 *
 * @Brief:        Declares one raw IMU sample used by Alpha's UKF.
 *
 * @Date:         07/10/2026
 *
 */

/* Header Guard */
#ifndef SRS_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_UKF_FILTER_IMU_SAMPLE_H
#define SRS_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_UKF_FILTER_IMU_SAMPLE_H

/* Function Includes */
/* None */

/* Object Include */
/* None */

/* Data include */
#include <Eigen/Dense>

/* Generic Libraries */
/* None */

namespace systems::alpha::alpha_localisation::alpha_ukf_filter
{

/*!
 * @brief           Stores one raw IMU sample as the UKF propagates it.
 */
struct ImuSample
{
    /*!
     * @brief           ROS timestamp carried by the source IMU message.
     *
     * @frame           N/A
     * @units           seconds
     */
    double timestamp_s{0.0};

    /*!
     * @brief           Specific force applied over the interval ending at
     *                  timestamp_s.
     *
     * @frame           body
     * @units           metres per second squared
     */
    Eigen::Vector3d linearAcceleration_body_mPs2{Eigen::Vector3d::Zero()};

    /*!
     * @brief           Angular velocity reported by the IMU.
     *
     * @frame           body
     * @units           radians per second
     */
    Eigen::Vector3d angularVelocity_body_radPs{Eigen::Vector3d::Zero()};
};

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */

#endif /* SRS_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_UKF_FILTER_IMU_SAMPLE_H */
