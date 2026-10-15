/*!
 * @File:         UkfStateIndexEnum.h
 *
 * @Brief:        Declares the UKF state-vector layout.
 *
 * @Date:         07/10/2026
 *
 */

/* Header Guard */
#ifndef SRS_SYSTEMS_ALPHA_ALPHA_UKF_FILTER_UKF_STATE_ENUM_H
#define SRS_SYSTEMS_ALPHA_ALPHA_UKF_FILTER_UKF_STATE_ENUM_H

/* Function Includes */
/* None */

/* Object Include */
/* None */

/* Data include */
/* None */

/* Generic Libraries */
#include <cstdint>

namespace systems::alpha::alpha_localisation::alpha_ukf_filter
{

/*!
 * @brief           Indexes the UKF's Euclidean state vector.
 *
 *                  Attitude uses roll/pitch/yaw Euler angles (ZYX order:
 *                  R = Rz(yaw) Ry(pitch) Rx(roll)) so every sigma-point
 *                  operation stays a plain weighted vector sum; the EKF's
 *                  quaternion is converted only at publication. Position
 *                  and velocity live in startup-fixed; biases in body.
 */
enum class UkfStateIndex : std::int32_t
{
    /*!
     * @brief       Startup-fixed position x, metres.
     */
    UKF_STATE_INDEX_POSITION_X = 0,

    /*!
     * @brief       Startup-fixed position y, metres.
     */
    UKF_STATE_INDEX_POSITION_Y = 1,

    /*!
     * @brief       Startup-fixed position z, metres.
     */
    UKF_STATE_INDEX_POSITION_Z = 2,

    /*!
     * @brief       Roll about body x, radians.
     */
    UKF_STATE_INDEX_ATTITUDE_ROLL = 3,

    /*!
     * @brief       Pitch about body y, radians.
     */
    UKF_STATE_INDEX_ATTITUDE_PITCH = 4,

    /*!
     * @brief       Yaw about body z, radians.
     */
    UKF_STATE_INDEX_ATTITUDE_YAW = 5,

    /*!
     * @brief       Startup-fixed velocity x, metres per second.
     */
    UKF_STATE_INDEX_VELOCITY_X = 6,

    /*!
     * @brief       Startup-fixed velocity y, metres per second.
     */
    UKF_STATE_INDEX_VELOCITY_Y = 7,

    /*!
     * @brief       Startup-fixed velocity z, metres per second.
     */
    UKF_STATE_INDEX_VELOCITY_Z = 8,

    /*!
     * @brief       Accelerometer bias x, body, metres per second squared.
     */
    UKF_STATE_INDEX_ACCELEROMETER_BIAS_X = 9,

    /*!
     * @brief       Accelerometer bias y, body, metres per second squared.
     */
    UKF_STATE_INDEX_ACCELEROMETER_BIAS_Y = 10,

    /*!
     * @brief       Accelerometer bias z, body, metres per second squared.
     */
    UKF_STATE_INDEX_ACCELEROMETER_BIAS_Z = 11,

    /*!
     * @brief       Gyroscope bias x, body, radians per second.
     */
    UKF_STATE_INDEX_GYROSCOPE_BIAS_X = 12,

    /*!
     * @brief       Gyroscope bias y, body, radians per second.
     */
    UKF_STATE_INDEX_GYROSCOPE_BIAS_Y = 13,

    /*!
     * @brief       Gyroscope bias z, body, radians per second.
     */
    UKF_STATE_INDEX_GYROSCOPE_BIAS_Z = 14,

    /*!
     * @brief       Number of scalar states.
     */
    UKF_STATE_INDEX_COUNT = 15
};

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */

#endif /* SRS_SYSTEMS_ALPHA_ALPHA_UKF_FILTER_UKF_STATE_ENUM_H \
        */
