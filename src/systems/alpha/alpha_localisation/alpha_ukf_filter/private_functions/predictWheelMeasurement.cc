/*!
 * @file            predictWheelMeasurement.cc
 *
 * @brief           Implements wheel-velocity sigma-point mapping.
 *
 * @date            07/10/2026
 */

/* Matching Declaration Include */
#include "private_functions/private_functions.h"

/* Object Includes */
#include "objects/UkfStateIndexEnum.h"

namespace systems::alpha::alpha_localisation::alpha_ukf_filter
{

Eigen::Vector3d predictWheelMeasurement(
    const AlphaUkfFilterNode::StateVector &state_in)
{
    const Eigen::Matrix3d rotation_bodyToFixed =
        AlphaUkfFilterNode::rotationFromRpy(
            state_in(static_cast<Eigen::Index>(
                UkfStateIndex::UKF_STATE_INDEX_ATTITUDE_ROLL)),
            state_in(static_cast<Eigen::Index>(
                UkfStateIndex::UKF_STATE_INDEX_ATTITUDE_PITCH)),
            state_in(static_cast<Eigen::Index>(
                UkfStateIndex::UKF_STATE_INDEX_ATTITUDE_YAW)));
    return rotation_bodyToFixed.transpose() *
           state_in.segment<3>(static_cast<Eigen::Index>(
               UkfStateIndex::UKF_STATE_INDEX_VELOCITY_X));
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
