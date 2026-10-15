/*!
 * @file            predictVisualMeasurement.cc
 *
 * @brief           Implements visual-pose sigma-point mapping.
 *
 * @date            07/10/2026
 */

/* Matching Declaration Include */
#include "private_functions/private_functions.h"

/* Object Includes */
#include "objects/UkfStateIndexEnum.h"

namespace systems::alpha::alpha_localisation::alpha_ukf_filter
{

Eigen::Matrix<double, 6, 1> predictVisualMeasurement(
    const AlphaUkfFilterNode::StateVector &state_in)
{
    Eigen::Matrix<double, 6, 1> predicted;
    predicted(0) = state_in(static_cast<Eigen::Index>(
        UkfStateIndex::UKF_STATE_INDEX_POSITION_X));
    predicted(1) = state_in(static_cast<Eigen::Index>(
        UkfStateIndex::UKF_STATE_INDEX_POSITION_Y));
    predicted(2) = state_in(static_cast<Eigen::Index>(
        UkfStateIndex::UKF_STATE_INDEX_POSITION_Z));
    predicted(3) = state_in(static_cast<Eigen::Index>(
        UkfStateIndex::UKF_STATE_INDEX_ATTITUDE_ROLL));
    predicted(4) = state_in(static_cast<Eigen::Index>(
        UkfStateIndex::UKF_STATE_INDEX_ATTITUDE_PITCH));
    predicted(5) = state_in(static_cast<Eigen::Index>(
        UkfStateIndex::UKF_STATE_INDEX_ATTITUDE_YAW));
    return predicted;
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
