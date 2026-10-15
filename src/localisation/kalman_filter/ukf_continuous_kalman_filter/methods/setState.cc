/*!
 * @File:         setState.cc
 *
 * @Brief:        Implements validated state overwrite.
 *
 * @Date:         07/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/ContinuousUnscentedKalmanFilterClass.h"

/* Generic Libraries */
/* None */

namespace localisation::kalman_filter::ukf_continuous_kalman_filter
{

UkfStatus ContinuousUnscentedKalmanFilter::setState(
    const Eigen::VectorXd &state_in) noexcept
{
    /* Nothing to overwrite before initialization. */
    if (!isInitialized)
    {
        return UkfStatus::UKF_STATUS_NOT_INITIALIZED;
    }

    /* Replacement state must be sized and finite. */
    if (state_in.size() != stateSize || !state_in.allFinite())
    {
        return UkfStatus::UKF_STATUS_INVALID_INPUT;
    }

    state = state_in;

    return UkfStatus::UKF_STATUS_SUCCESS;
}

} /* namespace localisation::kalman_filter::ukf_continuous_kalman_filter */
