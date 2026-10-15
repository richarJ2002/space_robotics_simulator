/*!
 * @File:         terminate.cc
 *
 * @Brief:        Implements return to the uninitialized lifecycle.
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

UkfStatus ContinuousUnscentedKalmanFilter::terminate() noexcept
{
    /* Clearing an already-uninitialized filter is still a success. */
    state.resize(0);
    covariance.resize(0, 0);
    stateSize     = 0;
    isInitialized = false;

    return UkfStatus::UKF_STATUS_SUCCESS;
}

} /* namespace localisation::kalman_filter::ukf_continuous_kalman_filter */
