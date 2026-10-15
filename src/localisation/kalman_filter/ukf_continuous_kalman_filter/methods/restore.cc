/*!
 * @File:         restore.cc
 *
 * @Brief:        Implements validated estimator checkpoint restoration.
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

UkfStatus ContinuousUnscentedKalmanFilter::restore(
    const Eigen::VectorXd &state_in,
    const Eigen::MatrixXd &covariance_in) noexcept
{
    /* Nothing to restore before initialization. */
    if (!isInitialized)
    {
        return UkfStatus::UKF_STATUS_NOT_INITIALIZED;
    }

    /* State and covariance must be sized and the state finite. */
    if (state_in.size() != stateSize || covariance_in.rows() != stateSize ||
        covariance_in.cols() != stateSize || !state_in.allFinite())
    {
        return UkfStatus::UKF_STATUS_INVALID_INPUT;
    }

    /* Symmetrize the candidate before validating it. */
    const Eigen::MatrixXd symmetricCovariance =
        0.5 * (covariance_in + covariance_in.transpose());

    /* Refuse a checkpoint whose covariance is not valid. */
    if (!isCovarianceValid(symmetricCovariance))
    {
        return UkfStatus::UKF_STATUS_INVALID_INPUT;
    }

    /* Commit the validated checkpoint to the retained state. */
    state      = state_in;
    covariance = symmetricCovariance;

    return UkfStatus::UKF_STATUS_SUCCESS;
}

} /* namespace localisation::kalman_filter::ukf_continuous_kalman_filter */
