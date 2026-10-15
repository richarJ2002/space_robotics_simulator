/*!
 * @File:         getCovariance.cc
 *
 * @Brief:        Implements the current-covariance accessor.
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

const Eigen::MatrixXd &
    ContinuousUnscentedKalmanFilter::getCovariance() const noexcept
{
    return covariance;
}

} /* namespace localisation::kalman_filter::ukf_continuous_kalman_filter */
