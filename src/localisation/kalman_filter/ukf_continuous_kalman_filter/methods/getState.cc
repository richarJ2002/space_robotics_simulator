/*!
 * @File:         getState.cc
 *
 * @Brief:        Implements the current-state accessor.
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

const Eigen::VectorXd &
    ContinuousUnscentedKalmanFilter::getState() const noexcept
{
    return state;
}

} /* namespace localisation::kalman_filter::ukf_continuous_kalman_filter */
