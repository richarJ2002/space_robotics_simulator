/*!
 * @File:         generateSigmaPoints.cc
 *
 * @Brief:        Implements scaled sigma-point generation from the
 *                current state.
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

UkfStatus ContinuousUnscentedKalmanFilter::generateSigmaPoints(
    Eigen::MatrixXd &sigmaPoints_out,
    Eigen::VectorXd &weightsMean_out,
    Eigen::VectorXd &weightsCovariance_out) const noexcept
{
    /* Nothing to expand before initialization. */
    if (!isInitialized)
    {
        return UkfStatus::UKF_STATUS_NOT_INITIALIZED;
    }

    /* Factor the scaled covariance once; its columns offset the mean. */
    const double scaling = static_cast<double>(stateSize) + lambdaWeight;
    const Eigen::MatrixXd scaledCovariance =
        0.5 * scaling * (covariance + covariance.transpose());
    const Eigen::LLT<Eigen::MatrixXd> decomposition(scaledCovariance);
    if (decomposition.info() != Eigen::Success)
    {
        return UkfStatus::UKF_STATUS_NUMERICAL_FAILURE;
    }
    const Eigen::MatrixXd spread = decomposition.matrixL();
    if (!spread.allFinite())
    {
        return UkfStatus::UKF_STATUS_NUMERICAL_FAILURE;
    }

    /* Column 0 is the mean; each Cholesky column offsets one pair. */
    sigmaPoints_out.resize(stateSize, 2 * stateSize + 1);
    sigmaPoints_out.col(0) = state;
    for (Eigen::Index index = 0; index < stateSize; index++)
    {
        sigmaPoints_out.col(1 + index) = state + spread.col(index);
        sigmaPoints_out.col(1 + stateSize + index) =
            state - spread.col(index);
    }

    calculateSigmaWeights(stateSize,
                          lambdaWeight,
                          sigmaAlpha,
                          sigmaBeta,
                          weightsMean_out,
                          weightsCovariance_out);

    return UkfStatus::UKF_STATUS_SUCCESS;
}

} /* namespace localisation::kalman_filter::ukf_continuous_kalman_filter */
