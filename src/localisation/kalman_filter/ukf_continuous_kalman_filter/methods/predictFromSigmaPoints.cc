/*!
 * @File:         predictFromSigmaPoints.cc
 *
 * @Brief:        Implements one unscented prediction recombination step.
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

UkfStatus ContinuousUnscentedKalmanFilter::predictFromSigmaPoints(
    const Eigen::MatrixXd &propagatedSigmaPoints_in,
    const Eigen::MatrixXd &processNoise_in) noexcept
{
    /* Nothing to propagate before initialization. */
    if (!isInitialized)
    {
        return UkfStatus::UKF_STATUS_NOT_INITIALIZED;
    }

    /* Every caller-supplied matrix must match the configured sizes. */
    if (propagatedSigmaPoints_in.rows() != stateSize ||
        propagatedSigmaPoints_in.cols() != 2 * stateSize + 1 ||
        processNoise_in.rows() != stateSize ||
        processNoise_in.cols() != stateSize)
    {
        return UkfStatus::UKF_STATUS_INVALID_INPUT;
    }

    /* Every caller input must be finite before use. */
    if (!propagatedSigmaPoints_in.allFinite() ||
        !processNoise_in.allFinite())
    {
        return UkfStatus::UKF_STATUS_INVALID_INPUT;
    }

    /* Recombine the propagated points with the scaled weights. */
    Eigen::VectorXd weightsMean;
    Eigen::VectorXd weightsCovariance;
    calculateSigmaWeights(stateSize,
                          lambdaWeight,
                          sigmaAlpha,
                          sigmaBeta,
                          weightsMean,
                          weightsCovariance);

    /* Predicted mean is the weight-averaged propagated cloud. */
    const Eigen::VectorXd predictedState =
        propagatedSigmaPoints_in * weightsMean;

    /* Predicted covariance is the weighted spread plus process noise,
     * computed into a temporary so a rejected step cannot corrupt the
     * last valid posterior. */
    Eigen::MatrixXd predictedCovariance = processNoise_in;
    for (Eigen::Index index = 0; index < 2 * stateSize + 1; index++)
    {
        const Eigen::VectorXd deviation =
            propagatedSigmaPoints_in.col(index) - predictedState;
        predictedCovariance += weightsCovariance(index) * deviation *
                               deviation.transpose();
    }

    /* Re-symmetrize to cancel the asymmetry floating-point arithmetic
     * accumulates over repeated predictions. */
    predictedCovariance =
        0.5 * (predictedCovariance + predictedCovariance.transpose());

    if (!predictedState.allFinite() ||
        !isCovarianceValid(predictedCovariance))
    {
        return UkfStatus::UKF_STATUS_NUMERICAL_FAILURE;
    }

    /* Commit the validated prediction to the retained state. */
    state      = predictedState;
    covariance = predictedCovariance;

    return UkfStatus::UKF_STATUS_SUCCESS;
}

} /* namespace localisation::kalman_filter::ukf_continuous_kalman_filter */
