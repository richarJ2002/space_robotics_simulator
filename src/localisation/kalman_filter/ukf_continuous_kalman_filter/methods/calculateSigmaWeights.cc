/*!
 * @File:         calculateSigmaWeights.cc
 *
 * @Brief:        Implements scaled unscented-transform weight evaluation.
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

void ContinuousUnscentedKalmanFilter::calculateSigmaWeights(
    Eigen::Index     stateSize_in,
    double           lambda_in,
    double           sigmaAlpha_in,
    double           sigmaBeta_in,
    Eigen::VectorXd &weightsMean_out,
    Eigen::VectorXd &weightsCovariance_out) noexcept
{
    /* Composite denominator shared by every weight below. */
    const double denominator =
        static_cast<double>(stateSize_in) + lambda_in;

    /* Size both weight vectors for the 2n+1 sigma points. */
    weightsMean_out.resize(2 * stateSize_in + 1);
    weightsCovariance_out.resize(2 * stateSize_in + 1);

    /* Central point carries the lambda-weighted share, plus the
     * prior-knowledge correction on the covariance weight. */
    weightsMean_out(0) = lambda_in / denominator;
    weightsCovariance_out(0) = lambda_in / denominator +
                               (1.0 - sigmaAlpha_in * sigmaAlpha_in +
                                sigmaBeta_in);

    /* Every off-central point shares the remaining weight equally. */
    const double offCentralWeight = 1.0 / (2.0 * denominator);
    for (Eigen::Index index = 1; index < 2 * stateSize_in + 1; index++)
    {
        weightsMean_out(index)       = offCentralWeight;
        weightsCovariance_out(index) = offCentralWeight;
    }
}

} /* namespace localisation::kalman_filter::ukf_continuous_kalman_filter */
