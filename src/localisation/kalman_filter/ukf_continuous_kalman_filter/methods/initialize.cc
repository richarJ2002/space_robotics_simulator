/*!
 * @File:         initialize.cc
 *
 * @Brief:        Implements sizing, seeding and sigma-point scaling of
 *                the filter.
 *
 * @Date:         07/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/ContinuousUnscentedKalmanFilterClass.h"

/* C++ Standard Library Includes */
#include <cmath>

/* Generic Libraries */
/* None */

namespace localisation::kalman_filter::ukf_continuous_kalman_filter
{

UkfStatus ContinuousUnscentedKalmanFilter::initialize(
    Eigen::Index            stateSize_in,
    const Eigen::VectorXd  &initialState_in,
    const Eigen::MatrixXd  &initialCovariance_in,
    const UkfConfiguration &configuration_in) noexcept
{
    /* A non-positive state size has no meaningful filter to construct. */
    if (stateSize_in <= 0)
    {
        return UkfStatus::UKF_STATUS_INVALID_CONFIGURATION;
    }

    /* Every dimension below must exactly match the requested state size. */
    if (initialState_in.size() != stateSize_in ||
        initialCovariance_in.rows() != stateSize_in ||
        initialCovariance_in.cols() != stateSize_in)
    {
        return UkfStatus::UKF_STATUS_INVALID_CONFIGURATION;
    }

    /* Reject a configuration containing NaN/Inf before it can corrupt
     * the filter's state. */
    if (!initialState_in.allFinite() || !initialCovariance_in.allFinite())
    {
        return UkfStatus::UKF_STATUS_INVALID_CONFIGURATION;
    }

    /* Every diagonal covariance entry must be strictly positive. */
    for (Eigen::Index index = 0; index < stateSize_in; index++)
    {
        if (initialCovariance_in(index, index) <= 0.0)
        {
            return UkfStatus::UKF_STATUS_INVALID_CONFIGURATION;
        }
    }

    /* The spread must be finite and positive, and the composite scaling
     * must keep a positive sigma-point weight denominator. */
    if (!std::isfinite(configuration_in.sigmaAlpha) ||
        !std::isfinite(configuration_in.sigmaBeta) ||
        !std::isfinite(configuration_in.sigmaKappa) ||
        !(configuration_in.sigmaAlpha > 0.0))
    {
        return UkfStatus::UKF_STATUS_INVALID_CONFIGURATION;
    }
    const double lambda =
        configuration_in.sigmaAlpha * configuration_in.sigmaAlpha *
            (static_cast<double>(stateSize_in) +
             configuration_in.sigmaKappa) -
        static_cast<double>(stateSize_in);
    if (!std::isfinite(lambda) ||
        !(static_cast<double>(stateSize_in) + lambda > 0.0))
    {
        return UkfStatus::UKF_STATUS_INVALID_CONFIGURATION;
    }

    /* Symmetrize the caller-supplied covariance, guarding against a
     * caller that passed a not-quite-symmetric matrix. */
    const Eigen::MatrixXd symmetricCovariance =
        0.5 * (initialCovariance_in + initialCovariance_in.transpose());

    if (!isCovarianceValid(symmetricCovariance))
    {
        return UkfStatus::UKF_STATUS_INVALID_CONFIGURATION;
    }

    /* Record the configured size, allocating this filter's dynamic
     * matrices at exactly that size. */
    stateSize   = stateSize_in;
    state       = initialState_in;
    covariance  = symmetricCovariance;
    sigmaAlpha  = configuration_in.sigmaAlpha;
    sigmaBeta   = configuration_in.sigmaBeta;
    sigmaKappa  = configuration_in.sigmaKappa;
    lambdaWeight = lambda;

    /* Only now that every field above is valid does the filter become
     * usable by its predict/update/accessor calls. */
    isInitialized = true;

    return UkfStatus::UKF_STATUS_SUCCESS;
}

} /* namespace localisation::kalman_filter::ukf_continuous_kalman_filter */
