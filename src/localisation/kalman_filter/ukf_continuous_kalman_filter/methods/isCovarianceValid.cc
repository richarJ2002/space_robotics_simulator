/*!
 * @File:         isCovarianceValid.cc
 *
 * @Brief:        Implements covariance validity checks.
 *
 * @Date:         07/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/ContinuousUnscentedKalmanFilterClass.h"

/* External Library Includes */
#include <Eigen/Eigenvalues>

/* Generic Libraries */
/* None */

namespace localisation::kalman_filter::ukf_continuous_kalman_filter
{

bool ContinuousUnscentedKalmanFilter::isCovarianceValid(
    const Eigen::MatrixXd &covariance_in) noexcept
{
    /* A covariance is square, non-empty and finite. */
    if (covariance_in.rows() <= 0 ||
        covariance_in.rows() != covariance_in.cols() ||
        !covariance_in.allFinite())
    {
        return false;
    }

    /* Tight symmetry with a small negative-eigenvalue slack. */
    constexpr double SYMMETRY_TOLERANCE = 1.0e-12;
    constexpr double MINIMUM_EIGENVALUE = -1.0e-10;

    /* Reject a matrix that is not symmetric up to tolerance. */
    if (!covariance_in.isApprox(covariance_in.transpose(), SYMMETRY_TOLERANCE))
    {
        return false;
    }

    /* Eigendecompose the symmetric candidate once. */
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> decomposition(
        covariance_in,
        Eigen::EigenvaluesOnly);

    return decomposition.info() == Eigen::Success &&
           decomposition.eigenvalues().minCoeff() >= MINIMUM_EIGENVALUE;
}

} /* namespace localisation::kalman_filter::ukf_continuous_kalman_filter */
