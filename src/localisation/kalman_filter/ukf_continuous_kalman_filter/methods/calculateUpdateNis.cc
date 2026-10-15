/*!
 * @File:         calculateUpdateNis.cc
 *
 * @Brief:        Implements non-mutating normalized innovation squared
 *                evaluation for gating.
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

UkfStatus ContinuousUnscentedKalmanFilter::calculateUpdateNis(
    const Eigen::MatrixXd &predictedMeasurementSigma_in,
    const Eigen::VectorXd &measurement_in,
    const Eigen::MatrixXd &measurementNoise_in,
    double                &nis_out) const noexcept
{
    /* Nothing to gate before initialization. */
    if (!isInitialized)
    {
        return UkfStatus::UKF_STATUS_NOT_INITIALIZED;
    }

    /* Measurement dimension is whatever the caller's measurement
     * carries; an empty measurement has no innovation to score. */
    const Eigen::Index measurementSize = measurement_in.size();
    if (measurementSize <= 0)
    {
        return UkfStatus::UKF_STATUS_INVALID_INPUT;
    }

    /* Every caller-supplied matrix must match the configured sizes. */
    if (predictedMeasurementSigma_in.rows() != measurementSize ||
        predictedMeasurementSigma_in.cols() != 2 * stateSize + 1 ||
        measurementNoise_in.rows() != measurementSize ||
        measurementNoise_in.cols() != measurementSize)
    {
        return UkfStatus::UKF_STATUS_INVALID_INPUT;
    }

    if (!predictedMeasurementSigma_in.allFinite() ||
        !measurement_in.allFinite() || !measurementNoise_in.allFinite())
    {
        return UkfStatus::UKF_STATUS_INVALID_INPUT;
    }

    /* Predicted measurement mean and innovation covariance P_yy: the
     * weighted spread of the measurement cloud plus noise. */
    Eigen::VectorXd weightsMean;
    Eigen::VectorXd weightsCovariance;
    calculateSigmaWeights(stateSize,
                          lambdaWeight,
                          sigmaAlpha,
                          sigmaBeta,
                          weightsMean,
                          weightsCovariance);
    const Eigen::VectorXd predictedMeasurement =
        predictedMeasurementSigma_in * weightsMean;
    Eigen::MatrixXd innovationCovariance = measurementNoise_in;
    for (Eigen::Index index = 0; index < 2 * stateSize + 1; index++)
    {
        const Eigen::VectorXd deviation =
            predictedMeasurementSigma_in.col(index) - predictedMeasurement;
        innovationCovariance += weightsCovariance(index) * deviation *
                                deviation.transpose();
    }
    innovationCovariance =
        0.5 * (innovationCovariance + innovationCovariance.transpose());

    /* Factor P_yy so the innovation can be scored without an explicit
     * inverse. */
    const Eigen::LDLT<Eigen::MatrixXd> decomposition(innovationCovariance);
    if (decomposition.info() != Eigen::Success ||
        !decomposition.isPositive())
    {
        return UkfStatus::UKF_STATUS_NUMERICAL_FAILURE;
    }

    /* Score the innovation against the factored covariance. */
    const Eigen::VectorXd innovation = measurement_in - predictedMeasurement;
    const double          nis =
        innovation.dot(decomposition.solve(innovation));
    if (!std::isfinite(nis))
    {
        return UkfStatus::UKF_STATUS_NUMERICAL_FAILURE;
    }

    nis_out = nis;
    return UkfStatus::UKF_STATUS_SUCCESS;
}

} /* namespace localisation::kalman_filter::ukf_continuous_kalman_filter */
