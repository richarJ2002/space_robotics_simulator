/*!
 * @File:         updateFromSigmaPoints.cc
 *
 * @Brief:        Implements one unscented measurement correction step.
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

UkfStatus ContinuousUnscentedKalmanFilter::updateFromSigmaPoints(
    const Eigen::MatrixXd &predictedMeasurementSigma_in,
    const Eigen::MatrixXd &stateSigmaPoints_in,
    const Eigen::VectorXd &measurement_in,
    const Eigen::MatrixXd &measurementNoise_in,
    double                &nis_out) noexcept
{
    /* Nothing to correct before initialization. */
    if (!isInitialized)
    {
        return UkfStatus::UKF_STATUS_NOT_INITIALIZED;
    }

    /* Measurement dimension is whatever the caller's measurement
     * carries; an empty correction would silently apply a zero-effect
     * update. */
    const Eigen::Index measurementSize = measurement_in.size();
    if (measurementSize <= 0)
    {
        return UkfStatus::UKF_STATUS_INVALID_INPUT;
    }

    /* Every caller-supplied matrix must match the configured sizes. */
    if (predictedMeasurementSigma_in.rows() != measurementSize ||
        predictedMeasurementSigma_in.cols() != 2 * stateSize + 1 ||
        stateSigmaPoints_in.rows() != stateSize ||
        stateSigmaPoints_in.cols() != 2 * stateSize + 1 ||
        measurementNoise_in.rows() != measurementSize ||
        measurementNoise_in.cols() != measurementSize)
    {
        return UkfStatus::UKF_STATUS_INVALID_INPUT;
    }

    if (!predictedMeasurementSigma_in.allFinite() ||
        !stateSigmaPoints_in.allFinite() || !measurement_in.allFinite() ||
        !measurementNoise_in.allFinite())
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

    /* Factor P_yy so the gain can be solved without an explicit
     * inverse. */
    const Eigen::LDLT<Eigen::MatrixXd> decomposition(innovationCovariance);
    if (decomposition.info() != Eigen::Success ||
        !decomposition.isPositive())
    {
        return UkfStatus::UKF_STATUS_NUMERICAL_FAILURE;
    }

    /* Cross-covariance P_xy between the state and measurement clouds. */
    const Eigen::VectorXd predictedState =
        stateSigmaPoints_in * weightsMean;
    Eigen::MatrixXd crossCovariance =
        Eigen::MatrixXd::Zero(stateSize, measurementSize);
    for (Eigen::Index index = 0; index < 2 * stateSize + 1; index++)
    {
        crossCovariance += weightsCovariance(index) *
                           (stateSigmaPoints_in.col(index) - predictedState) *
                           (predictedMeasurementSigma_in.col(index) -
                            predictedMeasurement)
                               .transpose();
    }

    /* Kalman gain K = P_xy P_yy^-1, via the LDLT solve above. */
    const Eigen::MatrixXd gain =
        crossCovariance *
        decomposition.solve(
            Eigen::MatrixXd::Identity(measurementSize, measurementSize));

    /* Score the innovation before committing anything. */
    const Eigen::VectorXd innovation = measurement_in - predictedMeasurement;
    const double          nis =
        innovation.dot(decomposition.solve(innovation));

    /* Compute into temporaries so a rejected numerical update cannot
     * corrupt the last valid posterior retained by the engine. */
    const Eigen::VectorXd correctedState = state + gain * innovation;
    Eigen::MatrixXd       correctedCovariance =
        covariance - gain * innovationCovariance * gain.transpose();

    /* Re-symmetrize to cancel the asymmetry floating-point arithmetic
     * accumulates over repeated updates. */
    correctedCovariance =
        0.5 * (correctedCovariance + correctedCovariance.transpose());

    if (!std::isfinite(nis) || !correctedState.allFinite() ||
        !isCovarianceValid(correctedCovariance))
    {
        return UkfStatus::UKF_STATUS_NUMERICAL_FAILURE;
    }

    state      = correctedState;
    covariance = correctedCovariance;
    nis_out    = nis;

    return UkfStatus::UKF_STATUS_SUCCESS;
}

} /* namespace localisation::kalman_filter::ukf_continuous_kalman_filter */
