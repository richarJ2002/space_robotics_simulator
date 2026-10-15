/*!
 * @File:         fuseMeasurementRecord.cc
 *
 * @Brief:        Implements NIS-gated unscented measurement fusion.
 *
 * @Date:         07/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/AlphaUkfFilterNodeClass.h"

/* Function Includes */
#include "private_functions/private_functions.h"

/* Object Includes */
#include "objects/UkfStateIndexEnum.h"

/* Generic Libraries */
/* None */

namespace systems::alpha::alpha_localisation::alpha_ukf_filter
{

MeasurementFusionResult AlphaUkfFilterNode::fuseMeasurementRecord(
    const FusedMeasurement &record_in,
    double                 &nis_out,
    double                 &correctionNorm_out)
{
    /* Expand the current state into sigma points once; both mappings read
     * the same cloud. */
    Eigen::MatrixXd stateSigma;
    Eigen::VectorXd weightsMean;
    Eigen::VectorXd weightsCovariance;
    FilterStatus sigmaStatus =
        filter.generateSigmaPoints(stateSigma, weightsMean, weightsCovariance);
    if (sigmaStatus != FilterStatus::UKF_STATUS_SUCCESS)
    {
        return MeasurementFusionResult::
            MEASUREMENT_FUSION_RESULT_NUMERICAL_REJECTED;
    }

    /* Map every state point through this record's observation model. */
    const bool         isVisual =
        record_in.kind == MeasurementKind::MEASUREMENT_KIND_VISUAL;
    const Eigen::Index measurementSize =
        isVisual ? VISUAL_MEASUREMENT_SIZE : WHEEL_MEASUREMENT_SIZE;
    Eigen::MatrixXd measurementSigma(measurementSize, 2 * STATE_SIZE + 1);
    for (Eigen::Index point = 0; point < 2 * STATE_SIZE + 1; point++)
    {
        const StateVector pointState = stateSigma.col(point);
        if (isVisual)
        {
            measurementSigma.col(point) = predictVisualMeasurement(pointState);
        }
        else
        {
            measurementSigma.col(point) = predictWheelMeasurement(pointState);
        }
    }

    /* Score the innovation before committing anything. */
    double       nis         = 0.0;
    const FilterStatus nisStatus = filter.calculateUpdateNis(measurementSigma,
                                                             record_in
                                                                 .measurement,
                                                             record_in.noise,
                                                             nis);
    if (nisStatus != FilterStatus::UKF_STATUS_SUCCESS)
    {
        return MeasurementFusionResult::
            MEASUREMENT_FUSION_RESULT_NUMERICAL_REJECTED;
    }
    nis_out = nis;

    /* Gate on the 99% chi-square limit for this update's active dimension
     * unless the source overrides it; a rejection leaves the filter
     * unchanged. */
    const double gateThreshold = selectNisThreshold(
        isVisual ? visualNisThreshold : wheelNisThreshold,
        measurementSize);
    if (!(nis <= gateThreshold))
    {
        return MeasurementFusionResult::
            MEASUREMENT_FUSION_RESULT_NIS_REJECTED;
    }

    /* Apply the unscented correction at the current epoch. */
    const StateVector priorState = filter.getState();
    const FilterStatus updateStatus =
        filter.updateFromSigmaPoints(measurementSigma,
                                     stateSigma,
                                     record_in.measurement,
                                     record_in.noise,
                                     nis);
    if (updateStatus != FilterStatus::UKF_STATUS_SUCCESS)
    {
        return MeasurementFusionResult::
            MEASUREMENT_FUSION_RESULT_NUMERICAL_REJECTED;
    }
    nis_out = nis;

    /* Keep attitude canonical after the correction. */
    static constexpr Eigen::Index ATTITUDE_INDEX =
        static_cast<Eigen::Index>(
            UkfStateIndex::UKF_STATE_INDEX_ATTITUDE_ROLL);
    StateVector wrappedState = filter.getState();
    wrappedState(ATTITUDE_INDEX) = wrapAngle(wrappedState(ATTITUDE_INDEX));
    wrappedState(ATTITUDE_INDEX + 1) = wrapAngle(
        wrappedState(ATTITUDE_INDEX + 1));
    wrappedState(ATTITUDE_INDEX + 2) = wrapAngle(
        wrappedState(ATTITUDE_INDEX + 2));
    const FilterStatus wrapStatus = filter.setState(wrappedState);
    if (wrapStatus != FilterStatus::UKF_STATUS_SUCCESS)
    {
        return MeasurementFusionResult::
            MEASUREMENT_FUSION_RESULT_NUMERICAL_REJECTED;
    }

    /* Report the applied correction with wrapped attitude differences. */
    StateVector correction = filter.getState() - priorState;
    correction(ATTITUDE_INDEX) = wrapAngle(correction(ATTITUDE_INDEX));
    correction(ATTITUDE_INDEX + 1) = wrapAngle(
        correction(ATTITUDE_INDEX + 1));
    correction(ATTITUDE_INDEX + 2) = wrapAngle(
        correction(ATTITUDE_INDEX + 2));
    correctionNorm_out = correction.norm();

    latestState      = filter.getState();
    latestCovariance = filter.getCovariance();

    return MeasurementFusionResult::MEASUREMENT_FUSION_RESULT_FUSED;
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
