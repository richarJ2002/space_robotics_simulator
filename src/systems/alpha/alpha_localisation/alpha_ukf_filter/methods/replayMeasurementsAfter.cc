/*!
 * @File:         replayMeasurementsAfter.cc
 *
 * @Brief:        Implements measurement replay after a rollback.
 *
 * @Date:         07/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/AlphaUkfFilterNodeClass.h"

/* Generic Libraries */
/* None */

namespace systems::alpha::alpha_localisation::alpha_ukf_filter
{

AlphaUkfFilterNode::FilterStatus
    AlphaUkfFilterNode::replayMeasurementsAfter(
        double rollbackTimestamp_s_in,
        double presentTimestamp_s_in)
{
    /* Retain the present state so a failed replay can restore it. */
    const StateVector     presentState      = filter.getState();
    const StateCovariance presentCovariance = filter.getCovariance();

    for (const FusedMeasurement &record : measurementHistory)
    {
        if (record.timestamp_s <= rollbackTimestamp_s_in ||
            record.timestamp_s > presentTimestamp_s_in)
        {
            continue;
        }
        const FilterStatus predictStatus =
            predictTo(record.timestamp_s, true);
        if (predictStatus != FilterStatus::UKF_STATUS_SUCCESS)
        {
            const FilterStatus restoreStatus =
                filter.restore(presentState, presentCovariance);
            if (restoreStatus == FilterStatus::UKF_STATUS_SUCCESS)
            {
                stateTimestamp_s = presentTimestamp_s_in;
                latestState      = presentState;
                latestCovariance = presentCovariance;
            }
            return predictStatus;
        }
        double                  nis            = 0.0;
        double                  correctionNorm = 0.0;
        const MeasurementFusionResult fuseResult =
            fuseMeasurementRecord(record, nis, correctionNorm);
        if (fuseResult ==
            MeasurementFusionResult::MEASUREMENT_FUSION_RESULT_FUSED)
        {
            if (record.kind == MeasurementKind::MEASUREMENT_KIND_VISUAL)
            {
                visualDiagnostics.replayedCount++;
            }
            else
            {
                wheelDiagnostics.replayedCount++;
            }
        }
        else if (fuseResult == MeasurementFusionResult::
                                   MEASUREMENT_FUSION_RESULT_NIS_REJECTED)
        {
            /* A record the gate now rejects is skipped but kept for later
             * replays. */
            if (record.kind == MeasurementKind::MEASUREMENT_KIND_VISUAL)
            {
                visualDiagnostics.replayRejectedCount++;
            }
            else
            {
                wheelDiagnostics.replayRejectedCount++;
            }
        }
        else
        {
            const FilterStatus restoreStatus =
                filter.restore(presentState, presentCovariance);
            if (restoreStatus == FilterStatus::UKF_STATUS_SUCCESS)
            {
                stateTimestamp_s = presentTimestamp_s_in;
                latestState      = presentState;
                latestCovariance = presentCovariance;
            }
            return FilterStatus::UKF_STATUS_NUMERICAL_FAILURE;
        }
    }

    return predictTo(presentTimestamp_s_in, true);
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
