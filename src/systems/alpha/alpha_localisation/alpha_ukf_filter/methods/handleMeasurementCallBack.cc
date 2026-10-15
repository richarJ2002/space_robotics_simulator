/*!
 * @File:         handleMeasurementCallBack.cc
 *
 * @Brief:        Implements age-checked, rollback-capable UKF correction.
 *
 * @Date:         07/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/AlphaUkfFilterNodeClass.h"

/* Other Project Module Includes */
#include "console/console.h"

/* C++ Standard Library Includes */
#include <cmath>

/* Generic Libraries */
/* None */

namespace systems::alpha::alpha_localisation::alpha_ukf_filter
{

void AlphaUkfFilterNode::handleMeasurementCallBack(
    const nav_msgs::msg::Odometry &message_in,
    MeasurementKind                kind_in)
{
    constexpr double TIMESTAMP_TOLERANCE_S = 1.0e-9;
    const bool       isVisual =
        kind_in == MeasurementKind::MEASUREMENT_KIND_VISUAL;
    SourceDiagnostics *p_diagnostics =
        isVisual ? &visualDiagnostics : &wheelDiagnostics;

    const double admissionTimestamp_s = now().seconds();
    p_diagnostics->receivedCount++;
    p_diagnostics->callbackAdmissionTimestamp_s = admissionTimestamp_s;
    p_diagnostics->processingStartTimestamp_s   = admissionTimestamp_s;
    const auto finishDiagnostics                = [this, p_diagnostics]()
    {
        p_diagnostics->processingEndTimestamp_s = now().seconds();
        p_diagnostics->estimatorTimestamp_s     = stateTimestamp_s;
    };

    /* Measurements arriving before the IMU-seeded initial state exists
     * are rejected. */
    if (!hasInitialState)
    {
        p_diagnostics->preInitRejectedCount++;
        p_diagnostics->ageRejectedCount++;
        finishDiagnostics();
        return;
    }

    const double measurementTimestamp_s =
        rclcpp::Time(message_in.header.stamp, RCL_ROS_TIME).seconds();
    p_diagnostics->publicationTimestamp_s = measurementTimestamp_s;
    if (!std::isfinite(measurementTimestamp_s))
    {
        p_diagnostics->numericalRejectedCount++;
        finishDiagnostics();
        return;
    }

    /* Reject a stamp leading this node's clock beyond tolerance: the
     * measurement path can deliver a message a few milliseconds before
     * the corresponding /clock update reaches this node. */
    if (measurementTimestamp_s > admissionTimestamp_s + maximumFutureStamp_s)
    {
        p_diagnostics->negativeAgeRejectedCount++;
        p_diagnostics->ageRejectedCount++;
        finishDiagnostics();
        return;
    }

    /* Drop a measurement older than its source's maximum age. */
    const double maximumAgeS =
        isVisual ? maximumVisualMeasurementAgeS : maximumWheelMeasurementAgeS;
    if (admissionTimestamp_s - measurementTimestamp_s > maximumAgeS)
    {
        p_diagnostics->tooOldRejectedCount++;
        p_diagnostics->ageRejectedCount++;
        finishDiagnostics();
        return;
    }
    p_diagnostics->acceptedCount++;

    /* Convert once; the record holds only state-independent values. */
    FusedMeasurement record;
    if (!buildMeasurementRecord(message_in,
                                kind_in,
                                measurementTimestamp_s,
                                record))
    {
        p_diagnostics->numericalRejectedCount++;
        finishDiagnostics();
        return;
    }

    /* A measurement stamped before the current epoch rolls back to its
     * stamp, fuses there, and replays newer records in order. */
    if (measurementTimestamp_s < stateTimestamp_s - TIMESTAMP_TOLERANCE_S)
    {
        const double presentTimestamp_s = stateTimestamp_s;
        if (!restoreFilterCheckpointAtOrBefore(measurementTimestamp_s))
        {
            p_diagnostics->rollbackFailedCount++;
            p_diagnostics->ageRejectedCount++;
            finishDiagnostics();
            return;
        }
        discardFilterCheckpointsAfter(measurementTimestamp_s);
        const FilterStatus predictStatus =
            predictTo(measurementTimestamp_s, true);
        if (predictStatus != FilterStatus::UKF_STATUS_SUCCESS)
        {
            p_diagnostics->predictFailedCount++;
            logStepFailure(predictStatus);
            finishDiagnostics();
            return;
        }
        double                    nis            = 0.0;
        double                    correctionNorm = 0.0;
        const MeasurementFusionResult fuseResult =
            fuseMeasurementRecord(record, nis, correctionNorm);
        p_diagnostics->normalizedInnovationSquared = nis;
        if (fuseResult !=
            MeasurementFusionResult::MEASUREMENT_FUSION_RESULT_FUSED)
        {
            if (fuseResult == MeasurementFusionResult::
                                 MEASUREMENT_FUSION_RESULT_NIS_REJECTED)
            {
                p_diagnostics->nisRejectedCount++;
            }
            else
            {
                p_diagnostics->numericalRejectedCount++;
            }
            finishDiagnostics();
            return;
        }
        p_diagnostics->fusedCount++;
        p_diagnostics->correctionNorm = correctionNorm;
        const FilterStatus replayStatus =
            replayMeasurementsAfter(measurementTimestamp_s,
                                    presentTimestamp_s);
        if (replayStatus != FilterStatus::UKF_STATUS_SUCCESS)
        {
            p_diagnostics->predictFailedCount++;
            logStepFailure(replayStatus);
            finishDiagnostics();
            return;
        }
    }
    else
    {
        const FilterStatus predictStatus =
            predictTo(measurementTimestamp_s, true);
        if (predictStatus != FilterStatus::UKF_STATUS_SUCCESS)
        {
            p_diagnostics->predictFailedCount++;
            logStepFailure(predictStatus);
            finishDiagnostics();
            return;
        }
        double                    nis            = 0.0;
        double                    correctionNorm = 0.0;
        const MeasurementFusionResult fuseResult =
            fuseMeasurementRecord(record, nis, correctionNorm);
        p_diagnostics->normalizedInnovationSquared = nis;
        if (fuseResult !=
            MeasurementFusionResult::MEASUREMENT_FUSION_RESULT_FUSED)
        {
            if (fuseResult == MeasurementFusionResult::
                                 MEASUREMENT_FUSION_RESULT_NIS_REJECTED)
            {
                p_diagnostics->nisRejectedCount++;
            }
            else
            {
                p_diagnostics->numericalRejectedCount++;
            }
            finishDiagnostics();
            return;
        }
        p_diagnostics->fusedCount++;
        p_diagnostics->correctionNorm = correctionNorm;
    }

    /* Retain the fused record in timestamp order for future replays,
     * dropping the oldest once the history is full. */
    std::size_t insertIndex = measurementHistory.size();
    for (std::size_t index = 0U; index < measurementHistory.size(); index++)
    {
        if (measurementHistory[index].timestamp_s > measurementTimestamp_s)
        {
            insertIndex = index;
            break;
        }
    }
    measurementHistory.insert(measurementHistory.begin() +
                                  static_cast<std::deque<
                                      FusedMeasurement>::difference_type>(
                                      insertIndex),
                              record);
    while (measurementHistory.size() > MEASUREMENT_HISTORY_CAPACITY)
    {
        measurementHistory.pop_front();
    }

    publishEstimate();
    finishDiagnostics();
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
