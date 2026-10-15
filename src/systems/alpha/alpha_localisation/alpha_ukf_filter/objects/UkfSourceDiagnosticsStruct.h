/*!
 * @File:         SourceDiagnosticsStruct.h
 *
 * @Brief:        Declares bounded estimator source diagnostics.
 *
 * @Date:         07/10/2026
 *
 */

/* Header Guard */
#ifndef SRS_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_UKF_FILTER_SOURCE_DIAG_H
#define SRS_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_UKF_FILTER_SOURCE_DIAG_H

/* Function Includes */
/* None */

/* Object Include */
/* None */

/* Data include */
/* None */

/* Generic Libraries */
#include <cstdint>

namespace systems::alpha::alpha_localisation::alpha_ukf_filter
{

/*!
 * @brief           Accumulates one source's counts and latest timing values.
 *
 *                  The published key set matches the EKF's source status
 *                  exactly so a future bag_diagnostics.py extension can
 *                  parse both; until then the UKF statuses are a recorded
 *                  documented-ignore there.
 */
struct SourceDiagnostics
{
    /*!
     * @brief           Number of callbacks admitted.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t receivedCount{0U};

    /*!
     * @brief           Number of measurements passing boundary and age
     *                  validation.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t acceptedCount{0U};

    /*!
     * @brief           Number of measurements rejected for age or timestamp
     *                  order: always the sum of the six reason-specific
     *                  counters below. The IMU source instead counts
     *                  backwards clock jumps here.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t ageRejectedCount{0U};

    /*!
     * @brief           Number of measurements rejected before the
     *                  IMU-seeded initial state existed.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t preInitRejectedCount{0U};

    /*!
     * @brief           Number of measurements rejected because their stamp
     *                  led this node's ROS clock by more than
     *                  maximum_future_stamp_s.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t negativeAgeRejectedCount{0U};

    /*!
     * @brief           Number of measurements rejected because their stamp
     *                  was older than the maximum measurement age.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t tooOldRejectedCount{0U};

    /*!
     * @brief           Number of measurements rejected because their stamp
     *                  was too far from the estimator epoch.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t stateGapRejectedCount{0U};

    /*!
     * @brief           Number of measurements rejected because no rollback
     *                  checkpoint could be restored and replayed to their
     *                  stamp.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t rollbackFailedCount{0U};

    /*!
     * @brief           Number of measurements rejected because forward
     *                  prediction to their stamp failed.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t predictFailedCount{0U};

    /*!
     * @brief           Number of measurements rejected by the NIS gate.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t nisRejectedCount{0U};

    /*!
     * @brief           Number of malformed or numerically rejected
     *                  measurements.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t numericalRejectedCount{0U};

    /*!
     * @brief           Number of measurements successfully applied to the
     *                  estimate.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t fusedCount{0U};

    /*!
     * @brief           Number of already-fused measurements re-applied after
     *                  an older measurement caused a rollback.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t replayedCount{0U};

    /*!
     * @brief           Number of replayed measurements that the NIS gate
     *                  rejected against the rolled-back state; they stay
     *                  retained for later replays.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t replayRejectedCount{0U};

    /*!
     * @brief           Latest source publication timestamp.
     *
     * @frame           N/A
     * @units           ROS seconds
     */
    double publicationTimestamp_s{0.0};

    /*!
     * @brief           Latest callback admission timestamp.
     *
     * @frame           N/A
     * @units           ROS seconds
     */
    double callbackAdmissionTimestamp_s{0.0};

    /*!
     * @brief           Latest processing-start timestamp.
     *
     * @frame           N/A
     * @units           ROS seconds
     */
    double processingStartTimestamp_s{0.0};

    /*!
     * @brief           Latest callback completion timestamp.
     *
     * @frame           N/A
     * @units           ROS seconds
     */
    double processingEndTimestamp_s{0.0};

    /*!
     * @brief           Estimator epoch after the latest callback.
     *
     * @frame           N/A
     * @units           ROS seconds
     */
    double estimatorTimestamp_s{0.0};

    /*!
     * @brief           Latest normalized innovation squared value.
     *
     * @frame           N/A
     * @units           dimensionless
     */
    double normalizedInnovationSquared{0.0};

    /*!
     * @brief           Norm of the latest state correction.
     *
     * @frame           Mixed; see UkfStateIndex
     * @units           Mixed; see UkfStateIndex
     */
    double correctionNorm{0.0};
};

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */

#endif /* SRS_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_UKF_FILTER_SOURCE_DIAG_H */
