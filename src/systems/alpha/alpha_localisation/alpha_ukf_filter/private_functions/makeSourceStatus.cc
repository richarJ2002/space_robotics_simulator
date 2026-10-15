/*!
 * @file            makeSourceStatus.cc
 *
 * @brief           Implements per-source UKF diagnostics status.
 *
 * @date            07/10/2026
 */

/* Matching Declaration Include */
#include "private_functions/private_functions.h"

/* Other Project Module Includes */
#include "console/console.h"
#include "diagnostics/diagnostics.h"

namespace systems::alpha::alpha_localisation::alpha_ukf_filter
{

diagnostic_msgs::msg::DiagnosticStatus
    makeSourceStatus(const std::string       &sourceName_in,
                     const SourceDiagnostics &source_in)
{
    namespace diagnostics = common::diagnostics;

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name    = "continuous_ukf/" + sourceName_in;
    status.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = common::console::formatText(
        "%s fused %llu of %llu",
        sourceName_in.c_str(),
        static_cast<unsigned long long>(source_in.fusedCount),
        static_cast<unsigned long long>(source_in.receivedCount));
    diagnostics::addCountValue("received", source_in.receivedCount, status);
    diagnostics::addCountValue("accepted", source_in.acceptedCount, status);
    diagnostics::addCountValue("age_rejected",
                               source_in.ageRejectedCount,
                               status);
    diagnostics::addCountValue("nis_rejected",
                               source_in.nisRejectedCount,
                               status);
    diagnostics::addCountValue("numerical_rejected",
                               source_in.numericalRejectedCount,
                               status);
    diagnostics::addCountValue("fused", source_in.fusedCount, status);
    diagnostics::addRealValue("publication",
                              source_in.publicationTimestamp_s,
                              status);
    diagnostics::addRealValue("admission",
                              source_in.callbackAdmissionTimestamp_s,
                              status);
    diagnostics::addRealValue("start",
                              source_in.processingStartTimestamp_s,
                              status);
    diagnostics::addRealValue("end",
                              source_in.processingEndTimestamp_s,
                              status);
    diagnostics::addRealValue("estimator_epoch",
                              source_in.estimatorTimestamp_s,
                              status);
    diagnostics::addRealValue("nis",
                              source_in.normalizedInnovationSquared,
                              status);
    diagnostics::addRealValue("correction_norm",
                              source_in.correctionNorm,
                              status);
    diagnostics::addCountValue("pre_init",
                               source_in.preInitRejectedCount,
                               status);
    diagnostics::addCountValue("negative_age",
                               source_in.negativeAgeRejectedCount,
                               status);
    diagnostics::addCountValue("too_old",
                               source_in.tooOldRejectedCount,
                               status);
    diagnostics::addCountValue("state_gap",
                               source_in.stateGapRejectedCount,
                               status);
    diagnostics::addCountValue("rollback_failed",
                               source_in.rollbackFailedCount,
                               status);
    diagnostics::addCountValue("predict_failed",
                               source_in.predictFailedCount,
                               status);
    diagnostics::addCountValue("replayed", source_in.replayedCount, status);
    diagnostics::addCountValue("replay_rejected",
                               source_in.replayRejectedCount,
                               status);
    return status;
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
