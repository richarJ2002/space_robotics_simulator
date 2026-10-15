/*!
 * @File:         publishDiagnosticsCallBack.cc
 *
 * @Brief:        Publishes periodic optimizer diagnostics.
 *
 * @Date:         08/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/KeyframeLocalOptimizationNodeClass.h"

/* External Library Includes */
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

/* Other Project Module Includes */
#include "console/console.h"
#include "diagnostics/diagnostics.h"

/* Generic Libraries */
/* None */

namespace localisation::keyframe_local_optimization
{

void KeyframeLocalOptimizationNode::publishDiagnosticsCallBack()
{
    namespace diagnostics = common::diagnostics;

    /* Readiness contract: comparison-only, so this status never gates
     * READY; the supervisor only aggregates required_components.
     * Pose-only (ekf) inputs carry no observations, so the
     * optimizer stays INSUFFICIENT_DATA and no odometry is ever
     * published: report pose-only idle (non-ready) instead of
     * tracking once the window fills. */
    std::string readinessReason = "tracking";
    if (!isPoseAvailable)
    {
        readinessReason = "pose unavailable";
    }
    else if (isPoseOnly)
    {
        readinessReason = "pose-only idle";
    }
    else if (triggerModeName == "offline-first")
    {
        readinessReason = "offline-first idle";
    }
    else if (keyframeWindow.keyframeCount() < windowConfiguration.windowSize)
    {
        readinessReason = "filling window";
    }
    const bool isReady = (readinessReason == "tracking");

    /* Status machine-read by bag_diagnostics.py's documented-ignore
     * rule (see that module): only the keys below are parsed there. */
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name        = "keyframe_local_optimization";
    status.hardware_id = baseFrame;
    status.level       = isReady ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                 : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message =
        common::console::formatText("BA %s keys %zu marks %zu cost %.4f",
                                    readinessReason.c_str(),
                                    keyframeWindow.keyframeCount(),
                                    keyframeWindow.landmarkCount(),
                                    latestCost_m2);
    diagnostics::addFlagValue("ready", isReady, status);
    diagnostics::addTextValue("reason", readinessReason, status);
    diagnostics::addTextValue("trigger_mode", triggerModeName, status);
    diagnostics::addCountValue("received", receivedInputCount, status);
    diagnostics::addCountValue("stale_discarded", staleDiscardedCount, status);
    diagnostics::addCountValue("optimizations", optimizationCount, status);
    diagnostics::addCountValue("keyframes",
                               keyframeWindow.keyframeCount(),
                               status);
    diagnostics::addCountValue("landmarks",
                               keyframeWindow.landmarkCount(),
                               status);
    diagnostics::addRealValue("last_cost_m2", latestCost_m2, status);
    diagnostics::addRealValue("last_optimization_ms",
                              latestOptimizationMs,
                              status);

    /* One status per record, stamped on the simulated clock. */
    diagnostic_msgs::msg::DiagnosticArray diagnosticsArray;
    diagnosticsArray.header.stamp = now();
    diagnosticsArray.status.push_back(status);
    p_diagnosticsPublisher->publish(diagnosticsArray);
}

} /* namespace localisation::keyframe_local_optimization */
