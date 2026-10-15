/*!
 * @File:         runOptimization.cc
 *
 * @Brief:        Runs one Full BA over the window and publishes.
 *
 * @Date:         08/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/KeyframeLocalOptimizationNodeClass.h"

/* External Library Includes */
/* None */

/* Other Project Module Includes */
#include "console/console.h"

/* Generic Libraries */
/* None */

namespace localisation::keyframe_local_optimization
{

void KeyframeLocalOptimizationNode::runOptimization()
{
    /* Fewer than two keyframes leave the geometry empty. */
    if (keyframeWindow.keyframeCount() < 2U)
    {
        return;
    }

    /* Time the optimization against the 100 ms RelWithDebInfo budget. */
    const rclcpp::Time                    optimizationStart = now();
    const KeyframeLocalOptimizationStatus optimizeStatus =
        bundleOptimizer.optimize(keyframeWindow);
    const double optimizationDurationMs =
        (now() - optimizationStart).seconds() * 1000.0;
    latestOptimizationMs = optimizationDurationMs;

    if (optimizeStatus !=
        KeyframeLocalOptimizationStatus::KEYFRAME_LOCAL_OPTIMIZATION_SUCCESS)
    {
        SRS_LOG_DEBUG(get_logger(),
                      "Keyframe optimization skipped: status %d",
                      static_cast<int>(optimizeStatus));
        return;
    }
    optimizationCount++;

    /* Record the post-optimization robust cost for diagnostics. */
    double                                optimizedCost_m2 = 0.0;
    const KeyframeLocalOptimizationStatus costStatus =
        keyframeWindow.computeTotalError(solverConfiguration, optimizedCost_m2);
    if (costStatus ==
        KeyframeLocalOptimizationStatus::KEYFRAME_LOCAL_OPTIMIZATION_SUCCESS)
    {
        latestCost_m2 = optimizedCost_m2;
    }

    /* Paired-mode-only publish: pose-only inputs carry no
     * observations, so success implies paired mode; the guard
     * keeps that invariant explicit. */
    if (!isPoseOnly)
    {
        publishOdometry();
        KeyframeSample newestSample{};
        if (keyframeWindow.newestKeyframe(newestSample) ==
            KeyframeLocalOptimizationStatus::
                KEYFRAME_LOCAL_OPTIMIZATION_SUCCESS)
        {
            const rclcpp::Time newestStamp(
                static_cast<std::int64_t>(newestSample.stamp_s * 1.0e9));
            publishPointCloud(newestStamp);
            publishFeatureImage(newestStamp);
        }
    }

    /* Operator-visible cost and timing stay on diagnostics; the
     * console keeps one short line per optimization at most, plus
     * a throttled INFO summary so the terminal shows BA health
     * without 2 Hz spam at the default log level. */
    SRS_LOG_DEBUG(get_logger(),
                  "Keyframe BA #%llu: cost %.4f m2, %.1f ms",
                  static_cast<unsigned long long>(optimizationCount),
                  latestCost_m2,
                  latestOptimizationMs);
    SRS_LOG_INFO_THROTTLE(get_logger(),
                          *get_clock(),
                          10000,
                          "Keyframe BA #%llu: cost %.4f m2, %zu keys %zu marks",
                          static_cast<unsigned long long>(optimizationCount),
                          latestCost_m2,
                          keyframeWindow.keyframeCount(),
                          keyframeWindow.landmarkCount());
}

} /* namespace localisation::keyframe_local_optimization */
