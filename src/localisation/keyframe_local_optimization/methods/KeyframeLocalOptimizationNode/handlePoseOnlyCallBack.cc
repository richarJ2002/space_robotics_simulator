/*!
 * @File:         handlePoseOnlyCallBack.cc
 *
 * @Brief:        Accepts one EKF pose in pose-only mode.
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

void KeyframeLocalOptimizationNode::handlePoseOnlyCallBack(
    const nav_msgs::msg::Odometry &odometry_in)
{
    receivedInputCount++;

    /* Post-sync age discard: a stale pose marks the pose unavailable
     * instead of coasting it into the window. */
    const double poseAge_s = (now() - odometry_in.header.stamp).seconds();
    if (!(poseAge_s <= maximumInputAge_s))
    {
        staleDiscardedCount++;
        isPoseAvailable = false;
        SRS_LOG_DEBUG(get_logger(),
                      "Keyframe pose stale: age %.2f s, marked",
                      poseAge_s);
        return;
    }

    /* Pose-only keyframes carry no observations; the cloud topic stays
     * unsubscribed and no cloud or features are published. */
    KeyframeSample keyframeSample{};
    buildKeyframeSample(odometry_in, keyframeSample);
    isPoseAvailable = true;
    const KeyframeLocalOptimizationStatus pushStatus =
        keyframeWindow.pushKeyframe(keyframeSample);
    if ((pushStatus != KeyframeLocalOptimizationStatus::
                           KEYFRAME_LOCAL_OPTIMIZATION_SUCCESS) &&
        (pushStatus != KeyframeLocalOptimizationStatus::
                           KEYFRAME_LOCAL_OPTIMIZATION_WINDOW_OVERFLOW))
    {
        SRS_LOG_DEBUG(get_logger(),
                      "Keyframe push rejected: status %d",
                      static_cast<int>(pushStatus));
        return;
    }

    /* Count mode optimizes every time the window fills. */
    if ((triggerModeName == "count") &&
        (keyframeWindow.keyframeCount() >= windowConfiguration.windowSize))
    {
        runOptimization();
    }
}

} /* namespace localisation::keyframe_local_optimization */
