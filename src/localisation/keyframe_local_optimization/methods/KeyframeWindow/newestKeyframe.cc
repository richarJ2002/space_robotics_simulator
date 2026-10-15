/*!
 * @File:         newestKeyframe.cc
 *
 * @Brief:        Copies the newest stored keyframe.
 *
 * @Date:         08/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/KeyframeWindowClass.h"

/* External Library Includes */
/* None */

/* Other Project Module Includes */
/* None */

/* Generic Libraries */
/* None */

namespace localisation::keyframe_local_optimization
{

KeyframeLocalOptimizationStatus
    KeyframeWindow::newestKeyframe(KeyframeSample &sample_out) const
{
    /* Uninitialized use is a caller defect, never a silent zero. */
    if (!isInitialized)
    {
        return KeyframeLocalOptimizationStatus::
            KEYFRAME_LOCAL_OPTIMIZATION_NOT_INITIALIZED;
    }
    if (storedKeyframeCount == 0U)
    {
        return KeyframeLocalOptimizationStatus::
            KEYFRAME_LOCAL_OPTIMIZATION_INSUFFICIENT_DATA;
    }

    /* Copy the newest ring entry's pose, twist, and covariance for
     * republication; its local points stay inside the window. */
    const std::size_t ringIndex =
        (ringStartIndex + storedKeyframeCount - 1U) % MAX_WINDOW_KEYFRAMES;
    const StoredKeyframe &storedKeyframe = keyframeRing[ringIndex];
    sample_out.stamp_s                   = storedKeyframe.stamp_s;
    sample_out.position_fixed_m          = storedKeyframe.position_fixed_m;
    sample_out.orientation_fixedToBody = storedKeyframe.orientation_fixedToBody;
    sample_out.poseCovariance          = storedKeyframe.poseCovariance;
    sample_out.twistLinear_body_mPs    = storedKeyframe.twistLinear_body_mPs;
    sample_out.twistAngular_body_radPs = storedKeyframe.twistAngular_body_radPs;
    sample_out.localPointCount         = 0U;
    sample_out.isPoseAvailable         = true;
    return KeyframeLocalOptimizationStatus::KEYFRAME_LOCAL_OPTIMIZATION_SUCCESS;
}

} /* namespace localisation::keyframe_local_optimization */
