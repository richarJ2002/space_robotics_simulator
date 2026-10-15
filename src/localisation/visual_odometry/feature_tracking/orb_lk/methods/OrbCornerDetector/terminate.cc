/*!
 * @File:         terminate.cc
 *
 * @Brief:        Implements OpenCV ORB detector teardown.
 *
 * @Date:         07/10/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "feature_tracking/orb_lk/objects/OrbCornerDetector.h"

/* Data include */
/* None */

/* Generic Libraries */
/* None */

namespace localisation::visual_odometry::feature_tracking::orb_lk
{

feature_tracking::gftt_lk::FeatureTrackingStatus
OrbCornerDetector::terminate() noexcept
{
    /* Release the owned ORB instance. */
    p_orbDetector.release();

    /* Return every configuration field to its default, uninitialized
     * value. */
    width_           = 0;
    height_          = 0;
    maximumFeatures_ = 0U;

    /* A subsequent detect() call must once again be rejected until the
     * next successful initialize(). */
    isInitialized_ = false;

    return feature_tracking::gftt_lk::FeatureTrackingStatus::
        FEATURE_TRACKING_STATUS_SUCCESS;
}

} /* namespace localisation::visual_odometry::feature_tracking::orb_lk */
