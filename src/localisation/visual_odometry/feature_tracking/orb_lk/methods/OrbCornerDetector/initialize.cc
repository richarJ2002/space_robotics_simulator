/*!
 * @File:         initialize.cc
 *
 * @Brief:        Implements OpenCV ORB detector initialization.
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
#include <cmath>
#include <cstddef>

namespace localisation::visual_odometry::feature_tracking::orb_lk
{

/* This is an established public API: every parameter is independently
 * range-validated below, so a swapped call is rejected by that
 * validation, not merely by hoping the caller orders arguments
 * correctly. */
feature_tracking::gftt_lk::FeatureTrackingStatus OrbCornerDetector::initialize(
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    int         width_in,
    int         height_in,
    std::size_t maximumFeatures_in,
    double      scaleFactor_in,
    int         levels_in,
    int         fastThreshold_in) noexcept
{
    /* Reject a resolution outside the configured sanity ceiling before
     * ever touching it, rather than creating a detector for a frame
     * size the pipeline never produces. */
    if (width_in <= 0 ||
        width_in > feature_tracking::gftt_lk::MAXIMUM_SUPPORTED_WIDTH_PX ||
        height_in <= 0 ||
        height_in > feature_tracking::gftt_lk::MAXIMUM_SUPPORTED_HEIGHT_PX)
    {
        return feature_tracking::gftt_lk::FeatureTrackingStatus::
            FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION;
    }

    /* Reject a feature cap outside the compile-time capacity every
     * output buffer is sized to. */
    if (maximumFeatures_in == 0U ||
        maximumFeatures_in > MAXIMUM_SUPPORTED_FEATURES)
    {
        return feature_tracking::gftt_lk::FeatureTrackingStatus::
            FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION;
    }

    /* ORB needs a genuine pyramid decimation ratio. */
    if (!std::isfinite(scaleFactor_in) || !(scaleFactor_in > 1.0))
    {
        return feature_tracking::gftt_lk::FeatureTrackingStatus::
            FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION;
    }

    /* Bound the pyramid depth so one initialize() call cannot ask
     * OpenCV for an unbounded level count. */
    if (levels_in < 1 || levels_in > 16)
    {
        return feature_tracking::gftt_lk::FeatureTrackingStatus::
            FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION;
    }

    /* FAST needs a positive acceptance threshold. */
    if (fastThreshold_in <= 0)
    {
        return feature_tracking::gftt_lk::FeatureTrackingStatus::
            FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION;
    }

    /* Every parameter validated; commit the configuration. */
    width_ = width_in;

    /* Store the validated height. */
    height_ = height_in;

    /* Store the validated feature cap. */
    maximumFeatures_ = maximumFeatures_in;

    /* Create the owned ORB instance; an OpenCV failure here (out of
     * memory for the pyramid tables) is reported as a configuration
     * rejection rather than thrown across this noexcept boundary. */
    try
    {
        p_orbDetector = cv::ORB::create(static_cast<int>(maximumFeatures_in),
                                        static_cast<float>(scaleFactor_in),
                                        levels_in,
                                        OrbCornerDetector::EDGE_THRESHOLD_PX,
                                        0,
                                        2,
                                        cv::ORB::HARRIS_SCORE,
                                        OrbCornerDetector::PATCH_SIZE_PX,
                                        fastThreshold_in);
    }
    catch (...)
    {
        p_orbDetector.release();
        isInitialized_ = false;
        return feature_tracking::gftt_lk::FeatureTrackingStatus::
            FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION;
    }
    if (p_orbDetector.empty())
    {
        isInitialized_ = false;
        return feature_tracking::gftt_lk::FeatureTrackingStatus::
            FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION;
    }

    /* The defined initialization phase is now complete. */
    isInitialized_ = true;

    return feature_tracking::gftt_lk::FeatureTrackingStatus::
        FEATURE_TRACKING_STATUS_SUCCESS;
}

} /* namespace localisation::visual_odometry::feature_tracking::orb_lk */
