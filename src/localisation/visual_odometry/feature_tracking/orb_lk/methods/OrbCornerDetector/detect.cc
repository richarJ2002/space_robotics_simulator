/*!
 * @File:         detect.cc
 *
 * @Brief:        Implements OpenCV ORB corner detection over one image.
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
#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>

namespace localisation::visual_odometry::feature_tracking::orb_lk
{

feature_tracking::gftt_lk::FeatureTrackingStatus OrbCornerDetector::detect(
    const feature_tracking::gftt_lk::ImageView &image_in,
    std::array<feature_tracking::gftt_lk::Point2D,
               feature_tracking::gftt_lk::MAXIMUM_SUPPORTED_FEATURES>
        &features_out,
    std::size_t &featureCount_out) noexcept
{
    /* No output is valid until this call succeeds. */
    featureCount_out = 0U;

    /* Reject use before a successful initialize(). */
    if (!isInitialized_ || p_orbDetector.empty())
    {
        return feature_tracking::gftt_lk::FeatureTrackingStatus::
            FEATURE_TRACKING_STATUS_NOT_INITIALIZED;
    }

    /* Reject an image whose dimensions do not match initialize(). */
    if (image_in.p_pixels == nullptr || image_in.width != width_ ||
        image_in.height != height_)
    {
        return feature_tracking::gftt_lk::FeatureTrackingStatus::
            FEATURE_TRACKING_STATUS_INVALID_INPUT;
    }

    /* Wrap the borrowed buffer in an OpenCV header without copying;
     * ORB detect never mutates its input, so the const_cast only
     * satisfies cv::Mat's own non-const constructor. */
    cv::Mat imageMat(
        image_in.height,
        image_in.width,
        CV_8UC1,
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        const_cast<std::uint8_t *>(image_in.p_pixels),
        image_in.strideBytes);

    /* Run the owned ORB detector; an OpenCV failure here is reported
     * as invalid input rather than thrown across this boundary. */
    std::vector<cv::KeyPoint> keypoints;
    try
    {
        p_orbDetector->detect(imageMat, keypoints);
    }
    catch (...)
    {
        featureCount_out = 0U;
        return feature_tracking::gftt_lk::FeatureTrackingStatus::
            FEATURE_TRACKING_STATUS_INVALID_INPUT;
    }

    /* Cap the accepted count at both the configured maximum and the
     * compile-time capacity every output buffer is sized to. */
    std::size_t acceptedCount = keypoints.size();
    if (acceptedCount > maximumFeatures_)
    {
        acceptedCount = maximumFeatures_;
    }
    if (acceptedCount > MAXIMUM_SUPPORTED_FEATURES)
    {
        acceptedCount = MAXIMUM_SUPPORTED_FEATURES;
    }

    /* Report the accepted keypoint positions in OpenCV's own order. */
    for (std::size_t featureIndex = 0U; featureIndex < acceptedCount;
         featureIndex++)
    {
        features_out[featureIndex].x = keypoints[featureIndex].pt.x;
        features_out[featureIndex].y = keypoints[featureIndex].pt.y;
    }
    featureCount_out = acceptedCount;

    return feature_tracking::gftt_lk::FeatureTrackingStatus::
        FEATURE_TRACKING_STATUS_SUCCESS;
}

} /* namespace localisation::visual_odometry::feature_tracking::orb_lk */
