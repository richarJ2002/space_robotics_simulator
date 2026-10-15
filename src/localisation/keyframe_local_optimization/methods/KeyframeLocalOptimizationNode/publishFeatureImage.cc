/*!
 * @File:         publishFeatureImage.cc
 *
 * @Brief:        Publishes the landmark-scatter features image.
 *
 * @Date:         08/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/KeyframeLocalOptimizationNodeClass.h"

/* External Library Includes */
/* None */

/* Other Project Module Includes */
/* None */

/* Generic Libraries */
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace localisation::keyframe_local_optimization
{

void KeyframeLocalOptimizationNode::publishFeatureImage(
    const builtin_interfaces::msg::Time &stamp_in)
{
    /* Export live landmarks into the member scratch buffer. */
    std::size_t                           exportedLandmarkTotal = 0U;
    const KeyframeLocalOptimizationStatus exportStatus =
        keyframeWindow.exportLandmarks(scratchLandmarks.data(),
                                       MAX_LANDMARKS,
                                       exportedLandmarkTotal);
    if (exportStatus !=
        KeyframeLocalOptimizationStatus::KEYFRAME_LOCAL_OPTIMIZATION_SUCCESS)
    {
        return;
    }

    /* Fit the landmark X/Y span to the frame with a margin, so the
     * scatter carries the optimized topology, not raw tracks: the BA
     * consumes odometry plus cloud only, so no tracked-feature image
     * exists to mirror here. */
    double minimumX_m = 0.0;
    double maximumX_m = 1.0;
    double minimumY_m = 0.0;
    double maximumY_m = 1.0;
    if (exportedLandmarkTotal > 0U)
    {
        minimumX_m = scratchLandmarks[0].x();
        maximumX_m = scratchLandmarks[0].x();
        minimumY_m = scratchLandmarks[0].y();
        maximumY_m = scratchLandmarks[0].y();
        for (std::size_t landmarkOrdinal = 1U;
             landmarkOrdinal < exportedLandmarkTotal;
             landmarkOrdinal++)
        {
            const Eigen::Vector3d &landmarkPosition_m =
                scratchLandmarks[landmarkOrdinal];
            if (!std::isfinite(landmarkPosition_m.x()) ||
                !std::isfinite(landmarkPosition_m.y()))
            {
                continue;
            }
            if (landmarkPosition_m.x() < minimumX_m)
            {
                minimumX_m = landmarkPosition_m.x();
            }
            if (landmarkPosition_m.x() > maximumX_m)
            {
                maximumX_m = landmarkPosition_m.x();
            }
            if (landmarkPosition_m.y() < minimumY_m)
            {
                minimumY_m = landmarkPosition_m.y();
            }
            if (landmarkPosition_m.y() > maximumY_m)
            {
                maximumY_m = landmarkPosition_m.y();
            }
        }
    }
    double spanX_m = maximumX_m - minimumX_m;
    double spanY_m = maximumY_m - minimumY_m;
    if (!(spanX_m > 1.0e-6))
    {
        spanX_m = 1.0e-6;
    }
    if (!(spanY_m > 1.0e-6))
    {
        spanY_m = 1.0e-6;
    }

    /* Black mono8 frame, one white dot per finite landmark. */
    std::vector<std::uint8_t> imagePixels(SCATTER_IMAGE_WIDTH_PX *
                                              SCATTER_IMAGE_HEIGHT_PX,
                                          0U);
    for (std::size_t landmarkOrdinal = 0U;
         landmarkOrdinal < exportedLandmarkTotal;
         landmarkOrdinal++)
    {
        const Eigen::Vector3d &landmarkPosition_m =
            scratchLandmarks[landmarkOrdinal];
        if (!std::isfinite(landmarkPosition_m.x()) ||
            !std::isfinite(landmarkPosition_m.y()))
        {
            continue;
        }
        const std::size_t pixelColumn = static_cast<std::size_t>(
            (landmarkPosition_m.x() - minimumX_m) / spanX_m *
            static_cast<double>(SCATTER_IMAGE_WIDTH_PX - 1U));
        const std::size_t pixelRow = static_cast<std::size_t>(
            (landmarkPosition_m.y() - minimumY_m) / spanY_m *
            static_cast<double>(SCATTER_IMAGE_HEIGHT_PX - 1U));
        imagePixels[pixelRow * SCATTER_IMAGE_WIDTH_PX + pixelColumn] = 255U;
    }

    /* Message to populate and publish below. */
    sensor_msgs::msg::Image image;
    image.header.stamp    = stamp_in;
    image.header.frame_id = odomFrame;
    image.height          = static_cast<std::uint32_t>(SCATTER_IMAGE_HEIGHT_PX);
    image.width           = static_cast<std::uint32_t>(SCATTER_IMAGE_WIDTH_PX);
    image.encoding        = "mono8";
    image.is_bigendian    = false;
    image.step            = static_cast<std::uint32_t>(SCATTER_IMAGE_WIDTH_PX);
    image.data            = imagePixels;

    /* Comparison-only output: recorded only under --record-images. */
    p_featureImagePublisher->publish(image);
}

} /* namespace localisation::keyframe_local_optimization */
