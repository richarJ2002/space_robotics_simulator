/*!
 * @File:         publishPointCloud.cc
 *
 * @Brief:        Publishes the optimized landmark cloud.
 *
 * @Date:         08/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/KeyframeLocalOptimizationNodeClass.h"

/* External Library Includes */
#include <sensor_msgs/point_cloud2_iterator.hpp>

/* Other Project Module Includes */
/* None */

/* Generic Libraries */
#include <cmath>
#include <cstddef>

namespace localisation::keyframe_local_optimization
{

void KeyframeLocalOptimizationNode::publishPointCloud(
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

    /* Message to populate and publish below. */
    sensor_msgs::msg::PointCloud2 cloud;

    /* Timestamp the cloud with the newest keyframe's stamp. */
    cloud.header.stamp = stamp_in;

    /* Every point below is already expressed in this frame. */
    cloud.header.frame_id = odomFrame;

    /* An unorganized point cloud is always exactly one row tall. */
    cloud.height = 1U;

    /* Declare a plain float32 x/y/z point cloud. This ROS API is
     * itself a C-style variadic function; there is no non-vararg
     * alternative. */
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    modifier.setPointCloud2FieldsByString(1, "xyz");

    /* Collect finite landmarks; a non-finite one is skipped rather
     * than corrupting the packed buffer. */
    std::size_t publishedPointTotal = 0U;
    for (std::size_t landmarkOrdinal = 0U;
         landmarkOrdinal < exportedLandmarkTotal;
         landmarkOrdinal++)
    {
        const Eigen::Vector3d &landmarkPosition_m =
            scratchLandmarks[landmarkOrdinal];
        if (std::isfinite(landmarkPosition_m.x()) &&
            std::isfinite(landmarkPosition_m.y()) &&
            std::isfinite(landmarkPosition_m.z()))
        {
            publishedPointTotal++;
        }
    }

    /* Size the underlying buffer for exactly this many points. */
    modifier.resize(publishedPointTotal);

    /* Writable views over the message's x/y/z fields. */
    sensor_msgs::PointCloud2Iterator<float> cloudXIterator(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> cloudYIterator(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> cloudZIterator(cloud, "z");

    /* Copy every finite landmark into the message's packed buffer. */
    for (std::size_t landmarkOrdinal = 0U;
         landmarkOrdinal < exportedLandmarkTotal;
         landmarkOrdinal++)
    {
        const Eigen::Vector3d &landmarkPosition_m =
            scratchLandmarks[landmarkOrdinal];
        if (!std::isfinite(landmarkPosition_m.x()) ||
            !std::isfinite(landmarkPosition_m.y()) ||
            !std::isfinite(landmarkPosition_m.z()))
        {
            continue;
        }
        *cloudXIterator = static_cast<float>(landmarkPosition_m.x());
        *cloudYIterator = static_cast<float>(landmarkPosition_m.y());
        *cloudZIterator = static_cast<float>(landmarkPosition_m.z());

        /* Advance to the next point's slots. Prefix increment: these
         * sensor_msgs iterators declare no postfix operator++. */
        ++cloudXIterator;
        ++cloudYIterator;
        ++cloudZIterator;
    }

    /*!
     * Publishing unconditionally, even with zero points, lets
     * subscribers tell "no landmarks this window" apart from "node
     * not running".
     */
    cloud.is_dense = true;
    p_pointCloudPublisher->publish(cloud);
}

} /* namespace localisation::keyframe_local_optimization */
