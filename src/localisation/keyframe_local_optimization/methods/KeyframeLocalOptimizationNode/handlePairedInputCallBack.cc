/*!
 * @File:         handlePairedInputCallBack.cc
 *
 * @Brief:        Accepts one synchronized pose-cloud pair.
 *
 * @Date:         08/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/KeyframeLocalOptimizationNodeClass.h"

/* External Library Includes */
#include <sensor_msgs/point_cloud2_iterator.hpp>

/* Other Project Module Includes */
#include "console/console.h"

/* Generic Libraries */
#include <cmath>

namespace localisation::keyframe_local_optimization
{

void KeyframeLocalOptimizationNode::handlePairedInputCallBack(
    const nav_msgs::msg::Odometry::ConstSharedPtr       &p_odometry_in,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &p_cloud_in)
{
    /* Guard the synchronizer contract: null pairs carry nothing. */
    if (!p_odometry_in || !p_cloud_in)
    {
        return;
    }
    receivedInputCount++;

    /* Post-sync age discard mirroring the VO input age: a stale pair
     * marks the pose unavailable instead of coasting it in. */
    const double pairAge_s = (now() - p_odometry_in->header.stamp).seconds();
    if (!(pairAge_s <= maximumInputAge_s))
    {
        staleDiscardedCount++;
        isPoseAvailable = false;
        SRS_LOG_DEBUG(get_logger(),
                      "Keyframe pair stale: age %.2f s, marked",
                      pairAge_s);
        return;
    }

    /* The visual cloud is expressed in the odom frame (see the VO
     * publishPointCloud contract): a foreign frame would corrupt
     * the window silently, so discard it like a stale pair. */
    if (p_cloud_in->header.frame_id != odomFrame)
    {
        staleDiscardedCount++;
        isPoseAvailable = false;
        SRS_LOG_DEBUG(get_logger(),
                      "Keyframe pair wrong frame: %s, marked",
                      p_cloud_in->header.frame_id.c_str());
        return;
    }

    /* Convert the paired pose into a source-agnostic sample. */
    KeyframeSample keyframeSample{};
    buildKeyframeSample(*p_odometry_in, keyframeSample);

    /* The visual cloud is expressed in the odom frame (see the VO
     * publishPointCloud contract); lift its points into the
     * keyframe-local frame with the paired pose, stride-sampling a
     * denser cloud down to the per-keyframe cap. */
    const Eigen::Matrix3d localFromWorldRotation =
        keyframeSample.orientation_fixedToBody.toRotationMatrix().transpose();
    const std::size_t cloudPointTotal =
        static_cast<std::size_t>(p_cloud_in->width) *
        static_cast<std::size_t>(p_cloud_in->height);
    std::size_t pointStride = 1U;
    if (cloudPointTotal > MAX_OBSERVATIONS_PER_KEYFRAME)
    {
        pointStride = cloudPointTotal / MAX_OBSERVATIONS_PER_KEYFRAME;
    }
    sensor_msgs::PointCloud2ConstIterator<float> cloudXIterator(*p_cloud_in,
                                                                "x");
    sensor_msgs::PointCloud2ConstIterator<float> cloudYIterator(*p_cloud_in,
                                                                "y");
    sensor_msgs::PointCloud2ConstIterator<float> cloudZIterator(*p_cloud_in,
                                                                "z");
    for (std::size_t cloudIndex = 0U; cloudIndex < cloudPointTotal;
         cloudIndex++)
    {
        const bool isKeptPoint = (cloudIndex % pointStride) == 0U;
        const bool hasRoom =
            keyframeSample.localPointCount < MAX_OBSERVATIONS_PER_KEYFRAME;
        if (isKeptPoint && hasRoom)
        {
            const float pointX_m = *cloudXIterator;
            const float pointY_m = *cloudYIterator;
            const float pointZ_m = *cloudZIterator;
            if (std::isfinite(pointX_m) && std::isfinite(pointY_m) &&
                std::isfinite(pointZ_m))
            {
                const Eigen::Vector3d worldPoint_m(
                    static_cast<double>(pointX_m),
                    static_cast<double>(pointY_m),
                    static_cast<double>(pointZ_m));
                const std::size_t localIndex = keyframeSample.localPointCount;
                keyframeSample.localPoints[localIndex] =
                    localFromWorldRotation *
                    (worldPoint_m - keyframeSample.position_fixed_m);
                keyframeSample.localPointCount++;
            }
        }
        ++cloudXIterator;
        ++cloudYIterator;
        ++cloudZIterator;
    }

    /* A fresh pair restores availability before entering the window. */
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
