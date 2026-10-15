/*!
 * @File:         publishOdometry.cc
 *
 * @Brief:        Publishes the newest optimized keyframe pose.
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
#include <cstddef>

namespace localisation::keyframe_local_optimization
{

void KeyframeLocalOptimizationNode::publishOdometry()
{
    /* Newest optimized keyframe, or nothing when the window is empty. */
    KeyframeSample newestSample{};
    if (keyframeWindow.newestKeyframe(newestSample) !=
        KeyframeLocalOptimizationStatus::KEYFRAME_LOCAL_OPTIMIZATION_SUCCESS)
    {
        return;
    }

    /* Message to populate and publish below. */
    nav_msgs::msg::Odometry output;

    /* Timestamp the message with the newest keyframe's stamp. */
    const rclcpp::Time newestStamp(
        static_cast<std::int64_t>(newestSample.stamp_s * 1.0e9));
    output.header.stamp = newestStamp;

    /* Pose below is expressed in this fixed frame. */
    output.header.frame_id = odomFrame;

    /* Child frame this odometry describes the motion of. */
    output.child_frame_id = baseFrame;

    /* Copy the optimized position. */
    output.pose.pose.position.x = newestSample.position_fixed_m.x();
    output.pose.pose.position.y = newestSample.position_fixed_m.y();
    output.pose.pose.position.z = newestSample.position_fixed_m.z();

    /* Copy the optimized orientation. */
    output.pose.pose.orientation.w = newestSample.orientation_fixedToBody.w();
    output.pose.pose.orientation.x = newestSample.orientation_fixedToBody.x();
    output.pose.pose.orientation.y = newestSample.orientation_fixedToBody.y();
    output.pose.pose.orientation.z = newestSample.orientation_fixedToBody.z();

    /* Carry the input pose covariance, translation then rotation. */
    for (std::size_t covarianceIndex = 0U; covarianceIndex < 36U;
         covarianceIndex++)
    {
        const std::size_t rowIndex    = covarianceIndex / 6U;
        const std::size_t columnIndex = covarianceIndex % 6U;
        output.pose.covariance[covarianceIndex] =
            newestSample.poseCovariance(static_cast<Eigen::Index>(rowIndex),
                                        static_cast<Eigen::Index>(columnIndex));
    }

    /* Carry the input body-frame twist unchanged. */
    output.twist.twist.linear.x  = newestSample.twistLinear_body_mPs.x();
    output.twist.twist.linear.y  = newestSample.twistLinear_body_mPs.y();
    output.twist.twist.linear.z  = newestSample.twistLinear_body_mPs.z();
    output.twist.twist.angular.x = newestSample.twistAngular_body_radPs.x();
    output.twist.twist.angular.y = newestSample.twistAngular_body_radPs.y();
    output.twist.twist.angular.z = newestSample.twistAngular_body_radPs.z();

    /* Comparison-only output: never fused, never gates READY. */
    p_odometryPublisher->publish(output);
}

} /* namespace localisation::keyframe_local_optimization */
