/*!
 * @File:         buildKeyframeSample.cc
 *
 * @Brief:        Converts odometry into a keyframe sample.
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

namespace localisation::keyframe_local_optimization
{

void KeyframeLocalOptimizationNode::buildKeyframeSample(
    const nav_msgs::msg::Odometry &odometry_in,
    KeyframeSample                &sample_out)
{
    /* Stamp in seconds from the odometry header. */
    sample_out.stamp_s =
        static_cast<double>(odometry_in.header.stamp.sec) +
        static_cast<double>(odometry_in.header.stamp.nanosec) * 1.0e-9;

    /* Keyframe position in the odom frame, in metres. */
    sample_out.position_fixed_m =
        Eigen::Vector3d(odometry_in.pose.pose.position.x,
                        odometry_in.pose.pose.position.y,
                        odometry_in.pose.pose.position.z);

    /* Keyframe orientation; a degenerate quaternion falls back to
     * identity rather than injecting NaN into the window. */
    Eigen::Quaterniond orientation_fixedToBody(
        odometry_in.pose.pose.orientation.w,
        odometry_in.pose.pose.orientation.x,
        odometry_in.pose.pose.orientation.y,
        odometry_in.pose.pose.orientation.z);
    if (!(orientation_fixedToBody.squaredNorm() > 1.0e-12))
    {
        orientation_fixedToBody = Eigen::Quaterniond::Identity();
    }
    sample_out.orientation_fixedToBody = orientation_fixedToBody.normalized();

    /* Pose covariance, translation then rotation, row-major. */
    for (std::size_t rowIndex = 0U; rowIndex < 6U; rowIndex++)
    {
        for (std::size_t columnIndex = 0U; columnIndex < 6U; columnIndex++)
        {
            sample_out.poseCovariance(static_cast<Eigen::Index>(rowIndex),
                                      static_cast<Eigen::Index>(columnIndex)) =
                odometry_in.pose.covariance[rowIndex * 6U + columnIndex];
        }
    }

    /* Body-frame twist carried through for republication. */
    sample_out.twistLinear_body_mPs =
        Eigen::Vector3d(odometry_in.twist.twist.linear.x,
                        odometry_in.twist.twist.linear.y,
                        odometry_in.twist.twist.linear.z);
    sample_out.twistAngular_body_radPs =
        Eigen::Vector3d(odometry_in.twist.twist.angular.x,
                        odometry_in.twist.twist.angular.y,
                        odometry_in.twist.twist.angular.z);

    /* Local points are filled by the paired callback afterwards. */
    sample_out.localPointCount = 0U;
    sample_out.isPoseAvailable = true;
}

} /* namespace localisation::keyframe_local_optimization */
