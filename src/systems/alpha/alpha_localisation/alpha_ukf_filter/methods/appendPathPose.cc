/*!
 * @File:         appendPathPose.cc
 *
 * @Brief:        Implements time-sampled bounded path accumulation.
 *
 * @Date:         07/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/AlphaUkfFilterNodeClass.h"

/* Generic Libraries */
/* None */

namespace systems::alpha::alpha_localisation::alpha_ukf_filter
{

void AlphaUkfFilterNode::appendPathPose(
    const nav_msgs::msg::Odometry          &odometry_in,
    nav_msgs::msg::Path                    &path_inout,
    std::int64_t                           &lastStampNs_inout,
    rclcpp::Publisher<nav_msgs::msg::Path> &publisher_inout)
{
    const std::int64_t stampNs = stampToNanoseconds(odometry_in.header.stamp);

    /* Keep the path sampled at the configured period, not at the full
     * estimate rate. */
    if (lastStampNs_inout >= 0 &&
        stampNs - lastStampNs_inout < pathSamplePeriodNs)
    {
        return;
    }
    lastStampNs_inout = stampNs;

    /* Carry the estimate's stamp and parent frame onto the path. */
    path_inout.header.stamp    = odometry_in.header.stamp;
    path_inout.header.frame_id = odometry_in.header.frame_id;

    /* Append this estimate's pose to the retained history. */
    geometry_msgs::msg::PoseStamped pathPose;
    pathPose.header = odometry_in.header;
    pathPose.pose   = odometry_in.pose.pose;
    path_inout.poses.push_back(pathPose);

    /* Drop the oldest poses once the retained history is full. */
    while (path_inout.poses.size() > pathMaximumPoses)
    {
        path_inout.poses.erase(path_inout.poses.begin());
    }

    publisher_inout.publish(path_inout);
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
