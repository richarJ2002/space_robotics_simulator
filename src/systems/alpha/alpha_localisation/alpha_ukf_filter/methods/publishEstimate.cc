/*!
 * @File:         publishEstimate.cc
 *
 * @Brief:        Implements rate-capped UKF estimate publication.
 *
 * @Date:         07/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/AlphaUkfFilterNodeClass.h"

/* C++ Standard Library Includes */
#include <cstddef>

/* Object Includes */
#include "objects/UkfStateIndexEnum.h"

/* Generic Libraries */
/* None */

namespace systems::alpha::alpha_localisation::alpha_ukf_filter
{

void AlphaUkfFilterNode::publishEstimate()
{
    if (!hasEstimate)
    {
        return;
    }

    /* Honor the configured output cap in simulation time. */
    if (lastPublishedStateTimestamp_s >= 0.0 &&
        stateTimestamp_s - lastPublishedStateTimestamp_s < minimumOutputPeriodS)
    {
        return;
    }
    lastPublishedStateTimestamp_s = stateTimestamp_s;

    static constexpr Eigen::Index POSITION_INDEX =
        static_cast<Eigen::Index>(UkfStateIndex::UKF_STATE_INDEX_POSITION_X);
    static constexpr Eigen::Index ATTITUDE_INDEX =
        static_cast<Eigen::Index>(
            UkfStateIndex::UKF_STATE_INDEX_ATTITUDE_ROLL);
    static constexpr Eigen::Index VELOCITY_INDEX =
        static_cast<Eigen::Index>(UkfStateIndex::UKF_STATE_INDEX_VELOCITY_X);
    static constexpr Eigen::Index GYROSCOPE_BIAS_INDEX =
        static_cast<Eigen::Index>(
            UkfStateIndex::UKF_STATE_INDEX_GYROSCOPE_BIAS_X);

    const Eigen::Matrix3d rotation_bodyToFixed = rotationFromRpy(
        latestState(ATTITUDE_INDEX),
        latestState(ATTITUDE_INDEX + 1),
        latestState(ATTITUDE_INDEX + 2));
    const Eigen::Quaterniond quaternion_bodyToFixed(rotation_bodyToFixed);
    const Eigen::Vector3d    velocity_body_mPs =
        rotation_bodyToFixed.transpose() *
        latestState.segment<3>(VELOCITY_INDEX);

    nav_msgs::msg::Odometry output;
    output.header.stamp = rclcpp::Time(
        static_cast<std::int64_t>(
            stateTimestamp_s * NANOSECONDS_PER_SECOND),
        RCL_ROS_TIME);
    output.header.frame_id      = odomFrame;
    output.child_frame_id       = baseFrame;
    output.pose.pose.position.x = latestState(POSITION_INDEX);
    output.pose.pose.position.y = latestState(POSITION_INDEX + 1);
    output.pose.pose.position.z = latestState(POSITION_INDEX + 2);
    output.pose.pose.orientation.w = quaternion_bodyToFixed.w();
    output.pose.pose.orientation.x = quaternion_bodyToFixed.x();
    output.pose.pose.orientation.y = quaternion_bodyToFixed.y();
    output.pose.pose.orientation.z = quaternion_bodyToFixed.z();
    output.twist.twist.linear.x  = velocity_body_mPs.x();
    output.twist.twist.linear.y  = velocity_body_mPs.y();
    output.twist.twist.linear.z  = velocity_body_mPs.z();
    output.twist.twist.angular.x = latestAngularVelocity_body_radPs.x();
    output.twist.twist.angular.y = latestAngularVelocity_body_radPs.y();
    output.twist.twist.angular.z = latestAngularVelocity_body_radPs.z();

    /*!
     * Publish the position and attitude blocks directly: position maps
     * one-to-one, and roll/pitch/yaw uncertainty is reported as the
     * orientation block (a small-angle approximation documented here
     * rather than hidden). Twist velocity maps through R^T P_vv R; the
     * angular block reports the gyroscope-bias variance.
     */
    Eigen::Matrix<double, 6, 6> poseCovariance =
        Eigen::Matrix<double, 6, 6>::Zero();
    poseCovariance.block<3, 3>(0, 0) =
        latestCovariance.block<3, 3>(POSITION_INDEX, POSITION_INDEX);
    poseCovariance.block<3, 3>(3, 3) =
        latestCovariance.block<3, 3>(ATTITUDE_INDEX, ATTITUDE_INDEX);
    Eigen::Matrix<double, 6, 6> twistCovariance =
        Eigen::Matrix<double, 6, 6>::Zero();
    twistCovariance.block<3, 3>(0, 0) =
        rotation_bodyToFixed.transpose() *
        latestCovariance.block<3, 3>(VELOCITY_INDEX, VELOCITY_INDEX) *
        rotation_bodyToFixed;
    twistCovariance.block<3, 3>(3, 3) =
        latestCovariance.block<3, 3>(GYROSCOPE_BIAS_INDEX,
                                     GYROSCOPE_BIAS_INDEX);

    for (Eigen::Index row = 0; row < 6; row++)
    {
        for (Eigen::Index column = 0; column < 6; column++)
        {
            const std::size_t covarianceIndex =
                static_cast<std::size_t>(row * 6 + column);
            output.pose.covariance[covarianceIndex] =
                poseCovariance(row, column);
            output.twist.covariance[covarianceIndex] =
                twistCovariance(row, column);
        }
    }

    nav_msgs::msg::Odometry reframedOutput;
    if (!prepareOdometry(output, odomFrame, baseFrame, reframedOutput))
    {
        return;
    }
    p_outputPublisher->publish(reframedOutput);
    publishEstimatedPath(reframedOutput);
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
