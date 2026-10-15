/*!
 * @File:         handleImuCallBack.cc
 *
 * @Brief:        Implements stationary initialization and raw-IMU UKF
 *                prediction.
 *
 * @Date:         07/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/AlphaUkfFilterNodeClass.h"

/* Other Project Module Includes */
#include "console/console.h"

/* C++ Standard Library Includes */
#include <cmath>

/* Generic Libraries */
/* None */

namespace systems::alpha::alpha_localisation::alpha_ukf_filter
{

void AlphaUkfFilterNode::handleImuCallBack(
    const sensor_msgs::msg::Imu &message_in)
{
    const double admissionTimestamp_s = now().seconds();
    imuDiagnostics.receivedCount++;
    imuDiagnostics.callbackAdmissionTimestamp_s = admissionTimestamp_s;
    imuDiagnostics.processingStartTimestamp_s   = admissionTimestamp_s;
    const auto finishDiagnostics                = [this]()
    {
        imuDiagnostics.processingEndTimestamp_s = now().seconds();
        imuDiagnostics.estimatorTimestamp_s     = stateTimestamp_s;
    };

    ImuSample sample;
    sample.timestamp_s =
        rclcpp::Time(message_in.header.stamp, RCL_ROS_TIME).seconds();
    sample.linearAcceleration_body_mPs2 =
        Eigen::Vector3d(message_in.linear_acceleration.x,
                        message_in.linear_acceleration.y,
                        message_in.linear_acceleration.z);
    sample.angularVelocity_body_radPs =
        Eigen::Vector3d(message_in.angular_velocity.x,
                        message_in.angular_velocity.y,
                        message_in.angular_velocity.z);
    imuDiagnostics.publicationTimestamp_s = sample.timestamp_s;

    if (!std::isfinite(sample.timestamp_s) ||
        !sample.linearAcceleration_body_mPs2.allFinite() ||
        !sample.angularVelocity_body_radPs.allFinite())
    {
        imuDiagnostics.numericalRejectedCount++;
        finishDiagnostics();
        return;
    }

    /* A backwards clock jump invalidates every retained sample, so the
     * estimator restarts from uninitialized rather than fusing across
     * the discontinuity. */
    if (!imuBuffer.empty() && sample.timestamp_s < imuBuffer.back().timestamp_s)
    {
        imuBuffer.clear();
        clearFilterCheckpoints();
        measurementHistory.clear();
        if (hasInitialState)
        {
            const FilterStatus terminateStatus = filter.terminate();
            if (terminateStatus != FilterStatus::UKF_STATUS_SUCCESS)
            {
                logStepFailure(terminateStatus);
            }
        }
        hasInitialState              = false;
        hasEstimate                  = false;
        lastPublishedStateTimestamp_s = -1.0;
        imuInitializationSampleCount = 0U;
        initializationSpecificForceSum_body_mPs2.setZero();
        initializationAngularVelocitySum_body_radPs.setZero();
        imuDiagnostics.ageRejectedCount++;
        finishDiagnostics();
        return;
    }

    /* Retain the sample, dropping the oldest once the buffer is full. */
    imuBuffer.push_back(sample);
    while (imuBuffer.size() > IMU_BUFFER_CAPACITY)
    {
        imuBuffer.pop_front();
    }

    /* Accumulate stationary samples until calibration completes. */
    if (!hasInitialState)
    {
        initializationSpecificForceSum_body_mPs2 +=
            sample.linearAcceleration_body_mPs2;
        initializationAngularVelocitySum_body_radPs +=
            sample.angularVelocity_body_radPs;
        imuInitializationSampleCount++;
        if (imuInitializationSampleCount >= imuInitializationSampleTarget)
        {
            const FilterStatus initializeStatus =
                initializeFilter(sample.timestamp_s);
            if (initializeStatus != FilterStatus::UKF_STATUS_SUCCESS)
            {
                logStepFailure(initializeStatus);
                imuInitializationSampleCount = 0U;
                initializationSpecificForceSum_body_mPs2.setZero();
                initializationAngularVelocitySum_body_radPs.setZero();
            }
        }
        finishDiagnostics();
        return;
    }

    /* Propagate the filter to this sample's timestamp. */
    const FilterStatus predictStatus = predictTo(sample.timestamp_s, true);
    if (predictStatus != FilterStatus::UKF_STATUS_SUCCESS)
    {
        imuDiagnostics.predictFailedCount++;
        logStepFailure(predictStatus);
        finishDiagnostics();
        return;
    }

    /* Publish the propagated state, rate-capped like the fused output. */
    publishEstimate();
    finishDiagnostics();
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
