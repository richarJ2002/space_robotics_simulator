/*!
 * @File:         buildMeasurementRecord.cc
 *
 * @Brief:        Implements odometry-to-record conversion for replay.
 *
 * @Date:         07/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/AlphaUkfFilterNodeClass.h"

/* C++ Standard Library Includes */
#include <cmath>

/* Object Includes */
#include "objects/UkfStateIndexEnum.h"

/* Generic Libraries */
/* None */

namespace systems::alpha::alpha_localisation::alpha_ukf_filter
{

bool AlphaUkfFilterNode::buildMeasurementRecord(
    const nav_msgs::msg::Odometry &message_in,
    MeasurementKind                kind_in,
    double                         measurementTimestamp_s_in,
    FusedMeasurement              &record_out) const
{
    FusedMeasurement record;
    record.kind        = kind_in;
    record.timestamp_s = measurementTimestamp_s_in;

    if (kind_in == MeasurementKind::MEASUREMENT_KIND_VISUAL)
    {
        /* Reject a degenerate visual orientation before conversion. */
        const Eigen::Quaterniond measuredQuaternion(
            message_in.pose.pose.orientation.w,
            message_in.pose.pose.orientation.x,
            message_in.pose.pose.orientation.y,
            message_in.pose.pose.orientation.z);
        if (!measuredQuaternion.coeffs().allFinite() ||
            measuredQuaternion.squaredNorm() <= 1.0e-12)
        {
            return false;
        }
        double roll_rad  = 0.0;
        double pitch_rad = 0.0;
        double yaw_rad   = 0.0;
        rpyFromRotation(measuredQuaternion.normalized().toRotationMatrix(),
                        roll_rad,
                        pitch_rad,
                        yaw_rad);
        record.measurement.resize(VISUAL_MEASUREMENT_SIZE);
        record.measurement(0) = message_in.pose.pose.position.x;
        record.measurement(1) = message_in.pose.pose.position.y;
        record.measurement(2) = message_in.pose.pose.position.z;
        record.measurement(3) = roll_rad;
        record.measurement(4) = pitch_rad;
        record.measurement(5) = yaw_rad;

        /* Position uses the reported variances; roll/pitch use the
         * deliberately loose attitude floor so visual odometry anchors
         * long-run drift without overriding the IMU short-term. */
        const MeasurementVarianceVector variances =
            measurementVariances(message_in, visualVariance);
        record.noise =
            Eigen::MatrixXd::Zero(VISUAL_MEASUREMENT_SIZE,
                                  VISUAL_MEASUREMENT_SIZE);
        record.noise(0, 0) = variances(0);
        record.noise(1, 1) = variances(1);
        record.noise(2, 2) = variances(2);
        record.noise(3, 3) = visualAttitudeVariance;
        record.noise(4, 4) = visualAttitudeVariance;
        record.noise(5, 5) = variances(5);
    }
    else
    {
        record.measurement.resize(WHEEL_MEASUREMENT_SIZE);
        record.measurement(0) = message_in.twist.twist.linear.x;
        record.measurement(1) = message_in.twist.twist.linear.y;
        record.measurement(2) = message_in.twist.twist.linear.z;

        /* Wheel body velocity uses the reported twist variances. */
        const MeasurementVarianceVector variances =
            measurementVariances(message_in, wheelVariance);
        record.noise =
            Eigen::MatrixXd::Zero(WHEEL_MEASUREMENT_SIZE,
                                  WHEEL_MEASUREMENT_SIZE);
        record.noise(0, 0) = variances(6);
        record.noise(1, 1) = variances(7);
        record.noise(2, 2) = variances(8);
    }

    if (!record.measurement.allFinite() || !record.noise.allFinite())
    {
        return false;
    }

    record_out = record;
    return true;
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
