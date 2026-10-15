/*!
 * @File:         rotationFromRpy.cc
 *
 * @Brief:        Implements roll/pitch/yaw to rotation conversion.
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

Eigen::Matrix3d AlphaUkfFilterNode::rotationFromRpy(double rollRad_in,
                                                   double pitchRad_in,
                                                   double yawRad_in)
{
    /* ZYX order: yaw about fixed z, then pitch, then roll. */
    const Eigen::AngleAxisd rollAngle(rollRad_in, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitchAngle(pitchRad_in, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yawAngle(yawRad_in, Eigen::Vector3d::UnitZ());

    return (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
