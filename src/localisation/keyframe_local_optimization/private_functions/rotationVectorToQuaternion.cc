/*!
 * @file            rotationVectorToQuaternion.cc
 *
 * @brief           Implements rotation-vector to quaternion mapping.
 *
 * @date            08/10/2026
 */

/* Matching Declaration Include */
#include "private_functions/private_functions.h"

namespace localisation::keyframe_local_optimization
{

Eigen::Quaterniond
    rotationVectorToQuaternion(const Eigen::Vector3d &rotationVector_in)
{
    const double rotationAngle_rad = rotationVector_in.norm();
    if (rotationAngle_rad < 1.0e-12)
    {
        return Eigen::Quaterniond::Identity();
    }
    const Eigen::Vector3d rotationAxis = rotationVector_in / rotationAngle_rad;
    return Eigen::Quaterniond(
        Eigen::AngleAxisd(rotationAngle_rad, rotationAxis));
}

} /* namespace localisation::keyframe_local_optimization */
