/*!
 * @file            quaternionToRotationVector.cc
 *
 * @brief           Implements quaternion to rotation-vector mapping.
 *
 * @date            08/10/2026
 */

/* Matching Declaration Include */
#include "private_functions/private_functions.h"

namespace localisation::keyframe_local_optimization
{

Eigen::Vector3d
    quaternionToRotationVector(const Eigen::Quaterniond &orientation_in)
{
    Eigen::AngleAxisd angleAxis(orientation_in);
    return angleAxis.angle() * angleAxis.axis();
}

} /* namespace localisation::keyframe_local_optimization */
