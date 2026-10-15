/*!
 * @file            skewOfVector.cc
 *
 * @brief           Implements the skew-symmetric matrix of a vector.
 *
 * @date            08/10/2026
 */

/* Matching Declaration Include */
#include "private_functions/private_functions.h"

namespace localisation::keyframe_local_optimization
{

Eigen::Matrix3d skewOfVector(const Eigen::Vector3d &vector_in)
{
    Eigen::Matrix3d skewMatrix;
    skewMatrix << 0.0, -vector_in.z(), vector_in.y(), vector_in.z(), 0.0,
        -vector_in.x(), -vector_in.y(), vector_in.x(), 0.0;
    return skewMatrix;
}

} /* namespace localisation::keyframe_local_optimization */
