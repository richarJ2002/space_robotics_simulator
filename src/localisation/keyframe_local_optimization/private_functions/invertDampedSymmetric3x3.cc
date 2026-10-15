/*!
 * @file            invertDampedSymmetric3x3.cc
 *
 * @brief           Implements damped 3x3 symmetric inversion.
 *
 * @date            08/10/2026
 */

/* Matching Declaration Include */
#include "private_functions/private_functions.h"

/* Generic Libraries */
#include <cmath>

namespace localisation::keyframe_local_optimization
{

bool invertDampedSymmetric3x3(const Eigen::Matrix3d &matrix_in,
                              double                 damping_in,
                              Eigen::Matrix3d       &inverse_out)
{
    const Eigen::Matrix3d dampedMatrix =
        matrix_in + damping_in * Eigen::Matrix3d::Identity();
    const double determinant = dampedMatrix.determinant();
    if (!std::isfinite(determinant) || (std::fabs(determinant) < 1.0e-18))
    {
        return false;
    }
    inverse_out = dampedMatrix.inverse();
    return true;
}

} /* namespace localisation::keyframe_local_optimization */
