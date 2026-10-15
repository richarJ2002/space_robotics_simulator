/*!
 * @File:         rpyFromRotation.cc
 *
 * @Brief:        Implements rotation to roll/pitch/yaw conversion.
 *
 * @Date:         07/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/AlphaUkfFilterNodeClass.h"

/* C++ Standard Library Includes */
#include <cmath>

/* Generic Libraries */
/* None */

namespace systems::alpha::alpha_localisation::alpha_ukf_filter
{

void AlphaUkfFilterNode::rpyFromRotation(const Eigen::Matrix3d &rotation_in,
                                         double                &rollRad_out,
                                         double                &pitchRad_out,
                                         double                &yawRad_out)
{
    /* ZYX extraction matching rotationFromRpy(); pitch stays within
     * [-pi/2, pi/2] by construction of asin. */
    rollRad_out  = std::atan2(rotation_in(2, 1), rotation_in(2, 2));
    pitchRad_out = std::asin(
        std::max(-1.0, std::min(1.0, -rotation_in(2, 0))));
    yawRad_out = std::atan2(rotation_in(1, 0), rotation_in(0, 0));
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
