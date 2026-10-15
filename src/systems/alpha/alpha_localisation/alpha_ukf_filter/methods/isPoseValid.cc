/*!
 * @File:         isPoseValid.cc
 *
 * @Brief:        Implements validation of a pose before republication.
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

bool AlphaUkfFilterNode::isPoseValid(const geometry_msgs::msg::Pose &pose_in)
{
    /*!
     * A zero-norm quaternion cannot represent a rotation; computing its
     * squared norm lets both that case and non-finite components be
     * checked together below.
     */
    const double orientationNormSquared =
        pose_in.orientation.x * pose_in.orientation.x +
        pose_in.orientation.y * pose_in.orientation.y +
        pose_in.orientation.z * pose_in.orientation.z +
        pose_in.orientation.w * pose_in.orientation.w;

    /*!
     * A degenerate (near-zero-norm) quaternion cannot be normalized and
     * would make the published estimate meaningless, so the squared norm
     * is checked alongside the usual finiteness of every component.
     */
    return std::isfinite(pose_in.position.x) &&
           std::isfinite(pose_in.position.y) &&
           std::isfinite(pose_in.position.z) &&
           std::isfinite(orientationNormSquared) &&
           orientationNormSquared > 1.0e-12;
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
