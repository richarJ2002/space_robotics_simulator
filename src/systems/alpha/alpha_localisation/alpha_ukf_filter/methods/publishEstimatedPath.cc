/*!
 * @File:         publishEstimatedPath.cc
 *
 * @Brief:        Implements retained-path publication without TF.
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

void AlphaUkfFilterNode::publishEstimatedPath(
    const nav_msgs::msg::Odometry &odometry_in)
{
    /* This node is comparison-only and never broadcasts TF: only the
     * retained path is updated from a validated estimate. */
    appendPathPose(odometry_in,
                   estimatedPath,
                   lastEstimatedPathStampNs,
                   *p_estimatedPathPublisher);
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
