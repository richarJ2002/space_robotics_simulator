/*!
 * @File:         prepareOdometry.cc
 *
 * @Brief:        Implements validation and reframing of estimate odometry.
 *
 * @Date:         07/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/AlphaUkfFilterNodeClass.h"

/* Function Includes */
#include "console/console.h"

/* Generic Libraries */
/* None */

namespace systems::alpha::alpha_localisation::alpha_ukf_filter
{

bool AlphaUkfFilterNode::prepareOdometry(
    const nav_msgs::msg::Odometry &odometry_in,
    const std::string             &parentFrame_in,
    const std::string             &childFrame_in,
    nav_msgs::msg::Odometry       &odometry_out)
{
    /* Reject the message outright rather than append a NaN/degenerate
     * pose to the retained path. */
    if (!isPoseValid(odometry_in.pose.pose))
    {
        /* Bound log spam if the estimator keeps producing bad poses. */
        SRS_LOG_WARN_THROTTLE(get_logger(),
                              *get_clock(),
                              2000,
                              "Ignoring an invalid UKF pose");

        /* Nothing further can be done with this sample; tell the caller
         * so it also stops. */
        return false;
    }

    /* Start from a copy of the validated odometry message. */
    odometry_out = odometry_in;

    /* Overwrite the parent frame id with this node's configured value. */
    odometry_out.header.frame_id = parentFrame_in;

    /* Overwrite the child frame id the same way. */
    odometry_out.child_frame_id = childFrame_in;

    /* Tell the caller the reframed message is valid to use. */
    return true;
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
