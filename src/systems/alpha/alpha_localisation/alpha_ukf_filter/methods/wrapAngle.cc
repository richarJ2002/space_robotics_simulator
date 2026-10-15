/*!
 * @File:         wrapAngle.cc
 *
 * @Brief:        Implements angle wrapping into [-pi, pi].
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

double AlphaUkfFilterNode::wrapAngle(double angleRad_in)
{
    /* Fold into [-pi, pi] without branching on the common case. */
    double wrappedAngle_rad = std::fmod(angleRad_in + M_PI, 2.0 * M_PI);
    if (wrappedAngle_rad < 0.0)
    {
        wrappedAngle_rad += 2.0 * M_PI;
    }
    return wrappedAngle_rad - M_PI;
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
