/*!
 * @File:         setNoiseTriplet.cc
 *
 * @Brief:        Implements process-noise density triplet assignment.
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

void AlphaUkfFilterNode::setNoiseTriplet(StateCovariance &noise_inout,
                                         Eigen::Index     firstIndex_in,
                                         double           variance_in)
{
    /* Floor above zero so the density stays numerically conditioning. */
    const double flooredVariance =
        variance_in > 0.0 ? variance_in : 1.0e-12;
    noise_inout(firstIndex_in, firstIndex_in)         = flooredVariance;
    noise_inout(firstIndex_in + 1, firstIndex_in + 1) = flooredVariance;
    noise_inout(firstIndex_in + 2, firstIndex_in + 2) = flooredVariance;
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
