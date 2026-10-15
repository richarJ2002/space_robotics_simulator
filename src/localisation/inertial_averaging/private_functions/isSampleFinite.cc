/*!
 * @file            isSampleFinite.cc
 *
 * @brief           Implements one-sample averaging-input validation.
 *
 * @date            07/10/2026
 */

/* Matching Declaration Include */
#include "private_functions/private_functions.h"

/* Generic Libraries */
#include <cmath>
#include <cstddef>

namespace localisation::inertial_averaging
{

bool isSampleFinite(const InertialAveragingSample &sample_in)
{
    if (!std::isfinite(sample_in.stamp_s))
    {
        return false;
    }
    for (std::size_t axisIndex = 0U; axisIndex < 3U; axisIndex++)
    {
        if (!std::isfinite(sample_in.acceleration_mPs2[axisIndex]) ||
            !std::isfinite(sample_in.angularRate_radPs[axisIndex]))
        {
            return false;
        }
    }
    return true;
}

} /* namespace localisation::inertial_averaging */
