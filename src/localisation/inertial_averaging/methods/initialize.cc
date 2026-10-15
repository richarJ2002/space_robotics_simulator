/*!
 * @File:         initialize.cc
 *
 * @Brief:        Validates the inertial averager window configuration.
 *
 * @Date:         07/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/InertialAveragerClass.h"

/* External Library Includes */
/* None */

/* Other Project Module Includes */
/* None */

/* Generic Libraries */
#include <cmath>

namespace localisation::inertial_averaging
{

InertialAveragingStatus InertialAverager::initialize(
    const InertialAveragingConfiguration &configuration_in)
{
    /* Window bounds are positive counts capped at the ring capacity. */
    const bool isSampleTargetValid = configuration_in.windowSampleTarget >= 1 &&
                                     configuration_in.windowSampleTarget <=
                                         static_cast<int>(MAX_AVERAGER_SAMPLES);

    /* A non-positive or non-finite time bound would evict everything. */
    const bool isDurationValid =
        std::isfinite(configuration_in.windowDuration_ms) &&
        configuration_in.windowDuration_ms > 0.0;

    /* Reject bad tuning without touching the current state. */
    if (!isSampleTargetValid || !isDurationValid)
    {
        return InertialAveragingStatus::
            INERTIAL_AVERAGING_INVALID_CONFIGURATION;
    }

    /* Accepted tuning takes effect on a freshly emptied window. */
    configuration     = configuration_in;
    ringOldestIndex   = 0U;
    storedSampleCount = 0U;
    newestStamp_s     = 0.0;
    isInitialized     = true;
    return InertialAveragingStatus::INERTIAL_AVERAGING_SUCCESS;
}

} /* namespace localisation::inertial_averaging */
