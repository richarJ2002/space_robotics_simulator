/*!
 * @File:         initialize.cc
 *
 * @Brief:        Validates the keyframe window configuration.
 *
 * @Date:         08/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/KeyframeWindowClass.h"

/* External Library Includes */
/* None */

/* Other Project Module Includes */
/* None */

/* Generic Libraries */
#include <cmath>

namespace localisation::keyframe_local_optimization
{

KeyframeLocalOptimizationStatus KeyframeWindow::initialize(
    const KeyframeWindowConfiguration &configuration_in)
{
    /* A zero window retains nothing; beyond the cap breaks the fixed
     * ring and the solver's reduced-system bound. */
    const bool isWindowSizeValid =
        (configuration_in.windowSize >= 1U) &&
        (configuration_in.windowSize <= MAX_WINDOW_KEYFRAMES);
    const bool isGateValid =
        std::isfinite(configuration_in.associationGate_m) &&
        (configuration_in.associationGate_m > 0.0);
    if (!isWindowSizeValid || !isGateValid)
    {
        return KeyframeLocalOptimizationStatus::
            KEYFRAME_LOCAL_OPTIMIZATION_INVALID_CONFIGURATION;
    }

    /* Fixed storage needs no heap: reset counts so every slot reads as
     * empty, and start the landmark table with no live entries. */
    configuration         = configuration_in;
    storedKeyframeCount   = 0U;
    ringStartIndex        = 0U;
    freeLandmarkCount     = 0U;
    landmarkHighWaterMark = 0U;
    for (std::size_t landmarkIndex = 0U; landmarkIndex < MAX_LANDMARKS;
         landmarkIndex++)
    {
        landmarkObservationCounts[landmarkIndex] = 0U;
    }
    isInitialized = true;
    return KeyframeLocalOptimizationStatus::KEYFRAME_LOCAL_OPTIMIZATION_SUCCESS;
}

} /* namespace localisation::keyframe_local_optimization */
