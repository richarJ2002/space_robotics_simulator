/*!
 * @File:         terminate.cc
 *
 * @Brief:        Releases the keyframe window.
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
/* None */

namespace localisation::keyframe_local_optimization
{

KeyframeLocalOptimizationStatus KeyframeWindow::terminate()
{
    /* Terminating twice is a caller defect, never a silent reset. */
    if (!isInitialized)
    {
        return KeyframeLocalOptimizationStatus::
            KEYFRAME_LOCAL_OPTIMIZATION_NOT_INITIALIZED;
    }

    /* Fixed storage holds no heap: dropping the counts releases every
     * slot logically, with nothing to deallocate. */
    storedKeyframeCount   = 0U;
    ringStartIndex        = 0U;
    freeLandmarkCount     = 0U;
    landmarkHighWaterMark = 0U;
    isInitialized         = false;
    return KeyframeLocalOptimizationStatus::KEYFRAME_LOCAL_OPTIMIZATION_SUCCESS;
}

} /* namespace localisation::keyframe_local_optimization */
