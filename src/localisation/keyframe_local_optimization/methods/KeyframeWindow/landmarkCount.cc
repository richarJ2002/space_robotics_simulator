/*!
 * @File:         landmarkCount.cc
 *
 * @Brief:        Counts the stored landmarks.
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

std::size_t KeyframeWindow::landmarkCount() const
{
    /* An uninitialized window holds nothing by construction. */
    if (!isInitialized)
    {
        return 0U;
    }

    /* Live landmarks are those with at least one observation. */
    std::size_t liveLandmarkCount = 0U;
    for (std::size_t landmarkIndex = 0U; landmarkIndex < landmarkHighWaterMark;
         landmarkIndex++)
    {
        if (landmarkObservationCounts[landmarkIndex] > 0U)
        {
            liveLandmarkCount++;
        }
    }
    return liveLandmarkCount;
}

} /* namespace localisation::keyframe_local_optimization */
