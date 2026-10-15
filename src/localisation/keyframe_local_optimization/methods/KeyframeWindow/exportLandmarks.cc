/*!
 * @File:         exportLandmarks.cc
 *
 * @Brief:        Exports live landmark positions in table order.
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

KeyframeLocalOptimizationStatus
    KeyframeWindow::exportLandmarks(Eigen::Vector3d *p_positions_out,
                                    std::size_t      capacity_in,
                                    std::size_t     &exportedCount_out) const
{
    /* Uninitialized use is a caller defect, never a silent empty. */
    if (!isInitialized)
    {
        return KeyframeLocalOptimizationStatus::
            KEYFRAME_LOCAL_OPTIMIZATION_NOT_INITIALIZED;
    }

    /* Count live landmarks first so an undersized caller buffer is
     * reported before any partial write. */
    std::size_t liveLandmarkCount = 0U;
    for (std::size_t tableIndex = 0U; tableIndex < landmarkHighWaterMark;
         tableIndex++)
    {
        if (landmarkObservationCounts[tableIndex] > 0U)
        {
            liveLandmarkCount++;
        }
    }
    if (capacity_in < liveLandmarkCount)
    {
        return KeyframeLocalOptimizationStatus::
            KEYFRAME_LOCAL_OPTIMIZATION_INVALID_CONFIGURATION;
    }

    /* Export live positions in stable table order. */
    std::size_t exportedTotal = 0U;
    for (std::size_t tableIndex = 0U; tableIndex < landmarkHighWaterMark;
         tableIndex++)
    {
        if (landmarkObservationCounts[tableIndex] > 0U)
        {
            p_positions_out[exportedTotal] = landmarkPositions[tableIndex];
            exportedTotal++;
        }
    }
    exportedCount_out = exportedTotal;
    return KeyframeLocalOptimizationStatus::KEYFRAME_LOCAL_OPTIMIZATION_SUCCESS;
}

} /* namespace localisation::keyframe_local_optimization */
