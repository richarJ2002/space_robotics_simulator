/*!
 * @File:         KeyframeLocalOptimizationStatusEnum.h
 *
 * @Brief:        Declares the result of each keyframe BA operation.
 *
 * @Date:         08/10/2026
 *
 */

#ifndef SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_STATUS_ENUM_H
#define SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_STATUS_ENUM_H

/* Data include */
/* None */

/* Generic Libraries */
#include <cstdint>

namespace localisation::keyframe_local_optimization
{

/*!
 * @brief       Result of a keyframe window or optimizer operation.
 *
 *              Follows the engine status pattern: pre-init calls
 *              report NOT_INITIALIZED, bad tuning reports
 *              INVALID_CONFIGURATION, and the two window edges have
 *              named values (WINDOW_OVERFLOW when the oldest keyframe
 *              was dropped to make room, INSUFFICIENT_DATA when fewer
 *              than two keyframes or no observation is available).
 */
enum class KeyframeLocalOptimizationStatus : std::uint8_t
{
    /*!
     * @brief       Operation completed; outputs hold valid data.
     */
    KEYFRAME_LOCAL_OPTIMIZATION_SUCCESS = 0U,

    /*!
     * @brief       Called before initialize().
     */
    KEYFRAME_LOCAL_OPTIMIZATION_NOT_INITIALIZED = 1U,

    /*!
     * @brief       initialize() rejected the configuration.
     */
    KEYFRAME_LOCAL_OPTIMIZATION_INVALID_CONFIGURATION = 2U,

    /*!
     * @brief       The oldest keyframe was dropped; the window is still valid.
     */
    KEYFRAME_LOCAL_OPTIMIZATION_WINDOW_OVERFLOW = 3U,

    /*!
     * @brief       Fewer than two keyframes or no observation to constrain.
     */
    KEYFRAME_LOCAL_OPTIMIZATION_INSUFFICIENT_DATA = 4U
};

} /* namespace localisation::keyframe_local_optimization */

#endif /* SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_STATUS_ENUM_H */
