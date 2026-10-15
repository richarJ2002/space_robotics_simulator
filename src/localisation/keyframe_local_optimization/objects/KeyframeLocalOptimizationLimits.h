/*!
 * @File:         KeyframeLocalOptimizationLimits.h
 *
 * @Brief:        Declares the compile-time capacity of the keyframe
 *                bundle-adjustment window.
 *
 * @Date:         08/10/2026
 *
 */

#ifndef SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_LIMITS_H
#define SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_LIMITS_H

/* Data include */
/* None */

/* Generic Libraries */
#include <cstddef>

namespace localisation::keyframe_local_optimization
{

/*!
 * @brief       Maximum keyframes retained in the sliding window.
 *
 *              Bounds both memory (one fixed ring, never reallocated)
 *              and the reduced camera-system solve (at most 15 free
 *              poses once the first keyframe anchors the datum).
 */
inline constexpr std::size_t MAX_WINDOW_KEYFRAMES = 16U;

/*!
 * @brief       Maximum distinct landmarks held across the window.
 *
 *              Equals 1024 (the hard-cap MAXIMUM_SUPPORTED_FEATURES of
 *              the visual front-end) times MAX_WINDOW_KEYFRAMES, so a
 *              fully populated window can never overflow the table.
 */
inline constexpr std::size_t MAX_LANDMARKS = 16384U;

/*!
 * @brief       Maximum 3-D observations stored per keyframe.
 *
 *              Mirrors the visual front-end hard-cap
 *              MAXIMUM_SUPPORTED_FEATURES: a denser cloud is
 *              stride-sampled down to this bound on input.
 */
inline constexpr std::size_t MAX_OBSERVATIONS_PER_KEYFRAME = 1024U;

/*!
 * @brief       Maximum free pose parameters in one optimization.
 *
 *              The first keyframe anchors the datum, so at most
 *              (MAX_WINDOW_KEYFRAMES - 1) poses times 6 parameters
 *              enter the reduced camera system.
 */
inline constexpr std::size_t MAX_FREE_POSE_PARAMETERS =
    (MAX_WINDOW_KEYFRAMES - 1U) * 6U;

} /* namespace localisation::keyframe_local_optimization */

#endif /* SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_LIMITS_H */
