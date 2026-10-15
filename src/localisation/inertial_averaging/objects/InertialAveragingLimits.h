/*!
 * @File:         InertialAveragingLimits.h
 *
 * @Brief:        Declares the window capacity of the inertial averager.
 *
 * @Date:         07/10/2026
 *
 */

#ifndef SRS_LOCALISATION_INERTIAL_AVERAGING_LIMITS_H
#define SRS_LOCALISATION_INERTIAL_AVERAGING_LIMITS_H

/* Data include */
/* None */

/* Generic Libraries */
#include <cstddef>

namespace localisation::inertial_averaging
{

/*!
 * @brief       Maximum IMU samples retained in the averaging window.
 *
 *              Bounds both memory (one fixed ring, never reallocated) and
 *              lag. The count and time bounds each drop the oldest sample
 *              first once full.
 */
inline constexpr std::size_t MAX_AVERAGER_SAMPLES = 32U;

} /* namespace localisation::inertial_averaging */

#endif /* SRS_LOCALISATION_INERTIAL_AVERAGING_LIMITS_H */
