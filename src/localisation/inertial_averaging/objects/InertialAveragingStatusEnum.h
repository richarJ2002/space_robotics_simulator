/*!
 * @File:         InertialAveragingStatusEnum.h
 *
 * @Brief:        Declares the result of each inertial averager operation.
 *
 * @Date:         07/10/2026
 *
 */

#ifndef SRS_LOCALISATION_INERTIAL_AVERAGING_STATUS_ENUM_H
#define SRS_LOCALISATION_INERTIAL_AVERAGING_STATUS_ENUM_H

/* Data include */
/* None */

/* Generic Libraries */
#include <cstdint>

namespace localisation::inertial_averaging
{

/*!
 * @brief       Result of an inertial averager operation.
 *
 *              Follows the engine status pattern: pre-init calls report
 *              NOT_INITIALIZED, bad tuning reports INVALID_CONFIGURATION,
 *              and the two window edges have named values (EMPTY_WINDOW
 *              when no sample can be averaged, WINDOW_OVERFLOW when the
 *              oldest sample was dropped to make room).
 */
enum class InertialAveragingStatus : std::uint8_t
{
    /*! The output sample holds the current window mean. */
    INERTIAL_AVERAGING_SUCCESS = 0U,

    /*! push() or sampleCount() ran before initialize(). */
    INERTIAL_AVERAGING_NOT_INITIALIZED = 1U,

    /*! initialize() rejected the configuration. */
    INERTIAL_AVERAGING_INVALID_CONFIGURATION = 2U,

    /*! No sample could be averaged (empty window, output untouched). */
    INERTIAL_AVERAGING_EMPTY_WINDOW = 3U,

    /*! The oldest sample was dropped; the output mean is still valid. */
    INERTIAL_AVERAGING_WINDOW_OVERFLOW = 4U
};

} /* namespace localisation::inertial_averaging */

#endif /* SRS_LOCALISATION_INERTIAL_AVERAGING_STATUS_ENUM_H */
