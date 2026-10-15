/*!
 * @File:         sampleCount.cc
 *
 * @Brief:        Counts the samples stored in the averaging window.
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
#include <cstddef>

namespace localisation::inertial_averaging
{

std::size_t InertialAverager::sampleCount() const
{
    /* Infallible accessor: uninitialized state simply holds nothing. */
    return storedSampleCount;
}

} /* namespace localisation::inertial_averaging */
