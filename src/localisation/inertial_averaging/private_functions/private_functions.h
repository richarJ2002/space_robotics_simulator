/*!
 * @file            private_functions.h
 *
 * @brief           Declares the inertial averaging module's internal
 *                  helpers.
 *
 * @date            07/10/2026
 */

#ifndef SRS_LOCALISATION_INERTIAL_AVERAGING_PRIVATE_FUNCTIONS_H
#define SRS_LOCALISATION_INERTIAL_AVERAGING_PRIVATE_FUNCTIONS_H

/* Object Includes */
#include "objects/InertialAveragingSampleStruct.h"

namespace localisation::inertial_averaging
{

/*!
 * @brief       Checks one sample for usable averaging input.
 *
 * @param[in]   sample_in
 *              Sample to check.
 *
 * @return      True when stamp, acceleration and rate are all finite.
 */
bool isSampleFinite(const InertialAveragingSample &sample_in);

} /* namespace localisation::inertial_averaging */

#endif /* SRS_LOCALISATION_INERTIAL_AVERAGING_PRIVATE_FUNCTIONS_H */
