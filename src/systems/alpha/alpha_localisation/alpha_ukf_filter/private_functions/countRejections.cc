/*!
 * @file            countRejections.cc
 *
 * @brief           Implements per-source rejection counting.
 *
 * @date            07/10/2026
 */

/* Matching Declaration Include */
#include "private_functions/private_functions.h"

namespace systems::alpha::alpha_localisation::alpha_ukf_filter
{

std::uint64_t countRejections(const SourceDiagnostics &source_in)
{
    return source_in.ageRejectedCount + source_in.nisRejectedCount +
           source_in.numericalRejectedCount;
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
