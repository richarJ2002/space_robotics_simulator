/*!
 * @File:         clearFilterCheckpoints.cc
 *
 * @Brief:        Implements full checkpoint retirement.
 *
 * @Date:         07/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/AlphaUkfFilterNodeClass.h"

/* Generic Libraries */
/* None */

namespace systems::alpha::alpha_localisation::alpha_ukf_filter
{

void AlphaUkfFilterNode::clearFilterCheckpoints() noexcept
{
    nextFilterCheckpointIndex   = 0U;
    oldestFilterCheckpointIndex = 0U;
    filterCheckpointCount       = 0U;
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
