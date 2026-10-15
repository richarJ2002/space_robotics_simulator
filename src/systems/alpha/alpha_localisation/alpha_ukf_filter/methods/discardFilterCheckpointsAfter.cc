/*!
 * @File:         discardFilterCheckpointsAfter.cc
 *
 * @Brief:        Implements checkpoint truncation after a rollback epoch.
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

void AlphaUkfFilterNode::discardFilterCheckpointsAfter(
    double targetTimestamp_s_in) noexcept
{
    /* Retire every checkpoint newer than the rollback epoch, keeping the
     * ring's oldest-first order intact. */
    while (filterCheckpointCount > 0U)
    {
        const std::size_t newestIndex =
            (nextFilterCheckpointIndex + FILTER_CHECKPOINT_CAPACITY - 1U) %
            FILTER_CHECKPOINT_CAPACITY;
        if (filterCheckpoints[newestIndex].timestamp_s <= targetTimestamp_s_in)
        {
            break;
        }
        nextFilterCheckpointIndex = newestIndex;
        filterCheckpointCount--;
    }
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
