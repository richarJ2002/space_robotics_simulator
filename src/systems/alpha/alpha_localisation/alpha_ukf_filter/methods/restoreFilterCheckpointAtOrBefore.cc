/*!
 * @File:         restoreFilterCheckpointAtOrBefore.cc
 *
 * @Brief:        Implements rollback to a retained checkpoint.
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

bool AlphaUkfFilterNode::restoreFilterCheckpointAtOrBefore(
    double targetTimestamp_s_in)
{
    /* Scan from newest to oldest for the first checkpoint at or before
     * the target epoch. */
    for (std::size_t offset = 0U; offset < filterCheckpointCount; offset++)
    {
        const std::size_t slotIndex =
            (nextFilterCheckpointIndex + FILTER_CHECKPOINT_CAPACITY - 1U -
             offset) %
            FILTER_CHECKPOINT_CAPACITY;
        const FilterCheckpoint &candidate = filterCheckpoints[slotIndex];
        if (candidate.timestamp_s <= targetTimestamp_s_in)
        {
            const FilterStatus restoreStatus =
                filter.restore(candidate.state, candidate.covariance);
            if (restoreStatus != FilterStatus::UKF_STATUS_SUCCESS)
            {
                return false;
            }
            stateTimestamp_s   = candidate.timestamp_s;
            latestState        = candidate.state;
            latestCovariance   = candidate.covariance;
            return true;
        }
    }
    return false;
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
