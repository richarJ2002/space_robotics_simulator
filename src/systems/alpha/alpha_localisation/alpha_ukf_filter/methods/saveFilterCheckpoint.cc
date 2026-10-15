/*!
 * @File:         saveFilterCheckpoint.cc
 *
 * @Brief:        Implements checkpoint retention at the current epoch.
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

void AlphaUkfFilterNode::saveFilterCheckpoint()
{
    /* Overwrite the entry at the write index, advancing the oldest marker
     * once the ring is full so the horizon stays bounded. */
    FilterCheckpoint &slot = filterCheckpoints[nextFilterCheckpointIndex];
    slot.timestamp_s       = stateTimestamp_s;
    slot.state             = filter.getState();
    slot.covariance        = filter.getCovariance();
    nextFilterCheckpointIndex =
        (nextFilterCheckpointIndex + 1U) % FILTER_CHECKPOINT_CAPACITY;
    if (filterCheckpointCount < FILTER_CHECKPOINT_CAPACITY)
    {
        filterCheckpointCount++;
    }
    else
    {
        oldestFilterCheckpointIndex =
            (oldestFilterCheckpointIndex + 1U) % FILTER_CHECKPOINT_CAPACITY;
    }
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
