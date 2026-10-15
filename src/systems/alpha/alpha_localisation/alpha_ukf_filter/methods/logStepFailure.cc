/*!
 * @File:         logStepFailure.cc
 *
 * @Brief:        Implements throttled logging of a rejected UKF step.
 *
 * @Date:         07/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/AlphaUkfFilterNodeClass.h"

/* Function Includes */
#include "console/console.h"

/* Generic Libraries */
/* None */

namespace systems::alpha::alpha_localisation::alpha_ukf_filter
{

void AlphaUkfFilterNode::logStepFailure(FilterStatus status_in)
{
    /*!
     * Throttled to avoid flooding the log if the filter rejects steps
     * repeatedly (e.g. during a sustained numerical failure).
     */
    SRS_LOG_WARN_THROTTLE(get_logger(),
                          *get_clock(),
                          2000,
                          "UKF step failed (status %u)",
                          static_cast<unsigned int>(status_in));
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
