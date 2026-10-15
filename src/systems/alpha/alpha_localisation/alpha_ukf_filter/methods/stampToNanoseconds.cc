/*!
 * @File:         stampToNanoseconds.cc
 *
 * @Brief:        Implements ROS timestamp to nanosecond conversion.
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

std::int64_t AlphaUkfFilterNode::stampToNanoseconds(
    const builtin_interfaces::msg::Time &stamp_in)
{
    /*!
     * ROS splits a timestamp into whole seconds and a nanosecond remainder;
     * combine both fields into one linear count so samples can be compared
     * and subtracted directly.
     */
    return static_cast<std::int64_t>(stamp_in.sec) *
               static_cast<std::int64_t>(NANOSECONDS_PER_SECOND) +
           static_cast<std::int64_t>(stamp_in.nanosec);
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
