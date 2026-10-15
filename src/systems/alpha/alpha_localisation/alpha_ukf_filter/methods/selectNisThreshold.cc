/*!
 * @File:         selectNisThreshold.cc
 *
 * @Brief:        Implements configured or 99% chi-square NIS thresholds.
 *
 * @Date:         07/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/AlphaUkfFilterNodeClass.h"

/* C++ Standard Library Includes */
#include <array>
#include <limits>

/* Generic Libraries */
/* None */

namespace systems::alpha::alpha_localisation::alpha_ukf_filter
{

double AlphaUkfFilterNode::selectNisThreshold(
    double       configuredThreshold_in,
    Eigen::Index measurementDimension_in)
{
    if (configuredThreshold_in > 0.0)
    {
        return configuredThreshold_in;
    }

    /* 99th percentiles of chi-square distributions for 1..12 degrees of
     * freedom, covering every currently configurable active update size.
     * Duplicated from the EKF wrapper: the UKF innovation covariance P_yy
     * plays the role of the EKF's S with the same dimension mapping. */
    constexpr std::array<double, 12> CHI_SQUARE_99_PERCENT{
        6.635,  9.210,  11.345, 13.277, 15.086, 16.812,
        18.475, 20.090, 21.666, 23.209, 24.725, 26.217};
    if (measurementDimension_in <= 0 ||
        measurementDimension_in >
            static_cast<Eigen::Index>(CHI_SQUARE_99_PERCENT.size()))
    {
        return std::numeric_limits<double>::infinity();
    }
    return CHI_SQUARE_99_PERCENT[static_cast<std::size_t>(
        measurementDimension_in - 1)];
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
