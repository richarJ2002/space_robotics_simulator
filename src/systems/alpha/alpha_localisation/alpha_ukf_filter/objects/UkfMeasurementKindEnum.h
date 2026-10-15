/*!
 * @File:         MeasurementKindEnum.h
 *
 * @Brief:        Declares which odometry source produced a measurement.
 *
 * @Date:         07/10/2026
 *
 */

/* Header Guard */
#ifndef SRS_SYSTEMS_ALPHA_ALPHA_UKF_FILTER_MEASUREMENT_ENUM_H
#define SRS_SYSTEMS_ALPHA_ALPHA_UKF_FILTER_MEASUREMENT_ENUM_H

/* Function Includes */
/* None */

/* Object Include */
/* None */

/* Data include */
/* None */

/* Generic Libraries */
#include <cstdint>

namespace systems::alpha::alpha_localisation::alpha_ukf_filter
{

/*!
 * @brief           Identifies which upstream odometry topic produced one
 *                  measurement passed into
 *                  AlphaUkfFilterNode::handleMeasurementCallBack.
 *
 * Each value selects a distinct UKF observation mapping and variance
 * source; see AlphaUkfFilterNodeClass.h for the fusion contract.
 */
enum class MeasurementKind : std::uint8_t
{
    /*!
     * @brief       Measurement originates from visual_odometry.
     */
    MEASUREMENT_KIND_VISUAL = 0U,

    /*!
     * @brief       Measurement originates from wheel_odometry.
     */
    MEASUREMENT_KIND_WHEEL = 1U
};

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */

#endif /* SRS_SYSTEMS_ALPHA_ALPHA_UKF_FILTER_MEASUREMENT_ENUM_H \
        */
