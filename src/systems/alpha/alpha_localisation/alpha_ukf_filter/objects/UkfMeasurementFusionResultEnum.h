/*!
 * @File:         MeasurementFusionResultEnum.h
 *
 * @Brief:        Declares the outcome of applying one odometry update.
 *
 * @Date:         07/10/2026
 *
 */

/* Header Guard */
#ifndef SRS_SYSTEMS_ALPHA_ALPHA_UKF_FILTER_FUSION_RESULT_H
#define SRS_SYSTEMS_ALPHA_ALPHA_UKF_FILTER_FUSION_RESULT_H

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
 * @brief           Distinguishes a statistical rejection from a numerical
 *                  one, because the node counts them separately.
 */
enum class MeasurementFusionResult : std::uint8_t
{
    /*!
     * @brief           The unscented update succeeded.
     */
    MEASUREMENT_FUSION_RESULT_FUSED = 0U,

    /*!
     * @brief           The normalized innovation squared exceeded the gate;
     *                  the filter was not modified.
     */
    MEASUREMENT_FUSION_RESULT_NIS_REJECTED = 1U,

    /*!
     * @brief           Sigma-point generation, NIS scoring or the update
     *                  failed numerically.
     */
    MEASUREMENT_FUSION_RESULT_NUMERICAL_REJECTED = 2U
};

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */

#endif /* SRS_SYSTEMS_ALPHA_ALPHA_UKF_FILTER_FUSION_RESULT_H \
        */
