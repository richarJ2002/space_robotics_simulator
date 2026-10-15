/*!
 * @File:         UkfStatusEnum.h
 *
 * @Brief:        Declares continuous UKF lifecycle result values.
 *
 * @Date:         07/10/2026
 *
 */

/* Header Guard */
#ifndef SRS_LOCALISATION_UKF_CONTINUOUS_KALMAN_FILTER_UKF_STATUS_ENUM_H
#define SRS_LOCALISATION_UKF_CONTINUOUS_KALMAN_FILTER_UKF_STATUS_ENUM_H

/* Function Includes */
/* None */

/* Object Include */
/* None */

/* Data include */
/* None */

/* Generic Libraries */
#include <cstdint>

namespace localisation::kalman_filter::ukf_continuous_kalman_filter
{

/*!
 * @brief           Reports the outcome of a filter lifecycle operation.
 */
enum class UkfStatus : std::uint8_t
{
    /*!
     * @brief       Operation completed successfully.
     */
    UKF_STATUS_SUCCESS = 0U,

    /*!
     * @brief       Configuration contains a non-positive state size, a
     *              dimension mismatch, non-finite values, or an invalid
     *              sigma-point scaling (non-positive spread or
     *              non-positive composite scaling).
     */
    UKF_STATUS_INVALID_CONFIGURATION = 1U,

    /*!
     * @brief       Call input contains a dimension mismatch against the
     *              configured state size, or non-finite/invalid values.
     */
    UKF_STATUS_INVALID_INPUT = 2U,

    /*!
     * @brief       Operation was requested before the filter was
     *              initialized.
     */
    UKF_STATUS_NOT_INITIALIZED = 3U,

    /*!
     * @brief       Matrix operations produced a non-finite result, or a
     *              Cholesky factorization of the scaled covariance failed.
     */
    UKF_STATUS_NUMERICAL_FAILURE = 4U
};

} /* namespace localisation::kalman_filter::ukf_continuous_kalman_filter */

#endif /* SRS_LOCALISATION_UKF_CONTINUOUS_KALMAN_FILTER_UKF_STATUS_ENUM_H */
