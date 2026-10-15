/*!
 * @File:         UkfConfigurationStruct.h
 *
 * @Brief:        Declares the sigma-point scaling of the continuous UKF.
 *
 * @Date:         07/10/2026
 *
 */

/* Header Guard */
#ifndef SRS_LOCALISATION_UKF_CONTINUOUS_KALMAN_FILTER_UKF_CONFIGURATION_H
#define SRS_LOCALISATION_UKF_CONTINUOUS_KALMAN_FILTER_UKF_CONFIGURATION_H

/* Function Includes */
/* None */

/* Object Include */
/* None */

/* Data include */
/* None */

/* Generic Libraries */
/* None */

namespace localisation::kalman_filter::ukf_continuous_kalman_filter
{

/*!
 * @brief           Sigma-point scaling of the unscented transform.
 *
 *                  The composite scaling is
 *                  lambda = alpha^2 (n + kappa) - n for a state of
 *                  dimension n. The YAML file holding these values is
 *                  the authority; the defaults below are starting
 *                  points only.
 */
struct UkfConfiguration
{
    /*!
     * @brief       Primary spread control, in (0, 1]; small values keep
     *              sigma points near the mean. Default 1.0e-3.
     */
    double sigmaAlpha{1.0e-3};

    /*!
     * @brief       Prior-knowledge weight on the central point;
     *              2.0 is optimal for a Gaussian prior.
     */
    double sigmaBeta{2.0};

    /*!
     * @brief       Secondary spread control, usually 0.0.
     */
    double sigmaKappa{0.0};
};

} /* namespace localisation::kalman_filter::ukf_continuous_kalman_filter */

#endif /* SRS_LOCALISATION_UKF_CONTINUOUS_KALMAN_FILTER_UKF_CONFIGURATION_H */
