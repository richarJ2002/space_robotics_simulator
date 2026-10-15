/*!
 * @File:         ContinuousUnscentedKalmanFilterClass.h
 *
 * @Brief:        Declares a reusable, model-agnostic continuous-discrete
 *                UKF math engine.
 *
 * @Date:         07/10/2026
 *
 */

/* Header Guard */
#ifndef SRS_LOCALISATION_UKF_CONTINUOUS_KALMAN_FILTER_CONTINUOUS_UKF_CLASS_H
#define SRS_LOCALISATION_UKF_CONTINUOUS_KALMAN_FILTER_CONTINUOUS_UKF_CLASS_H

/* Function Includes */
/* None */

/* Object Include */
#include "objects/UkfConfigurationStruct.h"
#include "objects/UkfStatusEnum.h"

/* Data include */
#include <Eigen/Dense>

/* Generic Libraries */
/* None */

namespace localisation::kalman_filter::ukf_continuous_kalman_filter
{

/*!
 * @brief           Executes unscented prediction and correction for an
 *                   arbitrary, runtime-sized nonlinear state-space model
 *                   with additive noise.
 *
 * This engine holds no knowledge of what any state or measurement means:
 * the caller generates sigma points at the engine's current state --
 * read via getState()/getCovariance() -- propagates each point through
 * its own nonlinear process model, maps each point through its own
 * nonlinear observation model, and hands the resulting sigma-point
 * matrices to predictFromSigmaPoints() and updateFromSigmaPoints().
 * The engine performs only the weighted recombination algebra: sigma
 * weights follow the scaled unscented transform, the predicted
 * measurement covariance P_yy and cross-covariance are formed
 * explicitly, and the gain is solved without an explicit inverse. It
 * does not track wall/sim time, bound integration step size, or wrap
 * any state component (e.g. an angle) -- all of that is the caller's
 * model-specific responsibility, exercised by calling generateSigmaPoints()
 * and predictFromSigmaPoints() repeatedly with a bounded dt and by
 * calling setState() after any wrapping.
 *
 * State and covariance are sized once by initialize() and are not resized
 * afterward; every subsequent call is dimension-checked against that size.
 * The object owns no resources beyond its own dynamically-sized Eigen
 * matrices and performs no allocation once initialized (its methods only
 * allocate the local intermediates their math requires). It is not
 * thread-safe; its owner shall serialize lifecycle calls.
 */
class ContinuousUnscentedKalmanFilter
{
  public:
    /*!
     * @brief           Creates a valid, uninitialized filter.
     */
    ContinuousUnscentedKalmanFilter() noexcept = default;

    /*!
     * @brief           Releases the filter without external side effects.
     */
    ~ContinuousUnscentedKalmanFilter() noexcept = default;

    /*!
     * @brief           Copying is forbidden: the engine retains
     *                      mutable filter state with single ownership.
     */
    ContinuousUnscentedKalmanFilter(
        const ContinuousUnscentedKalmanFilter &otherFilter_in) = delete;

    /*!
     * @brief           Copy assignment is forbidden; see above.
     */
    ContinuousUnscentedKalmanFilter &operator=(
        const ContinuousUnscentedKalmanFilter &otherFilter_in) = delete;

    /*!
     * @brief           Moving is forbidden; the retained state stays
     *                      with its constructing owner.
     */
    ContinuousUnscentedKalmanFilter(
        ContinuousUnscentedKalmanFilter &&otherFilter_in) = delete;

    /*!
     * @brief           Move assignment is forbidden; see above.
     */
    ContinuousUnscentedKalmanFilter &
        operator=(ContinuousUnscentedKalmanFilter &&otherFilter_in) = delete;

    /* ---------------------------------------------------------------------- *
     * PUBLIC METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Sizes and seeds the filter's state, covariance and
     *                  sigma-point scaling.
     *
     * @param[in]       stateSize_in
     *                  Number of scalar states; must be positive.
     *
     * @param[in]       initialState_in
     *                  Initial state, of length stateSize_in.
     *
     * @param[in]       initialCovariance_in
     *                  Initial state covariance, stateSize_in x
     *                  stateSize_in.
     *
     * @param[in]       configuration_in
     *                  Sigma-point scaling; the composite scaling
     *                  lambda = alpha^2 (n + kappa) - n must keep
     *                  (n + lambda) positive.
     *
     * @return          Lifecycle status.
     *
     * @post            A successful call permits generateSigmaPoints(),
     *                  predictFromSigmaPoints(),
     *                  calculateUpdateNis(), updateFromSigmaPoints(),
     *                  getState(), getCovariance(), setState() and
     *                  terminate() calls.
     */
    [[nodiscard]] UkfStatus
        initialize(Eigen::Index           stateSize_in,
                   const Eigen::VectorXd &initialState_in,
                   const Eigen::MatrixXd &initialCovariance_in,
                   const UkfConfiguration &configuration_in) noexcept;

    /*!
     * @brief           Writes the 2n+1 scaled sigma points of the current
     *                  state and their mean/covariance weights.
     *
     *                  Column 0 is the mean; columns 1..n add and columns
     *                  n+1..2n subtract one Cholesky column each of the
     *                  scaled covariance (n + lambda) P.
     *
     * @param[out]      sigmaPoints_out
     *                  Sigma-point matrix, stateSize x (2 stateSize + 1).
     *
     * @param[out]      weightsMean_out
     *                  Mean recombination weights, length 2 stateSize + 1.
     *
     * @param[out]      weightsCovariance_out
     *                  Covariance recombination weights, length
     *                  2 stateSize + 1.
     *
     * @return          Lifecycle or numerical status; the Cholesky
     *                  factorization fails on a numerically indefinite
     *                  covariance.
     */
    [[nodiscard]] UkfStatus generateSigmaPoints(
        Eigen::MatrixXd &sigmaPoints_out,
        Eigen::VectorXd &weightsMean_out,
        Eigen::VectorXd &weightsCovariance_out) const noexcept;

    /*!
     * @brief           Recombines caller-propagated sigma points into a
     *                  predicted mean and covariance, adding process
     *                  noise.
     *
     * @param[in]       propagatedSigmaPoints_in
     *                  Each sigma point pushed through the caller's
     *                  nonlinear process model over one bounded step,
     *                  stateSize x (2 stateSize + 1).
     *
     * @param[in]       processNoise_in
     *                  Additive discrete process-noise covariance,
     *                  stateSize x stateSize.
     *
     * @return          Lifecycle or numerical status.
     */
    [[nodiscard]] UkfStatus predictFromSigmaPoints(
        const Eigen::MatrixXd &propagatedSigmaPoints_in,
        const Eigen::MatrixXd &processNoise_in) noexcept;

    /*!
     * @brief           Computes the normalized innovation squared of one
     *                  measurement without changing the filter state, so
     *                  the caller can gate the update before applying it.
     *
     *                  The predicted measurement covariance is
     *                  P_yy = sum Wc (Y - y)(Y - y)^T + R over the
     *                  measurement sigma points Y with predicted mean y.
     *
     * @param[in]       predictedMeasurementSigma_in
     *                  Each state sigma point mapped through the
     *                  caller's nonlinear observation model, m x
     *                  (2 stateSize + 1).
     *
     * @param[in]       measurement_in
     *                  Actual measurement, already wrapped/normalized by
     *                  the caller where the observed quantity requires
     *                  it (e.g. an angle); length m.
     *
     * @param[in]       measurementNoise_in
     *                  Measurement-noise covariance, m x m.
     *
     * @param[out]      nis_out
     *                  Normalized innovation squared, written only when
     *                  P_yy factors successfully.
     *
     * @return          Lifecycle or numerical status.
     */
    [[nodiscard]] UkfStatus calculateUpdateNis(
        const Eigen::MatrixXd &predictedMeasurementSigma_in,
        const Eigen::VectorXd &measurement_in,
        const Eigen::MatrixXd &measurementNoise_in,
        double                &nis_out) const noexcept;

    /*!
     * @brief           Applies one unscented measurement correction from
     *                  caller-evaluated measurement sigma points.
     *
     *                  The gain is K = P_xy P_yy^-1 with the
     *                  state-measurement cross-covariance P_xy, solved
     *                  without an explicit inverse.
     *
     * @param[in]       predictedMeasurementSigma_in
     *                  Each state sigma point mapped through the
     *                  caller's nonlinear observation model, m x
     *                  (2 stateSize + 1).
     *
     * @param[in]       stateSigmaPoints_in
     *                  State sigma points the measurement points were
     *                  mapped from, stateSize x (2 stateSize + 1).
     *
     * @param[in]       measurement_in
     *                  Actual measurement, already wrapped/normalized by
     *                  the caller; length m.
     *
     * @param[in]       measurementNoise_in
     *                  Measurement-noise covariance, m x m.
     *
     * @param[out]      nis_out
     *                  Normalized innovation squared, written whenever
     *                  P_yy factored successfully.
     *
     * @return          Lifecycle or numerical status.
     */
    [[nodiscard]] UkfStatus updateFromSigmaPoints(
        const Eigen::MatrixXd &predictedMeasurementSigma_in,
        const Eigen::MatrixXd &stateSigmaPoints_in,
        const Eigen::VectorXd &measurement_in,
        const Eigen::MatrixXd &measurementNoise_in,
        double                &nis_out) noexcept;

    /*!
     * @brief           Clears state and returns to the uninitialized
     *                   lifecycle, freeing the dynamically-sized matrices.
     *
     * @return          Lifecycle status.
     */
    [[nodiscard]] UkfStatus terminate() noexcept;

    /*!
     * @brief           Returns the current state.
     *
     * @return          Current state, empty (size 0) if never initialized.
     */
    [[nodiscard]] const Eigen::VectorXd &getState() const noexcept;

    /*!
     * @brief           Returns the current state covariance.
     *
     * @return          Current covariance, empty if never initialized.
     */
    [[nodiscard]] const Eigen::MatrixXd &getCovariance() const noexcept;

    /*!
     * @brief           Overwrites the current state, e.g. after the caller
     *                  wraps angular components into their canonical
     *                  range.
     *
     * @param[in]       state_in
     *                  Replacement state, length stateSize.
     *
     * @return          Lifecycle or numerical status.
     */
    [[nodiscard]] UkfStatus
        setState(const Eigen::VectorXd &state_in) noexcept;

    /*!
     * @brief           Restores a previously validated state and covariance.
     *
     * @param[in]       state_in
     *                  State checkpoint matching the configured state size.
     *
     * @param[in]       covariance_in
     *                  Finite symmetric PSD covariance checkpoint.
     *
     * @return          Lifecycle, input, or numerical status.
     */
    [[nodiscard]] UkfStatus
        restore(const Eigen::VectorXd &state_in,
                const Eigen::MatrixXd &covariance_in) noexcept;

  private:
    /*!
     * @brief           Checks covariance finiteness, symmetry and PSD.
     *
     * @param[in]       covariance_in
     *                  Candidate square covariance matrix.
     *
     * @return          True when valid within numerical roundoff tolerance.
     */
    static bool
        isCovarianceValid(const Eigen::MatrixXd &covariance_in) noexcept;

    /*!
     * @brief           Writes the scaled unscented-transform weights for
     *                  a state size and composite scaling.
     *
     * @param[in]       stateSize_in
     *                  Number of scalar states; must be positive.
     *
     * @param[in]       lambda_in
     *                  Composite scaling; (stateSize_in + lambda_in)
     *                  must be positive.
     *
     * @param[in]       sigmaAlpha_in
     *                  Primary spread control, for the central
     *                  covariance weight.
     *
     * @param[in]       sigmaBeta_in
     *                  Prior-knowledge weight on the central point.
     *
     * @param[out]      weightsMean_out
     *                  Mean weights, length 2 stateSize_in + 1.
     *
     * @param[out]      weightsCovariance_out
     *                  Covariance weights, length 2 stateSize_in + 1.
     */
    static void calculateSigmaWeights(Eigen::Index   stateSize_in,
                                      double         lambda_in,
                                      double         sigmaAlpha_in,
                                      double         sigmaBeta_in,
                                      Eigen::VectorXd &weightsMean_out,
                                      Eigen::VectorXd
                                          &weightsCovariance_out) noexcept;

    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       Current state estimate.
     */
    Eigen::VectorXd state;

    /*!
     * @brief       Current state covariance.
     */
    Eigen::MatrixXd covariance;

    /*!
     * @brief       Configured number of scalar states; 0 until initialize()
     *              succeeds.
     */
    Eigen::Index stateSize{0};

    /*!
     * @brief       True after a successful initialize() call.
     */
    bool isInitialized{false};

    /*!
     * @brief       Primary sigma-point spread control.
     */
    double sigmaAlpha{1.0e-3};

    /*!
     * @brief       Prior-knowledge weight on the central sigma point.
     */
    double sigmaBeta{2.0};

    /*!
     * @brief       Secondary sigma-point spread control.
     */
    double sigmaKappa{0.0};

    /*!
     * @brief       Composite scaling lambda = alpha^2 (n + kappa) - n.
     */
    double lambdaWeight{0.0};
};

} /* namespace localisation::kalman_filter::ukf_continuous_kalman_filter */

#endif /* SRS_LOCALISATION_UKF_CONTINUOUS_KALMAN_FILTER_CONTINUOUS_UKF_CLASS_H \
        */
