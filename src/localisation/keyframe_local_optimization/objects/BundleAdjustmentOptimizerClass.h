/*!
 * @File:         BundleAdjustmentOptimizerClass.h
 *
 * @Brief:        Declares the Eigen-only Full BA solver.
 *
 * @Date:         08/10/2026
 *
 */

#ifndef SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_OPTIMIZER_CLASS_H
#define SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_OPTIMIZER_CLASS_H

/* Object Includes */
#include "objects/BundleAdjustmentConfigurationStruct.h"
#include "objects/KeyframeLocalOptimizationLimits.h"
#include "objects/KeyframeLocalOptimizationStatusEnum.h"
#include "objects/KeyframeWindowClass.h"

/* Data include */
/* None */

/* Generic Libraries */
#include <array>
#include <cstddef>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace localisation::keyframe_local_optimization
{

/*!
 * @brief           Eigen-only Full bundle adjustment (poses +
 *                  landmarks).
 *
 *                  Gauss-Newton over the window's free keyframe poses
 *                  (6 parameters each; the first keyframe anchors the
 *                  datum) and landmark positions, minimizing the
 *                  Huber-weighted squared 3-D point residuals with a
 *                  weak diagonal pose prior (datum keyframe held
 *                  fixed, unobserved poses regularized). Landmarks are
 *                  marginalized with a Schur-complement reduced camera system
 *                  (at most 90 unknowns), solved with an in-place Cholesky on a
 *                  member buffer. All heap is sized in initialize() only: the
 *                  member buffers below are resized there to their compile-time
 *                  caps and optimize() performs only indexed writes into them,
 *                  never a resize, reserve, or allocation (any future deviation
 *                  needs an explicit recorded deviation with bound and
 *                  verification). Construction never fails; initialize()
 *                  validates the solver tuning, and every fallible call returns
 *                  a checked KeyframeLocalOptimizationStatus. Not thread-safe:
 *                  the owning node calls it from one callback group only.
 */
class BundleAdjustmentOptimizer
{
  public:
    /*!
     * @brief           Establishes a valid empty uninitialized state.
     */
    BundleAdjustmentOptimizer() = default;

    /*!
     * @brief           Prevents constructing a new BundleAdjustmentOptimizer by
     *                  copying another object.
     *
     * @param[in]       otherOptimizer_in
     *                  Source optimizer that would otherwise be copied.
     */
    BundleAdjustmentOptimizer(
        const BundleAdjustmentOptimizer &otherOptimizer_in) = delete;

    /*!
     * @brief           Prevents copying another optimizer into this existing
     *                  BundleAdjustmentOptimizer.
     *
     * @param[in]       otherOptimizer_in
     *                  Source optimizer that would otherwise be copied.
     */
    BundleAdjustmentOptimizer &
        operator=(const BundleAdjustmentOptimizer &otherOptimizer_in) = delete;

    /*!
     * @brief           Prevents constructing a new BundleAdjustmentOptimizer by
     *                  moving resources from another object.
     *
     * @param[in]       otherOptimizer_in
     *                  Source optimizer whose resources would otherwise be
     *                  moved. This deleted operation cannot modify the source.
     */
    BundleAdjustmentOptimizer(BundleAdjustmentOptimizer &&otherOptimizer_in) =
        delete;

    /*!
     * @brief           Prevents moving resources from another optimizer into
     *                  this existing BundleAdjustmentOptimizer.
     *
     * @param[in]       otherOptimizer_in
     *                  Source optimizer whose resources would otherwise be
     *                  moved. This deleted operation cannot modify the source.
     */
    BundleAdjustmentOptimizer &
        operator=(BundleAdjustmentOptimizer &&otherOptimizer_in) = delete;

    /*!
     * @brief           Validates the solver tuning and sizes the heap.
     *
     * @param[in]       configuration_in
     *                  Solver tuning; maximumIterations must be at
     *                  least 1 and huberDelta_m finite and positive
     *                  when useHuber is set.
     *
     * @return          SUCCESS, or INVALID_CONFIGURATION (state left
     *                  uninitialized).
     */
    [[nodiscard]] KeyframeLocalOptimizationStatus
        initialize(const BundleAdjustmentConfiguration &configuration_in);

    /*!
     * @brief           Releases all sized buffers.
     *
     * @return          SUCCESS, or NOT_INITIALIZED.
     */
    [[nodiscard]] KeyframeLocalOptimizationStatus terminate();

    /*!
     * @brief           Refines the window's poses and landmarks.
     *
     *                  Runs at most maximumIterations Gauss-Newton
     *                  steps, keeping an update only while the robust
     *                  cost decreases. A window left unchanged still
     *                  reports SUCCESS when its cost was already
     *                  minimal.
     *
     * @param[in,out]   window_inout
     *                  Window to refine in place (must be
     *                  initialized).
     *
     * @return          SUCCESS, NOT_INITIALIZED,
     *                  INSUFFICIENT_DATA (fewer than two keyframes or
     *                  no observation), or INVALID_CONFIGURATION when
     *                  this optimizer was never initialized.
     */
    [[nodiscard]] KeyframeLocalOptimizationStatus
        optimize(KeyframeWindow &window_inout);

  private:
    /*!
     * @brief       Weak diagonal pose prior weight.
     *
     *              Regularizes unobserved free poses only; the datum
     *              keyframe (slot 0, held fixed) anchors the solution,
     *              so the prior stays negligible beside real
     *              observations. Input covariances are carried for
     *              republication, not used as solver weights.
     */
    static constexpr double WEAK_POSE_PRIOR_WEIGHT = 1.0e-6;

    /*!
     * @brief       True once initialize() accepted a configuration.
     */
    bool isInitialized{false};

    /*!
     * @brief       Accepted solver tuning.
     */
    BundleAdjustmentConfiguration configuration{};

    /*!
     * @brief       Free-pose Hessian accumulator (90 x 90 at most).
     */
    Eigen::MatrixXd freePoseHessian{};

    /*!
     * @brief       Free-pose gradient accumulator (90 at most).
     */
    Eigen::VectorXd freePoseGradient{};

    /*!
     * @brief       Pose-landmark Jacobian accumulator (90 x 3L).
     */
    Eigen::MatrixXd poseLandmarkJacobian{};

    /*!
     * @brief       One landmark's pose block workspace (90 x 3).
     */
    Eigen::MatrixXd poseLandmarkWorkspace{};

    /*!
     * @brief       Pose-block-times-inverse workspace (90 x 3).
     */
    Eigen::MatrixXd poseLandmarkProduct{};

    /*!
     * @brief       Per-landmark 3x3 Hessians, one per table slot.
     */
    std::vector<Eigen::Matrix3d> landmarkHessians{};

    /*!
     * @brief       Per-landmark 3-gradients, one per table slot.
     */
    std::vector<Eigen::Vector3d> landmarkGradients{};

    /*!
     * @brief       Reduced camera-system Hessian (90 x 90 at most).
     */
    Eigen::MatrixXd reducedHessian{};

    /*!
     * @brief       Reduced camera-system gradient (90 at most).
     */
    Eigen::VectorXd reducedGradient{};

    /*!
     * @brief       Solved free-pose step (90 at most).
     */
    Eigen::VectorXd freePoseStep{};

    /*!
     * @brief       In-place Cholesky factor workspace (90 x 90).
     */
    Eigen::MatrixXd choleskyFactor{};

    /*!
     * @brief       Compact column per landmark table slot, or
     *              MAX_LANDMARKS when the landmark is inactive.
     */
    std::vector<std::size_t> landmarkColumns{};

    /*!
     * @brief       Input positions anchoring the pose prior, by window
     *              slot (fixed storage, no heap).
     */
    std::array<Eigen::Vector3d, MAX_WINDOW_KEYFRAMES> priorPositions{};

    /*!
     * @brief       Input orientations anchoring the pose prior, by
     *              window slot (fixed storage, no heap).
     */
    std::array<Eigen::Quaterniond, MAX_WINDOW_KEYFRAMES> priorOrientations{};
};

} /* namespace localisation::keyframe_local_optimization */

#endif /* SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_OPTIMIZER_CLASS_H */
