/*!
 * @File:         optimize.cc
 *
 * @Brief:        Refines window poses and landmarks with Full BA.
 *
 * @Date:         08/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/BundleAdjustmentOptimizerClass.h"

/* Function Includes */
#include "private_functions/private_functions.h"

/* External Library Includes */
/* None */

/* Other Project Module Includes */
/* None */

/* Generic Libraries */
/* None */

namespace localisation::keyframe_local_optimization
{

KeyframeLocalOptimizationStatus
    BundleAdjustmentOptimizer::optimize(KeyframeWindow &window_inout)
{
    /* Uninitialized use is a caller defect, never a silent no-op. */
    if (!isInitialized)
    {
        return KeyframeLocalOptimizationStatus::
            KEYFRAME_LOCAL_OPTIMIZATION_NOT_INITIALIZED;
    }
    if (!window_inout.isInitialized)
    {
        return KeyframeLocalOptimizationStatus::
            KEYFRAME_LOCAL_OPTIMIZATION_INVALID_CONFIGURATION;
    }

    /* Fewer than two keyframes leave the relative geometry empty. */
    const std::size_t storedWindowSize = window_inout.storedKeyframeCount;
    if (storedWindowSize < 2U)
    {
        return KeyframeLocalOptimizationStatus::
            KEYFRAME_LOCAL_OPTIMIZATION_INSUFFICIENT_DATA;
    }

    /* Map each live landmark table slot to a compact column used by
     * the accumulators below; inactive slots keep the sentinel. */
    std::size_t activeLandmarkCount    = 0U;
    std::size_t storedObservationTotal = 0U;
    for (std::size_t tableIndex = 0U;
         tableIndex < window_inout.landmarkHighWaterMark;
         tableIndex++)
    {
        if (window_inout.landmarkObservationCounts[tableIndex] > 0U)
        {
            landmarkColumns[tableIndex] = activeLandmarkCount;
            activeLandmarkCount++;
            storedObservationTotal +=
                window_inout.landmarkObservationCounts[tableIndex];
        }
        else
        {
            landmarkColumns[tableIndex] = MAX_LANDMARKS;
        }
    }
    if ((activeLandmarkCount == 0U) || (storedObservationTotal == 0U))
    {
        return KeyframeLocalOptimizationStatus::
            KEYFRAME_LOCAL_OPTIMIZATION_INSUFFICIENT_DATA;
    }

    /*!
     * Anchor the diagonal-covariance pose prior at the input poses so weakly
     * observed keyframes stay regularized to their odometry.
     */
    for (std::size_t windowSlot = 0U; windowSlot < storedWindowSize;
         windowSlot++)
    {
        const std::size_t ringIndex =
            (window_inout.ringStartIndex + windowSlot) % MAX_WINDOW_KEYFRAMES;
        priorPositions[windowSlot] =
            window_inout.keyframeRing[ringIndex].position_fixed_m;
        priorOrientations[windowSlot] =
            window_inout.keyframeRing[ringIndex].orientation_fixedToBody;
    }

    /* Baseline robust cost; updates are kept only while it falls. */
    double bestCost_m2 = 0.0;
    {
        double                                baselineCost_m2 = 0.0;
        const KeyframeLocalOptimizationStatus baselineStatus =
            window_inout.computeTotalError(configuration, baselineCost_m2);
        if (baselineStatus != KeyframeLocalOptimizationStatus::
                                  KEYFRAME_LOCAL_OPTIMIZATION_SUCCESS)
        {
            return baselineStatus;
        }
        bestCost_m2 = baselineCost_m2;
    }

    /* Free poses exclude the datum keyframe: F = 6 (W - 1) <= 90. */
    const std::size_t  freePoseCount = 6U * (storedWindowSize - 1U);
    const Eigen::Index freePoseEnd   = static_cast<Eigen::Index>(freePoseCount);

    for (std::size_t iterationIndex = 0U;
         iterationIndex < configuration.maximumIterations;
         iterationIndex++)
    {
        /* Zero only the active blocks of the sized buffers. */
        freePoseHessian.topLeftCorner(freePoseEnd, freePoseEnd).setZero();
        freePoseGradient.head(freePoseEnd).setZero();
        poseLandmarkJacobian
            .topLeftCorner(freePoseEnd,
                           static_cast<Eigen::Index>(3U * activeLandmarkCount))
            .setZero();
        for (std::size_t tableIndex = 0U;
             tableIndex < window_inout.landmarkHighWaterMark;
             tableIndex++)
        {
            if (landmarkColumns[tableIndex] < MAX_LANDMARKS)
            {
                landmarkHessians[tableIndex].setZero();
                landmarkGradients[tableIndex].setZero();
            }
        }

        /*!
         * Linearize every stored observation. Residual r = R^T (L - p) - z;
         * Jacobians dr/dp = -R^T,
         * dr/dphi = skew(R^T (L - p)), dr/dL = R^T.
         */
        for (std::size_t windowSlot = 0U; windowSlot < storedWindowSize;
             windowSlot++)
        {
            const std::size_t ringIndex =
                (window_inout.ringStartIndex + windowSlot) %
                MAX_WINDOW_KEYFRAMES;
            const KeyframeWindow::StoredKeyframe &storedKeyframe =
                window_inout.keyframeRing[ringIndex];
            const Eigen::Matrix3d keyframeRotation =
                storedKeyframe.orientation_fixedToBody.toRotationMatrix();
            const Eigen::Matrix3d localFromWorldRotation =
                keyframeRotation.transpose();
            for (std::size_t observationIndex = 0U;
                 observationIndex < storedKeyframe.observationCount;
                 observationIndex++)
            {
                const std::size_t tableIndex =
                    storedKeyframe.landmarkIndices[observationIndex];
                const std::size_t compactColumn = landmarkColumns[tableIndex];
                if (compactColumn >= MAX_LANDMARKS)
                {
                    continue;
                }
                const Eigen::Vector3d predictedLocalPoint =
                    localFromWorldRotation *
                    (window_inout.landmarkPositions[tableIndex] -
                     storedKeyframe.position_fixed_m);
                const Eigen::Vector3d residual_m =
                    predictedLocalPoint -
                    storedKeyframe.localPoints[observationIndex];
                const double residualNorm_m = residual_m.norm();
                double       robustWeight   = 1.0;
                if (configuration.useHuber &&
                    (residualNorm_m > configuration.huberDelta_m))
                {
                    robustWeight = configuration.huberDelta_m / residualNorm_m;
                }
                const Eigen::Matrix3d positionJacobian =
                    -localFromWorldRotation;
                const Eigen::Matrix3d rotationJacobian =
                    skewOfVector(predictedLocalPoint);
                const Eigen::Matrix3d landmarkJacobian = localFromWorldRotation;
                const Eigen::Vector3d weightedResidual_m =
                    robustWeight * residual_m;

                /* Landmark blocks always accumulate, datum included. */
                landmarkHessians[tableIndex].noalias() +=
                    robustWeight * landmarkJacobian.transpose() *
                    landmarkJacobian;
                landmarkGradients[tableIndex].noalias() -=
                    landmarkJacobian.transpose() * weightedResidual_m;

                /* Pose blocks skip the datum keyframe (slot 0). */
                if (windowSlot > 0U)
                {
                    const Eigen::Index poseOffset =
                        static_cast<Eigen::Index>(6U * (windowSlot - 1U));
                    /* Stacked transposed Jacobian (6 x 3): the
                     * position/rotation blocks enter transposed, so
                     * the products below read J^T J and J^T r. */
                    Eigen::Matrix<double, 6, 3> poseJacobianTranspose;
                    poseJacobianTranspose.topRows(3) =
                        positionJacobian.transpose();
                    poseJacobianTranspose.bottomRows(3) =
                        rotationJacobian.transpose();
                    freePoseHessian.block(poseOffset, poseOffset, 6, 6)
                        .noalias() += robustWeight * poseJacobianTranspose *
                                      poseJacobianTranspose.transpose();
                    freePoseGradient.segment(poseOffset, 6).noalias() -=
                        poseJacobianTranspose * weightedResidual_m;
                    poseLandmarkJacobian
                        .block(poseOffset,
                               static_cast<Eigen::Index>(3U * compactColumn),
                               6,
                               3)
                        .noalias() +=
                        robustWeight * poseJacobianTranspose * landmarkJacobian;
                }
            }
        }

        /*!
         * Weak diagonal pose prior keeps unobserved keyframes regularized to
         * their input odometry without biasing observed ones (datum slot 0 is
         * held fixed instead).
         */
        for (std::size_t windowSlot = 1U; windowSlot < storedWindowSize;
             windowSlot++)
        {
            const std::size_t ringIndex =
                (window_inout.ringStartIndex + windowSlot) %
                MAX_WINDOW_KEYFRAMES;
            const KeyframeWindow::StoredKeyframe &storedKeyframe =
                window_inout.keyframeRing[ringIndex];
            const Eigen::Index poseOffset =
                static_cast<Eigen::Index>(6U * (windowSlot - 1U));
            const Eigen::Vector3d priorPositionError_m =
                storedKeyframe.position_fixed_m - priorPositions[windowSlot];
            const Eigen::Vector3d priorRotationError_rad =
                quaternionToRotationVector(
                    priorOrientations[windowSlot].conjugate() *
                    storedKeyframe.orientation_fixedToBody);
            for (std::size_t diagonalIndex = 0U; diagonalIndex < 6U;
                 diagonalIndex++)
            {
                const Eigen::Index diagonalOffset =
                    poseOffset + static_cast<Eigen::Index>(diagonalIndex);
                freePoseHessian(diagonalOffset, diagonalOffset) +=
                    WEAK_POSE_PRIOR_WEIGHT;
                const double priorErrorComponent =
                    (diagonalIndex < 3U)
                        ? priorPositionError_m(
                              static_cast<Eigen::Index>(diagonalIndex))
                        : priorRotationError_rad(
                              static_cast<Eigen::Index>(diagonalIndex - 3U));
                freePoseGradient(diagonalOffset) -=
                    WEAK_POSE_PRIOR_WEIGHT * priorErrorComponent;
            }
        }

        /* Schur reduction over landmarks into the camera system. */
        reducedHessian.topLeftCorner(freePoseEnd, freePoseEnd) =
            freePoseHessian.topLeftCorner(freePoseEnd, freePoseEnd);
        reducedGradient.head(freePoseEnd) = freePoseGradient.head(freePoseEnd);
        bool isReductionValid             = true;
        for (std::size_t tableIndex = 0U;
             tableIndex < window_inout.landmarkHighWaterMark;
             tableIndex++)
        {
            const std::size_t compactColumn = landmarkColumns[tableIndex];
            if (compactColumn >= MAX_LANDMARKS)
            {
                continue;
            }
            Eigen::Matrix3d landmarkInverse;
            if (!invertDampedSymmetric3x3(landmarkHessians[tableIndex],
                                          1.0e-9,
                                          landmarkInverse))
            {
                isReductionValid = false;
                break;
            }
            landmarkHessians[tableIndex] = landmarkInverse;
            /* Copy the landmark's pose block into the member
             * workspace (no allocation), then reduce through a
             * second workspace so no product temporary is needed. */
            poseLandmarkWorkspace.topLeftCorner(freePoseEnd, 3) =
                poseLandmarkJacobian.block(
                    0,
                    static_cast<Eigen::Index>(3U * compactColumn),
                    freePoseEnd,
                    3);
            poseLandmarkProduct.topLeftCorner(freePoseEnd, 3).noalias() =
                poseLandmarkWorkspace.topLeftCorner(freePoseEnd, 3) *
                landmarkInverse;
            reducedHessian.topLeftCorner(freePoseEnd, freePoseEnd).noalias() -=
                poseLandmarkProduct.topLeftCorner(freePoseEnd, 3) *
                poseLandmarkWorkspace.topLeftCorner(freePoseEnd, 3).transpose();
            reducedGradient.head(freePoseEnd).noalias() -=
                poseLandmarkProduct.topLeftCorner(freePoseEnd, 3) *
                landmarkGradients[tableIndex];
        }
        if (!isReductionValid)
        {
            break;
        }

        /* Solve the reduced system with diagonal damping fallback. */
        choleskyFactor.topLeftCorner(freePoseEnd, freePoseEnd) =
            reducedHessian.topLeftCorner(freePoseEnd, freePoseEnd);
        if (!factorizeCholesky(choleskyFactor, freePoseCount))
        {
            for (std::size_t diagonalIndex = 0U; diagonalIndex < freePoseCount;
                 diagonalIndex++)
            {
                choleskyFactor(static_cast<Eigen::Index>(diagonalIndex),
                               static_cast<Eigen::Index>(diagonalIndex)) +=
                    1.0e-6;
            }
            if (!factorizeCholesky(choleskyFactor, freePoseCount))
            {
                break;
            }
        }
        solveCholesky(choleskyFactor,
                      reducedGradient,
                      freePoseStep,
                      freePoseCount);

        /* Backtracking line search on the robust cost: halve the
         * whole Newton step until it improves, else stop. */
        double stepScale      = 1.0;
        bool   isStepAccepted = false;
        for (std::size_t attemptIndex = 0U; attemptIndex < 6U; attemptIndex++)
        {
            /* Apply the scaled pose step, datum keyframe untouched. */
            for (std::size_t windowSlot = 1U; windowSlot < storedWindowSize;
                 windowSlot++)
            {
                const std::size_t ringIndex =
                    (window_inout.ringStartIndex + windowSlot) %
                    MAX_WINDOW_KEYFRAMES;
                KeyframeWindow::StoredKeyframe &storedKeyframe =
                    window_inout.keyframeRing[ringIndex];
                const Eigen::Index poseOffset =
                    static_cast<Eigen::Index>(6U * (windowSlot - 1U));
                const Eigen::Vector3d scaledPositionStep_m =
                    freePoseStep.segment(poseOffset, 3) * stepScale;
                const Eigen::Vector3d scaledRotationStep_rad =
                    freePoseStep.segment(poseOffset + 3, 3) * stepScale;
                storedKeyframe.position_fixed_m += scaledPositionStep_m;
                storedKeyframe.orientation_fixedToBody =
                    (storedKeyframe.orientation_fixedToBody *
                     rotationVectorToQuaternion(scaledRotationStep_rad))
                        .normalized();
            }

            /* Apply the scaled landmark steps (dx_l = S (bl - G^T
             * dx_p), scaled by the same factor). */
            for (std::size_t tableIndex = 0U;
                 tableIndex < window_inout.landmarkHighWaterMark;
                 tableIndex++)
            {
                const std::size_t compactColumn = landmarkColumns[tableIndex];
                if (compactColumn >= MAX_LANDMARKS)
                {
                    continue;
                }
                /* Copy the pose block into the member workspace
                 * (no allocation); the 3-vector products below stay
                 * on the stack. */
                poseLandmarkWorkspace.topLeftCorner(freePoseEnd, 3) =
                    poseLandmarkJacobian.block(
                        0,
                        static_cast<Eigen::Index>(3U * compactColumn),
                        freePoseEnd,
                        3);
                const Eigen::Vector3d landmarkStep_m =
                    landmarkHessians[tableIndex] *
                    (landmarkGradients[tableIndex] -
                     poseLandmarkWorkspace.topLeftCorner(freePoseEnd, 3)
                             .transpose() *
                         freePoseStep.head(freePoseEnd));
                window_inout.landmarkPositions[tableIndex] +=
                    stepScale * landmarkStep_m;
            }

            /* Keep the scaled step only while the robust cost falls. */
            double                                candidateCost_m2 = 0.0;
            const KeyframeLocalOptimizationStatus candidateStatus =
                window_inout.computeTotalError(configuration, candidateCost_m2);
            const bool isCostImproved =
                (candidateStatus == KeyframeLocalOptimizationStatus::
                                        KEYFRAME_LOCAL_OPTIMIZATION_SUCCESS) &&
                (candidateCost_m2 < bestCost_m2);
            if (isCostImproved)
            {
                bestCost_m2    = candidateCost_m2;
                isStepAccepted = true;
                break;
            }

            /* Revert the scaled step exactly, then try a half step. */
            for (std::size_t windowSlot = 1U; windowSlot < storedWindowSize;
                 windowSlot++)
            {
                const std::size_t ringIndex =
                    (window_inout.ringStartIndex + windowSlot) %
                    MAX_WINDOW_KEYFRAMES;
                KeyframeWindow::StoredKeyframe &storedKeyframe =
                    window_inout.keyframeRing[ringIndex];
                const Eigen::Index poseOffset =
                    static_cast<Eigen::Index>(6U * (windowSlot - 1U));
                const Eigen::Vector3d scaledPositionStep_m =
                    freePoseStep.segment(poseOffset, 3) * stepScale;
                const Eigen::Vector3d scaledRotationStep_rad =
                    freePoseStep.segment(poseOffset + 3, 3) * stepScale;
                storedKeyframe.position_fixed_m -= scaledPositionStep_m;
                storedKeyframe.orientation_fixedToBody =
                    (storedKeyframe.orientation_fixedToBody *
                     rotationVectorToQuaternion(-scaledRotationStep_rad))
                        .normalized();
            }
            for (std::size_t tableIndex = 0U;
                 tableIndex < window_inout.landmarkHighWaterMark;
                 tableIndex++)
            {
                const std::size_t compactColumn = landmarkColumns[tableIndex];
                if (compactColumn >= MAX_LANDMARKS)
                {
                    continue;
                }
                /* Copy the pose block into the member workspace
                 * (no allocation); the 3-vector products below stay
                 * on the stack. */
                poseLandmarkWorkspace.topLeftCorner(freePoseEnd, 3) =
                    poseLandmarkJacobian.block(
                        0,
                        static_cast<Eigen::Index>(3U * compactColumn),
                        freePoseEnd,
                        3);
                const Eigen::Vector3d landmarkStep_m =
                    landmarkHessians[tableIndex] *
                    (landmarkGradients[tableIndex] -
                     poseLandmarkWorkspace.topLeftCorner(freePoseEnd, 3)
                             .transpose() *
                         freePoseStep.head(freePoseEnd));
                window_inout.landmarkPositions[tableIndex] -=
                    stepScale * landmarkStep_m;
            }
            stepScale *= 0.5;
        }
        if (!isStepAccepted)
        {
            break;
        }
    }

    return KeyframeLocalOptimizationStatus::KEYFRAME_LOCAL_OPTIMIZATION_SUCCESS;
}

} /* namespace localisation::keyframe_local_optimization */
