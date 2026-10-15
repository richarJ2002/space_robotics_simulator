/*!
 * @File:         computeTotalError.cc
 *
 * @Brief:        Sums the robust 3-D residual error of the window.
 *
 * @Date:         08/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/KeyframeWindowClass.h"

/* External Library Includes */
/* None */

/* Other Project Module Includes */
/* None */

/* Generic Libraries */
#include <cmath>

namespace localisation::keyframe_local_optimization
{

KeyframeLocalOptimizationStatus KeyframeWindow::computeTotalError(
    const BundleAdjustmentConfiguration &configuration_in,
    double                              &totalError_out) const
{
    /* Uninitialized use is a caller defect, never a silent zero. */
    if (!isInitialized)
    {
        return KeyframeLocalOptimizationStatus::
            KEYFRAME_LOCAL_OPTIMIZATION_NOT_INITIALIZED;
    }

    /* Sum the Huber-weighted squared 3-D residuals over every stored
     * observation; the optimizer drives this same cost down. */
    double      accumulatedError_m2 = 0.0;
    std::size_t observationTotal    = 0U;
    for (std::size_t keyframeOffset = 0U; keyframeOffset < storedKeyframeCount;
         keyframeOffset++)
    {
        const std::size_t ringIndex =
            (ringStartIndex + keyframeOffset) % MAX_WINDOW_KEYFRAMES;
        const StoredKeyframe &storedKeyframe = keyframeRing[ringIndex];
        const Eigen::Matrix3d keyframeRotation =
            storedKeyframe.orientation_fixedToBody.toRotationMatrix();
        const Eigen::Matrix3d worldFromLocalRotation =
            keyframeRotation.transpose();
        for (std::size_t observationIndex = 0U;
             observationIndex < storedKeyframe.observationCount;
             observationIndex++)
        {
            const std::size_t landmarkIndex =
                storedKeyframe.landmarkIndices[observationIndex];
            const Eigen::Vector3d predictedLocalPoint =
                worldFromLocalRotation * (landmarkPositions[landmarkIndex] -
                                          storedKeyframe.position_fixed_m);
            const Eigen::Vector3d residual_m =
                predictedLocalPoint -
                storedKeyframe.localPoints[observationIndex];
            const double residualNorm_m = residual_m.norm();
            double       weight         = 1.0;
            if (configuration_in.useHuber &&
                (residualNorm_m > configuration_in.huberDelta_m))
            {
                weight = configuration_in.huberDelta_m / residualNorm_m;
            }
            accumulatedError_m2 += weight * residual_m.squaredNorm();
            observationTotal++;
        }
    }

    /* No observation means no constraint to score. */
    if (observationTotal == 0U)
    {
        return KeyframeLocalOptimizationStatus::
            KEYFRAME_LOCAL_OPTIMIZATION_INSUFFICIENT_DATA;
    }
    totalError_out = accumulatedError_m2;
    return KeyframeLocalOptimizationStatus::KEYFRAME_LOCAL_OPTIMIZATION_SUCCESS;
}

} /* namespace localisation::keyframe_local_optimization */
