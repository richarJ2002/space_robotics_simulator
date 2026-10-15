/*!
 * @File:         initialize.cc
 *
 * @Brief:        Validates the BA tuning and sizes the heap.
 *
 * @Date:         08/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/BundleAdjustmentOptimizerClass.h"

/* External Library Includes */
/* None */

/* Other Project Module Includes */
/* None */

/* Generic Libraries */
#include <cmath>

namespace localisation::keyframe_local_optimization
{

KeyframeLocalOptimizationStatus BundleAdjustmentOptimizer::initialize(
    const BundleAdjustmentConfiguration &configuration_in)
{
    /* Zero iterations optimize nothing; a non-positive Huber delta
     * divides by zero in the robust weight. */
    const bool isIterationValid = configuration_in.maximumIterations >= 1U;
    const bool isHuberValid     = !configuration_in.useHuber ||
                              (std::isfinite(configuration_in.huberDelta_m) &&
                               (configuration_in.huberDelta_m > 0.0));
    if (!isIterationValid || !isHuberValid)
    {
        return KeyframeLocalOptimizationStatus::
            KEYFRAME_LOCAL_OPTIMIZATION_INVALID_CONFIGURATION;
    }

    /* Sole heap sizing: every buffer below is dimensioned to its
     * compile-time cap here, so optimize() performs only indexed
     * writes and never resizes, reserves, or allocates. */
    configuration = configuration_in;
    freePoseHessian.resize(MAX_FREE_POSE_PARAMETERS, MAX_FREE_POSE_PARAMETERS);
    freePoseGradient.resize(MAX_FREE_POSE_PARAMETERS);
    poseLandmarkJacobian.resize(MAX_FREE_POSE_PARAMETERS, 3U * MAX_LANDMARKS);
    poseLandmarkWorkspace.resize(MAX_FREE_POSE_PARAMETERS, 3U);
    poseLandmarkProduct.resize(MAX_FREE_POSE_PARAMETERS, 3U);
    landmarkHessians.resize(MAX_LANDMARKS);
    landmarkGradients.resize(MAX_LANDMARKS);
    reducedHessian.resize(MAX_FREE_POSE_PARAMETERS, MAX_FREE_POSE_PARAMETERS);
    reducedGradient.resize(MAX_FREE_POSE_PARAMETERS);
    freePoseStep.resize(MAX_FREE_POSE_PARAMETERS);
    choleskyFactor.resize(MAX_FREE_POSE_PARAMETERS, MAX_FREE_POSE_PARAMETERS);
    landmarkColumns.resize(MAX_LANDMARKS);
    isInitialized = true;
    return KeyframeLocalOptimizationStatus::KEYFRAME_LOCAL_OPTIMIZATION_SUCCESS;
}

} /* namespace localisation::keyframe_local_optimization */
