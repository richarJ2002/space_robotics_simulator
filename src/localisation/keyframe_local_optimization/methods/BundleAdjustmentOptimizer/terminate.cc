/*!
 * @File:         terminate.cc
 *
 * @Brief:        Releases the BA solver buffers.
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
/* None */

namespace localisation::keyframe_local_optimization
{

KeyframeLocalOptimizationStatus BundleAdjustmentOptimizer::terminate()
{
    /* Terminating twice is a caller defect, never a silent reset. */
    if (!isInitialized)
    {
        return KeyframeLocalOptimizationStatus::
            KEYFRAME_LOCAL_OPTIMIZATION_NOT_INITIALIZED;
    }

    /* Release the initialize()-sized heap back to the system. */
    freePoseHessian.resize(0, 0);
    freePoseGradient.resize(0);
    poseLandmarkJacobian.resize(0, 0);
    poseLandmarkWorkspace.resize(0, 0);
    poseLandmarkProduct.resize(0, 0);
    landmarkHessians.clear();
    landmarkHessians.shrink_to_fit();
    landmarkGradients.clear();
    landmarkGradients.shrink_to_fit();
    reducedHessian.resize(0, 0);
    reducedGradient.resize(0);
    freePoseStep.resize(0);
    choleskyFactor.resize(0, 0);
    landmarkColumns.clear();
    landmarkColumns.shrink_to_fit();
    isInitialized = false;
    return KeyframeLocalOptimizationStatus::KEYFRAME_LOCAL_OPTIMIZATION_SUCCESS;
}

} /* namespace localisation::keyframe_local_optimization */
