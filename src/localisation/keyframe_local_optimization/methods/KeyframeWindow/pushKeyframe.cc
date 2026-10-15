/*!
 * @File:         pushKeyframe.cc
 *
 * @Brief:        Inserts one keyframe with landmark association.
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
#include <limits>

namespace localisation::keyframe_local_optimization
{

KeyframeLocalOptimizationStatus
    KeyframeWindow::pushKeyframe(const KeyframeSample &sample_in)
{
    /* Uninitialized use is a caller defect, never silent buffering. */
    if (!isInitialized)
    {
        return KeyframeLocalOptimizationStatus::
            KEYFRAME_LOCAL_OPTIMIZATION_NOT_INITIALIZED;
    }

    /* A stale input marks the pose unavailable upstream (VO
     * KeyframeAction::MARK_POSE_UNAVAILABLE semantics): it is never
     * silently coasted into the window. A non-finite stamp,
     * position, or orientation would seed NaN landmarks and freeze
     * the solver, so each is rejected the same way. */
    if (!sample_in.isPoseAvailable || !std::isfinite(sample_in.stamp_s) ||
        !std::isfinite(sample_in.position_fixed_m.x()) ||
        !std::isfinite(sample_in.position_fixed_m.y()) ||
        !std::isfinite(sample_in.position_fixed_m.z()) ||
        !std::isfinite(sample_in.orientation_fixedToBody.w()) ||
        !std::isfinite(sample_in.orientation_fixedToBody.x()) ||
        !std::isfinite(sample_in.orientation_fixedToBody.y()) ||
        !std::isfinite(sample_in.orientation_fixedToBody.z()))
    {
        return KeyframeLocalOptimizationStatus::
            KEYFRAME_LOCAL_OPTIMIZATION_INSUFFICIENT_DATA;
    }

    /* Overflow drops the oldest keyframe first, recycling landmarks
     * it alone observed back onto the free-list. */
    bool hasOverflowed = false;
    if (storedKeyframeCount >= configuration.windowSize)
    {
        const StoredKeyframe &oldestKeyframe = keyframeRing[ringStartIndex];
        for (std::size_t observationIndex = 0U;
             observationIndex < oldestKeyframe.observationCount;
             observationIndex++)
        {
            const std::size_t landmarkIndex =
                oldestKeyframe.landmarkIndices[observationIndex];
            if (landmarkObservationCounts[landmarkIndex] > 0U)
            {
                landmarkObservationCounts[landmarkIndex]--;
                if (landmarkObservationCounts[landmarkIndex] == 0U)
                {
                    freeLandmarkIndices[freeLandmarkCount] = landmarkIndex;
                    freeLandmarkCount++;
                }
            }
        }
        ringStartIndex = (ringStartIndex + 1U) % MAX_WINDOW_KEYFRAMES;
        storedKeyframeCount--;
        hasOverflowed = true;
    }

    /* Claim the next ring slot for the incoming keyframe. */
    const std::size_t ringIndex =
        (ringStartIndex + storedKeyframeCount) % MAX_WINDOW_KEYFRAMES;
    StoredKeyframe &storedKeyframe         = keyframeRing[ringIndex];
    storedKeyframe.stamp_s                 = sample_in.stamp_s;
    storedKeyframe.position_fixed_m        = sample_in.position_fixed_m;
    storedKeyframe.orientation_fixedToBody = sample_in.orientation_fixedToBody;
    storedKeyframe.poseCovariance          = sample_in.poseCovariance;
    storedKeyframe.twistLinear_body_mPs    = sample_in.twistLinear_body_mPs;
    storedKeyframe.twistAngular_body_radPs = sample_in.twistAngular_body_radPs;
    storedKeyframe.observationCount        = 0U;

    /* Rotation of the incoming pose, used to lift local points into
     * the odom frame for association below. */
    const Eigen::Matrix3d keyframeRotation =
        sample_in.orientation_fixedToBody.toRotationMatrix();

    /* Associate each local point to the nearest world-frame landmark
     * inside the gate, else seed a new landmark while the table has
     * room; points beyond a full table are ignored. */
    const double gateSquared_m2 =
        configuration.associationGate_m * configuration.associationGate_m;
    std::size_t pointLimit = sample_in.localPointCount;
    if (pointLimit > MAX_OBSERVATIONS_PER_KEYFRAME)
    {
        pointLimit = MAX_OBSERVATIONS_PER_KEYFRAME;
    }
    for (std::size_t pointIndex = 0U; pointIndex < pointLimit; pointIndex++)
    {
        const Eigen::Vector3d &localPoint = sample_in.localPoints[pointIndex];
        if (!std::isfinite(localPoint.x()) || !std::isfinite(localPoint.y()) ||
            !std::isfinite(localPoint.z()))
        {
            continue;
        }
        const Eigen::Vector3d worldPoint =
            keyframeRotation * localPoint + sample_in.position_fixed_m;

        /* Nearest live landmark by world-frame distance. */
        std::size_t bestLandmarkIndex = MAX_LANDMARKS;
        double bestDistanceSquared_m2 = std::numeric_limits<double>::infinity();
        for (std::size_t landmarkIndex = 0U;
             landmarkIndex < landmarkHighWaterMark;
             landmarkIndex++)
        {
            if (landmarkObservationCounts[landmarkIndex] == 0U)
            {
                continue;
            }
            const double candidateDistanceSquared_m2 =
                (landmarkPositions[landmarkIndex] - worldPoint).squaredNorm();
            if (candidateDistanceSquared_m2 < bestDistanceSquared_m2)
            {
                bestDistanceSquared_m2 = candidateDistanceSquared_m2;
                bestLandmarkIndex      = landmarkIndex;
            }
        }

        /* Join the nearest landmark inside the gate, else seed new. */
        std::size_t assignedLandmarkIndex = MAX_LANDMARKS;
        if ((bestLandmarkIndex < MAX_LANDMARKS) &&
            (bestDistanceSquared_m2 <= gateSquared_m2))
        {
            assignedLandmarkIndex = bestLandmarkIndex;
        }
        else if (freeLandmarkCount > 0U)
        {
            freeLandmarkCount--;
            assignedLandmarkIndex = freeLandmarkIndices[freeLandmarkCount];
            landmarkPositions[assignedLandmarkIndex] = worldPoint;
        }
        else if (landmarkHighWaterMark < MAX_LANDMARKS)
        {
            assignedLandmarkIndex = landmarkHighWaterMark;
            landmarkHighWaterMark++;
            landmarkPositions[assignedLandmarkIndex] = worldPoint;
        }
        if (assignedLandmarkIndex >= MAX_LANDMARKS)
        {
            continue;
        }
        landmarkObservationCounts[assignedLandmarkIndex]++;
        const std::size_t observationIndex = storedKeyframe.observationCount;
        storedKeyframe.landmarkIndices[observationIndex] =
            assignedLandmarkIndex;
        storedKeyframe.localPoints[observationIndex] = localPoint;
        storedKeyframe.observationCount++;
    }
    storedKeyframeCount++;

    if (hasOverflowed)
    {
        return KeyframeLocalOptimizationStatus::
            KEYFRAME_LOCAL_OPTIMIZATION_WINDOW_OVERFLOW;
    }
    return KeyframeLocalOptimizationStatus::KEYFRAME_LOCAL_OPTIMIZATION_SUCCESS;
}

} /* namespace localisation::keyframe_local_optimization */
