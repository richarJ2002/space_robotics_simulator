/*!
 * @File:         push.cc
 *
 * @Brief:        Inserts one IMU sample and averages the window.
 *
 * @Date:         07/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/InertialAveragerClass.h"

/* Function Includes */
#include "private_functions/private_functions.h"

/* External Library Includes */
/* None */

/* Other Project Module Includes */
/* None */

/* Generic Libraries */
/* None */

namespace localisation::inertial_averaging
{

InertialAveragingStatus
    InertialAverager::push(const InertialAveragingSample &rawImu_in,
                           InertialAveragingSample       &filteredImu_out)
{
    /* Uninitialized use is a caller defect, never silent averaging. */
    if (!isInitialized)
    {
        return InertialAveragingStatus::INERTIAL_AVERAGING_NOT_INITIALIZED;
    }

    /* A non-finite or jittered (non-monotonic) stamp carries no new
     * information, so the sample is ignored and the current window is
     * averaged instead. */
    const bool isNewSample =
        isSampleFinite(rawImu_in) &&
        (storedSampleCount == 0U || rawImu_in.stamp_s > newestStamp_s);
    if (!isNewSample && storedSampleCount == 0U)
    {
        return InertialAveragingStatus::INERTIAL_AVERAGING_EMPTY_WINDOW;
    }

    bool hasOverflowed = false;
    if (isNewSample)
    {
        /* Count overflow drops the oldest sample to make room. */
        const std::size_t countCapacity =
            static_cast<std::size_t>(configuration.windowSampleTarget);
        if (storedSampleCount >= countCapacity)
        {
            ringOldestIndex = (ringOldestIndex + 1U) % MAX_AVERAGER_SAMPLES;
            storedSampleCount--;
            hasOverflowed = true;
        }

        /* Append at the ring tail; each sample is inserted once. */
        const std::size_t tailIndex =
            (ringOldestIndex + storedSampleCount) % MAX_AVERAGER_SAMPLES;
        ringSamples[tailIndex] = rawImu_in;
        storedSampleCount++;
        newestStamp_s = rawImu_in.stamp_s;

        /* Time overflow drops every sample older than the window. */
        const double oldestAllowedStamp_s =
            newestStamp_s - configuration.windowDuration_ms / 1000.0;
        while (storedSampleCount > 0U &&
               ringSamples[ringOldestIndex].stamp_s < oldestAllowedStamp_s)
        {
            ringOldestIndex = (ringOldestIndex + 1U) % MAX_AVERAGER_SAMPLES;
            storedSampleCount--;
        }
    }

    /* Mean over the stored window; stamp tracks the newest sample. */
    InertialAveragingSample meanSample{};
    meanSample.stamp_s = newestStamp_s;
    for (std::size_t storedIndex = 0U; storedIndex < storedSampleCount;
         storedIndex++)
    {
        const InertialAveragingSample &storedSample =
            ringSamples[(ringOldestIndex + storedIndex) % MAX_AVERAGER_SAMPLES];
        for (std::size_t axisIndex = 0U; axisIndex < 3U; axisIndex++)
        {
            meanSample.acceleration_mPs2[axisIndex] +=
                storedSample.acceleration_mPs2[axisIndex];
            meanSample.angularRate_radPs[axisIndex] +=
                storedSample.angularRate_radPs[axisIndex];
        }
    }
    const double storedCount = static_cast<double>(storedSampleCount);
    for (std::size_t axisIndex = 0U; axisIndex < 3U; axisIndex++)
    {
        meanSample.acceleration_mPs2[axisIndex] /= storedCount;
        meanSample.angularRate_radPs[axisIndex] /= storedCount;
    }
    filteredImu_out = meanSample;
    return hasOverflowed
               ? InertialAveragingStatus::INERTIAL_AVERAGING_WINDOW_OVERFLOW
               : InertialAveragingStatus::INERTIAL_AVERAGING_SUCCESS;
}

} /* namespace localisation::inertial_averaging */
