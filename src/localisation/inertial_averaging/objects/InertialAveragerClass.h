/*!
 * @File:         InertialAveragerClass.h
 *
 * @Brief:        Declares the fixed-cap windowed IMU averager.
 *
 * @Date:         07/10/2026
 *
 */

#ifndef SRS_LOCALISATION_INERTIAL_AVERAGER_CLASS_H
#define SRS_LOCALISATION_INERTIAL_AVERAGER_CLASS_H

/* Object Includes */
#include "objects/InertialAveragingConfigurationStruct.h"
#include "objects/InertialAveragingLimits.h"
#include "objects/InertialAveragingSampleStruct.h"
#include "objects/InertialAveragingStatusEnum.h"

/* Data include */
/* None */

/* Generic Libraries */
#include <array>
#include <cstddef>

namespace localisation::inertial_averaging
{

/*!
 * @brief           Windowed mean of bias-corrected IMU samples.
 *
 *                  Fixed-cap ring (MAX_AVERAGER_SAMPLES, never reallocated:
 *                  no allocation in the push() hot path). Each push runs
 *                  in amortized O(1): every sample is inserted and evicted
 *                  once. Construction never fails; initialize() validates
 *                  the window, and every fallible call returns a checked
 *                  InertialAveragingStatus. Not thread-safe: the owning
 *                  node calls it from one callback group only.
 */
class InertialAverager
{
  public:
    /*!
     * @brief           Establishes a valid empty uninitialized state.
     */
    InertialAverager() = default;

    /*!
     * @brief           Validates the window configuration.
     *
     * @param[in]       configuration_in
     *                  Window tuning; windowSampleTarget must lie in
     *                  1..MAX_AVERAGER_SAMPLES and windowDuration_ms must
     *                  be finite and positive.
     *
     * @return          SUCCESS, or INVALID_CONFIGURATION (state left
     *                  uninitialized).
     */
    [[nodiscard]] InertialAveragingStatus
        initialize(const InertialAveragingConfiguration &configuration_in);

    /*!
     * @brief           Inserts one bias-corrected sample and averages.
     *
     *                  Non-finite or non-monotonic (jittered) stamps are
     *                  ignored: the current window is averaged instead.
     *                  Count or time overflow drops the oldest sample
     *                  first.
     *
     * @param[in]       rawImu_in
     *                  Bias-corrected body-frame sample to insert.
     * @param[out]      filteredImu_out
     *                  Window mean (stamp of the newest stored sample);
     *                  untouched unless SUCCESS or WINDOW_OVERFLOW.
     *
     * @return          SUCCESS, WINDOW_OVERFLOW (mean still valid),
     *                  NOT_INITIALIZED, or EMPTY_WINDOW.
     */
    [[nodiscard]] InertialAveragingStatus
        push(const InertialAveragingSample &rawImu_in,
             InertialAveragingSample       &filteredImu_out);

    /*!
     * @brief           Counts the samples currently stored.
     *
     * @return          Stored sample count (0 .. MAX_AVERAGER_SAMPLES).
     */
    std::size_t sampleCount() const;

  private:
    /*!
     * @brief       True once initialize() accepted a configuration.
     */
    bool isInitialized{false};

    /*!
     * @brief       Accepted window tuning.
     */
    InertialAveragingConfiguration configuration{};

    /*!
     * @brief       Ring storage; only the first storedSampleCount entries
     *              from ringOldestIndex are meaningful.
     */
    std::array<InertialAveragingSample, MAX_AVERAGER_SAMPLES> ringSamples{};

    /*!
     * @brief       Ring position of the oldest stored sample.
     */
    std::size_t ringOldestIndex{0U};

    /*!
     * @brief       Number of samples currently stored.
     */
    std::size_t storedSampleCount{0U};

    /*!
     * @brief       Stamp of the newest stored sample, in ROS seconds.
     */
    double newestStamp_s{0.0};
};

} /* namespace localisation::inertial_averaging */

#endif /* SRS_LOCALISATION_INERTIAL_AVERAGER_CLASS_H */
