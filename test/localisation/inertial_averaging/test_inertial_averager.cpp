/**
 * @file            test_inertial_averager.cpp
 *
 * @brief           Verifies the windowed inertial averager.
 *
 * @date            07/10/2026
 */

#include "objects/InertialAveragerClass.h"
#include "objects/InertialAveragingConfigurationStruct.h"
#include "objects/InertialAveragingLimits.h"
#include "objects/InertialAveragingSampleStruct.h"
#include "objects/InertialAveragingStatusEnum.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>

namespace
{

using localisation::inertial_averaging::InertialAverager;
using localisation::inertial_averaging::InertialAveragingConfiguration;
using localisation::inertial_averaging::InertialAveragingSample;
using localisation::inertial_averaging::InertialAveragingStatus;
using localisation::inertial_averaging::MAX_AVERAGER_SAMPLES;

constexpr InertialAveragingStatus SUCCESS =
    InertialAveragingStatus::INERTIAL_AVERAGING_SUCCESS;

InertialAveragingConfiguration makeWideOpenConfiguration()
{
    InertialAveragingConfiguration configuration{};
    configuration.windowSampleTarget = static_cast<int>(MAX_AVERAGER_SAMPLES);
    configuration.windowDuration_ms  = 100000.0;
    return configuration;
}

InertialAveragingSample makeSample(double stamp_s,
                                   double acceleration_mPs2,
                                   double angularRate_radPs)
{
    InertialAveragingSample sample{};
    sample.stamp_s           = stamp_s;
    sample.acceleration_mPs2 = {acceleration_mPs2,
                                acceleration_mPs2,
                                acceleration_mPs2};
    sample.angularRate_radPs = {angularRate_radPs,
                                angularRate_radPs,
                                angularRate_radPs};
    return sample;
}

TEST(InertialAverager, PushBeforeInitializeReportsNotInitialized)
{
    InertialAverager        averager;
    InertialAveragingSample filtered{};
    EXPECT_EQ(averager.push(makeSample(1.0, 0.0, 0.0), filtered),
              InertialAveragingStatus::INERTIAL_AVERAGING_NOT_INITIALIZED);
    EXPECT_EQ(averager.sampleCount(), 0U);
}

TEST(InertialAverager, InitializeRejectsInvalidConfiguration)
{
    const double invalidDurations_ms[] = {
        0.0,
        -200.0,
        std::numeric_limits<double>::quiet_NaN()};
    const int invalidTargets[] = {0, -4, 33, 1000};

    for (double duration_ms : invalidDurations_ms)
    {
        InertialAverager               averager;
        InertialAveragingConfiguration configuration{};
        configuration.windowDuration_ms = duration_ms;
        EXPECT_EQ(
            averager.initialize(configuration),
            InertialAveragingStatus::INERTIAL_AVERAGING_INVALID_CONFIGURATION);
        EXPECT_EQ(averager.sampleCount(), 0U);
    }
    for (int target : invalidTargets)
    {
        InertialAverager               averager;
        InertialAveragingConfiguration configuration{};
        configuration.windowSampleTarget = target;
        EXPECT_EQ(
            averager.initialize(configuration),
            InertialAveragingStatus::INERTIAL_AVERAGING_INVALID_CONFIGURATION);
        EXPECT_EQ(averager.sampleCount(), 0U);
    }
}

TEST(InertialAverager, MeanOfKnownSequence)
{
    InertialAverager averager;
    ASSERT_EQ(averager.initialize(makeWideOpenConfiguration()), SUCCESS);

    InertialAveragingSample filtered{};
    for (int index = 1; index <= 4; index++)
    {
        const double stamp_s = static_cast<double>(index) * 0.02;
        ASSERT_EQ(averager.push(makeSample(stamp_s,
                                           static_cast<double>(index),
                                           0.5 * static_cast<double>(index)),
                                filtered),
                  SUCCESS);
    }

    EXPECT_EQ(averager.sampleCount(), 4U);
    EXPECT_NEAR(filtered.stamp_s, 0.08, 1.0e-12);
    for (std::size_t axis = 0U; axis < 3U; axis++)
    {
        EXPECT_NEAR(filtered.acceleration_mPs2[axis], 2.5, 1.0e-12);
        EXPECT_NEAR(filtered.angularRate_radPs[axis], 1.25, 1.0e-12);
    }
}

TEST(InertialAverager, AlternatingNoiseAveragesTowardZero)
{
    InertialAverager averager;
    ASSERT_EQ(averager.initialize(makeWideOpenConfiguration()), SUCCESS);

    /* Alternating +/-0.1 has input variance 0.01; the 10-sample mean is
     * exactly zero, proving the variance-reduction property. */
    InertialAveragingSample filtered{};
    for (int index = 0; index < 10; index++)
    {
        const double sign    = (index % 2 == 0) ? 1.0 : -1.0;
        const double stamp_s = static_cast<double>(index + 1) * 0.02;
        ASSERT_EQ(averager.push(makeSample(stamp_s, 0.1 * sign, 0.0), filtered),
                  SUCCESS);
    }

    for (std::size_t axis = 0U; axis < 3U; axis++)
    {
        EXPECT_NEAR(filtered.acceleration_mPs2[axis], 0.0, 1.0e-12);
        EXPECT_NEAR(filtered.angularRate_radPs[axis], 0.0, 1.0e-12);
    }
}

TEST(InertialAverager, JitteredStampsAreIgnored)
{
    InertialAverager averager;
    ASSERT_EQ(averager.initialize(makeWideOpenConfiguration()), SUCCESS);

    InertialAveragingSample filtered{};
    ASSERT_EQ(averager.push(makeSample(1.00, 2.0, 0.0), filtered), SUCCESS);
    ASSERT_EQ(averager.push(makeSample(1.02, 4.0, 0.0), filtered), SUCCESS);
    EXPECT_EQ(averager.sampleCount(), 2U);

    /* Duplicate and out-of-order stamps carry no new information. */
    ASSERT_EQ(averager.push(makeSample(1.02, 100.0, 0.0), filtered), SUCCESS);
    ASSERT_EQ(averager.push(makeSample(0.50, 100.0, 0.0), filtered), SUCCESS);
    EXPECT_EQ(averager.sampleCount(), 2U);
    for (std::size_t axis = 0U; axis < 3U; axis++)
    {
        EXPECT_NEAR(filtered.acceleration_mPs2[axis], 3.0, 1.0e-12);
    }
}

TEST(InertialAverager, TimeBoundDropsOldestFirst)
{
    InertialAverager               averager;
    InertialAveragingConfiguration configuration{};
    configuration.windowSampleTarget = 32;
    configuration.windowDuration_ms  = 50.0;
    ASSERT_EQ(averager.initialize(configuration), SUCCESS);

    /* 20 ms spacing with a 50 ms window retains the newest 3 samples. */
    InertialAveragingSample filtered{};
    for (int index = 0; index < 5; index++)
    {
        const double stamp_s = static_cast<double>(index) * 0.02;
        ASSERT_EQ(averager.push(makeSample(stamp_s, 1.0, 0.0), filtered),
                  SUCCESS);
    }
    EXPECT_EQ(averager.sampleCount(), 3U);

    /* Newest stamps 0.06 and 0.08 keep values 1.0 either way; shift the
     * values instead so eviction is observable in the mean. */
    InertialAverager rampAverager;
    ASSERT_EQ(rampAverager.initialize(configuration), SUCCESS);
    for (int index = 0; index < 5; index++)
    {
        const double stamp_s = static_cast<double>(index) * 0.02;
        ASSERT_EQ(rampAverager.push(
                      makeSample(stamp_s, static_cast<double>(index), 0.0),
                      filtered),
                  SUCCESS);
    }
    EXPECT_EQ(rampAverager.sampleCount(), 3U);
    EXPECT_NEAR(filtered.acceleration_mPs2[0], 3.0, 1.0e-12);
}

TEST(InertialAverager, OverflowDropsOldestWithinCap)
{
    InertialAverager averager;
    ASSERT_EQ(averager.initialize(makeWideOpenConfiguration()), SUCCESS);

    /* Fill the 32-sample ring with values 0..31. */
    InertialAveragingSample filtered{};
    for (int index = 0; index < 32; index++)
    {
        const double stamp_s = static_cast<double>(index + 1) * 0.02;
        ASSERT_EQ(
            averager.push(makeSample(stamp_s, static_cast<double>(index), 0.0),
                          filtered),
            SUCCESS);
    }
    EXPECT_EQ(averager.sampleCount(), MAX_AVERAGER_SAMPLES);

    /* One more push overflows: oldest (0.0) drops, mean covers 1..32. */
    ASSERT_EQ(averager.push(makeSample(0.66, 32.0, 0.0), filtered),
              InertialAveragingStatus::INERTIAL_AVERAGING_WINDOW_OVERFLOW);
    EXPECT_EQ(averager.sampleCount(), MAX_AVERAGER_SAMPLES);
    EXPECT_NEAR(filtered.acceleration_mPs2[0], 16.5, 1.0e-9);
}

TEST(InertialAverager, EmptyWindowStatus)
{
    InertialAverager averager;
    ASSERT_EQ(averager.initialize(makeWideOpenConfiguration()), SUCCESS);

    /* A non-finite first sample cannot seed the window. */
    InertialAveragingSample filtered{};
    InertialAveragingSample badSample =
        makeSample(std::numeric_limits<double>::quiet_NaN(), 1.0, 0.0);
    EXPECT_EQ(averager.push(badSample, filtered),
              InertialAveragingStatus::INERTIAL_AVERAGING_EMPTY_WINDOW);
    EXPECT_EQ(averager.sampleCount(), 0U);
}

} /* namespace */
