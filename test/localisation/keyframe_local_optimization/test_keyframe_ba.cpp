/**
 * @file            test_keyframe_ba.cpp
 *
 * @brief           Verifies the keyframe Full BA window and solver.
 *
 * @date            08/10/2026
 */

#include "objects/BundleAdjustmentConfigurationStruct.h"
#include "objects/BundleAdjustmentOptimizerClass.h"
#include "objects/KeyframeLocalOptimizationLimits.h"
#include "objects/KeyframeLocalOptimizationStatusEnum.h"
#include "objects/KeyframeSampleStruct.h"
#include "objects/KeyframeWindowClass.h"
#include "objects/KeyframeWindowConfigurationStruct.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>

namespace
{

using localisation::keyframe_local_optimization::BundleAdjustmentConfiguration;
using localisation::keyframe_local_optimization::BundleAdjustmentOptimizer;
using localisation::keyframe_local_optimization::
    KeyframeLocalOptimizationStatus;
using localisation::keyframe_local_optimization::KeyframeSample;
using localisation::keyframe_local_optimization::KeyframeWindow;
using localisation::keyframe_local_optimization::KeyframeWindowConfiguration;
using localisation::keyframe_local_optimization::MAX_LANDMARKS;
using localisation::keyframe_local_optimization::MAX_WINDOW_KEYFRAMES;

constexpr KeyframeLocalOptimizationStatus SUCCESS =
    KeyframeLocalOptimizationStatus::KEYFRAME_LOCAL_OPTIMIZATION_SUCCESS;
constexpr KeyframeLocalOptimizationStatus NOT_INITIALIZED =
    KeyframeLocalOptimizationStatus::
        KEYFRAME_LOCAL_OPTIMIZATION_NOT_INITIALIZED;
constexpr KeyframeLocalOptimizationStatus INVALID_CONFIGURATION =
    KeyframeLocalOptimizationStatus::
        KEYFRAME_LOCAL_OPTIMIZATION_INVALID_CONFIGURATION;
constexpr KeyframeLocalOptimizationStatus WINDOW_OVERFLOW =
    KeyframeLocalOptimizationStatus::
        KEYFRAME_LOCAL_OPTIMIZATION_WINDOW_OVERFLOW;
constexpr KeyframeLocalOptimizationStatus INSUFFICIENT_DATA =
    KeyframeLocalOptimizationStatus::
        KEYFRAME_LOCAL_OPTIMIZATION_INSUFFICIENT_DATA;

KeyframeWindowConfiguration makeWindowConfiguration(std::size_t windowSize)
{
    KeyframeWindowConfiguration configuration{};
    configuration.windowSize        = windowSize;
    configuration.associationGate_m = 0.50;
    return configuration;
}

BundleAdjustmentConfiguration makeSolverConfiguration()
{
    BundleAdjustmentConfiguration configuration{};
    configuration.maximumIterations = 10U;
    configuration.useHuber          = true;
    configuration.huberDelta_m      = 0.05;
    return configuration;
}

KeyframeSample makePoseOnlySample(double stamp_s, double positionX_m)
{
    KeyframeSample sample{};
    sample.stamp_s                 = stamp_s;
    sample.position_fixed_m        = Eigen::Vector3d(positionX_m, 0.0, 0.0);
    sample.orientation_fixedToBody = Eigen::Quaterniond::Identity();
    sample.poseCovariance          = Eigen::Matrix<double, 6, 6>::Identity();
    sample.localPointCount         = 0U;
    sample.isPoseAvailable         = true;
    return sample;
}

TEST(KeyframeWindow, PushBeforeInitializeReportsNotInitialized)
{
    KeyframeWindow window;
    EXPECT_EQ(window.pushKeyframe(makePoseOnlySample(1.0, 0.0)),
              NOT_INITIALIZED);
    EXPECT_EQ(window.keyframeCount(), 0U);
    EXPECT_EQ(window.landmarkCount(), 0U);
}

TEST(KeyframeWindow, InitializeRejectsInvalidConfiguration)
{
    KeyframeWindow window;
    EXPECT_EQ(window.initialize(makeWindowConfiguration(0U)),
              INVALID_CONFIGURATION);
    EXPECT_EQ(
        window.initialize(makeWindowConfiguration(MAX_WINDOW_KEYFRAMES + 1U)),
        INVALID_CONFIGURATION);
    KeyframeWindowConfiguration negativeGate = makeWindowConfiguration(5U);
    negativeGate.associationGate_m           = -0.10;
    EXPECT_EQ(window.initialize(negativeGate), INVALID_CONFIGURATION);
}

TEST(BundleAdjustmentOptimizer, OptimizeBeforeInitializeReportsNotInitialized)
{
    KeyframeWindow            window;
    BundleAdjustmentOptimizer optimizer;
    EXPECT_EQ(optimizer.optimize(window), NOT_INITIALIZED);
}

TEST(BundleAdjustmentOptimizer, InitializeRejectsInvalidConfiguration)
{
    BundleAdjustmentOptimizer     optimizer;
    BundleAdjustmentConfiguration configuration = makeSolverConfiguration();
    configuration.maximumIterations             = 0U;
    EXPECT_EQ(optimizer.initialize(configuration), INVALID_CONFIGURATION);
}

TEST(KeyframeWindow, OverflowDropsOldest)
{
    KeyframeWindow window;
    ASSERT_EQ(window.initialize(makeWindowConfiguration(3U)), SUCCESS);
    EXPECT_EQ(window.pushKeyframe(makePoseOnlySample(1.0, 0.0)), SUCCESS);
    EXPECT_EQ(window.pushKeyframe(makePoseOnlySample(2.0, 1.0)), SUCCESS);
    EXPECT_EQ(window.pushKeyframe(makePoseOnlySample(3.0, 2.0)), SUCCESS);
    EXPECT_EQ(window.keyframeCount(), 3U);
    EXPECT_EQ(window.pushKeyframe(makePoseOnlySample(4.0, 3.0)),
              WINDOW_OVERFLOW);
    EXPECT_EQ(window.keyframeCount(), 3U);

    /* The oldest keyframe left the window: the newest remains. */
    KeyframeSample newestSample{};
    ASSERT_EQ(window.newestKeyframe(newestSample), SUCCESS);
    EXPECT_DOUBLE_EQ(newestSample.stamp_s, 4.0);
    EXPECT_DOUBLE_EQ(newestSample.position_fixed_m.x(), 3.0);
}

TEST(KeyframeWindow, StalePoseNeverCoasts)
{
    KeyframeWindow window;
    ASSERT_EQ(window.initialize(makeWindowConfiguration(3U)), SUCCESS);
    KeyframeSample staleSample  = makePoseOnlySample(1.0, 0.0);
    staleSample.isPoseAvailable = false;
    EXPECT_EQ(window.pushKeyframe(staleSample), INSUFFICIENT_DATA);
    EXPECT_EQ(window.keyframeCount(), 0U);
}

TEST(BundleAdjustmentOptimizer, ReprojectionErrorDecreases)
{
    /* Three keyframes observe one shared landmark; the middle pose
     * starts perturbed, so joint refinement must lower the cost. */
    KeyframeWindow window;
    ASSERT_EQ(window.initialize(makeWindowConfiguration(5U)), SUCCESS);

    const Eigen::Vector3d landmark_m(2.0, 0.5, -1.0);
    const double          truePositions_m[] = {0.0, 1.0, 2.0};
    for (std::size_t keyframeIndex = 0U; keyframeIndex < 3U; keyframeIndex++)
    {
        KeyframeSample sample =
            makePoseOnlySample(1.0 + static_cast<double>(keyframeIndex),
                               truePositions_m[keyframeIndex]);
        /* True local observation, then perturb the middle input pose
         * (the datum keyframe stays exact): the residual starts
         * nonzero and joint refinement must lower it. */
        sample.localPointCount = 1U;
        sample.localPoints[0]  = landmark_m - sample.position_fixed_m;
        if (keyframeIndex == 1U)
        {
            sample.position_fixed_m.x() += 0.20;
            sample.position_fixed_m.y() += 0.10;
        }
        ASSERT_EQ(window.pushKeyframe(sample), SUCCESS);
    }
    ASSERT_EQ(window.landmarkCount(), 1U);

    const BundleAdjustmentConfiguration solverConfiguration =
        makeSolverConfiguration();
    double initialCost_m2 = 0.0;
    ASSERT_EQ(window.computeTotalError(solverConfiguration, initialCost_m2),
              SUCCESS);
    EXPECT_GT(initialCost_m2, 0.0);

    BundleAdjustmentOptimizer optimizer;
    ASSERT_EQ(optimizer.initialize(solverConfiguration), SUCCESS);
    EXPECT_EQ(optimizer.optimize(window), SUCCESS);

    double optimizedCost_m2 = 0.0;
    ASSERT_EQ(window.computeTotalError(solverConfiguration, optimizedCost_m2),
              SUCCESS);
    EXPECT_LT(optimizedCost_m2, initialCost_m2);
}

TEST(BundleAdjustmentOptimizer, EmptyWindowReportsInsufficientData)
{
    KeyframeWindow window;
    ASSERT_EQ(window.initialize(makeWindowConfiguration(5U)), SUCCESS);
    BundleAdjustmentOptimizer optimizer;
    ASSERT_EQ(optimizer.initialize(makeSolverConfiguration()), SUCCESS);
    EXPECT_EQ(optimizer.optimize(window), INSUFFICIENT_DATA);
}

TEST(KeyframeWindow, RejectsNonFinitePosition)
{
    /* A NaN position would seed NaN landmarks and freeze the
     * solver, so every component is rejected like a stale pose. */
    KeyframeWindow window;
    ASSERT_EQ(window.initialize(makeWindowConfiguration(3U)), SUCCESS);
    for (std::size_t componentIndex = 0U; componentIndex < 3U; componentIndex++)
    {
        KeyframeSample badSample = makePoseOnlySample(1.0, 0.0);
        badSample.position_fixed_m(static_cast<Eigen::Index>(componentIndex)) =
            std::nan("");
        EXPECT_EQ(window.pushKeyframe(badSample), INSUFFICIENT_DATA);
    }
    EXPECT_EQ(window.keyframeCount(), 0U);
    EXPECT_EQ(window.landmarkCount(), 0U);
}

TEST(KeyframeWindow, RejectsNonFiniteOrientation)
{
    /* A non-finite orientation would lift points to NaN world
     * positions, so every coefficient is rejected like a stale
     * pose. */
    KeyframeWindow window;
    ASSERT_EQ(window.initialize(makeWindowConfiguration(3U)), SUCCESS);
    for (std::size_t componentIndex = 0U; componentIndex < 4U; componentIndex++)
    {
        KeyframeSample badSample = makePoseOnlySample(1.0, 0.0);
        badSample.orientation_fixedToBody.coeffs()(static_cast<Eigen::Index>(
            componentIndex))     = std::numeric_limits<double>::infinity();
        EXPECT_EQ(window.pushKeyframe(badSample), INSUFFICIENT_DATA);
    }
    EXPECT_EQ(window.keyframeCount(), 0U);
    EXPECT_EQ(window.landmarkCount(), 0U);
}

TEST(BundleAdjustmentOptimizer, PoseOnlyWindowReportsInsufficientData)
{
    /* Pose-only keyframes carry no observations: the optimizer
     * stays INSUFFICIENT_DATA, so runOptimization() returns before
     * publishOdometry() and no odometry is ever published. */
    KeyframeWindow window;
    ASSERT_EQ(window.initialize(makeWindowConfiguration(5U)), SUCCESS);
    ASSERT_EQ(window.pushKeyframe(makePoseOnlySample(1.0, 0.0)), SUCCESS);
    ASSERT_EQ(window.pushKeyframe(makePoseOnlySample(2.0, 1.0)), SUCCESS);
    ASSERT_EQ(window.pushKeyframe(makePoseOnlySample(3.0, 2.0)), SUCCESS);
    EXPECT_EQ(window.keyframeCount(), 3U);
    EXPECT_EQ(window.landmarkCount(), 0U);
    BundleAdjustmentOptimizer optimizer;
    ASSERT_EQ(optimizer.initialize(makeSolverConfiguration()), SUCCESS);
    EXPECT_EQ(optimizer.optimize(window), INSUFFICIENT_DATA);
}

TEST(KeyframeWindow, TighterGateSeparatesNearLandmarks)
{
    /* WP06-07 square-bag mechanism: two distinct features 7 cm
     * apart merged under the old 0.10 m gate, letting one bad
     * landmark drag poses; the retuned 0.05 m gate keeps them
     * separate (under-merging degrades to the visual input). */
    KeyframeSample firstSample   = makePoseOnlySample(1.0, 0.0);
    firstSample.localPointCount  = 1U;
    firstSample.localPoints[0]   = Eigen::Vector3d(1.0, 0.0, 0.0);
    KeyframeSample secondSample  = makePoseOnlySample(2.0, 0.0);
    secondSample.localPointCount = 1U;
    secondSample.localPoints[0]  = Eigen::Vector3d(1.07, 0.0, 0.0);

    KeyframeWindow              tightWindow;
    KeyframeWindowConfiguration tightConfiguration =
        makeWindowConfiguration(5U);
    tightConfiguration.associationGate_m = 0.05;
    ASSERT_EQ(tightWindow.initialize(tightConfiguration), SUCCESS);
    ASSERT_EQ(tightWindow.pushKeyframe(firstSample), SUCCESS);
    ASSERT_EQ(tightWindow.pushKeyframe(secondSample), SUCCESS);
    EXPECT_EQ(tightWindow.landmarkCount(), 2U);

    KeyframeWindow              looseWindow;
    KeyframeWindowConfiguration looseConfiguration =
        makeWindowConfiguration(5U);
    looseConfiguration.associationGate_m = 0.10;
    ASSERT_EQ(looseWindow.initialize(looseConfiguration), SUCCESS);
    ASSERT_EQ(looseWindow.pushKeyframe(firstSample), SUCCESS);
    ASSERT_EQ(looseWindow.pushKeyframe(secondSample), SUCCESS);
    EXPECT_EQ(looseWindow.landmarkCount(), 1U);
}

TEST(KeyframeWindow, TighterHuberDownweightsOutlierMore)
{
    /* A 0.10 m residual (the old gate scale) costs delta * r, so
     * the retuned 0.03 m delta charges less than 0.05 m and one
     * mis-associated landmark pulls poses less. */
    KeyframeWindow window;
    ASSERT_EQ(window.initialize(makeWindowConfiguration(5U)), SUCCESS);
    KeyframeSample firstSample  = makePoseOnlySample(1.0, 0.0);
    firstSample.localPointCount = 1U;
    firstSample.localPoints[0]  = Eigen::Vector3d(1.0, 0.0, 0.0);
    ASSERT_EQ(window.pushKeyframe(firstSample), SUCCESS);
    KeyframeSample secondSample  = makePoseOnlySample(2.0, 0.0);
    secondSample.localPointCount = 1U;
    secondSample.localPoints[0]  = Eigen::Vector3d(1.10, 0.0, 0.0);
    ASSERT_EQ(window.pushKeyframe(secondSample), SUCCESS);
    ASSERT_EQ(window.landmarkCount(), 1U);

    BundleAdjustmentConfiguration tightSolver = makeSolverConfiguration();
    tightSolver.huberDelta_m                  = 0.03;
    BundleAdjustmentConfiguration looseSolver = makeSolverConfiguration();
    looseSolver.huberDelta_m                  = 0.05;
    double tightCost_m2                       = 0.0;
    double looseCost_m2                       = 0.0;
    ASSERT_EQ(window.computeTotalError(tightSolver, tightCost_m2), SUCCESS);
    ASSERT_EQ(window.computeTotalError(looseSolver, looseCost_m2), SUCCESS);
    EXPECT_GT(looseCost_m2, 0.0);
    EXPECT_LT(tightCost_m2, looseCost_m2);
}

} // namespace
