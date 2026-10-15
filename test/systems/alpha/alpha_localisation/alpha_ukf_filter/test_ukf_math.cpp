/*!
 * @File:         test_ukf_math.cpp
 *
 * @Brief:        Tests the reusable, model-agnostic continuous unscented
 *                Kalman filter engine plus the UKF wrapper's pure math.
 *
 * @Date:         07/10/2026
 *
 */

/* Function Includes */
#include <gtest/gtest.h>

/* Object Include */
#include "objects/AlphaUkfFilterNodeClass.h"
#include "objects/ContinuousExtendedKalmanFilterClass.h"
#include "objects/ContinuousUnscentedKalmanFilterClass.h"
#include "objects/UkfConfigurationStruct.h"
#include "objects/UkfStateIndexEnum.h"

/* Data include */
#include <Eigen/Dense>

/* Generic Libraries */
#include <cmath>
#include <limits>

namespace
{

using UkfFilter = localisation::kalman_filter::ukf_continuous_kalman_filter::
    ContinuousUnscentedKalmanFilter;
using UkfStatus = localisation::kalman_filter::ukf_continuous_kalman_filter::
    UkfStatus;
using UkfConfiguration = localisation::kalman_filter::
    ukf_continuous_kalman_filter::UkfConfiguration;
using EkfFilter = localisation::kalman_filter::ekf_continuous_kalman_filter::
    ContinuousExtendedKalmanFilter;
using EkfStatus = localisation::kalman_filter::ekf_continuous_kalman_filter::
    FilterStatus;
using AlphaUkf = systems::alpha::alpha_localisation::alpha_ukf_filter::
    AlphaUkfFilterNode;

UkfConfiguration makeDefaultConfiguration()
{
    UkfConfiguration configuration;
    configuration.sigmaAlpha = 1.0e-3;
    configuration.sigmaBeta  = 2.0;
    configuration.sigmaKappa = 0.0;
    return configuration;
}

TEST(UkfMath, RejectsInvalidConfiguration)
{
    UkfFilter filter;
    const Eigen::VectorXd state    = Eigen::VectorXd::Zero(3);
    const Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(3, 3);

    /* Non-positive state size is rejected. */
    EXPECT_EQ(filter.initialize(0, state, covariance,
                                makeDefaultConfiguration()),
              UkfStatus::UKF_STATUS_INVALID_CONFIGURATION);

    /* Dimension mismatches are rejected. */
    EXPECT_EQ(filter.initialize(4, state, covariance,
                                makeDefaultConfiguration()),
              UkfStatus::UKF_STATUS_INVALID_CONFIGURATION);

    /* A non-positive sigma spread is rejected. */
    UkfConfiguration badAlpha = makeDefaultConfiguration();
    badAlpha.sigmaAlpha       = 0.0;
    EXPECT_EQ(filter.initialize(3, state, covariance, badAlpha),
              UkfStatus::UKF_STATUS_INVALID_CONFIGURATION);

    /* Calls before initialization report not-initialized. */
    Eigen::MatrixXd sigmaPoints;
    Eigen::VectorXd weightsMean;
    Eigen::VectorXd weightsCovariance;
    EXPECT_EQ(filter.generateSigmaPoints(sigmaPoints,
                                         weightsMean,
                                         weightsCovariance),
              UkfStatus::UKF_STATUS_NOT_INITIALIZED);
    EXPECT_EQ(filter.predictFromSigmaPoints(
                  Eigen::MatrixXd::Zero(3, 7),
                  Eigen::MatrixXd::Identity(3, 3)),
              UkfStatus::UKF_STATUS_NOT_INITIALIZED);
}

TEST(UkfMath, SigmaPointsRecoverMeanAndCovariance)
{
    /* A fixed non-trivial state and covariance. */
    Eigen::VectorXd state(5);
    state << 1.0, -2.0, 0.5, 3.0, -0.25;
    Eigen::MatrixXd factor(5, 5);
    factor << 1.0, 0.2, 0.0, 0.1, 0.0, 0.0, 2.0, 0.3, 0.0, 0.1, 0.0, 0.0,
        0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5, 0.2, 0.0, 0.0, 0.0, 0.0, 0.8;
    const Eigen::MatrixXd covariance =
        factor * factor.transpose() + Eigen::MatrixXd::Identity(5, 5);

    UkfFilter filter;
    ASSERT_EQ(filter.initialize(5, state, covariance,
                                makeDefaultConfiguration()),
              UkfStatus::UKF_STATUS_SUCCESS);

    Eigen::MatrixXd sigmaPoints;
    Eigen::VectorXd weightsMean;
    Eigen::VectorXd weightsCovariance;
    ASSERT_EQ(filter.generateSigmaPoints(sigmaPoints,
                                         weightsMean,
                                         weightsCovariance),
              UkfStatus::UKF_STATUS_SUCCESS);
    ASSERT_EQ(sigmaPoints.rows(), 5);
    ASSERT_EQ(sigmaPoints.cols(), 11);

    /* Mean weights sum to one (up to the roundoff the tiny default
     * spread's catastrophic cancellation leaves behind). */
    EXPECT_NEAR(weightsMean.sum(), 1.0, 1.0e-9);

    /* Recombination recovers the mean and covariance exactly (up to
     * roundoff): the scaled unscented transform is exact for the first
     * two moments by construction. */
    const Eigen::VectorXd recoveredMean = sigmaPoints * weightsMean;
    EXPECT_TRUE(recoveredMean.isApprox(state, 1.0e-9));
    Eigen::MatrixXd recoveredCovariance =
        Eigen::MatrixXd::Zero(5, 5);
    for (Eigen::Index point = 0; point < 11; point++)
    {
        const Eigen::VectorXd deviation = sigmaPoints.col(point) - state;
        recoveredCovariance += weightsCovariance(point) * deviation *
                               deviation.transpose();
    }
    EXPECT_TRUE(recoveredCovariance.isApprox(covariance, 1.0e-9));
}

TEST(UkfMath, AgreesWithEkfOnLinearPredict)
{
    constexpr Eigen::Index STATE_SIZE = 4;
    constexpr double       TIME_STEP_S = 0.1;

    Eigen::VectorXd state(STATE_SIZE);
    state << 1.0, -0.5, 2.0, 0.25;
    const Eigen::MatrixXd covariance =
        0.1 * Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    Eigen::MatrixXd processJacobian(STATE_SIZE, STATE_SIZE);
    processJacobian << 0.0, 1.0, 0.0, 0.0, -0.5, -0.1, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, -0.2, -0.05;
    const Eigen::MatrixXd processNoise =
        1.0e-4 * Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);

    /* Reference EKF prediction through the same linearization. */
    EkfFilter ekf;
    ASSERT_EQ(ekf.initialize(STATE_SIZE, state, covariance),
              EkfStatus::FILTER_STATUS_SUCCESS);
    ASSERT_EQ(ekf.predict(TIME_STEP_S,
                          processJacobian * state,
                          processJacobian,
                          processNoise),
              EkfStatus::FILTER_STATUS_SUCCESS);

    /* UKF prediction with the exactly-linear point propagation. */
    UkfFilter ukf;
    ASSERT_EQ(ukf.initialize(STATE_SIZE,
                             state,
                             covariance,
                             makeDefaultConfiguration()),
              UkfStatus::UKF_STATUS_SUCCESS);
    Eigen::MatrixXd sigmaPoints;
    Eigen::VectorXd weightsMean;
    Eigen::VectorXd weightsCovariance;
    ASSERT_EQ(ukf.generateSigmaPoints(sigmaPoints,
                                       weightsMean,
                                       weightsCovariance),
              UkfStatus::UKF_STATUS_SUCCESS);
    const Eigen::MatrixXd transition =
        Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) +
        processJacobian * TIME_STEP_S;
    Eigen::MatrixXd propagatedSigma(STATE_SIZE, 2 * STATE_SIZE + 1);
    for (Eigen::Index point = 0; point < 2 * STATE_SIZE + 1; point++)
    {
        propagatedSigma.col(point) = transition * sigmaPoints.col(point);
    }
    ASSERT_EQ(ukf.predictFromSigmaPoints(propagatedSigma,
                                         processNoise * TIME_STEP_S),
              UkfStatus::UKF_STATUS_SUCCESS);

    /* The unscented transform is exact for linear maps, so both engines
     * must agree to solver tolerance. */
    EXPECT_TRUE(ukf.getState().isApprox(ekf.getState(), 1.0e-9));
    EXPECT_TRUE(ukf.getCovariance().isApprox(ekf.getCovariance(), 1.0e-9));
}

TEST(UkfMath, AgreesWithEkfOnLinearUpdate)
{
    constexpr Eigen::Index STATE_SIZE       = 4;
    constexpr Eigen::Index MEASUREMENT_SIZE = 2;

    Eigen::VectorXd state(STATE_SIZE);
    state << 1.0, -0.5, 2.0, 0.25;
    const Eigen::MatrixXd covariance =
        0.5 * Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    Eigen::MatrixXd observationMatrix(MEASUREMENT_SIZE, STATE_SIZE);
    observationMatrix << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    const Eigen::MatrixXd measurementNoise =
        0.01 * Eigen::MatrixXd::Identity(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
    Eigen::VectorXd measurement(MEASUREMENT_SIZE);
    measurement << 1.2, 1.8;

    /* Reference EKF correction. */
    EkfFilter ekf;
    ASSERT_EQ(ekf.initialize(STATE_SIZE, state, covariance),
              EkfStatus::FILTER_STATUS_SUCCESS);
    ASSERT_EQ(ekf.update(measurement - observationMatrix * state,
                         observationMatrix,
                         measurementNoise),
              EkfStatus::FILTER_STATUS_SUCCESS);

    /* UKF correction with the exactly-linear observation mapping. */
    UkfFilter ukf;
    ASSERT_EQ(ukf.initialize(STATE_SIZE,
                             state,
                             covariance,
                             makeDefaultConfiguration()),
              UkfStatus::UKF_STATUS_SUCCESS);
    Eigen::MatrixXd sigmaPoints;
    Eigen::VectorXd weightsMean;
    Eigen::VectorXd weightsCovariance;
    ASSERT_EQ(ukf.generateSigmaPoints(sigmaPoints,
                                       weightsMean,
                                       weightsCovariance),
              UkfStatus::UKF_STATUS_SUCCESS);
    Eigen::MatrixXd measurementSigma(MEASUREMENT_SIZE,
                                     2 * STATE_SIZE + 1);
    for (Eigen::Index point = 0; point < 2 * STATE_SIZE + 1; point++)
    {
        measurementSigma.col(point) =
            observationMatrix * sigmaPoints.col(point);
    }
    double nis = 0.0;
    ASSERT_EQ(ukf.updateFromSigmaPoints(measurementSigma,
                                        sigmaPoints,
                                        measurement,
                                        measurementNoise,
                                        nis),
              UkfStatus::UKF_STATUS_SUCCESS);

    /* Same gain algebra, so the posteriors and the NIS agree. */
    EXPECT_TRUE(ukf.getState().isApprox(ekf.getState(), 1.0e-6));
    EXPECT_TRUE(ukf.getCovariance().isApprox(ekf.getCovariance(), 1.0e-6));
    const Eigen::VectorXd innovation = measurement - observationMatrix * state;
    const Eigen::MatrixXd innovationCovariance =
        observationMatrix * covariance * observationMatrix.transpose() +
        measurementNoise;
    const double expectedNis =
        innovation.dot(innovationCovariance.ldlt().solve(innovation));
    EXPECT_NEAR(nis, expectedNis, 1.0e-9);

    /* The non-mutating NIS preview scores the same value. */
    double previewNis = 0.0;
    UkfFilter preview;
    ASSERT_EQ(preview.initialize(STATE_SIZE,
                                 state,
                                 covariance,
                                 makeDefaultConfiguration()),
              UkfStatus::UKF_STATUS_SUCCESS);
    ASSERT_EQ(preview.calculateUpdateNis(measurementSigma,
                                         measurement,
                                         measurementNoise,
                                         previewNis),
              UkfStatus::UKF_STATUS_SUCCESS);
    EXPECT_NEAR(previewNis, expectedNis, 1.0e-9);
    EXPECT_TRUE(preview.getState().isApprox(state, 1.0e-12));
}

TEST(UkfMath, SelectNisThresholdMatchesChiSquareTable)
{
    /* Zero selects the 99% gate for the active dimension: 3 and 6 match
     * the wheel and visual UKF updates. */
    EXPECT_NEAR(AlphaUkf::selectNisThreshold(0.0, 3), 11.345, 1.0e-12);
    EXPECT_NEAR(AlphaUkf::selectNisThreshold(0.0, 6), 16.812, 1.0e-12);
    EXPECT_NEAR(AlphaUkf::selectNisThreshold(0.0, 1), 6.635, 1.0e-12);

    /* A positive value overrides the table. */
    EXPECT_NEAR(AlphaUkf::selectNisThreshold(42.0, 3), 42.0, 1.0e-12);

    /* An uncovered dimension never gates (infinite threshold). */
    EXPECT_TRUE(std::isinf(AlphaUkf::selectNisThreshold(0.0, 13)));
    EXPECT_TRUE(std::isinf(AlphaUkf::selectNisThreshold(0.0, 0)));
}

TEST(UkfMath, WrapAngleKeepsCanonicalRange)
{
    EXPECT_NEAR(AlphaUkf::wrapAngle(0.0), 0.0, 1.0e-12);
    EXPECT_NEAR(AlphaUkf::wrapAngle(3.0 * M_PI), -M_PI, 1.0e-9);
    EXPECT_NEAR(AlphaUkf::wrapAngle(-3.0 * M_PI), -M_PI, 1.0e-9);
    EXPECT_NEAR(AlphaUkf::wrapAngle(10.0),
                std::fmod(10.0 + M_PI, 2.0 * M_PI) - M_PI,
                1.0e-9);
}

TEST(UkfMath, RpyRoundTrip)
{
    const double roll  = 0.1;
    const double pitch = -0.2;
    const double yaw   = 1.0;
    double     recoveredRoll  = 0.0;
    double     recoveredPitch = 0.0;
    double     recoveredYaw   = 0.0;
    AlphaUkf::rpyFromRotation(AlphaUkf::rotationFromRpy(roll, pitch, yaw),
                              recoveredRoll,
                              recoveredPitch,
                              recoveredYaw);
    EXPECT_NEAR(recoveredRoll, roll, 1.0e-12);
    EXPECT_NEAR(recoveredPitch, pitch, 1.0e-12);
    EXPECT_NEAR(recoveredYaw, yaw, 1.0e-12);
}

TEST(UkfMath, PropagateSigmaPointMovesWithImu)
{
    /* Level, stationary state under lunar gravity plus a forward
     * specific-force excess integrates forward velocity and position. */
    AlphaUkf::StateVector state = AlphaUkf::StateVector::Zero();
    Eigen::Vector3d       specificForce(0.5, 0.0, 1.62);
    Eigen::Vector3d       angularVelocity(0.0, 0.0, 0.1);
    AlphaUkf::StateVector propagated;
    AlphaUkf::propagateSigmaPoint(state,
                                  specificForce,
                                  angularVelocity,
                                  Eigen::Vector3d(0.0, 0.0, -1.62),
                                  0.02,
                                  propagated);
    EXPECT_NEAR(propagated(0), 0.5 * 0.02 * 0.02, 1.0e-12);
    EXPECT_NEAR(propagated(6), 0.5 * 0.02, 1.0e-12);
    EXPECT_NEAR(propagated(2), 0.0, 1.0e-12);
    EXPECT_NEAR(propagated(5), 0.1 * 0.02, 1.0e-12);
}

} /* anonymous namespace */
