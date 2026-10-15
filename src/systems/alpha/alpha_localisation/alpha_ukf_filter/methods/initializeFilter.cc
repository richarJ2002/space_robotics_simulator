/*!
 * @File:         initializeFilter.cc
 *
 * @Brief:        Implements UKF seeding from stationary calibration.
 *
 * @Date:         07/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/AlphaUkfFilterNodeClass.h"

/* C++ Standard Library Includes */
#include <cmath>

/* Object Includes */
#include "objects/UkfStateIndexEnum.h"

/* Generic Libraries */
/* None */

namespace systems::alpha::alpha_localisation::alpha_ukf_filter
{

AlphaUkfFilterNode::FilterStatus
    AlphaUkfFilterNode::initializeFilter(double timestamp_s_in)
{
    /* Average the accumulated stationary samples. */
    const double sampleCount =
        static_cast<double>(imuInitializationSampleCount);
    const Eigen::Vector3d meanSpecificForce_body_mPs2 =
        initializationSpecificForceSum_body_mPs2 / sampleCount;
    const Eigen::Vector3d meanAngularVelocity_body_radPs =
        initializationAngularVelocitySum_body_radPs / sampleCount;

    /* Level roll/pitch from the mean specific force: stationary, the
     * accelerometer reports R^T (0, 0, g), so pitch = asin(-fx / g) and
     * roll = atan2(fy, fz). Yaw is unobservable stationary and starts at
     * zero; the accelerometer bias starts at zero. */
    const double clampedPitchSine =
        std::max(-1.0,
                 std::min(1.0,
                          -meanSpecificForce_body_mPs2.x() /
                              gravityMagnitude_mPs2));
    StateVector initialState = StateVector::Zero();
    initialState(static_cast<Eigen::Index>(
        UkfStateIndex::UKF_STATE_INDEX_ATTITUDE_ROLL)) = std::atan2(
        meanSpecificForce_body_mPs2.y(),
        meanSpecificForce_body_mPs2.z());
    initialState(static_cast<Eigen::Index>(
        UkfStateIndex::UKF_STATE_INDEX_ATTITUDE_PITCH)) =
        std::asin(clampedPitchSine);
    initialState.segment<3>(static_cast<Eigen::Index>(
        UkfStateIndex::UKF_STATE_INDEX_GYROSCOPE_BIAS_X)) =
        meanAngularVelocity_body_radPs;

    /* Seed the covariance diagonal from the configured variances. */
    StateCovariance initialCovariance = StateCovariance::Zero();
    for (Eigen::Index index = 0; index < 3; index++)
    {
        initialCovariance(
            static_cast<Eigen::Index>(
                UkfStateIndex::UKF_STATE_INDEX_POSITION_X) +
                index,
            static_cast<Eigen::Index>(
                UkfStateIndex::UKF_STATE_INDEX_POSITION_X) +
                index) = initialPositionVariance;
        initialCovariance(
            static_cast<Eigen::Index>(
                UkfStateIndex::UKF_STATE_INDEX_ATTITUDE_ROLL) +
                index,
            static_cast<Eigen::Index>(
                UkfStateIndex::UKF_STATE_INDEX_ATTITUDE_ROLL) +
                index) = initialAttitudeVariance;
        initialCovariance(
            static_cast<Eigen::Index>(
                UkfStateIndex::UKF_STATE_INDEX_VELOCITY_X) +
                index,
            static_cast<Eigen::Index>(
                UkfStateIndex::UKF_STATE_INDEX_VELOCITY_X) +
                index) = initialVelocityVariance;
        initialCovariance(
            static_cast<Eigen::Index>(
                UkfStateIndex::UKF_STATE_INDEX_ACCELEROMETER_BIAS_X) +
                index,
            static_cast<Eigen::Index>(
                UkfStateIndex::UKF_STATE_INDEX_ACCELEROMETER_BIAS_X) +
                index) = initialAccelerometerBiasVariance;
        initialCovariance(
            static_cast<Eigen::Index>(
                UkfStateIndex::UKF_STATE_INDEX_GYROSCOPE_BIAS_X) +
                index,
            static_cast<Eigen::Index>(
                UkfStateIndex::UKF_STATE_INDEX_GYROSCOPE_BIAS_X) +
                index) = initialGyroscopeBiasVariance;
    }

    /* Sigma scaling comes from the constructor-validated parameters. */
    FilterConfiguration sigmaConfiguration;
    sigmaConfiguration.sigmaAlpha = sigmaAlpha;
    sigmaConfiguration.sigmaBeta  = sigmaBeta;
    sigmaConfiguration.sigmaKappa = sigmaKappa;
    const FilterStatus initializeStatus = filter.initialize(
        STATE_SIZE,
        initialState,
        initialCovariance,
        sigmaConfiguration);
    if (initializeStatus != FilterStatus::UKF_STATUS_SUCCESS)
    {
        return initializeStatus;
    }

    stateTimestamp_s   = timestamp_s_in;
    latestState        = initialState;
    latestCovariance   = initialCovariance;
    hasInitialState    = true;
    hasEstimate        = false;
    initializationTimestamp_s = timestamp_s_in;
    visualFusedCountAtInitialization = visualDiagnostics.fusedCount;
    saveFilterCheckpoint();

    return FilterStatus::UKF_STATUS_SUCCESS;
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
