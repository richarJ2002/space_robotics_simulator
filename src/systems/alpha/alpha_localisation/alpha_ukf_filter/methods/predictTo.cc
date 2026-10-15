/*!
 * @File:         predictTo.cc
 *
 * @Brief:        Implements timestamp-ordered raw-IMU UKF prediction.
 *
 * @Date:         07/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/AlphaUkfFilterNodeClass.h"

/* C++ Standard Library Includes */
#include <algorithm>
#include <cstddef>

/* Object Includes */
#include "objects/UkfStateIndexEnum.h"

/* Generic Libraries */
/* None */

namespace systems::alpha::alpha_localisation::alpha_ukf_filter
{

AlphaUkfFilterNode::FilterStatus
    AlphaUkfFilterNode::predictTo(double targetTimestamp_s_in,
                                  bool   shouldSaveCheckpoints_in)
{
    constexpr double TIMESTAMP_TOLERANCE_S = 1.0e-9;
    constexpr double MAXIMUM_SUBSTEP_S     = 0.02;

    if (!hasInitialState)
    {
        return FilterStatus::UKF_STATUS_NOT_INITIALIZED;
    }
    if (targetTimestamp_s_in < stateTimestamp_s - TIMESTAMP_TOLERANCE_S)
    {
        return FilterStatus::UKF_STATUS_INVALID_INPUT;
    }
    if (targetTimestamp_s_in <= stateTimestamp_s + TIMESTAMP_TOLERANCE_S)
    {
        return FilterStatus::UKF_STATUS_SUCCESS;
    }

    /* Pushes every sigma point through the nominal dynamics over one
     * bounded substep and recombines; duplicated 0.02 s bound keeps each
     * point's linearization-equivalent propagation valid. */
    const auto propagateInterval =
        [this, MAXIMUM_SUBSTEP_S](
            double                 intervalEndTimestamp_s_in,
            const Eigen::Vector3d &specificForceBody_mPs2_in,
            const Eigen::Vector3d &angularVelocityBodyRadPerS_in)
        -> FilterStatus
    {
        static constexpr Eigen::Index ATTITUDE_INDEX =
            static_cast<Eigen::Index>(
                UkfStateIndex::UKF_STATE_INDEX_ATTITUDE_ROLL);
        double remainingTime_s = intervalEndTimestamp_s_in - stateTimestamp_s;
        while (remainingTime_s > 0.0)
        {
            const double timeStep_s =
                std::min(remainingTime_s, MAXIMUM_SUBSTEP_S);
            Eigen::MatrixXd sigmaPoints;
            Eigen::VectorXd weightsMean;
            Eigen::VectorXd weightsCovariance;
            FilterStatus sigmaStatus = filter.generateSigmaPoints(
                sigmaPoints,
                weightsMean,
                weightsCovariance);
            if (sigmaStatus != FilterStatus::UKF_STATUS_SUCCESS)
            {
                return sigmaStatus;
            }
            Eigen::MatrixXd propagatedSigma(
                STATE_SIZE,
                2 * STATE_SIZE + 1);
            for (Eigen::Index point = 0; point < 2 * STATE_SIZE + 1; point++)
            {
                const StateVector pointState = sigmaPoints.col(point);
                StateVector       propagatedPoint;
                propagateSigmaPoint(pointState,
                                    specificForceBody_mPs2_in,
                                    angularVelocityBodyRadPerS_in,
                                    gravityAcceleration_fixed_mPs2,
                                    timeStep_s,
                                    propagatedPoint);
                propagatedSigma.col(point) = propagatedPoint;
            }
            /* Discrete noise is the continuous density times dt. */
            const StateCovariance discreteNoise =
                processNoiseDensity * timeStep_s;
            const FilterStatus predictionStatus =
                filter.predictFromSigmaPoints(propagatedSigma,
                                              discreteNoise);
            if (predictionStatus != FilterStatus::UKF_STATUS_SUCCESS)
            {
                return predictionStatus;
            }
            /* Keep attitude canonical after recombination. */
            StateVector wrappedState = filter.getState();
            wrappedState(ATTITUDE_INDEX)     = wrapAngle(
                wrappedState(ATTITUDE_INDEX));
            wrappedState(ATTITUDE_INDEX + 1) = wrapAngle(
                wrappedState(ATTITUDE_INDEX + 1));
            wrappedState(ATTITUDE_INDEX + 2) = wrapAngle(
                wrappedState(ATTITUDE_INDEX + 2));
            const FilterStatus wrapStatus = filter.setState(wrappedState);
            if (wrapStatus != FilterStatus::UKF_STATUS_SUCCESS)
            {
                return wrapStatus;
            }
            stateTimestamp_s += timeStep_s;
            remainingTime_s -= timeStep_s;
        }
        return FilterStatus::UKF_STATUS_SUCCESS;
    };

    const std::size_t retainedSampleCount = imuBuffer.size();
    for (std::size_t sampleIndex = 0U; sampleIndex < retainedSampleCount;
         sampleIndex++)
    {
        const ImuSample &sample = imuBuffer[sampleIndex];
        if (sample.timestamp_s <= stateTimestamp_s + TIMESTAMP_TOLERANCE_S)
        {
            continue;
        }
        const double intervalEndTimestamp_s =
            std::min(sample.timestamp_s, targetTimestamp_s_in);
        const FilterStatus propagationStatus =
            propagateInterval(intervalEndTimestamp_s,
                              sample.linearAcceleration_body_mPs2,
                              sample.angularVelocity_body_radPs);
        if (propagationStatus != FilterStatus::UKF_STATUS_SUCCESS)
        {
            return propagationStatus;
        }
        if (shouldSaveCheckpoints_in)
        {
            saveFilterCheckpoint();
        }
        if (sample.timestamp_s > targetTimestamp_s_in + TIMESTAMP_TOLERANCE_S)
        {
            break;
        }
    }

    if (targetTimestamp_s_in > stateTimestamp_s + TIMESTAMP_TOLERANCE_S)
    {
        if (imuBuffer.empty() ||
            targetTimestamp_s_in - imuBuffer.back().timestamp_s >
                maximumImuMeasurementAgeS)
        {
            return FilterStatus::UKF_STATUS_INVALID_INPUT;
        }
        const FilterStatus propagationStatus = propagateInterval(
            targetTimestamp_s_in,
            imuBuffer.back().linearAcceleration_body_mPs2,
            imuBuffer.back().angularVelocity_body_radPs);
        if (propagationStatus != FilterStatus::UKF_STATUS_SUCCESS)
        {
            return propagationStatus;
        }
    }

    if (!imuBuffer.empty())
    {
        latestAngularVelocity_body_radPs =
            imuBuffer.back().angularVelocity_body_radPs -
            filter.getState().segment<3>(static_cast<Eigen::Index>(
                UkfStateIndex::UKF_STATE_INDEX_GYROSCOPE_BIAS_X));
    }
    latestState      = filter.getState();
    latestCovariance = filter.getCovariance();
    hasEstimate      = true;
    if (shouldSaveCheckpoints_in)
    {
        saveFilterCheckpoint();
    }
    return FilterStatus::UKF_STATUS_SUCCESS;
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
