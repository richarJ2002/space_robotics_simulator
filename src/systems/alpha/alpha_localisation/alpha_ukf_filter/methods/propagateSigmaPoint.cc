/*!
 * @File:         propagateSigmaPoint.cc
 *
 * @Brief:        Implements one sigma point's nominal-dynamics propagation.
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

void AlphaUkfFilterNode::propagateSigmaPoint(
    const StateVector    &state_in,
    const Eigen::Vector3d &specificForceBody_mPs2_in,
    const Eigen::Vector3d &angularVelocityBodyRadPerS_in,
    const Eigen::Vector3d &gravityAccelerationFixed_mPs2_in,
    double                 timeStepS_in,
    StateVector           &propagatedState_out)
{
    static constexpr Eigen::Index POSITION_INDEX =
        static_cast<Eigen::Index>(UkfStateIndex::UKF_STATE_INDEX_POSITION_X);
    static constexpr Eigen::Index ATTITUDE_INDEX =
        static_cast<Eigen::Index>(
            UkfStateIndex::UKF_STATE_INDEX_ATTITUDE_ROLL);
    static constexpr Eigen::Index VELOCITY_INDEX =
        static_cast<Eigen::Index>(UkfStateIndex::UKF_STATE_INDEX_VELOCITY_X);
    static constexpr Eigen::Index ACCELEROMETER_BIAS_INDEX =
        static_cast<Eigen::Index>(
            UkfStateIndex::UKF_STATE_INDEX_ACCELEROMETER_BIAS_X);
    static constexpr Eigen::Index GYROSCOPE_BIAS_INDEX =
        static_cast<Eigen::Index>(
            UkfStateIndex::UKF_STATE_INDEX_GYROSCOPE_BIAS_X);

    /* Correct the raw readings by this point's bias states. */
    const Eigen::Vector3d correctedSpecificForce_body_mPs2 =
        specificForceBody_mPs2_in -
        state_in.segment<3>(ACCELEROMETER_BIAS_INDEX);
    const Eigen::Vector3d correctedAngularVelocity_body_radPs =
        angularVelocityBodyRadPerS_in -
        state_in.segment<3>(GYROSCOPE_BIAS_INDEX);

    /* Rotate specific force into startup-fixed and add gravity. */
    const Eigen::Matrix3d rotation_bodyToFixed = rotationFromRpy(
        state_in(ATTITUDE_INDEX),
        state_in(ATTITUDE_INDEX + 1),
        state_in(ATTITUDE_INDEX + 2));
    const Eigen::Vector3d acceleration_fixed_mPs2 =
        rotation_bodyToFixed * correctedSpecificForce_body_mPs2 +
        gravityAccelerationFixed_mPs2_in;

    /* Integrate velocity, then position with the updated velocity. */
    propagatedState_out = state_in;
    propagatedState_out.segment<3>(VELOCITY_INDEX) +=
        acceleration_fixed_mPs2 * timeStepS_in;
    propagatedState_out.segment<3>(POSITION_INDEX) +=
        propagatedState_out.segment<3>(VELOCITY_INDEX) * timeStepS_in;

    /* Integrate attitude through the ZYX Euler rate matrix, clamped away
     * from the pitch singularity the level-terrain rover never reaches. */
    const double roll_rad  = state_in(ATTITUDE_INDEX);
    const double pitch_rad = state_in(ATTITUDE_INDEX + 1);
    const double clampedPitch_rad =
        std::max(-1.55, std::min(1.55, pitch_rad));
    const double sinRoll   = std::sin(roll_rad);
    const double cosRoll   = std::cos(roll_rad);
    const double cosPitch  = std::cos(clampedPitch_rad);
    const double tanPitch  = std::tan(clampedPitch_rad);
    Eigen::Matrix3d eulerRateMatrix;
    eulerRateMatrix << 1.0,
        sinRoll * tanPitch,
        cosRoll * tanPitch,
        0.0,
        cosRoll,
        -sinRoll,
        0.0,
        sinRoll / cosPitch,
        cosRoll / cosPitch;
    propagatedState_out.segment<3>(ATTITUDE_INDEX) +=
        eulerRateMatrix * correctedAngularVelocity_body_radPs * timeStepS_in;

    /* Wrap attitude back into its canonical range. */
    propagatedState_out(ATTITUDE_INDEX)     = wrapAngle(
        propagatedState_out(ATTITUDE_INDEX));
    propagatedState_out(ATTITUDE_INDEX + 1) = wrapAngle(
        propagatedState_out(ATTITUDE_INDEX + 1));
    propagatedState_out(ATTITUDE_INDEX + 2) = wrapAngle(
        propagatedState_out(ATTITUDE_INDEX + 2));
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
