/*!
 * @File:         publishDiagnosticsCallBack.cc
 *
 * @Brief:        Implements periodic UKF estimator diagnostics: a recorded
 *                DiagnosticArray every simulated second and a compact
 *                console health line every five.
 *
 * @Date:         07/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/AlphaUkfFilterNodeClass.h"

/* Function Includes */
#include "private_functions/private_functions.h"

/* C++ Standard Library Includes */
#include <cmath>

/* External Library Includes */
#include <Eigen/Eigenvalues>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

/* Other Project Module Includes */
#include "console/console.h"
#include "diagnostics/diagnostics.h"

/* Object Includes */
#include "objects/UkfStateIndexEnum.h"

namespace systems::alpha::alpha_localisation::alpha_ukf_filter
{

void AlphaUkfFilterNode::publishDiagnosticsCallBack()
{
    namespace diagnostics = common::diagnostics;

    /* Filter-wide covariance health, evaluated only once a state exists. */
    double covarianceTrace             = 0.0;
    double minimumCovarianceEigenvalue = 0.0;
    double minimumCovarianceDiagonal   = 0.0;
    double maximumCovarianceDiagonal   = 0.0;
    if (hasInitialState && latestCovariance.allFinite())
    {
        covarianceTrace           = latestCovariance.trace();
        minimumCovarianceDiagonal = latestCovariance.diagonal().minCoeff();
        maximumCovarianceDiagonal = latestCovariance.diagonal().maxCoeff();
        const Eigen::SelfAdjointEigenSolver<StateCovariance> decomposition(
            0.5 * (latestCovariance + latestCovariance.transpose()),
            Eigen::EigenvaluesOnly);
        if (decomposition.info() == Eigen::Success)
        {
            minimumCovarianceEigenvalue =
                decomposition.eigenvalues().minCoeff();
        }
    }

    /* Biases read straight from the state; the quaternion is reconstructed
     * from roll/pitch/yaw, so its norm is 1 by construction. */
    static constexpr Eigen::Index ACCELEROMETER_BIAS_INDEX =
        static_cast<Eigen::Index>(
            UkfStateIndex::UKF_STATE_INDEX_ACCELEROMETER_BIAS_X);
    static constexpr Eigen::Index GYROSCOPE_BIAS_INDEX =
        static_cast<Eigen::Index>(
            UkfStateIndex::UKF_STATE_INDEX_GYROSCOPE_BIAS_X);
    static constexpr Eigen::Index POSITION_INDEX =
        static_cast<Eigen::Index>(UkfStateIndex::UKF_STATE_INDEX_POSITION_X);
    const double quaternionNorm = hasInitialState ? 1.0 : 0.0;
    Eigen::Vector3d accelerometerBias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyroscopeBias     = Eigen::Vector3d::Zero();
    if (hasInitialState)
    {
        accelerometerBias = latestState.segment<3>(ACCELEROMETER_BIAS_INDEX);
        gyroscopeBias     = latestState.segment<3>(GYROSCOPE_BIAS_INDEX);
    }
    /* Console deltas since the previous health line; also the headline of
     * the filter-wide status. */
    const std::uint64_t rejectedCount =
        countRejections(visualDiagnostics) + countRejections(wheelDiagnostics);
    const std::string healthLine =
        hasInitialState ? common::console::formatText(
                              "UKF fused v%llu w%llu rej %llu",
                              static_cast<unsigned long long>(
                                  visualDiagnostics.fusedCount -
                                  previousConsoleVisualFusedCount),
                              static_cast<unsigned long long>(
                                  wheelDiagnostics.fusedCount -
                                  previousConsoleWheelFusedCount),
                              static_cast<unsigned long long>(
                                  rejectedCount - previousConsoleRejectedCount))
                        : std::string("UKF waiting for IMU init");

    /* Readiness contract read by the start-up supervisor. Comparison-only:
     * required_components never lists continuous_ukf, so this status never
     * gates motion. Unlike the EKF's increment mode, visual pose mode keeps
     * position variance bounded, so the guard always applies. */
    const double positionVarianceM2 =
        hasInitialState
            ? latestCovariance.block<3, 3>(POSITION_INDEX, POSITION_INDEX)
                  .trace()
            : 0.0;
    const std::uint64_t visualUpdatesSinceInit =
        visualDiagnostics.fusedCount - visualFusedCountAtInitialization;
    const double sinceInitialization_s =
        now().seconds() - initializationTimestamp_s;
    std::string readinessReason = "tracking";
    if (!hasInitialState)
    {
        readinessReason = "waiting for IMU init";
    }
    else if (visualUpdatesSinceInit < readinessMinimumVisualUpdates)
    {
        readinessReason = common::console::formatText(
            "%llu/%llu visual updates",
            static_cast<unsigned long long>(visualUpdatesSinceInit),
            static_cast<unsigned long long>(readinessMinimumVisualUpdates));
    }
    else if (sinceInitialization_s < readinessSettleTimeS)
    {
        readinessReason = common::console::formatText("settling %.1f/%.1f s",
                                                      sinceInitialization_s,
                                                      readinessSettleTimeS);
    }
    else if (!std::isfinite(positionVarianceM2) ||
             positionVarianceM2 > readinessMaximumPositionVarianceM2)
    {
        readinessReason =
            common::console::formatText("position variance %.2f m2",
                                        positionVarianceM2);
    }
    const bool isReady = readinessReason == "tracking";

    /* Filter-wide snapshot shared by every source's record. */
    diagnostic_msgs::msg::DiagnosticStatus filterStatus;
    filterStatus.name        = "continuous_ukf";
    filterStatus.hardware_id = baseFrame;
    filterStatus.level = isReady ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                 : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    filterStatus.message     = healthLine;
    diagnostics::addFlagValue("ready", isReady, filterStatus);
    diagnostics::addTextValue("reason", readinessReason, filterStatus);
    diagnostics::addFlagValue("initialized", hasInitialState, filterStatus);
    diagnostics::addRealValue("position_variance_m2",
                              positionVarianceM2,
                              filterStatus);
    diagnostics::addRealValue("covariance_trace",
                              covarianceTrace,
                              filterStatus);
    diagnostics::addRealValue("covariance_min_eigenvalue",
                              minimumCovarianceEigenvalue,
                              filterStatus);
    diagnostics::addRealValue("covariance_diagonal_min",
                              minimumCovarianceDiagonal,
                              filterStatus);
    diagnostics::addRealValue("covariance_diagonal_max",
                              maximumCovarianceDiagonal,
                              filterStatus);
    diagnostics::addRealValue("quaternion_norm", quaternionNorm, filterStatus);
    diagnostics::addRealValue("accel_bias_x_mps2",
                              accelerometerBias.x(),
                              filterStatus);
    diagnostics::addRealValue("accel_bias_y_mps2",
                              accelerometerBias.y(),
                              filterStatus);
    diagnostics::addRealValue("accel_bias_z_mps2",
                              accelerometerBias.z(),
                              filterStatus);
    diagnostics::addRealValue("gyro_bias_x_radps",
                              gyroscopeBias.x(),
                              filterStatus);
    diagnostics::addRealValue("gyro_bias_y_radps",
                              gyroscopeBias.y(),
                              filterStatus);
    diagnostics::addRealValue("gyro_bias_z_radps",
                              gyroscopeBias.z(),
                              filterStatus);

    diagnostic_msgs::msg::DiagnosticArray diagnosticsMessage;
    diagnosticsMessage.header.stamp = now();
    diagnosticsMessage.status.push_back(filterStatus);
    diagnosticsMessage.status.push_back(makeSourceStatus("imu",
                                                         imuDiagnostics));
    diagnosticsMessage.status.push_back(makeSourceStatus("visual",
                                                         visualDiagnostics));
    diagnosticsMessage.status.push_back(makeSourceStatus("wheel",
                                                         wheelDiagnostics));
    p_diagnosticsPublisher->publish(diagnosticsMessage);

    /* Compact console health line every fifth record. */
    diagnosticsRecordCount++;
    if (diagnosticsRecordCount % CONSOLE_HEALTH_PERIOD_TICKS == 0U)
    {
        previousConsoleVisualFusedCount = visualDiagnostics.fusedCount;
        previousConsoleWheelFusedCount  = wheelDiagnostics.fusedCount;
        previousConsoleRejectedCount   = rejectedCount;
        if (isReady)
        {
            SRS_LOG_INFO(get_logger(), "%s", healthLine.c_str());
        }
        else
        {
            SRS_LOG_INFO(get_logger(),
                         "UKF not ready: %s",
                         readinessReason.c_str());
        }
    }
}

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */
