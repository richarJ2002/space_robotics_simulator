/*!
 * @File:         AlphaUkfFilterNodeClass.h
 *
 * @Brief:        Declares the ROS wrapper that fuses Alpha odometry through
 *                the reusable continuous-discrete UKF engine.
 *
 * @Date:         07/10/2026
 *
 */

/* Header Guard */
#ifndef SRS_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_UKF_FILTER_NODE_CLASS_H
#define SRS_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_UKF_FILTER_NODE_CLASS_H

/* Function Includes */
#include "console/console.h"

/* Object Include */
#include "objects/ContinuousUnscentedKalmanFilterClass.h"
#include "objects/UkfFusedMeasurementStruct.h"
#include "objects/UkfImuSampleStruct.h"
#include "objects/UkfMeasurementFusionResultEnum.h"
#include "objects/UkfMeasurementKindEnum.h"
#include "objects/UkfSourceDiagnosticsStruct.h"
#include "objects/UkfConfigurationStruct.h"
#include "objects/UkfStateIndexEnum.h"
#include "objects/UkfStatusEnum.h"

/* Data include */
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>

/* Generic Libraries */
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace systems::alpha::alpha_localisation::alpha_ukf_filter
{

/*!
 * @brief           Fuses raw IMU, visual pose, and wheel velocity
 *                  measurements into a bias-aware rover estimate with an
 *                  unscented transform, side-by-side with the EKF.
 *
 * This node owns Alpha's 15-component Euclidean state (startup-fixed
 * position, body-to-fixed roll/pitch/yaw, startup-fixed velocity,
 * body accelerometer bias, body gyroscope bias). Raw IMU samples drive
 * timestamp-ordered unscented prediction; visual odometry as an
 * absolute pose and wheel body velocity apply corrections through the
 * reusable UKF engine. The estimate is published once per IMU-driven
 * propagation, stamped with the state's own timestamp and capped at
 * prediction_rate_hz of simulation time. Every fused estimate is
 * published on output_topic as nav_msgs::msg::Odometry and appended to
 * a retained, rate-limited path on estimated_path_topic (see
 * publishEstimatedPath()). This node never broadcasts TF and never
 * gates READY: it is comparison-only against the default EKF. It owns
 * the ContinuousUnscentedKalmanFilter instance exclusively; its
 * callbacks run in the node's mutually exclusive default callback
 * group, which is the only synchronization this integration needs.
 */
class AlphaUkfFilterNode final : public rclcpp::Node
{
  public:
    /*!
     * @brief           Declares parameters and wires sensor subscriptions,
     *                  output publishers, and the output timer.
     *
     * @throws          std::invalid_argument if a configured
     *                  measurement-age, path, variance, sigma-point or
     *                  readiness constraint is invalid.
     */
    AlphaUkfFilterNode() :
        Node("continuous_ukf")
    {
        /*!
         * Declared first so the topic defaults below can be rooted at the
         * owning system's namespace.
         */
        const std::string systemName =
            declare_parameter<std::string>("system_name", "alpha");

        /* Raw specific force and angular rate are corrected by the UKF's
         * own bias states before propagation. */
        const std::string rawImuTopic =
            declare_parameter<std::string>("raw_imu_topic",
                                           "/" + systemName + "/imu");

        /* Input topic: visual_odometry's odometry output. */
        const std::string visualTopic = declare_parameter<std::string>(
            "visual_odometry_topic",
            "/" + systemName + "/localisation/visual/odometry");

        /* Input topic: wheel_odometry's odometry output. */
        const std::string wheelTopic = declare_parameter<std::string>(
            "wheel_odometry_topic",
            "/" + systemName + "/localisation/wheel/odometry");

        /* Output topic: this node's fused pose and twist estimate. */
        const std::string outputTopic = declare_parameter<std::string>(
            "output_topic",
            "/" + systemName + "/localisation/ukf/odometry");

        /* Output topic: the retained, rate-limited estimated path. */
        const std::string estimatedPathTopic = declare_parameter<std::string>(
            "estimated_path_topic",
            "/" + systemName + "/localisation/ukf/path");

        /* Output topic: periodic estimator diagnostics, shared by every
         * node of the owning system and recorded with every run. */
        const std::string diagnosticsTopic =
            declare_parameter<std::string>("diagnostics_topic",
                                           "/" + systemName + "/diagnostics");

        /* Parent frame published with the fused estimate. */
        odomFrame =
            declare_parameter<std::string>("odom_frame",
                                           systemName + "/startup_fixed");

        /* Child frame published with the fused estimate. */
        baseFrame =
            declare_parameter<std::string>("base_frame", "alpha/base_link");

        /* Read the raw path-length limit before validating it below. */
        const std::int64_t configuredMaximumPoses =
            declare_parameter<std::int64_t>("path_maximum_poses", 5000);

        /* Read the raw sample period before validating it below. */
        const double pathSamplePeriodS =
            declare_parameter<double>("path_sample_period_s", 0.1);

        /* Reject a configuration that could never retain or sample a path. */
        if (configuredMaximumPoses <= 0 || pathSamplePeriodS <= 0.0)
        {
            /* Fail fast at construction rather than misbehave later. */
            throw std::invalid_argument(
                "UKF path limits must be greater than zero");
        }

        /* Narrow the validated parameter to the member's storage type. */
        pathMaximumPoses = static_cast<std::size_t>(configuredMaximumPoses);

        /*!
         * Convert the configured period from seconds to nanoseconds so it
         * can be compared directly against message timestamps.
         */
        pathSamplePeriodNs = static_cast<std::int64_t>(pathSamplePeriodS *
                                                       NANOSECONDS_PER_SECOND);

        /* Configured prediction-timer rate, validated below. */
        const double predictionRateHz =
            declare_parameter<double>("prediction_rate_hz", 100.0);

        /* Maximum age a visual-odometry measurement may have before it is
         * dropped instead of fused; validated immediately below. */
        maximumVisualMeasurementAgeS =
            declare_parameter<double>("maximum_visual_measurement_age_s", 0.75);

        /* Maximum age a wheel-odometry measurement may have before it is
         * dropped instead of fused; validated immediately below. */
        maximumWheelMeasurementAgeS =
            declare_parameter<double>("maximum_wheel_measurement_age_s", 0.75);

        /* Maximum interval over which the latest IMU sample may be held
         * constant when prediction reaches beyond its timestamp. */
        maximumImuMeasurementAgeS =
            declare_parameter<double>("maximum_imu_measurement_age_s", 0.25);

        /* Source ablations preserve the independent measurement roles. */
        shouldFuseVisualPose =
            declare_parameter<bool>("fuse_visual_pose", true);
        shouldFuseWheelTwist =
            declare_parameter<bool>("fuse_wheel_twist", true);

        /* Zero selects the 99% chi-square limit for the update's actual
         * active dimension; a positive value explicitly overrides it. */
        visualNisThreshold =
            declare_parameter<double>("visual_nis_threshold", 0.0);
        wheelNisThreshold =
            declare_parameter<double>("wheel_nis_threshold", 0.0);
        if (visualNisThreshold < 0.0 || wheelNisThreshold < 0.0)
        {
            throw std::invalid_argument(
                "NIS thresholds must be zero (automatic) or positive");
        }

        /* Largest amount a measurement stamp may lead this node's clock
         * and still be fused. */
        maximumFutureStamp_s =
            declare_parameter<double>("maximum_future_stamp_s", 0.02);
        if (!(maximumFutureStamp_s >= 0.0))
        {
            throw std::invalid_argument(
                "maximum_future_stamp_s must not be negative");
        }

        /* A non-positive age limit would make every measurement either
         * always or never stale, which is never useful. */
        if (maximumVisualMeasurementAgeS <= 0.0 ||
            maximumWheelMeasurementAgeS <= 0.0 ||
            maximumImuMeasurementAgeS <= 0.0)
        {
            /* Fail fast at construction rather than misbehave later. */
            throw std::invalid_argument(
                "measurement age limits must be positive");
        }

        const std::int64_t configuredInitializationSamples =
            declare_parameter<std::int64_t>("imu_initialization_samples", 100);
        gravityMagnitude_mPs2 = declare_parameter<double>("gravity_mps2", 1.62);
        if (configuredInitializationSamples <= 0 ||
            gravityMagnitude_mPs2 <= 0.0)
        {
            throw std::invalid_argument(
                "IMU initialization samples and gravity must be positive");
        }
        imuInitializationSampleTarget =
            static_cast<std::size_t>(configuredInitializationSamples);
        gravityAcceleration_fixed_mPs2 =
            Eigen::Vector3d(0.0, 0.0, -gravityMagnitude_mPs2);

        /* Variance floor applied to visual-odometry measurements. */
        visualVariance =
            declare_parameter<double>("visual_measurement_variance", 0.03);

        /*!
         * Deliberately loose variance applied to visual odometry's
         * roll/pitch correction (see handleMeasurementCallBack()) so it
         * anchors long-run IMU attitude drift without overriding the
         * IMU's better short-term roll/pitch estimate.
         */
        visualAttitudeVariance =
            declare_parameter<double>("visual_attitude_measurement_variance",
                                      0.5);

        /* Variance floor applied to wheel-odometry measurements. */
        wheelVariance =
            declare_parameter<double>("wheel_measurement_variance", 0.10);

        /* Sigma-point scaling; the YAML values are the authority. */
        sigmaAlpha =
            declare_parameter<double>("sigma_alpha", 1.0e-3);
        sigmaBeta =
            declare_parameter<double>("sigma_beta", 2.0);
        sigmaKappa =
            declare_parameter<double>("sigma_kappa", 0.0);
        if (!(sigmaAlpha > 0.0) ||
            !std::isfinite(sigmaAlpha) ||
            !std::isfinite(sigmaBeta) ||
            !std::isfinite(sigmaKappa))
        {
            throw std::invalid_argument(
                "sigma_alpha must be positive and sigma weights finite");
        }

        /* Raw-IMU and bias random-walk noise are continuous-time
         * variances; the discrete noise of one substep is this density
         * times dt (see predictTo()). */
        processNoiseDensity = StateCovariance::Zero();
        setNoiseTriplet(
            processNoiseDensity,
            static_cast<Eigen::Index>(
                UkfStateIndex::UKF_STATE_INDEX_ATTITUDE_ROLL),
            declare_parameter<double>("gyroscope_noise_variance", 0.000064));
        setNoiseTriplet(
            processNoiseDensity,
            static_cast<Eigen::Index>(
                UkfStateIndex::UKF_STATE_INDEX_VELOCITY_X),
            declare_parameter<double>("accelerometer_noise_variance", 0.0025));
        setNoiseTriplet(
            processNoiseDensity,
            static_cast<Eigen::Index>(
                UkfStateIndex::UKF_STATE_INDEX_ACCELEROMETER_BIAS_X),
            declare_parameter<double>("accelerometer_bias_random_walk_variance",
                                      1.0e-6));
        setNoiseTriplet(
            processNoiseDensity,
            static_cast<Eigen::Index>(
                UkfStateIndex::UKF_STATE_INDEX_GYROSCOPE_BIAS_X),
            declare_parameter<double>("gyroscope_bias_random_walk_variance",
                                      1.0e-8));
        initialPositionVariance =
            declare_parameter<double>("initial_position_variance", 0.01);
        initialAttitudeVariance =
            declare_parameter<double>("initial_attitude_variance", 0.01);
        initialVelocityVariance =
            declare_parameter<double>("initial_velocity_variance", 0.01);
        initialAccelerometerBiasVariance =
            declare_parameter<double>("initial_accelerometer_bias_variance",
                                      0.01);
        initialGyroscopeBiasVariance =
            declare_parameter<double>("initial_gyroscope_bias_variance", 0.001);
        if (!(initialPositionVariance > 0.0) ||
            !(initialAttitudeVariance > 0.0) ||
            !(initialVelocityVariance > 0.0) ||
            !(initialAccelerometerBiasVariance > 0.0) ||
            !(initialGyroscopeBiasVariance > 0.0))
        {
            throw std::invalid_argument(
                "initial variances must be positive");
        }

        /* Readiness reported to the start-up supervisor: initialised, fed
         * enough visual corrections, settled for a while, and not
         * diverged. Comparison-only: required_components never lists
         * this node, so this status never gates motion. */
        const std::int64_t configuredReadinessVisualUpdates =
            declare_parameter<std::int64_t>("readiness_minimum_visual_updates",
                                            3);
        readinessSettleTimeS =
            declare_parameter<double>("readiness_settle_time_s", 5.0);
        readinessMaximumPositionVarianceM2 =
            declare_parameter<double>("readiness_maximum_position_variance_m2",
                                      1.0);
        if (configuredReadinessVisualUpdates < 0 ||
            !(readinessSettleTimeS >= 0.0) ||
            !(readinessMaximumPositionVarianceM2 > 0.0))
        {
            throw std::invalid_argument(
                "UKF readiness limits are outside valid bounds");
        }
        readinessMinimumVisualUpdates =
            static_cast<std::uint64_t>(configuredReadinessVisualUpdates);

        /* Publish the fused estimate as ordinary odometry. */
        p_outputPublisher =
            create_publisher<nav_msgs::msg::Odometry>(outputTopic,
                                                     rclcpp::QoS(10));

        /*!
         * Latch the path so a newly opened RViz view sees the current
         * history immediately, rather than waiting for the next sample.
         */
        p_estimatedPathPublisher = create_publisher<nav_msgs::msg::Path>(
            estimatedPathTopic,
            rclcpp::QoS(1).reliable().transient_local());

        /* Every raw IMU message is retained and processed in timestamp
         * order. */
        p_imuSubscription = create_subscription<sensor_msgs::msg::Imu>(
            rawImuTopic,
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Imu::ConstSharedPtr p_message_in)
            { handleImuCallBack(*p_message_in); });

        /* Every visual-odometry message is fused as a visual measurement. */
        p_visualSubscription = create_subscription<nav_msgs::msg::Odometry>(
            visualTopic,
            rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr p_message_in)
            {
                /* Delegate to the shared fusion logic, tagged so it applies
                 * the visual-specific observation mapping, variance and age
                 * check. */
                handleMeasurementCallBack(
                    *p_message_in,
                    MeasurementKind::MEASUREMENT_KIND_VISUAL);
            });

        /* Every wheel-odometry message is fused as a wheel measurement. */
        p_wheelSubscription = create_subscription<nav_msgs::msg::Odometry>(
            wheelTopic,
            rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr p_message_in)
            {
                /* Delegate to the shared fusion logic, tagged so it applies
                 * the wheel-specific observation mapping and variance. */
                handleMeasurementCallBack(
                    *p_message_in,
                    MeasurementKind::MEASUREMENT_KIND_WHEEL);
            });

        /*!
         * The estimate is published once per IMU-driven state propagation,
         * stamped with the state's own time, so no message repeats a stamp
         * or an unchanged state. prediction_rate_hz caps that output rate
         * in simulation time (a non-positive value is floored at 1 Hz);
         * at the 50 Hz IMU rate and the default 100 Hz cap every
         * propagation is published.
         */
        const double safeRateHz = std::max(1.0, predictionRateHz);
        minimumOutputPeriodS    = 1.0 / safeRateHz;

        /* Report bounded counters separately from the high-rate data path,
         * once per simulated second so the recorded diagnostics share the
         * clock of every other recorded topic. */
        p_diagnosticsPublisher =
            create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                diagnosticsTopic,
                rclcpp::QoS(10));
        p_diagnosticsTimer =
            create_timer(std::chrono::seconds(1),
                         [this]() { publishDiagnosticsCallBack(); });

        /* The resolved configuration is in the run's parameter snapshot, so
         * it is debug detail rather than operator output. */
        SRS_LOG_DEBUG(get_logger(),
                      "Comparison-only UKF fusing raw IMU, visual and wheel "
                      "measurements; publishing at most %.1f Hz",
                      safeRateHz);
    }

    /*!
     * @brief           Terminates the owned UKF and logs a failure if that
     *                  lifecycle transition is rejected.
     */
    ~AlphaUkfFilterNode() noexcept override
    {
        /* Return the owned engine to its uninitialized lifecycle state. */
        const FilterStatus status = filter.terminate();

        /* Termination only fails if the engine was never successfully
         * initialized in the first place; report that rather than hide
         * it, even though there is nothing left to do about it here. */
        if (status != FilterStatus::UKF_STATUS_SUCCESS)
        {
            /* Log at error severity since this indicates a lifecycle bug. */
            SRS_LOG_ERROR(get_logger(), "UKF termination failed");
        }
    }

    AlphaUkfFilterNode(const AlphaUkfFilterNode &otherNode_in) = delete;
    AlphaUkfFilterNode &
        operator=(const AlphaUkfFilterNode &otherNode_in)    = delete;
    AlphaUkfFilterNode(AlphaUkfFilterNode &&otherNode_in) = delete;
    AlphaUkfFilterNode &
        operator=(AlphaUkfFilterNode &&otherNode_in) = delete;

    /* ---------------------------------------------------------------------- *
     * PUBLIC TYPES AND CONSTANTS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       Reusable continuous-discrete UKF engine type.
     */
    using Filter = localisation::kalman_filter::
        ukf_continuous_kalman_filter::ContinuousUnscentedKalmanFilter;

    /*!
     * @brief       Filter lifecycle/operation status type.
     */
    using FilterStatus = localisation::kalman_filter::
        ukf_continuous_kalman_filter::UkfStatus;

    /*!
     * @brief       Sigma-point configuration type.
     */
    using FilterConfiguration = localisation::kalman_filter::
        ukf_continuous_kalman_filter::UkfConfiguration;

    /*!
     * @brief           Fixed-size UKF state for Alpha's estimate.
     *
     * The state stores startup-fixed position, body-to-fixed
     * roll/pitch/yaw, startup-fixed linear velocity, and body-frame
     * accelerometer and gyroscope biases in UkfStateIndex order.
     */
    using StateVector =
        Eigen::Matrix<double,
                      static_cast<Eigen::Index>(
                          UkfStateIndex::UKF_STATE_INDEX_COUNT),
                      1>;

    /*!
     * @brief           State covariance and discrete process-noise type.
     *
     * Row and column order matches StateVector.
     */
    using StateCovariance = Eigen::Matrix<
        double,
        static_cast<Eigen::Index>(UkfStateIndex::UKF_STATE_INDEX_COUNT),
        static_cast<Eigen::Index>(UkfStateIndex::UKF_STATE_INDEX_COUNT)>;

    /*!
     * @brief           Per-axis pose and twist measurement-variance vector.
     *
     * The first six entries follow ROS pose covariance order and the final
     * six entries follow ROS twist covariance order.
     */
    using MeasurementVarianceVector = Eigen::Matrix<double, 12, 1>;

    /*!
     * @brief           Number of scalar components in the state.
     */
    static constexpr Eigen::Index STATE_SIZE =
        static_cast<Eigen::Index>(UkfStateIndex::UKF_STATE_INDEX_COUNT);

    /*!
     * @brief           Visual pose measurement dimension: position xyz plus
     *                  roll/pitch/yaw.
     */
    static constexpr Eigen::Index VISUAL_MEASUREMENT_SIZE = 6;

    /*!
     * @brief           Wheel body-velocity measurement dimension.
     */
    static constexpr Eigen::Index WHEEL_MEASUREMENT_SIZE = 3;

    /*!
     * @brief           Converts roll/pitch/yaw to a body-to-fixed rotation.
     *
     *                  Uses ZYX order: R = Rz(yaw) Ry(pitch) Rx(roll).
     *
     * @param[in]       rollRad_in
     *                  Roll about body x, radians.
     * @param[in]       pitchRad_in
     *                  Pitch about body y, radians.
     * @param[in]       yawRad_in
     *                  Yaw about body z, radians.
     * @return          Body-to-fixed rotation matrix.
     */
    static Eigen::Matrix3d rotationFromRpy(double rollRad_in,
                                           double pitchRad_in,
                                           double yawRad_in);

    /*!
     * @brief           Converts a body-to-fixed rotation to roll/pitch/yaw.
     *
     * @param[in]       rotation_in
     *                  Body-to-fixed rotation matrix.
     * @param[out]      rollRad_out
     *                  Roll about body x, radians, in [-pi, pi].
     * @param[out]      pitchRad_out
     *                  Pitch about body y, radians, in [-pi/2, pi/2].
     * @param[out]      yawRad_out
     *                  Yaw about body z, radians, in [-pi, pi].
     */
    static void rpyFromRotation(const Eigen::Matrix3d &rotation_in,
                                double                &rollRad_out,
                                double                &pitchRad_out,
                                double                &yawRad_out);

    /*!
     * @brief           Propagates one sigma point through Alpha's nominal
     *                  dynamics over one bounded step.
     *
     * Raw body-frame specific force and angular rate are corrected by
     * the point's bias states. Attitude integrates the ZYX Euler rate
     * matrix; near the pitch singularity the rate matrix is evaluated
     * at a clamped pitch, which the level-terrain rover never reaches.
     *
     * @param[in]       state_in
     *                  Sigma-point state at the beginning of the interval.
     * @param[in]       specificForceBody_mPs2_in
     *                  Raw body-frame specific force in metres per second
     *                  squared.
     * @param[in]       angularVelocityBodyRadPerS_in
     *                  Raw body-frame angular velocity in radians per second.
     * @param[in]       gravityAccelerationFixed_mPs2_in
     *                  Physical gravity vector in startup-fixed, in metres
     *                  per second squared.
     * @param[in]       timeStepS_in
     *                  Positive propagation interval in seconds.
     * @param[out]      propagatedState_out
     *                  Sigma-point state at the end of the interval, with
     *                  attitude wrapped into [-pi, pi].
     */
    static void propagateSigmaPoint(
        const StateVector   &state_in,
        const Eigen::Vector3d &specificForceBody_mPs2_in,
        const Eigen::Vector3d &angularVelocityBodyRadPerS_in,
        const Eigen::Vector3d &gravityAccelerationFixed_mPs2_in,
        double                 timeStepS_in,
        StateVector           &propagatedState_out);

    /*!
     * @brief           Selects a configured or dimension-dependent NIS gate.
     *
     *                  Duplicates the EKF's 0.02 s-era 99% chi-square
     *                  table locally; the UKF innovation covariance P_yy
     *                  plays the role of the EKF's S with the same
     *                  measurement-dimension mapping.
     *
     * @param[in]       configuredThreshold_in
     *                  Positive override, or zero for the 99% chi-square
     *                  gate.
     * @param[in]       measurementDimension_in
     *                  Number of active scalar measurement channels.
     * @return          Positive NIS rejection threshold.
     */
    static double selectNisThreshold(double       configuredThreshold_in,
                                     Eigen::Index measurementDimension_in);

    /*!
     * @brief           Wraps an angle into the canonical [-pi, pi] range.
     *
     * @param[in]       angleRad_in
     *                  Angle in radians.
     * @return          Equivalent angle in radians within [-pi, pi].
     */
    static double wrapAngle(double angleRad_in);

    /*!
     * @brief           Derives per-state measurement variances from a
     *                   message's reported covariance, falling back to a
     *                   minimum floor for unreported or non-positive
     *                   entries.
     *
     * @param[in]       message_in
     *                  Odometry message whose diagonal pose/twist
     *                  covariance entries are read.
     * @param[in]       minimumVariance_in
     *                  Variance floor applied when a covariance entry is
     *                  missing, non-finite, or non-positive.
     * @return          Twelve-element variance vector in the UKF's state
     *                  units squared.
     */
    static MeasurementVarianceVector
        measurementVariances(const nav_msgs::msg::Odometry &message_in,
                             double                         minimumVariance_in);

  private:
    /*!
     * @brief           Complete estimator state retained for bounded-lag
     *                  rollback.
     */
    struct FilterCheckpoint
    {
        /*!
         * @brief           Epoch at which the checkpoint is valid.
         *
         * @frame           N/A
         * @units           ROS seconds
         */
        double timestamp_s{0.0};

        /*!
         * @brief           State at the epoch.
         *
         * @frame           Mixed; see UkfStateIndex
         * @units           Mixed; see UkfStateIndex
         */
        StateVector state{StateVector::Zero()};

        /*!
         * @brief           State covariance at the epoch.
         *
         * @frame           Mixed; see UkfStateIndex
         * @units           Squared state units
         */
        StateCovariance covariance{StateCovariance::Identity()};
    };

    /* ---------------------------------------------------------------------- *
     * CALLBACK METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Calibrates from or propagates with one raw IMU sample.
     *
     * Raw specific force and angular rate are corrected by the bias
     * states. The callback executes in the node's mutually exclusive
     * default callback group.
     *
     * @param[in]       message_in
     *                  Raw IMU message. Specific force is in metres per
     *                  second squared and angular velocity is in radians
     *                  per second, both in the body frame.
     */
    void handleImuCallBack(const sensor_msgs::msg::Imu &message_in);

    /*!
     * @brief           Applies one odometry measurement to the UKF through
     *                  its source's nonlinear observation mapping.
     *
     * Measurements arriving before the IMU-seeded initial state exists
     * are rejected. A measurement stamped before the current filter
     * epoch rolls the filter back to its stamp, is fused there, and
     * then every measurement already fused after that stamp is replayed
     * in order (see replayMeasurementsAfter()), so a lagged source never
     * erases newer updates from the estimate.
     *
     * @param[in]       message_in
     *                  Odometry measurement from one upstream source.
     * @param[in]       kind_in
     *                  Selects the observation mapping, minimum variance,
     *                  and age check.
     */
    void handleMeasurementCallBack(const nav_msgs::msg::Odometry &message_in,
                                   MeasurementKind                kind_in);

    /*!
     * @brief           Publishes per-source counters, timing and filter
     *                  health on the diagnostics topic and, every fifth
     *                  call, a compact console health line.
     *
     *                  Runs from a simulation-time timer once per second in
     *                  the node's mutually exclusive default callback
     *                  group, so it reads the filter state without locking.
     *                  Statuses are named continuous_ukf[/<source>] and
     *                  share the EKF's key set; the bag parser records a
     *                  documented-ignore for them (comparison-only).
     */
    void publishDiagnosticsCallBack();

    /* ---------------------------------------------------------------------- *
     * PRIVATE METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Sets three consecutive diagonal process-noise
     *                  density entries to one variance, floored above zero
     *                  for numerical conditioning.
     *
     * @param[in,out]   noise_inout
     *                  Process-noise density matrix whose diagonal is
     *                  updated.
     * @param[in]       firstIndex_in
     *                  First of three consecutive state indices to set.
     * @param[in]       variance_in
     *                  Configured variance in squared state units per
     *                  second.
     */
    static void setNoiseTriplet(StateCovariance &noise_inout,
                                Eigen::Index     firstIndex_in,
                                double           variance_in);

    /*!
     * @brief           Propagates the owned filter to a requested monotonic
     *                   timestamp, re-running the unscented prediction every
     *                   bounded substep.
     *
     *                  Each substep regenerates sigma points at the current
     *                  state, pushes every point through
     *                  propagateSigmaPoint(), and recombines with a discrete
     *                  process noise of density times dt. No substep exceeds
     *                  0.02 s (duplicated EKF bound).
     *
     * @param[in]       targetTimestamp_s_in
     *                  Monotonic target time in seconds; must not precede
     *                  the filter's current time.
     * @return          Lifecycle or numerical status.
     */
    FilterStatus predictTo(double targetTimestamp_s_in,
                           bool   shouldSaveCheckpoints_in = true);

    /*!
     * @brief           Seeds a level, zero-velocity state from stationary
     *                  calibration samples and starts the filter.
     *
     *                  Roll/pitch come from specific-force leveling, yaw
     *                  starts at zero, and the gyroscope bias starts at the
     *                  sample mean; the accelerometer bias starts at zero.
     *
     * @param[in]       timestamp_s_in
     *                  Sensor epoch at which the seeded state is valid, in
     *                  ROS seconds.
     *
     * @return          Filter lifecycle status.
     */
    [[nodiscard]] FilterStatus initializeFilter(double timestamp_s_in);

    /*!
     * @brief           Saves or replaces the checkpoint at the current filter
     *                  epoch.
     */
    void saveFilterCheckpoint();

    /*!
     * @brief           Restores the newest checkpoint no later than a target.
     * @return          True when a suitable valid checkpoint was restored.
     */
    [[nodiscard]] bool
        restoreFilterCheckpointAtOrBefore(double targetTimestamp_s_in);

    /*!
     * @brief           Discards checkpoints newer than the supplied epoch.
     *
     * @param[in]       targetTimestamp_s_in
     *                  Epoch after which checkpoints are discarded, ROS
     *                  seconds.
     */
    void discardFilterCheckpointsAfter(double targetTimestamp_s_in) noexcept;

    /*!
     * @brief           Removes all retained rollback checkpoints.
     */
    void clearFilterCheckpoints() noexcept;

    /*!
     * @brief           Converts one admitted odometry message into a
     *                  replayable measurement record.
     *
     *                  Only state-independent values are captured: the
     *                  measured quantities and their noise variances. The
     *                  visual record is [position xyz startup-fixed,
     *                  roll/pitch/yaw]; the wheel record is body velocity
     *                  xyz.
     *
     * @param[in]       message_in
     *                  Odometry measurement from one upstream source.
     * @param[in]       kind_in
     *                  Source that produced message_in.
     * @param[in]       measurementTimestamp_s_in
     *                  Header stamp of message_in, ROS seconds.
     * @param[out]      record_out
     *                  Record written only when this method returns true.
     *
     * @return          True when every measured value was finite.
     */
    [[nodiscard]] bool
        buildMeasurementRecord(const nav_msgs::msg::Odometry &message_in,
                               MeasurementKind                kind_in,
                               double            measurementTimestamp_s_in,
                               FusedMeasurement &record_out) const;

    /*!
     * @brief           Gates and applies one measurement record at the
     *                  current filter epoch.
     *
     *                  Measurement sigma points are mapped from the current
     *                  state sigma points, so the caller must first bring
     *                  the filter to the record's timestamp. The NIS gate
     *                  uses selectNisThreshold() against the UKF's P_yy
     *                  with the record's active dimension.
     *
     * @param[in]       record_in
     *                  Measurement record to apply.
     * @param[out]      nis_out
     *                  Normalized innovation squared, written whenever the
     *                  innovation covariance was valid.
     * @param[out]      correctionNorm_out
     *                  Norm of the applied state correction, written only
     *                  when the update succeeded.
     *
     * @return          Fused, NIS-rejected (filter unchanged) or
     *                  numerically rejected.
     */
    [[nodiscard]] MeasurementFusionResult
        fuseMeasurementRecord(const FusedMeasurement &record_in,
                              double                 &nis_out,
                              double                 &correctionNorm_out);

    /*!
     * @brief           Re-applies every retained measurement stamped after
     *                  a rollback epoch, then propagates to the present.
     *
     *                  Called after a delayed measurement was fused at its
     *                  own epoch. Without this, the measurements already
     *                  fused between that epoch and the present would be
     *                  lost from the estimate until the next update.
     *                  Checkpoints are saved as the replay advances. A
     *                  record that the gate now rejects is skipped but kept
     *                  for later replays.
     *
     * @param[in]       rollbackTimestamp_s_in
     *                  Epoch of the delayed measurement just fused, ROS
     *                  seconds; only records strictly after it replay.
     * @param[in]       presentTimestamp_s_in
     *                  Filter epoch before the rollback, ROS seconds.
     *
     * @return          Success, or the first prediction or numerical
     *                  failure; the caller then restores the present state.
     */
    [[nodiscard]] FilterStatus
        replayMeasurementsAfter(double rollbackTimestamp_s_in,
                                double presentTimestamp_s_in);

    /*!
     * @brief           Logs a throttled warning describing a rejected UKF
     *                   step.
     *
     * @param[in]       status_in
     *                  Non-success status returned by the filter or by
     *                  predictTo().
     */
    void logStepFailure(FilterStatus status_in);

    /*!
     * @brief           Publishes the latest cached estimate as odometry and
     *                  hands it to publishEstimatedPath(), unless the state
     *                  timestamp was already published or is closer than the
     *                  minimum output period to the previous publication.
     */
    void publishEstimate();

    /*!
     * @brief           Tests whether a pose can be published safely.
     *
     * @param[in]       pose_in
     *                  Pose expressed in the odom frame.
     *
     * @return          True when position and orientation are finite and the
     *                  quaternion has a nonzero norm.
     */
    static bool isPoseValid(const geometry_msgs::msg::Pose &pose_in);

    /*!
     * @brief           Converts a ROS timestamp to nanoseconds.
     *
     * @param[in]       stamp_in
     *                  Timestamp to convert.
     *
     * @return          Timestamp in nanoseconds.
     */
    static std::int64_t
        stampToNanoseconds(const builtin_interfaces::msg::Time &stamp_in);

    /*!
     * @brief           Validates and reframes one estimate odometry message.
     *
     * @param[in]       odometry_in
     *                  Odometry to validate and reframe.
     *
     * @param[in]       parentFrame_in
     *                  Frame id written into the output header.
     *
     * @param[in]       childFrame_in
     *                  Frame id written into the output child frame.
     *
     * @param[out]      odometry_out
     *                  Reframed odometry, valid only when this method returns
     *                  true.
     *
     * @return          True when the input pose was valid and odometry_out
     *                  was written.
     */
    bool prepareOdometry(const nav_msgs::msg::Odometry &odometry_in,
                         const std::string             &parentFrame_in,
                         const std::string             &childFrame_in,
                         nav_msgs::msg::Odometry       &odometry_out);

    /*!
     * @brief           Appends a time-sampled pose to a bounded path.
     *
     * @param[in]       odometry_in
     *                  Valid odometry already expressed in the target
     *                  frames.
     *
     * @param[in,out]   path_inout
     *                  Path accumulator updated in place.
     *
     * @param[in,out]   lastStampNs_inout
     *                  Timestamp of the most recently retained sample,
     *                  updated in place.
     *
     * @param[in,out]   publisher_inout
     *                  Publisher used to republish the updated path.
     */
    void
        appendPathPose(const nav_msgs::msg::Odometry &odometry_in,
                       nav_msgs::msg::Path           &path_inout,
                       std::int64_t                  &lastStampNs_inout,
                       rclcpp::Publisher<nav_msgs::msg::Path> &publisher_inout);

    /*!
     * @brief           Validates one fused estimate and, if valid, appends
     *                   it to the retained estimated path. Never broadcasts
     *                   TF: this node is comparison-only.
     *
     * @param[in]       odometry_in
     *                  Just-published fused estimate, in odomFrame/
     *                  baseFrame.
     */
    void publishEstimatedPath(const nav_msgs::msg::Odometry &odometry_in);

    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Nanoseconds in one second, used to convert path-sample
     *              period.
     */
    static constexpr double NANOSECONDS_PER_SECOND = 1.0e9;

    /*!
     * @brief           Fixed checkpoint capacity for bounded-lag rollback.
     *
     * @frame           N/A
     * @units           count
     */
    static constexpr std::size_t FILTER_CHECKPOINT_CAPACITY = 256U;

    /*!
     * @brief           Largest number of raw IMU samples retained for
     *                  prediction; older samples are dropped.
     *
     * @frame           N/A
     * @units           count
     */
    static constexpr std::size_t IMU_BUFFER_CAPACITY = 1024U;

    /*!
     * @brief           Largest number of fused measurements retained for
     *                  replay after a rollback; older records are dropped.
     *
     * @frame           N/A
     * @units           count
     */
    static constexpr std::size_t MEASUREMENT_HISTORY_CAPACITY = 64U;

    /*!
     * @brief           Owned continuous-discrete UKF engine.
     *
     * The engine stores the 15-component Euclidean state and its
     * covariance. The node's mutually exclusive callback group is the
     * sole synchronization mechanism protecting it.
     *
     * @frame           Mixed; see UkfStateIndex
     * @units           Mixed; see UkfStateIndex
     */
    Filter filter;

    /*!
     * @brief           Most recently cached state for publication.
     *
     * @frame           Mixed; see UkfStateIndex
     * @units           Mixed; see UkfStateIndex
     */
    StateVector latestState{StateVector::Zero()};

    /*!
     * @brief           Most recently cached state covariance.
     *
     * @frame           Mixed; see UkfStateIndex
     * @units           Squared state units
     */
    StateCovariance latestCovariance{StateCovariance::Identity()};

    /*!
     * @brief           Continuous process-noise density; the discrete
     *                  noise of one substep is this density times dt.
     *
     * @frame           Mixed; see UkfStateIndex
     * @units           Squared state units per second
     */
    StateCovariance processNoiseDensity{StateCovariance::Zero()};

    /*!
     * @brief           ROS timestamp at which the filter state and
     *                  covariance are valid.
     *
     * @frame           N/A
     * @units           seconds
     */
    double stateTimestamp_s{0.0};

    /*!
     * @brief           Recent raw IMU samples retained for prediction.
     *
     * Samples are copied from ROS messages and consumed chronologically
     * by predictTo(). The node's mutually exclusive callback group
     * serializes all access, so the buffer requires no internal lock.
     *
     * @frame           body
     * @units           seconds, metres per second squared, radians per second
     */
    std::deque<ImuSample> imuBuffer;

    /*!
     * @brief           Fixed-capacity chronological rollback checkpoints.
     *
     * @frame           Mixed; see FilterCheckpoint
     * @units           Mixed; see FilterCheckpoint
     */
    std::array<FilterCheckpoint, FILTER_CHECKPOINT_CAPACITY>
        filterCheckpoints{};

    /*!
     * @brief           Storage index where the next checkpoint is written.
     *
     * @frame           N/A
     * @units           index
     */
    std::size_t nextFilterCheckpointIndex{0U};

    /*!
     * @brief           Storage index of the oldest retained checkpoint.
     *
     * @frame           N/A
     * @units           index
     */
    std::size_t oldestFilterCheckpointIndex{0U};

    /*!
     * @brief           Number of valid retained checkpoints.
     *
     * @frame           N/A
     * @units           count
     */
    std::size_t filterCheckpointCount{0U};

    /*!
     * @brief           Recently fused visual and wheel measurements, in
     *                  timestamp order, replayed after a rollback.
     *
     * @frame           Mixed; see FusedMeasurement
     * @units           Mixed; see FusedMeasurement
     */
    std::deque<FusedMeasurement> measurementHistory;

    /*!
     * @brief       Publishes the fused odometry estimate.
     */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr p_outputPublisher;

    /*!
     * @brief           Subscribes to the raw IMU output.
     *
     * @frame           body
     * @units           metres per second squared and radians per second
     */
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr p_imuSubscription;

    /*!
     * @brief       Subscribes to visual_odometry's odometry output.
     */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        p_visualSubscription;

    /*!
     * @brief       Subscribes to wheel_odometry's odometry output.
     */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        p_wheelSubscription;

    /*!
     * @brief           Shortest simulated interval between two published
     *                  estimates (the inverse of prediction_rate_hz).
     *
     * @frame           N/A
     * @units           seconds
     */
    double minimumOutputPeriodS{0.01};

    /*!
     * @brief           State timestamp of the latest published estimate;
     *                  negative before the first.
     *
     * @frame           N/A
     * @units           ROS seconds
     */
    double lastPublishedStateTimestamp_s{-1.0};

    /*!
     * @brief           Simulation-time timer driving
     *                  publishDiagnosticsCallBack() once per second.
     *
     * @frame           N/A
     * @units           N/A
     */
    rclcpp::TimerBase::SharedPtr p_diagnosticsTimer;

    /*!
     * @brief           Publishes periodic estimator diagnostics on the
     *                  system's shared diagnostics topic.
     *
     * @frame           N/A
     * @units           N/A
     */
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        p_diagnosticsPublisher;

    /*!
     * @brief       Publishes a retained, bounded estimated path.
     */
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr p_estimatedPathPublisher;

    /*!
     * @brief       Accumulated estimated poses, all expressed in odomFrame.
     */
    nav_msgs::msg::Path estimatedPath;

    /*!
     * @brief       Frame published as the parent of the fused estimate.
     */
    std::string odomFrame;

    /*!
     * @brief       Frame published as the child of the fused estimate.
     */
    std::string baseFrame;

    /*!
     * @brief       Maximum number of poses retained in the estimated path.
     */
    std::size_t pathMaximumPoses{5000U};

    /*!
     * @brief       Minimum time between retained path samples in
     *              nanoseconds.
     */
    std::int64_t pathSamplePeriodNs{100000000};

    /*!
     * @brief       Timestamp of the most recently retained path sample.
     */
    std::int64_t lastEstimatedPathStampNs{-1};

    /*!
     * @brief       Minimum visual measurement variance floor.
     */
    double visualVariance{0.03};

    /*!
     * @brief       Deliberately loose variance applied to visual
     *              odometry's roll/pitch correction, so it anchors
     *              long-run IMU attitude drift without overriding the
     *              IMU's own better short-term estimate.
     */
    double visualAttitudeVariance{0.5};

    /*!
     * @brief       Minimum wheel measurement variance floor.
     */
    double wheelVariance{0.10};

    /*!
     * @brief           Primary sigma-point spread control, in (0, 1].
     *
     * @frame           N/A
     * @units           dimensionless
     */
    double sigmaAlpha{1.0e-3};

    /*!
     * @brief           Prior-knowledge weight on the central sigma point.
     *
     * @frame           N/A
     * @units           dimensionless
     */
    double sigmaBeta{2.0};

    /*!
     * @brief           Secondary sigma-point spread control.
     *
     * @frame           N/A
     * @units           dimensionless
     */
    double sigmaKappa{0.0};

    /*!
     * @brief           Known lunar gravitational-acceleration magnitude.
     *
     * @frame           N/A
     * @units           metres per second squared
     */
    double gravityMagnitude_mPs2{1.62};

    /*!
     * @brief           Physical gravitational acceleration.
     *
     * @frame           startup-fixed
     * @units           metres per second squared
     */
    Eigen::Vector3d gravityAcceleration_fixed_mPs2{
        Eigen::Vector3d(0.0, 0.0, -1.62)};

    /*!
     * @brief           Sum of stationary raw specific-force initialization
     *                  samples.
     *
     * @frame           body
     * @units           metres per second squared
     */
    Eigen::Vector3d initializationSpecificForceSum_body_mPs2{
        Eigen::Vector3d::Zero()};

    /*!
     * @brief           Sum of stationary raw angular-rate initialization
     *                  samples.
     *
     * @frame           body
     * @units           radians per second
     */
    Eigen::Vector3d initializationAngularVelocitySum_body_radPs{
        Eigen::Vector3d::Zero()};

    /*!
     * @brief           Required number of stationary initialization samples.
     *
     * @frame           N/A
     * @units           count
     */
    std::size_t imuInitializationSampleTarget{100U};

    /*!
     * @brief           Number of stationary initialization samples received.
     *
     * @frame           N/A
     * @units           count
     */
    std::size_t imuInitializationSampleCount{0U};

    /*!
     * @brief           Initial position covariance diagonal.
     *
     * @frame           startup-fixed
     * @units           square metres
     */
    double initialPositionVariance{0.01};

    /*!
     * @brief           Initial attitude covariance diagonal.
     *
     * @frame           body
     * @units           radians squared
     */
    double initialAttitudeVariance{0.01};

    /*!
     * @brief           Initial velocity covariance diagonal.
     *
     * @frame           startup-fixed
     * @units           (metres per second)^2
     */
    double initialVelocityVariance{0.01};

    /*!
     * @brief           Initial accelerometer-bias covariance diagonal.
     *
     * @frame           body
     * @units           (metres per second squared)^2
     */
    double initialAccelerometerBiasVariance{0.01};

    /*!
     * @brief           Initial gyroscope-bias covariance diagonal.
     *
     * @frame           body
     * @units           (radians per second)^2
     */
    double initialGyroscopeBiasVariance{0.001};

    /*!
     * @brief           Latest bias-corrected body angular velocity for
     *                  publication.
     *
     * @frame           body
     * @units           radians per second
     */
    Eigen::Vector3d latestAngularVelocity_body_radPs{Eigen::Vector3d::Zero()};

    /*!
     * @brief       Maximum accepted visual-odometry measurement age in seconds.
     */
    double maximumVisualMeasurementAgeS{0.75};

    /*!
     * @brief       Maximum accepted wheel-odometry measurement age in seconds.
     */
    double maximumWheelMeasurementAgeS{0.75};

    /*!
     * @brief           Largest lead of a measurement stamp over this node's
     *                  clock that is still fused.
     *
     * @frame           N/A
     * @units           seconds
     */
    double maximumFutureStamp_s{0.02};

    /*!
     * @brief           Maximum permitted age of a held IMU sample.
     *
     * @frame           N/A
     * @units           seconds
     */
    double maximumImuMeasurementAgeS{0.25};

    /*!
     * @brief           Visual update NIS override; zero selects the automatic
     *                  limit.
     *
     * @frame           N/A
     * @units           dimensionless
     */
    double visualNisThreshold{0.0};

    /*!
     * @brief           Wheel update NIS override; zero selects the automatic
     *                  limit.
     *
     * @frame           N/A
     * @units           dimensionless
     */
    double wheelNisThreshold{0.0};

    /*!
     * @brief           Whether visual pose channels are fused.
     *
     * @frame           N/A
     * @units           N/A
     */
    bool shouldFuseVisualPose{true};

    /*!
     * @brief           Whether wheel body-twist channels are fused.
     *
     * @frame           N/A
     * @units           N/A
     */
    bool shouldFuseWheelTwist{true};

    /*!
     * @brief           Diagnostics for raw IMU process input.
     *
     * @frame           N/A
     * @units           Mixed; see SourceDiagnostics
     */
    SourceDiagnostics imuDiagnostics;

    /*!
     * @brief           Diagnostics for visual odometry corrections.
     *
     * @frame           N/A
     * @units           Mixed; see SourceDiagnostics
     */
    SourceDiagnostics visualDiagnostics;

    /*!
     * @brief           Diagnostics for wheel odometry corrections.
     *
     * @frame           N/A
     * @units           Mixed; see SourceDiagnostics
     */
    SourceDiagnostics wheelDiagnostics;

    /*!
     * @brief       True once the filter's initial state and time origin are
     *              seeded.
     */
    bool hasInitialState{false};

    /*!
     * @brief       True once filter prediction has produced at least one
     *              posterior state.
     */
    bool hasEstimate{false};

    /*!
     * @brief           Number of diagnostics records between console health
     *                  lines: one line every five simulated seconds.
     *
     * @frame           N/A
     * @units           count
     */
    static constexpr std::uint64_t CONSOLE_HEALTH_PERIOD_TICKS = 5U;

    /*!
     * @brief           Number of diagnostics records published.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t diagnosticsRecordCount{0U};

    /*!
     * @brief           Visual fused count at the previous console health
     *                  line.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t previousConsoleVisualFusedCount{0U};

    /*!
     * @brief           Wheel fused count at the previous console health line.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t previousConsoleWheelFusedCount{0U};

    /*!
     * @brief           Visual and wheel rejections (age, NIS and numerical)
     *                  at the previous console health line.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t previousConsoleRejectedCount{0U};

    /*!
     * @brief           Visual updates that must be fused after
     *                  initialisation before this node reports itself
     *                  ready.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t readinessMinimumVisualUpdates{3U};

    /*!
     * @brief           Shortest time after initialisation before this node
     *                  reports itself ready.
     *
     * @frame           N/A
     * @units           seconds
     */
    double readinessSettleTimeS{5.0};

    /*!
     * @brief           Largest position-covariance trace for which this
     *                  node still reports itself ready (divergence guard).
     *
     * @frame           startup-fixed
     * @units           square metres
     */
    double readinessMaximumPositionVarianceM2{1.0};

    /*!
     * @brief           Stamp of the IMU sample that completed the latest
     *                  initialisation.
     *
     * @frame           N/A
     * @units           ROS seconds
     */
    double initializationTimestamp_s{0.0};

    /*!
     * @brief           Visual fused count when the filter was last
     *                  initialised, so readiness counts only updates since.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t visualFusedCountAtInitialization{0U};
};

} /* namespace systems::alpha::alpha_localisation::alpha_ukf_filter */

#endif /* SRS_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_UKF_FILTER_NODE_CLASS_H */
