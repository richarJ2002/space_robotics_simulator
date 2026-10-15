/*!
 * @File:         KeyframeLocalOptimizationNodeClass.h
 *
 * @Brief:        Declares the sliding-window keyframe Full BA node.
 *
 * @Date:         08/10/2026
 *
 */

#ifndef SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_NODE_CLASS_H
#define SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_NODE_CLASS_H

/* Function Includes */
#include "console/console.h"

/* Object Include */
#include "objects/BundleAdjustmentConfigurationStruct.h"
#include "objects/BundleAdjustmentOptimizerClass.h"
#include "objects/KeyframeLocalOptimizationLimits.h"
#include "objects/KeyframeLocalOptimizationStatusEnum.h"
#include "objects/KeyframeSampleStruct.h"
#include "objects/KeyframeWindowClass.h"
#include "objects/KeyframeWindowConfigurationStruct.h"

/* Data include */
#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

/* Generic Libraries */
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>

namespace localisation::keyframe_local_optimization
{

/*!
 * @brief           Refines source-agnostic keyframes with sliding-window
 *                  Full bundle adjustment, comparison-only.
 *
 *                  Keyframe poses arrive as nav_msgs Odometry (visual
 *                  odometry or the EKF, selected by input_source) and
 *                  feature observations as the visual PointCloud2,
 *                  paired by message_filters ApproximateTime with the
 *                  cloud tied to the pose source: ekf mode is
 *                  pose-only (no cloud subscription, no ApproximateTime,
 *                  no point_cloud/features publication) until a fused
 *                  cloud exists, so ekf pose plus visual cloud mixing
 *                  is forbidden. Each accepted pair enters the
 *                  KeyframeWindow; trigger_mode count optimizes every
 *                  time the window fills, time optimizes on a sim-time
 *                  period, and offline-first never auto-optimizes. The
 *                  Eigen-only BundleAdjustmentOptimizer refines poses
 *                  plus landmarks in place, published as odometry,
 *                  point_cloud, and a landmark-scatter features image
 *                  (paired mode only: pose-only inputs carry no
 *                  observations, so the optimizer stays
 *                  INSUFFICIENT_DATA and no odometry is published).
 *                  Comparison-only: outputs are never added to EKF
 *                  fuse_* and this node's diagnostics never gate READY
 *                  (startup_supervisor.yaml unchanged). Stale input
 *                  marks the pose unavailable instead of coasting.
 *                  No TF broadcast, no map re-anchor, no direct /clock
 *                  subscription; timers run on use_sim_time.
 *                  Callbacks run in the node's mutually exclusive
 *                  default callback group, which is the only
 *                  synchronization this integration needs.
 */
class KeyframeLocalOptimizationNode final : public rclcpp::Node
{
  public:
    /*!
     * @brief           Pairing policy for pose-plus-cloud keyframes.
     */
    using PairedPolicy = message_filters::sync_policies::
        ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;

    /*!
     * @brief           Declares parameters and wires keyframe I/O.
     *
     * @throws          std::invalid_argument if window_size leaves
     *                  1..MAX_WINDOW_KEYFRAMES, max_iterations is
     *                  zero, trigger_mode/input_source leave their
     *                  allow-lists, an explicit topic override is
     *                  inconsistent with input_source or outside the
     *                  allow-list, or any age/queue/period bound is
     *                  invalid.
     */
    KeyframeLocalOptimizationNode() :
        Node("keyframe_local_optimization")
    {
        /*!
         * Declared first so the topic defaults below can be rooted at
         * the owning system's namespace.
         */
        const std::string systemName =
            declare_parameter<std::string>("system_name", "alpha");

        /* Keyframe pose producer: visual odometry or the EKF. */
        const std::string inputSource =
            declare_parameter<std::string>("input_source", "visual");
        if ((inputSource != "visual") && (inputSource != "ekf"))
        {
            throw std::invalid_argument("input_source must be visual or ekf");
        }
        isPoseOnly = (inputSource == "ekf");

        /* Optimization trigger: count, time, or offline-first. */
        const std::string triggerMode =
            declare_parameter<std::string>("trigger_mode", "count");
        if ((triggerMode != "count") && (triggerMode != "time") &&
            (triggerMode != "offline-first"))
        {
            throw std::invalid_argument(
                "trigger_mode must be count, time or offline-first");
        }
        triggerModeName = triggerMode;

        /* Allow-listed keyframe pose topics for this system. */
        const std::string visualOdomTopic =
            "/" + systemName + "/localisation/visual/odometry";
        const std::string ekfOdomTopic =
            "/" + systemName + "/localisation/kalman_filter/odometry";
        const std::string expectedOdomTopic =
            isPoseOnly ? ekfOdomTopic : visualOdomTopic;

        /* An explicit odometry override must name exactly the topic
         * its input_source selects; anything else (including the
         * other source, ground-truth topics, or inertial filtered
         * IMU) throws instead of silently mixing estimators. */
        const std::string configuredOdomTopic =
            declare_parameter<std::string>("input_odom_topic", "");
        const std::string inputOdomTopic = configuredOdomTopic.empty()
                                               ? expectedOdomTopic
                                               : configuredOdomTopic;
        if (inputOdomTopic != expectedOdomTopic)
        {
            throw std::invalid_argument(
                "input_odom_topic is inconsistent with input_source");
        }

        /* Allow-listed feature-observation topic for this system. */
        const std::string defaultCloudTopic =
            "/" + systemName + "/localisation/visual/point_cloud";
        const std::string configuredCloudTopic =
            declare_parameter<std::string>("input_cloud_topic", "");
        if (!configuredCloudTopic.empty() &&
            (configuredCloudTopic != defaultCloudTopic))
        {
            throw std::invalid_argument(
                "input_cloud_topic must name the visual point_cloud");
        }
        const std::string inputCloudTopic = configuredCloudTopic.empty()
                                                ? defaultCloudTopic
                                                : configuredCloudTopic;

        /* Output topic: the newest optimized keyframe pose. */
        const std::string outputOdomTopic = declare_parameter<std::string>(
            "output_odom_topic",
            "/" + systemName + "/localisation/keyframe_optimized/odometry");

        /* Output topic: the optimized landmark cloud. */
        const std::string outputCloudTopic = declare_parameter<std::string>(
            "output_cloud_topic",
            "/" + systemName + "/localisation/keyframe_optimized/point_cloud");

        /* Output topic: a landmark-scatter features image. */
        const std::string outputFeatureTopic = declare_parameter<std::string>(
            "output_feature_topic",
            "/" + systemName + "/localisation/keyframe_optimized/features");

        /* Output topic: periodic optimizer status, shared by every
         * node of the owning system and recorded with every run. */
        const std::string diagnosticsTopic =
            declare_parameter<std::string>("diagnostics_topic",
                                           "/" + systemName + "/diagnostics");

        /* Fixed world frame shared with every other node. */
        odomFrame =
            declare_parameter<std::string>("odom_frame",
                                           systemName + "/startup_fixed");

        /* Rover body frame this node's odometry describes. */
        baseFrame =
            declare_parameter<std::string>("base_frame", "alpha/base_link");

        /* Keyframes retained; the solver caps at MAX_WINDOW_KEYFRAMES. */
        const std::int64_t configuredWindowSize =
            declare_parameter<std::int64_t>("window_size", 5);
        if ((configuredWindowSize < 1) ||
            (configuredWindowSize >
             static_cast<std::int64_t>(MAX_WINDOW_KEYFRAMES)))
        {
            throw std::invalid_argument(
                "window_size must lie in 1..MAX_WINDOW_KEYFRAMES");
        }
        windowConfiguration.windowSize =
            static_cast<std::size_t>(configuredWindowSize);

        /* Gauss-Newton iterations per optimization. */
        const std::int64_t configuredIterations =
            declare_parameter<std::int64_t>("max_iterations", 10);
        if (configuredIterations < 1)
        {
            throw std::invalid_argument("max_iterations must be at least 1");
        }
        solverConfiguration.maximumIterations =
            static_cast<std::size_t>(configuredIterations);

        /* Huber robustifier switch and transition distance. */
        solverConfiguration.useHuber =
            declare_parameter<bool>("use_huber", true);
        solverConfiguration.huberDelta_m =
            declare_parameter<double>("huber_delta_m", 0.03);
        if (solverConfiguration.useHuber &&
            (solverConfiguration.huberDelta_m <= 0.0))
        {
            throw std::invalid_argument("huber_delta_m must be positive");
        }

        /* Nearest-neighbour association gate in metres. */
        windowConfiguration.associationGate_m =
            declare_parameter<double>("association_gate_m", 0.05);
        if (windowConfiguration.associationGate_m <= 0.0)
        {
            throw std::invalid_argument("association_gate_m must be positive");
        }

        /* Post-sync age discard mirroring the VO input age. */
        maximumInputAge_s =
            declare_parameter<double>("maximum_input_age_s", 0.25);
        if (maximumInputAge_s <= 0.0)
        {
            throw std::invalid_argument("maximum_input_age_s must be positive");
        }

        /* Pairing queue depth mirroring VO image_queue_depth. */
        const std::int64_t configuredQueueDepth =
            declare_parameter<std::int64_t>("input_queue_depth", 2);
        if ((configuredQueueDepth < 1) || (configuredQueueDepth > 10))
        {
            throw std::invalid_argument("input_queue_depth must be 1 to 10");
        }
        const std::size_t inputQueueDepth =
            static_cast<std::size_t>(configuredQueueDepth);

        /* Time-trigger period in seconds (trigger_mode time only). */
        triggerPeriod_s = declare_parameter<double>("trigger_period_s", 1.0);
        if (triggerPeriod_s <= 0.0)
        {
            throw std::invalid_argument("trigger_period_s must be positive");
        }

        /* Engine storage is heap-sized once here, never per message. */
        const KeyframeLocalOptimizationStatus windowStatus =
            keyframeWindow.initialize(windowConfiguration);
        const KeyframeLocalOptimizationStatus solverStatus =
            bundleOptimizer.initialize(solverConfiguration);
        const bool isEngineReady =
            (windowStatus == KeyframeLocalOptimizationStatus::
                                 KEYFRAME_LOCAL_OPTIMIZATION_SUCCESS) &&
            (solverStatus == KeyframeLocalOptimizationStatus::
                                 KEYFRAME_LOCAL_OPTIMIZATION_SUCCESS);
        if (!isEngineReady)
        {
            throw std::invalid_argument(
                "keyframe window or solver configuration invalid");
        }

        /* Optimized keyframe pose, reliable so the recorder captures
         * it with no QoS override. */
        p_odometryPublisher = create_publisher<nav_msgs::msg::Odometry>(
            outputOdomTopic,
            rclcpp::QoS(10).reliable());

        /* Sensor subscription QoS mirroring the EKF subscribers. */
        const rmw_qos_profile_t sensorQos = rclcpp::SensorDataQoS()
                                                .keep_last(inputQueueDepth)
                                                .get_rmw_qos_profile();

        if (isPoseOnly)
        {
            /* EKF pose-only mode: no cloud subscription, no
             * ApproximateTime, and input_cloud_topic ignored until a
             * fused-cloud source exists. */
            p_poseOnlySubscription =
                create_subscription<nav_msgs::msg::Odometry>(
                    inputOdomTopic,
                    rclcpp::SensorDataQoS().keep_last(inputQueueDepth),
                    [this](nav_msgs::msg::Odometry::ConstSharedPtr p_message_in)
                    { handlePoseOnlyCallBack(*p_message_in); });

            /* Pose-only mode publishes no cloud or features image. */
            SRS_LOG_DEBUG(get_logger(),
                          "Keyframe BA pose-only: %s -> %s",
                          inputOdomTopic.c_str(),
                          outputOdomTopic.c_str());
        }
        else
        {
            /* Optimized landmarks, reliable like the visual cloud. */
            p_pointCloudPublisher =
                create_publisher<sensor_msgs::msg::PointCloud2>(
                    outputCloudTopic,
                    rclcpp::QoS(10).reliable());

            /* Landmark-scatter image, reliable like visual/features
             * and recorded only under --record-images. */
            p_featureImagePublisher = create_publisher<sensor_msgs::msg::Image>(
                outputFeatureTopic,
                rclcpp::QoS(10).reliable());

            /* Feed the selected odometry into the pair synchronizer. */
            odometrySubscriber.subscribe(this, inputOdomTopic, sensorQos);

            /* Feed the visual cloud into the pair synchronizer. */
            cloudSubscriber.subscribe(this, inputCloudTopic, sensorQos);

            /* Pair poses with the cloud observed alongside them. */
            p_synchronizer =
                std::make_unique<message_filters::Synchronizer<PairedPolicy>>(
                    PairedPolicy(static_cast<std::uint32_t>(inputQueueDepth)),
                    odometrySubscriber,
                    cloudSubscriber);

            /* Every synchronized pose-cloud pair triggers
             * handlePairedInputCallBack(). A lambda does not satisfy
             * message_filters::Synchronizer's own template-deduced
             * callback-signature matching here; std::bind is kept
             * deliberately. */
            p_synchronizer->registerCallback(
                // NOLINTNEXTLINE(modernize-avoid-bind)
                std::bind(
                    &KeyframeLocalOptimizationNode::handlePairedInputCallBack,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

            /* Topic wiring is already captured by the run's parameter
             * snapshot, so it is debug detail. */
            SRS_LOG_DEBUG(get_logger(),
                          "Keyframe BA paired: [%s, %s] -> [%s, %s, %s]",
                          inputOdomTopic.c_str(),
                          inputCloudTopic.c_str(),
                          outputOdomTopic.c_str(),
                          outputCloudTopic.c_str(),
                          outputFeatureTopic.c_str());
        }

        /* Time trigger runs the window on a sim-time period. */
        if (triggerMode == "time")
        {
            const std::chrono::duration<double> triggerPeriod(triggerPeriod_s);
            p_triggerTimer = create_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    triggerPeriod),
                [this]() { runOptimization(); });
        }

        /* Report optimizer status once per simulated second, on the
         * same clock as every other recorded topic. */
        p_diagnosticsPublisher =
            create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                diagnosticsTopic,
                rclcpp::QoS(10));
        p_diagnosticsTimer =
            create_timer(std::chrono::seconds(1),
                         [this]() { publishDiagnosticsCallBack(); });

        /* Comparison-only: never fused, never gates READY. */
        SRS_LOG_INFO(get_logger(),
                     "Keyframe BA ready: %s mode, window %zu",
                     triggerMode.c_str(),
                     windowConfiguration.windowSize);
    }

    KeyframeLocalOptimizationNode(
        const KeyframeLocalOptimizationNode &otherNode_in) = delete;
    KeyframeLocalOptimizationNode &
        operator=(const KeyframeLocalOptimizationNode &otherNode_in) = delete;
    KeyframeLocalOptimizationNode(
        KeyframeLocalOptimizationNode &&otherNode_in) = delete;
    KeyframeLocalOptimizationNode &
        operator=(KeyframeLocalOptimizationNode &&otherNode_in) = delete;

    /*!
     * @brief           Accepts one synchronized pose-cloud pair.
     *
     *                  Discards pairs older than maximum_input_age_s
     *                  or whose cloud frame differs from odom_frame
     *                  (marking the pose unavailable, never coasting),
     *                  lifts the cloud into the keyframe-local frame,
     *                  pushes the keyframe, and optimizes when the
     *                  window fills in count mode.
     *
     * @param[in]       odometry_in
     *                  Keyframe pose with odometry covariances.
     * @param[in]       cloud_in
     *                  Feature observations in the odom frame.
     */
    void handlePairedInputCallBack(
        const nav_msgs::msg::Odometry::ConstSharedPtr       &p_odometry_in,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &p_cloud_in);

    /*!
     * @brief           Accepts one EKF pose in pose-only mode.
     *
     *                  Pushes a keyframe with no observations and
     *                  optimizes when the window fills in count mode.
     *
     * @param[in]       odometry_in
     *                  Keyframe pose with odometry covariances.
     */
    void handlePoseOnlyCallBack(const nav_msgs::msg::Odometry &odometry_in);

    /*!
     * @brief           Runs one Full BA over the window and publishes.
     *
     *                  Needs at least two keyframes with observations;
     *                  publishes the newest optimized pose on success,
     *                  plus the landmark cloud and scatter image outside
     *                  pose-only mode. Pose-only inputs carry no
     *                  observations, so optimization stays
     *                  INSUFFICIENT_DATA and nothing is published.
     */
    void runOptimization();

    /*!
     * @brief           Publishes the newest optimized keyframe pose.
     */
    void publishOdometry();

    /*!
     * @brief           Publishes the optimized landmark cloud.
     *
     * @param[in]       stamp_in
     *                  Newest keyframe stamp for the cloud header.
     */
    void publishPointCloud(const builtin_interfaces::msg::Time &stamp_in);

    /*!
     * @brief           Publishes the landmark-scatter features image.
     *
     * @param[in]       stamp_in
     *                  Newest keyframe stamp for the image header.
     */
    void publishFeatureImage(const builtin_interfaces::msg::Time &stamp_in);

    /*!
     * @brief           Publishes periodic optimizer diagnostics.
     */
    void publishDiagnosticsCallBack();

    /*!
     * @brief           Converts odometry into a keyframe sample.
     *
     *                  Copies stamp, pose, covariance, and twist; the
     *                  caller fills localPoints afterwards. A
     *                  degenerate orientation falls back to identity
     *                  rather than injecting NaN.
     *
     * @param[in]       odometry_in
     *                  Keyframe pose with odometry covariances.
     * @param[out]      sample_out
     *                  Converted sample with no local points yet.
     */
    void buildKeyframeSample(const nav_msgs::msg::Odometry &odometry_in,
                             KeyframeSample                &sample_out);

  private:
    /*!
     * @brief       Sliding keyframe window (heap sized at ctor).
     */
    KeyframeWindow keyframeWindow;

    /*!
     * @brief       Full BA solver (heap sized at ctor).
     */
    BundleAdjustmentOptimizer bundleOptimizer;

    /*!
     * @brief       Accepted window tuning.
     */
    KeyframeWindowConfiguration windowConfiguration{};

    /*!
     * @brief       Accepted solver tuning.
     */
    BundleAdjustmentConfiguration solverConfiguration{};

    /*!
     * @brief       Fixed world frame of poses and landmarks.
     */
    std::string odomFrame;

    /*!
     * @brief       Rover body frame of the published odometry.
     */
    std::string baseFrame;

    /*!
     * @brief       Post-sync age discard in seconds.
     *
     * @units       Seconds.
     */
    double maximumInputAge_s{0.25};

    /*!
     * @brief       Time-trigger period in seconds.
     *
     * @units       Seconds.
     */
    double triggerPeriod_s{1.0};

    /*!
     * @brief       True in ekf pose-only mode (no cloud I/O).
     */
    bool isPoseOnly{false};

    /*!
     * @brief       Accepted trigger policy (count, time, offline-first).
     */
    std::string triggerModeName{"count"};

    /*!
     * @brief       False once stale input marks the pose unavailable.
     */
    bool isPoseAvailable{true};

    /*!
     * @brief       Received pairs (or poses in pose-only mode).
     */
    std::uint64_t receivedInputCount{0U};

    /*!
     * @brief       Pairs discarded as stale past maximum_input_age_s
     *              or with a cloud frame outside odom_frame.
     */
    std::uint64_t staleDiscardedCount{0U};

    /*!
     * @brief       Completed optimizations.
     */
    std::uint64_t optimizationCount{0U};

    /*!
     * @brief       Robust cost after the latest optimization.
     *
     * @units       Metres squared.
     */
    double latestCost_m2{0.0};

    /*!
     * @brief       Wall duration of the latest optimization.
     *
     * @units       Milliseconds.
     */
    double latestOptimizationMs{0.0};

    /*!
     * @brief       Width of the scatter image in pixels.
     */
    static constexpr std::size_t SCATTER_IMAGE_WIDTH_PX = 96U;

    /*!
     * @brief       Height of the scatter image in pixels.
     */
    static constexpr std::size_t SCATTER_IMAGE_HEIGHT_PX = 64U;

    /*!
     * @brief       Scratch landmark export buffer (member, never stack).
     */
    std::array<Eigen::Vector3d, MAX_LANDMARKS> scratchLandmarks{};

    /*!
     * @brief       Optimized odometry publisher.
     */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr p_odometryPublisher;

    /*!
     * @brief       Optimized landmark-cloud publisher (paired mode).
     */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        p_pointCloudPublisher;

    /*!
     * @brief       Landmark-scatter image publisher (paired mode).
     */
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
        p_featureImagePublisher;

    /*!
     * @brief       Diagnostics publisher shared with every node.
     */
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        p_diagnosticsPublisher;

    /*!
     * @brief       Pose-only odometry subscription (ekf mode).
     */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        p_poseOnlySubscription;

    /*!
     * @brief       Selected odometry feed of the pair synchronizer.
     */
    message_filters::Subscriber<nav_msgs::msg::Odometry> odometrySubscriber;

    /*!
     * @brief       Visual cloud feed of the pair synchronizer.
     */
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloudSubscriber;

    /*!
     * @brief       Pose-cloud ApproximateTime synchronizer.
     */
    std::unique_ptr<message_filters::Synchronizer<PairedPolicy>> p_synchronizer;

    /*!
     * @brief       Time-trigger optimization timer (time mode only).
     */
    rclcpp::TimerBase::SharedPtr p_triggerTimer;

    /*!
     * @brief       Periodic diagnostics timer.
     */
    rclcpp::TimerBase::SharedPtr p_diagnosticsTimer;
};

} /* namespace localisation::keyframe_local_optimization */

#endif /* SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_NODE_CLASS_H */
