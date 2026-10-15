/*!
 * @File:         KeyframeWindowClass.h
 *
 * @Brief:        Declares the sliding keyframe window with landmark
 *                association.
 *
 * @Date:         08/10/2026
 *
 */

#ifndef SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_WINDOW_CLASS_H
#define SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_WINDOW_CLASS_H

/* Object Includes */
#include "objects/BundleAdjustmentConfigurationStruct.h"
#include "objects/KeyframeLocalOptimizationLimits.h"
#include "objects/KeyframeLocalOptimizationStatusEnum.h"
#include "objects/KeyframeSampleStruct.h"
#include "objects/KeyframeWindowConfigurationStruct.h"

/* Data include */
/* None */

/* Generic Libraries */
#include <array>
#include <cstddef>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace localisation::keyframe_local_optimization
{

/*!
 * @brief           Forward declaration of the Full BA solver, which
 *                  refines this window's poses and landmarks in place.
 */
class BundleAdjustmentOptimizer;

/*!
 * @brief           Sliding window of keyframes with associated 3-D
 *                  landmarks.
 *
 *                  Fixed-cap ring (MAX_WINDOW_KEYFRAMES, never reallocated)
 *                  plus a fixed landmark table (MAX_LANDMARKS) with a
 *                  free-list, so pushKeyframe() never allocates. New cloud
 *                  points associate to the nearest world-frame landmark inside
 *                  associationGate_m, else seed a new landmark while the table
 *                  has room; overflow drops the oldest keyframe first and
 *                  recycles its orphaned landmarks. Construction never fails;
 *                  initialize() validates the window, and every fallible call
 *                  returns a checked KeyframeLocalOptimizationStatus. Not
 *                  thread-safe: the owning node calls it from one callback
 *                  group only.
 */
class KeyframeWindow
{
  public:
    /*!
     * @brief           Grants the Full BA solver in-place refinement.
     */
    friend class BundleAdjustmentOptimizer;

    /*!
     * @brief           Establishes a valid empty uninitialized state.
     */
    KeyframeWindow() = default;

    KeyframeWindow(const KeyframeWindow &otherWindow_in)            = delete;
    KeyframeWindow &operator=(const KeyframeWindow &otherWindow_in) = delete;
    KeyframeWindow(KeyframeWindow &&otherWindow_in)                 = delete;
    KeyframeWindow &operator=(KeyframeWindow &&otherWindow_in)      = delete;

    /*!
     * @brief           Validates the window configuration.
     *
     * @param[in]       configuration_in
     *                  Window tuning; windowSize must lie in
     *                  1..MAX_WINDOW_KEYFRAMES and associationGate_m
     *                  must be finite and positive.
     *
     * @return          SUCCESS, or INVALID_CONFIGURATION (state left
     *                  uninitialized).
     */
    [[nodiscard]] KeyframeLocalOptimizationStatus
        initialize(const KeyframeWindowConfiguration &configuration_in);

    /*!
     * @brief           Releases the window to its uninitialized state.
     *
     * @return          SUCCESS, or NOT_INITIALIZED.
     */
    [[nodiscard]] KeyframeLocalOptimizationStatus terminate();

    /*!
     * @brief           Inserts one keyframe, associating its points.
     *
     *                  Overflow drops the oldest keyframe first and
     *                  recycles landmarks it alone observed. Points
     *                  beyond a full landmark table are ignored.
     *
     * @param[in]       sample_in
     *                  Source-agnostic keyframe pose plus keyframe-
     *                  local observed points.
     *
     * @return          SUCCESS, WINDOW_OVERFLOW (window still valid),
     *                  NOT_INITIALIZED, or INSUFFICIENT_DATA when the
     *                  pose was marked unavailable or the stamp,
     *                  position, or orientation was non-finite
     *                  (stale or non-finite input is never silently
     *                  coasted).
     */
    [[nodiscard]] KeyframeLocalOptimizationStatus
        pushKeyframe(const KeyframeSample &sample_in);

    /*!
     * @brief           Counts the keyframes currently stored.
     *
     * @return          Stored keyframe count (0 .. windowSize), or 0
     *                  before initialize().
     */
    std::size_t keyframeCount() const;

    /*!
     * @brief           Counts the landmarks currently stored.
     *
     * @return          Stored landmark count (0 .. MAX_LANDMARKS), or
     *                  0 before initialize().
     */
    std::size_t landmarkCount() const;

    /*!
     * @brief           Sums the Huber-weighted squared 3-D residuals.
     *
     * @param[in]       configuration_in
     *                  Robustifier tuning (useHuber, huberDelta_m).
     * @param[out]      totalError_out
     *                  Weighted squared error in metres squared;
     *                  untouched unless SUCCESS.
     *
     * @return          SUCCESS, NOT_INITIALIZED, or
     *                  INSUFFICIENT_DATA (fewer than one observation).
     */
    [[nodiscard]] KeyframeLocalOptimizationStatus
        computeTotalError(const BundleAdjustmentConfiguration &configuration_in,
                          double &totalError_out) const;

    /*!
     * @brief           Copies the newest stored keyframe.
     *
     * @param[out]      sample_out
     *                  Newest keyframe pose, twist, and covariance
     *                  (no local points); untouched unless SUCCESS.
     *
     * @return          SUCCESS, NOT_INITIALIZED, or
     *                  INSUFFICIENT_DATA (empty window).
     */
    [[nodiscard]] KeyframeLocalOptimizationStatus
        newestKeyframe(KeyframeSample &sample_out) const;

    /*!
     * @brief           Exports live landmark positions in table order.
     *
     * @param[out]      p_positions_out
     *                  Caller-owned array with room for at least
     *                  capacity_in entries; never null.
     * @param[in]       capacity_in
     *                  Entries available at p_positions_out.
     * @param[out]      exportedCount_out
     *                  Landmarks written; untouched unless SUCCESS.
     *
     * @return          SUCCESS, NOT_INITIALIZED, or
     *                  INVALID_CONFIGURATION when capacity_in is
     *                  smaller than the live landmark count.
     */
    [[nodiscard]] KeyframeLocalOptimizationStatus
        exportLandmarks(Eigen::Vector3d *p_positions_out,
                        std::size_t      capacity_in,
                        std::size_t     &exportedCount_out) const;

  private:
    /*!
     * @brief       One stored keyframe with landmark associations.
     */
    struct StoredKeyframe
    {
        /*! Keyframe stamp in seconds. */
        double stamp_s{0.0};

        /*! Keyframe position in the odom frame, in metres. */
        Eigen::Vector3d position_fixed_m{Eigen::Vector3d::Zero()};

        /*! Keyframe orientation, odom frame to body frame. */
        Eigen::Quaterniond orientation_fixedToBody{
            Eigen::Quaterniond::Identity()};

        /*! Pose covariance, translation then rotation. */
        Eigen::Matrix<double, 6, 6> poseCovariance{
            Eigen::Matrix<double, 6, 6>::Identity()};

        /*! Body-frame linear twist carried for republication. */
        Eigen::Vector3d twistLinear_body_mPs{Eigen::Vector3d::Zero()};

        /*! Body-frame angular twist carried for republication. */
        Eigen::Vector3d twistAngular_body_radPs{Eigen::Vector3d::Zero()};

        /*! Landmark index per observation below. */
        std::array<std::size_t, MAX_OBSERVATIONS_PER_KEYFRAME>
            landmarkIndices{};

        /*! Keyframe-local observed point per observation below. */
        std::array<Eigen::Vector3d, MAX_OBSERVATIONS_PER_KEYFRAME>
            localPoints{};

        /*! Valid entries in landmarkIndices/localPoints. */
        std::size_t observationCount{0U};
    };

    /*!
     * @brief       True once initialize() accepted a configuration.
     */
    bool isInitialized{false};

    /*!
     * @brief       Accepted window tuning.
     */
    KeyframeWindowConfiguration configuration{};

    /*!
     * @brief       Fixed keyframe ring; only the first
     *              storedKeyframeCount entries from ringStartIndex are
     *              valid.
     */
    std::array<StoredKeyframe, MAX_WINDOW_KEYFRAMES> keyframeRing{};

    /*!
     * @brief       Valid keyframes in keyframeRing.
     */
    std::size_t storedKeyframeCount{0U};

    /*!
     * @brief       Ring index of the oldest stored keyframe.
     */
    std::size_t ringStartIndex{0U};

    /*!
     * @brief       World-frame landmark positions by landmark index.
     */
    std::array<Eigen::Vector3d, MAX_LANDMARKS> landmarkPositions{};

    /*!
     * @brief       Live observation count per landmark index.
     */
    std::array<std::size_t, MAX_LANDMARKS> landmarkObservationCounts{};

    /*!
     * @brief       Recycled landmark indices ready for reuse.
     */
    std::array<std::size_t, MAX_LANDMARKS> freeLandmarkIndices{};

    /*!
     * @brief       Entries used in freeLandmarkIndices.
     */
    std::size_t freeLandmarkCount{0U};

    /*!
     * @brief       Highest landmark index ever issued plus one.
     */
    std::size_t landmarkHighWaterMark{0U};
};

} /* namespace localisation::keyframe_local_optimization */

#endif /* SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_WINDOW_CLASS_H */
