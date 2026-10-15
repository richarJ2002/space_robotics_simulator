/*!
 * @file            OrbCornerDetector.h
 *
 * @brief           Declares an OpenCV ORB corner detector with the same
 *                  output shape as the dependency-free GFTT detector.
 *
 * @date            07/10/2026
 */

#ifndef SRS_LOCALISATION_ORB_CORNER_DETECTOR_H
#define SRS_LOCALISATION_ORB_CORNER_DETECTOR_H

/* C++ Standard Library Includes */
#include <array>
#include <cstddef>

/* External Library Includes */
#include <opencv2/features2d.hpp>

/* Object Include */
#include "feature_tracking/gftt_lk/objects/FeatureTrackingLimits.h"
#include "feature_tracking/gftt_lk/objects/FeatureTrackingStatus.h"
#include "feature_tracking/gftt_lk/objects/ImageView.h"
#include "feature_tracking/gftt_lk/objects/Point2D.h"

namespace localisation::visual_odometry::feature_tracking::orb_lk
{

/*!
 * @brief           Detects corner-like features with OpenCV's ORB
 *                  detector, reporting them in the same fixed-capacity
 *                  `Point2D` shape the GFTT detector uses.
 *
 * Lifecycle: construct (non-failing), `initialize()` once with the
 * fixed image resolution and ORB tuning, then any number of
 * `detect()` calls, then `terminate()`. Not copyable, not movable
 * (owns a configured `cv::ORB` instance sized by `initialize()`),
 * not thread-safe, and intended for a single-threaded caller,
 * matching `VisualOdometryNode`'s own documented executor
 * assumption.
 *
 * This class lives outside the JSF AV applicability profile
 * (`docs/compliance/feature_tracking/` covers
 * `feature_tracking/gftt_lk/` only): it depends on OpenCV and
 * allocates inside `detect()` through OpenCV's own internals.
 */
class OrbCornerDetector
{
  public:
    /*!
     * @brief           Creates a valid, uninitialized detector.
     */
    OrbCornerDetector() noexcept = default;

    /*!
     * @brief           Releases the owned ORB instance, if any was
     *                  created by `initialize()`.
     */
    ~OrbCornerDetector() noexcept = default;

    OrbCornerDetector(const OrbCornerDetector &otherDetector_in) = delete;
    OrbCornerDetector &operator=(
        const OrbCornerDetector &otherDetector_in) = delete;
    OrbCornerDetector(OrbCornerDetector &&otherDetector_in) = delete;
    OrbCornerDetector &operator=(
        OrbCornerDetector &&otherDetector_in) = delete;

    /* ---------------------------------------------------------------------- *
     * PUBLIC METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Creates the owned ORB detector for a fixed image
     *                  resolution and stores the detection tuning used by
     *                  every later `detect()` call.
     *
     * Calling `initialize()` again after a prior successful call
     * discards the previous ORB instance and creates a new one for
     * the new configuration.
     *
     * @param[in]       width_in
     *                  Fixed image width, in pixels, every later
     *                  `detect()` call's `ImageView` must match exactly.
     * @param[in]       height_in
     *                  Fixed image height, in pixels, every later
     *                  `detect()` call's `ImageView` must match exactly.
     * @param[in]       maximumFeatures_in
     *                  Upper bound on features accepted per `detect()`
     *                  call; must not exceed `MAXIMUM_SUPPORTED_FEATURES`.
     * @param[in]       scaleFactor_in
     *                  ORB pyramid decimation ratio between levels; must
     *                  be finite and greater than 1.
     * @param[in]       levels_in
     *                  ORB pyramid depth; must be in `[1, 16]`.
     * @param[in]       fastThreshold_in
     *                  FAST acceptance threshold; must be positive.
     *
     * @return          Lifecycle status.
     */
    [[nodiscard]] feature_tracking::gftt_lk::FeatureTrackingStatus initialize(
        int         width_in,
        int         height_in,
        std::size_t maximumFeatures_in,
        double      scaleFactor_in,
        int         levels_in,
        int         fastThreshold_in) noexcept;

    /*!
     * @brief           Detects corner-like features in one image.
     *
     * @param[in]       image_in
     *                  Image to detect features in; its `width`/`height`
     *                  must exactly match the values `initialize()` was
     *                  called with.
     * @param[out]      features_out
     *                  Detected feature positions, valid only in
     *                  `[0, featureCount_out)`, in the order OpenCV
     *                  returns them.
     * @param[out]      featureCount_out
     *                  Number of valid entries written to `features_out`.
     *
     * @return          Operation status.
     */
    [[nodiscard]] feature_tracking::gftt_lk::FeatureTrackingStatus detect(
        const feature_tracking::gftt_lk::ImageView &image_in,
        std::array<feature_tracking::gftt_lk::Point2D,
                   feature_tracking::gftt_lk::MAXIMUM_SUPPORTED_FEATURES>
            &features_out,
        std::size_t &featureCount_out) noexcept;

    /*!
     * @brief           Releases the owned ORB instance and returns to
     *                  the uninitialized lifecycle state.
     *
     * @return          Lifecycle status (always success).
     */
    [[nodiscard]] feature_tracking::gftt_lk::FeatureTrackingStatus
        terminate() noexcept;

    /* ---------------------------------------------------------------------- *
     * PUBLIC MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       Upper bound on the number of features `detect()` may
     *              report in one call; every output buffer is sized to
     *              this constant.
     */
    static constexpr std::size_t MAXIMUM_SUPPORTED_FEATURES =
        feature_tracking::gftt_lk::MAXIMUM_SUPPORTED_FEATURES;

    /*!
     * @brief       ORB border within which features are not detected, in
     *              pixels; matches the OpenCV default.
     */
    static constexpr int EDGE_THRESHOLD_PX = 31;

    /*!
     * @brief       ORB descriptor patch side length, in pixels; matches
     *              the OpenCV default.
     */
    static constexpr int PATCH_SIZE_PX = 31;

  private:
    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       True once `initialize()` has succeeded and `terminate()`
     *              has not yet been called.
     */
    bool isInitialized_{false};

    /*!
     * @brief       Image width `initialize()` was called with, in pixels.
     */
    int width_{0};

    /*!
     * @brief       Image height `initialize()` was called with, in
     *              pixels.
     */
    int height_{0};

    /*!
     * @brief       Configured cap on features accepted per `detect()`
     *              call.
     */
    std::size_t maximumFeatures_{0U};

    /*!
     * @brief       Owned OpenCV ORB detector created by `initialize()`.
     */
    cv::Ptr<cv::ORB> p_orbDetector;
};

} /* namespace localisation::visual_odometry::feature_tracking::orb_lk */

#endif /* SRS_LOCALISATION_ORB_CORNER_DETECTOR_H */
