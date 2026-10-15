/*!
 * @File:         test_orb_detector.cpp
 *
 * @Brief:        Unit tests for OrbCornerDetector.
 *
 * @Date:         07/10/2026
 *
 */

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "feature_tracking/gftt_lk/objects/FeatureTrackingStatus.h"
#include "feature_tracking/gftt_lk/objects/ImageView.h"
#include "feature_tracking/gftt_lk/objects/Point2D.h"
#include "feature_tracking/orb_lk/objects/OrbCornerDetector.h"

namespace
{

using localisation::visual_odometry::feature_tracking::gftt_lk::
    FeatureTrackingStatus;
using localisation::visual_odometry::feature_tracking::gftt_lk::ImageView;
using localisation::visual_odometry::feature_tracking::gftt_lk::Point2D;
using localisation::visual_odometry::feature_tracking::orb_lk::
    OrbCornerDetector;

/*!
 * @brief           Wraps a tightly-packed pixel buffer in an `ImageView`.
 */
ImageView makeView(const std::vector<std::uint8_t> &pixels_in, int width_in,
                   int height_in)
{
    ImageView view;
    view.p_pixels = pixels_in.data();
    view.width = width_in;
    view.height = height_in;
    view.strideBytes = static_cast<std::size_t>(width_in);
    return view;
}

/*!
 * @brief           Builds a checkerboard pattern with `squarePx_in`-sized
 *                  squares.
 */
std::vector<std::uint8_t> makeCheckerboardImage(int width_in, int height_in,
                                                int squarePx_in)
{
    std::vector<std::uint8_t> pixels(
        static_cast<std::size_t>(width_in) *
        static_cast<std::size_t>(height_in));

    for (int row = 0; row < height_in; ++row)
    {
        for (int column = 0; column < width_in; ++column)
        {
            const bool isLightSquare =
                ((row / squarePx_in) + (column / squarePx_in)) % 2 == 0;
            pixels[(static_cast<std::size_t>(row) *
                    static_cast<std::size_t>(width_in)) +
                   static_cast<std::size_t>(column)] =
                isLightSquare ? 220U : 20U;
        }
    }

    return pixels;
}

} /* anonymous namespace */

TEST(OrbCornerDetectorTest, RequiresInitializeBeforeDetect)
{
    OrbCornerDetector detector;
    std::array<Point2D, OrbCornerDetector::MAXIMUM_SUPPORTED_FEATURES>
        features{};
    std::size_t featureCount = 0U;
    const std::vector<std::uint8_t> pixels(64U * 64U, 128U);
    const ImageView view = makeView(pixels, 64, 64);
    EXPECT_EQ(detector.detect(view, features, featureCount),
              FeatureTrackingStatus::FEATURE_TRACKING_STATUS_NOT_INITIALIZED);
}

TEST(OrbCornerDetectorTest, RejectsInvalidConfiguration)
{
    OrbCornerDetector detector;
    EXPECT_EQ(detector.initialize(0, 64, 100U, 1.2, 8, 20),
              FeatureTrackingStatus::
                  FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION);
    EXPECT_EQ(detector.initialize(64, 64, 0U, 1.2, 8, 20),
              FeatureTrackingStatus::
                  FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION);
    EXPECT_EQ(detector.initialize(
                  64,
                  64,
                  OrbCornerDetector::MAXIMUM_SUPPORTED_FEATURES + 1U,
                  1.2,
                  8,
                  20),
              FeatureTrackingStatus::
                  FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION);
    EXPECT_EQ(detector.initialize(64, 64, 100U, 1.0, 8, 20),
              FeatureTrackingStatus::
                  FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION);
    EXPECT_EQ(detector.initialize(64, 64, 100U, 1.2, 0, 20),
              FeatureTrackingStatus::
                  FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION);
    EXPECT_EQ(detector.initialize(64, 64, 100U, 1.2, 17, 20),
              FeatureTrackingStatus::
                  FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION);
    EXPECT_EQ(detector.initialize(64, 64, 100U, 1.2, 8, 0),
              FeatureTrackingStatus::
                  FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION);
}

TEST(OrbCornerDetectorTest, RejectsImageDimensionMismatch)
{
    OrbCornerDetector detector;
    EXPECT_EQ(detector.initialize(64, 64, 100U, 1.2, 8, 20),
              FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);
    std::array<Point2D, OrbCornerDetector::MAXIMUM_SUPPORTED_FEATURES>
        features{};
    std::size_t featureCount = 0U;
    const std::vector<std::uint8_t> pixels(32U * 32U, 128U);
    const ImageView view = makeView(pixels, 32, 32);
    EXPECT_EQ(detector.detect(view, features, featureCount),
              FeatureTrackingStatus::FEATURE_TRACKING_STATUS_INVALID_INPUT);
    EXPECT_EQ(detector.terminate(),
              FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);
}

TEST(OrbCornerDetectorTest, DetectsFeaturesOnCheckerboard)
{
    OrbCornerDetector detector;
    EXPECT_EQ(detector.initialize(200, 200, 500U, 1.2, 8, 20),
              FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);
    const std::vector<std::uint8_t> pixels =
        makeCheckerboardImage(200, 200, 20);
    const ImageView view = makeView(pixels, 200, 200);
    std::array<Point2D, OrbCornerDetector::MAXIMUM_SUPPORTED_FEATURES>
        features{};
    std::size_t featureCount = 0U;
    EXPECT_EQ(detector.detect(view, features, featureCount),
              FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);
    EXPECT_GT(featureCount, 0U);
    EXPECT_LE(featureCount, 500U);
    for (std::size_t featureIndex = 0U; featureIndex < featureCount;
         ++featureIndex)
    {
        EXPECT_GE(features[featureIndex].x, 0.0F);
        EXPECT_LE(features[featureIndex].x, 200.0F);
        EXPECT_GE(features[featureIndex].y, 0.0F);
        EXPECT_LE(features[featureIndex].y, 200.0F);
    }
    EXPECT_EQ(detector.terminate(),
              FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);
}

TEST(OrbCornerDetectorTest, CapsAcceptedFeaturesAtConfiguredMaximum)
{
    OrbCornerDetector detector;
    EXPECT_EQ(detector.initialize(200, 200, 10U, 1.2, 8, 20),
              FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);
    const std::vector<std::uint8_t> pixels =
        makeCheckerboardImage(200, 200, 20);
    const ImageView view = makeView(pixels, 200, 200);
    std::array<Point2D, OrbCornerDetector::MAXIMUM_SUPPORTED_FEATURES>
        features{};
    std::size_t featureCount = 0U;
    EXPECT_EQ(detector.detect(view, features, featureCount),
              FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);
    EXPECT_LE(featureCount, 10U);
    EXPECT_EQ(detector.terminate(),
              FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);
}

TEST(OrbCornerDetectorTest, RejectsDetectAfterTerminate)
{
    OrbCornerDetector detector;
    EXPECT_EQ(detector.initialize(64, 64, 100U, 1.2, 8, 20),
              FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);
    EXPECT_EQ(detector.terminate(),
              FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);
    std::array<Point2D, OrbCornerDetector::MAXIMUM_SUPPORTED_FEATURES>
        features{};
    std::size_t featureCount = 0U;
    const std::vector<std::uint8_t> pixels(64U * 64U, 128U);
    const ImageView view = makeView(pixels, 64, 64);
    EXPECT_EQ(detector.detect(view, features, featureCount),
              FeatureTrackingStatus::FEATURE_TRACKING_STATUS_NOT_INITIALIZED);
}
