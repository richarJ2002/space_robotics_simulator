/*!
 * @File:         KeyframeSampleStruct.h
 *
 * @Brief:        Declares one source-agnostic keyframe input sample.
 *
 * @Date:         08/10/2026
 *
 */

#ifndef SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_SAMPLE_STRUCT_H
#define SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_SAMPLE_STRUCT_H

/* Object Includes */
#include "objects/KeyframeLocalOptimizationLimits.h"

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
 * @brief       One source-agnostic keyframe input sample.
 *
 *              Carries a keyframe pose with its odometry covariance
 *              plus the observed 3-D points expressed in the
 *              keyframe-local body frame. Producer-agnostic: visual
 *              odometry and the EKF both publish nav_msgs Odometry,
 *              so either fills this struct; the PointCloud2 that
 *              carries the local points itself has no covariance.
 */
struct KeyframeSample
{
    /*!
     * @brief       Keyframe stamp in seconds.
     */
    double stamp_s{0.0};

    /*!
     * @brief       Keyframe position in the odom frame, in metres.
     *
     * @frame       odom_frame (default <system>/startup_fixed).
     * @units       Metres.
     */
    Eigen::Vector3d position_Fix_m{Eigen::Vector3d::Zero()};

    /*!
     * @brief       Keyframe orientation, odom frame to body frame.
     *
     * @frame       Fix Frame to Body Frame.
     * @units       N/A
     */
    Eigen::Quaterniond quaternionOrientation_FixToBod{
        Eigen::Quaterniond::Identity()};

    /*!
     * @brief       Pose covariance, translation then rotation.
     *
     * @frame       N/A
     * @units       Metres squared, radians squared.
     */
    Eigen::Matrix<double, 6, 6> poseCovariance{
        Eigen::Matrix<double, 6, 6>::Identity()};

    /*!
     * @brief       Body-frame linear twist carried for republication.
     *
     * @frame       Body Frame.
     * @units       Metres per second.
     */
    Eigen::Vector3d twistLinear_Bod_mPs{Eigen::Vector3d::Zero()};

    /*!
     * @brief       Body-frame angular twist carried for republication.
     *
     * @frame       Body Frame.
     * @units       Radians per second.
     */
    Eigen::Vector3d twistAngular_Bod_radPs{Eigen::Vector3d::Zero()};

    /*!
     * @brief       Observed 3-D points in the keyframe-local frame.
     *
     *              Only the first localPointCount entries are valid.
     *
     * @frame       Body Frame at time `stamp_s`.
     * @units       Metres.
     */
    std::array<Eigen::Vector3d, MAX_OBSERVATIONS_PER_KEYFRAME> localPoints{};

    /*!
     * @brief       Valid entries in localPoints.
     *
     * @frame       N/A
     * @units       N/A
     */
    std::size_t localPointCount{0U};

    /*!
     * @brief       True when the pose was accepted (not stale).
     *
     * @frame       N/A
     * @units       N/A
     */
    bool isPoseAvailable{true};
};

} /* namespace localisation::keyframe_local_optimization */

#endif /* SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_SAMPLE_STRUCT_H */
