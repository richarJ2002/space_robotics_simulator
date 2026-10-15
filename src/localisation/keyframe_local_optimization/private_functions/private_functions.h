/*!
 * @file            private_functions.h
 *
 * @brief           Declares the keyframe local optimization module's
 *                  internal helpers.
 *
 * @date            08/10/2026
 */

#ifndef SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_PRIVATE_FUNCTIONS_H
#define SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_PRIVATE_FUNCTIONS_H

/* Data include */
/* None */

/* Generic Libraries */
#include <cstddef>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace localisation::keyframe_local_optimization
{

/*!
 * @brief       Builds the skew-symmetric matrix of a vector.
 *
 *              For the residual r = R^T (L - p) - z with the rotation
 *              perturbed as R Exp(dphi), the rotation Jacobian is
 *              exactly skew(R^T (L - p)) (dimensions 3 x 3, units
 *              of metres per radian).
 *
 * @param[in]   vector_in
 *              Vector to cross with, in metres.
 *
 * @return      Skew-symmetric matrix with vector_in as its axial
 *              vector.
 */
Eigen::Matrix3d skewOfVector(const Eigen::Vector3d &vector_in);

/*!
 * @brief       Maps a rotation vector to a unit quaternion.
 *
 *              Rodrigues form; the zero vector maps to identity and
 *              tiny angles use the series limit, so the datum pose
 *              never injects NaN.
 *
 * @param[in]   rotationVector_in
 *              Axis times angle, in radians.
 *
 * @return      Unit quaternion for the rotation.
 */
Eigen::Quaterniond
    rotationVectorToQuaternion(const Eigen::Vector3d &rotationVector_in);

/*!
 * @brief       Maps a unit quaternion to its rotation vector.
 *
 * @param[in]   orientation_in
 *              Unit quaternion to convert.
 *
 * @return      Axis times angle, in radians.
 */
Eigen::Vector3d
    quaternionToRotationVector(const Eigen::Quaterniond &orientation_in);

/*!
 * @brief       Inverts a symmetric 3x3 with diagonal damping.
 *
 *              Closed-form adjugate over determinant, so no heap
 *              workspace is needed for the per-landmark solves.
 *
 * @param[in]   matrix_in
 *              Symmetric matrix to invert.
 * @param[in]   damping_in
 *              Value added to the diagonal before inversion.
 * @param[out]  inverse_out
 *              Inverse; untouched unless true is returned.
 *
 * @return      True when the damped determinant is usable.
 */
bool invertDampedSymmetric3x3(const Eigen::Matrix3d &matrix_in,
                              double                 damping_in,
                              Eigen::Matrix3d       &inverse_out);

/*!
 * @brief       Factorizes a symmetric positive-definite block.
 *
 *              Cholesky-Banachiewicz in place on the top-left block,
 *              using only indexed writes into the caller's
 *              initialize()-sized buffer.
 *
 * @param[in,out] factor_inout
 *              Hessian block on entry, lower factor on success.
 * @param[in]   size_in
 *              Active dimension (at most MAX_FREE_POSE_PARAMETERS).
 *
 * @return      True when every pivot stayed positive.
 */
bool factorizeCholesky(Eigen::MatrixXd &factor_inout, std::size_t size_in);

/*!
 * @brief       Solves a Cholesky-factored system in place.
 *
 *              Forward then backward substitution over the active
 *              block, writing only into the caller's buffers.
 *
 * @param[in]   factor_in
 *              Lower factor from factorizeCholesky().
 * @param[in]   rightHandSide_in
 *              Right-hand side vector.
 * @param[out]  solution_out
 *              Solution; first size_in entries valid on return.
 * @param[in]   size_in
 *              Active dimension (at most MAX_FREE_POSE_PARAMETERS).
 */
void solveCholesky(const Eigen::MatrixXd &factor_in,
                   const Eigen::VectorXd &rightHandSide_in,
                   Eigen::VectorXd       &solution_out,
                   std::size_t            size_in);

} /* namespace localisation::keyframe_local_optimization */

#endif /* SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_PRIVATE_FUNCTIONS_H */
