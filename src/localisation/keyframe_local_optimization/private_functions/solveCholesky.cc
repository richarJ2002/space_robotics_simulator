/*!
 * @file            solveCholesky.cc
 *
 * @brief           Implements Cholesky-factored system solve.
 *
 * @date            08/10/2026
 */

/* Matching Declaration Include */
#include "private_functions/private_functions.h"

/* Generic Libraries */
#include <cstddef>

#include <Eigen/Dense>

namespace localisation::keyframe_local_optimization
{

void solveCholesky(const Eigen::MatrixXd &factor_in,
                   const Eigen::VectorXd &rightHandSide_in,
                   Eigen::VectorXd       &solution_out,
                   std::size_t            size_in)
{
    for (std::size_t rowIndex = 0U; rowIndex < size_in; rowIndex++)
    {
        double forwardSum =
            rightHandSide_in(static_cast<Eigen::Index>(rowIndex));
        for (std::size_t innerIndex = 0U; innerIndex < rowIndex; innerIndex++)
        {
            forwardSum -= factor_in(static_cast<Eigen::Index>(rowIndex),
                                    static_cast<Eigen::Index>(innerIndex)) *
                          solution_out(static_cast<Eigen::Index>(innerIndex));
        }
        solution_out(static_cast<Eigen::Index>(rowIndex)) =
            forwardSum / factor_in(static_cast<Eigen::Index>(rowIndex),
                                   static_cast<Eigen::Index>(rowIndex));
    }
    for (std::size_t reverseIndex = size_in; reverseIndex > 0U; reverseIndex--)
    {
        const std::size_t rowIndex = reverseIndex - 1U;
        double backwardSum = solution_out(static_cast<Eigen::Index>(rowIndex));
        for (std::size_t innerIndex = rowIndex + 1U; innerIndex < size_in;
             innerIndex++)
        {
            backwardSum -= factor_in(static_cast<Eigen::Index>(innerIndex),
                                     static_cast<Eigen::Index>(rowIndex)) *
                           solution_out(static_cast<Eigen::Index>(innerIndex));
        }
        solution_out(static_cast<Eigen::Index>(rowIndex)) =
            backwardSum / factor_in(static_cast<Eigen::Index>(rowIndex),
                                    static_cast<Eigen::Index>(rowIndex));
    }
}

} /* namespace localisation::keyframe_local_optimization */
