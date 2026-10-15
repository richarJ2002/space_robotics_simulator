/*!
 * @file            factorizeCholesky.cc
 *
 * @brief           Implements in-place Cholesky factorization.
 *
 * @date            08/10/2026
 */

/* Matching Declaration Include */
#include "private_functions/private_functions.h"

/* Generic Libraries */
#include <cmath>
#include <cstddef>

#include <Eigen/Dense>

namespace localisation::keyframe_local_optimization
{

bool factorizeCholesky(Eigen::MatrixXd &factor_inout, std::size_t size_in)
{
    for (std::size_t pivotIndex = 0U; pivotIndex < size_in; pivotIndex++)
    {
        double pivotSum = factor_inout(static_cast<Eigen::Index>(pivotIndex),
                                       static_cast<Eigen::Index>(pivotIndex));
        for (std::size_t innerIndex = 0U; innerIndex < pivotIndex; innerIndex++)
        {
            const double factorValue =
                factor_inout(static_cast<Eigen::Index>(pivotIndex),
                             static_cast<Eigen::Index>(innerIndex));
            pivotSum -= factorValue * factorValue;
        }
        if (!(pivotSum > 1.0e-18))
        {
            return false;
        }
        const double pivotRoot = std::sqrt(pivotSum);
        factor_inout(static_cast<Eigen::Index>(pivotIndex),
                     static_cast<Eigen::Index>(pivotIndex)) = pivotRoot;
        for (std::size_t rowIndex = pivotIndex + 1U; rowIndex < size_in;
             rowIndex++)
        {
            double rowSum = factor_inout(static_cast<Eigen::Index>(rowIndex),
                                         static_cast<Eigen::Index>(pivotIndex));
            for (std::size_t innerIndex = 0U; innerIndex < pivotIndex;
                 innerIndex++)
            {
                rowSum -= factor_inout(static_cast<Eigen::Index>(rowIndex),
                                       static_cast<Eigen::Index>(innerIndex)) *
                          factor_inout(static_cast<Eigen::Index>(pivotIndex),
                                       static_cast<Eigen::Index>(innerIndex));
            }
            factor_inout(static_cast<Eigen::Index>(rowIndex),
                         static_cast<Eigen::Index>(pivotIndex)) =
                rowSum / pivotRoot;
        }
    }
    return true;
}

} /* namespace localisation::keyframe_local_optimization */
