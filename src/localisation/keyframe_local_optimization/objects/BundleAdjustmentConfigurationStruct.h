/*!
 * @File:         BundleAdjustmentConfigurationStruct.h
 *
 * @Brief:        Declares the Full BA solver tuning.
 *
 * @Date:         08/10/2026
 *
 */

#ifndef SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_BA_CONFIG_STRUCT_H
#define SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_BA_CONFIG_STRUCT_H

/* Data include */
/* None */

/* Generic Libraries */
#include <cstddef>

namespace localisation::keyframe_local_optimization
{

/*!
 * @brief       Full BA (poses + landmarks) solver tuning.
 *
 *              Gauss-Newton with a Schur-complement reduced camera
 *              system and an optional Huber robustifier on the 3-D
 *              point residuals.
 */
struct BundleAdjustmentConfiguration
{
    /*!
     * @brief       Gauss-Newton iterations per optimization.
     */
    std::size_t maximumIterations{10U};

    /*!
     * @brief       True applies Huber weighting to each residual.
     */
    bool useHuber{true};

    /*!
     * @brief       Huber linear-transition distance in metres.
     *
     * @units       Metres.
     */
    double huberDelta_m{0.05};
};

} /* namespace localisation::keyframe_local_optimization */

#endif /* SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_BA_CONFIG_STRUCT_H */
