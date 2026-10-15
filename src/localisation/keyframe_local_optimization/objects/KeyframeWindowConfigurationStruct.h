/*!
 * @File:         KeyframeWindowConfigurationStruct.h
 *
 * @Brief:        Declares the sliding-window tuning of the keyframe BA.
 *
 * @Date:         08/10/2026
 *
 */

#ifndef SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_WINDOW_CONFIG_STRUCT_H
#define SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_WINDOW_CONFIG_STRUCT_H

/* Data include */
/* None */

/* Generic Libraries */
#include <cstddef>

namespace localisation::keyframe_local_optimization
{

/*!
 * @brief       Sliding-window tuning of the keyframe BA.
 *
 *              windowSize counts keyframes (trigger_mode count); it is
 *              clamped to 1..MAX_WINDOW_KEYFRAMES by initialize().
 */
struct KeyframeWindowConfiguration
{
    /*!
     * @brief       Keyframes retained in the sliding window.
     */
    std::size_t windowSize{5U};

    /*!
     * @brief       Nearest-neighbour association gate in metres.
     *
     *              A new cloud point joins the closest landmark within
     *              this world-frame distance; otherwise it seeds a new
     *              landmark while the table has room.
     *
     * @units       Metres.
     */
    double associationGate_m{0.10};
};

} /* namespace localisation::keyframe_local_optimization */

#endif /* SRS_LOCALISATION_KEYFRAME_LOCAL_OPTIMIZATION_WINDOW_CONFIG_STRUCT_H  \
        */
