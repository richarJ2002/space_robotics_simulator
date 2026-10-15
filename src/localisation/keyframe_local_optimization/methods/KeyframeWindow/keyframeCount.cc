/*!
 * @File:         keyframeCount.cc
 *
 * @Brief:        Counts the stored keyframes.
 *
 * @Date:         08/10/2026
 *
 */

/* Matching Declaration Include */
#include "objects/KeyframeWindowClass.h"

/* External Library Includes */
/* None */

/* Other Project Module Includes */
/* None */

/* Generic Libraries */
/* None */

namespace localisation::keyframe_local_optimization
{

std::size_t KeyframeWindow::keyframeCount() const
{
    /* An uninitialized window holds nothing by construction. */
    if (!isInitialized)
    {
        return 0U;
    }
    return storedKeyframeCount;
}

} /* namespace localisation::keyframe_local_optimization */
