/*!
 * @File:         InertialAveragingConfigurationStruct.h
 *
 * @Brief:        Declares the inertial averager window tuning.
 *
 * @Date:         07/10/2026
 *
 */

#ifndef SRS_LOCALISATION_INERTIAL_AVERAGING_CONFIGURATION_STRUCT_H
#define SRS_LOCALISATION_INERTIAL_AVERAGING_CONFIGURATION_STRUCT_H

/* Data include */
/* None */

/* Generic Libraries */
/* None */

namespace localisation::inertial_averaging
{

/*!
 * @brief       Window tuning for the inertial averager.
 *
 *              Whichever bound fills first drops the oldest sample: more
 *              than windowSampleTarget stored samples, or samples older
 *              than windowDuration_ms behind the newest stamp.
 */
struct InertialAveragingConfiguration
{
  public:
    /*!
     * @brief       Maximum stored samples; valid range is 1 to
     *              MAX_AVERAGER_SAMPLES inclusive.
     *
     * @frame       N/A
     * @units       samples
     */
    int windowSampleTarget{10};

    /*!
     * @brief       Maximum sample age behind the newest stamp; must be
     *              finite and positive.
     *
     * @frame       N/A
     * @units       ms
     */
    double windowDuration_ms{200.0};
};

} /* namespace localisation::inertial_averaging */

#endif /* SRS_LOCALISATION_INERTIAL_AVERAGING_CONFIGURATION_STRUCT_H */
