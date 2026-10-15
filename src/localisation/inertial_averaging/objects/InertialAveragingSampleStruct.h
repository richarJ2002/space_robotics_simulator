/*!
 * @File:         InertialAveragingSampleStruct.h
 *
 * @Brief:        Declares one bias-corrected body-frame IMU sample.
 *
 * @Date:         07/10/2026
 *
 */

#ifndef SRS_LOCALISATION_INERTIAL_AVERAGING_SAMPLE_STRUCT_H
#define SRS_LOCALISATION_INERTIAL_AVERAGING_SAMPLE_STRUCT_H

/* Data include */
/* None */

/* Generic Libraries */
#include <array>

namespace localisation::inertial_averaging
{

/*!
 * @brief       One bias-corrected body-frame IMU sample.
 *
 *              Plain value type so the averager stays ROS-free; the owning
 *              node translates to and from `sensor_msgs::msg::Imu` at its
 *              boundary. push() averages these; it never sees raw biases.
 */
struct InertialAveragingSample
{
  public:
    /*!
     * @brief       Sample time in ROS seconds.
     *
     * @frame       N/A
     * @units       s
     */
    double stamp_s{0.0};

    /*!
     * @brief       Bias-corrected body-frame linear acceleration.
     *
     * @frame       Body
     * @units       m/s^2
     */
    std::array<double, 3> acceleration_mPs2{0.0, 0.0, 0.0};

    /*!
     * @brief       Bias-corrected body-frame angular rate.
     *
     * @frame       Body
     * @units       rad/s
     */
    std::array<double, 3> angularRate_radPs{0.0, 0.0, 0.0};
};

} /* namespace localisation::inertial_averaging */

#endif /* SRS_LOCALISATION_INERTIAL_AVERAGING_SAMPLE_STRUCT_H */
