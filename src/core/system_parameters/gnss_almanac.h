/*!
 * \file gnss_almanac.h
 * \brief Base class for GNSS almanac storage
 * \author Carles Fernandez, 2021. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GNSS_ALMANAC_H
#define GNSS_SDR_GNSS_ALMANAC_H

#include <array>
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief Base class for GNSS almanac storage
 */
class Gnss_Almanac
{
public:
    /*!
     * Default constructor
     */
    Gnss_Almanac() = default;

    /*!
     * \brief Computes prediction of the Doppler shift for a given time and receiver's position and velocity.
     * \f[
     * f_{d} = - \mathbf{v} \frac{\mathbf{x}^{T}}{\left| \mathbf{x} \right| } \frac{f_{L}}{c}
     * \f]
     * where:
     * \f[
     * \mathbf{v} = \mathbf{v}_{sat} - \mathbf{v}_{rx}
     * \f]
     * \f[
     * \mathbf{x} = \mathbf{x}_{sat} - \mathbf{x}_{rx}
     * \f]
     * \f[
     * \left| \mathbf{x} \right| = \sqrt{\mathbf{x}\mathbf{x}^{T}}
     * \f]
     *
     * @param[in] rx_time_s Time of Week in seconds
     * @param[in] lat Receiver's latitude in degrees
     * @param[in] lon Receiver's longitude in degrees
     * @param[in] h   Receiver's height in meters
     * @param[in] ve  Receiver's velocity in the East direction [m/s]
     * @param[in] vn  Receiver's velocity in the North direction [m/s]
     * @param[in] vu  Receiver's velocity in the Up direction [m/s]
     * @param[in] band Signal band for which the Doppler will be computed
     *                 (1: L1 C/A, E1B, BI1; 2: L2C, BI2; 3: BI3; 5: L5/E5a; 6: E6B; 7: E5b; 8: E5a+E5b)
     */
    double predicted_doppler(double rx_time_s,
        double lat,
        double lon,
        double h,
        double ve,
        double vn,
        double vu,
        int band) const;

    /*!
     * \brief Computes satellite Position and Velocity, in ECEF, for a given time (expressed in seconds of week)
     */
    void satellitePosVelComputation(double transmitTime, std::array<double, 7>& pos_vel_dtr) const;

    uint32_t PRN{};     //!< SV PRN NUMBER
    double delta_i{};   //!< Inclination Angle at Reference Time (relative to i_0 = 0.30 semi-circles)
    int32_t toa{};      //!< Almanac data reference time of week [s]
    int32_t WNa{};      //!< Almanac week number
    double M_0{};       //!< Mean Anomaly at Reference Time [semi-circles]
    double ecc{};       //!< Eccentricity [dimensionless]
    double sqrtA{};     //!< Square Root of the Semi-Major Axis [sqrt(m)]
    double OMEGA_0{};   //!< Longitude of Ascending Node of Orbit Plane at Weekly Epoch [semi-circles]
    double omega{};     //!< Argument of Perigee [semi-cicles]
    double OMEGAdot{};  //!< Rate of Right Ascension [semi-circles/s]
    double af0{};       //!< Coefficient 0 of code phase offset model [s]
    double af1{};       //!< Coefficient 1 of code phase offset model [s/s]

protected:
    char System{};  //!< Character ID of the GNSS system. 'G': GPS. 'E': Galileo. 'B': BeiDou
private:
    double check_t(double time) const;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GNSS_ALMANAC_H
