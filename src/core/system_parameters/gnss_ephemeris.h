/*!
 * \file gnss_ephemeris.h
 * \brief Base class for GNSS Ephemeris
 * \author Carles Fernandez, 2021. cfernandez(at)cttc.es
 *
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


#ifndef GNSS_SDR_GNSS_EPHEMERIS_H
#define GNSS_SDR_GNSS_EPHEMERIS_H

#include <array>
#include <cstdint>

/*!
 * \brief Base class for GNSS ephemeris storage
 */
class Gnss_Ephemeris
{
public:
    Gnss_Ephemeris() = default;

    /*!
     * \brief Sets (\a satClkDrift) and (\a dtr), and returns the clock drift in
     * seconds according to the User Algorithm for SV Clock Correction
     * (IS-GPS-200M, 20.3.3.3.3.1, and Galileo OS SIS ICD, 5.1.4).
     */
    double sv_clock_drift(double transmitTime);

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
    double predicted_doppler(double rx_time_s, double lat, double lon, double h, double ve, double vn, double vu, int band) const;

    void satellitePosition(double transmitTime);  //!< Computes the ECEF SV coordinates and ECEF velocity

    uint32_t PRN{};     //!< SV ID
    double M_0{};       //!< Mean anomaly at reference time [semi-circles]
    double delta_n{};   //!< Mean motion difference from computed value [semi-circles/sec]
    double ecc{};       //!< Eccentricity
    double sqrtA{};     //!< Square root of the semi-major axis [meters^1/2]
    double OMEGA_0{};   //!< Longitude of ascending node of orbital plane at weekly epoch [semi-circles]
    double i_0{};       //!< Inclination angle at reference time [semi-circles]
    double omega{};     //!< Argument of perigee [semi-circles]
    double OMEGAdot{};  //!< Rate of right ascension [semi-circles/sec]
    double idot{};      //!< Rate of inclination angle [semi-circles/sec]
    double Cuc{};       //!< Amplitude of the cosine harmonic correction term to the argument of latitude [radians]
    double Cus{};       //!< Amplitude of the sine harmonic correction term to the argument of latitude [radians]
    double Crc{};       //!< Amplitude of the cosine harmonic correction term to the orbit radius [meters]
    double Crs{};       //!< Amplitude of the sine harmonic correction term to the orbit radius [meters]
    double Cic{};       //!< Amplitude of the cosine harmonic correction term to the angle of inclination [radians]
    double Cis{};       //!< Amplitude of the sine harmonic correction term to the angle of inclination [radians]
    int32_t toe{};      //!< Ephemeris reference time [s]

    // Clock correction parameters
    int32_t toc{};  //!< Clock correction data reference Time of Week [sec]
    double af0{};   //!< SV clock bias correction coefficient [s]
    double af1{};   //!< SV clock drift correction coefficient [s/s]
    double af2{};   //!< SV clock drift rate correction coefficient [s/s^2]

    double satClkDrift{};  //!< SV clock drift
    double dtr{};          //!< Relativistic clock correction term

    // Time
    int32_t WN{};   //!< Week number
    int32_t tow{};  //!< Time of Week

    // satellite positions
    double satpos_X{};  //!< Earth-fixed coordinate x of the satellite [m]. Intersection of the IERS Reference Meridian (IRM) and the plane passing through the origin and normal to the Z-axis.
    double satpos_Y{};  //!< Earth-fixed coordinate y of the satellite [m]. Completes a right-handed, Earth-Centered, Earth-Fixed orthogonal coordinate system.
    double satpos_Z{};  //!< Earth-fixed coordinate z of the satellite [m]. The direction of the IERS (International Earth Rotation and Reference Systems Service) Reference Pole (IRP).

    // Satellite velocity
    double satvel_X{};  //!< Earth-fixed velocity coordinate x of the satellite [m]
    double satvel_Y{};  //!< Earth-fixed velocity coordinate y of the satellite [m]
    double satvel_Z{};  //!< Earth-fixed velocity coordinate z of the satellite [m]

protected:
    char System{};  //!< Character ID of the GNSS system. 'G': GPS.  'E': Galileo.  'B': BeiDou

private:
    void satellitePosVelComputation(double transmitTime, std::array<double, 7>& pos_vel_dtr) const;
    double check_t(double time) const;
    double sv_clock_relativistic_term(double transmitTime) const;
};

#endif  // GNSS_SDR_GNSS_EPHEMERIS_H
