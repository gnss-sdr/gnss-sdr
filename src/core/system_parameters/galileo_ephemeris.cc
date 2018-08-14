/*!
 * \file galileo_ephemeris.cc
 * \brief  Interface of a Galileo EPHEMERIS storage and orbital model functions
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 * \author Mara Branzanti 2013. mara.branzanti(at)gmail.com
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "galileo_ephemeris.h"
#include "Galileo_E1.h"
#include <cmath>


Galileo_Ephemeris::Galileo_Ephemeris()
{
    flag_all_ephemeris = false;
    IOD_ephemeris = 0;
    IOD_nav_1 = 0;
    SV_ID_PRN_4 = 0;
    M0_1 = 0.0;         // Mean anomaly at reference time [semi-circles]
    delta_n_3 = 0.0;    // Mean motion difference from computed value  [semi-circles/sec]
    e_1 = 0.0;          // Eccentricity
    A_1 = 0.0;          // Square root of the semi-major axis [meters^1/2]
    OMEGA_0_2 = 0.0;    // Longitude of ascending node of orbital plane at weekly epoch [semi-circles]
    i_0_2 = 0.0;        // Inclination angle at reference time  [semi-circles]
    omega_2 = 0.0;      // Argument of perigee [semi-circles]
    OMEGA_dot_3 = 0.0;  // Rate of right ascension [semi-circles/sec]
    iDot_2 = 0.0;       // Rate of inclination angle [semi-circles/sec]
    C_uc_3 = 0.0;       // Amplitude of the cosine harmonic correction term to the argument of latitude [radians]
    C_us_3 = 0.0;       // Amplitude of the sine harmonic correction term to the argument of latitude [radians]
    C_rc_3 = 0.0;       // Amplitude of the cosine harmonic correction term to the orbit radius [meters]
    C_rs_3 = 0.0;       // Amplitude of the sine harmonic correction term to the orbit radius [meters]
    C_ic_4 = 0.0;       // Amplitude of the cosine harmonic correction     term to the angle of inclination [radians]
    C_is_4 = 0.0;       // Amplitude of the sine harmonic correction term to the angle of inclination [radians]
    t0e_1 = 0.0;        // Ephemeris reference time [s]

    // Clock correction parameters
    t0c_4 = 0.0;  // Clock correction data reference Time of Week [sec]
    af0_4 = 0.0;  // SV clock bias correction coefficient [s]
    af1_4 = 0.0;  // SV clock drift correction coefficient [s/s]
    af2_4 = 0.0;  // SV clock drift rate correction coefficient [s/s^2]

    // GST
    WN_5 = 0.0;
    TOW_5 = 0.0;

    // SV status
    SISA_3 = 0.0;
    E5a_HS = 0U;
    E5b_HS_5 = 0.0;
    E1B_HS_5 = 0.0;
    E5a_DVS = false;
    E5b_DVS_5 = 0.0;
    E1B_DVS_5 = 0.0;
    BGD_E1E5a_5 = 0.0;  // E1-E5a Broadcast Group Delay [s]
    BGD_E1E5b_5 = 0.0;  // E1-E5b Broadcast Group Delay [s]

    Galileo_satClkDrift = 0.0;
    Galileo_dtr = 0.0;

    // satellite positions
    d_satpos_X = 0.0;
    d_satpos_Y = 0.0;
    d_satpos_Z = 0.0;

    // Satellite velocity
    d_satvel_X = 0.0;
    d_satvel_Y = 0.0;
    d_satvel_Z = 0.0;

    i_satellite_PRN = 0U;
}


double Galileo_Ephemeris::Galileo_System_Time(double WN, double TOW)
{
    /* GALIELO SYSTEM TIME, ICD 5.1.2
     * input parameter:
     * WN: The Week Number is an integer counter that gives the sequential week number
       from the origin of the Galileo time. It covers 4096 weeks (about 78 years).
       Then the counter is reset to zero to cover additional period modulo 4096

       TOW: The Time of Week is defined as the number of seconds that have occurred since
       the transition from the previous week. The TOW covers an entire week from 0 to
       604799 seconds and is reset to zero at the end of each week

       WN and TOW are received in page 5

       output:
       t: it is the transmitted time in Galileo System Time (expressed in seconds)

       The GST start epoch shall be 00:00 UT on Sunday 22nd August 1999 (midnight between 21st and 22nd August).
       At the start epoch, GST shall be ahead of UTC by thirteen (13)
       leap seconds. Since the next leap second was inserted at 01.01.2006, this implies that
       as of 01.01.2006 GST is ahead of UTC by fourteen (14) leap seconds.

       The epoch denoted in the navigation messages by TOW and WN
       will be measured relative to the leading edge of the first chip of the
       first code sequence of the first page symbol. The transmission timing of the navigation
       message provided through the TOW is synchronised to each satellite’s version of Galileo System Time (GST).
     *
     */
    double t = 0;
    double sec_in_day = 86400;
    double day_in_week = 7;
    t = WN * sec_in_day * day_in_week + TOW;  // second from the origin of the Galileo time
    return t;
}


double Galileo_Ephemeris::sv_clock_drift(double transmitTime)
{
    // Satellite Time Correction Algorithm, ICD 5.1.4
    double dt;
    dt = transmitTime - t0c_4;
    Galileo_satClkDrift = af0_4 + af1_4 * dt + af2_4 * (dt * dt) + sv_clock_relativistic_term(transmitTime);  //+Galileo_dtr;
    return Galileo_satClkDrift;
}


// compute the relativistic correction term
double Galileo_Ephemeris::sv_clock_relativistic_term(double transmitTime)  // Satellite Time Correction Algorithm, ICD 5.1.4
{
    double tk;
    double a;
    double n;
    double n0;
    double E;
    double E_old;
    double dE;
    double M;

    // Restore semi-major axis
    a = A_1 * A_1;

    n0 = sqrt(GALILEO_GM / (a * a * a));

    // Time from ephemeris reference epoch
    //t = WN_5*86400*7 + TOW_5; //WN_5*86400*7 are the second from the origin of the Galileo time
    tk = transmitTime - t0e_1;

    // Corrected mean motion
    n = n0 + delta_n_3;

    // Mean anomaly
    M = M0_1 + n * tk;

    // Reduce mean anomaly to between 0 and 2pi
    M = fmod((M + 2 * GALILEO_PI), (2 * GALILEO_PI));

    // Initial guess of eccentric anomaly
    E = M;

    // --- Iteratively compute eccentric anomaly ----------------------------
    for (int32_t ii = 1; ii < 20; ii++)
        {
            E_old = E;
            E = M + e_1 * sin(E);
            dE = fmod(E - E_old, 2 * GALILEO_PI);
            if (fabs(dE) < 1e-12)
                {
                    //Necessary precision is reached, exit from the loop
                    break;
                }
        }

    // Compute relativistic correction term
    Galileo_dtr = GALILEO_F * e_1 * A_1 * sin(E);
    return Galileo_dtr;
}


void Galileo_Ephemeris::satellitePosition(double transmitTime)
{
    // when this function in used, the input must be the transmitted time (t) in second computed by Galileo_System_Time (above function)
    double tk;  // Time from ephemeris reference epoch
    double a;   // Semi-major axis
    double n;   // Corrected mean motion
    double n0;  // Computed mean motion
    double M;   // Mean anomaly
    double E;   // Eccentric Anomaly (to be solved by iteration)
    double E_old;
    double dE;
    double nu;   // True anomaly
    double phi;  // Argument of Latitude
    double u;    // Correct argument of latitude
    double r;    // Correct radius
    double i;
    double Omega;

    // Find Galileo satellite's position ----------------------------------------------

    // Restore semi-major axis
    a = A_1 * A_1;

    // Computed mean motion
    n0 = sqrt(GALILEO_GM / (a * a * a));

    // Time from ephemeris reference epoch
    tk = transmitTime - t0e_1;

    // Corrected mean motion
    n = n0 + delta_n_3;

    // Mean anomaly
    M = M0_1 + n * tk;

    // Reduce mean anomaly to between 0 and 2pi
    M = fmod((M + 2 * GALILEO_PI), (2 * GALILEO_PI));

    // Initial guess of eccentric anomaly
    E = M;

    // --- Iteratively compute eccentric anomaly ----------------------------
    for (int32_t ii = 1; ii < 20; ii++)
        {
            E_old = E;
            E = M + e_1 * sin(E);
            dE = fmod(E - E_old, 2 * GALILEO_PI);
            if (fabs(dE) < 1e-12)
                {
                    //Necessary precision is reached, exit from the loop
                    break;
                }
        }

    // Compute the true anomaly

    double tmp_Y = sqrt(1.0 - e_1 * e_1) * sin(E);
    double tmp_X = cos(E) - e_1;
    nu = atan2(tmp_Y, tmp_X);

    // Compute angle phi (argument of Latitude)
    phi = nu + omega_2;

    // Reduce phi to between 0 and 2*pi rad
    phi = fmod((phi), (2 * GALILEO_PI));

    // Correct argument of latitude
    u = phi + C_uc_3 * cos(2 * phi) + C_us_3 * sin(2 * phi);

    // Correct radius
    r = a * (1 - e_1 * cos(E)) + C_rc_3 * cos(2 * phi) + C_rs_3 * sin(2 * phi);

    // Correct inclination
    i = i_0_2 + iDot_2 * tk + C_ic_4 * cos(2 * phi) + C_is_4 * sin(2 * phi);

    // Compute the angle between the ascending node and the Greenwich meridian
    Omega = OMEGA_0_2 + (OMEGA_dot_3 - GALILEO_OMEGA_EARTH_DOT) * tk - GALILEO_OMEGA_EARTH_DOT * t0e_1;

    // Reduce to between 0 and 2*pi rad
    Omega = fmod((Omega + 2 * GALILEO_PI), (2 * GALILEO_PI));

    // --- Compute satellite coordinates in Earth-fixed coordinates
    d_satpos_X = cos(u) * r * cos(Omega) - sin(u) * r * cos(i) * sin(Omega);
    d_satpos_Y = cos(u) * r * sin(Omega) + sin(u) * r * cos(i) * cos(Omega);  // ********NOTE: in GALILEO ICD this expression is not correct because it has minus (- sin(u) * r * cos(i) * cos(Omega)) instead of plus
    d_satpos_Z = sin(u) * r * sin(i);

    // Satellite's velocity. Can be useful for Vector Tracking loops
    double Omega_dot = OMEGA_dot_3 - GALILEO_OMEGA_EARTH_DOT;
    d_satvel_X = -Omega_dot * (cos(u) * r + sin(u) * r * cos(i)) + d_satpos_X * cos(Omega) - d_satpos_Y * cos(i) * sin(Omega);
    d_satvel_Y = Omega_dot * (cos(u) * r * cos(Omega) - sin(u) * r * cos(i) * sin(Omega)) + d_satpos_X * sin(Omega) + d_satpos_Y * cos(i) * cos(Omega);
    d_satvel_Z = d_satpos_Y * sin(i);
}
