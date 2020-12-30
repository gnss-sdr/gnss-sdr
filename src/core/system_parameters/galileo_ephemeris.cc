/*!
 * \file galileo_ephemeris.cc
 * \brief  Interface of a Galileo EPHEMERIS storage and orbital model functions
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 * \author Mara Branzanti 2013. mara.branzanti(at)gmail.com
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "galileo_ephemeris.h"
#include "MATH_CONSTANTS.h"
#include <cmath>


double Galileo_Ephemeris::Galileo_System_Time(double WN, double TOW)
{
    /* GALILEO SYSTEM TIME, ICD 5.1.2
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
    const double sec_in_day = 86400;
    const double day_in_week = 7;
    double t = WN * sec_in_day * day_in_week + TOW;  // second from the origin of the Galileo time
    return t;
}


double Galileo_Ephemeris::sv_clock_drift(double transmitTime)
{
    // Satellite Time Correction Algorithm, ICD 5.1.4
    const double dt = transmitTime - t0c_4;
    Galileo_satClkDrift = af0_4 + af1_4 * dt + af2_4 * (dt * dt) + sv_clock_relativistic_term(transmitTime);  // +Galileo_dtr;
    return Galileo_satClkDrift;
}


// compute the relativistic correction term
double Galileo_Ephemeris::sv_clock_relativistic_term(double transmitTime)  // Satellite Time Correction Algorithm, ICD 5.1.4
{
    // Restore semi-major axis
    const double a = A_1 * A_1;

    const double n0 = sqrt(GALILEO_GM / (a * a * a));

    // Time from ephemeris reference epoch
    // t = WN_5*86400*7 + TOW_5; //WN_5*86400*7 are the second from the origin of the Galileo time
    const double tk = check_t(transmitTime - static_cast<double>(t0e_1));

    // Corrected mean motion
    const double n = n0 + delta_n_3;

    // Mean anomaly
    double M = M0_1 + n * tk;

    // Reduce mean anomaly to between 0 and 2pi
    M = fmod((M + 2.0 * GNSS_PI), (2.0 * GNSS_PI));

    // Initial guess of eccentric anomaly
    double E = M;
    double E_old;
    double dE;

    // --- Iteratively compute eccentric anomaly ----------------------------
    for (int32_t ii = 1; ii < 20; ii++)
        {
            E_old = E;
            E = M + e_1 * sin(E);
            dE = fmod(E - E_old, 2.0 * GNSS_PI);
            if (fabs(dE) < 1e-12)
                {
                    // Necessary precision is reached, exit from the loop
                    break;
                }
        }

    // Compute relativistic correction term
    Galileo_dtr = GALILEO_F * e_1 * A_1 * sin(E);
    return Galileo_dtr;
}


void Galileo_Ephemeris::satellitePosition(double transmitTime)
{
    // when this function in used, the input must be the transmitted time (t) in
    // seconds computed by Galileo_System_Time (above function)

    // Find Galileo satellite's position ---------------------------------------

    // Restore semi-major axis
    const double a = A_1 * A_1;

    // Computed mean motion
    const double n0 = sqrt(GALILEO_GM / (a * a * a));

    // Time from ephemeris reference epoch
    const double tk = check_t(transmitTime - static_cast<double>(t0e_1));

    // Corrected mean motion
    const double n = n0 + delta_n_3;

    // Mean anomaly
    double M = M0_1 + n * tk;

    // Reduce mean anomaly to between 0 and 2pi
    M = fmod((M + 2.0 * GNSS_PI), (2.0 * GNSS_PI));

    // Initial guess of eccentric anomaly
    double E = M;
    double E_old;
    double dE;

    // --- Iteratively compute eccentric anomaly -------------------------------
    for (int32_t ii = 1; ii < 20; ii++)
        {
            E_old = E;
            E = M + e_1 * sin(E);
            dE = fmod(E - E_old, 2.0 * GNSS_PI);
            if (fabs(dE) < 1e-12)
                {
                    // Necessary precision is reached, exit from the loop
                    break;
                }
        }

    const double sek = sin(E);
    const double cek = cos(E);
    const double OneMinusecosE = 1.0 - e_1 * cek;
    const double sq1e2 = sqrt(1.0 - e_1 * e_1);
    const double ekdot = n / OneMinusecosE;

    // Compute the true anomaly
    const double tmp_Y = sq1e2 * sek;
    const double tmp_X = cek - e_1;
    const double nu = atan2(tmp_Y, tmp_X);

    // Compute angle phi (argument of Latitude)
    double phi = nu + omega_2;

    // Reduce phi to between 0 and 2*pi rad
    phi = fmod((phi), (2.0 * GNSS_PI));
    const double s2pk = sin(2.0 * phi);
    const double c2pk = cos(2.0 * phi);
    const double pkdot = sq1e2 * ekdot / OneMinusecosE;

    // Correct argument of latitude
    const double u = phi + C_uc_3 * c2pk + C_us_3 * s2pk;
    const double suk = sin(u);
    const double cuk = cos(u);
    const double ukdot = pkdot * (1.0 + 2.0 * (C_us_3 * c2pk - C_uc_3 * s2pk));

    // Correct radius
    const double r = a * OneMinusecosE + C_rc_3 * c2pk + C_rs_3 * s2pk;
    const double rkdot = a * e_1 * sek * ekdot + 2.0 * pkdot * (C_rs_3 * c2pk - C_rc_3 * s2pk);

    // Correct inclination
    const double i = i_0_2 + iDot_2 * tk + C_ic_4 * c2pk + C_is_4 * s2pk;
    const double sik = sin(i);
    const double cik = cos(i);
    const double ikdot = iDot_2 + 2.0 * pkdot * (C_is_4 * c2pk - C_ic_4 * s2pk);

    // Compute the angle between the ascending node and the Greenwich meridian
    const double Omega_dot = OMEGA_dot_3 - GNSS_OMEGA_EARTH_DOT;
    double Omega = OMEGA_0_2 + Omega_dot * tk - GNSS_OMEGA_EARTH_DOT * static_cast<double>(t0e_1);

    // Reduce to between 0 and 2*pi rad
    Omega = fmod((Omega + 2.0 * GNSS_PI), (2.0 * GNSS_PI));
    const double sok = sin(Omega);
    const double cok = cos(Omega);

    // --- Compute satellite coordinates in Earth-fixed coordinates
    const double xprime = r * cuk;
    const double yprime = r * suk;
    d_satpos_X = xprime * cok - yprime * cik * sok;
    d_satpos_Y = xprime * sok + yprime * cik * cok;  // ********NOTE: in GALILEO ICD this expression is not correct because it has minus (- sin(u) * r * cos(i) * cos(Omega)) instead of plus
    d_satpos_Z = yprime * sik;

    // Satellite's velocity. Can be useful for Vector Tracking loops
    const double xpkdot = rkdot * cuk - yprime * ukdot;
    const double ypkdot = rkdot * suk + xprime * ukdot;
    const double tmp = ypkdot * cik - d_satpos_Z * ikdot;

    d_satvel_X = -Omega_dot * d_satpos_Y + xpkdot * cok - tmp * sok;
    d_satvel_Y = Omega_dot * d_satpos_X + xpkdot * sok + tmp * cok;
    d_satvel_Z = yprime * cik * ikdot + ypkdot * sik;
}


double Galileo_Ephemeris::check_t(double time)
{
    const double half_week = 302400.0;  // seconds
    double corrTime = time;
    if (time > half_week)
        {
            corrTime = time - 2.0 * half_week;
        }
    else if (time < -half_week)
        {
            corrTime = time + 2.0 * half_week;
        }
    return corrTime;
}
