/*!
 * \file beidou_dnav_ephemeris.cc
 * \brief  Interface of a BeiDou EPHEMERIS storage and orbital model functions
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
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

#include "beidou_dnav_ephemeris.h"
#include "Beidou_DNAV.h"
#include "gnss_satellite.h"
#include <cmath>


Beidou_Dnav_Ephemeris::Beidou_Dnav_Ephemeris()
{
    auto gnss_sat = Gnss_Satellite();
    const std::string _system("Beidou");
    for (unsigned int i = 1; i < 36; i++)
        {
            satelliteBlock[i] = gnss_sat.what_block(_system, i);
        }
}


double Beidou_Dnav_Ephemeris::check_t(double time)
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


double Beidou_Dnav_Ephemeris::sv_clock_drift(double transmitTime)
{
    double dt = check_t(transmitTime - d_Toc);

    for (int i = 0; i < 2; i++)
        {
            dt -= d_A_f0 + d_A_f1 * dt + d_A_f2 * (dt * dt);
        }
    d_satClkDrift = d_A_f0 + d_A_f1 * dt + d_A_f2 * (dt * dt);

    return d_satClkDrift;
}


// compute the relativistic correction term
double Beidou_Dnav_Ephemeris::sv_clock_relativistic_term(double transmitTime)
{
    // Restore semi-major axis
    const double a = d_sqrt_A * d_sqrt_A;

    // Time from ephemeris reference epoch
    const double tk = check_t(transmitTime - d_Toe);

    // Computed mean motion
    const double n0 = sqrt(BEIDOU_GM / (a * a * a));

    // Corrected mean motion
    const double n = n0 + d_Delta_n;

    // Mean anomaly
    double M = d_M_0 + n * tk;

    // Reduce mean anomaly to between 0 and 2pi
    M = fmod((M + 2.0 * GNSS_PI), (2.0 * GNSS_PI));

    // Initial guess of eccentric anomaly
    double E = M;
    double E_old;
    double dE;

    // --- Iteratively compute eccentric anomaly -------------------------------
    for (int ii = 1; ii < 20; ii++)
        {
            E_old = E;
            E = M + d_eccentricity * sin(E);
            dE = fmod(E - E_old, 2.0 * GNSS_PI);
            if (fabs(dE) < 1e-12)
                {
                    // Necessary precision is reached, exit from the loop
                    break;
                }
        }

    // Compute relativistic correction term
    d_dtr = BEIDOU_F * d_eccentricity * d_sqrt_A * sin(E);
    return d_dtr;
}


double Beidou_Dnav_Ephemeris::satellitePosition(double transmitTime)
{
    // Find satellite's position -----------------------------------------------
    // Restore semi-major axis
    const double a = d_sqrt_A * d_sqrt_A;

    // Time from ephemeris reference epoch
    double tk = check_t(transmitTime - d_Toe);

    // Computed mean motion
    const double n0 = sqrt(BEIDOU_GM / (a * a * a));

    // Corrected mean motion
    const double n = n0 + d_Delta_n;

    // Mean anomaly
    double M = d_M_0 + n * tk;

    // Reduce mean anomaly to between 0 and 2pi
    M = fmod((M + 2.0 * GNSS_PI), (2.0 * GNSS_PI));

    // Initial guess of eccentric anomaly
    double E = M;
    double E_old;
    double dE;

    // --- Iteratively compute eccentric anomaly -------------------------------
    for (int ii = 1; ii < 20; ii++)
        {
            E_old = E;
            E = M + d_eccentricity * sin(E);
            dE = fmod(E - E_old, 2.0 * GNSS_PI);
            if (fabs(dE) < 1e-12)
                {
                    // Necessary precision is reached, exit from the loop
                    break;
                }
        }

    // Compute the true anomaly
    const double sek = sin(E);
    const double cek = cos(E);
    const double OneMinusecosE = 1.0 - d_eccentricity * cek;
    const double ekdot = n / OneMinusecosE;

    // Compute the true anomaly
    const double sq1e2 = sqrt(1.0 - d_eccentricity * d_eccentricity);
    const double tmp_Y = sq1e2 * sek;
    const double tmp_X = cek - d_eccentricity;
    const double nu = atan2(tmp_Y, tmp_X);

    // Compute angle phi (argument of Latitude)
    const double phi = nu + d_OMEGA;
    double pkdot = sq1e2 * ekdot / OneMinusecosE;

    // Reduce phi to between 0 and 2*pi rad
    // phi = fmod((phi), (2.0 * GNSS_PI));
    const double s2pk = sin(2.0 * phi);
    const double c2pk = cos(2.0 * phi);

    // Correct argument of latitude
    const double u = phi + d_Cuc * c2pk + d_Cus * s2pk;
    const double cuk = cos(u);
    const double suk = sin(u);
    const double ukdot = pkdot * (1.0 + 2.0 * (d_Cus * c2pk - d_Cuc * s2pk));

    // Correct radius
    const double r = a * (1.0 - d_eccentricity * cek) + d_Crc * c2pk + d_Crs * s2pk;

    const double rkdot = a * d_eccentricity * sek * ekdot + 2.0 * pkdot * (d_Crs * c2pk - d_Crc * s2pk);

    // Correct inclination
    const double i = d_i_0 + d_IDOT * tk + d_Cic * c2pk + d_Cis * s2pk;
    const double sik = sin(i);
    const double cik = cos(i);
    const double ikdot = d_IDOT + 2.0 * pkdot * (d_Cis * c2pk - d_Cic * s2pk);

    // Compute the angle between the ascending node and the Greenwich meridian
    const double Omega = d_OMEGA0 + (d_OMEGA_DOT - BEIDOU_OMEGA_EARTH_DOT) * tk - BEIDOU_OMEGA_EARTH_DOT * d_Toe;
    const double sok = sin(Omega);
    const double cok = cos(Omega);

    // --- Compute satellite coordinates in Earth-fixed coordinates
    const double xprime = r * cuk;
    const double yprime = r * suk;
    d_satpos_X = xprime * cok - yprime * cik * sok;
    d_satpos_Y = xprime * sok + yprime * cik * cok;
    d_satpos_Z = yprime * sik;

    // Satellite's velocity. Can be useful for Vector Tracking loops
    const double Omega_dot = d_OMEGA_DOT - BEIDOU_OMEGA_EARTH_DOT;

    const double xpkdot = rkdot * cuk - yprime * ukdot;
    const double ypkdot = rkdot * suk + xprime * ukdot;
    const double tmp = ypkdot * cik - d_satpos_Z * ikdot;

    d_satvel_X = -Omega_dot * d_satpos_Y + xpkdot * cok - tmp * sok;
    d_satvel_Y = Omega_dot * d_satpos_X + xpkdot * sok + tmp * cok;
    d_satvel_Z = yprime * cik * ikdot + ypkdot * sik;

    // Time from ephemeris reference clock
    tk = check_t(transmitTime - d_Toc);

    double dtr_s = d_A_f0 + d_A_f1 * tk + d_A_f2 * tk * tk;

    /* relativity correction */
    dtr_s -= 2.0 * sqrt(BEIDOU_GM * a) * d_eccentricity * sek / (SPEED_OF_LIGHT_M_S * SPEED_OF_LIGHT_M_S);

    return dtr_s;
}
