/*!
 * \file beidou_dnav_ephemeris.cc
 * \brief  Interface of a BeiDou EPHEMERIS storage and orbital model functions
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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

#include "beidou_dnav_ephemeris.h"
#include "Beidou_DNAV.h"
#include "gnss_satellite.h"
#include <cmath>

Beidou_Dnav_Ephemeris::Beidou_Dnav_Ephemeris()
{
    i_satellite_PRN = 0;
    d_TOW = 0;
    d_Crs = 0;
    d_Delta_n = 0;
    d_M_0 = 0;
    d_Cuc = 0;
    d_eccentricity = 0;
    d_Cus = 0;
    d_sqrt_A = 0;
    d_Toe = 0;
    d_Toc = 0;
    d_Cic = 0;
    d_OMEGA0 = 0;
    d_Cis = 0;
    d_i_0 = 0;
    d_Crc = 0;
    d_OMEGA = 0;
    d_OMEGA_DOT = 0;
    d_IDOT = 0;
    i_BEIDOU_week = 0;
    i_SV_accuracy = 0;
    i_SV_health = 0;
    d_AODE = 0;
    d_TGD1 = 0;
    d_TGD2 = 0;
    d_AODC = 0;  // Issue of Data, Clock
    i_AODO = 0;  // Age of Data Offset (AODO) term for the navigation message correction table (NMCT) contained in subframe 4 (reference paragraph 20.3.3.5.1.9) [s]
    d_AODC = 0;
    b_fit_interval_flag = false;  // indicates the curve-fit interval used by the CS (Block II/IIA/IIR/IIR-M/IIF) and SS (Block IIIA) in determining the ephemeris parameters, as follows: 0  =  4 hours, 1  =  greater than 4 hours.
    d_spare1 = 0;
    d_spare2 = 0;

    i_sig_type = 0;
    i_nav_type = 0;

    d_A_f0 = 0;  // Coefficient 0 of code phase offset model [s]
    d_A_f1 = 0;  // Coefficient 1 of code phase offset model [s/s]
    d_A_f2 = 0;  // Coefficient 2 of code phase offset model [s/s^2]

    b_integrity_status_flag = false;
    b_alert_flag = false;         // If true, indicates  that the SV URA may be worse than indicated in d_SV_accuracy, use that SV at our own risk.
    b_antispoofing_flag = false;  //  If true, the AntiSpoofing mode is ON in that SV

    auto gnss_sat = Gnss_Satellite();
    std::string _system("Beidou");
    for (unsigned int i = 1; i < 36; i++)
        {
            satelliteBlock[i] = gnss_sat.what_block(_system, i);
        }

    d_satClkDrift = 0.0;
    d_dtr = 0.0;
    d_satpos_X = 0.0;
    d_satpos_Y = 0.0;
    d_satpos_Z = 0.0;
    d_satvel_X = 0.0;
    d_satvel_Y = 0.0;
    d_satvel_Z = 0.0;
}


double Beidou_Dnav_Ephemeris::check_t(double time)
{
    double corrTime;
    double half_week = 302400.0;  // seconds
    corrTime = time;
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


// 20.3.3.3.3.1 User Algorithm for SV Clock Correction.
double Beidou_Dnav_Ephemeris::sv_clock_drift(double transmitTime)
{
    double dt;
    dt = check_t(transmitTime - d_Toc);

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
    double tk;
    double a;
    double n;
    double n0;
    double E;
    double E_old;
    double dE;
    double M;

    // Restore semi-major axis
    a = d_sqrt_A * d_sqrt_A;

    // Time from ephemeris reference epoch
    tk = check_t(transmitTime - d_Toe);

    // Computed mean motion
    n0 = sqrt(BEIDOU_DNAV_GM / (a * a * a));
    // Corrected mean motion
    n = n0 + d_Delta_n;
    // Mean anomaly
    M = d_M_0 + n * tk;

    // Reduce mean anomaly to between 0 and 2pi
    M = fmod((M + 2.0 * BEIDOU_DNAV_PI), (2.0 * BEIDOU_DNAV_PI));

    // Initial guess of eccentric anomaly
    E = M;

    // --- Iteratively compute eccentric anomaly ----------------------------
    for (int ii = 1; ii < 20; ii++)
        {
            E_old = E;
            E = M + d_eccentricity * sin(E);
            dE = fmod(E - E_old, 2.0 * BEIDOU_DNAV_PI);
            if (fabs(dE) < 1e-12)
                {
                    //Necessary precision is reached, exit from the loop
                    break;
                }
        }

    // Compute relativistic correction term
    d_dtr = BEIDOU_DNAV_F * d_eccentricity * d_sqrt_A * sin(E);
    return d_dtr;
}


double Beidou_Dnav_Ephemeris::satellitePosition(double transmitTime)
{
    double tk;
    double a;
    double n;
    double n0;
    double M;
    double E;
    double E_old;
    double dE;
    double nu;
    double phi;
    double u;
    double r;
    double i;
    double Omega;

    // Find satellite's position ----------------------------------------------

    // Restore semi-major axis
    a = d_sqrt_A * d_sqrt_A;

    // Time from ephemeris reference epoch
    tk = check_t(transmitTime - d_Toe);

    // Computed mean motion
    n0 = sqrt(BEIDOU_DNAV_GM / (a * a * a));

    // Corrected mean motion
    n = n0 + d_Delta_n;

    // Mean anomaly
    M = d_M_0 + n * tk;

    // Reduce mean anomaly to between 0 and 2pi
    M = fmod((M + 2.0 * BEIDOU_DNAV_PI), (2.0 * BEIDOU_DNAV_PI));

    // Initial guess of eccentric anomaly
    E = M;

    // --- Iteratively compute eccentric anomaly ----------------------------
    for (int ii = 1; ii < 20; ii++)
        {
            E_old = E;
            E = M + d_eccentricity * sin(E);
            dE = fmod(E - E_old, 2.0 * BEIDOU_DNAV_PI);
            if (fabs(dE) < 1e-12)
                {
                    //Necessary precision is reached, exit from the loop
                    break;
                }
        }

    // Compute the true anomaly
    double tmp_Y = sqrt(1.0 - d_eccentricity * d_eccentricity) * sin(E);
    double tmp_X = cos(E) - d_eccentricity;
    nu = atan2(tmp_Y, tmp_X);

    // Compute angle phi (argument of Latitude)
    phi = nu + d_OMEGA;

    // Reduce phi to between 0 and 2*pi rad
    phi = fmod((phi), (2.0 * BEIDOU_DNAV_PI));

    // Correct argument of latitude
    u = phi + d_Cuc * cos(2.0 * phi) + d_Cus * sin(2.0 * phi);

    // Correct radius
    r = a * (1.0 - d_eccentricity * cos(E)) + d_Crc * cos(2.0 * phi) + d_Crs * sin(2.0 * phi);

    // Correct inclination
    i = d_i_0 + d_IDOT * tk + d_Cic * cos(2.0 * phi) + d_Cis * sin(2.0 * phi);

    // Compute the angle between the ascending node and the Greenwich meridian
    Omega = d_OMEGA0 + (d_OMEGA_DOT - BEIDOU_DNAV_OMEGA_EARTH_DOT) * tk - BEIDOU_DNAV_OMEGA_EARTH_DOT * d_Toe;

    // Reduce to between 0 and 2*pi rad
    Omega = fmod((Omega + 2.0 * BEIDOU_DNAV_PI), (2.0 * BEIDOU_DNAV_PI));

    // --- Compute satellite coordinates in Earth-fixed coordinates
    d_satpos_X = cos(u) * r * cos(Omega) - sin(u) * r * cos(i) * sin(Omega);
    d_satpos_Y = cos(u) * r * sin(Omega) + sin(u) * r * cos(i) * cos(Omega);
    d_satpos_Z = sin(u) * r * sin(i);

    // Satellite's velocity. Can be useful for Vector Tracking loops
    double Omega_dot = d_OMEGA_DOT - BEIDOU_DNAV_OMEGA_EARTH_DOT;
    d_satvel_X = -Omega_dot * (cos(u) * r + sin(u) * r * cos(i)) + d_satpos_X * cos(Omega) - d_satpos_Y * cos(i) * sin(Omega);
    d_satvel_Y = Omega_dot * (cos(u) * r * cos(Omega) - sin(u) * r * cos(i) * sin(Omega)) + d_satpos_X * sin(Omega) + d_satpos_Y * cos(i) * cos(Omega);
    d_satvel_Z = d_satpos_Y * sin(i);

    // Time from ephemeris reference clock
    tk = check_t(transmitTime - d_Toc);

    double dtr_s = d_A_f0 + d_A_f1 * tk + d_A_f2 * tk * tk;

    /* relativity correction */
    dtr_s -= 2.0 * sqrt(BEIDOU_DNAV_GM * a) * d_eccentricity * sin(E) / (BEIDOU_DNAV_C_M_S * BEIDOU_DNAV_C_M_S);

    return dtr_s;
}
