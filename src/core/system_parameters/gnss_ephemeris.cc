/*!
 * \file gnss_ephemeris.cc
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

#include "gnss_ephemeris.h"
#include "MATH_CONSTANTS.h"
#include <cmath>


void Gnss_Ephemeris::satellitePosition(double transmitTime)
{
    // Restore semi-major axis
    const double a = this->sqrtA * this->sqrtA;

    // Computed mean motion
    double n0;
    if (this->System == 'E')
        {
            n0 = sqrt(GALILEO_GM / (a * a * a));
        }
    else if (this->System == 'B')
        {
            n0 = sqrt(BEIDOU_GM / (a * a * a));
        }
    else
        {
            n0 = sqrt(GPS_GM / (a * a * a));
        }

    // Time from ephemeris reference epoch
    double tk = check_t(transmitTime - static_cast<double>(this->toe));

    // Corrected mean motion
    const double n = n0 + this->delta_n;

    // Mean anomaly
    double M = this->M_0 + n * tk;

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
            E = M + this->ecc * sin(E);
            dE = fmod(E - E_old, 2.0 * GNSS_PI);
            if (fabs(dE) < 1e-12)
                {
                    // Necessary precision is reached, exit from the loop
                    break;
                }
        }

    const double sek = sin(E);
    const double cek = cos(E);
    const double OneMinusecosE = 1.0 - this->ecc * cek;
    const double sq1e2 = sqrt(1.0 - this->ecc * this->ecc);
    const double ekdot = n / OneMinusecosE;

    // Compute the true anomaly
    const double tmp_Y = sq1e2 * sek;
    const double tmp_X = cek - this->ecc;
    const double nu = atan2(tmp_Y, tmp_X);

    // Compute angle phi (argument of Latitude)
    double phi = nu + this->omega;

    // Reduce phi to between 0 and 2*pi rad
    phi = fmod((phi), (2.0 * GNSS_PI));
    const double s2pk = sin(2.0 * phi);
    const double c2pk = cos(2.0 * phi);
    const double pkdot = sq1e2 * ekdot / OneMinusecosE;

    // Correct argument of latitude
    const double u = phi + this->Cuc * c2pk + this->Cus * s2pk;
    const double suk = sin(u);
    const double cuk = cos(u);
    const double ukdot = pkdot * (1.0 + 2.0 * (this->Cus * c2pk - this->Cuc * s2pk));

    // Correct radius
    const double r = a * OneMinusecosE + this->Crc * c2pk + this->Crs * s2pk;
    const double rkdot = a * this->ecc * sek * ekdot + 2.0 * pkdot * (this->Crs * c2pk - this->Crc * s2pk);

    // Correct inclination
    const double i = this->i_0 + this->idot * tk + this->Cic * c2pk + this->Cis * s2pk;
    const double sik = sin(i);
    const double cik = cos(i);
    const double ikdot = this->idot + 2.0 * pkdot * (this->Cis * c2pk - this->Cic * s2pk);

    // Compute the angle between the ascending node and the Greenwich meridian
    double Omega;
    double Omega_dot;
    if (this->System == 'B')
        {
            Omega_dot = this->OMEGAdot - BEIDOU_OMEGA_EARTH_DOT;
            Omega = this->OMEGA_0 + Omega_dot * tk - BEIDOU_OMEGA_EARTH_DOT * static_cast<double>(this->toe);
        }
    else
        {
            Omega_dot = this->OMEGAdot - GNSS_OMEGA_EARTH_DOT;
            Omega = this->OMEGA_0 + Omega_dot * tk - GNSS_OMEGA_EARTH_DOT * static_cast<double>(this->toe);
        }

    // Reduce to between 0 and 2*pi rad
    Omega = fmod((Omega + 2.0 * GNSS_PI), (2.0 * GNSS_PI));
    const double sok = sin(Omega);
    const double cok = cos(Omega);

    // --- Compute satellite coordinates in Earth-fixed coordinates
    const double xprime = r * cuk;
    const double yprime = r * suk;
    this->satpos_X = xprime * cok - yprime * cik * sok;
    this->satpos_Y = xprime * sok + yprime * cik * cok;  // ********NOTE: in GALILEO ICD this expression is not correct because it has minus (- sin(u) * r * cos(i) * cos(Omega)) instead of plus
    this->satpos_Z = yprime * sik;

    // Satellite's velocity. Can be useful for Vector Tracking loops
    const double xpkdot = rkdot * cuk - yprime * ukdot;
    const double ypkdot = rkdot * suk + xprime * ukdot;
    const double tmp = ypkdot * cik - this->satpos_Z * ikdot;

    this->satvel_X = -Omega_dot * this->satpos_Y + xpkdot * cok - tmp * sok;
    this->satvel_Y = Omega_dot * this->satpos_X + xpkdot * sok + tmp * cok;
    this->satvel_Z = yprime * cik * ikdot + ypkdot * sik;

    // Time from ephemeris reference clock
    tk = check_t(transmitTime - this->toc);

    this->dtr = this->af0 + this->af1 * tk + this->af2 * tk * tk;

    if (this->System == 'E')
        {
            this->dtr -= 2.0 * sqrt(GALILEO_GM * a) * this->ecc * sek / (SPEED_OF_LIGHT_M_S * SPEED_OF_LIGHT_M_S);
        }
    else if (this->System == 'B')
        {
            this->dtr -= 2.0 * sqrt(BEIDOU_GM * a) * this->ecc * sek / (SPEED_OF_LIGHT_M_S * SPEED_OF_LIGHT_M_S);
        }
    else
        {
            this->dtr -= 2.0 * sqrt(GPS_GM * a) * this->ecc * sek / (SPEED_OF_LIGHT_M_S * SPEED_OF_LIGHT_M_S);
        }
}


double Gnss_Ephemeris::sv_clock_drift(double transmitTime)
{
    const double dt = check_t(transmitTime - this->toc);
    this->dtr = sv_clock_relativistic_term(transmitTime);
    this->satClkDrift = this->af0 + this->af1 * dt + this->af2 * (dt * dt) + this->dtr;
    return this->satClkDrift;
}


double Gnss_Ephemeris::sv_clock_relativistic_term(double transmitTime) const
{
    // Restore semi-major axis
    const double a = this->sqrtA * this->sqrtA;

    // Time from ephemeris reference epoch
    const double tk = check_t(transmitTime - this->toe);

    // Computed mean motion
    double n0;
    if (this->System == 'E')
        {
            n0 = sqrt(GALILEO_GM / (a * a * a));
        }
    else if (this->System == 'E')
        {
            n0 = sqrt(BEIDOU_GM / (a * a * a));
        }
    else
        {
            n0 = sqrt(GPS_GM / (a * a * a));
        }
    // Corrected mean motion
    const double n = n0 + this->delta_n;

    // Mean anomaly
    const double M = this->M_0 + n * tk;

    // Reduce mean anomaly to between 0 and 2pi
    // M = fmod((M + 2.0 * GNSS_PI), (2.0 * GNSS_PI));

    // Initial guess of eccentric anomaly
    double E = M;
    double E_old;
    double dE;
    // --- Iteratively compute eccentric anomaly ----------------------------
    for (int32_t ii = 1; ii < 20; ii++)
        {
            E_old = E;
            E = M + this->ecc * sin(E);
            dE = fmod(E - E_old, 2.0 * GNSS_PI);
            if (fabs(dE) < 1e-12)
                {
                    // Necessary precision is reached, exit from the loop
                    break;
                }
        }
    const double sek = sin(E);

    // Compute relativistic correction term
    double dtr_;
    if (this->System == 'E')
        {
            dtr_ = GALILEO_F * this->ecc * this->sqrtA * sek;
        }
    else if (this->System == 'B')
        {
            dtr_ = BEIDOU_F * this->ecc * this->sqrtA * sek;
        }
    else
        {
            dtr_ = GPS_F * this->ecc * this->sqrtA * sek;
        }
    return dtr_;
}


double Gnss_Ephemeris::check_t(double time) const
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
