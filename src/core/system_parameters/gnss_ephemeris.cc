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
#include "gnss_frequencies.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>
#include <vector>


double Gnss_Ephemeris::sv_clock_drift(double transmitTime)
{
    const double dt = check_t(transmitTime - this->toc);
    this->dtr = sv_clock_relativistic_term(transmitTime);
    this->satClkDrift = this->af0 + this->af1 * dt + this->af2 * (dt * dt) + this->dtr;
    return this->satClkDrift;
}


double Gnss_Ephemeris::predicted_doppler(double rx_time_s,
    double lat,
    double lon,
    double h,
    double ve,
    double vn,
    double vu,
    int band) const
{
    const double RE_WGS84 = 6378137.0;              //!< earth semimajor axis (WGS84) (m)
    const double FE_WGS84 = (1.0 / 298.257223563);  //!< earth flattening (WGS84)
    const double lat_rad = lat * D2R;
    const double lon_rad = lon * D2R;

    const double sinp = sin(lat_rad);
    const double cosp = cos(lat_rad);
    const double sinl = sin(lon_rad);
    const double cosl = cos(lon_rad);

    const double e2 = FE_WGS84 * (2.0 - FE_WGS84);
    const double v = RE_WGS84 / std::sqrt(1.0 - e2 * sinp * sinp);

    // Position in EFEF
    const std::vector<double> pos_rx = {(v + h) * cosp * cosl, (v + h) * cosp * sinl, (v * (1.0 - e2) + h) * sinp};

    // Velocity in EFEF
    const double t = cosp * vu - sinp * vn;
    const std::vector<double> vel_rx = {cosl * t - sinl * ve, sinl * t + cosl * ve, sinp * vu + cosp * vn};

    std::array<double, 7> sat_pos_vel = {0};
    satellitePosVelComputation(rx_time_s, sat_pos_vel);
    const std::vector<double> pos_sat = {sat_pos_vel[0], sat_pos_vel[1], sat_pos_vel[2]};
    const std::vector<double> vel_sat = {sat_pos_vel[3], sat_pos_vel[4], sat_pos_vel[5]};

    std::vector<double> x_sr = pos_sat;
    std::transform(x_sr.begin(), x_sr.end(), pos_rx.begin(), x_sr.begin(), std::minus<double>());  // pos_sat - pos_rx

    const double norm_x_sr = std::sqrt(std::inner_product(x_sr.begin(), x_sr.end(), x_sr.begin(), 0.0));  // Euclidean norm

    std::vector<double> v_sr = vel_sat;
    std::transform(v_sr.begin(), v_sr.end(), vel_rx.begin(), v_sr.begin(), std::minus<double>());  // vel_sat - vel_rx

    const double radial_vel = std::inner_product(v_sr.begin(), v_sr.end(), x_sr.begin(), 0.0) / norm_x_sr;
    const double predicted_doppler_normalized = -(radial_vel / SPEED_OF_LIGHT_M_S);
    double predicted_doppler = 0.0;
    if (this->System == 'E')  // Galileo
        {
            if (band == 1)
                {
                    predicted_doppler = predicted_doppler_normalized * FREQ1;
                }
            else if (band == 5)
                {
                    predicted_doppler = predicted_doppler_normalized * FREQ5;
                }
            else if (band == 6)
                {
                    predicted_doppler = predicted_doppler_normalized * FREQ6;
                }
            else if (band == 7)
                {
                    predicted_doppler = predicted_doppler_normalized * FREQ7;
                }
            else if (band == 8)
                {
                    predicted_doppler = predicted_doppler_normalized * FREQ8;
                }
            else
                {
                    predicted_doppler = 0.0;
                }
        }
    else if (this->System == 'G')  // GPS
        {
            if (band == 1)
                {
                    predicted_doppler = predicted_doppler_normalized * FREQ1;
                }
            else if (band == 2)
                {
                    predicted_doppler = predicted_doppler_normalized * FREQ2;
                }
            else if (band == 5)
                {
                    predicted_doppler = predicted_doppler_normalized * FREQ5;
                }
            else
                {
                    predicted_doppler = 0.0;
                }
        }
    else if (this->System == 'B')  // Beidou
        {
            if (band == 1)
                {
                    predicted_doppler = predicted_doppler_normalized * FREQ1_BDS;
                }
            else if (band == 2)
                {
                    predicted_doppler = predicted_doppler_normalized * FREQ2_BDS;
                }
            else if (band == 3)
                {
                    predicted_doppler = predicted_doppler_normalized * FREQ3_BDS;
                }
            else
                {
                    predicted_doppler = 0.0;
                }
        }
    else
        {
            predicted_doppler = 0.0;
        }
    return predicted_doppler;
}


void Gnss_Ephemeris::satellitePosition(double transmitTime)
{
    std::array<double, 7> pos_vel_dtr = {0};
    satellitePosVelComputation(transmitTime, pos_vel_dtr);
    this->satpos_X = pos_vel_dtr[0];
    this->satpos_Y = pos_vel_dtr[1];
    this->satpos_Z = pos_vel_dtr[2];
    this->satvel_X = pos_vel_dtr[3];
    this->satvel_Y = pos_vel_dtr[4];
    this->satvel_Z = pos_vel_dtr[5];
    this->dtr = pos_vel_dtr[6];
}


void Gnss_Ephemeris::satellitePosVelComputation(double transmitTime, std::array<double, 7>& pos_vel_dtr) const
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
    const double M = this->M_0 + n * tk;

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
    const double phi = nu + this->omega;

    // Reduce phi to between 0 and 2*pi rad
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

    const double sok = sin(Omega);
    const double cok = cos(Omega);

    // --- Compute satellite coordinates in Earth-fixed coordinates
    const double xprime = r * cuk;
    const double yprime = r * suk;

    pos_vel_dtr[0] = xprime * cok - yprime * cik * sok;
    pos_vel_dtr[1] = xprime * sok + yprime * cik * cok;  // ********NOTE: in GALILEO ICD this expression is not correct because it has minus (- sin(u) * r * cos(i) * cos(Omega)) instead of plus
    pos_vel_dtr[2] = yprime * sik;

    // Satellite's velocity. Can be useful for Vector Tracking loops
    const double xpkdot = rkdot * cuk - yprime * ukdot;
    const double ypkdot = rkdot * suk + xprime * ukdot;
    const double tmp = ypkdot * cik - pos_vel_dtr[2] * ikdot;

    pos_vel_dtr[3] = -Omega_dot * pos_vel_dtr[1] + xpkdot * cok - tmp * sok;
    pos_vel_dtr[4] = Omega_dot * pos_vel_dtr[0] + xpkdot * sok + tmp * cok;
    pos_vel_dtr[5] = yprime * cik * ikdot + ypkdot * sik;

    // Time from ephemeris reference clock
    tk = check_t(transmitTime - this->toc);

    pos_vel_dtr[6] = this->af0 + this->af1 * tk + this->af2 * tk * tk;

    if (this->System == 'E')
        {
            pos_vel_dtr[6] -= 2.0 * sqrt(GALILEO_GM * a) * this->ecc * sek / (SPEED_OF_LIGHT_M_S * SPEED_OF_LIGHT_M_S);
        }
    else if (this->System == 'B')
        {
            pos_vel_dtr[6] -= 2.0 * sqrt(BEIDOU_GM * a) * this->ecc * sek / (SPEED_OF_LIGHT_M_S * SPEED_OF_LIGHT_M_S);
        }
    else
        {
            pos_vel_dtr[6] -= 2.0 * sqrt(GPS_GM * a) * this->ecc * sek / (SPEED_OF_LIGHT_M_S * SPEED_OF_LIGHT_M_S);
        }
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
    else if (this->System == 'B')
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
