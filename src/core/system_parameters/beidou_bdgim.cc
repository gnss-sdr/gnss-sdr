/*!
 * \file beidou_bdgim.cc
 * \brief BeiDou Global Ionospheric delay Model (BDGIM), BDS-SIS-ICD-B1C-1.0 §7.8
 * \author Wenhao Ou, 2026. ouwh(at)mail2.sysu.edu.cn
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */
#include "beidou_bdgim.h"
#include "MATH_CONSTANTS.h"
#include <cmath>

namespace
{
constexpr double K_BDGIM_HION_M = 400000.0;
constexpr double K_BDGIM_RE_M = 6378000.0;
constexpr double K_BDGIM_LAT_M_RAD = 80.27 * GNSS_PI / 180.0;
constexpr double K_BDGIM_LON_M_RAD = -72.58 * GNSS_PI / 180.0;
constexpr double K_BDGIM_TEC_FACTOR = 40.28e16;

/* Table 7-11: (n,m) for broadcast Ai, i=1..9 */
const int K_AI_N[9] = {0, 1, 1, 1, 2, 2, 2, 2, 2};
const int K_AI_M[9] = {0, 0, 1, -1, 0, 1, -1, 2, -2};

/* Table 7-12: (n,m) for non-broadcast Bj, j=1..17 */
const int K_BJ_N[17] = {3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5};
const int K_BJ_M[17] = {0, 1, -1, 2, -2, 3, -3, 0, 1, -1, 2, -2, 0, 1, -1, 2, -2};

/* Periods Tk [days] for k=1..12 (k=0 is a0 constant) */
const double K_PERIOD_DAYS[13] = {
    0.0, 1.0, 0.5, 0.33, 14.6, 27.0, 121.6, 182.51, 365.25, 4028.71, 2014.35, 1342.90, 1007.18};

/* a0,j for j=1..17 */
const double K_A0[17] = {
    -0.61, -1.31, -2.00, -0.03, 0.15, -0.48, -0.40, 2.28, -0.16, -0.21, -0.10, -0.13, 0.21, 0.68, 1.06, 0.0, -0.12};

/* ak,j and bk,j for k=1..12, j=1..17 (row-major: [k-1][j-1]) */
const double K_AK[12][17] = {
    {-0.51, -0.43, 0.34, -0.01, 0.17, 0.02, -0.06, 0.30, 0.44, -0.28, -0.31, -0.17, 0.04, 0.39, -0.12, 0.12, 0.0},
    {-0.06, -0.05, 0.06, 0.17, 0.15, 0.0, 0.11, -0.05, -0.16, 0.02, 0.11, 0.04, 0.12, 0.07, 0.02, -0.14, -0.14},
    {0.01, -0.03, 0.01, -0.01, 0.05, -0.03, 0.05, -0.03, -0.01, 0.0, -0.08, -0.04, 0.0, -0.02, -0.03, 0.0, -0.03},
    {-0.01, 0.0, 0.01, 0.0, 0.01, 0.0, -0.01, -0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.03, 0.01, 0.02, 0.01, 0.0, -0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {-0.19, -0.02, 0.12, -0.10, 0.06, 0.0, -0.02, -0.08, -0.02, -0.07, 0.01, 0.03, 0.15, 0.06, -0.05, -0.03, -0.10},
    {-0.18, 0.06, -0.55, -0.02, 0.09, -0.08, 0.0, 0.86, -0.18, -0.05, -0.07, 0.04, 0.14, -0.03, 0.37, -0.11, -0.12},
    {1.09, -0.14, -0.21, 0.52, 0.27, 0.0, 0.11, 0.17, 0.23, 0.35, -0.05, 0.02, -0.60, 0.02, 0.01, 0.27, 0.32},
    {-0.34, -0.09, -1.22, 0.05, 0.15, -0.29, -0.17, 1.58, -0.06, -0.15, 0.0, 0.13, 0.28, -0.08, 0.62, -0.01, -0.04},
    {-0.13, 0.07, -0.37, 0.05, 0.06, -0.11, -0.07, 0.46, 0.0, -0.04, 0.01, 0.07, 0.09, -0.05, 0.15, -0.01, 0.01},
    {-0.06, 0.13, -0.07, 0.03, 0.02, -0.05, -0.05, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {-0.03, 0.08, -0.01, 0.04, 0.01, -0.02, -0.02, -0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

const double K_BK[12][17] = {
    {0.23, -0.20, -0.31, 0.16, -0.03, 0.02, 0.04, 0.18, 0.34, 0.45, 0.19, -0.25, -0.12, 0.18, 0.40, -0.09, 0.21},
    {0.02, -0.08, -0.06, -0.11, 0.15, -0.14, 0.01, 0.01, 0.04, -0.14, -0.05, 0.08, 0.08, -0.01, 0.01, 0.11, -0.12},
    {0.0, -0.02, -0.03, -0.05, -0.01, -0.07, -0.03, -0.01, 0.02, -0.01, 0.03, -0.10, 0.01, 0.05, -0.01, 0.04, 0.0},
    {0.0, -0.02, 0.01, 0.0, -0.01, 0.01, 0.0, -0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.01, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {-0.09, 0.07, 0.03, 0.06, 0.09, 0.01, 0.02, 0.0, -0.04, -0.02, -0.01, 0.01, -0.10, 0.0, -0.01, 0.02, 0.05},
    {0.15, -0.31, 0.13, 0.05, -0.09, -0.03, 0.06, -0.36, 0.08, 0.05, 0.06, -0.02, -0.05, 0.06, -0.20, 0.04, 0.07},
    {0.50, -0.08, -0.38, 0.36, 0.14, 0.04, 0.0, 0.25, 0.17, 0.27, -0.03, -0.03, -0.32, -0.10, 0.20, 0.10, 0.30},
    {0.0, -0.11, -0.22, 0.01, 0.02, -0.03, -0.01, 0.49, -0.03, -0.02, 0.01, 0.02, 0.04, -0.04, 0.16, -0.02, -0.01},
    {0.05, 0.03, 0.07, 0.02, -0.01, 0.03, 0.02, -0.04, -0.01, -0.01, 0.02, 0.03, 0.02, -0.04, -0.04, -0.01, 0.0},
    {0.03, -0.02, 0.04, -0.01, -0.03, 0.02, 0.01, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.04, -0.02, -0.04, 0.0, -0.01, 0.0, 0.01, 0.07, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

double factorial_d(int n)
{
    double f = 1.0;
    for (int i = 2; i <= n; i++)
        {
            f *= static_cast<double>(i);
        }
    return f;
}

double double_factorial_odd(int n)
{
    /* (2n-1)!! */
    double f = 1.0;
    for (int k = 1; k <= n; k++)
        {
            f *= static_cast<double>(2 * k - 1);
        }
    return f;
}

double norm_factor(int n, int m)
{
    const int am = (m < 0) ? -m : m;
    const double delta = (am == 0) ? 1.0 : 0.0;
    const double num = factorial_d(n - am) * (2.0 * static_cast<double>(n) + 1.0) * (2.0 - delta);
    const double den = factorial_d(n + am);
    return std::sqrt(num / den);
}

/* Associated Legendre P_n,m(x) for m>=0 via recurrence (ICD 7-13) */
double legendre_p(int n, int m, double x)
{
    if (n < 0 || m < 0 || m > n)
        {
            return 0.0;
        }
    if (n == 0 && m == 0)
        {
            return 1.0;
        }
    if (n == m)
        {
            const double s = 1.0 - x * x;
            if (s < 0.0)
                {
                    return 0.0;
                }
            return double_factorial_odd(n) * std::pow(s, 0.5 * static_cast<double>(n));
        }
    if (n == m + 1)
        {
            return x * (2.0 * static_cast<double>(m) + 1.0) * legendre_p(m, m, x);
        }
    return ((2.0 * static_cast<double>(n) - 1.0) * x * legendre_p(n - 1, m, x) -
               (static_cast<double>(n + m) - 1.0) * legendre_p(n - 2, m, x)) /
           static_cast<double>(n - m);
}

double spherical_harmonic(int n, int m, double phi_p, double lam_p)
{
    const int am = (m < 0) ? -m : m;
    const double pnm = norm_factor(n, am) * legendre_p(n, am, std::sin(phi_p));
    if (m >= 0)
        {
            return pnm * std::cos(static_cast<double>(m) * lam_p);
        }
    return pnm * std::sin(static_cast<double>(-m) * lam_p);
}

double mapping_function(double el_rad)
{
    const double sin_arg = (K_BDGIM_RE_M / (K_BDGIM_RE_M + K_BDGIM_HION_M)) * std::cos(el_rad);
    if (sin_arg <= -1.0 || sin_arg >= 1.0)
        {
            return 1.0;
        }
    return 1.0 / std::sqrt(1.0 - sin_arg * sin_arg);
}
}  // namespace


double beidou_bdgim_delay_m(
    double time_mjd,
    double lat_rad,
    double lon_rad,
    double az_rad,
    double el_rad,
    const double alpha[9],
    double freq_hz)
{
    if (el_rad <= 0.0 || freq_hz <= 0.0)
        {
            return 0.0;
        }

    /* IPP earth-centered angle (7-7) */
    const double sin_arg = (K_BDGIM_RE_M / (K_BDGIM_RE_M + K_BDGIM_HION_M)) * std::cos(el_rad);
    if (sin_arg <= -1.0 || sin_arg >= 1.0)
        {
            return 0.0;
        }
    const double psi = GNSS_PI / 2.0 - el_rad - std::asin(sin_arg);

    /* Geographic lat/lon of IPP (7-8) */
    const double sin_phi_g = std::sin(lat_rad) * std::cos(psi) +
                             std::cos(lat_rad) * std::sin(psi) * std::cos(az_rad);
    double phi_g = std::asin(sin_phi_g);
    const double lam_g = lon_rad + std::atan2(
                                       std::sin(psi) * std::sin(az_rad) * std::cos(lat_rad),
                                       std::cos(psi) - std::sin(lat_rad) * std::sin(phi_g));

    /* Geomagnetic lat/lon (7-9) */
    const double sin_phi_m = std::sin(K_BDGIM_LAT_M_RAD) * std::sin(phi_g) +
                             std::cos(K_BDGIM_LAT_M_RAD) * std::cos(phi_g) * std::cos(lam_g - K_BDGIM_LON_M_RAD);
    double phi_m = std::asin(sin_phi_m);
    const double lam_m = std::atan2(
        std::cos(phi_g) * std::sin(lam_g - K_BDGIM_LON_M_RAD) * std::cos(K_BDGIM_LAT_M_RAD),
        std::sin(K_BDGIM_LAT_M_RAD) * std::sin(phi_m) - std::sin(phi_g));

    /* Sun-fixed geomagnetic (7-10) */
    const double frac_day = time_mjd - std::floor(time_mjd);
    const double slon = GNSS_PI * (1.0 - 2.0 * frac_day);
    const double phi_p = phi_m;
    const double lam_p = lam_m - std::atan2(
                                     std::sin(slon - K_BDGIM_LON_M_RAD),
                                     std::sin(K_BDGIM_LAT_M_RAD) * std::cos(slon - K_BDGIM_LON_M_RAD));

    /* Ai from broadcast alphas (7-11) */
    double sum_alpha_a = 0.0;
    for (int i = 0; i < 9; i++)
        {
            sum_alpha_a += alpha[i] * spherical_harmonic(K_AI_N[i], K_AI_M[i], phi_p, lam_p);
        }

    /* A0 from non-broadcast betas (7-14)(7-15) */
    double a0 = 0.0;
    const double tp = time_mjd;
    for (int j = 0; j < 17; j++)
        {
            double beta = K_A0[j];
            for (int k = 1; k <= 12; k++)
                {
                    const double omega = 2.0 * GNSS_PI / K_PERIOD_DAYS[k];
                    beta += K_AK[k - 1][j] * std::cos(omega * tp) + K_BK[k - 1][j] * std::sin(omega * tp);
                }
            a0 += beta * spherical_harmonic(K_BJ_N[j], K_BJ_M[j], phi_p, lam_p);
        }

    const double vtec = a0 + sum_alpha_a; /* TECu */
    const double mf = mapping_function(el_rad);
    return mf * (K_BDGIM_TEC_FACTOR / (freq_hz * freq_hz)) * vtec;
}
