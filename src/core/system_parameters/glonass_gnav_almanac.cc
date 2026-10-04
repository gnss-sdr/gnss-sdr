/*!
 * \file glonass_gnav_almanac.cc
 * \brief GLONASS almanac orbit propagation for satellite acquisition
 * \author Carles Fernandez, 2026. cfernandez(at)cttc.es
 *
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

#include "glonass_gnav_almanac.h"
#include <cmath>

std::array<double, 6> Glonass_Gnav_Almanac::perturbations(double a, double inclination,
    double l, double h, double longitude, double nt)
{
    // GLONASS ICD 5.1, A.3.2.2, equation (1). The first component is delta-a / a.
    const double j = 1.5 * 1082.63e-6 * std::pow(6378.136 / a, 2);
    const double si = std::sin(inclination);
    const double ci = std::cos(inclination);
    const double s1 = std::sin(longitude);
    const double c1 = std::cos(longitude);
    const double s2 = std::sin(2.0 * longitude);
    const double c2 = std::cos(2.0 * longitude);
    const double s3 = std::sin(3.0 * longitude);
    const double c3 = std::cos(3.0 * longitude);
    const double s4 = std::sin(4.0 * longitude);
    const double c4 = std::cos(4.0 * longitude);
    const double b = 1.0 - 1.5 * si * si;
    const double node = nt + 3.5 * l * s1 - 2.5 * h * c1 - 0.5 * s2 - 7.0 / 6.0 * l * s3 + 7.0 / 6.0 * h * c3;
    return {{2.0 * j * b * (l * c1 + h * s1) + j * si * si * (0.5 * h * s1 - 0.5 * l * c1 + c2 + 3.5 * l * c3 + 3.5 * h * s3),
        j * b * (l * nt + s1 + 1.5 * l * s2 - 1.5 * h * c2) - 0.25 * j * si * si * (s1 - 7.0 / 3.0 * s3 + 5.0 * l * s2 - 8.5 * l * s4 + 8.5 * h * c4 + h * c2) + j * ci * ci * (l * nt - 0.5 * l * s2),
        j * b * (-h * nt + c1 + 1.5 * l * c2 + 1.5 * h * s2) - 0.25 * j * si * si * (-c1 - 7.0 / 3.0 * c3 - 5.0 * h * s2 - 8.5 * l * c4 - 8.5 * h * s4 + l * c2) + j * ci * ci * (-h * nt + 0.5 * h * s2),
        -j * ci * node,
        0.5 * j * si * ci * (-l * c1 + h * s1 + c2 + 7.0 / 3.0 * l * c3 + 7.0 / 3.0 * h * s3),
        2.0 * j * b * (nt + 1.75 * l * s1 - 1.75 * h * c1) + 3.0 * j * si * si * (-7.0 / 24.0 * h * c1 - 7.0 / 24.0 * l * s1 - 49.0 / 72.0 * h * c3 + 49.0 / 72.0 * l * s3 + 0.25 * s2) + j * ci * ci * node}};
}


bool Glonass_Gnav_Almanac::satellite_position(double elapsed_s, std::array<double, 3>& position_m) const
{
    // ICD 5.1, Appendix A.3.2. Units inside the algorithm are kilometres/radians.
    const double pi = 3.14159265358979323846;
    const double mu = 398600.44;
    const double c20 = -1082.63e-6;
    const double earth_rate = 7.292115e-5;  // Table 3.2 (the A.3.2.2 value has a typographical error).
    const double e = d_epsilon_n_A;
    const double i = 63.0 * pi / 180.0 + d_Delta_i_n_A;
    const double period = 43200.0 + d_Delta_T_n_A;
    if (!std::isfinite(elapsed_s) || !std::isfinite(e) || e < 0.0 || e >= 1.0 ||
        !std::isfinite(i) || !std::isfinite(period) || period <= 0.0 ||
        !std::isfinite(d_Delta_T_n_A_dot) || !std::isfinite(d_omega_n_A) || !std::isfinite(d_lambda_n_A))
        {
            return false;
        }
    double a = std::cbrt(mu * std::pow(period / (2.0 * pi), 2));
    bool converged = false;
    for (int iteration = 0; iteration < 20; ++iteration)
        {
            const double p = a * (1.0 - e * e);
            const double node_factor = 1.0 + e * std::cos(d_omega_n_A);
            const double correction = 1.5 * c20 * std::pow(6378.136 / p, 2) *
                                      ((2.0 - 2.5 * std::pow(std::sin(i), 2)) * std::pow(1.0 - e * e, 1.5) / (node_factor * node_factor) +
                                          std::pow(node_factor, 3) / (1.0 - e * e));
            const double next = std::cbrt(mu * std::pow(period / ((1.0 + correction) * 2.0 * pi), 2));
            converged = std::abs(next - a) < 1.0e-3;
            a = next;
            if (converged)
                {
                    break;
                }
        }
    if (!converged || !std::isfinite(a) || a <= 6378.136)
        {
            return false;
        }
    const double revolutions = std::floor(elapsed_s / period);
    const double node_elapsed = period * revolutions + d_Delta_T_n_A_dot * revolutions * revolutions;
    const double tau = elapsed_s - node_elapsed;
    const double n = 2.0 * pi / period;
    const double node_rate = 1.5 * c20 * n * std::pow(6378.136 / a, 2) * std::cos(i) / std::pow(1.0 - e * e, 2);
    const double eccentric_node = 2.0 * std::atan2(std::sqrt(1.0 - e) * std::sin(-d_omega_n_A / 2.0),
                                            std::sqrt(1.0 + e) * std::cos(-d_omega_n_A / 2.0));
    const double mean_node = eccentric_node - e * std::sin(eccentric_node);
    const double l = e * std::cos(d_omega_n_A);
    const double h = e * std::sin(d_omega_n_A);
    const auto initial = perturbations(a, i, l, h, mean_node + d_omega_n_A, 0.0);
    const auto current = perturbations(a, i, l, h, mean_node + d_omega_n_A + n * tau, n * tau);
    const double hi = h + current[1] - initial[1];
    const double li = l + current[2] - initial[2];
    const double ei = std::hypot(hi, li);
    const double omega = std::atan2(hi, li);
    const double ai = a * (1.0 + current[0] - initial[0]);
    const double ii = i + current[4] - initial[4];
    // Rotate the inertial result back to ECEF at the query epoch. Sidereal
    // time cancels, leaving Earth rotation since the reference ascending node.
    const double node = d_lambda_n_A + node_rate * node_elapsed - earth_rate * elapsed_s + current[3] - initial[3];
    const double mean = std::remainder(mean_node + d_omega_n_A + n * tau + current[5] - initial[5] - omega, 2.0 * pi);
    if (!std::isfinite(ei) || ei >= 1.0 || !std::isfinite(mean) || ai <= 0.0)
        {
            return false;
        }
    double eccentric = mean;
    converged = false;
    for (int iteration = 0; iteration < 30; ++iteration)
        {
            const double correction = (eccentric - ei * std::sin(eccentric) - mean) / (1.0 - ei * std::cos(eccentric));
            eccentric -= correction;
            if (std::abs(correction) < 1.0e-12)
                {
                    converged = true;
                    break;
                }
        }
    if (!converged)
        {
            return false;
        }
    const double u = std::atan2(std::sqrt(1.0 - ei * ei) * std::sin(eccentric), std::cos(eccentric) - ei) + omega;
    const double radius_m = 1000.0 * ai * (1.0 - ei * std::cos(eccentric));
    position_m = {{radius_m * (std::cos(u) * std::cos(node) - std::sin(u) * std::sin(node) * std::cos(ii)),
        radius_m * (std::cos(u) * std::sin(node) + std::sin(u) * std::cos(node) * std::cos(ii)),
        radius_m * std::sin(u) * std::sin(ii)}};
    return std::isfinite(position_m[0]) && std::isfinite(position_m[1]) && std::isfinite(position_m[2]);
}
