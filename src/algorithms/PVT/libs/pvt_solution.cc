/*!
 * \file pvt_solution.cc
 * \brief Implementation of a base class for a PVT solution
 * \author Carles Fernandez-Prades, 2015. cfernandez(at)cttc.es
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

#include "pvt_solution.h"
#include "MATH_CONSTANTS.h"
#include <glog/logging.h>
#include <cmath>
#include <cstddef>


int Pvt_Solution::cart2geo(double X, double Y, double Z, int elipsoid_selection)
{
    /* Conversion of Cartesian coordinates (X,Y,Z) to geographical
     coordinates (latitude, longitude, h) on a selected reference ellipsoid.

       Choices of Reference Ellipsoid for Geographical Coordinates
                 0. International Ellipsoid 1924
                 1. International Ellipsoid 1967
                 2. World Geodetic System 1972
                 3. Geodetic Reference System 1980
                 4. World Geodetic System 1984
     */

    const std::array<double, 5> a = {6378388.0, 6378160.0, 6378135.0, 6378137.0, 6378137.0};
    const std::array<double, 5> f = {1.0 / 297.0, 1.0 / 298.247, 1.0 / 298.26, 1.0 / 298.257222101, 1.0 / 298.257223563};

    const double lambda = atan2(Y, X);
    const double ex2 = (2.0 - f[elipsoid_selection]) * f[elipsoid_selection] / ((1.0 - f[elipsoid_selection]) * (1.0 - f[elipsoid_selection]));
    const double c = a[elipsoid_selection] * sqrt(1.0 + ex2);
    double phi = atan(Z / ((sqrt(X * X + Y * Y) * (1.0 - (2.0 - f[elipsoid_selection])) * f[elipsoid_selection])));

    double h = 0.1;
    double oldh = 0.0;
    double N;
    int iterations = 0;
    do
        {
            oldh = h;
            N = c / sqrt(1 + ex2 * (cos(phi) * cos(phi)));
            phi = atan(Z / ((sqrt(X * X + Y * Y) * (1.0 - (2.0 - f[elipsoid_selection]) * f[elipsoid_selection] * N / (N + h)))));
            h = sqrt(X * X + Y * Y) / cos(phi) - N;
            iterations = iterations + 1;
            if (iterations > 100)
                {
                    DLOG(WARNING) << "Failed to approximate h with desired precision. h-oldh= " << h - oldh;
                    break;
                }
        }
    while (std::abs(h - oldh) > 1.0e-12);

    d_latitude_d = phi * R2D;
    d_longitude_d = lambda * R2D;
    d_height_m = h;
    return 0;
}


double Pvt_Solution::get_time_offset_s() const
{
    return d_rx_dt_s;
}


void Pvt_Solution::set_time_offset_s(double offset)
{
    d_rx_dt_s = offset;
}


double Pvt_Solution::get_clock_drift_ppm() const
{
    return d_rx_clock_drift_ppm;
}


void Pvt_Solution::set_clock_drift_ppm(double clock_drift_ppm)
{
    d_rx_clock_drift_ppm = clock_drift_ppm;
}


double Pvt_Solution::get_latitude() const
{
    return d_latitude_d;
}


double Pvt_Solution::get_longitude() const
{
    return d_longitude_d;
}


double Pvt_Solution::get_height() const
{
    return d_height_m;
}


double Pvt_Solution::get_speed_over_ground() const
{
    return d_speed_over_ground_m_s;
}


void Pvt_Solution::set_speed_over_ground(double speed_m_s)
{
    d_speed_over_ground_m_s = speed_m_s;
}


void Pvt_Solution::set_course_over_ground(double cog_deg)
{
    d_course_over_ground_d = cog_deg;
}


double Pvt_Solution::get_course_over_ground() const
{
    return d_course_over_ground_d;
}


bool Pvt_Solution::is_valid_position() const
{
    return d_valid_position;
}


void Pvt_Solution::set_valid_position(bool is_valid)
{
    d_valid_position = is_valid;
}


void Pvt_Solution::set_rx_pos(const std::array<double, 3> &pos)
{
    d_rx_pos = pos;
    Pvt_Solution::cart2geo(d_rx_pos[0], d_rx_pos[1], d_rx_pos[2], 4);
}


std::array<double, 3> Pvt_Solution::get_rx_pos() const
{
    return d_rx_pos;
}


void Pvt_Solution::set_rx_vel(const std::array<double, 3> &vel)
{
    d_rx_vel = vel;
}


std::array<double, 3> Pvt_Solution::get_rx_vel() const
{
    return d_rx_vel;
}


boost::posix_time::ptime Pvt_Solution::get_position_UTC_time() const
{
    return d_position_UTC_time;
}


void Pvt_Solution::set_position_UTC_time(const boost::posix_time::ptime &pt)
{
    d_position_UTC_time = pt;
}


int Pvt_Solution::get_num_valid_observations() const
{
    return d_valid_observations;
}


void Pvt_Solution::set_num_valid_observations(int num)
{
    d_valid_observations = num;
}


void Pvt_Solution::set_pre_2009_file(bool pre_2009_file)
{
    d_pre_2009_file = pre_2009_file;
}


bool Pvt_Solution::is_pre_2009() const
{
    return d_pre_2009_file;
}
