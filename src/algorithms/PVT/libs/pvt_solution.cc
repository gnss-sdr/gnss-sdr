/*!
 * \file pvt_solution.cc
 * \brief Implementation of a base class for a PVT solution
 * \author Carles Fernandez-Prades, 2015. cfernandez(at)cttc.es
 *
 *
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

#include "pvt_solution.h"
#include "GPS_L1_CA.h"
#include "geofunctions.h"
#include <glog/logging.h>
#include <exception>


using google::LogMessage;


Pvt_Solution::Pvt_Solution()
{
    d_latitude_d = 0.0;
    d_longitude_d = 0.0;
    d_height_m = 0.0;
    d_speed_over_ground_m_s = 0.0;
    d_course_over_ground_d = 0.0;
    d_avg_latitude_d = 0.0;
    d_avg_longitude_d = 0.0;
    d_avg_height_m = 0.0;
    d_flag_averaging = false;
    b_valid_position = false;
    d_averaging_depth = 0;
    d_valid_observations = 0;
    d_rx_pos = arma::zeros(3, 1);
    d_rx_dt_s = 0.0;
}

arma::vec Pvt_Solution::rotateSatellite(double const traveltime, const arma::vec &X_sat)
{
    /*
     *  Returns rotated satellite ECEF coordinates due to Earth
     * rotation during signal travel time
     *
     *   Inputs:
     *       travelTime  - signal travel time
     *       X_sat       - satellite's ECEF coordinates
     *
     *   Returns:
     *       X_sat_rot   - rotated satellite's coordinates (ECEF)
     */

    //--- Find rotation angle --------------------------------------------------
    double omegatau;
    omegatau = OMEGA_EARTH_DOT * traveltime;

    //--- Build a rotation matrix ----------------------------------------------
    arma::mat R3 = {{cos(omegatau), sin(omegatau), 0.0},
        {-sin(omegatau), cos(omegatau), 0.0},
        {0.0, 0.0, 1.0}};

    //--- Do the rotation ------------------------------------------------------
    arma::vec X_sat_rot;
    X_sat_rot = R3 * X_sat;
    return X_sat_rot;
}


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

    const double a[5] = {6378388.0, 6378160.0, 6378135.0, 6378137.0, 6378137.0};
    const double f[5] = {1.0 / 297.0, 1.0 / 298.247, 1.0 / 298.26, 1.0 / 298.257222101, 1.0 / 298.257223563};

    double lambda = atan2(Y, X);
    double ex2 = (2.0 - f[elipsoid_selection]) * f[elipsoid_selection] / ((1.0 - f[elipsoid_selection]) * (1.0 - f[elipsoid_selection]));
    double c = a[elipsoid_selection] * sqrt(1.0 + ex2);
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
    d_latitude_d = phi * 180.0 / GPS_PI;
    d_longitude_d = lambda * 180.0 / GPS_PI;
    d_height_m = h;
    //todo: refactor this class. Mix of duplicated functions, use either RTKLIB geodetic functions or geofunctions.h
    return 0;
}


int Pvt_Solution::tropo(double *ddr_m, double sinel, double hsta_km, double p_mb, double t_kel, double hum, double hp_km, double htkel_km, double hhum_km)
{
    /*   Inputs:
           sinel     - sin of elevation angle of satellite
           hsta_km   - height of station in km
           p_mb      - atmospheric pressure in mb at height hp_km
           t_kel     - surface temperature in degrees Kelvin at height htkel_km
           hum       - humidity in % at height hhum_km
           hp_km     - height of pressure measurement in km
           htkel_km  - height of temperature measurement in km
           hhum_km   - height of humidity measurement in km

       Outputs:
           ddr_m     - range correction (meters)

     Reference
     Goad, C.C. & Goodman, L. (1974) A Modified Hopfield Tropospheric
     Refraction Correction Model. Paper presented at the
     American Geophysical Union Annual Fall Meeting, San
     Francisco, December 12-17

     Translated to C++ by Carles Fernandez from a Matlab implementation by Kai Borre
     */

    const double a_e = 6378.137;  // semi-major axis of earth ellipsoid
    const double b0 = 7.839257e-5;
    const double tlapse = -6.5;
    const double em = -978.77 / (2.8704e6 * tlapse * 1.0e-5);

    double tkhum = t_kel + tlapse * (hhum_km - htkel_km);
    double atkel = 7.5 * (tkhum - 273.15) / (237.3 + tkhum - 273.15);
    double e0 = 0.0611 * hum * pow(10, atkel);
    double tksea = t_kel - tlapse * htkel_km;
    double tkelh = tksea + tlapse * hhum_km;
    double e0sea = e0 * pow((tksea / tkelh), (4 * em));
    double tkelp = tksea + tlapse * hp_km;
    double psea = p_mb * pow((tksea / tkelp), em);

    if (sinel < 0)
        {
            sinel = 0.0;
        }

    double tropo_delay = 0.0;
    bool done = false;
    double refsea = 77.624e-6 / tksea;
    double htop = 1.1385e-5 / refsea;
    refsea = refsea * psea;
    double ref = refsea * pow(((htop - hsta_km) / htop), 4);

    double a;
    double b;
    double rtop;

    while (1)
        {
            rtop = pow((a_e + htop), 2) - pow((a_e + hsta_km), 2) * (1 - pow(sinel, 2));

            // check to see if geometry is crazy
            if (rtop < 0)
                {
                    rtop = 0;
                }

            rtop = sqrt(rtop) - (a_e + hsta_km) * sinel;

            a = -sinel / (htop - hsta_km);
            b = -b0 * (1 - pow(sinel, 2)) / (htop - hsta_km);

            arma::vec rn = arma::vec(8);
            rn.zeros();

            for (int i = 0; i < 8; i++)
                {
                    rn(i) = pow(rtop, (i + 1 + 1));
                }

            arma::rowvec alpha = {2 * a, 2 * pow(a, 2) + 4 * b / 3, a * (pow(a, 2) + 3 * b),
                pow(a, 4) / 5 + 2.4 * pow(a, 2) * b + 1.2 * pow(b, 2), 2 * a * b * (pow(a, 2) + 3 * b) / 3,
                pow(b, 2) * (6 * pow(a, 2) + 4 * b) * 1.428571e-1, 0, 0};

            if (pow(b, 2) > 1.0e-35)
                {
                    alpha(6) = a * pow(b, 3) / 2;
                    alpha(7) = pow(b, 4) / 9;
                }

            double dr = rtop;
            arma::mat aux_ = alpha * rn;
            dr = dr + aux_(0, 0);
            tropo_delay = tropo_delay + dr * ref * 1000;

            if (done == true)
                {
                    *ddr_m = tropo_delay;
                    break;
                }

            done = true;
            refsea = (371900.0e-6 / tksea - 12.92e-6) / tksea;
            htop = 1.1385e-5 * (1255 / tksea + 0.05) / refsea;
            ref = refsea * e0sea * pow(((htop - hsta_km) / htop), 4);
        }
    return 0;
}


void Pvt_Solution::set_averaging_depth(int depth)
{
    d_averaging_depth = depth;
}


void Pvt_Solution::set_averaging_flag(bool flag)
{
    d_flag_averaging = flag;
}


void Pvt_Solution::perform_pos_averaging()
{
    // MOVING AVERAGE PVT
    bool avg = d_flag_averaging;
    if (avg == true)
        {
            if (d_hist_longitude_d.size() == static_cast<unsigned int>(d_averaging_depth))
                {
                    // Pop oldest value
                    d_hist_longitude_d.pop_back();
                    d_hist_latitude_d.pop_back();
                    d_hist_height_m.pop_back();
                    // Push new values
                    d_hist_longitude_d.push_front(d_longitude_d);
                    d_hist_latitude_d.push_front(d_latitude_d);
                    d_hist_height_m.push_front(d_height_m);

                    d_avg_latitude_d = 0.0;
                    d_avg_longitude_d = 0.0;
                    d_avg_height_m = 0.0;
                    for (unsigned int i = 0; i < d_hist_longitude_d.size(); i++)
                        {
                            d_avg_latitude_d = d_avg_latitude_d + d_hist_latitude_d.at(i);
                            d_avg_longitude_d = d_avg_longitude_d + d_hist_longitude_d.at(i);
                            d_avg_height_m = d_avg_height_m + d_hist_height_m.at(i);
                        }
                    d_avg_latitude_d = d_avg_latitude_d / static_cast<double>(d_averaging_depth);
                    d_avg_longitude_d = d_avg_longitude_d / static_cast<double>(d_averaging_depth);
                    d_avg_height_m = d_avg_height_m / static_cast<double>(d_averaging_depth);
                    b_valid_position = true;
                }
            else
                {
                    //int current_depth=d_hist_longitude_d.size();
                    // Push new values
                    d_hist_longitude_d.push_front(d_longitude_d);
                    d_hist_latitude_d.push_front(d_latitude_d);
                    d_hist_height_m.push_front(d_height_m);

                    d_avg_latitude_d = d_latitude_d;
                    d_avg_longitude_d = d_longitude_d;
                    d_avg_height_m = d_height_m;
                    b_valid_position = false;
                }
        }
    else
        {
            b_valid_position = true;
        }
}


double Pvt_Solution::get_time_offset_s() const
{
    return d_rx_dt_s;
}


void Pvt_Solution::set_time_offset_s(double offset)
{
    d_rx_dt_s = offset;
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


double Pvt_Solution::get_avg_latitude() const
{
    return d_avg_latitude_d;
}


double Pvt_Solution::get_avg_longitude() const
{
    return d_avg_longitude_d;
}


double Pvt_Solution::get_avg_height() const
{
    return d_avg_height_m;
}


bool Pvt_Solution::is_averaging() const
{
    return d_flag_averaging;
}


bool Pvt_Solution::is_valid_position() const
{
    return b_valid_position;
}


void Pvt_Solution::set_valid_position(bool is_valid)
{
    b_valid_position = is_valid;
}


void Pvt_Solution::set_rx_pos(const arma::vec &pos)
{
    d_rx_pos = pos;
    d_latitude_d = d_rx_pos(0);
    d_longitude_d = d_rx_pos(1);
    d_height_m = d_rx_pos(2);
}


arma::vec Pvt_Solution::get_rx_pos() const
{
    return d_rx_pos;
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
