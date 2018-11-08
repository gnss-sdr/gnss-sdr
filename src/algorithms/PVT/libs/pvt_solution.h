/*!
 * \file pvt_solution.h
 * \brief Interface of a base class for a PVT solution
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

#ifndef GNSS_SDR_PVT_SOLUTION_H_
#define GNSS_SDR_PVT_SOLUTION_H_


#include <boost/date_time/posix_time/posix_time.hpp>
#include <deque>
#include <armadillo>

const unsigned int PVT_MAX_CHANNELS = 90;
const unsigned int PVT_MAX_PRN = 127;  // 126 is SBAS

/*!
 * \brief Base class for a PVT solution
 *
 */
class Pvt_Solution
{
private:
    double d_rx_dt_s;  // RX time offset [s]

    double d_latitude_d;             // RX position Latitude WGS84 [deg]
    double d_longitude_d;            // RX position Longitude WGS84 [deg]
    double d_height_m;               // RX position height WGS84 [m]
    double d_speed_over_ground_m_s;  // RX speed over ground [m/s]
    double d_course_over_ground_d;   // RX course over ground [deg]

    double d_avg_latitude_d;   // Averaged latitude in degrees
    double d_avg_longitude_d;  // Averaged longitude in degrees
    double d_avg_height_m;     // Averaged height [m]

    bool b_valid_position;

    std::deque<double> d_hist_latitude_d;
    std::deque<double> d_hist_longitude_d;
    std::deque<double> d_hist_height_m;

    bool d_flag_averaging;
    int d_averaging_depth;  // Length of averaging window

    arma::vec d_rx_pos;
    boost::posix_time::ptime d_position_UTC_time;
    int d_valid_observations;

public:
    Pvt_Solution();

    double get_time_offset_s() const;       //!< Get RX time offset [s]
    void set_time_offset_s(double offset);  //!< Set RX time offset [s]

    double get_latitude() const;   //!< Get RX position Latitude WGS84 [deg]
    double get_longitude() const;  //!< Get RX position Longitude WGS84 [deg]
    double get_height() const;     //!< Get RX position height WGS84 [m]

    double get_speed_over_ground() const;          //!< Get RX speed over ground [m/s]
    void set_speed_over_ground(double speed_m_s);  //!< Set RX speed over ground [m/s]

    double get_course_over_ground() const;        //!< Get RX course over ground [deg]
    void set_course_over_ground(double cog_deg);  //!< Set RX course over ground [deg]

    double get_avg_latitude() const;   //!< Get RX position averaged Latitude WGS84 [deg]
    double get_avg_longitude() const;  //!< Get RX position averaged Longitude WGS84 [deg]
    double get_avg_height() const;     //!< Get RX position averaged height WGS84 [m]

    void set_rx_pos(const arma::vec &pos);  //!< Set position: Latitude [deg], longitude [deg], height [m]
    arma::vec get_rx_pos() const;

    bool is_valid_position() const;
    void set_valid_position(bool is_valid);

    boost::posix_time::ptime get_position_UTC_time() const;
    void set_position_UTC_time(const boost::posix_time::ptime &pt);

    int get_num_valid_observations() const;    //!< Get the number of valid pseudorange observations (valid satellites)
    void set_num_valid_observations(int num);  //!< Set the number of valid pseudorange observations (valid satellites)

    //averaging
    void perform_pos_averaging();
    void set_averaging_depth(int depth);  //!< Set length of averaging window
    bool is_averaging() const;
    void set_averaging_flag(bool flag);

    arma::vec rotateSatellite(double traveltime, const arma::vec &X_sat);

    /*!
      * \brief Conversion of Cartesian coordinates (X,Y,Z) to geographical
      * coordinates (d_latitude_d, d_longitude_d, d_height_m) on a selected reference ellipsoid.
      *
      * \param[in] X [m] Cartesian coordinate
      * \param[in] Y [m] Cartesian coordinate
      * \param[in] Z [m] Cartesian coordinate
      * \param[in] elipsoid_selection. Choices of Reference Ellipsoid for Geographical Coordinates:
      * 0 - International Ellipsoid 1924.
      * 1 - International Ellipsoid 1967.
      * 2 - World Geodetic System 1972.
      * 3 - Geodetic Reference System 1980.
      * 4 - World Geodetic System 1984.
      *
      */
    int cart_to_geo(double X, double Y, double Z, int elipsoid_selection);

    /*!
      * \brief Tropospheric correction
      *
      *  \param[in] sinel     - sin of elevation angle of satellite
      *  \param[in] hsta_km   - height of station in km
      *  \param[in] p_mb      - atmospheric pressure in mb at height hp_km
      *  \param[in] t_kel     - surface temperature in degrees Kelvin at height htkel_km
      *  \param[in] hum       - humidity in % at height hhum_km
      *  \param[in] hp_km     - height of pressure measurement in km
      *  \param[in] htkel_km  - height of temperature measurement in km
      *  \param[in] hhum_km   - height of humidity measurement in km
      *
      *  \param[out] ddr_m     - range correction (meters)
      *
      *
      * Reference:
      * Goad, C.C. & Goodman, L. (1974) A Modified Hopfield Tropospheric
      *   Refraction Correction Model. Paper presented at the
      *   American Geophysical Union Annual Fall Meeting, San
      *   Francisco, December 12-17
      *
      * Translated to C++ by Carles Fernandez from a Matlab implementation by Kai Borre
      */
    int tropo(double *ddr_m, double sinel, double hsta_km, double p_mb, double t_kel, double hum, double hp_km, double htkel_km, double hhum_km);
};

#endif
