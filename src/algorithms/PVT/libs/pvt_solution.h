/*!
 * \file pvt_solution.h
 * \brief Interface of a base class for a PVT solution
 * \author Carles Fernandez-Prades, 2015. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
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

    double d_latitude_d;   // RX position Latitude WGS84 [deg]
    double d_longitude_d;  // RX position Longitude WGS84 [deg]
    double d_height_m;     // RX position height WGS84 [m]

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

    arma::mat d_Q;
    double d_GDOP;
    double d_PDOP;
    double d_HDOP;
    double d_VDOP;
    double d_TDOP;

    int d_visible_satellites_IDs[PVT_MAX_CHANNELS] = {};          // Array with the IDs of the valid satellites
    double d_visible_satellites_El[PVT_MAX_CHANNELS] = {};        // Array with the LOS Elevation of the valid satellites
    double d_visible_satellites_Az[PVT_MAX_CHANNELS] = {};        // Array with the LOS Azimuth of the valid satellites
    double d_visible_satellites_Distance[PVT_MAX_CHANNELS] = {};  // Array with the LOS Distance of the valid satellites
    double d_visible_satellites_CN0_dB[PVT_MAX_CHANNELS] = {};    // Array with the IDs of the valid satellites

public:
    Pvt_Solution();

    double get_time_offset_s() const;       //!< Get RX time offset [s]
    void set_time_offset_s(double offset);  //!< Set RX time offset [s]

    double get_latitude() const;   //!< Get RX position Latitude WGS84 [deg]
    double get_longitude() const;  //!< Get RX position Longitude WGS84 [deg]
    double get_height() const;     //!< Get RX position height WGS84 [m]

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

    bool set_visible_satellites_ID(size_t index, unsigned int prn);  //!< Set the ID of the visible satellite index channel
    unsigned int get_visible_satellites_ID(size_t index) const;      //!< Get the ID of the visible satellite index channel

    bool set_visible_satellites_El(size_t index, double el);  //!< Set the LOS Elevation, in degrees, of the visible satellite index channel
    double get_visible_satellites_El(size_t index) const;     //!< Get the LOS Elevation, in degrees, of the visible satellite index channel

    bool set_visible_satellites_Az(size_t index, double az);  //!< Set the LOS Azimuth, in degrees, of the visible satellite index channel
    double get_visible_satellites_Az(size_t index) const;     //!< Get the LOS Azimuth, in degrees, of the visible satellite index channel

    bool set_visible_satellites_Distance(size_t index, double dist);  //!< Set the LOS Distance of the visible satellite index channel
    double get_visible_satellites_Distance(size_t index) const;       //!< Get the LOS Distance of the visible satellite index channel

    bool set_visible_satellites_CN0_dB(size_t index, double cn0);  //!< Set the CN0 in dB of the visible satellite index channel
    double get_visible_satellites_CN0_dB(size_t index) const;      //!< Get the CN0 in dB of the visible satellite index channel

    //averaging
    void perform_pos_averaging();
    void set_averaging_depth(int depth);  //!< Set length of averaging window
    bool is_averaging() const;
    void set_averaging_flag(bool flag);

    // DOP estimations
    void set_Q(const arma::mat &Q);
    int compute_DOP();  //!< Compute Dilution Of Precision parameters

    double get_GDOP() const;
    double get_PDOP() const;
    double get_HDOP() const;
    double get_VDOP() const;
    double get_TDOP() const;

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
    int cart2geo(double X, double Y, double Z, int elipsoid_selection);

    /*!
      * \brief Transformation of vector dx into topocentric coordinate system with origin at x
      *
      * \param[in] x    Vector origin coordinates (in ECEF system [X; Y; Z;])
      * \param[in] dx   Vector ([dX; dY; dZ;]).
      *
      * \param[out] D   Vector length. Units like the input
      * \param[out] Az  Azimuth from north positive clockwise, degrees
      * \param[out] El  Elevation angle, degrees
      *
      * Based on a Matlab function by Kai Borre
      */
    int topocent(double *Az, double *El, double *D, const arma::vec &x, const arma::vec &dx);

    /*!
      * \brief Subroutine to calculate geodetic coordinates latitude, longitude,
      * height given Cartesian coordinates X,Y,Z, and reference ellipsoid
      * values semi-major axis (a) and the inverse of flattening (finv).
      *
      *  The output units of angular quantities will be in decimal degrees
      *  (15.5 degrees not 15 deg 30 min). The output units of h will be the
      *  same as the units of X,Y,Z,a.
      *
      *  \param[in] a           - semi-major axis of the reference ellipsoid
      *  \param[in] finv        - inverse of flattening of the reference ellipsoid
      *  \param[in] X,Y,Z       - Cartesian coordinates
      *
      *  \param[out] dphi        - latitude
      *  \param[out] dlambda     - longitude
      *  \param[out] h           - height above reference ellipsoid
      *
      * Based in a Matlab function by Kai Borre
      */
    int togeod(double *dphi, double *dlambda, double *h, double a, double finv, double X, double Y, double Z);

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
