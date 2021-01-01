/*!
 * \file pvt_solution.h
 * \brief Interface of a base class for a PVT solution
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

#ifndef GNSS_SDR_PVT_SOLUTION_H
#define GNSS_SDR_PVT_SOLUTION_H

#include <boost/date_time/posix_time/posix_time.hpp>
#include <array>
#include <deque>

/** \addtogroup PVT
 * \{ */
/** \addtogroup PVT_libs
 * \{ */


/*!
 * \brief Base class for a PVT solution
 *
 */
class Pvt_Solution
{
public:
    Pvt_Solution();
    virtual ~Pvt_Solution() = default;

    void set_rx_pos(const std::array<double, 3> &pos);  //!< Set position: X, Y, Z in Cartesian ECEF coordinates [m]
    void set_rx_vel(const std::array<double, 3> &vel);  //!< Set velocity: East [m/s], North [m/s], Up [m/s]
    void set_position_UTC_time(const boost::posix_time::ptime &pt);
    void set_time_offset_s(double offset);             //!< Set RX time offset [s]
    void set_clock_drift_ppm(double clock_drift_ppm);  //!< Set the Rx clock drift [ppm]
    void set_speed_over_ground(double speed_m_s);      //!< Set RX speed over ground [m/s]
    void set_course_over_ground(double cog_deg);       //!< Set RX course over ground [deg]
    void set_valid_position(bool is_valid);
    void set_num_valid_observations(int num);    //!< Set the number of valid pseudorange observations (valid satellites)
    void set_pre_2009_file(bool pre_2009_file);  //!< Flag for the week rollover computation in post processing mode for signals older than 2009
    // averaging
    void set_averaging_depth(int depth);  //!< Set length of averaging window
    void set_averaging_flag(bool flag);
    void perform_pos_averaging();

    std::array<double, 3> get_rx_pos() const;
    std::array<double, 3> get_rx_vel() const;
    boost::posix_time::ptime get_position_UTC_time() const;
    double get_latitude() const;             //!< Get RX position Latitude WGS84 [deg]
    double get_longitude() const;            //!< Get RX position Longitude WGS84 [deg]
    double get_height() const;               //!< Get RX position height WGS84 [m]
    double get_time_offset_s() const;        //!< Get RX time offset [s]
    double get_clock_drift_ppm() const;      //!< Get the Rx clock drift [ppm]
    double get_speed_over_ground() const;    //!< Get RX speed over ground [m/s]
    double get_course_over_ground() const;   //!< Get RX course over ground [deg]
    double get_avg_latitude() const;         //!< Get RX position averaged Latitude WGS84 [deg]
    double get_avg_longitude() const;        //!< Get RX position averaged Longitude WGS84 [deg]
    double get_avg_height() const;           //!< Get RX position averaged height WGS84 [m]
    int get_num_valid_observations() const;  //!< Get the number of valid pseudorange observations (valid satellites)
    bool is_pre_2009() const;
    bool is_valid_position() const;
    bool is_averaging() const;

    virtual double get_hdop() const = 0;
    virtual double get_vdop() const = 0;
    virtual double get_pdop() const = 0;
    virtual double get_gdop() const = 0;

private:
    /*
     * Conversion of Cartesian coordinates (X,Y,Z) to geographical
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

    std::array<double, 3> d_rx_pos{};
    std::array<double, 3> d_rx_vel{};
    boost::posix_time::ptime d_position_UTC_time;

    std::deque<double> d_hist_latitude_d;
    std::deque<double> d_hist_longitude_d;
    std::deque<double> d_hist_height_m;

    double d_latitude_d;             // RX position Latitude WGS84 [deg]
    double d_longitude_d;            // RX position Longitude WGS84 [deg]
    double d_height_m;               // RX position height WGS84 [m]
    double d_rx_dt_s;                // RX time offset [s]
    double d_rx_clock_drift_ppm;     // RX clock drift [ppm]
    double d_speed_over_ground_m_s;  // RX speed over ground [m/s]
    double d_course_over_ground_d;   // RX course over ground [deg]

    double d_avg_latitude_d;   // Averaged latitude in degrees
    double d_avg_longitude_d;  // Averaged longitude in degrees
    double d_avg_height_m;     // Averaged height [m]

    int d_averaging_depth;  // Length of averaging window
    int d_valid_observations;

    bool d_pre_2009_file;  // Flag to correct week rollover in post processing mode for signals older than 2009
    bool b_valid_position;
    bool d_flag_averaging;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_PVT_SOLUTION_H
