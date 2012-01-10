/*!
 * \file gps_l1_ca_ls_pvt.h
 * \brief Interface of a Least Squares Position, Velocity, and Time (PVT)
 * solver, based on K.Borre's Matlab receiver.
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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
#ifndef GNSS_SDR_GPS_L1_CA_LS_PVT_H_
#define GNSS_SDR_GPS_L1_CA_LS_PVT_H_

#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <map>
#include <algorithm>
#include "gps_navigation_message.h"
#include "GPS_L1_CA.h"
#include "armadillo"

/*!
 * \brief This class implements a simple PVT Least Squares solution
 */
class gps_l1_ca_ls_pvt
{
private:
    arma::vec leastSquarePos(arma::mat satpos, arma::vec obs, arma::mat w);
    arma::vec e_r_corr(double traveltime, arma::vec X_sat);
    //void topocent();
public:
    int d_nchannels;      //! Number of available channels for positioning
    gps_navigation_message* d_ephemeris;
    double d_pseudoranges_time_ms;
    double d_latitude_d;  //! Latitude in degrees
    double d_longitude_d; //! Longitude in degrees
    double d_height_m;    //! Height [m]
    //averaging
    std::deque<double> d_hist_latitude_d;
    std::deque<double> d_hist_longitude_d;
    std::deque<double> d_hist_height_m;
    int d_averaging_depth;    //! Length of averaging window

    double d_avg_latitude_d;  //! Averaged latitude in degrees
    double d_avg_longitude_d; //! Averaged longitude in degrees
    double d_avg_height_m;    //! Averaged height [m]

    double d_x_m;
    double d_y_m;
    double d_z_m;

    bool d_flag_dump_enabled;
    std::string d_dump_filename;
    std::ofstream d_dump_file;

    void set_averaging_depth(int depth);

    gps_l1_ca_ls_pvt(int nchannels,std::string dump_filename, bool flag_dump_to_file);
    ~gps_l1_ca_ls_pvt();

    bool get_PVT(std::map<int,gnss_pseudorange> pseudoranges,double GPS_current_time,bool flag_averaging);

    /*!
     * \brief Conversion of Cartesian coordinates (X,Y,Z) to geographical
     * coordinates (d_latitude_d, d_longitude_d, d_height_m) on a selected reference ellipsoid.
     *
     * \param[in] X [m] Cartesian coordinate
     * \param[in] Y [m] Cartesian coordinate
     * \param[in] Z [m] Cartesian coordinate
     * \param[in] elipsoid_selection Choices of Reference Ellipsoid for Geographical Coordinates:
     * 0 - International Ellipsoid 1924.
     * 1 - International Ellipsoid 1967.
     * 2 - World Geodetic System 1972.
     * 3 - Geodetic Reference System 1980.
     * 4 - World Geodetic System 1984.
     *
     */
    void cart2geo(double X, double Y, double Z, int elipsoid_selection);
};

#endif
