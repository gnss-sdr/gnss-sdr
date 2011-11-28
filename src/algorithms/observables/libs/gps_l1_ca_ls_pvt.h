/*!
 * \file gps_l1_ca_ls_pvt.h
 * \brief Least Squares Position, Velocity, and Time (PVT) solver, based on
 * K.Borre Matlab receiver.
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
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
#ifndef GPS_L1_CA_LS_PVT_H_
#define GPS_L1_CA_LS_PVT_H_

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

//#include <itpp/itbase.h>
//#include <itpp/stat/misc_stat.h>
//#include <itpp/base/matfunc.h>

#include "armadillo"

//using namespace arma;

//using namespace itpp;

class gps_l1_ca_ls_pvt
{
private:
    arma::vec leastSquarePos(arma::mat satpos, arma::vec obs, arma::mat w);
    arma::vec e_r_corr(double traveltime, arma::vec X_sat);
    //void cart2geo();
    //void topocent();
public:
    int d_nchannels;
    gps_navigation_message* d_ephemeris;
    double d_pseudoranges_time_ms;
    double d_latitude_d;
    double d_longitude_d;
    double d_height_m;
    double d_x_m;
    double d_y_m;
    double d_z_m;

    gps_l1_ca_ls_pvt(int nchannels);
    ~gps_l1_ca_ls_pvt();

    void get_PVT(std::map<int,float> pseudoranges,double GPS_current_time);
    void cart2geo(double X, double Y, double Z, int elipsoid_selection);
};

#endif
