/**
 * Copyright notice
 */

/**
 * Author: Javier Arribas, 2011. jarribas(at)cttc.es
 *         Luis Esteve, 2011. luis(at)epsilon-formacion.com
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

#include <itpp/itbase.h>
#include <itpp/stat/misc_stat.h>
#include <itpp/base/matfunc.h>

using namespace itpp;

class gps_l1_ca_ls_pvt
{
private:
    vec leastSquarePos(mat satpos, vec obs, mat w);
    vec e_r_corr(double traveltime, vec X_sat);
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
