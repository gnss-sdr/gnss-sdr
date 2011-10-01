/*!
 * \file gps_navigation_message.h
 * \brief  This class implements GPS L1 C/A navigation message decoding
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
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

#ifndef GPS_NAVIGATION_MESSAGE_H
#define GPS_NAVIGATION_MESSAGE_H

#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <algorithm>
#include <bitset>
#include "boost/assign.hpp"
#include <math.h>

using namespace boost::assign;

#include "GPS_L1_CA.h"

/*!
 * \brief  This class implements GPS L1 C/A navigation message decoding
 * and computes satellite position
 *
 * It defines the specific GPS navigation (NAV) data structure,
 * as defined in the document Interface Specification IS-GPS-200 Revision E, Appendix II
 */
class gps_navigation_message
{

private:
  unsigned long int read_navigation_unsigned(std::bitset<GPS_SUBFRAME_BITS> bits, const bits_slice *slices, int num_of_slices);
  signed long int read_navigation_signed(std::bitset<GPS_SUBFRAME_BITS> bits, const bits_slice *slices, int num_of_slices);
  double check_t(double time);
public:
    //broadcast orbit 1
    double d_TOW;
    double d_IODE_SF2;
    double d_IODE_SF3;
    double d_Crs;
    double d_Delta_n;
    double d_M_0;
    //broadcast orbit 2
    double d_Cuc;
    double d_e_eccentricity;
    double d_Cus;
    double d_sqrt_A;
    //broadcast orbit 3
    double d_Toe;
    double d_Toc;
    double d_Cic;
    double d_OMEGA0;
    double d_Cis;
    //broadcast orbit 4
    double d_i_0;
    double d_Crc;
    double d_OMEGA;
    double d_OMEGA_DOT;
    //broadcast orbit 5
    double d_IDOT;
    double d_codes_on_L2;
    double d_GPS_week;
    double d_L2_P_data_flag;
    //broadcast orbit 6
    double d_SV_accuracy;
    double d_SV_health;
    double d_TGD;
    double d_IODC;
    //broadcast orbit 7
    double d_transmission_time;
    double d_fit_interval;
    double d_spare1;
    double d_spare2;

    double d_A_f0;
    double d_A_f1;
    double d_A_f2;


    // clock terms
    double d_master_clock; //GPS transmission time
    double d_satClkCorr; // GPS clock error
    double d_dtr; // relativistic clock correction term

    // satellite positions
    double d_satpos_X;
    double d_satpos_Y;
    double d_satpos_Z;

    // satellite identification info

    int d_channel_ID;
    int d_satellite_PRN;

    // public functions
    void reset();
    int subframe_decoder(char *subframe);
    void master_clock();
    void satpos();
    void relativistic_clock_correction();
    bool satellite_validation();
    gps_navigation_message();
};

#endif
