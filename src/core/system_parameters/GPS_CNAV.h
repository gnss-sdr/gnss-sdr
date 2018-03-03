/*!
 * \file GPS_CNAV.h
 * \brief  Defines parameters for GPS CNAV
 * \author Antonio Ramos, 2017. antonio.ramos(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
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


#ifndef GNSS_SDR_GPS_CNAV_H_
#define GNSS_SDR_GPS_CNAV_H_

#include "MATH_CONSTANTS.h"
#include <cstdint>
#include <vector>
#include <utility>  // std::pair


// CNAV GPS NAVIGATION MESSAGE STRUCTURE
// NAVIGATION MESSAGE FIELDS POSITIONS (from IS-GPS-200E Appendix III)

#define GPS_CNAV_PREAMBLE      \
    {                          \
        1, 0, 0, 0, 1, 0, 1, 1 \
    }
#define GPS_CNAV_PREAMBLE_STR "10001011"
#define GPS_CNAV_INV_PREAMBLE_STR "01110100"

const int GPS_CNAV_DATA_PAGE_BITS = 300;

// common to all messages
const std::vector<std::pair<int, int> > CNAV_PRN({{9, 6}});
const std::vector<std::pair<int, int> > CNAV_MSG_TYPE({{15, 6}});
const std::vector<std::pair<int, int> > CNAV_TOW({{21, 17}});  //GPS Time Of Week in seconds
const double CNAV_TOW_LSB = 6.0;
const std::vector<std::pair<int, int> > CNAV_ALERT_FLAG({{38, 1}});

// MESSAGE TYPE 10 (Ephemeris 1)

const std::vector<std::pair<int, int> > CNAV_WN({{39, 13}});
const std::vector<std::pair<int, int> > CNAV_HEALTH({{52, 3}});
const std::vector<std::pair<int, int> > CNAV_TOP1({{55, 11}});
const double CNAV_TOP1_LSB = 300.0;
const std::vector<std::pair<int, int> > CNAV_URA({{66, 5}});

const std::vector<std::pair<int, int> > CNAV_TOE1({{71, 11}});
const double CNAV_TOE1_LSB = 300.0;

const std::vector<std::pair<int, int> > CNAV_DELTA_A({{82, 26}});  //Relative to AREF = 26,559,710 meters
const double CNAV_DELTA_A_LSB = TWO_N9;

const std::vector<std::pair<int, int> > CNAV_A_DOT({{108, 25}});
const double CNAV_A_DOT_LSB = TWO_N21;

const std::vector<std::pair<int, int> > CNAV_DELTA_N0({{133, 17}});
const double CNAV_DELTA_N0_LSB = TWO_N44 * PI;  //semi-circles to radians
const std::vector<std::pair<int, int> > CNAV_DELTA_N0_DOT({{150, 23}});
const double CNAV_DELTA_N0_DOT_LSB = TWO_N57 * PI;  //semi-circles to radians
const std::vector<std::pair<int, int> > CNAV_M0({{173, 33}});
const double CNAV_M0_LSB = TWO_N32 * PI;  //semi-circles to radians
const std::vector<std::pair<int, int> > CNAV_E_ECCENTRICITY({{206, 33}});
const double CNAV_E_ECCENTRICITY_LSB = TWO_N34;
const std::vector<std::pair<int, int> > CNAV_OMEGA({{239, 33}});
const double CNAV_OMEGA_LSB = TWO_N32 * PI;  //semi-circles to radians
const std::vector<std::pair<int, int> > CNAV_INTEGRITY_FLAG({{272, 1}});
const std::vector<std::pair<int, int> > CNAV_L2_PHASING_FLAG({{273, 1}});

// MESSAGE TYPE 11 (Ephemeris 2)

const std::vector<std::pair<int, int> > CNAV_TOE2({{39, 11}});
const double CNAV_TOE2_LSB = 300.0;
const std::vector<std::pair<int, int> > CNAV_OMEGA0({{50, 33}});
const double CNAV_OMEGA0_LSB = TWO_N32 * PI;  //semi-circles to radians
const std::vector<std::pair<int, int> > CNAV_I0({{83, 33}});
const double CNAV_I0_LSB = TWO_N32 * PI;                                    //semi-circles to radians
const std::vector<std::pair<int, int> > CNAV_DELTA_OMEGA_DOT({{116, 17}});  //Relative to REF = -2.6 x 10-9 semi-circles/second.
const double CNAV_DELTA_OMEGA_DOT_LSB = TWO_N44 * PI;                       //semi-circles to radians
const std::vector<std::pair<int, int> > CNAV_I0_DOT({{133, 15}});
const double CNAV_I0_DOT_LSB = TWO_N44 * PI;  //semi-circles to radians
const std::vector<std::pair<int, int> > CNAV_CIS({{148, 16}});
const double CNAV_CIS_LSB = TWO_N30;
const std::vector<std::pair<int, int> > CNAV_CIC({{164, 16}});
const double CNAV_CIC_LSB = TWO_N30;
const std::vector<std::pair<int, int> > CNAV_CRS({{180, 24}});
const double CNAV_CRS_LSB = TWO_N8;
const std::vector<std::pair<int, int> > CNAV_CRC({{204, 24}});
const double CNAV_CRC_LSB = TWO_N8;
const std::vector<std::pair<int, int> > CNAV_CUS({{228, 21}});
const double CNAV_CUS_LSB = TWO_N30;
const std::vector<std::pair<int, int> > CNAV_CUC({{249, 21}});
const double CNAV_CUC_LSB = TWO_N30;


// MESSAGE TYPE 30 (CLOCK, IONO, GRUP DELAY)

const std::vector<std::pair<int, int> > CNAV_TOP2({{39, 11}});
const double CNAV_TOP2_LSB = 300.0;
const std::vector<std::pair<int, int> > CNAV_URA_NED0({{50, 5}});
const std::vector<std::pair<int, int> > CNAV_URA_NED1({{55, 3}});
const std::vector<std::pair<int, int> > CNAV_URA_NED2({{58, 3}});
const std::vector<std::pair<int, int> > CNAV_TOC({{61, 11}});
const double CNAV_TOC_LSB = 300.0;
const std::vector<std::pair<int, int> > CNAV_AF0({{72, 26}});
const double CNAV_AF0_LSB = TWO_N60;
const std::vector<std::pair<int, int> > CNAV_AF1({{98, 20}});
const double CNAV_AF1_LSB = TWO_N48;
const std::vector<std::pair<int, int> > CNAV_AF2({{118, 10}});
const double CNAV_AF2_LSB = TWO_N35;
const std::vector<std::pair<int, int> > CNAV_TGD({{128, 13}});
const double CNAV_TGD_LSB = TWO_N35;
const std::vector<std::pair<int, int> > CNAV_ISCL1({{141, 13}});
const double CNAV_ISCL1_LSB = TWO_N35;
const std::vector<std::pair<int, int> > CNAV_ISCL2({{154, 13}});
const double CNAV_ISCL2_LSB = TWO_N35;
const std::vector<std::pair<int, int> > CNAV_ISCL5I({{167, 13}});
const double CNAV_ISCL5I_LSB = TWO_N35;
const std::vector<std::pair<int, int> > CNAV_ISCL5Q({{180, 13}});
const double CNAV_ISCL5Q_LSB = TWO_N35;
//Ionospheric parameters
const std::vector<std::pair<int, int> > CNAV_ALPHA0({{193, 8}});
const double CNAV_ALPHA0_LSB = TWO_N30;
const std::vector<std::pair<int, int> > CNAV_ALPHA1({{201, 8}});
const double CNAV_ALPHA1_LSB = TWO_N27;
const std::vector<std::pair<int, int> > CNAV_ALPHA2({{209, 8}});
const double CNAV_ALPHA2_LSB = TWO_N24;
const std::vector<std::pair<int, int> > CNAV_ALPHA3({{217, 8}});
const double CNAV_ALPHA3_LSB = TWO_N24;
const std::vector<std::pair<int, int> > CNAV_BETA0({{225, 8}});
const double CNAV_BETA0_LSB = TWO_P11;
const std::vector<std::pair<int, int> > CNAV_BETA1({{233, 8}});
const double CNAV_BETA1_LSB = TWO_P14;
const std::vector<std::pair<int, int> > CNAV_BETA2({{241, 8}});
const double CNAV_BETA2_LSB = TWO_P16;
const std::vector<std::pair<int, int> > CNAV_BETA3({{249, 8}});
const double CNAV_BETA3_LSB = TWO_P16;
const std::vector<std::pair<int, int> > CNAV_WNOP({{257, 8}});


// MESSAGE TYPE 33 (CLOCK and UTC)

const std::vector<std::pair<int, int> > CNAV_A0({{128, 16}});
const double CNAV_A0_LSB = TWO_N35;
const std::vector<std::pair<int, int> > CNAV_A1({{144, 13}});
const double CNAV_A1_LSB = TWO_N51;
const std::vector<std::pair<int, int> > CNAV_A2({{157, 7}});
const double CNAV_A2_LSB = TWO_N68;
const std::vector<std::pair<int, int> > CNAV_DELTA_TLS({{164, 8}});
const double CNAV_DELTA_TLS_LSB = 1;
const std::vector<std::pair<int, int> > CNAV_TOT({{172, 16}});
const double CNAV_TOT_LSB = TWO_P4;
const std::vector<std::pair<int, int> > CNAV_WN_OT({{188, 13}});
const double CNAV_WN_OT_LSB = 1;
const std::vector<std::pair<int, int> > CNAV_WN_LSF({{201, 13}});
const double CNAV_WN_LSF_LSB = 1;
const std::vector<std::pair<int, int> > CNAV_DN({{214, 4}});
const double CNAV_DN_LSB = 1;
const std::vector<std::pair<int, int> > CNAV_DELTA_TLSF({{218, 8}});
const double CNAV_DELTA_TLSF_LSB = 1;


// TODO: Add more frames (Almanac, etc...)


#endif /* GNSS_SDR_GPS_CNAV_H_ */
