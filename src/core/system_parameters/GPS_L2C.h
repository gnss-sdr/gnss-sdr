/*!
 * \file GPS_L2C.h
 * \brief  Defines system parameters for GPS L2C signal
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
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


#ifndef GNSS_SDR_GPS_L2C_H_
#define GNSS_SDR_GPS_L2C_H_

#include <cstdint>
#include <vector>
#include <utility> // std::pair
#include "MATH_CONSTANTS.h"

// Physical constants
const double GPS_L2_C_m_s       = 299792458.0;      //!< The speed of light, [m/s]
const double GPS_L2_C_m_ms      = 299792.4580;      //!< The speed of light, [m/ms]
const double GPS_L2_PI          = 3.1415926535898;  //!< Pi as defined in IS-GPS-200E
const double GPS_L2_TWO_PI      = 6.283185307179586;//!< 2Pi as defined in IS-GPS-200E
const double GPS_L2_OMEGA_EARTH_DOT = 7.2921151467e-5;  //!< Earth rotation rate, [rad/s]
const double GPS_L2_GM              = 3.986005e14;      //!< Universal gravitational constant times the mass of the Earth, [m^3/s^2]
const double GPS_L2_F               = -4.442807633e-10; //!< Constant, [s/(m)^(1/2)]


// carrier and code frequencies
const double GPS_L2_FREQ_HZ = 1.2276e9;          //!< L2 [Hz]

const double GPS_L2_M_CODE_RATE_HZ = 0.5115e6;   //!< GPS L2 M code rate [chips/s]
const int GPS_L2_M_CODE_LENGTH_CHIPS = 10230;    //!< GPS L2 M code length [chips]
const double GPS_L2_M_PERIOD = 0.02;             //!< GPS L2 M code period [seconds]

const double GPS_L2_L_CODE_RATE_HZ = 0.5115e6;   //!< GPS L2 L code rate [chips/s]
const int GPS_L2_L_CODE_LENGTH_CHIPS = 767250;   //!< GPS L2 L code length [chips]
const double GPS_L2_L_PERIOD = 1.5;              //!< GPS L2 L code period [seconds]

const int GPS_L2C_HISTORY_DEEP = 5;

const int32_t GPS_L2C_M_INIT_REG[115] =
        {0742417664, 0756014035,0002747144,0066265724, // 1:4
                0601403471, 0703232733, 0124510070, 0617316361, // 5:8
                0047541621, 0733031046, 0713512145, 0024437606,
                0021264003, 0230655351, 0001314400, 0222021506,
                0540264026, 0205521705, 0064022144, 0120161274,
                0044023533, 0724744327, 0045743577, 0741201660,
                0700274134, 0010247261, 0713433445, 0737324162,
                0311627434, 0710452007, 0722462133, 0050172213,
                0500653703, 0755077436, 0136717361, 0756675453,
                0435506112, 0771353753, 0226107701, 0022025110,
                0402466344, 0752566114, 0702011164, 0041216771,
                0047457275, 0266333164, 0713167356, 0060546335,
                0355173035, 0617201036, 0157465571, 0767360553,
                0023127030, 0431343777, 0747317317, 0045706125,
                0002744276, 0060036467, 0217744147, 0603340174,//57:60
                0326616775, 0063240065, 0111460621, //61:63
                0604055104, 0157065232, 0013305707, 0603552017,//159:162
                0230461355, 0603653437, 0652346475, 0743107103,
                0401521277, 0167335110, 0014013575, 0362051132,
                0617753265, 0216363634, 0755561123, 0365304033,
                0625025543, 0054420334,  0415473671,  0662364360,
                0373446602,  0417564100,  0000526452,  0226631300,
                0113752074,  0706134401,  0041352546,  0664630154,
                0276524255,  0714720530,  0714051771,  0044526647,
                0207164322,  0262120161,  0204244652,  0202133131,
                0714351204,  0657127260,  0130567507,  0670517677,
                0607275514,  0045413633,  0212645405,  0613700455,
                0706202440,  0705056276,  0020373522,  0746013617,
                0132720621,  0434015513,  0566721727,  0140633660};

// CNAV GPS NAVIGATION MESSAGE STRUCTURE
// NAVIGATION MESSAGE FIELDS POSITIONS (from IS-GPS-200E Appendix III)

#define GPS_CNAV_PREAMBLE {1, 0, 0, 0, 1, 0, 1, 1}

const int GPS_L2_CNAV_DATA_PAGE_BITS = 300; //!< GPS L2 CNAV page length, including preamble and CRC [bits]
const int GPS_L2_SYMBOLS_PER_BIT = 2;
const int GPS_L2_SAMPLES_PER_SYMBOL = 1;
const int GPS_L2_CNAV_DATA_PAGE_DURATION_S = 12;
const int GPS_L2_CNAV_DATA_PAGE_BITS_EXTENDED_BYTES = 304; //!< GPS L2 CNAV page length, including preamble and CRC [bits]

// common to all messages
const std::vector<std::pair<int,int> > CNAV_PRN( { {9,6} } );
const std::vector<std::pair<int,int> > CNAV_MSG_TYPE( { {15,6} } );
const std::vector<std::pair<int,int> > CNAV_TOW( { {21,17} } ); //GPS Time Of Week in seconds
const double CNAV_TOW_LSB = 6.0;
const std::vector<std::pair<int,int> > CNAV_ALERT_FLAG( { {38,1} } );

// MESSAGE TYPE 10 (Ephemeris 1)

const std::vector<std::pair<int,int> > CNAV_WN({{39,13}});
const std::vector<std::pair<int,int> > CNAV_HEALTH({{52,3}});
const std::vector<std::pair<int,int> > CNAV_TOP1({{55,11}});
const double CNAV_TOP1_LSB = 300.0;
const std::vector<std::pair<int,int> > CNAV_URA({{66,5}});

const std::vector<std::pair<int,int> > CNAV_TOE1({{71,11}});
const double CNAV_TOE1_LSB = 300.0;

const std::vector<std::pair<int,int> > CNAV_DELTA_A({{82,26}}); //Relative to AREF = 26,559,710 meters
const double CNAV_DELTA_A_LSB = TWO_N9;

const std::vector<std::pair<int,int> > CNAV_A_DOT({{108,25}});
const double CNAV_A_DOT_LSB = TWO_N21;

const std::vector<std::pair<int,int> > CNAV_DELTA_N0({{133,17}});
const double CNAV_DELTA_N0_LSB = TWO_N44;
const std::vector<std::pair<int,int> > CNAV_DELTA_N0_DOT({{150,23}});
const double CNAV_DELTA_N0_DOT_LSB = TWO_N57;
const std::vector<std::pair<int,int> > CNAV_M0({{173,33}});
const double CNAV_M0_LSB = TWO_N32;
const std::vector<std::pair<int,int> > CNAV_E_ECCENTRICITY({{206,33}});
const double CNAV_E_ECCENTRICITY_LSB = TWO_N34;
const std::vector<std::pair<int,int> > CNAV_OMEGA({{239,33}});
const double CNAV_OMEGA_LSB = TWO_N32;
const std::vector<std::pair<int,int> > CNAV_INTEGRITY_FLAG({{272,1}});
const std::vector<std::pair<int,int> > CNAV_L2_PHASING_FLAG({{273,1}});

// MESSAGE TYPE 11 (Ephemeris 2)

const std::vector<std::pair<int,int> > CNAV_TOE2({{39,11}});
const double CNAV_TOE2_LSB = 300.0;
const std::vector<std::pair<int,int> > CNAV_OMEGA0({{50,33}});
const double CNAV_OMEGA0_LSB = TWO_N32;
const std::vector<std::pair<int,int> > CNAV_I0({{83,33}});
const double CNAV_I0_LSB = TWO_N32;
const std::vector<std::pair<int,int> > CNAV_DELTA_OMEGA_DOT({{116,17}}); //Relative to REF = -2.6 x 10-9 semi-circles/second.
const double CNAV_DELTA_OMEGA_DOT_LSB = TWO_N44;
const std::vector<std::pair<int,int> > CNAV_I0_DOT({{133,15}});
const double CNAV_I0_DOT_LSB = TWO_N44;
const std::vector<std::pair<int,int> > CNAV_CIS({{148,16}});
const double CNAV_CIS_LSB = TWO_N30;
const std::vector<std::pair<int,int> > CNAV_CIC({{164,16}});
const double CNAV_CIC_LSB = TWO_N30;
const std::vector<std::pair<int,int> > CNAV_CRS({{180,24}});
const double CNAV_CRS_LSB = TWO_N8;
const std::vector<std::pair<int,int> > CNAV_CRC({{204,24}});
const double CNAV_CRC_LSB = TWO_N8;
const std::vector<std::pair<int,int> > CNAV_CUS({{228,21}});
const double CNAV_CUS_LSB = TWO_N30;
const std::vector<std::pair<int,int> > CNAV_CUC({{249,21}});
const double CNAV_CUC_LSB = TWO_N30;


// MESSAGE TYPE 30 (CLOCK, IONO, GRUP DELAY)

const std::vector<std::pair<int,int> > CNAV_TOP2({{39,11}});
const double CNAV_TOP2_LSB = 300.0;

const std::vector<std::pair<int,int> > CNAV_URA_NED0({{50,5}});
const std::vector<std::pair<int,int> > CNAV_URA_NED1({{55,3}});
const std::vector<std::pair<int,int> > CNAV_URA_NED2({{58,3}});
const std::vector<std::pair<int,int> > CNAV_TOC({{61,11}});
const double CNAV_TOC_LSB = 300.0;
const std::vector<std::pair<int,int> > CNAV_AF0({{72,26}});
const double CNAV_AF0_LSB = TWO_N60;
const std::vector<std::pair<int,int> > CNAV_AF1({{98,20}});
const double CNAV_AF1_LSB = TWO_N48;
const std::vector<std::pair<int,int> > CNAV_AF2({{118,10}});
const double CNAV_AF2_LSB = TWO_N35;
const std::vector<std::pair<int,int> > CNAV_TGD({{128,13}});
const double CNAV_TGD_LSB = TWO_N35;
const std::vector<std::pair<int,int> > CNAV_ISCL1({{141,13}});
const double CNAV_ISCL1_LSB = TWO_N35;
const std::vector<std::pair<int,int> > CNAV_ISCL2({{154,13}});
const double CNAV_ISCL2_LSB = TWO_N35;
const std::vector<std::pair<int,int> > CNAV_ISCL5I({{167,13}});
const double CNAV_ISCL5I_LSB = TWO_N35;
const std::vector<std::pair<int,int> > CNAV_ISCL5Q({{180,13}});
const double CNAV_ISCL5Q_LSB = TWO_N35;
//Ionospheric parameters
const std::vector<std::pair<int,int> > CNAV_ALPHA0({{193,8}});
const double CNAV_ALPHA0_LSB = TWO_N30;
const std::vector<std::pair<int,int> > CNAV_ALPHA1({{201,8}});
const double CNAV_ALPHA1_LSB = TWO_N27;
const std::vector<std::pair<int,int> > CNAV_ALPHA2({{209,8}});
const double CNAV_ALPHA2_LSB = TWO_N24;
const std::vector<std::pair<int,int> > CNAV_ALPHA3({{217,8}});
const double CNAV_ALPHA3_LSB = TWO_N24;
const std::vector<std::pair<int,int> > CNAV_BETA0({{225,8}});
const double CNAV_BETA0_LSB = TWO_P11;
const std::vector<std::pair<int,int> > CNAV_BETA1({{233,8}});
const double CNAV_BETA1_LSB = TWO_P14;
const std::vector<std::pair<int,int> > CNAV_BETA2({{241,8}});
const double CNAV_BETA2_LSB = TWO_P16;
const std::vector<std::pair<int,int> > CNAV_BETA3({{249,8}});
const double CNAV_BETA3_LSB = TWO_P16;
const std::vector<std::pair<int,int> > CNAV_WNOP({{257,8}});


// TODO: Add more frames (Almanac, etc...)



#endif /* GNSS_SDR_GPS_L2C_H_ */
