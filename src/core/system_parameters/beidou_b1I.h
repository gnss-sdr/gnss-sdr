/*!
 * \file beidou_b1I.h
 * \brief  Defines system parameters for BeiDou B1I signal and D1 NAV data
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
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


#ifndef GNSS_SDR_BEIDOU_B1I_H_
#define GNSS_SDR_BEIDOU_B1I_H_

#include <vector>
#include <utility> // std::pair
#include "MATH_CONSTANTS.h"

// Physical constants
const double BEIDOU_C_m_s       = 299792458.0;      //!< The speed of light, [m/s]
const double BEIDOU_C_m_ms      = 299792.4580;      //!< The speed of light, [m/ms]
const double BEIDOU_PI          = 3.1415926535898;  //!< Pi 
const double BEIDOU_TWO_PI      = 6.283185307179586;//!< 2Pi 
const double BEIDOU_OMEGA_EARTH_DOT    = 7.2921150e-5;  //!< Earth rotation rate, [rad/s] as defined in CGCS2000
const double BEIDOU_GM                 = 3.986004418e14;      //!< Universal gravitational constant times the mass of the Earth, [m^3/s^2] as defined in CGCS2000
const double BEIDOU_F                  = -4.442807309e-10; //!< Constant, [s/(m)^(1/2)] F=-2(GM)^.5/C^2


// carrier and code frequencies
const double BEIDOU_B1I_FREQ_HZ              = 1.561098e9; //!< b1I [Hz]
const double BEIDOU_B1I_CODE_RATE_HZ         = 2.046e6;   //!< beidou b1I code rate [chips/s]
const double BEIDOU_B1I_CODE_LENGTH_CHIPS    = 2046.0;    //!< beidou b1I code length [chips]
const double BEIDOU_B1I_CODE_PERIOD          = 0.001;     //!< beidou b1I code period [seconds]
const unsigned int BEIDOU_B1I_CODE_PERIOD_MS = 1;    //!< GPS L1 C/A code period [ms]
const double BEIDOU_B1I_CHIP_PERIOD          = 4.8875e-07;     //!< beidou b1I chip period [seconds]

/*!
 * \brief Maximum Time-Of-Arrival (TOA) difference between satellites for a receiver operated on Earth surface is 20 ms
 *
 * According to the GPS orbit model described in [1] Pag. 32.
 * It should be taken into account to set the buffer size for the PRN start timestamp in the pseudoranges block.
 * [1] J. Bao-Yen Tsui, Fundamentals of Global Positioning System Receivers. A Software Approach, John Wiley & Sons,
 * Inc., Hoboken, NJ, 2nd edition, 2005.
 */
const double BEIDOU_MAX_TOA_DELAY_MS = 20; //******************

//#define NAVIGATION_SOLUTION_RATE_MS 1000 // this cannot go here
const double BEIDOU_STARTOFFSET_ms = 68.802; //**************[ms] Initial sign. travel time (this cannot go here)


// OBSERVABLE HISTORY DEEP FOR INTERPOLATION
const int BEIDOU_B1I_HISTORY_DEEP = 100; // ****************

// NAVIGATION MESSAGE DEMODULATION AND DECODING

#define BEIDOU_PREAMBLE {1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0}
const int BEIDOU_B1I_PREAMBLE_LENGTH_BITS = 11;
const int BEIDOU_B1I_PREAMBLE_LENGTH_SYMBOLS = 220;   // **************
const double BEIDOU_B1I_PREAMBLE_DURATION_S = 0.220;
const int BEIDOU_B1I_PREAMBLE_DURATION_MS = 220;
const int BEIDOU_B1I_TELEMETRY_RATE_BITS_SECOND = 50;   //!< D1 NAV message bit rate [bits/s]
const int BEIDOU_B1I_TELEMETRY_SYMBOLS_PER_BIT = 20;  // *************
const int BEIDOU_B1I_TELEMETRY_RATE_SYMBOLS_SECOND = BEIDOU_B1I_TELEMETRY_RATE_BITS_SECOND*BEIDOU_B1I_TELEMETRY_SYMBOLS_PER_BIT;   //************!< NAV message bit rate [symbols/s]
const int BEIDOU_WORD_LENGTH = 4;                      //**************!< CRC + BEIDOU WORD (-2 -1 0 ... 29) Bits = 4 bytes
const int BEIDOU_SUBFRAME_LENGTH = 40;                 //**************!< BEIDOU_WORD_LENGTH x 10 = 40 bytes
const int BEIDOU_SUBFRAME_BITS = 300;                  //!< Number of bits per subframe in the NAV message [bits]
const int BEIDOU_SUBFRAME_SECONDS = 6;                 //!< Subframe duration [seconds]
const int BEIDOU_SUBFRAME_MS = 6000;                 //!< Subframe duration [miliseconds]
const int BEIDOU_WORD_BITS = 30;                       //!< Number of bits per word in the NAV message [bits]
const int BEIDOU_B1I_NH_CODE_LENGTH = 20;
const int BEIDOU_B1I_NH_CODE[20] = {0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0};
const std::string BEIDOU_B1I_NH_CODE_STR = "00000100110101001110";


// BEIDOU D1 NAVIGATION MESSAGE STRUCTURE
// GENERAL
const std::vector<std::pair<int,int> > D1_PRE( { {1,11} } );
const std::vector<std::pair<int,int> > D1_FRAID( { {16,3} } );
const std::vector<std::pair<int,int> > D1_SOW( { {19,8},{31,12} } );
const std::vector<std::pair<int,int> > D1_PNUM( { {44,7} } );

// SUBFRAME 1
const std::vector<std::pair<int,int> > D1_SAT_H1( { {43,1} } );
const std::vector<std::pair<int,int> > D1_AODC( { {44,5} } );
const std::vector<std::pair<int,int> > D1_URAI( { {49,4} } );
const std::vector<std::pair<int,int> > D1_WN( { {61,13} } );
const std::vector<std::pair<int,int> > D1_TOC( { {74,9},{91,8} } );
const double D1_TOC_LSB = TWO_P3;
const std::vector<std::pair<int,int> > D1_TGD1( { {99,10} } );
const double D1_TGD1_LSB = 0.1;
const std::vector<std::pair<int,int> > D1_TGD2( { {121,6} } );
const double D1_TGD2_LSB = 0.1;
const std::vector<std::pair<int,int> > D1_ALPHA0( { {127,8} } );
const double D1_ALPHA0_LSB = TWO_N30;
const std::vector<std::pair<int,int> > D1_ALPHA1( { {135,8} } );
const double D1_ALPHA1_LSB = TWO_N27;
const std::vector<std::pair<int,int> > D1_ALPHA2( { {151,8} } );
const double D1_ALPHA2_LSB = TWO_N24;
const std::vector<std::pair<int,int> > D1_ALPHA3( { {159,8} } );
const double D1_ALPHA3_LSB = TWO_N24;
const std::vector<std::pair<int,int> > D1_BETA0( { {167,6}, {181,2} } );
const double D1_BETA0_LSB = TWO_P11;
const std::vector<std::pair<int,int> > D1_BETA1( { {183,8} } );
const double D1_BETA1_LSB = TWO_P14;
const std::vector<std::pair<int,int> > D1_BETA2( { {191,8} } );
const double D1_BETA2_LSB = TWO_P16;
const std::vector<std::pair<int,int> > D1_BETA3( { {199,4},{211,4} } );
const double D1_BETA3_LSB = TWO_P16;
const std::vector<std::pair<int,int> > D1_A2( { {215,11} } );
const double D1_A2_LSB = TWO_N66;
const std::vector<std::pair<int,int> > D1_A0( { {226,7},{241,17} } );
const double D1_A0_LSB = TWO_N33;
const std::vector<std::pair<int,int> > D1_A1( { {258,5},{271,17} } );
const double D1_A1_LSB = TWO_N50;
const std::vector<std::pair<int,int> > D1_AODE( { {288,5} } );

//SUBFRAME 2
const std::vector<std::pair<int,int> > D1_DELTA_N( { {43,10},{61,6} } );
const double D1_DELTA_N_LSB =  PI_TWO_N43;
const std::vector<std::pair<int,int> > D1_CUC( { {67,16},{91,2} } );
const double D1_CUC_LSB = TWO_N31;
const std::vector<std::pair<int,int> > D1_M0( { {93,20}, {121,12} } );
const double D1_M0_LSB =  PI_TWO_N31;
const std::vector<std::pair<int,int> > D1_E( { {133,10},{151,22} } );
const double D1_E_LSB = TWO_N33;
const std::vector<std::pair<int,int> > D1_CUS( { {181,18} } );
const double D1_CUS_LSB = TWO_N31;
const std::vector<std::pair<int,int> > D1_CRC( { {199,4},{211,14} } );
const double D1_CRC_LSB = TWO_N6;
const std::vector<std::pair<int,int> > D1_CRS( { {225,8},{241,10} } );
const double D1_CRS_LSB = TWO_N6;
const std::vector<std::pair<int,int> > D1_SQRT_A( { {251,12},{271,20} } );
const double D1_SQRT_A_LSB = TWO_N19;
const std::vector<std::pair<int,int> > D1_TOE( { {291,2} } );
const double D1_TOE_LSB = TWO_P3;

//SUBFRAME 3
const std::vector<std::pair<int,int> > D1_TOE_MSB2( { {43,10},{61,5} } );
const std::vector<std::pair<int,int> > D1_I0( { {66,17},{91,15} } );
const double D1_I0_LSB =  PI_TWO_N31;
const std::vector<std::pair<int,int> > D1_CIC( { {106,7},{121,11} } );
const double D1_CIC_LSB = TWO_N31;
const std::vector<std::pair<int,int> > D1_OMEGA_DOT( { {132,11},{151,13} } );
const double D1_OMEGA_DOT_LSB =  PI_TWO_N43;
const std::vector<std::pair<int,int> > D1_CIS( { {164,9},{181,9} } );
const double D1_CIS_LSB = TWO_N31;
const std::vector<std::pair<int,int> > D1_IDOT( { {190,13},{211,1} } );
const double D1_IDOT_LSB =  PI_TWO_N43;
const std::vector<std::pair<int,int> > D1_OMEGA0( { {212,21},{241,11} } );
const double D1_OMEGA0_LSB =  PI_TWO_N31;
const std::vector<std::pair<int,int> > D1_OMEGA( { {252,11},{271,21} } );
const double D1_OMEGA_LSB =  PI_TWO_N31;

//SUBFRAME 4 AND PAGES 1 THROUGH 6 IN SUBFRAME 5
const std::vector<std::pair<int,int> > D1_SQRT_A_ALMANAC( { {51,2},{61,22} } );
const double D1_SQRT_A_ALMANAC_LSB = TWO_N11;
const std::vector<std::pair<int,int> > D1_A1_ALMANAC( { {91,11} } );
const double D1_A1_ALMANAC_LSB = TWO_N38;
const std::vector<std::pair<int,int> > D1_A0_ALMANAC( { {102,11} } );
const double D1_A0_ALMANAC_LSB = TWO_N20;
const std::vector<std::pair<int,int> > D1_OMEGA0_ALMANAC( { {121,22},{151,2} } );
const double D1_OMEGA0_ALMANAC_LSB =  PI_TWO_N23;
const std::vector<std::pair<int,int> > D1_E_ALMANAC( { {153,17} } );
const double D1_E_ALMANAC_LSB = TWO_N21;
const std::vector<std::pair<int,int> > D1_DELTA_I( { {170,3},{181,13} } );
const double D1_DELTA_I_LSB =  PI_TWO_N19;
const std::vector<std::pair<int,int> > D1_TOA( { {194,8} } );
const double D1_TOA_LSB = TWO_P12;
const std::vector<std::pair<int,int> > D1_OMEGA_DOT_ALMANAC( { {202,1}, {211,16} } );
const double D1_OMEGA_DOT_ALMANAC_LSB =  PI_TWO_N38;
const std::vector<std::pair<int,int> > D1_OMEGA_ALMANAC( { {227,6},{241,18} } );
const double D1_OMEGA_ALMANAC_LSB =  PI_TWO_N23;
const std::vector<std::pair<int,int> > D1_M0_ALMANAC( { {259,4},{271,20} } );
const double D1_M0_ALMANAC_LSB =  PI_TWO_N23;

//SUBFRAME 5 PAGE 7
const std::vector<std::pair<int,int> > D1_HEA1( { {51,2},{61,7} } );
const std::vector<std::pair<int,int> > D1_HEA2( { {68,9} } );
const std::vector<std::pair<int,int> > D1_HEA3( { {77,6},{91,3} } );
const std::vector<std::pair<int,int> > D1_HEA4( { {94,9} } );
const std::vector<std::pair<int,int> > D1_HEA5( { {103,9} } );
const std::vector<std::pair<int,int> > D1_HEA6( { {112,1},{121,8} } );
const std::vector<std::pair<int,int> > D1_HEA7( { {129,9} } );
const std::vector<std::pair<int,int> > D1_HEA8( { {138,5},{151,4} } );
const std::vector<std::pair<int,int> > D1_HEA9( { {155,9} } );
const std::vector<std::pair<int,int> > D1_HEA10( { {164,9} } );
const std::vector<std::pair<int,int> > D1_HEA11( { {181,9} } );
const std::vector<std::pair<int,int> > D1_HEA12( { {190,9} } );
const std::vector<std::pair<int,int> > D1_HEA13( { {199,4},{211,5} } );
const std::vector<std::pair<int,int> > D1_HEA14( { {216,9} } );
const std::vector<std::pair<int,int> > D1_HEA15( { {225,8},{241,1} } );
const std::vector<std::pair<int,int> > D1_HEA16( { {242,9} } );
const std::vector<std::pair<int,int> > D1_HEA17( { {251,9} } );
const std::vector<std::pair<int,int> > D1_HEA18( { {260,3},{271,6} } );
const std::vector<std::pair<int,int> > D1_HEA19( { {277,9} } );



//SUBFRAME 5 PAGE 8
const std::vector<std::pair<int,int> > D1_HEA20( { {51,2},{61,7} } );
const std::vector<std::pair<int,int> > D1_HEA21( { {68,9} } );
const std::vector<std::pair<int,int> > D1_HEA22( { {77,6},{91,3} } );
const std::vector<std::pair<int,int> > D1_HEA23( { {94,9} } );
const std::vector<std::pair<int,int> > D1_HEA24( { {103,9} } );
const std::vector<std::pair<int,int> > D1_HEA25( { {112,1},{121,8} } );
const std::vector<std::pair<int,int> > D1_HEA26( { {129,9} } );
const std::vector<std::pair<int,int> > D1_HEA27( { {138,5},{151,4} } );
const std::vector<std::pair<int,int> > D1_HEA28( { {155,9} } );
const std::vector<std::pair<int,int> > D1_HEA29( { {164,9} } );
const std::vector<std::pair<int,int> > D1_HEA30( { {181,9} } );
const std::vector<std::pair<int,int> > D1_WNA( { {190,8} } );
const std::vector<std::pair<int,int> > D1_TOA2( { {198,5},{211,3} } );

//SUBFRAME 5 PAGE 9
const std::vector<std::pair<int,int> > D1_A0GPS( { {97,14} } );
const double D1_A0GPS_LSB =  0.1;
const std::vector<std::pair<int,int> > D1_A1GPS( { {111,2},{121,14} } );
const double D1_A1GPS_LSB =  0.1;
const std::vector<std::pair<int,int> > D1_A0GAL( { {135,8},{151,6} } );
const double D1_A0GAL_LSB =  0.1;
const std::vector<std::pair<int,int> > D1_A1GAL( { {157,16} } );
const double D1_A1GAL_LSB =  0.1;
const std::vector<std::pair<int,int> > D1_A0GLO( { {181,14} } );
const double D1_A0GLO_LSB =  0.1;
const std::vector<std::pair<int,int> > D1_A1GLO( { {195,8},{211,8} } );
const double D1_A1GLO_LSB =  0.1;

//SUBFRAME 5 PAGE 10
const std::vector<std::pair<int,int> > D1_DELTA_T_LS( { {51,2},{61,6} } );
const std::vector<std::pair<int,int> > D1_DELTA_T_LSF( { {67,8} } );
const std::vector<std::pair<int,int> > D1_WN_LSF( { {75,8} } );
const std::vector<std::pair<int,int> > D1_A0UTC( { {91,22},{121,10} } );
const double D1_A0UTC_LSB =  TWO_N30;
const std::vector<std::pair<int,int> > D1_A1UTC( { {131,12},{151,12} } );
const double D1_A1UTC_LSB =  TWO_N50;
const std::vector<std::pair<int,int> > D1_DN( { {163,8} } );

#endif /* GNSS_SDR_BEIDOU_B1I_H_ */
