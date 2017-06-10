/*!
 * \file BEIDOU_B1I.h
 * \brief  Defines system parameters for BeiDou B1 signal and NAV data
 * \author Giorgio Savastano, 2015. giorgio.savastano(at)uniroma1.it
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

#include <complex>
#include <vector>
#include <utility> // std::pair
#include <gnss_satellite.h>
#include "MATH_CONSTANTS.h"

#define BEIDOU_DEBUG 0

// Physical constants
const double BEIDOU_C_m_s       = 299792458.0;      //!< The speed of light, [m/s]
const double BEIDOU_C_m_ms      = 299792.4580;      //!< The speed of light, [m/ms]
const double BEIDOU_PI          = 3.1415926535898;  //!< Pi as defined in ICD-BEIDOU
const double BEIDOU_TWO_PI      = 6.283185307179586;//!< 2Pi

// carrier and code frequencies
const double BEIDOU_B1I_FREQ_HZ              = 1.561098e9;  //!< B1I [Hz]
const double BEIDOU_B1I_CODE_RATE_HZ         = 2.046e6;     //!< BeiDou B1 C/A code rate [chips/s]
const double BEIDOU_B1I_CODE_LENGTH_CHIPS    = 2046.0;      //!< BeiDou B1 C/A code length [chips]
const double BEIDOU_B1I_CODE_PERIOD          = 0.001;       //!< BeiDou B1 C/A code period [seconds]
//const double BEIDOU_B1I_SECONDARY_CODE_LENGTH =;


// NH code
const double NH_BITS_RATE  			 = 1000;                                     // NH code bit rate [bits/s]
const double NH_BIT_DURATION 		 = 0.001;                                    // NH code bit duration [s] --> 1 ms
const int NH_length       			 = 20;
const signed int NH_CODE[NH_length]  = {1, 1, 1, 1, 1, -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, 1, -1, -1, -1,  1};

// D1 NAV Message

#define BEIDOU_PREAMBLE {1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0}  //!< Preamble as defined in ICD-BEIDOU (modified Barker code of 11 bits)

const int  BEIDOU_D1_NAV_BITS_RATE  = 50;               // D1 NAV message bit rate        [bits/s]
const int  BEIDOU_B1I_PREAMBLE_LENGTH_BITS           = 11;                                           //!< [bits]
const int  BEIDOU_B1I_TELEMETRY_RATE_BITS_SECOND     = 50;                                           //!< NAV message bit rate [bits/s]
const int  BEIDOU_B1I_TELEMETRY_SYMBOLS_PER_BIT      = 20;
const int  BEIDOU_B1I_PREAMBLE_LENGTH_SYMBOLS        = BEIDOU_B1I_PREAMBLE_LENGTH_BITS * BEIDOU_B1I_TELEMETRY_SYMBOLS_PER_BIT;
const int  BEIDOU_B1I_TELEMETRY_RATE_SYMBOLS_SECOND  = BEIDOU_B1I_TELEMETRY_RATE_BITS_SECOND * BEIDOU_B1I_TELEMETRY_SYMBOLS_PER_BIT;     //!< NAV message bit rate [symbols/s]

const int  BEIDOU_SUPERFRAME_BITS   = 36000;            // SUPERFRAME  (24 frames)        [bits]
const int  BEIDOU_SUPERFRAME_SECS   = 720;              // SUPERFRAME duration            [seconds]

const int  BEIDOU_FRAME_BITS        = 1500;             // FRAME       (5 subframes)      [bits]
const int  BEIDOU_FRAME_SECONDS     = 30;               // FRAME duration                 [seconds]

const int  BEIDOU_SUBFRAME_BITS     = 300;              // SUBFRAME    (10 words)         [bits]
const int  BEIDOU_SUBFRAME_SECONDS  = 6;                // SUBFRAME duration              [seconds]
const int  BEIDOU_SUBFRAME_MS       = BEIDOU_SUBFRAME_SECONDS * 1000;                // SUBFRAME duration [ms]

const int     BEIDOU_WORD_BITS      = 30;               // WORD                           [bits]
const double  BEIDOU_WORD_SECONDS   = 0.6;              // WORD duration                  [seconds]


///////////////////////////////////////////////////////////////////////////////////////////////////////////

/***************** BEIDOU NAVIGATION MESSAGE STRUCTURE D1 *****************/
// NAVIGATION MESSAGE FIELDS POSITIONS (from ICD VERSION 2.0)

// Fields for SUBFRAME N (Common for all of them)
// WORD 1    ( 1 - 30 bits) [26 info + 4 parity]
// WORD 2-10 (31 - 60 bits) [22 info + 8 parity]
const std::vector<std::pair<int,int>> P             ({{27 ,4},
                                                      {53 ,8},
                                                      {83 ,8},
                                                      {113,8},
                                                      {143,8},
                                                      {173,8},
                                                      {203,8},
                                                      {233,8},
                                                      {263,8},
                                                      {293,8}});
const std::vector<std::pair<int,int>> PRE           ({{1,11 }});     // PREAMBLE from modified Barker code of 11 bits
const std::vector<std::pair<int,int>> REV           ({{12,4 },
                                                      {292,1}});     // This pair is only for SUBFRAME 3
const std::vector<std::pair<int,int>> SUBFRAMEID    ({{16,3 }});     // 3 bits for subframe identification
const std::vector<std::pair<int,int>> SOW           ({{19,8 },       // first 8 bits for seconds of week (the number of seconds occurred since the last Sunday, 00:00:00 of BDT)
                                                      {31,12}});     // last 12 bits for seconds of week

/** SUBFRAME 1-5 **/
/** SUBFRAME 1 (In format D1) **/

// WORD 2 (31 - 60)
const std::vector<std::pair<int,int>> SATH1         ({{43,1}});     // satellite health flag ("0" means sat is good, "1" means not)
const std::vector<std::pair<int,int>> AODC          ({{44,5}});     // Age of Data, Clock
const std::vector<std::pair<int,int>> URAI          ({{49,4}});     // User Range Accuracy Index

// WORD 3 (61  - 90)
// WORD 4 (91  - 120)
// WORD 5 (121 - 130)
const std::vector<std::pair<int,int>> WN            ({{61,13}});     // Week Number (the integral week count of BDT with the range of 0 through 8191)
const std::vector<std::pair<int,int>> TOC           ({{74,9 },       // Reference time of clock parameters (in seconds with the effective range of 0-604792)
                                                      {91,8 }});
const std::vector<std::pair<int,int>> T_GD1         ({{99 ,10}});
const std::vector<std::pair<int,int>> T_GD2         ({{109,4 },
                                                      {121,6 }});
const std::vector<std::pair<int,int>> AL0           ({{127,8}});
const std::vector<std::pair<int,int>> AL1           ({{135,8}});


// WORD 6  (151 - 180)
// WORD 7  (181 - 210)
// WORD 8  (211 - 240)
// WORD 9  (241 - 270)
// WORD 10 (271 - 300)
const std::vector<std::pair<int,int>> AL2           ({{151,8}});
const std::vector<std::pair<int,int>> AL3           ({{159,8}});

const std::vector<std::pair<int,int>> BT0            ({{167,6},
                                                       {181,2}
                                                      });
const std::vector<std::pair<int,int>> BT1            ({{183,8}});
const std::vector<std::pair<int,int>> BT2            ({{191,8}});
const std::vector<std::pair<int,int>> BT3            ({{199,4},
                                                       {211,4}});

const std::vector<std::pair<int,int>> A2             ({{215,11}});
const std::vector<std::pair<int,int>> A0             ({{226,7 },
                                                       {241,17}});
const std::vector<std::pair<int,int>> A1             ({{258,5 },
                                                       {271,17}});
const std::vector<std::pair<int,int>> AODE           ({{288,5 }});



/** SUBFRAME 2 (In format D1) **/
const std::vector<std::pair<int,int>> A_N            ({{43,10},
                                                       {61, 6}});
const std::vector<std::pair<int,int>> CUC            ({{67,16},
                                                       {91, 2}});
const std::vector<std::pair<int,int>> M0             ({{93,20},
                                                       {121,12}});
const std::vector<std::pair<int,int>> ECC            ({{133,10},
                                                       {151,22}});
const std::vector<std::pair<int,int>> CUS            ({{181,18}});
const std::vector<std::pair<int,int>> CRC            ({{199,4 },
                                                       {211,14}});
const std::vector<std::pair<int,int>> CRS            ({{225,8 },
                                                       {241,10}});
const std::vector<std::pair<int,int>> SQRTA          ({{251,12},
                                                       {271,20}});
const std::vector<std::pair<int,int>> TOE            ({{291,2},
                                                       {43,10},     // These two last pairs are related with the SUBFRAME 3
                                                       {61, 5}});   //

/** SUBFRAME 3 (In format D1) **/
const std::vector<std::pair<int,int>> I_O            ({{66,17},
                                                       {91,15}});
const std::vector<std::pair<int,int>> C_INC           ({{106,7},
                                                       {121,11}});
const std::vector<std::pair<int,int>> OMG            ({{132,11},
                                                       {151,13}});
const std::vector<std::pair<int,int>> C_INS           ({{164,9},
                                                       {181,9}});
const std::vector<std::pair<int,int>> IDOT           ({{190,13},
                                                       {211,1}});
const std::vector<std::pair<int,int>> OMG_0          ({{212,21},
                                                       {241,11}});
const std::vector<std::pair<int,int>> PRGREE         ({{252,11},
                                                       {271,21}});







//const std::vector<std::pair<int,int>> L2_P_DATA_FLAG    ({{91 ,1 }});
//const std::vector<std::pair<int,int>> T_GD1             ({{197,8 }});
//
//const std::vector<std::pair<int,int>> IODC({{83,2},{211,8}});
//const std::vector<std::pair<int,int>> T_OC({{219,16}});
//const double T_OC_LSB = TWO_P4;
//const std::vector<std::pair<int,int>> A_F2({{241,8}});
//const double A_F2_LSB = TWO_N55;
//const std::vector<std::pair<int,int>> A_F1({{249,16}});
//const double A_F1_LSB = TWO_N43;
//const std::vector<std::pair<int,int>> A_F0({{271,22}});
//const double A_F0_LSB = TWO_N31;
//
//// SUBFRAME 2
//const std::vector<std::pair<int,int>> IODE_SF2({{61,8}});
//const std::vector<std::pair<int,int>> C_RS({{69,16}});
//const double C_RS_LSB = TWO_N5;
//const std::vector<std::pair<int,int>> DELTA_N({{91,16}});
//const double DELTA_N_LSB = PI_TWO_N43;
//const std::vector<std::pair<int,int>> M_0({{107,8},{121,24}});
//const double M_0_LSB = PI_TWO_N31;
//const std::vector<std::pair<int,int>> C_UC({{151,16}});
//const double C_UC_LSB = TWO_N29;
//const std::vector<std::pair<int,int>> E({{167,8},{181,24}});
//const double E_LSB = TWO_N33;
//const std::vector<std::pair<int,int>> C_US({{211,16}});
//const double C_US_LSB = TWO_N29;
//const std::vector<std::pair<int,int>> SQRT_A({{227,8},{241,24}});
//const double SQRT_A_LSB = TWO_N19;
//const std::vector<std::pair<int,int>> T_OE({{271,16}});
//const double T_OE_LSB = TWO_P4;
//const std::vector<std::pair<int,int>> FIT_INTERVAL_FLAG({{271,1}});
//const std::vector<std::pair<int,int>> AODO({{272,5}});
//const int AODO_LSB = 900;
//
//// SUBFRAME 3
//const std::vector<std::pair<int,int>> C_IC({{61,16}});
//const double C_IC_LSB = TWO_N29;
//const std::vector<std::pair<int,int>> OMEGA_0({{77,8},{91,24}});
//const double OMEGA_0_LSB = PI_TWO_N31;
//const std::vector<std::pair<int,int>> C_IS({{121,16}});
//const double C_IS_LSB = TWO_N29;
//const std::vector<std::pair<int,int>> I_0({{137,8},{151,24}});
//const double I_0_LSB = PI_TWO_N31;
//const std::vector<std::pair<int,int>> C_RC({{181,16}});
//const double C_RC_LSB = TWO_N5;
//const std::vector<std::pair<int,int>> OMEGA({{197,8},{211,24}});
//const double OMEGA_LSB = PI_TWO_N31;
//const std::vector<std::pair<int,int>> OMEGA_DOT({{241,24}});
//const double OMEGA_DOT_LSB = PI_TWO_N43;
//const std::vector<std::pair<int,int>> IODE_SF3({{271,8}});
//const std::vector<std::pair<int,int>> I_DOT({{279,14}});
//const double I_DOT_LSB = PI_TWO_N43;
//
//// SUBFRAME 4-5
//const std::vector<std::pair<int,int>> SV_DATA_ID({{61,2}});
//const std::vector<std::pair<int,int>> SV_PAGE({{63,6}});
//
//// SUBFRAME 4
////! \todo read all pages of subframe 4
//// Page 18 - Ionospheric and UTC data
//const std::vector<std::pair<int,int>> ALPHA_0({{69,8}});
//const double ALPHA_0_LSB = TWO_N30;
//const std::vector<std::pair<int,int>> ALPHA_1({{77,8}});
//const double ALPHA_1_LSB = TWO_N27;
//const std::vector<std::pair<int,int>> ALPHA_2({{91,8}});
//const double ALPHA_2_LSB = TWO_N24;
//const std::vector<std::pair<int,int>> ALPHA_3({{99,8}});
//const double ALPHA_3_LSB = TWO_N24;
//const std::vector<std::pair<int,int>> BETA_0({{107,8}});
//const double BETA_0_LSB = TWO_P11;
//const std::vector<std::pair<int,int>> BETA_1({{121,8}});
//const double BETA_1_LSB = TWO_P14;
//const std::vector<std::pair<int,int>> BETA_2({{129,8}});
//const double BETA_2_LSB = TWO_P16;
//const std::vector<std::pair<int,int>> BETA_3({{137,8}});
//const double BETA_3_LSB = TWO_P16;
//const std::vector<std::pair<int,int>> A_1({{151,24}});
//const double A_1_LSB = TWO_N50;
//const std::vector<std::pair<int,int>> A_0({{181,24},{211,8}});
//const double A_0_LSB = TWO_N30;
//const std::vector<std::pair<int,int>> T_OT({{219,8}});
//const double T_OT_LSB = TWO_P12;
//const std::vector<std::pair<int,int>> WN_T({{227,8}});
//const double WN_T_LSB = 1;
//const std::vector<std::pair<int,int>> DELTAT_LS({{241,8}});
//const double DELTAT_LS_LSB = 1;
//const std::vector<std::pair<int,int>> WN_LSF({{249,8}});
//const double WN_LSF_LSB = 1;
//const std::vector<std::pair<int,int>> DN({{257,8}});
//const double DN_LSB = 1;
//const std::vector<std::pair<int,int>> DELTAT_LSF({{271,8}});
//const double DELTAT_LSF_LSB = 1;
//
//// Page 25 - Antispoofing, SV config and SV health (PRN 25 -32)
//const std::vector<std::pair<int,int>> HEALTH_SV25({{229,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV26({{241,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV27({{247,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV28({{253,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV29({{259,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV30({{271,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV31({{277,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV32({{283,6}});
//
//// SUBFRAME 5
////! \todo read all pages of subframe 5
//
//// page 25 - Health (PRN 1 - 24)
//const std::vector<std::pair<int,int>> T_OA({{69,8}});
//const double T_OA_LSB = TWO_P12;
//const std::vector<std::pair<int,int>> WN_A({{77,8}});
//const std::vector<std::pair<int,int>> HEALTH_SV1({{91,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV2({{97,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV3({{103,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV4({{109,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV5({{121,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV6({{127,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV7({{133,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV8({{139,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV9({{151,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV10({{157,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV11({{163,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV12({{169,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV13({{181,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV14({{187,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV15({{193,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV16({{199,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV17({{211,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV18({{217,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV19({{223,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV20({{229,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV21({{241,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV22({{247,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV23({{253,6}});
//const std::vector<std::pair<int,int>> HEALTH_SV24({{259,6}});

#endif /* GNSS_SDR_GPS_L1_CA_H_ */
