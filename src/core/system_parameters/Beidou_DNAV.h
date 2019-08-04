/*!
 * \file Beidou_DNAV.h
 * \brief  Defines system parameters for BeiDou DNAV data processing
 * \author Damian Miralles, 2018. dmiralles2009@gmail.com
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <httpS://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_BEIDOU_DNAV_H_
#define GNSS_SDR_BEIDOU_DNAV_H_

#include "MATH_CONSTANTS.h"
#include <cstdint>
#include <utility>
#include <vector>

const double BEIDOU_DNAV_C_M_S = 299792458.0;             //!< The speed of light, [m/s]
const double BEIDOU_DNAV_C_M_MS = 299792.4580;            //!< The speed of light, [m/ms]
const double BEIDOU_DNAV_PI = 3.1415926535898;            //!< BeiDou DNAV Pi
const double BEIDOU_DNAV_TWO_PI = 6.2831853071796;        //!< BeiDou DNAV 2Pi
const double BEIDOU_DNAV_OMEGA_EARTH_DOT = 7.2921150e-5;  //!< Earth rotation rate, [rad/s] as defined in CGCS2000
const double BEIDOU_DNAV_GM = 3.986004418e14;             //!< Universal gravitational constant times the mass of the Earth, [m^3/s^2] as defined in CGCS2000
const double BEIDOU_DNAV_F = -4.442807309e-10;            //!< Constant, [s/(m)^(1/2)] F=-2(GM)^.5/C^2

const int32_t BEIDOU_DNAV_PREAMBLE_LENGTH_BITS = 11;
const int32_t BEIDOU_DNAV_PREAMBLE_LENGTH_SYMBOLS = 11;  // **************
const int32_t BEIDOU_DNAV_PREAMBLE_PERIOD_SYMBOLS = 300;
const uint32_t BEIDOU_DNAV_SUBFRAME_SYMBOLS = 300;
const int32_t BEIDOU_DNAV_SUBFRAME_DATA_BITS = 300;  //!< Number of bits per subframe in the NAV message [bits]
const uint32_t BEIDOU_DNAV_WORDS_SUBFRAME = 10;
const uint32_t BEIDOU_DNAV_WORD_LENGTH_BITS = 30;
const std::string BEIDOU_DNAV_PREAMBLE = "11100010010";

// Number of leap seconds passed from the start of the GPS epoch up to the start of BeiDou epoch
const int32_t BEIDOU_DNAV_BDT2GPST_LEAP_SEC_OFFSET = 14;
// Number of weeks passed from the start of the GPS epoch up to the start of BeiDou epoch
const int32_t BEIDOU_DNAV_BDT2GPST_WEEK_NUM_OFFSET = 1356;

// BEIDOU D1 NAVIGATION MESSAGE STRUCTURE
// GENERAL
const std::vector<std::pair<int32_t, int32_t> > D1_PRE({{1, 11}});
const std::vector<std::pair<int32_t, int32_t> > D1_FRAID({{16, 3}});
const std::vector<std::pair<int32_t, int32_t> > D1_SOW({{19, 8}, {31, 12}});
const std::vector<std::pair<int32_t, int32_t> > D1_PNUM({{44, 7}});

// DNAV SCALE FACTORS
// EPH
const double D1_TOC_LSB = TWO_P3;
const double D1_TGD1_LSB = 0.1e-9;
const double D1_TGD2_LSB = 0.1e-9;
const double D1_ALPHA0_LSB = TWO_N30;
const double D1_ALPHA1_LSB = TWO_N27;
const double D1_ALPHA2_LSB = TWO_N24;
const double D1_ALPHA3_LSB = TWO_N24;
const double D1_BETA0_LSB = TWO_P11;
const double D1_BETA1_LSB = TWO_P14;
const double D1_BETA2_LSB = TWO_P16;
const double D1_BETA3_LSB = TWO_P16;
const double D1_A2_LSB = TWO_N66;
const double D1_A0_LSB = TWO_N33;
const double D1_A1_LSB = TWO_N50;
const double D1_DELTA_N_LSB = PI_TWO_N43;
const double D1_CUC_LSB = TWO_N31;
const double D1_M0_LSB = PI_TWO_N31;
const double D1_E_LSB = TWO_N33;
const double D1_CUS_LSB = TWO_N31;
const double D1_CRC_LSB = TWO_N6;
const double D1_CRS_LSB = TWO_N6;
const double D1_SQRT_A_LSB = TWO_N19;
const double D1_TOE_LSB = TWO_P3;
const double D1_I0_LSB = PI_TWO_N31;
const double D1_CIC_LSB = TWO_N31;
const double D1_OMEGA_DOT_LSB = PI_TWO_N43;
const double D1_CIS_LSB = TWO_N31;
const double D1_IDOT_LSB = PI_TWO_N43;
const double D1_OMEGA0_LSB = PI_TWO_N31;
const double D1_OMEGA_LSB = PI_TWO_N31;
//ALM
const double D1_SQRT_A_ALMANAC_LSB = TWO_N11;
const double D1_A1_ALMANAC_LSB = TWO_N38;
const double D1_A0_ALMANAC_LSB = TWO_N20;
const double D1_OMEGA0_ALMANAC_LSB = PI_TWO_N23;
const double D1_E_ALMANAC_LSB = TWO_N21;
const double D1_DELTA_I_LSB = PI_TWO_N19;
const double D1_TOA_LSB = TWO_P12;
const double D1_OMEGA_DOT_ALMANAC_LSB = PI_TWO_N38;
const double D1_OMEGA_ALMANAC_LSB = PI_TWO_N23;
const double D1_M0_ALMANAC_LSB = PI_TWO_N23;
const double D1_A0GPS_LSB = 0.1e-9;
const double D1_A1GPS_LSB = 0.1e-9;
const double D1_A0GAL_LSB = 0.1e-9;
const double D1_A1GAL_LSB = 0.1e-9;
const double D1_A0GLO_LSB = 0.1e-9;
const double D1_A1GLO_LSB = 0.1e-9;
const double D1_A0UTC_LSB = TWO_N30;
const double D1_A1UTC_LSB = TWO_N50;

// SUBFRAME 1
const std::vector<std::pair<int32_t, int32_t> > D1_SAT_H1({{43, 1}});
const std::vector<std::pair<int32_t, int32_t> > D1_AODC({{44, 5}});
const std::vector<std::pair<int32_t, int32_t> > D1_URAI({{49, 4}});
const std::vector<std::pair<int32_t, int32_t> > D1_WN({{61, 13}});
const std::vector<std::pair<int32_t, int32_t> > D1_TOC({{74, 9}, {91, 8}});
const std::vector<std::pair<int32_t, int32_t> > D1_TGD1({{99, 10}});
const std::vector<std::pair<int32_t, int32_t> > D1_TGD2({{121, 6}});
const std::vector<std::pair<int32_t, int32_t> > D1_ALPHA0({{127, 8}});
const std::vector<std::pair<int32_t, int32_t> > D1_ALPHA1({{135, 8}});
const std::vector<std::pair<int32_t, int32_t> > D1_ALPHA2({{151, 8}});
const std::vector<std::pair<int32_t, int32_t> > D1_ALPHA3({{159, 8}});
const std::vector<std::pair<int32_t, int32_t> > D1_BETA0({{167, 6}, {181, 2}});
const std::vector<std::pair<int32_t, int32_t> > D1_BETA1({{183, 8}});
const std::vector<std::pair<int32_t, int32_t> > D1_BETA2({{191, 8}});
const std::vector<std::pair<int32_t, int32_t> > D1_BETA3({{199, 4}, {211, 4}});
const std::vector<std::pair<int32_t, int32_t> > D1_A2({{215, 11}});
const std::vector<std::pair<int32_t, int32_t> > D1_A0({{226, 7}, {241, 17}});
const std::vector<std::pair<int32_t, int32_t> > D1_A1({{258, 5}, {271, 17}});
const std::vector<std::pair<int32_t, int32_t> > D1_AODE({{288, 5}});

//SUBFRAME 2
const std::vector<std::pair<int32_t, int32_t> > D1_DELTA_N({{43, 10}, {61, 6}});
const std::vector<std::pair<int32_t, int32_t> > D1_CUC({{67, 16}, {91, 2}});
const std::vector<std::pair<int32_t, int32_t> > D1_M0({{93, 20}, {121, 12}});
const std::vector<std::pair<int32_t, int32_t> > D1_E({{133, 10}, {151, 22}});
const std::vector<std::pair<int32_t, int32_t> > D1_CUS({{181, 18}});
const std::vector<std::pair<int32_t, int32_t> > D1_CRC({{199, 4}, {211, 14}});
const std::vector<std::pair<int32_t, int32_t> > D1_CRS({{225, 8}, {241, 10}});
const std::vector<std::pair<int32_t, int32_t> > D1_SQRT_A({{251, 12}, {271, 20}});
const std::vector<std::pair<int32_t, int32_t> > D1_TOE_SF2({{291, 2}});

//SUBFRAME 3
const std::vector<std::pair<int32_t, int32_t> > D1_TOE_SF3({{43, 10}, {61, 5}});
const std::vector<std::pair<int32_t, int32_t> > D1_I0({{66, 17}, {91, 15}});
const std::vector<std::pair<int32_t, int32_t> > D1_CIC({{106, 7}, {121, 11}});
const std::vector<std::pair<int32_t, int32_t> > D1_OMEGA_DOT({{132, 11}, {151, 13}});
const std::vector<std::pair<int32_t, int32_t> > D1_CIS({{164, 9}, {181, 9}});
const std::vector<std::pair<int32_t, int32_t> > D1_IDOT({{190, 13}, {211, 1}});
const std::vector<std::pair<int32_t, int32_t> > D1_OMEGA0({{212, 21}, {241, 11}});
const std::vector<std::pair<int32_t, int32_t> > D1_OMEGA({{252, 11}, {271, 21}});

//SUBFRAME 4 AND PAGES 1 THROUGH 6 IN SUBFRAME 5
const std::vector<std::pair<int32_t, int32_t> > D1_SQRT_A_ALMANAC({{51, 2}, {61, 22}});
const std::vector<std::pair<int32_t, int32_t> > D1_A1_ALMANAC({{91, 11}});
const std::vector<std::pair<int32_t, int32_t> > D1_A0_ALMANAC({{102, 11}});
const std::vector<std::pair<int32_t, int32_t> > D1_OMEGA0_ALMANAC({{121, 22}, {151, 2}});
const std::vector<std::pair<int32_t, int32_t> > D1_E_ALMANAC({{153, 17}});
const std::vector<std::pair<int32_t, int32_t> > D1_DELTA_I({{170, 3}, {181, 13}});
const std::vector<std::pair<int32_t, int32_t> > D1_TOA({{194, 8}});
const std::vector<std::pair<int32_t, int32_t> > D1_OMEGA_DOT_ALMANAC({{202, 1}, {211, 16}});
const std::vector<std::pair<int32_t, int32_t> > D1_OMEGA_ALMANAC({{227, 6}, {241, 18}});
const std::vector<std::pair<int32_t, int32_t> > D1_M0_ALMANAC({{259, 4}, {271, 20}});

//SUBFRAME 5 PAGE 7
const std::vector<std::pair<int32_t, int32_t> > D1_HEA1({{51, 2}, {61, 7}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA2({{68, 9}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA3({{77, 6}, {91, 3}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA4({{94, 9}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA5({{103, 9}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA6({{112, 1}, {121, 8}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA7({{129, 9}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA8({{138, 5}, {151, 4}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA9({{155, 9}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA10({{164, 9}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA11({{181, 9}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA12({{190, 9}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA13({{199, 4}, {211, 5}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA14({{216, 9}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA15({{225, 8}, {241, 1}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA16({{242, 9}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA17({{251, 9}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA18({{260, 3}, {271, 6}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA19({{277, 9}});

//SUBFRAME 5 PAGE 8
const std::vector<std::pair<int32_t, int32_t> > D1_HEA20({{51, 2}, {61, 7}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA21({{68, 9}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA22({{77, 6}, {91, 3}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA23({{94, 9}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA24({{103, 9}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA25({{112, 1}, {121, 8}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA26({{129, 9}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA27({{138, 5}, {151, 4}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA28({{155, 9}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA29({{164, 9}});
const std::vector<std::pair<int32_t, int32_t> > D1_HEA30({{181, 9}});
const std::vector<std::pair<int32_t, int32_t> > D1_WNA({{190, 8}});
const std::vector<std::pair<int32_t, int32_t> > D1_TOA2({{198, 5}, {211, 3}});

//SUBFRAME 5 PAGE 9
const std::vector<std::pair<int32_t, int32_t> > D1_A0GPS({{97, 14}});
const std::vector<std::pair<int32_t, int32_t> > D1_A1GPS({{111, 2}, {121, 14}});
const std::vector<std::pair<int32_t, int32_t> > D1_A0GAL({{135, 8}, {151, 6}});
const std::vector<std::pair<int32_t, int32_t> > D1_A1GAL({{157, 16}});
const std::vector<std::pair<int32_t, int32_t> > D1_A0GLO({{181, 14}});
const std::vector<std::pair<int32_t, int32_t> > D1_A1GLO({{195, 8}, {211, 8}});

//SUBFRAME 5 PAGE 10
const std::vector<std::pair<int32_t, int32_t> > D1_DELTA_T_LS({{51, 2}, {61, 6}});
const std::vector<std::pair<int32_t, int32_t> > D1_DELTA_T_LSF({{67, 8}});
const std::vector<std::pair<int32_t, int32_t> > D1_WN_LSF({{75, 8}});
const std::vector<std::pair<int32_t, int32_t> > D1_A0UTC({{91, 22}, {121, 10}});
const std::vector<std::pair<int32_t, int32_t> > D1_A1UTC({{131, 12}, {151, 12}});
const std::vector<std::pair<int32_t, int32_t> > D1_DN({{163, 8}});

// D2 NAV Message Decoding Information
const std::vector<std::pair<int32_t, int32_t> > D2_PRE({{1, 11}});
const std::vector<std::pair<int32_t, int32_t> > D2_FRAID({{16, 3}});
const std::vector<std::pair<int32_t, int32_t> > D2_SOW({{19, 8}, {31, 12}});
const std::vector<std::pair<int32_t, int32_t> > D2_PNUM({{43, 4}});

// D2 NAV, SUBFRAME 1, PAGE 1
const std::vector<std::pair<int32_t, int32_t> > D2_SAT_H1({{47, 1}});
const std::vector<std::pair<int32_t, int32_t> > D2_AODC({{48, 5}});
const std::vector<std::pair<int32_t, int32_t> > D2_URAI({{61, 4}});
const std::vector<std::pair<int32_t, int32_t> > D2_WN({{65, 13}});
const std::vector<std::pair<int32_t, int32_t> > D2_TOC({{78, 5}, {91, 12}});
const std::vector<std::pair<int32_t, int32_t> > D2_TGD1({{103, 10}});
const std::vector<std::pair<int32_t, int32_t> > D2_TGD2({{121, 10}});

// D2 NAV, SUBFRAME 1, PAGE 2
const std::vector<std::pair<int32_t, int32_t> > D2_ALPHA0({{47, 6}, {61, 2}});
const std::vector<std::pair<int32_t, int32_t> > D2_ALPHA1({{63, 8}});
const std::vector<std::pair<int32_t, int32_t> > D2_ALPHA2({{71, 8}});
const std::vector<std::pair<int32_t, int32_t> > D2_ALPHA3({{79, 4}, {91, 4}});
const std::vector<std::pair<int32_t, int32_t> > D2_BETA0({{95, 8}});
const std::vector<std::pair<int32_t, int32_t> > D2_BETA1({{103, 8}});
const std::vector<std::pair<int32_t, int32_t> > D2_BETA2({{111, 2}, {121, 6}});
const std::vector<std::pair<int32_t, int32_t> > D2_BETA3({{127, 8}});

// D2 NAV, SUBFRAME 1, PAGE 3
const std::vector<std::pair<int32_t, int32_t> > D2_A0({{101, 12}, {121, 12}});
const std::vector<std::pair<int32_t, int32_t> > D2_A1_MSB({{133, 4}});
const std::vector<std::pair<int32_t, int32_t> > D2_A1_LSB({{47, 6}, {61, 12}});
const std::vector<std::pair<int32_t, int32_t> > D2_A1({{279, 22}});

// D2 NAV, SUBFRAME 1, PAGE 4
const std::vector<std::pair<int32_t, int32_t> > D2_A2({{73, 10}, {91, 1}});
const std::vector<std::pair<int32_t, int32_t> > D2_AODE({{92, 5}});
const std::vector<std::pair<int32_t, int32_t> > D2_DELTA_N({{97, 16}});
const std::vector<std::pair<int32_t, int32_t> > D2_CUC_MSB({{121, 14}});
const std::vector<std::pair<int32_t, int32_t> > D2_CUC_LSB({{47, 4}});
const std::vector<std::pair<int32_t, int32_t> > D2_CUC({{283, 18}});

// D2 NAV, SUBFRAME 1, PAGE 5
const std::vector<std::pair<int32_t, int32_t> > D2_M0({{51, 2}, {61, 22}, {91, 8}});
const std::vector<std::pair<int32_t, int32_t> > D2_CUS({{99, 14}, {121, 4}});
const std::vector<std::pair<int32_t, int32_t> > D2_E_MSB({{125, 10}});

// D2 NAV, SUBFRAME 1, PAGE 6
const std::vector<std::pair<int32_t, int32_t> > D2_E_LSB({{47, 6}, {61, 16}});
const std::vector<std::pair<int32_t, int32_t> > D2_SQRT_A({{77, 6}, {91, 22}, {121, 4}});
const std::vector<std::pair<int32_t, int32_t> > D2_CIC_MSB({{125, 10}});
const std::vector<std::pair<int32_t, int32_t> > D2_CIC_LSB({{47, 6}, {61, 2}});
const std::vector<std::pair<int32_t, int32_t> > D2_CIC({{283, 18}});

// D2 NAV, SUBFRAME 1, PAGE 7
const std::vector<std::pair<int32_t, int32_t> > D2_CIS({{63, 18}});
const std::vector<std::pair<int32_t, int32_t> > D2_TOE({{81, 2}, {91, 15}});
const std::vector<std::pair<int32_t, int32_t> > D2_I0_MSB({{106, 7}, {121, 14}});
const std::vector<std::pair<int32_t, int32_t> > D2_I0_LSB({{47, 6}, {61, 5}});
const std::vector<std::pair<int32_t, int32_t> > D2_I0({{269, 32}});

// D2 NAV, SUBFRAME 1, PAGE 8
const std::vector<std::pair<int32_t, int32_t> > D2_CRC({{66, 17}, {91, 1}});
const std::vector<std::pair<int32_t, int32_t> > D2_CRS({{92, 18}});
const std::vector<std::pair<int32_t, int32_t> > D2_OMEGA_DOT_MSB({{110, 3}, {121, 16}});
const std::vector<std::pair<int32_t, int32_t> > D2_OMEGA_DOT_LSB({{47, 5}});
const std::vector<std::pair<int32_t, int32_t> > D2_OMEGA_DOT({{277, 24}});

// D2 NAV, SUBFRAME 1, PAGE 9
const std::vector<std::pair<int32_t, int32_t> > D2_OMEGA0({{52, 1}, {61, 22}, {91, 9}});
const std::vector<std::pair<int32_t, int32_t> > D2_OMEGA_MSB({{100, 13}, {121, 14}});
const std::vector<std::pair<int32_t, int32_t> > D2_OMEGA_LSB({{47, 5}});
const std::vector<std::pair<int32_t, int32_t> > D2_OMEGA({{269, 32}});

// D2 NAV, SUBFRAME 1, PAGE 10
const std::vector<std::pair<int32_t, int32_t> > D2_IDOT({{52, 1}, {61, 13}});

#endif /* GNSS_SDR_BEIDOU_DNAV_H_ */
