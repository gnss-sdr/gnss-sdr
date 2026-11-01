/*!
 * \file GPS_CNAV2.h
 * \brief Defines the GPS CNAV-2 navigation message structure
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 * \author José Antonio Mayo, 2026. contact(at)tatjam.eu
 *
 * Field positions from IS-GPS-800J (01-AUG-2022) and IRN-IS-800J-004.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GPS_CNAV2_H
#define GNSS_SDR_GPS_CNAV2_H

#include "MATH_CONSTANTS.h"
#include <cstdint>
#include <utility>  // std::pair
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


// Field positions are {first bit, number of bits}, with the first bit of the
// subframe numbered 1 (MSB first), as in the figures of IS-GPS-800J.

// SUBFRAME 2: CLOCK, EPHEMERIS, ITOW (Figure 3.5-1, Table 3.5-1)
const std::vector<std::pair<int32_t, int32_t> > CNAV2_WN({{1, 13}});
const std::vector<std::pair<int32_t, int32_t> > CNAV2_ITOW({{14, 8}});
const std::vector<std::pair<int32_t, int32_t> > CNAV2_TOP({{22, 11}});
constexpr int32_t CNAV2_TOP_LSB = 300;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_L1C_HEALTH({{33, 1}});
const std::vector<std::pair<int32_t, int32_t> > CNAV2_URA_ED({{34, 5}});
const std::vector<std::pair<int32_t, int32_t> > CNAV2_TOE({{39, 11}});
constexpr int32_t CNAV2_TOE_LSB = 300;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_DELTA_A({{50, 26}});  // Relative to AREF = 26,559,710 meters
constexpr double CNAV2_DELTA_A_LSB = TWO_N9;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_A_DOT({{76, 25}});
constexpr double CNAV2_A_DOT_LSB = TWO_N21;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_DELTA_N0({{101, 17}});
constexpr double CNAV2_DELTA_N0_LSB = TWO_N44 * GNSS_PI;  // semi-circles to radians
const std::vector<std::pair<int32_t, int32_t> > CNAV2_DELTA_N0_DOT({{118, 23}});
constexpr double CNAV2_DELTA_N0_DOT_LSB = TWO_N57 * GNSS_PI;  // semi-circles to radians
const std::vector<std::pair<int32_t, int32_t> > CNAV2_M0({{141, 33}});
constexpr double CNAV2_M0_LSB = TWO_N32 * GNSS_PI;  // semi-circles to radians
const std::vector<std::pair<int32_t, int32_t> > CNAV2_E_ECCENTRICITY({{174, 33}});
constexpr double CNAV2_E_ECCENTRICITY_LSB = TWO_N34;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_OMEGA({{207, 33}});
constexpr double CNAV2_OMEGA_LSB = TWO_N32 * GNSS_PI;  // semi-circles to radians
const std::vector<std::pair<int32_t, int32_t> > CNAV2_OMEGA0({{240, 33}});
constexpr double CNAV2_OMEGA0_LSB = TWO_N32 * GNSS_PI;  // semi-circles to radians
const std::vector<std::pair<int32_t, int32_t> > CNAV2_I0({{273, 33}});
constexpr double CNAV2_I0_LSB = TWO_N32 * GNSS_PI;                                   // semi-circles to radians
const std::vector<std::pair<int32_t, int32_t> > CNAV2_DELTA_OMEGA_DOT({{306, 17}});  // Relative to REF = -2.6 x 10-9 semi-circles/second
constexpr double CNAV2_DELTA_OMEGA_DOT_LSB = TWO_N44 * GNSS_PI;                      // semi-circles to radians
const std::vector<std::pair<int32_t, int32_t> > CNAV2_I0_DOT({{323, 15}});
constexpr double CNAV2_I0_DOT_LSB = TWO_N44 * GNSS_PI;  // semi-circles to radians
const std::vector<std::pair<int32_t, int32_t> > CNAV2_CIS({{338, 16}});
constexpr double CNAV2_CIS_LSB = TWO_N30;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_CIC({{354, 16}});
constexpr double CNAV2_CIC_LSB = TWO_N30;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_CRS({{370, 24}});
constexpr double CNAV2_CRS_LSB = TWO_N8;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_CRC({{394, 24}});
constexpr double CNAV2_CRC_LSB = TWO_N8;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_CUS({{418, 21}});
constexpr double CNAV2_CUS_LSB = TWO_N30;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_CUC({{439, 21}});
constexpr double CNAV2_CUC_LSB = TWO_N30;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_URA_NED0({{460, 5}});
const std::vector<std::pair<int32_t, int32_t> > CNAV2_URA_NED1({{465, 3}});
const std::vector<std::pair<int32_t, int32_t> > CNAV2_URA_NED2({{468, 3}});
const std::vector<std::pair<int32_t, int32_t> > CNAV2_AF0({{471, 26}});
constexpr double CNAV2_AF0_LSB = TWO_N35;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_AF1({{497, 20}});
constexpr double CNAV2_AF1_LSB = TWO_N48;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_AF2({{517, 10}});
constexpr double CNAV2_AF2_LSB = TWO_N60;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_TGD({{527, 13}});
constexpr double CNAV2_TGD_LSB = TWO_N35;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_ISCL1CP({{540, 13}});
constexpr double CNAV2_ISCL1CP_LSB = TWO_N35;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_ISCL1CD({{553, 13}});
constexpr double CNAV2_ISCL1CD_LSB = TWO_N35;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_ISF({{566, 1}});
const std::vector<std::pair<int32_t, int32_t> > CNAV2_WNOP({{567, 8}});

/*!
 * \brief Value of a 13-bit TGD or ISC field meaning "not available"
 * (two's complement 1000000000000, IS-GPS-200 30.3.3.3.1.1)
 */
constexpr int64_t CNAV2_GROUP_DELAY_NOT_AVAILABLE = -4096;

// SUBFRAME 3, common header (3.5.4)
const std::vector<std::pair<int32_t, int32_t> > CNAV2_SF3_PRN({{1, 8}});
const std::vector<std::pair<int32_t, int32_t> > CNAV2_SF3_PAGE({{9, 6}});

// SUBFRAME 3, PAGE 1: UTC & IONO (Figure 3.5-2, Table 3.5-3)
const std::vector<std::pair<int32_t, int32_t> > CNAV2_A0({{15, 16}});
constexpr double CNAV2_A0_LSB = TWO_N35;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_A1({{31, 13}});
constexpr double CNAV2_A1_LSB = TWO_N51;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_A2({{44, 7}});
constexpr double CNAV2_A2_LSB = TWO_N68;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_DELTA_TLS({{51, 8}});
const std::vector<std::pair<int32_t, int32_t> > CNAV2_TOT({{59, 16}});
constexpr int32_t CNAV2_TOT_LSB = 16;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_WN_OT({{75, 13}});
const std::vector<std::pair<int32_t, int32_t> > CNAV2_WN_LSF({{88, 13}});
const std::vector<std::pair<int32_t, int32_t> > CNAV2_DN({{101, 4}});
const std::vector<std::pair<int32_t, int32_t> > CNAV2_DELTA_TLSF({{105, 8}});
const std::vector<std::pair<int32_t, int32_t> > CNAV2_ALPHA0({{113, 8}});
constexpr double CNAV2_ALPHA0_LSB = TWO_N30;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_ALPHA1({{121, 8}});
constexpr double CNAV2_ALPHA1_LSB = TWO_N27;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_ALPHA2({{129, 8}});
constexpr double CNAV2_ALPHA2_LSB = TWO_N24;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_ALPHA3({{137, 8}});
constexpr double CNAV2_ALPHA3_LSB = TWO_N24;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_BETA0({{145, 8}});
constexpr double CNAV2_BETA0_LSB = TWO_P11;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_BETA1({{153, 8}});
constexpr double CNAV2_BETA1_LSB = TWO_P14;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_BETA2({{161, 8}});
constexpr double CNAV2_BETA2_LSB = TWO_P16;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_BETA3({{169, 8}});
constexpr double CNAV2_BETA3_LSB = TWO_P16;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_ISCL1CA({{177, 13}});
constexpr double CNAV2_ISCL1CA_LSB = TWO_N35;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_ISCL2C({{190, 13}});
constexpr double CNAV2_ISCL2C_LSB = TWO_N35;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_ISCL5I5({{203, 13}});
constexpr double CNAV2_ISCL5I5_LSB = TWO_N35;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_ISCL5Q5({{216, 13}});
constexpr double CNAV2_ISCL5Q5_LSB = TWO_N35;

// SUBFRAME 3, PAGE 2: GGTO & EOP (Figure 3.5-3, Table 3.5-4). Unlike CNAV
// message type 35, the GGTO ID comes first (IRN-IS-800J-003 renamed it from
// GNSS ID): 0 = no data available, 1 = Galileo, 2 = GLONASS, 3 to 7 = unusable
const std::vector<std::pair<int32_t, int32_t> > CNAV2_GGTO_ID({{15, 3}});
const std::vector<std::pair<int32_t, int32_t> > CNAV2_T_GGTO({{18, 16}});
constexpr int32_t CNAV2_T_GGTO_LSB = 16;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_WN_GGTO({{34, 13}});
const std::vector<std::pair<int32_t, int32_t> > CNAV2_A0_GGTO({{47, 16}});
constexpr double CNAV2_A0_GGTO_LSB = TWO_N35;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_A1_GGTO({{63, 13}});
constexpr double CNAV2_A1_GGTO_LSB = TWO_N51;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_A2_GGTO({{76, 7}});
constexpr double CNAV2_A2_GGTO_LSB = TWO_N68;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_T_EOP({{83, 16}});
constexpr int32_t CNAV2_T_EOP_LSB = 16;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_PM_X({{99, 21}});
constexpr double CNAV2_PM_X_LSB = TWO_N20;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_PM_X_DOT({{120, 15}});
constexpr double CNAV2_PM_X_DOT_LSB = TWO_N21;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_PM_Y({{135, 21}});
constexpr double CNAV2_PM_Y_LSB = TWO_N20;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_PM_Y_DOT({{156, 15}});
constexpr double CNAV2_PM_Y_DOT_LSB = TWO_N21;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_DELTA_UT1({{171, 31}});
constexpr double CNAV2_DELTA_UT1_LSB = TWO_N23;
const std::vector<std::pair<int32_t, int32_t> > CNAV2_DELTA_UT1_DOT({{202, 19}});
constexpr double CNAV2_DELTA_UT1_DOT_LSB = TWO_N25;


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_CNAV2_H
