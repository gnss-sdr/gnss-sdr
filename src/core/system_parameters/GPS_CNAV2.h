/*!
 * \file GPS_CNAV2.h
 * \brief  Defines parameters for GPS CNAV2
 * \author José Antonio Mayo, 2026. contact(at)tatjam.eu
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
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


// CNAV2 GPS NAVIGATION MESSAGE STRUCTURE
// NAVIGATION MESSAGE FIELDS POSITIONS (from IS-GPS-800J)

constexpr int GPS_L1C_SF_2_DATA_BITS = 600;  //!< Number of bits in a decoded subframe 2
constexpr int GPS_L1C_SF_3_DATA_BITS = 274;  //!< Number of bits in a decoded subframe 3

// SF2 Message

const std::vector<std::pair<int32_t, int32_t> > CNAV2_WN({{1, 13}});                 //!< uint16_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_ITOW({{14, 8}});               //!< uint8_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_TOP({{22, 11}});               //!< uint16_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_L1C_HEALTH({{33, 1}});         //!< bool
const std::vector<std::pair<int32_t, int32_t> > CNAV2_URAED({{34, 5}});              //!< int16_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_TOE({{39, 11}});               //!< uint16_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_DELTA_A({{50, 26}});           //!< int32_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_DOT_A({{76, 25}});             //!< int32_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_DELTA_N({{101, 17}});          //!< int32_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_DELTA_DOT_N({{118, 23}});      //!< int32_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_M0({{141, 33}});               //!< int64_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_E({{174, 33}});                //!< uint64_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_OMEGA({{207, 33}});            //!< lowercase omega, int64_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_OMEGA0({{240, 33}});           //!< uppercas Omega, int64_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_I0({{273, 33}});               //!< int64_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_DELTA_OMEGA_DOT({{306, 17}});  //!< Omega, int32_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_IDOT({{323, 15}});             //!< int16_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_CIS({{338, 16}});              //!< int16_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_CIC({{354, 16}});              //!< int16_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_CRS({{370, 24}});              //!< int32_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_CRC({{394, 24}});              //!< int32_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_CUS({{418, 21}});              //!< int32_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_CUC({{439, 21}});              //!< int32_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_URA_NED0({{460, 5}});          //!< int8_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_URA_NED1({{465, 3}});          //!< uint8_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_URA_NED2({{468, 3}});          //!< uint8_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_AF0({{471, 26}});              //!< int32_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_AF1({{497, 20}});              //!< int32_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_AF2({{517, 10}});              //!< int16_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_TGD({{527, 13}});              //!< int16_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_ISC_L1CP({{540, 13}});         //!< int16_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_ISC_L1CD({{553, 13}});         //!< int16_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_ISF({{566, 1}});               //!< bool
const std::vector<std::pair<int32_t, int32_t> > CNAV2_WNOP({{567, 8}});              //!< uint8_t
const std::vector<std::pair<int32_t, int32_t> > CNAV2_RESERVED({{575, 2}});          //!< uint16_t

constexpr int32_t CNAV2_TOE_LSB = 300;
constexpr double CNAV2_DELTA_A_LSB = TWO_N9;
constexpr double CNAV2_A_REF = 26559710.0;  // [m] See IS-GPS-800J, Table 3.5-2.
constexpr double CNAV2_QZSS_A_REF = 42164200.0;
constexpr double CNAV2_A_DOT_LSB = TWO_N21;
constexpr double CNAV2_DELTA_N0_LSB = TWO_N44 * GNSS_PI;  // semi-circles to radians
constexpr double CNAV2_M0_LSB = TWO_N32 * GNSS_PI;        // semi-circles to radians
constexpr double CNAV2_E_ECCENTRICITY_LSB = TWO_N34;
constexpr double CNAV2_DELTA_OMEGA_DOT_LSB = TWO_N44 * GNSS_PI;  // semi-circles to radians
constexpr double CNAV2_OMEGA_DOT_REF = -2.6e-9;                  // [semicircles / s], see IS-GPS-200M, Table 30-I.
constexpr double CNAV2_I0_LSB = TWO_N32 * GNSS_PI;               // semi-circles to radians
constexpr double CNAV2_I0_DOT_LSB = TWO_N44 * GNSS_PI;           // semi-circles to radians
constexpr double CNAV2_CIS_LSB = TWO_N30;
constexpr double CNAV2_CIC_LSB = TWO_N30;
constexpr double CNAV2_CRS_LSB = TWO_N8;
constexpr double CNAV2_CRC_LSB = TWO_N8;
constexpr double CNAV2_CUS_LSB = TWO_N30;
constexpr double CNAV2_CUC_LSB = TWO_N30;
constexpr double CNAV2_AF0_LSB = TWO_N35;
constexpr double CNAV2_AF1_LSB = TWO_N48;
constexpr double CNAV2_AF2_LSB = TWO_N60;
constexpr double CNAV2_TGD_LSB = TWO_N35;
constexpr double CNAV2_DELTA_N0_DOT_LSB = TWO_N57 * GNSS_PI;  // semi-circles to radians
constexpr double CNAV2_OMEGA_LSB = TWO_N32 * GNSS_PI;         // semi-circles to radians
constexpr double CNAV2_OMEGA0_LSB = TWO_N32 * GNSS_PI;        // semi-circles to radians


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_CNAV2_H
