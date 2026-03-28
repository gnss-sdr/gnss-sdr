/*!
 * \file NAVIC_LNAV.h
 * \brief  Defines system parameters for NavIC (IRNSS) L-band NAV message processing
 * \author GNSS-SDR developers, 2024-2025
 *
 * Ref: IRNSS SIS ICD for SPS, Version 1.1, August 2017 (ISRO-IRNSS-ICD-SPS-1.1)
 *
 * Bit positions are 1-based within the 292-bit decoded subframe.
 * MSB is bit 1, transmitted first.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2025  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_NAVIC_LNAV_H
#define GNSS_SDR_NAVIC_LNAV_H

#include "MATH_CONSTANTS.h"
#include <cstdint>
#include <utility>
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

// ============================================================================
// NAVIC LNAV SCALE FACTORS (from ICD Tables 11, 12, 15, 16, 17)
// ============================================================================

// Subframe 1 - Clock and signal parameters
constexpr double NAVIC_LNAV_AF0_LSB = TWO_N31;             //!< Clock bias [sec], 2's complement
constexpr double NAVIC_LNAV_AF1_LSB = TWO_N43;             //!< Clock drift [sec/sec], 2's complement
constexpr double NAVIC_LNAV_AF2_LSB = TWO_N55;             //!< Clock drift rate [sec/sec^2], 2's complement
constexpr double NAVIC_LNAV_TOC_LSB = TWO_P4;              //!< Time of clock [sec], unsigned (scale = 16)
constexpr double NAVIC_LNAV_TGD_LSB = TWO_N31;             //!< Total group delay [sec], 2's complement
constexpr double NAVIC_LNAV_DELTA_N_LSB = PI_TWO_N41;      //!< Mean motion difference [rad/sec], 2's complement (2^-41 semi-circles/sec)
constexpr double NAVIC_LNAV_CUC_LSB = TWO_N28;             //!< Cuc [rad], 2's complement
constexpr double NAVIC_LNAV_CUS_LSB = TWO_N28;             //!< Cus [rad], 2's complement
constexpr double NAVIC_LNAV_CIC_LSB = TWO_N28;             //!< Cic [rad], 2's complement
constexpr double NAVIC_LNAV_CIS_LSB = TWO_N28;             //!< Cis [rad], 2's complement
constexpr double NAVIC_LNAV_CRC_LSB = TWO_N4;              //!< Crc [meters], 2's complement
constexpr double NAVIC_LNAV_CRS_LSB = TWO_N4;              //!< Crs [meters], 2's complement
constexpr double NAVIC_LNAV_IDOT_LSB = PI_TWO_N43;         //!< Rate of inclination [rad/sec], 2's complement (2^-43 semi-circles/sec)

// Subframe 2 - Ephemeris parameters
constexpr double NAVIC_LNAV_M0_LSB = PI_TWO_N31;           //!< Mean anomaly [rad], 2's complement (2^-31 semi-circles)
constexpr double NAVIC_LNAV_TOE_LSB = TWO_P4;              //!< Time of ephemeris [sec], unsigned (scale = 16)
constexpr double NAVIC_LNAV_E_LSB = TWO_N33;               //!< Eccentricity [dimensionless], unsigned
constexpr double NAVIC_LNAV_SQRT_A_LSB = TWO_N19;          //!< Square root of semi-major axis [sqrt(m)], unsigned
constexpr double NAVIC_LNAV_OMEGA0_LSB = PI_TWO_N31;       //!< Longitude of ascending node [rad], 2's complement (2^-31 semi-circles)
constexpr double NAVIC_LNAV_OMEGA_LSB = PI_TWO_N31;        //!< Argument of perigee [rad], 2's complement (2^-31 semi-circles)
constexpr double NAVIC_LNAV_OMEGA_DOT_LSB = PI_TWO_N41;    //!< Rate of RAAN [rad/sec], 2's complement (2^-41 semi-circles/sec)
constexpr double NAVIC_LNAV_I0_LSB = PI_TWO_N31;           //!< Inclination [rad], 2's complement (2^-31 semi-circles)

// Message Type 7 - Almanac parameters
// NOTE: Gnss_Almanac base class stores angular values in semi-circles,
// converting to radians internally during position computation.
// Therefore, almanac angular LSBs do NOT include the PI factor.
constexpr double NAVIC_LNAV_ALM_E_LSB = TWO_N21;           //!< Almanac eccentricity, unsigned
constexpr double NAVIC_LNAV_ALM_TOA_LSB = TWO_P4;          //!< Almanac reference time [sec], unsigned (scale = 16)
constexpr double NAVIC_LNAV_ALM_I0_LSB = TWO_N23;          //!< Almanac inclination [semi-circles], 2's complement (absolute, NOT delta)
constexpr double NAVIC_LNAV_ALM_OMEGA_DOT_LSB = TWO_N38;   //!< Almanac rate of RAAN [semi-circles/sec], 2's complement
constexpr double NAVIC_LNAV_ALM_SQRT_A_LSB = TWO_N11;      //!< Almanac sqrt(A) [sqrt(m)], unsigned
constexpr double NAVIC_LNAV_ALM_OMEGA0_LSB = TWO_N23;      //!< Almanac RAAN [semi-circles], 2's complement
constexpr double NAVIC_LNAV_ALM_OMEGA_LSB = TWO_N23;       //!< Almanac arg of perigee [semi-circles], 2's complement
constexpr double NAVIC_LNAV_ALM_M0_LSB = TWO_N23;          //!< Almanac mean anomaly [semi-circles], 2's complement
constexpr double NAVIC_LNAV_ALM_AF0_LSB = TWO_N20;         //!< Almanac clock bias [sec], 2's complement
constexpr double NAVIC_LNAV_ALM_AF1_LSB = TWO_N38;         //!< Almanac clock drift [sec/sec], 2's complement
constexpr double NAVIC_LNAV_ALM_ISC_LSB = TWO_N31;         //!< Inter-signal correction [sec], 2's complement

// Message Type 9/26 - UTC and GNSS time parameters
constexpr double NAVIC_LNAV_A0UTC_LSB = TWO_N35;           //!< IRNSS-UTC bias [sec], 2's complement
constexpr double NAVIC_LNAV_A1UTC_LSB = TWO_N51;           //!< IRNSS-UTC drift [sec/sec], 2's complement
constexpr double NAVIC_LNAV_A2UTC_LSB = TWO_N68;           //!< IRNSS-UTC drift rate [sec/sec^2], 2's complement
constexpr double NAVIC_LNAV_TOUTC_LSB = TWO_P4;            //!< UTC reference time [sec], unsigned (scale = 16)
constexpr double NAVIC_LNAV_A0_LSB = TWO_N35;              //!< IRNSS-GNSS bias [sec], 2's complement
constexpr double NAVIC_LNAV_A1_LSB = TWO_N51;              //!< IRNSS-GNSS drift [sec/sec], 2's complement
constexpr double NAVIC_LNAV_A2_LSB = TWO_N68;              //!< IRNSS-GNSS drift rate [sec/sec^2], 2's complement
constexpr double NAVIC_LNAV_TOT_LSB = TWO_P4;              //!< GNSS reference time [sec], unsigned (scale = 16)

// Message Type 11 - Ionosphere coefficients
constexpr double NAVIC_LNAV_ALPHA0_LSB = TWO_N30;          //!< Alpha 0 [sec], 2's complement
constexpr double NAVIC_LNAV_ALPHA1_LSB = TWO_N27;          //!< Alpha 1 [sec/semi-circle], 2's complement
constexpr double NAVIC_LNAV_ALPHA2_LSB = TWO_N24;          //!< Alpha 2 [sec/semi-circle^2], 2's complement
constexpr double NAVIC_LNAV_ALPHA3_LSB = TWO_N24;          //!< Alpha 3 [sec/semi-circle^3], 2's complement
constexpr double NAVIC_LNAV_BETA0_LSB = TWO_P11;           //!< Beta 0 [sec], 2's complement
constexpr double NAVIC_LNAV_BETA1_LSB = TWO_P14;           //!< Beta 1 [sec/semi-circle], 2's complement
constexpr double NAVIC_LNAV_BETA2_LSB = TWO_P16;           //!< Beta 2 [sec/semi-circle^2], 2's complement
constexpr double NAVIC_LNAV_BETA3_LSB = TWO_P16;           //!< Beta 3 [sec/semi-circle^3], 2's complement

// Message Type 11 - Earth Orientation Parameters
constexpr double NAVIC_LNAV_TEOP_LSB = TWO_P4;             //!< EOP reference time [sec], unsigned (scale = 16)
constexpr double NAVIC_LNAV_PM_X_LSB = TWO_N20;            //!< X-axis polar motion [arc-sec], 2's complement
constexpr double NAVIC_LNAV_PM_X_DOT_LSB = TWO_N21;        //!< X-axis polar motion drift [arc-sec/day], 2's complement
constexpr double NAVIC_LNAV_PM_Y_LSB = TWO_N20;            //!< Y-axis polar motion [arc-sec], 2's complement
constexpr double NAVIC_LNAV_PM_Y_DOT_LSB = TWO_N21;        //!< Y-axis polar motion drift [arc-sec/day], 2's complement
constexpr double NAVIC_LNAV_DELTA_UT1_LSB = TWO_N24;       //!< UT1-UTC difference [sec], 2's complement
constexpr double NAVIC_LNAV_DELTA_UT1_DOT_LSB = TWO_N25;   //!< UT1-UTC rate [sec/day], 2's complement

// ============================================================================
// NAVIC LNAV NAVIGATION MESSAGE BIT FIELD POSITIONS
// Format: vector of pairs (start_bit, num_bits), 1-based indexing
// Within the 292-bit decoded subframe
// ============================================================================

// ----- Common header fields (all subframes) -----
const std::vector<std::pair<int32_t, int32_t>> NAVIC_TLM({{1, 8}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_TOWC({{9, 17}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_ALERT({{26, 1}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_AUTONAV({{27, 1}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_SUBFRAME_ID({{28, 2}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_SPARE_BIT({{30, 1}});

// ----- Subframe 1 data fields (ICD Table 11, Figure 13) -----
const std::vector<std::pair<int32_t, int32_t>> NAVIC_WN({{31, 10}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_AF0({{41, 22}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_AF1({{63, 16}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_AF2({{79, 8}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_URA({{87, 4}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_TOC({{91, 16}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_TGD({{107, 8}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_DELTA_N({{115, 22}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_IODEC({{137, 8}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_SF1_RESERVED({{145, 10}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_L5_FLAG({{155, 1}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_S_FLAG({{156, 1}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_CUC({{157, 15}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_CUS({{172, 15}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_CIC({{187, 15}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_CIS({{202, 15}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_CRC({{217, 15}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_CRS({{232, 15}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_IDOT({{247, 14}});

// ----- Subframe 2 data fields (ICD Table 12, Figure 14) -----
const std::vector<std::pair<int32_t, int32_t>> NAVIC_M0({{31, 32}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_TOE({{63, 16}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_ECCENTRICITY({{79, 32}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_SQRT_A({{111, 32}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_OMEGA0({{143, 32}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_OMEGA({{175, 32}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_OMEGA_DOT({{207, 22}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_I0({{229, 32}});

// ----- Subframe 3 & 4 common fields (ICD Figure 12) -----
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MSG_ID({{31, 6}});
// Data area: bits 37-256 (220 bits) — contents depend on message type
const std::vector<std::pair<int32_t, int32_t>> NAVIC_PRN_ID({{257, 6}});

// ----- Message Type 5: Ionospheric Grid Parameters (ICD Table 14, Figure 15) -----
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT5_REGIONS_MASKED({{37, 10}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT5_REGION_ID({{47, 4}});
// 15 sets of (GIVEI, GIVD) follow: each GIVEI=4 bits, GIVD=9 bits = 13 bits × 15 = 195 bits
// Starting from bit 51:
// IGP 1: GIVEI bits 51-54, GIVD bits 55-63
// IGP 2: GIVEI bits 64-67, GIVD bits 68-76
// ... pattern repeats every 13 bits
// IGP 15: GIVEI bits 233-236, GIVD bits 237-245
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT5_IODI({{246, 3}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT5_SPARE({{249, 8}});

// ----- Message Type 7: Almanac (ICD Table 15, Figure 16) -----
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT7_WNA({{37, 10}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT7_E({{47, 16}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT7_TOA({{63, 16}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT7_I0({{79, 24}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT7_OMEGA_DOT({{103, 16}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT7_SQRT_A({{119, 24}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT7_OMEGA0({{143, 24}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT7_OMEGA({{167, 24}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT7_M0({{191, 24}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT7_AF0({{215, 11}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT7_AF1({{226, 11}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT7_PRN_ID_AL({{237, 6}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT7_ISC({{243, 8}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT7_SPARE({{251, 6}});

// ----- Message Type 9: UTC and GPS Time Parameters (ICD Table 16, Figure 17) -----
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT9_A0UTC({{37, 16}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT9_A1UTC({{53, 13}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT9_A2UTC({{66, 7}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT9_DELTA_TLS({{73, 8}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT9_TOUTC({{81, 16}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT9_WNUTC({{97, 10}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT9_WNLSF({{107, 10}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT9_DN({{117, 4}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT9_DELTA_TLSF({{121, 8}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT9_A0({{129, 16}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT9_A1({{145, 13}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT9_A2({{158, 7}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT9_TOT({{165, 16}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT9_WNOT({{181, 10}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT9_GNSS_ID({{191, 3}});

// ----- Message Type 11: EOP and Ionosphere Coefficients (ICD Table 17, Figure 18) -----
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT11_TEOP({{37, 16}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT11_PM_X({{53, 21}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT11_PM_X_DOT({{74, 15}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT11_PM_Y({{89, 21}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT11_PM_Y_DOT({{110, 15}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT11_DELTA_UT1({{125, 31}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT11_DELTA_UT1_DOT({{156, 19}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT11_ALPHA0({{175, 8}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT11_ALPHA1({{183, 8}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT11_ALPHA2({{191, 8}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT11_ALPHA3({{199, 8}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT11_BETA0({{207, 8}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT11_BETA1({{215, 8}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT11_BETA2({{223, 8}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT11_BETA3({{231, 8}});

// ----- Message Type 14: Differential Corrections (ICD Table 18, Figure 19) -----
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT14_RESERVED_BIT({{37, 1}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT14_PRN_ID_DC({{38, 6}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT14_DELTA_AF0({{44, 13}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT14_DELTA_AF1({{57, 8}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT14_UDRA({{65, 5}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT14_DELTA_ALPHA({{70, 14}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT14_DELTA_BETA({{84, 14}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT14_DELTA_GAMMA({{98, 15}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT14_DELTA_I({{113, 12}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT14_DELTA_OMEGA({{125, 12}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT14_DELTA_A({{137, 12}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT14_UDRA_DOT({{149, 5}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT14_IODEC_DC({{154, 8}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT14_DC_RESERVED({{162, 10}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT14_TOD({{172, 16}});

// ----- Message Type 18: Text Message (ICD Table 19, Figure 20) -----
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT18_TEXT_ID({{37, 4}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT18_BLOCK_COUNT({{41, 8}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT18_BLOCK_ID({{49, 8}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT18_TEXT_DATA({{57, 200}});

// ----- Message Type 26: UTC, UTC(NPLI) and Other GNSS Time (ICD Table 20, Figure 21) -----
// Same layout as MT9 for the first 9 parameters (UTC section)
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT26_A0UTC({{37, 16}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT26_A1UTC({{53, 13}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT26_A2UTC({{66, 7}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT26_DELTA_TLS({{73, 8}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT26_TOUTC({{81, 16}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT26_WNUTC({{97, 10}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT26_WNLSF({{107, 10}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT26_DN({{117, 4}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT26_DELTA_TLSF({{121, 8}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT26_A0({{129, 16}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT26_A1({{145, 13}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT26_A2({{158, 7}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT26_TOT({{165, 16}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT26_WNOT({{181, 10}});
const std::vector<std::pair<int32_t, int32_t>> NAVIC_MT26_GNSS_ID({{191, 3}});

// ----- Message Type 5: IGP Grid GIVEI/GIVD pairs -----
// Helper: bit positions for the 15 IGP entries in MT5
// Each IGP: 4-bit GIVEI + 9-bit GIVD = 13 bits, starting at bit 51
constexpr int32_t NAVIC_MT5_IGP_START_BIT = 51;
constexpr int32_t NAVIC_MT5_IGP_ENTRY_BITS = 13;
constexpr int32_t NAVIC_MT5_GIVEI_BITS = 4;
constexpr int32_t NAVIC_MT5_GIVD_BITS = 9;
constexpr double NAVIC_MT5_GIVD_LSB = 0.125;  //!< GIVD scale factor [meters]

// Message type identifiers (ICD Table 13)
constexpr int32_t NAVIC_MSG_TYPE_IONO_GRID = 5;
constexpr int32_t NAVIC_MSG_TYPE_ALMANAC = 7;
constexpr int32_t NAVIC_MSG_TYPE_UTC_GPS = 9;
constexpr int32_t NAVIC_MSG_TYPE_EOP_IONO = 11;
constexpr int32_t NAVIC_MSG_TYPE_DIFF_CORR = 14;
constexpr int32_t NAVIC_MSG_TYPE_TEXT = 18;
constexpr int32_t NAVIC_MSG_TYPE_UTC_GNSS = 26;
constexpr int32_t NAVIC_MSG_TYPE_NULL = 0;

// GNSS Type ID to Timescale mapping (ICD Table 30)
constexpr int32_t NAVIC_GNSS_ID_GPS = 0;
constexpr int32_t NAVIC_GNSS_ID_GALILEO = 1;
constexpr int32_t NAVIC_GNSS_ID_GLONASS = 2;
constexpr int32_t NAVIC_GNSS_ID_UTC_NPLI = 7;

// IRNSS time to GPS time offset constants
// IRNSS epoch: Aug 22, 1999 00:00:00 UTC = GPS week 1024, TOW 518400
// GPS epoch: Jan 6, 1980 00:00:00 UTC
constexpr int32_t NAVIC_IRNSS2GPST_WEEK_OFFSET = 1024;  //!< GPS week at start of IRNSS time

/** \} */
/** \} */
#endif  // GNSS_SDR_NAVIC_LNAV_H
