/*!
 * \file GPS_CNAV.h
 * \brief  Defines parameters for GPS CNAV
 * \author Antonio Ramos, 2017. antonio.ramos(at)cttc.es
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


#ifndef GNSS_SDR_GPS_CNAV_H
#define GNSS_SDR_GPS_CNAV_H

#include "MATH_CONSTANTS.h"
#include <cstdint>
#include <utility>  // std::pair
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


// CNAV GPS NAVIGATION MESSAGE STRUCTURE
// NAVIGATION MESSAGE FIELDS POSITIONS (from IS-GPS-200M Appendix III)

constexpr int32_t GPS_CNAV_DATA_PAGE_BITS = 300;

// common to all messages
const std::vector<std::pair<int32_t, int32_t> > CNAV_PRN({{9, 6}});
const std::vector<std::pair<int32_t, int32_t> > CNAV_MSG_TYPE({{15, 6}});
const std::vector<std::pair<int32_t, int32_t> > CNAV_TOW({{21, 17}});  // GPS Time Of Week in seconds
constexpr int32_t CNAV_TOW_LSB = 6;
const std::vector<std::pair<int32_t, int32_t> > CNAV_ALERT_FLAG({{38, 1}});

// MESSAGE TYPE 10 (Ephemeris 1)
const std::vector<std::pair<int32_t, int32_t> > CNAV_WN({{39, 13}});
const std::vector<std::pair<int32_t, int32_t> > CNAV_HEALTH({{52, 3}});
const std::vector<std::pair<int32_t, int32_t> > CNAV_TOP1({{55, 11}});
constexpr int32_t CNAV_TOP1_LSB = 300;
const std::vector<std::pair<int32_t, int32_t> > CNAV_URA({{66, 5}});

const std::vector<std::pair<int32_t, int32_t> > CNAV_TOE1({{71, 11}});
constexpr int32_t CNAV_TOE1_LSB = 300;

const std::vector<std::pair<int32_t, int32_t> > CNAV_DELTA_A({{82, 26}});  // Relative to AREF = 26,559,710 meters
constexpr double CNAV_DELTA_A_LSB = TWO_N9;

const std::vector<std::pair<int32_t, int32_t> > CNAV_A_DOT({{108, 25}});
constexpr double CNAV_A_DOT_LSB = TWO_N21;

const std::vector<std::pair<int32_t, int32_t> > CNAV_DELTA_N0({{133, 17}});
constexpr double CNAV_DELTA_N0_LSB = TWO_N44 * GNSS_PI;  // semi-circles to radians
const std::vector<std::pair<int32_t, int32_t> > CNAV_DELTA_N0_DOT({{150, 23}});
constexpr double CNAV_DELTA_N0_DOT_LSB = TWO_N57 * GNSS_PI;  // semi-circles to radians
const std::vector<std::pair<int32_t, int32_t> > CNAV_M0({{173, 33}});
constexpr double CNAV_M0_LSB = TWO_N32 * GNSS_PI;  // semi-circles to radians
const std::vector<std::pair<int32_t, int32_t> > CNAV_E_ECCENTRICITY({{206, 33}});
constexpr double CNAV_E_ECCENTRICITY_LSB = TWO_N34;
const std::vector<std::pair<int32_t, int32_t> > CNAV_OMEGA({{239, 33}});
constexpr double CNAV_OMEGA_LSB = TWO_N32 * GNSS_PI;  // semi-circles to radians
const std::vector<std::pair<int32_t, int32_t> > CNAV_INTEGRITY_FLAG({{272, 1}});
const std::vector<std::pair<int32_t, int32_t> > CNAV_L2_PHASING_FLAG({{273, 1}});

// MESSAGE TYPE 11 (Ephemeris 2)
const std::vector<std::pair<int32_t, int32_t> > CNAV_TOE2({{39, 11}});
constexpr int32_t CNAV_TOE2_LSB = 300;
const std::vector<std::pair<int32_t, int32_t> > CNAV_OMEGA0({{50, 33}});
constexpr double CNAV_OMEGA0_LSB = TWO_N32 * GNSS_PI;  // semi-circles to radians
const std::vector<std::pair<int32_t, int32_t> > CNAV_I0({{83, 33}});
constexpr double CNAV_I0_LSB = TWO_N32 * GNSS_PI;                                   // semi-circles to radians
const std::vector<std::pair<int32_t, int32_t> > CNAV_DELTA_OMEGA_DOT({{116, 17}});  // Relative to REF = -2.6 x 10-9 semi-circles/second.
constexpr double CNAV_DELTA_OMEGA_DOT_LSB = TWO_N44 * GNSS_PI;                      // semi-circles to radians
const std::vector<std::pair<int32_t, int32_t> > CNAV_I0_DOT({{133, 15}});
constexpr double CNAV_I0_DOT_LSB = TWO_N44 * GNSS_PI;  // semi-circles to radians
const std::vector<std::pair<int32_t, int32_t> > CNAV_CIS({{148, 16}});
constexpr double CNAV_CIS_LSB = TWO_N30;
const std::vector<std::pair<int32_t, int32_t> > CNAV_CIC({{164, 16}});
constexpr double CNAV_CIC_LSB = TWO_N30;
const std::vector<std::pair<int32_t, int32_t> > CNAV_CRS({{180, 24}});
constexpr double CNAV_CRS_LSB = TWO_N8;
const std::vector<std::pair<int32_t, int32_t> > CNAV_CRC({{204, 24}});
constexpr double CNAV_CRC_LSB = TWO_N8;
const std::vector<std::pair<int32_t, int32_t> > CNAV_CUS({{228, 21}});
constexpr double CNAV_CUS_LSB = TWO_N30;
const std::vector<std::pair<int32_t, int32_t> > CNAV_CUC({{249, 21}});
constexpr double CNAV_CUC_LSB = TWO_N30;


// MESSAGE TYPE 30 (CLOCK, IONO, GRUP DELAY)
const std::vector<std::pair<int32_t, int32_t> > CNAV_TOP2({{39, 11}});
constexpr int32_t CNAV_TOP2_LSB = 300;
const std::vector<std::pair<int32_t, int32_t> > CNAV_URA_NED0({{50, 5}});
const std::vector<std::pair<int32_t, int32_t> > CNAV_URA_NED1({{55, 3}});
const std::vector<std::pair<int32_t, int32_t> > CNAV_URA_NED2({{58, 3}});
const std::vector<std::pair<int32_t, int32_t> > CNAV_TOC({{61, 11}});
constexpr int32_t CNAV_TOC_LSB = 300;
const std::vector<std::pair<int, int> > CNAV_AF0({{72, 26}});
constexpr double CNAV_AF0_LSB = TWO_N35;
const std::vector<std::pair<int, int> > CNAV_AF1({{98, 20}});
constexpr double CNAV_AF1_LSB = TWO_N48;
const std::vector<std::pair<int, int> > CNAV_AF2({{118, 10}});
constexpr double CNAV_AF2_LSB = TWO_N60;
const std::vector<std::pair<int, int> > CNAV_TGD({{128, 13}});
constexpr double CNAV_TGD_LSB = TWO_N35;
const std::vector<std::pair<int32_t, int32_t> > CNAV_ISCL1({{141, 13}});
constexpr double CNAV_ISCL1_LSB = TWO_N35;
const std::vector<std::pair<int32_t, int32_t> > CNAV_ISCL2({{154, 13}});
constexpr double CNAV_ISCL2_LSB = TWO_N35;
const std::vector<std::pair<int32_t, int32_t> > CNAV_ISCL5I({{167, 13}});
constexpr double CNAV_ISCL5I_LSB = TWO_N35;
const std::vector<std::pair<int32_t, int32_t> > CNAV_ISCL5Q({{180, 13}});
constexpr double CNAV_ISCL5Q_LSB = TWO_N35;
// Ionospheric parameters
const std::vector<std::pair<int32_t, int32_t> > CNAV_ALPHA0({{193, 8}});
constexpr double CNAV_ALPHA0_LSB = TWO_N30;
const std::vector<std::pair<int32_t, int32_t> > CNAV_ALPHA1({{201, 8}});
constexpr double CNAV_ALPHA1_LSB = TWO_N27;
const std::vector<std::pair<int32_t, int32_t> > CNAV_ALPHA2({{209, 8}});
constexpr double CNAV_ALPHA2_LSB = TWO_N24;
const std::vector<std::pair<int32_t, int32_t> > CNAV_ALPHA3({{217, 8}});
constexpr double CNAV_ALPHA3_LSB = TWO_N24;
const std::vector<std::pair<int32_t, int32_t> > CNAV_BETA0({{225, 8}});
constexpr double CNAV_BETA0_LSB = TWO_P11;
const std::vector<std::pair<int32_t, int32_t> > CNAV_BETA1({{233, 8}});
constexpr double CNAV_BETA1_LSB = TWO_P14;
const std::vector<std::pair<int32_t, int32_t> > CNAV_BETA2({{241, 8}});
constexpr double CNAV_BETA2_LSB = TWO_P16;
const std::vector<std::pair<int32_t, int32_t> > CNAV_BETA3({{249, 8}});
constexpr double CNAV_BETA3_LSB = TWO_P16;
const std::vector<std::pair<int32_t, int32_t> > CNAV_WNOP({{257, 8}});


// MESSAGE TYPE 33 (CLOCK and UTC)
const std::vector<std::pair<int32_t, int32_t> > CNAV_A0({{128, 16}});
constexpr double CNAV_A0_LSB = TWO_N35;
const std::vector<std::pair<int32_t, int32_t> > CNAV_A1({{144, 13}});
constexpr double CNAV_A1_LSB = TWO_N51;
const std::vector<std::pair<int32_t, int32_t> > CNAV_A2({{157, 7}});
constexpr double CNAV_A2_LSB = TWO_N68;
const std::vector<std::pair<int32_t, int32_t> > CNAV_DELTA_TLS({{164, 8}});
constexpr int32_t CNAV_DELTA_TLS_LSB = 1;
const std::vector<std::pair<int32_t, int32_t> > CNAV_TOT({{172, 16}});
constexpr int32_t CNAV_TOT_LSB = TWO_P4;
const std::vector<std::pair<int32_t, int32_t> > CNAV_WN_OT({{188, 13}});
constexpr int32_t CNAV_WN_OT_LSB = 1;
const std::vector<std::pair<int32_t, int32_t> > CNAV_WN_LSF({{201, 13}});
constexpr int32_t CNAV_WN_LSF_LSB = 1;
const std::vector<std::pair<int32_t, int32_t> > CNAV_DN({{214, 4}});
constexpr int32_t CNAV_DN_LSB = 1;
const std::vector<std::pair<int32_t, int32_t> > CNAV_DELTA_TLSF({{218, 8}});
constexpr int32_t CNAV_DELTA_TLSF_LSB = 1;

constexpr double CNAV_A_REF = 26559710.0;       // [m] See IS-GPS-200M, Table 30-I.
constexpr double CNAV_OMEGA_DOT_REF = -2.6e-9;  // [semicircles / s], see IS-GPS-200M, Table 30-I.

// TODO: Add more frames (Almanac, etc...)


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_CNAV_H
