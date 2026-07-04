/*!
 * \file qzss.h
 * \brief  Defines system parameters for QZSS signals
 * \author Carles Fernández-Prades, 2026. cfernandez (at) cttc.es
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


#ifndef GNSS_SDR_QZSS_H
#define GNSS_SDR_QZSS_H

#include "gnss_frequencies.h"
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

static constexpr double QZSS_L1_FREQ_HZ = FREQ1;
static constexpr double QZSS_L5_FREQ_HZ = FREQ5;
static constexpr double QZSS_L1_CHIP_RATE = 1.023e6;
static constexpr double QZSS_L5_CHIP_RATE = 10.23e6;
static constexpr double QZSS_L1_CA_CODE_PERIOD_S = 0.001;
static constexpr double QZSS_L5I_CODE_PERIOD_S = 0.001;

static constexpr int QZSS_L1_CODE_LENGTH = 1023;
// The L1 replica is generated with 2 samples per chip, so a single buffer format
// holds either the L1 C/A BPSK chips (each repeated twice) or the L1 C/B
// sinBOC(1,1) subchips, selected per PRN (IS-QZSS-PNT-006)
static constexpr int QZSS_L1_SAMPLES_PER_CHIP = 2;
static constexpr int QZSS_L5_CODE_LENGTH = 10230;
static constexpr int QZSS_L1_PERIOD_MS = 1;
static constexpr int QZSS_L5I_PERIOD_MS = 1;
static constexpr int32_t QZSS_CA_PREAMBLE_LENGTH_SYMBOLS = 160;
static constexpr int32_t QZSS_CA_TELEMETRY_SYMBOLS_PER_BIT = 20;
static constexpr int32_t QZSS_L5_SAMPLES_PER_SYMBOL = 10;
static constexpr int32_t QZSS_L5Q_NH_CODE_LENGTH = 20;
static constexpr int32_t QZSS_L5I_NH_CODE_LENGTH = 10;
static constexpr uint32_t QZSS_L1_OPT_ACQ_FS_SPS = 2000000;
static constexpr uint32_t QZSS_L5_OPT_ACQ_FS_SPS = 10000000;

static constexpr uint16_t XA_ALL_ONES = 0x1FFF;      // 13 bits all ones
static constexpr uint16_t XA_SHORT_DECODE = 0x1FFD;  // 1111111111101 (ICD)

constexpr const char QZSS_CA_PREAMBLE_SYMBOLS_STR[161] = "1111111111111111111100000000000000000000000000000000000000000000000000000000000011111111111111111111000000000000000000001111111111111111111111111111111111111111";
constexpr const char QZSS_L5Q_NH_CODE_STR[21] = "00000100110101001110";
constexpr const char QZSS_L5I_NH_CODE_STR[11] = "0000110101";

static constexpr int32_t QZSS_LNAV_DATA_ID = 3;
static constexpr int32_t QZSS_ALMANAC_EPOCH_HEALTH_SV_ID = 51;
static constexpr int32_t QZSS_IONO_UTC_WIDE_AREA_SV_ID = 56;
static constexpr int32_t QZSS_IONO_UTC_JAPAN_AREA_SV_ID = 61;
static constexpr uint32_t QZSS_PRN_OFFSET = 192U;
static constexpr uint32_t QZSS_L5_MAX_PRN = 202U;  //!< L5 PRN codes are only defined for PRNs 193-202 (IS-QZSS-PNT, Table 3.2.5-1); L1 C/B PRNs 203-206 have no L5 counterpart
static constexpr double QZSS_QZO_ECCENTRICITY_REF = 0.06;
static constexpr double QZSS_QZO_INCLINATION_REF = 0.25;


//! \brief Maps QZSS L1 C/B PRNs to the PRN carrying the satellite's nominal PNT signals.
//! L1 C/B is broadcast with a dedicated PRN (203-206), but the transmitting satellite is
//! identified by the PRN of its nominal signals (RINEX 4.00, Table 6: QZSS PRN to RINEX
//! Satellite Identifier): 203->196 (J04), 204->197 (J05), 205->200 (J08), 206->201 (J09).
//! Returns the input PRN unchanged if it is not an L1 C/B alias; in particular,
//! PRN 202 (J10) broadcasts L1 C/B under its own PRN.
constexpr uint32_t qzss_l1cb_prn_to_nominal_prn(uint32_t prn)
{
    return prn == 203 ? 196 : (prn == 204 ? 197 : (prn == 205 ? 200 : (prn == 206 ? 201 : prn)));
}

/** \} */
/** \} */

#endif  // GNSS_SDR_QZSS_H
