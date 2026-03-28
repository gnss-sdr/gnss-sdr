/*!
 * \file NAVIC_L5.h
 * \brief  Defines system parameters for NavIC (IRNSS) L5 SPS signal
 * \author GNSS-SDR developers, 2024-2025
 *
 * Ref: IRNSS SIS ICD for SPS, Version 1.1, August 2017 (ISRO-IRNSS-ICD-SPS-1.1)
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

#ifndef GNSS_SDR_NAVIC_L5_H
#define GNSS_SDR_NAVIC_L5_H

#include "gnss_frequencies.h"
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

// carrier and code frequencies
constexpr double NAVIC_L5_FREQ_HZ = FREQ5;                //!< L5 carrier frequency [Hz] = 1176.45 MHz
constexpr double NAVIC_L5_CODE_RATE_CPS = 1.023e6;        //!< L5 SPS code rate [chips/s]
constexpr double NAVIC_L5_CODE_LENGTH_CHIPS = 1023.0;     //!< L5 SPS code length [chips]
constexpr double NAVIC_L5_CODE_PERIOD_S = 0.001;          //!< L5 SPS code period [seconds]
constexpr double NAVIC_L5_CHIP_PERIOD_S = 9.7752e-07;     //!< L5 SPS chip period [seconds]
constexpr uint32_t NAVIC_L5_CODE_PERIOD_MS = 1U;          //!< L5 SPS code period [ms]

// S-band frequency for reference
constexpr double NAVIC_S_FREQ_HZ = FREQ9;  //!< S-band carrier frequency [Hz] = 2492.028 MHz

// Navigation message timing
constexpr uint32_t NAVIC_L5_SYMBOL_PERIOD_MS = 20U;       //!< Symbol period [ms] (50 sps)
constexpr uint32_t NAVIC_L5_SYMBOLS_PER_BIT = 20U;        //!< Code periods per symbol (= 20 ms / 1 ms)
constexpr int32_t NAVIC_L5_TELEMETRY_RATE_SYMBOLS_SECOND = 50;  //!< Symbol rate [symbols/s] (after FEC)

// Optimum parameters
constexpr uint32_t NAVIC_L5_OPT_ACQ_FS_SPS = 2000000;    //!< Optimum acquisition sampling frequency [SPS]

// History depth for interpolation
constexpr int32_t NAVIC_L5_HISTORY_DEEP = 100;

// Subframe structure (ICD Section 5.9)
constexpr int32_t NAVIC_L5_SUBFRAME_SYMBOLS = 600;       //!< Symbols per subframe (16 sync + 584 data)
constexpr int32_t NAVIC_L5_SUBFRAME_DATA_SYMBOLS = 584;  //!< FEC-encoded data symbols per subframe
constexpr int32_t NAVIC_L5_SUBFRAME_BITS = 292;          //!< Decoded data bits per subframe (after FEC)
constexpr int32_t NAVIC_L5_SUBFRAME_SECONDS = 12;        //!< Subframe duration [seconds]
constexpr int32_t NAVIC_L5_SUBFRAME_MS = 12000;          //!< Subframe duration [ms]

// Master frame
constexpr int32_t NAVIC_L5_MASTER_FRAME_SUBFRAMES = 4;   //!< Number of subframes per master frame
constexpr int32_t NAVIC_L5_MASTER_FRAME_SECONDS = 48;    //!< Master frame duration [seconds]
constexpr int32_t NAVIC_L5_MASTER_FRAME_SYMBOLS = 2400;  //!< Symbols per master frame

// Preamble / Sync word (ICD Section 5.4)
// Sync pattern: 0xEB90 = 1110 1011 1001 0000
constexpr int32_t NAVIC_L5_PREAMBLE_LENGTH_BITS = 16;
constexpr int32_t NAVIC_L5_PREAMBLE_LENGTH_SYMBOLS = 16;
constexpr int32_t NAVIC_L5_PREAMBLE_PERIOD_SYMBOLS = 600;  //!< Preamble repeats every subframe
constexpr int32_t NAVIC_L5_PREAMBLE_DURATION_MS = 320;     //!< 16 symbols × 20 ms
constexpr char NAVIC_L5_PREAMBLE[17] = "1110101110010000";

// Preamble expanded to code-period resolution for bit synchronization
// Uses first 8 bits of sync word "11101011" × 20 code periods = 160 symbols
// (Same length as GPS L1 C/A bit sync pattern for reliable acquisition)
constexpr int32_t NAVIC_L5_PREAMBLE_LENGTH_SYMBOLS_TRK = 160;
constexpr char NAVIC_L5_PREAMBLE_SYMBOLS_STR[161] =
    "11111111111111111111"   // bit 1: '1'
    "11111111111111111111"   // bit 2: '1'
    "11111111111111111111"   // bit 3: '1'
    "00000000000000000000"   // bit 4: '0'
    "11111111111111111111"   // bit 5: '1'
    "00000000000000000000"   // bit 6: '0'
    "11111111111111111111"   // bit 7: '1'
    "11111111111111111111";  // bit 8: '1'

// Full 16-bit preamble expanded to code-period resolution for bit synchronization
// Uses all 16 bits of sync word "1110101110010000" × 20 code periods = 320 symbols
// Longer pattern reduces false bit sync at ±K code period offsets
constexpr int32_t NAVIC_L5_PREAMBLE_LENGTH_SYMBOLS_TRK_FULL = 320;
constexpr char NAVIC_L5_PREAMBLE_SYMBOLS_STR_FULL[321] =
    "11111111111111111111"   // bit 1:  '1'
    "11111111111111111111"   // bit 2:  '1'
    "11111111111111111111"   // bit 3:  '1'
    "00000000000000000000"   // bit 4:  '0'
    "11111111111111111111"   // bit 5:  '1'
    "00000000000000000000"   // bit 6:  '0'
    "11111111111111111111"   // bit 7:  '1'
    "11111111111111111111"   // bit 8:  '1'
    "11111111111111111111"   // bit 9:  '1'
    "00000000000000000000"   // bit 10: '0'
    "00000000000000000000"   // bit 11: '0'
    "11111111111111111111"   // bit 12: '1'
    "00000000000000000000"   // bit 13: '0'
    "00000000000000000000"   // bit 14: '0'
    "00000000000000000000"   // bit 15: '0'
    "00000000000000000000";  // bit 16: '0'

// FEC encoding parameters (ICD Section 5.2, Table 8)
constexpr int32_t NAVIC_L5_FEC_RATE = 2;           //!< Rate 1/2 → denominator is 2
constexpr int32_t NAVIC_L5_FEC_CONSTRAINT = 7;     //!< Constraint length K=7
constexpr int32_t NAVIC_L5_FEC_G1 = 0171;          //!< Generator polynomial G1 in octal (= 0x79 = 1111001)
constexpr int32_t NAVIC_L5_FEC_G2 = 0133;          //!< Generator polynomial G2 in octal (= 0x5B = 1011011)

// Interleaving parameters (ICD Section 5.3, Table 9)
constexpr int32_t NAVIC_L5_INTERLEAVER_COLS = 73;  //!< Block interleaver columns
constexpr int32_t NAVIC_L5_INTERLEAVER_ROWS = 8;   //!< Block interleaver rows

// CRC parameters (ICD Section 5.6)
// CRC-24Q polynomial: g_i = 1 for i = 0,1,3,4,5,6,7,10,11,14,17,18,23,24
constexpr int32_t NAVIC_L5_CRC_BITS = 24;
constexpr int32_t NAVIC_L5_TAIL_BITS = 6;

// Number of satellites
constexpr int32_t NAVIC_L5_NUM_SATS = 14;          //!< Maximum number of NavIC satellites

// IRNSS Time epoch (ICD Section 5.7)
// Start: 00:00 UT on Sunday August 22, 1999
// Ahead of UTC by 13 leap seconds at start epoch
constexpr int32_t NAVIC_IRNSS_LEAP_SECONDS_AT_EPOCH = 13;

// Week number parameters
constexpr int32_t NAVIC_WN_BITS = 10;              //!< Week number coded on 10 bits
constexpr int32_t NAVIC_WN_ROLLOVER = 1024;        //!< Week number rollover (2^10)

// TOWC parameters
constexpr int32_t NAVIC_TOWC_BITS = 17;
constexpr int32_t NAVIC_TOWC_SCALE = 12;           //!< TOWC × 12 = TOW in seconds
constexpr int32_t NAVIC_TOWC_MAX = 50400;          //!< Maximum TOWC value

// Subframe ID mapping (ICD Table 10)
// ID 00 = Subframe 1, 01 = Subframe 2, 10 = Subframe 3, 11 = Subframe 4

// PRN code generation polynomials (ICD Section 4.1.1)
// G1: X^10 + X^3 + 1
// G2: X^10 + X^9 + X^8 + X^6 + X^3 + X^2 + 1
// G1 taps: {3, 10} (1-indexed register positions)
// G2 taps: {2, 3, 6, 8, 9, 10} (1-indexed register positions)

// G2 initial conditions for L5-SPS (ICD Table 7)
// Each entry: 10-bit binary, MSB = register 1, LSB = register 10
// PRN 1-14
constexpr int32_t NAVIC_L5_G2_INIT[15] = {
    0,            // PRN 0 (unused, placeholder)
    0b1110100111, // PRN 1:  1110100111
    0b0000100110, // PRN 2:  0000100110
    0b1000110100, // PRN 3:  1000110100
    0b0101110010, // PRN 4:  0101110010
    0b1110110000, // PRN 5:  1110110000
    0b0001101011, // PRN 6:  0001101011
    0b0000010100, // PRN 7:  0000010100
    0b0100110000, // PRN 8:  0100110000
    0b0010011000, // PRN 9:  0010011000
    0b1101100100, // PRN 10: 1101100100
    0b0001001100, // PRN 11: 0001001100
    0b1101111100, // PRN 12: 1101111100
    0b1011010010, // PRN 13: 1011010010
    0b0111101010  // PRN 14: 0111101010
};

// G2 initial conditions for S-SPS (ICD Table 7)
constexpr int32_t NAVIC_S_G2_INIT[15] = {
    0,            // PRN 0 (unused, placeholder)
    0b0011101111, // PRN 1:  0011101111
    0b0101111101, // PRN 2:  0101111101
    0b1000110001, // PRN 3:  1000110001
    0b0010101011, // PRN 4:  0010101011
    0b1010010001, // PRN 5:  1010010001
    0b0100101100, // PRN 6:  0100101100
    0b0010001110, // PRN 7:  0010001110
    0b0100100110, // PRN 8:  0100100110
    0b1100001110, // PRN 9:  1100001110
    0b1010111110, // PRN 10: 1010111110
    0b1110010001, // PRN 11: 1110010001
    0b1101101001, // PRN 12: 1101101001
    0b0101000101, // PRN 13: 0101000101
    0b0100001101  // PRN 14: 0100001101
};

// G1 initial condition for all PRNs (ICD Section 4.1.1, v1.1)
constexpr int32_t NAVIC_G1_INIT = 0b1111111111;  //!< All 10 bits set to 1

// Physical constants for IRNSS (ICD Appendix B)
// IRNSS uses WGS-84 constants identical to GPS. Use existing:
//   GPS_GM (MATH_CONSTANTS.h) for gravitational constant [m^3/s^2]
//   GNSS_OMEGA_EARTH_DOT (MATH_CONSTANTS.h) for Earth rotation rate [rad/s]
//   GPS_F (MATH_CONSTANTS.h) for relativistic correction constant [s/(m)^(1/2)]

/** \} */
/** \} */
#endif  // GNSS_SDR_NAVIC_L5_H
