/*!
 * \file SBAS_L1.h
 * \brief  Defines system parameters for SBAS L1 signal
 * \author Miguel Gómez López, 2026. mgomezl(at)ing.uc3m.es
 * \author Víctor Castillo Agüero, 2026. victorcastilloaguero(at)gmail.com
 *
 * Reference: RTCA DO-229E, Minimum Operational Performance Standards for
 * Global Positioning System/Wide Area Augmentation System Airborne Equipment
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


#ifndef GNSS_SDR_SBAS_L1_H
#define GNSS_SDR_SBAS_L1_H

#include "gnss_frequencies.h"
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

// Carrier and code frequencies
constexpr double SBAS_L1_FREQ_HZ = FREQ1;             //!< SBAS L1 carrier frequency [Hz] (same as GPS L1)
constexpr double SBAS_L1_CODE_RATE_CPS = 1.023e6;     //!< SBAS L1 C/A code chip rate [chips/s]
constexpr double SBAS_L1_CODE_LENGTH_CHIPS = 1023.0;  //!< SBAS L1 C/A code length [chips]
constexpr double SBAS_L1_CODE_PERIOD_S = 0.001;       //!< SBAS L1 C/A code period [s]
constexpr uint32_t SBAS_L1_CODE_PERIOD_MS = 1U;       //!< SBAS L1 C/A code period [ms]

// Optimum acquisition sampling frequency
constexpr uint32_t SBAS_L1_OPT_ACQ_FS_SPS = 2000000U;  //!< Sampling frequency that maximises acquisition SNR [samples/s]

// PRN range (RTCA DO-229, Table A-1)
constexpr uint32_t SBAS_L1_PRN_MIN = 120U;  //!< Minimum SBAS PRN (inclusive)
constexpr uint32_t SBAS_L1_PRN_MAX = 138U;  //!< Maximum SBAS PRN (inclusive)

// Message rates and encoding
//   SBAS broadcasts at 250 raw data bits/s, convolutionally encoded
//   at rate 1/2, K=7, yielding a channel symbol rate of 500 symbols/s.
constexpr int32_t SBAS_L1_DATA_RATE_BPS = 250;            //!< Raw data bit rate [bits/s]
constexpr int32_t SBAS_L1_CHANNEL_SYMBOL_RATE_SPS = 500;  //!< Encoded channel symbol rate (rate-1/2 FEC) [symbols/s]
constexpr int32_t SBAS_L1_CHANNEL_SYMBOL_PERIOD_MS = 2;   //!< Encoded channel symbol period [ms] (1/500 sps)
constexpr int32_t SBAS_L1_MSG_LENGTH_BITS = 250;          //!< Information bits per 1-second message frame [bits]
constexpr int32_t SBAS_L1_MSG_LENGTH_BYTES = 32;          //!< Bytes holding a 250-bit frame with six trailing zero bits
constexpr int32_t SBAS_L1_DATA_FIELD_BYTES = 29;          //!< Bytes holding preamble, type, and 212-bit data field

// Maximum coherent integration extension in terms of 1 ms code periods.
// Kept at 1 (no extension): the tracking correlator dumps are not
// synchronized to the encoded symbol boundary, so extending beyond a single
// code period would straddle that boundary and corrupt the sample pairing
// that sbas_l1_telemetry_decoder_gs performs on its own.
constexpr int32_t SBAS_L1_MAX_COHERENT_INTEGRATION_SYMBOLS = 1;

// Preamble bytes (RTCA DO-229, Section A.4.4.2)
constexpr uint8_t SBAS_L1_PREAMBLE_1 = 0x53U;  //!< First preamble byte  (binary: 0101 0011)
constexpr uint8_t SBAS_L1_PREAMBLE_2 = 0x9AU;  //!< Second preamble byte (binary: 1001 1010)
constexpr uint8_t SBAS_L1_PREAMBLE_3 = 0xC6U;  //!< Third preamble byte  (binary: 1100 0110)

/** \} */
/** \} */

#endif  // GNSS_SDR_SBAS_L1_H
