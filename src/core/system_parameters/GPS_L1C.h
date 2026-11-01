/*!
 * \file GPS_L1C.h
 * \brief Defines system parameters for the GPS L1C signal
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 * \author José Antonio Mayo, 2026. contact(at)tatjam.eu
 *
 * Parameter values from IS-GPS-800J (01-AUG-2022).
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

#ifndef GNSS_SDR_GPS_L1C_H
#define GNSS_SDR_GPS_L1C_H

#include "gnss_frequencies.h"
#include <array>
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


// Carrier and codes (IS-GPS-800J, 3.2)
constexpr double GPS_L1C_FREQ_HZ = FREQ1;                                      //!< L1 [Hz]
constexpr double GPS_L1C_CODE_RATE_CPS = 1.023e6;                              //!< L1CP and L1CD chipping rate [chips/s]
constexpr double GPS_L1C_CODE_LENGTH_CHIPS = 10230.0;                          //!< L1CP and L1CD code length [chips]
constexpr double GPS_L1C_CODE_PERIOD_S = 0.010;                                //!< L1CP and L1CD code period [s]
constexpr uint32_t GPS_L1C_CODE_PERIOD_MS = 10U;                               //!< L1CP and L1CD code period [ms]
constexpr uint32_t GPS_L1C_OPT_ACQ_FS_SPS = 4000000;                           //!< Sampling frequency for BOC(1,1) acquisition [samples/s]
constexpr int32_t GPS_L1C_NUMBER_OF_PRNS = 63;                                 //!< PRN signal numbers 1-63 (Table 3.2-2)
constexpr int32_t GPS_L1C_WEIL_LENGTH = 10223;                                 //!< Length of the Legendre sequence and of the Weil codes
constexpr int32_t GPS_L1C_OVERLAY_CODE_LENGTH = 1800;                          //!< L1CO code length [chips], one chip per L1CP code period
constexpr int32_t GPS_L1C_TMBOC_SAMPLES_PER_CHIP = 12;                         //!< Samples per chip of a TMBOC(6,1,4/33) replica
constexpr int32_t GPS_L1C_BOC_SAMPLES_PER_CHIP = 2;                            //!< Samples per chip of a BOC(1,1) replica
constexpr std::array<int32_t, 4> GPS_L1C_TMBOC_BOC61_SLOTS = {{0, 4, 6, 29}};  //!< BOC(6,1) spreading symbols at t mod 33 (3.3)
constexpr float GPS_L1C_TMBOC_EARLY_LATE_SPACE_CHIPS = 0.2F;                   //!< Early-late spacing that keeps a unique DLL lock with a TMBOC replica [chips]
constexpr float GPS_L1C_TMBOC_EARLY_LATE_SPACE_NARROW_CHIPS = 0.07F;           //!< Narrow early-late spacing that keeps a unique DLL lock with a TMBOC replica [chips]
constexpr float GPS_L1C_OVERLAY_LOCK_RATIO = 0.9F;                             //!< Fraction of the 1800 overlay chips that must match to declare secondary code lock

// Navigation message structure (IS-GPS-800J, 3.2.3)
constexpr int32_t GPS_L1C_SYMBOLS_PER_BIT = 1;        //!< One message symbol per code period (100 sps)
constexpr int32_t GPS_L1C_FRAME_SYMBOLS = 1800;       //!< Symbols per 18 s frame
constexpr int32_t GPS_L1C_FRAME_PERIOD_S = 18;        //!< Frame duration [s]
constexpr uint32_t GPS_L1C_FRAME_PERIOD_MS = 18000U;  //!< Frame duration [ms]
constexpr int32_t GPS_L1C_TOI_SYMBOLS = 52;           //!< BCH(51,8)-encoded TOI plus its MSB
constexpr int32_t GPS_L1C_TOI_MAX = 399;              //!< Largest TOI count
constexpr int32_t GPS_L1C_SF2_BITS = 600;             //!< Subframe 2 bits, including CRC
constexpr int32_t GPS_L1C_SF3_BITS = 274;             //!< Subframe 3 bits, including CRC
constexpr int32_t GPS_L1C_SF2_SYMBOLS = 1200;         //!< LDPC-encoded subframe 2
constexpr int32_t GPS_L1C_SF3_SYMBOLS = 548;          //!< LDPC-encoded subframe 3
constexpr int32_t GPS_L1C_INTERLEAVER_ROWS = 38;      //!< Block interleaver rows (3.2.3.5)
constexpr int32_t GPS_L1C_INTERLEAVER_COLS = 46;      //!< Block interleaver columns (3.2.3.5)
constexpr int32_t GPS_L1C_CRC_BITS = 24;              //!< CRC-24Q parity bits per subframe

// Frame synchronization: about half of the parity checks fail for random data,
// and 42 % at Es/N0 = 2 dB, where the LDPC decoder still succeeds
constexpr float GPS_L1C_TOI_MIN_QUALITY = 0.6F;             //!< Minimum normalized TOI correlation of a frame candidate
constexpr float GPS_L1C_SF2_MAX_UNSATISFIED_RATIO = 0.47F;  //!< Maximum fraction of unsatisfied subframe 2 checks of a frame candidate

/*!
 * \brief Seconds in a two-hour ITOW epoch (3.5.3.2)
 */
constexpr int32_t GPS_L1C_ITOW_PERIOD_S = 7200;


// Table 3.2-2. L1C ranging code parameter assignments, PRN signal numbers 1-63
constexpr std::array<uint16_t, GPS_L1C_NUMBER_OF_PRNS> GPS_L1CP_WEIL_INDEX = {{5111, 5109, 5108, 5106, 5103, 5101, 5100, 5098, 5095, 5094,
    5093, 5091, 5090, 5081, 5080, 5069, 5068, 5054, 5044, 5027,
    5026, 5014, 5004, 4980, 4915, 4909, 4893, 4885, 4832, 4824,
    4591, 3706, 5092, 4986, 4965, 4920, 4917, 4858, 4847, 4790,
    4770, 4318, 4126, 3961, 3790, 4911, 4881, 4827, 4795, 4789,
    4725, 4675, 4539, 4535, 4458, 4197, 4096, 3484, 3481, 3393,
    3175, 2360, 1852}};

constexpr std::array<uint16_t, GPS_L1C_NUMBER_OF_PRNS> GPS_L1CP_INSERTION_INDEX = {{412, 161, 1, 303, 207, 4971, 4496, 5, 4557, 485,
    253, 4676, 1, 66, 4485, 282, 193, 5211, 729, 4848,
    982, 5955, 9805, 670, 464, 29, 429, 394, 616, 9457,
    4429, 4771, 365, 9705, 9489, 4193, 9947, 824, 864, 347,
    677, 6544, 6312, 9804, 278, 9461, 444, 4839, 4144, 9875,
    197, 1156, 4674, 10035, 4504, 5, 9937, 430, 5, 355,
    909, 1622, 6284}};

constexpr std::array<uint16_t, GPS_L1C_NUMBER_OF_PRNS> GPS_L1CD_WEIL_INDEX = {{5097, 5110, 5079, 4403, 4121, 5043, 5042, 5104, 4940, 5035,
    4372, 5064, 5084, 5048, 4950, 5019, 5076, 3736, 4993, 5060,
    5061, 5096, 4983, 4783, 4991, 4815, 4443, 4769, 4879, 4894,
    4985, 5056, 4921, 5036, 4812, 4838, 4855, 4904, 4753, 4483,
    4942, 4813, 4957, 4618, 4669, 4969, 5031, 5038, 4740, 4073,
    4843, 4979, 4867, 4964, 5025, 4579, 4390, 4763, 4612, 4784,
    3716, 4703, 4851}};

constexpr std::array<uint16_t, GPS_L1C_NUMBER_OF_PRNS> GPS_L1CD_INSERTION_INDEX = {{181, 359, 72, 1110, 1480, 5034, 4622, 1, 4547, 826,
    6284, 4195, 368, 1, 4796, 523, 151, 713, 9850, 5734,
    34, 6142, 190, 644, 467, 5384, 801, 594, 4450, 9437,
    4307, 5906, 378, 9448, 9432, 5849, 5547, 9546, 9132, 403,
    3766, 3, 684, 9711, 333, 6124, 10216, 4251, 9893, 9884,
    4627, 4449, 9798, 985, 4272, 126, 10024, 434, 1029, 561,
    289, 638, 4353}};


// Table 3.2-3. L1CO overlay code parameter assignments, PRN signal numbers 1-63.
// The S1 polynomial coefficients are given as 1, m10, ..., m1, 1 (octal 5111
// is 1 + x^3 + x^6 + x^9 + x^11). The initial bits are the initial condition
// n11, ..., n1 of the generator, and also the first 11 output bits.
constexpr std::array<uint16_t, GPS_L1C_NUMBER_OF_PRNS> GPS_L1CO_S1_POLYNOMIAL = {{05111, 05421, 05501, 05403, 06417, 06141, 06351, 06501,
    06205, 06235, 07751, 06623, 06733, 07627, 05667, 05051,
    07665, 06325, 04365, 04745, 07633, 06747, 04475, 04225,
    07063, 04423, 06651, 04161, 07237, 04473, 05477, 06163,
    07223, 06323, 07125, 07035, 04341, 04353, 04107, 05735,
    06741, 07071, 04563, 05755, 06127, 04671, 04511, 04533,
    05357, 05607, 06673, 06153, 07565, 07107, 06211, 04321,
    07201, 04451, 05411, 05141, 07041, 06637, 04577}};

constexpr std::array<uint16_t, GPS_L1C_NUMBER_OF_PRNS> GPS_L1CO_INITIAL_BITS = {{03266, 02040, 01527, 03307, 03756, 03026, 00562, 00420,
    03415, 00337, 00265, 01230, 02204, 01440, 02412, 03516,
    02761, 03750, 02701, 01206, 01544, 01774, 00546, 02213,
    03707, 02051, 03650, 01777, 03203, 01762, 02100, 00571,
    03710, 03535, 03110, 01426, 00255, 00321, 03124, 00572,
    01736, 03306, 01307, 03763, 01604, 01021, 02624, 00406,
    00114, 00077, 03477, 01000, 03460, 02607, 02057, 03467,
    00706, 02032, 01464, 00520, 01766, 03270, 00341}};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_L1C_H
