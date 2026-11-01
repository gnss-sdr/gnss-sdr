/*!
 * \file gps_l1c_signal_replica.h
 * \brief This library implements various functions for GPS L1C signal
 * replica generation
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 *
 * Codes generated as defined in IS-GPS-800J, Sections 3.2.2.1 and 3.3.
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

#ifndef GNSS_SDR_GPS_L1C_SIGNAL_REPLICA_H
#define GNSS_SDR_GPS_L1C_SIGNAL_REPLICA_H

#include <complex>
#include <cstdint>
#include <string>
#if HAS_STD_SPAN
#include <span>
namespace own = std;
#else
#include <gsl-lite/gsl-lite.hpp>
namespace own = gsl_lite;
#endif

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Algorithm_libs algorithms_libs
 * \{ */


/*!
 * \brief Generates the 10230 binary chips (0 or 1) of the L1CP (pilot = true)
 * or L1CD (pilot = false) ranging code for PRN signal numbers 1 to 63.
 * Leaves dest untouched for other PRNs.
 */
void gps_l1c_code_gen_int(own::span<int32_t> dest, bool pilot, uint32_t prn);

/*!
 * \brief Returns the 1800-chip L1CO overlay code of the pilot component as a
 * string of '0' and '1', or an empty string for PRNs outside 1 to 63.
 */
std::string gps_l1c_overlay_code(uint32_t prn);

/*!
 * \brief Generates one code period of the L1CP or L1CD spreading waveform
 * (values +1/-1, a "0" chip is in-phase), without overlay code.
 *
 * With tmboc = false, each chip is a sine-phased BOC(1,1) spreading symbol
 * sampled at 2 samples per chip, and dest.size() must be 20460. With
 * tmboc = true, chips are sampled at 12 samples per chip (dest.size() =
 * 122760) and the pilot uses BOC(6,1) spreading symbols in the TMBOC slots,
 * while the data component is still BOC(1,1).
 */
void gps_l1c_code_gen_float(own::span<float> dest, bool pilot, bool tmboc, uint32_t prn);

/*!
 * \brief Generates one code period of the L1CP or L1CD spreading waveform
 * sampled at sampling_freq, delayed by chip_shift chips, as a complex vector
 * with zero imaginary part.
 */
void gps_l1c_code_gen_complex_sampled(own::span<std::complex<float>> dest,
    bool pilot,
    bool tmboc,
    uint32_t prn,
    int32_t sampling_freq,
    uint32_t chip_shift);


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_L1C_SIGNAL_REPLICA_H
