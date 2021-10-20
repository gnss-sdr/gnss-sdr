/*!
 * \file gnss_sdr_flags.h
 * \brief Helper file for gnss-sdr commandline flags
 * \author Carles Fernandez-Prades, 2018. cfernandez(at)cttc.es
 *
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

#ifndef GNSS_SDR_GNSS_SDR_FLAGS_H
#define GNSS_SDR_GNSS_SDR_FLAGS_H


#include <gflags/gflags.h>
#include <cstdint>

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Gflags gnss_sdr_flags
 * Library for command-line handling.
 * \{ */


DECLARE_string(c);            //!< Path to the configuration file.
DECLARE_string(config_file);  //!< Path to the configuration file.

DECLARE_string(log_dir);  //!< Path to the folder in which logging will be stored.

// Declare flags for signal sources
DECLARE_string(s);                 //!< Path to the file containing the signal samples.
DECLARE_string(signal_source);     //!< Path to the file containing the signal samples.
DECLARE_string(timestamp_source);  //!< Path to the file containing the signal samples.
DECLARE_bool(rf_shutdown);         //!< Shutdown RF when program exits.

// Declare flags for acquisition blocks
DECLARE_int32(doppler_max);   //!< If defined, maximum Doppler value in the search grid, in Hz (overrides the configuration file).
DECLARE_int32(doppler_step);  //!< If defined, sets the frequency step in the search grid, in Hz, in Hz (overrides the configuration file).

// Declare flags for tracking blocks
DECLARE_int32(cn0_samples);            //!< Number of correlator outputs used for CN0 estimation.
DECLARE_int32(cn0_min);                //!< Minimum valid CN0 (in dB-Hz).
DECLARE_int32(max_lock_fail);          //!< Maximum number of code lock failures before dropping a satellite.
DECLARE_int32(max_carrier_lock_fail);  //!< Maximum number of carrier lock failures before dropping a satellite.
DECLARE_double(carrier_lock_th);       //!< Carrier lock threshold (in rad).
DECLARE_double(dll_bw_hz);             //!< Bandwidth of the DLL low pass filter, in Hz (overrides the configuration file).
DECLARE_double(pll_bw_hz);             //!< Bandwidth of the PLL low pass filter, in Hz (overrides the configuration file).

// Declare flags for observables block
DECLARE_int32(carrier_smoothing_factor);  //!< Sets carrier smoothing factor M (overrides the configuration file).
const int32_t DEFAULT_CARRIER_SMOOTHING_FACTOR = 200;

// Declare flags for PVT
DECLARE_string(RINEX_version);  //!< If defined, specifies the RINEX version (2.11 or 3.02). Overrides the configuration file.
DECLARE_string(RINEX_name);     //!< If defined, specifies the RINEX files base name
DECLARE_bool(keyboard);         //!< If set to false, disables the keyboard listener. Only for debug purposes (e.g. ASAN mode termination)

/** \} */
/** \} */
#endif  // GNSS_SDR_GNSS_SDR_FLAGS_H
