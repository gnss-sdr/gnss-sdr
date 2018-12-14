/*!
 * \file gnss_sdr_flags.h
 * \brief Helper file for gnss-sdr commandline flags
 * \author Carles Fernandez-Prades, 2018. cfernandez(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_FLAGS_H_
#define GNSS_SDR_FLAGS_H_


#include <gflags/gflags.h>

DECLARE_string(c);            //<! Path to the configuration file.
DECLARE_string(config_file);  //<! Path to the configuration file.

DECLARE_string(log_dir);  //<! Path to the folder in which logging will be stored.

// Declare flags for signal sources
DECLARE_string(s);              //<! Path to the file containing the signal samples.
DECLARE_string(signal_source);  //<! Path to the file containing the signal samples.

// Declare flags for acquisition blocks
DECLARE_int32(doppler_max);   //<! If defined, maximum Doppler value in the search grid, in Hz (overrides the configuration file).
DECLARE_int32(doppler_step);  //<! If defined, sets the frequency step in the search grid, in Hz, in Hz (overrides the configuration file).

// Declare flags for tracking blocks
DECLARE_int32(cn0_samples);       //<! Number of correlator outputs used for CN0 estimation.
DECLARE_int32(cn0_min);           //<! Minimum valid CN0 (in dB-Hz).
DECLARE_int32(max_lock_fail);     //<! Maximum number of lock failures before dropping a satellite.
DECLARE_double(carrier_lock_th);  //<! Carrier lock threshold (in rad).
DECLARE_double(dll_bw_hz);        //<! Bandwidth of the DLL low pass filter, in Hz (overrides the configuration file).
DECLARE_double(pll_bw_hz);        //<! Bandwidth of the PLL low pass filter, in Hz (overrides the configuration file).

// Declare flags for PVT
DECLARE_string(RINEX_version);  //<! If defined, specifies the RINEX version (2.11 or 3.02). Overrides the configuration file.


#endif
