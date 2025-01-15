/*!
 * \file gnss_sdr_flags.h
 * \brief Helper file for gnss-sdr commandline flags
 * \author Carles Fernandez-Prades, 2018-2024. cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2024  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GNSS_SDR_FLAGS_H
#define GNSS_SDR_GNSS_SDR_FLAGS_H

#include <cstdint>
#include <string>

#if USE_GLOG_AND_GFLAGS
#include <gflags/gflags.h>
#else
#include <absl/flags/declare.h>
#include <absl/flags/flag.h>
#include <cstdlib>
#include <iostream>
#include <sys/stat.h>
#include <vector>
#endif


/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Gflags gnss_sdr_flags
 * Library for command-line handling.
 * \{ */

#if USE_GLOG_AND_GFLAGS
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

#else
ABSL_DECLARE_FLAG(std::string, c);            //!< Path to the configuration file.
ABSL_DECLARE_FLAG(std::string, config_file);  //!< Path to the configuration file.

ABSL_DECLARE_FLAG(std::string, log_dir);  //!< Path to the folder in which logging will be stored.

// Declare flags for signal sources
ABSL_DECLARE_FLAG(std::string, s);                 //!< Path to the file containing the signal samples.
ABSL_DECLARE_FLAG(std::string, signal_source);     //!< Path to the file containing the signal samples.
ABSL_DECLARE_FLAG(std::string, timestamp_source);  //!< Path to the file containing the signal samples.
ABSL_DECLARE_FLAG(bool, rf_shutdown);              //!< Shutdown RF when program exits.

// Declare flags for acquisition blocks
ABSL_DECLARE_FLAG(int32_t, doppler_max);   //!< If defined, maximum Doppler value in the search grid, in Hz (overrides the configuration file).
ABSL_DECLARE_FLAG(int32_t, doppler_step);  //!< If defined, sets the frequency step in the search grid, in Hz, in Hz (overrides the configuration file).

// Declare flags for tracking blocks
ABSL_DECLARE_FLAG(int32_t, cn0_samples);            //!< Number of correlator outputs used for CN0 estimation.
ABSL_DECLARE_FLAG(int32_t, cn0_min);                //!< Minimum valid CN0 (in dB-Hz).
ABSL_DECLARE_FLAG(int32_t, max_lock_fail);          //!< Maximum number of code lock failures before dropping a satellite.
ABSL_DECLARE_FLAG(int32_t, max_carrier_lock_fail);  //!< Maximum number of carrier lock failures before dropping a satellite.
ABSL_DECLARE_FLAG(double, carrier_lock_th);         //!< Carrier lock threshold (in rad).
ABSL_DECLARE_FLAG(double, dll_bw_hz);               //!< Bandwidth of the DLL low pass filter, in Hz (overrides the configuration file).
ABSL_DECLARE_FLAG(double, pll_bw_hz);               //!< Bandwidth of the PLL low pass filter, in Hz (overrides the configuration file).

// Declare flags for observables block
ABSL_DECLARE_FLAG(int32_t, carrier_smoothing_factor);  //!< Sets carrier smoothing factor M (overrides the configuration file).
const int32_t DEFAULT_CARRIER_SMOOTHING_FACTOR = 200;

// Declare flags for PVT
ABSL_DECLARE_FLAG(std::string, RINEX_version);  //!< If defined, specifies the RINEX version (2.11 or 3.02). Overrides the configuration file.
ABSL_DECLARE_FLAG(std::string, RINEX_name);     //!< If defined, specifies the RINEX files base name
ABSL_DECLARE_FLAG(bool, keyboard);              //!< If set to false, disables the keyboard listener. Only for debug purposes (e.g. ASAN mode termination)

static inline void GetTempDirectories(std::vector<std::string>& list)
{
    list.clear();
    // Directories, in order of preference. If we find a dir that
    // exists, we stop adding other less-preferred dirs
    const char* candidates[] = {
        // Non-null only during unittest/regtest
        std::getenv("TEST_TMPDIR"),

        // Explicitly-supplied temp dirs
        std::getenv("TMPDIR"),
        std::getenv("TMP"),

        // If all else fails
        "/tmp",
    };
    for (auto d : candidates)
        {
            if (!d) continue;  // Empty env var

            // Make sure we don't surprise anyone who's expecting a '/'
            std::string dstr = d;
            if (dstr[dstr.size() - 1] != '/')
                {
                    dstr += "/";
                }
            list.push_back(dstr);

            struct stat statbuf;
            if (!stat(d, &statbuf) && S_ISDIR(statbuf.st_mode))
                {
                    // We found a dir that exists - we're done.
                    return;
                }
        }
}


static inline void GetExistingTempDirectories(std::vector<std::string>& list)
{
    GetTempDirectories(list);
    auto i_dir = list.begin();
    while (i_dir != list.end())
        {
            if (access(i_dir->c_str(), 0))
                {
                    i_dir = list.erase(i_dir);
                }
            else
                {
                    ++i_dir;
                }
        };
}


static inline std::string GetTempDir()
{
    std::vector<std::string> temp_directories_list;
    GetExistingTempDirectories(temp_directories_list);

    if (temp_directories_list.empty())
        {
            std::cerr << "No temporary directory found\n";
            exit(EXIT_FAILURE);
        }

    // Use first directory from list of existing temporary directories.
    return temp_directories_list.front();
}

bool ValidateFlags();

#endif

/** \} */
/** \} */
#endif  // GNSS_SDR_GNSS_SDR_FLAGS_H
