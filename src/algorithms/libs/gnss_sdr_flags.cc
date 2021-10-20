/*!
 * \file gnss_sdr_flags.cc
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


#include "gnss_sdr_flags.h"
#include "gnss_sdr_filesystem.h"
#include <iostream>
#include <string>


DEFINE_string(c, "-", "Path to the configuration file (if set, overrides --config_file).");

DEFINE_string(config_file, std::string(GNSSSDR_INSTALL_DIR "/share/gnss-sdr/conf/default.conf"),
    "Path to the configuration file.");

DEFINE_string(s, "-",
    "If defined, path to the file containing the signal samples (overrides the configuration file and --signal_source).");

DEFINE_string(signal_source, "-",
    "If defined, path to the file containing the signal samples (overrides the configuration file).");

DEFINE_string(timestamp_source, "-",
    "If defined, path to the file containing the signal timestamp data (overrides the configuration file).");

DEFINE_bool(rf_shutdown, true, "If set to false, AD9361 RF channels are not shut down when exiting the program. Useful to leave the AD9361 configured and running.");

DEFINE_int32(doppler_max, 0, "If defined, sets the maximum Doppler value in the search grid, in Hz (overrides the configuration file).");

DEFINE_int32(doppler_step, 0, "If defined, sets the frequency step in the search grid, in Hz (overrides the configuration file).");

DEFINE_int32(cn0_samples, 20, "Number of correlator outputs used for CN0 estimation.");

DEFINE_int32(cn0_min, 25, "Minimum valid CN0 (in dB-Hz).");

DEFINE_int32(max_carrier_lock_fail, 5000, "Maximum number of carrier lock failures before dropping a satellite.");

DEFINE_int32(max_lock_fail, 50, "Maximum number of code lock failures before dropping a satellite.");

// cos(2xError_angle)=0.7 -> Error_angle=22 deg
DEFINE_double(carrier_lock_th, 0.7, "Carrier lock threshold (in rad).");

DEFINE_double(dll_bw_hz, 0.0, "If defined, bandwidth of the DLL low pass filter, in Hz (overrides the configuration file).");

DEFINE_double(pll_bw_hz, 0.0, "If defined, bandwidth of the PLL low pass filter, in Hz (overrides the configuration file).");

DEFINE_int32(carrier_smoothing_factor, DEFAULT_CARRIER_SMOOTHING_FACTOR, "Sets carrier smoothing factor M (overrides the configuration file)");

DEFINE_string(RINEX_version, "-", "If defined, specifies the RINEX version (2.11 or 3.02). Overrides the configuration file.");

DEFINE_string(RINEX_name, "-", "If defined, specifies the RINEX files base name");

DEFINE_bool(keyboard, true, "If set to false, it disables the keyboard listener (so the receiver cannot be stopped with q+[Enter])");

#if GFLAGS_GREATER_2_0

static bool ValidateC(const char* flagname, const std::string& value)
{
    if (fs::exists(value) or value == "-")
        {  // value is ok
            return true;
        }
    std::cout << "Invalid value for flag -" << flagname << ". The file '" << value << "' does not exist.\n";
    std::cout << "GNSS-SDR program ended.\n";
    return false;
}

static bool ValidateConfigFile(const char* flagname, const std::string& value)
{
    if (fs::exists(value) or value == std::string(GNSSSDR_INSTALL_DIR "/share/gnss-sdr/conf/default.conf"))
        {  // value is ok
            return true;
        }
    std::cout << "Invalid value for flag -" << flagname << ". The file '" << value << "' does not exist.\n";
    std::cout << "GNSS-SDR program ended.\n";
    return false;
}

static bool ValidateS(const char* flagname, const std::string& value)
{
    if (fs::exists(value) or value == "-")
        {  // value is ok
            return true;
        }
    std::cout << "Invalid value for flag -" << flagname << ". The file '" << value << "' does not exist.\n";
    std::cout << "GNSS-SDR program ended.\n";
    return false;
}

static bool ValidateSignalSource(const char* flagname, const std::string& value)
{
    if (fs::exists(value) or value == "-")
        {  // value is ok
            return true;
        }
    std::cout << "Invalid value for flag -" << flagname << ". The file '" << value << "' does not exist.\n";
    std::cout << "GNSS-SDR program ended.\n";
    return false;
}

static bool ValidateDopplerMax(const char* flagname, int32_t value)
{
    const int32_t max_value = 1000000;
    if (value >= 0 && value < max_value)
        {  // value is ok
            return true;
        }
    std::cout << "Invalid value for flag -" << flagname << ": " << value << ". Allowed range is 0 < " << flagname << " < " << max_value << " Hz.\n";
    std::cout << "GNSS-SDR program ended.\n";
    return false;
}

static bool ValidateDopplerStep(const char* flagname, int32_t value)
{
    const int32_t max_value = 10000;
    if (value >= 0 && value < max_value)
        {  // value is ok
            return true;
        }
    std::cout << "Invalid value for flag -" << flagname << ": " << value << ". Allowed range is 0 < " << flagname << " < " << max_value << " Hz.\n";
    std::cout << "GNSS-SDR program ended.\n";
    return false;
}

static bool ValidateCn0Samples(const char* flagname, int32_t value)
{
    const int32_t max_value = 10000;
    if (value > 0 && value < max_value)
        {  // value is ok
            return true;
        }
    std::cout << "Invalid value for flag -" << flagname << ": " << value << ". Allowed range is 0 < " << flagname << " < " << max_value << " samples.\n";
    std::cout << "GNSS-SDR program ended.\n";
    return false;
}

static bool ValidateCn0Min(const char* flagname, int32_t value)
{
    const int32_t max_value = 100;
    if (value > 0 && value < max_value)
        {  // value is ok
            return true;
        }
    std::cout << "Invalid value for flag -" << flagname << ": " << value << ". Allowed range is 0 < " << flagname << " < " << max_value << " dB-Hz.\n";
    std::cout << "GNSS-SDR program ended.\n";
    return false;
}

static bool ValidateMaxLockFail(const char* flagname, int32_t value)
{
    const int32_t max_value = 10000;
    if (value > 0 && value < max_value)
        {  // value is ok
            return true;
        }
    std::cout << "Invalid value for flag -" << flagname << ": " << value << ". Allowed range is 0 < " << flagname << " < " << max_value << " fails.\n";
    std::cout << "GNSS-SDR program ended.\n";
    return false;
}

static bool ValidateCarrierLockTh(const char* flagname, double value)
{
    const double max_value = 1.508;
    if (value > 0.0 && value < max_value)
        {  // value is ok
            return true;
        }
    std::cout << "Invalid value for flag -" << flagname << ": " << value << ". Allowed range is 0 < " << flagname << " < " << max_value << " rad.\n";
    std::cout << "GNSS-SDR program ended.\n";
    return false;
}

static bool ValidateDllBw(const char* flagname, double value)
{
    const double max_value = 10000.0;
    if (value >= 0.0 && value < max_value)
        {  // value is ok
            return true;
        }
    std::cout << "Invalid value for flag -" << flagname << ": " << value << ". Allowed range is 0 < " << flagname << " < " << max_value << " Hz.\n";
    std::cout << "GNSS-SDR program ended.\n";
    return false;
}

static bool ValidatePllBw(const char* flagname, double value)
{
    const double max_value = 10000.0;
    if (value >= 0.0 && value < max_value)
        {  // value is ok
            return true;
        }
    std::cout << "Invalid value for flag -" << flagname << ": " << value << ". Allowed range is 0 < " << flagname << " < " << max_value << " Hz.\n";
    std::cout << "GNSS-SDR program ended.\n";
    return false;
}

static bool ValidateCarrierSmoothingFactor(const char* flagname, int32_t value)
{
    const int32_t min_value = 1;
    if (value >= min_value)
        {  // value is ok
            return true;
        }
    std::cout << "Invalid value for flag -" << flagname << ": " << value << ". Allowed range is 1 <= " << flagname << ".\n";
    std::cout << "GNSS-SDR program ended.\n";
    return false;
}


DEFINE_validator(c, &ValidateC);
DEFINE_validator(config_file, &ValidateConfigFile);
DEFINE_validator(s, &ValidateS);
DEFINE_validator(signal_source, &ValidateSignalSource);
DEFINE_validator(doppler_max, &ValidateDopplerMax);
DEFINE_validator(doppler_step, &ValidateDopplerStep);
DEFINE_validator(cn0_samples, &ValidateCn0Samples);
DEFINE_validator(cn0_min, &ValidateCn0Min);
DEFINE_validator(max_lock_fail, &ValidateMaxLockFail);
DEFINE_validator(carrier_lock_th, &ValidateCarrierLockTh);
DEFINE_validator(dll_bw_hz, &ValidateDllBw);
DEFINE_validator(pll_bw_hz, &ValidatePllBw);
DEFINE_validator(carrier_smoothing_factor, &ValidateCarrierSmoothingFactor);

#endif
