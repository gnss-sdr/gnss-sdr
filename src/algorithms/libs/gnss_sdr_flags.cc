/*!
 * \file gnss_sdr_flags.cc
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#include "gnss_sdr_flags.h"
#include <boost/filesystem/operations.hpp>  // for exists
#include <cstdint>
#include <iostream>
#include <string>

DEFINE_string(c, "-", "Path to the configuration file (if set, overrides --config_file).");

DEFINE_string(config_file, std::string(GNSSSDR_INSTALL_DIR "/share/gnss-sdr/conf/default.conf"),
    "Path to the configuration file.");

DEFINE_string(s, "-",
    "If defined, path to the file containing the signal samples (overrides the configuration file and --signal_source).");

DEFINE_string(signal_source, "-",
    "If defined, path to the file containing the signal samples (overrides the configuration file).");

DEFINE_int32(doppler_max, 0, "If defined, sets the maximum Doppler value in the search grid, in Hz (overrides the configuration file).");

DEFINE_int32(doppler_step, 0, "If defined, sets the frequency step in the search grid, in Hz (overrides the configuration file).");

DEFINE_int32(cn0_samples, 20, "Number of correlator outputs used for CN0 estimation.");

DEFINE_int32(cn0_min, 25, "Minimum valid CN0 (in dB-Hz).");

DEFINE_int32(max_lock_fail, 50, "Number number of lock failures before dropping satellite.");

DEFINE_double(carrier_lock_th, 0.85, "Carrier lock threshold (in rad).");

DEFINE_string(RINEX_version, "-", "If defined, specifies the RINEX version (2.11 or 3.02). Overrides the configuration file.");

DEFINE_double(dll_bw_hz, 0.0, "If defined, bandwidth of the DLL low pass filter, in Hz (overrides the configuration file).");

DEFINE_double(pll_bw_hz, 0.0, "If defined, bandwidth of the PLL low pass filter, in Hz (overrides the configuration file).");


#if GFLAGS_GREATER_2_0

static bool ValidateC(const char* flagname, const std::string& value)
{
    if (boost::filesystem::exists(value) or value.compare("-") == 0)  // value is ok
        return true;
    std::cout << "Invalid value for flag -" << flagname << ". The file '" << value << "' does not exist." << std::endl;
    std::cout << "GNSS-SDR program ended." << std::endl;
    return false;
}

static bool ValidateConfigFile(const char* flagname, const std::string& value)
{
    if (boost::filesystem::exists(value) or value.compare(std::string(GNSSSDR_INSTALL_DIR "/share/gnss-sdr/conf/default.conf")) == 0)  // value is ok
        return true;
    std::cout << "Invalid value for flag -" << flagname << ". The file '" << value << "' does not exist." << std::endl;
    std::cout << "GNSS-SDR program ended." << std::endl;
    return false;
}

static bool ValidateS(const char* flagname, const std::string& value)
{
    if (boost::filesystem::exists(value) or value.compare("-") == 0)  // value is ok
        return true;
    std::cout << "Invalid value for flag -" << flagname << ". The file '" << value << "' does not exist." << std::endl;
    std::cout << "GNSS-SDR program ended." << std::endl;
    return false;
}

static bool ValidateSignalSource(const char* flagname, const std::string& value)
{
    if (boost::filesystem::exists(value) or value.compare("-") == 0)  // value is ok
        return true;
    std::cout << "Invalid value for flag -" << flagname << ". The file '" << value << "' does not exist." << std::endl;
    std::cout << "GNSS-SDR program ended." << std::endl;
    return false;
}

static bool ValidateDopplerMax(const char* flagname, int32_t value)
{
    const int32_t max_value = 1000000;
    if (value >= 0 && value < max_value)  // value is ok
        return true;
    std::cout << "Invalid value for flag -" << flagname << ": " << value << ". Allowed range is 0 < " << flagname << " < " << max_value << " Hz." << std::endl;
    std::cout << "GNSS-SDR program ended." << std::endl;
    return false;
}

static bool ValidateDopplerStep(const char* flagname, int32_t value)
{
    const int32_t max_value = 10000;
    if (value >= 0 && value < max_value)  // value is ok
        return true;
    std::cout << "Invalid value for flag -" << flagname << ": " << value << ". Allowed range is 0 < " << flagname << " < " << max_value << " Hz." << std::endl;
    std::cout << "GNSS-SDR program ended." << std::endl;
    return false;
}

static bool ValidateCn0Samples(const char* flagname, int32_t value)
{
    const int32_t max_value = 10000;
    if (value > 0 && value < max_value)  // value is ok
        return true;
    std::cout << "Invalid value for flag -" << flagname << ": " << value << ". Allowed range is 0 < " << flagname << " < " << max_value << " samples." << std::endl;
    std::cout << "GNSS-SDR program ended." << std::endl;
    return false;
}

static bool ValidateCn0Min(const char* flagname, int32_t value)
{
    const int32_t max_value = 100;
    if (value > 0 && value < max_value)  // value is ok
        return true;
    std::cout << "Invalid value for flag -" << flagname << ": " << value << ". Allowed range is 0 < " << flagname << " < " << max_value << " dB-Hz." << std::endl;
    std::cout << "GNSS-SDR program ended." << std::endl;
    return false;
}

static bool ValidateMaxLockFail(const char* flagname, int32_t value)
{
    const int32_t max_value = 10000;
    if (value > 0 && value < max_value)  // value is ok
        return true;
    std::cout << "Invalid value for flag -" << flagname << ": " << value << ". Allowed range is 0 < " << flagname << " < " << max_value << " fails." << std::endl;
    std::cout << "GNSS-SDR program ended." << std::endl;
    return false;
}

static bool ValidateCarrierLockTh(const char* flagname, double value)
{
    const double max_value = 1.508;
    if (value > 0.0 && value < max_value)  // value is ok
        return true;
    std::cout << "Invalid value for flag -" << flagname << ": " << value << ". Allowed range is 0 < " << flagname << " < " << max_value << " rad." << std::endl;
    std::cout << "GNSS-SDR program ended." << std::endl;
    return false;
}

static bool ValidateDllBw(const char* flagname, double value)
{
    const double max_value = 10000.0;
    if (value >= 0.0 && value < max_value)  // value is ok
        return true;
    std::cout << "Invalid value for flag -" << flagname << ": " << value << ". Allowed range is 0 < " << flagname << " < " << max_value << " Hz." << std::endl;
    std::cout << "GNSS-SDR program ended." << std::endl;
    return false;
}

static bool ValidatePllBw(const char* flagname, double value)
{
    const double max_value = 10000.0;
    if (value >= 0.0 && value < max_value)  // value is ok
        return true;
    std::cout << "Invalid value for flag -" << flagname << ": " << value << ". Allowed range is 0 < " << flagname << " < " << max_value << " Hz." << std::endl;
    std::cout << "GNSS-SDR program ended." << std::endl;
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


#endif
