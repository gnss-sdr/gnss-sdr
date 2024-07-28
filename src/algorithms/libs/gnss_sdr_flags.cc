/*!
 * \file gnss_sdr_flags.cc
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


#include "gnss_sdr_flags.h"
#include "gnss_sdr_filesystem.h"
#include <iostream>

#if USE_GLOG_AND_GFLAGS
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
    if (fs::exists(value) || value == "-")
        {  // value is ok
            return true;
        }
    std::cout << "Invalid value for flag -" << flagname << ". The file '" << value << "' does not exist.\n";
    std::cout << "GNSS-SDR program ended.\n";
    return false;
}

static bool ValidateConfigFile(const char* flagname, const std::string& value)
{
    if (fs::exists(value) || value == std::string(GNSSSDR_INSTALL_DIR "/share/gnss-sdr/conf/default.conf"))
        {  // value is ok
            return true;
        }
    std::cout << "Invalid value for flag -" << flagname << ". The file '" << value << "' does not exist.\n";
    std::cout << "GNSS-SDR program ended.\n";
    return false;
}

static bool ValidateS(const char* flagname, const std::string& value)
{
    if (fs::exists(value) || value == "-")
        {  // value is ok
            return true;
        }
    std::cout << "Invalid value for flag -" << flagname << ". The file '" << value << "' does not exist.\n";
    std::cout << "GNSS-SDR program ended.\n";
    return false;
}

static bool ValidateSignalSource(const char* flagname, const std::string& value)
{
    if (fs::exists(value) || value == "-")
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

#else
ABSL_FLAG(std::string, c, "-", "Path to the configuration file (if set, overrides --config_file).");
ABSL_FLAG(std::string, config_file, std::string(GNSSSDR_INSTALL_DIR "/share/gnss-sdr/conf/default.conf"), "Path to the configuration file.");
ABSL_FLAG(std::string, log_dir, "", "Directory for log files");
ABSL_FLAG(std::string, s, "-", "If defined, path to the file containing the signal samples (overrides the configuration file and --signal_source).");
ABSL_FLAG(std::string, signal_source, "-", "If defined, path to the file containing the signal samples (overrides the configuration file).");
ABSL_FLAG(std::string, timestamp_source, "-", "If defined, path to the file containing the signal timestamp data (overrides the configuration file).");
ABSL_FLAG(bool, rf_shutdown, true, "If set to false, AD9361 RF channels are not shut down when exiting the program. Useful to leave the AD9361 configured and running.");
ABSL_FLAG(int32_t, doppler_max, 0, "If defined, sets the maximum Doppler value in the search grid, in Hz (overrides the configuration file).");
ABSL_FLAG(int32_t, doppler_step, 0, "If defined, sets the frequency step in the search grid, in Hz (overrides the configuration file).");
ABSL_FLAG(int32_t, cn0_samples, 20, "Number of correlator outputs used for CN0 estimation.");
ABSL_FLAG(int32_t, cn0_min, 25, "Minimum valid CN0 (in dB-Hz).");
ABSL_FLAG(int32_t, max_carrier_lock_fail, 5000, "Maximum number of carrier lock failures before dropping a satellite.");
ABSL_FLAG(int32_t, max_lock_fail, 50, "Maximum number of code lock failures before dropping a satellite.");
ABSL_FLAG(double, carrier_lock_th, 0.7, "Carrier lock threshold (in rad).");
ABSL_FLAG(double, dll_bw_hz, 0.0, "If defined, bandwidth of the DLL low pass filter, in Hz (overrides the configuration file).");
ABSL_FLAG(double, pll_bw_hz, 0.0, "If defined, bandwidth of the PLL low pass filter, in Hz (overrides the configuration file).");
ABSL_FLAG(int32_t, carrier_smoothing_factor, DEFAULT_CARRIER_SMOOTHING_FACTOR, "Sets carrier smoothing factor M (overrides the configuration file)");
ABSL_FLAG(std::string, RINEX_version, "-", "If defined, specifies the RINEX version (2.11 or 3.02). Overrides the configuration file.");
ABSL_FLAG(std::string, RINEX_name, "-", "If defined, specifies the RINEX files base name");
ABSL_FLAG(bool, keyboard, true, "If set to false, it disables the keyboard listener (so the receiver cannot be stopped with q+[Enter])");

bool ValidateFlags()
{
    bool success = true;

    auto value_c = absl::GetFlag(FLAGS_c);
    if (!(fs::exists(value_c) || value_c == "-"))
        {
            std::cerr << "Invalid value for flag -c. The file '" << value_c << "' does not exist.\n";
            success = false;
        }

    auto value_config_file = absl::GetFlag(FLAGS_config_file);
    if (!(fs::exists(value_config_file) || value_config_file == std::string(GNSSSDR_INSTALL_DIR "/share/gnss-sdr/conf/default.conf")))
        {
            std::cerr << "Invalid value for flag -config_file. The file '" << value_config_file << "' does not exist.\n";
            success = false;
        }

    auto value_s = absl::GetFlag(FLAGS_s);
    if (!(fs::exists(value_s) || value_s == "-"))
        {
            std::cerr << "Invalid value for flag -s. The file '" << value_s << "' does not exist.\n";
            success = false;
        }

    auto value_signal_source = absl::GetFlag(FLAGS_signal_source);
    if (!(fs::exists(value_signal_source) || value_signal_source == "-"))
        {
            std::cerr << "Invalid value for flag -signal_source. The file '" << value_signal_source << "' does not exist.\n";
            success = false;
        }

    auto value_doppler_max = absl::GetFlag(FLAGS_doppler_max);
    const int32_t max_doppler_value = 1000000;
    if (value_doppler_max < 0 || value_doppler_max > max_doppler_value)
        {
            std::cerr << "Invalid value for flag -doppler_max. Allowed range is 0 < doppler_max < " << max_doppler_value << " Hz.\n";
            success = false;
        }

    auto value_doppler_step = absl::GetFlag(FLAGS_doppler_step);
    const int32_t max_value_doppler_step = 10000;
    if (value_doppler_step < 0 || value_doppler_step > max_value_doppler_step)
        {
            std::cerr << "Invalid value for flag -doppler_step: " << value_doppler_step << ". Allowed range is 0 < doppler_step < " << max_value_doppler_step << " Hz.\n";
            success = false;
        }

    auto value_cn0_samples = absl::GetFlag(FLAGS_cn0_samples);
    const int32_t max_value_cn0_samples = 10000;
    if (value_cn0_samples < 0 || value_cn0_samples > max_value_cn0_samples)
        {
            std::cerr << "Invalid value for flag -cn0_samples: " << value_cn0_samples << ". Allowed range is 0 < cn0_samples < " << max_value_cn0_samples << " Hz.\n";
            success = false;
        }

    auto value_cn0_min = absl::GetFlag(FLAGS_cn0_min);
    const int32_t max_value_cn0_min = 100;
    if (value_cn0_min < 0 || value_cn0_min > max_value_cn0_min)
        {
            std::cerr << "Invalid value for flag -cn0_min: " << value_cn0_min << ". Allowed range is 0 < cn0_min < " << max_value_cn0_min << " Hz.\n";
            success = false;
        }

    auto value_max_lock_fail = absl::GetFlag(FLAGS_max_lock_fail);
    const int32_t max_value_max_lock_fail = 10000;
    if (value_max_lock_fail < 0 || value_max_lock_fail > max_value_max_lock_fail)
        {
            std::cerr << "Invalid value for flag -max_lock_fail: " << value_max_lock_fail << ". Allowed range is 0 < max_lock_fail < " << max_value_max_lock_fail << " Hz.\n";
            success = false;
        }

    auto value_carrier_lock_th = absl::GetFlag(FLAGS_carrier_lock_th);
    const double max_value_carrier_lock_th = 1.508;
    if (value_carrier_lock_th < 0.0 || value_carrier_lock_th > max_value_carrier_lock_th)
        {
            std::cerr << "Invalid value for flag -carrier_lock_th: " << value_carrier_lock_th << ". Allowed range is 0 < carrier_lock_th < " << max_value_carrier_lock_th << " Hz.\n";
            success = false;
        }

    auto value_dll_bw_hz = absl::GetFlag(FLAGS_dll_bw_hz);
    const double max_value_dll_bw_hz = 10000.0;
    if (value_dll_bw_hz < 0.0 || value_dll_bw_hz > max_value_dll_bw_hz)
        {
            std::cerr << "Invalid value for flag -dll_bw_hz: " << value_dll_bw_hz << ". Allowed range is 0 < dll_bw_hz < " << max_value_dll_bw_hz << " Hz.\n";
            success = false;
        }

    auto value_pll_bw_hz = absl::GetFlag(FLAGS_pll_bw_hz);
    const double max_value_pll_bw_hz = 10000.0;
    if (value_pll_bw_hz < 0.0 || value_pll_bw_hz > max_value_pll_bw_hz)
        {
            std::cerr << "Invalid value for flag -pll_bw_hz: " << value_pll_bw_hz << ". Allowed range is 0 < pll_bw_hz < " << max_value_pll_bw_hz << " Hz.\n";
            success = false;
        }

    auto value_carrier_smoothing_factor = absl::GetFlag(FLAGS_carrier_smoothing_factor);
    const int32_t min_value_carrier_smoothing_factor = 1;
    if (value_carrier_smoothing_factor < min_value_carrier_smoothing_factor)
        {
            std::cerr << "Invalid value for flag -carrier_smoothing_factor: " << value_carrier_smoothing_factor << ". Allowed range is 1 <= carrier_smoothing_factor.\n";
            success = false;
        }

    return success;
}

#endif
