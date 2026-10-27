/*!
 * \file tracking_dump_reader.h
 * \brief Helper file for unit testing
 * \author Javier Arribas, 2017. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_TRACKING_DUMP_READER_H
#define GNSS_SDR_TRACKING_DUMP_READER_H

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

/*!
 * \brief Record layout of a tracking dump file, which depends on the block
 * that wrote it.
 */
enum class Tracking_Dump_Layout
{
    DLL_PLL_VEML,       //!< dll_pll_veml_tracking: ends with PRN, TOW and WN (112 bytes)
    DLL_PLL_VEML_FPGA,  //!< dll_pll_veml_tracking_fpga and kf_tracking: ends with PRN (100 bytes)
    GPS_L1_CA_GAUSSIAN  //!< gps_l1_ca_gaussian_tracking_cc: no code frequency rate, ends with PRN (100 bytes)
};

class Tracking_Dump_Reader
{
public:
    explicit Tracking_Dump_Reader(Tracking_Dump_Layout layout = Tracking_Dump_Layout::DLL_PLL_VEML);
    ~Tracking_Dump_Reader();
    bool read_binary_obs();
    bool restart();
    int64_t num_epochs();
    bool open_obs_file(std::string out_file);

    // tracking dump variables
    // VEPLVL
    float abs_VE;
    float abs_E;
    float abs_P;
    float abs_L;
    float abs_VL;
    // PROMPT I and Q (to analyze navigation symbols)
    float prompt_I;
    float prompt_Q;
    // PRN start sample stamp
    uint64_t PRN_start_sample_count;

    // accumulated carrier phase
    double acc_carrier_phase_rad;

    // carrier and code frequency
    float carrier_doppler_hz;
    float carrier_doppler_rate_hz_s;
    float code_freq_chips;
    float code_freq_rate_chips{};  // not in GPS_L1_CA_GAUSSIAN dumps

    // PLL commands
    float carr_error_hz;
    float carr_noise_sigma2{};  // only in GPS_L1_CA_GAUSSIAN dumps
    float carr_error_filt_hz;

    // DLL commands
    float code_error_chips;
    float code_error_filt_chips;

    // CN0 and carrier lock test
    float CN0_SNV_dB_Hz;
    float carrier_lock_test;

    // AUX vars (for debug purposes)
    float aux1;
    double aux2;

    unsigned int PRN;
    uint64_t TOW_ms{};  // only in DLL_PLL_VEML dumps
    int32_t WN{};       // only in DLL_PLL_VEML dumps

private:
    std::string d_dump_filename;
    std::ifstream d_dump_file;
    Tracking_Dump_Layout d_layout;
};

#endif  // GNSS_SDR_TRACKING_DUMP_READER_H
