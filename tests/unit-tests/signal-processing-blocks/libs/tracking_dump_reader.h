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

class Tracking_Dump_Reader
{
public:
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
    float acc_carrier_phase_rad;

    // carrier and code frequency
    float carrier_doppler_hz;
    float carrier_doppler_rate_hz_s;
    float code_freq_chips;
    float code_freq_rate_chips;

    // PLL commands
    float carr_error_hz;
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

private:
    std::string d_dump_filename;
    std::ifstream d_dump_file;
};

#endif  // GNSS_SDR_TRACKING_DUMP_READER_H
