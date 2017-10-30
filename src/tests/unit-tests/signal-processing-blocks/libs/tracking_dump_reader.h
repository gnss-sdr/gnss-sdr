/*!
 * \file tracking_dump_reader.h
 * \brief Helper file for unit testing
 * \author Javier Arribas, 2017. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_TRACKING_DUMP_READER_H
#define GNSS_SDR_TRACKING_DUMP_READER_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

class tracking_dump_reader
{
public:
    ~tracking_dump_reader();
    bool read_binary_obs();
    bool restart();
    long int num_epochs();
    bool open_obs_file(std::string out_file);

    //tracking dump variables
    // EPR
    float abs_E;
    float abs_P;
    float abs_L;
    // PROMPT I and Q (to analyze navigation symbols)
    float prompt_I;
    float prompt_Q;
    // PRN start sample stamp
    unsigned long int PRN_start_sample_count;

    // accumulated carrier phase
    double acc_carrier_phase_rad;

    // carrier and code frequency
    double carrier_doppler_hz;
    double code_freq_chips;

    // PLL commands
    double carr_error_hz;
    double carr_error_filt_hz;

    // DLL commands
    double code_error_chips;
    double code_error_filt_chips;

    // CN0 and carrier lock test
    double CN0_SNV_dB_Hz;
    double carrier_lock_test;

    // AUX vars (for debug purposes)
    double aux1;
    double aux2;

    unsigned int PRN;

private:
    std::string d_dump_filename;
    std::ifstream d_dump_file;
};

#endif //GNSS_SDR_TRACKING_DUMP_READER_H
