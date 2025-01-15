/*!
 * \file observables_dump_reader.h
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

#ifndef GNSS_SDR_OBSERVABLES_DUMP_READER_H
#define GNSS_SDR_OBSERVABLES_DUMP_READER_H

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

class Observables_Dump_Reader
{
public:
    explicit Observables_Dump_Reader(int n_channels);
    ~Observables_Dump_Reader();
    bool read_binary_obs();
    bool restart();
    int64_t num_epochs();
    bool open_obs_file(std::string out_file);
    void close_obs_file();

    // dump variables
    std::vector<double> RX_time;
    std::vector<double> TOW_at_current_symbol_s;
    std::vector<double> Carrier_Doppler_hz;
    std::vector<double> Acc_carrier_phase_hz;
    std::vector<double> Pseudorange_m;
    std::vector<double> PRN;
    std::vector<double> valid;

private:
    int n_channels;
    std::string d_dump_filename;
    std::ifstream d_dump_file;
};

#endif  // GNSS_SDR_OBSERVABLES_DUMP_READER_H
