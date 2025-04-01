/*!
 * \file true_observables_reader.h
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

#ifndef GNSS_SDR_TRUE_OBSERVABLES_READER_H
#define GNSS_SDR_TRUE_OBSERVABLES_READER_H

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

class True_Observables_Reader
{
public:
    ~True_Observables_Reader();
    bool read_binary_obs();
    bool restart();
    int64_t num_epochs();
    bool open_obs_file(std::string out_file);

    double gps_time_sec[12];
    double doppler_l1_hz[12];
    double acc_carrier_phase_l1_cycles[12];
    double dist_m[12];
    double true_dist_m[12];
    double carrier_phase_l1_cycles[12];
    double prn[12];

private:
    std::string d_dump_filename;
    std::ifstream d_dump_file;
};

#endif  // GNSS_SDR_TRUE_OBSERVABLES_READER_H
