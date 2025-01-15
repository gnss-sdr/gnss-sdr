/*!
 * \file tracking_true_obs_reader.h
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

#ifndef GNSS_SDR_TRACKING_TRUE_OBS_READER_H
#define GNSS_SDR_TRACKING_TRUE_OBS_READER_H

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

class Tracking_True_Obs_Reader
{
public:
    ~Tracking_True_Obs_Reader();
    bool read_binary_obs();
    bool restart();
    int64_t num_epochs();
    bool open_obs_file(std::string out_file);
    void close_obs_file();
    bool d_dump;

    double signal_timestamp_s;
    double acc_carrier_phase_cycles;
    double doppler_l1_hz;
    double prn_delay_chips;
    double tow;

private:
    std::string d_dump_filename;
    std::ifstream d_dump_file;
};

#endif  // GNSS_SDR_RACKING_TRUE_OBS_READER_H
