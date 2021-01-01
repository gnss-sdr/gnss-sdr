/*!
 * \file tlm_dump_reader.h
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

#ifndef GNSS_SDR_TLM_DUMP_READER_H
#define GNSS_SDR_TLM_DUMP_READER_H

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

class Tlm_Dump_Reader
{
public:
    ~Tlm_Dump_Reader();
    bool read_binary_obs();
    bool restart();
    int64_t num_epochs();
    bool open_obs_file(std::string out_file);

    // telemetry decoder dump variables
    double TOW_at_current_symbol;
    uint64_t Tracking_sample_counter;
    double d_TOW_at_Preamble;
    int32_t nav_symbol;
    int32_t prn;

private:
    std::string d_dump_filename;
    std::ifstream d_dump_file;
};

#endif  // GNSS_SDR_TLM_DUMP_READER_H
