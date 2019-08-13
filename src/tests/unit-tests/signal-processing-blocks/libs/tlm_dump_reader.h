/*!
 * \file tlm_dump_reader.h
 * \brief Helper file for unit testing
 * \author Javier Arribas, 2017. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
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

    //telemetry decoder dump variables
    double TOW_at_current_symbol;
    uint64_t Tracking_sample_counter;
    double d_TOW_at_Preamble;

private:
    std::string d_dump_filename;
    std::ifstream d_dump_file;
};

#endif  //GNSS_SDR_TLM_DUMP_READER_H
