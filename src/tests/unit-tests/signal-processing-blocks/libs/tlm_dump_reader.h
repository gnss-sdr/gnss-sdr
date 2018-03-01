/*!
 * \file tlm_dump_reader.h
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

#ifndef GNSS_SDR_TLM_DUMP_READER_H
#define GNSS_SDR_TLM_DUMP_READER_H

#include <fstream>
#include <string>
#include <vector>

class tlm_dump_reader
{
public:
    ~tlm_dump_reader();
    bool read_binary_obs();
    bool restart();
    long int num_epochs();
    bool open_obs_file(std::string out_file);

    //telemetry decoder dump variables
    double TOW_at_current_symbol;
    unsigned long int Tracking_sample_counter;
    double d_TOW_at_Preamble;

private:
    std::string d_dump_filename;
    std::ifstream d_dump_file;
};

#endif //GNSS_SDR_TLM_DUMP_READER_H
