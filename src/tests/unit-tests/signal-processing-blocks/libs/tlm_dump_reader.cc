/*!
 * \file tlm_dump_reader.cc
 * \brief Helper file for unit testing
 * \author Javier Arribas, 2017. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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

#include "tlm_dump_reader.h"
#include <iostream>

bool tlm_dump_reader::read_binary_obs()
{
    try
        {
            d_dump_file.read(reinterpret_cast<char *>(&TOW_at_current_symbol), sizeof(double));
            d_dump_file.read(reinterpret_cast<char *>(&Tracking_sample_counter), sizeof(uint64_t));
            d_dump_file.read(reinterpret_cast<char *>(&d_TOW_at_Preamble), sizeof(double));
        }
    catch (const std::ifstream::failure &e)
        {
            return false;
        }
    return true;
}


bool tlm_dump_reader::restart()
{
    if (d_dump_file.is_open())
        {
            d_dump_file.clear();
            d_dump_file.seekg(0, std::ios::beg);
            return true;
        }
    else
        {
            return false;
        }
}


int64_t tlm_dump_reader::num_epochs()
{
    std::ifstream::pos_type size;
    int number_of_vars_in_epoch = 2;
    int epoch_size_bytes = sizeof(double) * number_of_vars_in_epoch + sizeof(uint64_t);
    std::ifstream tmpfile(d_dump_filename.c_str(), std::ios::binary | std::ios::ate);
    if (tmpfile.is_open())
        {
            size = tmpfile.tellg();
            int64_t nepoch = size / epoch_size_bytes;
            return nepoch;
        }
    else
        {
            return 0;
        }
}


bool tlm_dump_reader::open_obs_file(std::string out_file)
{
    if (d_dump_file.is_open() == false)
        {
            try
                {
                    d_dump_filename = out_file;
                    d_dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
                    d_dump_file.open(d_dump_filename.c_str(), std::ios::in | std::ios::binary);
                    std::cout << "TLM dump enabled, Log file: " << d_dump_filename.c_str() << std::endl;
                    return true;
                }
            catch (const std::ifstream::failure &e)
                {
                    std::cout << "Problem opening TLM dump Log file: " << d_dump_filename.c_str() << std::endl;
                    return false;
                }
        }
    else
        {
            return false;
        }
}


tlm_dump_reader::~tlm_dump_reader()
{
    if (d_dump_file.is_open() == true)
        {
            d_dump_file.close();
        }
}
