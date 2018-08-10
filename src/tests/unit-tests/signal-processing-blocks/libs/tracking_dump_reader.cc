/*!
 * \file tracking_dump_reader.cc
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

#include "tracking_dump_reader.h"
#include <iostream>

bool tracking_dump_reader::read_binary_obs()
{
    try
        {
            d_dump_file.read(reinterpret_cast<char *>(&abs_VE), sizeof(float));
            d_dump_file.read(reinterpret_cast<char *>(&abs_E), sizeof(float));
            d_dump_file.read(reinterpret_cast<char *>(&abs_P), sizeof(float));
            d_dump_file.read(reinterpret_cast<char *>(&abs_L), sizeof(float));
            d_dump_file.read(reinterpret_cast<char *>(&abs_VL), sizeof(float));
            d_dump_file.read(reinterpret_cast<char *>(&prompt_I), sizeof(float));
            d_dump_file.read(reinterpret_cast<char *>(&prompt_Q), sizeof(float));
            d_dump_file.read(reinterpret_cast<char *>(&PRN_start_sample_count), sizeof(uint64_t));
            d_dump_file.read(reinterpret_cast<char *>(&acc_carrier_phase_rad), sizeof(float));
            d_dump_file.read(reinterpret_cast<char *>(&carrier_doppler_hz), sizeof(float));
            d_dump_file.read(reinterpret_cast<char *>(&code_freq_chips), sizeof(float));
            d_dump_file.read(reinterpret_cast<char *>(&carr_error_hz), sizeof(float));
            d_dump_file.read(reinterpret_cast<char *>(&carr_error_filt_hz), sizeof(float));
            d_dump_file.read(reinterpret_cast<char *>(&code_error_chips), sizeof(float));
            d_dump_file.read(reinterpret_cast<char *>(&code_error_filt_chips), sizeof(float));
            d_dump_file.read(reinterpret_cast<char *>(&CN0_SNV_dB_Hz), sizeof(float));
            d_dump_file.read(reinterpret_cast<char *>(&carrier_lock_test), sizeof(float));
            d_dump_file.read(reinterpret_cast<char *>(&aux1), sizeof(float));
            d_dump_file.read(reinterpret_cast<char *>(&aux2), sizeof(double));
            d_dump_file.read(reinterpret_cast<char *>(&PRN), sizeof(unsigned int));
        }
    catch (const std::ifstream::failure &e)
        {
            return false;
        }
    return true;
}


bool tracking_dump_reader::restart()
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


int64_t tracking_dump_reader::num_epochs()
{
    std::ifstream::pos_type size;
    int number_of_double_vars = 1;
    int number_of_float_vars = 17;
    int epoch_size_bytes = sizeof(uint64_t) + sizeof(double) * number_of_double_vars +
                           sizeof(float) * number_of_float_vars + sizeof(unsigned int);
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


bool tracking_dump_reader::open_obs_file(std::string out_file)
{
    if (d_dump_file.is_open() == false)
        {
            try
                {
                    d_dump_filename = out_file;
                    d_dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
                    d_dump_file.open(d_dump_filename.c_str(), std::ios::in | std::ios::binary);
                    return true;
                }
            catch (const std::ifstream::failure &e)
                {
                    std::cout << "Problem opening Tracking dump Log file: " << d_dump_filename.c_str() << std::endl;
                    return false;
                }
        }
    else
        {
            return false;
        }
}


tracking_dump_reader::~tracking_dump_reader()
{
    if (d_dump_file.is_open() == true)
        {
            d_dump_file.close();
        }
}
