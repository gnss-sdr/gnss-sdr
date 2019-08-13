/*!
 * \file true_observables_reader.cc
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

#include "true_observables_reader.h"
#include <exception>
#include <iostream>
#include <utility>

bool True_Observables_Reader::read_binary_obs()
{
    try
        {
            for (int i = 0; i < 12; i++)
                {
                    d_dump_file.read(reinterpret_cast<char *>(&gps_time_sec[i]), sizeof(double));
                    d_dump_file.read(reinterpret_cast<char *>(&doppler_l1_hz[i]), sizeof(double));
                    d_dump_file.read(reinterpret_cast<char *>(&acc_carrier_phase_l1_cycles[i]), sizeof(double));
                    d_dump_file.read(reinterpret_cast<char *>(&dist_m[i]), sizeof(double));
                    d_dump_file.read(reinterpret_cast<char *>(&true_dist_m[i]), sizeof(double));
                    d_dump_file.read(reinterpret_cast<char *>(&carrier_phase_l1_cycles[i]), sizeof(double));
                    d_dump_file.read(reinterpret_cast<char *>(&prn[i]), sizeof(double));
                }
        }
    catch (const std::ifstream::failure &e)
        {
            return false;
        }
    return true;
}


bool True_Observables_Reader::restart()
{
    if (d_dump_file.is_open())
        {
            d_dump_file.clear();
            d_dump_file.seekg(0, std::ios::beg);
            return true;
        }
    return false;
}


int64_t True_Observables_Reader::num_epochs()
{
    std::ifstream::pos_type size;
    int number_of_vars_in_epoch = 6 * 12;
    int epoch_size_bytes = sizeof(double) * number_of_vars_in_epoch;
    std::ifstream tmpfile(d_dump_filename.c_str(), std::ios::binary | std::ios::ate);
    if (tmpfile.is_open())
        {
            size = tmpfile.tellg();
            int64_t nepoch = size / epoch_size_bytes;
            return nepoch;
        }
    return 0;
}


bool True_Observables_Reader::open_obs_file(std::string out_file)
{
    if (d_dump_file.is_open() == false)
        {
            try
                {
                    d_dump_filename = std::move(out_file);
                    d_dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
                    d_dump_file.open(d_dump_filename.c_str(), std::ios::in | std::ios::binary);
                    std::cout << "True observables Log file opened: " << d_dump_filename.c_str() << std::endl;
                    return true;
                }
            catch (const std::ifstream::failure &e)
                {
                    std::cout << "Problem opening true Observables Log file: " << d_dump_filename << std::endl;
                    return false;
                }
        }
    else
        {
            return false;
        }
}


True_Observables_Reader::~True_Observables_Reader()
{
    try
        {
            if (d_dump_file.is_open() == true)
                {
                    d_dump_file.close();
                }
        }
    catch (const std::ifstream::failure &e)
        {
            std::cerr << "Problem closing true Observables dump Log file: " << d_dump_filename << '\n';
        }
    catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
}
