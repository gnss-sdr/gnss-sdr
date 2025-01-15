/*!
 * \file tlm_dump_reader.cc
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

#include "tlm_dump_reader.h"
#include <exception>
#include <iostream>
#include <utility>

bool Tlm_Dump_Reader::read_binary_obs()
{
    try
        {
            d_dump_file.read(reinterpret_cast<char *>(&TOW_at_current_symbol), sizeof(double));
            d_dump_file.read(reinterpret_cast<char *>(&Tracking_sample_counter), sizeof(uint64_t));
            d_dump_file.read(reinterpret_cast<char *>(&d_TOW_at_Preamble), sizeof(double));
            d_dump_file.read(reinterpret_cast<char *>(&nav_symbol), sizeof(int32_t));
            d_dump_file.read(reinterpret_cast<char *>(&prn), sizeof(int32_t));
        }
    catch (const std::ifstream::failure &e)
        {
            return false;
        }
    return true;
}


bool Tlm_Dump_Reader::restart()
{
    if (d_dump_file.is_open())
        {
            d_dump_file.clear();
            d_dump_file.seekg(0, std::ios::beg);
            return true;
        }
    return false;
}


int64_t Tlm_Dump_Reader::num_epochs()
{
    std::ifstream::pos_type size;
    int number_of_double_vars_in_epoch = 2;
    int number_of_int_vars_in_epoch = 2;
    int epoch_size_bytes = sizeof(double) * number_of_double_vars_in_epoch + sizeof(uint64_t) + sizeof(int32_t) * number_of_int_vars_in_epoch;
    std::ifstream tmpfile(d_dump_filename.c_str(), std::ios::binary | std::ios::ate);
    if (tmpfile.is_open())
        {
            size = tmpfile.tellg();
            int64_t nepoch = size / epoch_size_bytes;
            return nepoch;
        }
    return 0;
}


bool Tlm_Dump_Reader::open_obs_file(std::string out_file)
{
    if (d_dump_file.is_open() == false)
        {
            try
                {
                    d_dump_filename = std::move(out_file);
                    d_dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
                    d_dump_file.open(d_dump_filename.c_str(), std::ios::in | std::ios::binary);
                    std::cout << "TLM dump enabled, Log file: " << d_dump_filename.c_str() << '\n';
                    return true;
                }
            catch (const std::ifstream::failure &e)
                {
                    std::cout << "Problem opening TLM dump Log file: " << d_dump_filename << '\n';
                    return false;
                }
        }
    else
        {
            return false;
        }
}


Tlm_Dump_Reader::~Tlm_Dump_Reader()
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
            std::cerr << "Problem closing TLM dump Log file: " << d_dump_filename << '\n';
        }
    catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
}
