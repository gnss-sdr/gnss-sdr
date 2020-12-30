/*!
 * \file tracking_dump_reader.cc
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

#include "tracking_dump_reader.h"
#include <exception>
#include <iostream>
#include <utility>

bool Tracking_Dump_Reader::read_binary_obs()
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
            d_dump_file.read(reinterpret_cast<char *>(&carrier_doppler_rate_hz_s), sizeof(float));
            d_dump_file.read(reinterpret_cast<char *>(&code_freq_chips), sizeof(float));
            d_dump_file.read(reinterpret_cast<char *>(&code_freq_rate_chips), sizeof(float));
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


bool Tracking_Dump_Reader::restart()
{
    if (d_dump_file.is_open())
        {
            d_dump_file.clear();
            d_dump_file.seekg(0, std::ios::beg);
            return true;
        }
    return false;
}


int64_t Tracking_Dump_Reader::num_epochs()
{
    std::ifstream::pos_type size;
    int number_of_double_vars = 1;
    int number_of_float_vars = 19;
    int epoch_size_bytes = sizeof(uint64_t) + sizeof(double) * number_of_double_vars +
                           sizeof(float) * number_of_float_vars + sizeof(unsigned int);
    std::ifstream tmpfile(d_dump_filename.c_str(), std::ios::binary | std::ios::ate);
    if (tmpfile.is_open())
        {
            size = tmpfile.tellg();
            int64_t nepoch = size / epoch_size_bytes;
            return nepoch;
        }


    return 0;
}


bool Tracking_Dump_Reader::open_obs_file(std::string out_file)
{
    if (d_dump_file.is_open() == false)
        {
            try
                {
                    d_dump_filename = std::move(out_file);
                    d_dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
                    d_dump_file.open(d_dump_filename.c_str(), std::ios::in | std::ios::binary);
                    return true;
                }
            catch (const std::ifstream::failure &e)
                {
                    std::cout << "Problem opening Tracking dump Log file: " << d_dump_filename << '\n';
                    return false;
                }
        }
    else
        {
            return false;
        }
}


Tracking_Dump_Reader::~Tracking_Dump_Reader()
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
            std::cerr << "Problem closing Tracking dump Log file: " << d_dump_filename << '\n';
        }
    catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
}
