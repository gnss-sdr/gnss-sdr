/*!
 * \file observables_dump_reader.cc
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

#include "observables_dump_reader.h"
#include <exception>
#include <iostream>
#include <utility>

bool Observables_Dump_Reader::read_binary_obs()
{
    try
        {
            for (int i = 0; i < n_channels; i++)
                {
                    d_dump_file.read(reinterpret_cast<char *>(&RX_time[i]), sizeof(double));
                    d_dump_file.read(reinterpret_cast<char *>(&TOW_at_current_symbol_s[i]), sizeof(double));
                    d_dump_file.read(reinterpret_cast<char *>(&Carrier_Doppler_hz[i]), sizeof(double));
                    d_dump_file.read(reinterpret_cast<char *>(&Acc_carrier_phase_hz[i]), sizeof(double));
                    d_dump_file.read(reinterpret_cast<char *>(&Pseudorange_m[i]), sizeof(double));
                    d_dump_file.read(reinterpret_cast<char *>(&PRN[i]), sizeof(double));
                    d_dump_file.read(reinterpret_cast<char *>(&valid[i]), sizeof(double));
                }
        }
    catch (const std::ifstream::failure &e)
        {
            return false;
        }
    return true;
}


bool Observables_Dump_Reader::restart()
{
    if (d_dump_file.is_open())
        {
            d_dump_file.clear();
            d_dump_file.seekg(0, std::ios::beg);
            return true;
        }
    return false;
}


int64_t Observables_Dump_Reader::num_epochs()
{
    std::ifstream::pos_type size;
    int number_of_vars_in_epoch = n_channels * 7;
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


bool Observables_Dump_Reader::open_obs_file(std::string out_file)
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
                    std::cout << "Problem opening Observables dump Log file: " << d_dump_filename << '\n';
                    return false;
                }
        }
    else
        {
            return false;
        }
}


void Observables_Dump_Reader::close_obs_file()
{
    if (d_dump_file.is_open() == false)
        {
            d_dump_file.close();
        }
}


Observables_Dump_Reader::Observables_Dump_Reader(int n_channels_)
    : n_channels(n_channels_)
{
    RX_time = std::vector<double>(n_channels);
    TOW_at_current_symbol_s = std::vector<double>(n_channels);
    Carrier_Doppler_hz = std::vector<double>(n_channels);
    Acc_carrier_phase_hz = std::vector<double>(n_channels);
    Pseudorange_m = std::vector<double>(n_channels);
    PRN = std::vector<double>(n_channels);
    valid = std::vector<double>(n_channels);
}


Observables_Dump_Reader::~Observables_Dump_Reader()
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
            std::cerr << "Problem closing Observables dump Log file: " << d_dump_filename << '\n';
        }
    catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
}
