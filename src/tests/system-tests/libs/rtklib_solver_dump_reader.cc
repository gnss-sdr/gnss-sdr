/*!
 * \file rtklib_solver_dump_reader.cc
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

#include "rtklib_solver_dump_reader.h"
#include <exception>
#include <iostream>
#include <utility>

bool Rtklib_Solver_Dump_Reader::read_binary_obs()
{
    try
        {
            d_dump_file.read(reinterpret_cast<char *>(&TOW_at_current_symbol_ms), sizeof(uint32_t));
            d_dump_file.read(reinterpret_cast<char *>(&week), sizeof(uint32_t));
            d_dump_file.read(reinterpret_cast<char *>(&RX_time), sizeof(double));
            d_dump_file.read(reinterpret_cast<char *>(&clk_offset_s), sizeof(double));
            d_dump_file.read(reinterpret_cast<char *>(&rr[0]), sizeof(double));
            d_dump_file.read(reinterpret_cast<char *>(&rr[1]), sizeof(double));
            d_dump_file.read(reinterpret_cast<char *>(&rr[2]), sizeof(double));
            d_dump_file.read(reinterpret_cast<char *>(&rr[3]), sizeof(double));
            d_dump_file.read(reinterpret_cast<char *>(&rr[4]), sizeof(double));
            d_dump_file.read(reinterpret_cast<char *>(&rr[5]), sizeof(double));
            d_dump_file.read(reinterpret_cast<char *>(&qr[0]), sizeof(double));
            d_dump_file.read(reinterpret_cast<char *>(&qr[1]), sizeof(double));
            d_dump_file.read(reinterpret_cast<char *>(&qr[2]), sizeof(double));
            d_dump_file.read(reinterpret_cast<char *>(&qr[3]), sizeof(double));
            d_dump_file.read(reinterpret_cast<char *>(&qr[4]), sizeof(double));
            d_dump_file.read(reinterpret_cast<char *>(&qr[5]), sizeof(double));
            d_dump_file.read(reinterpret_cast<char *>(&latitude), sizeof(double));
            d_dump_file.read(reinterpret_cast<char *>(&longitude), sizeof(double));
            d_dump_file.read(reinterpret_cast<char *>(&height), sizeof(double));
            d_dump_file.read(reinterpret_cast<char *>(&ns), sizeof(uint8_t));
            d_dump_file.read(reinterpret_cast<char *>(&status), sizeof(uint8_t));
            d_dump_file.read(reinterpret_cast<char *>(&type), sizeof(uint8_t));
            d_dump_file.read(reinterpret_cast<char *>(&AR_ratio), sizeof(float));
            d_dump_file.read(reinterpret_cast<char *>(&AR_thres), sizeof(float));
            d_dump_file.read(reinterpret_cast<char *>(&dop[0]), sizeof(double));
            d_dump_file.read(reinterpret_cast<char *>(&dop[1]), sizeof(double));
            d_dump_file.read(reinterpret_cast<char *>(&dop[2]), sizeof(double));
            d_dump_file.read(reinterpret_cast<char *>(&dop[3]), sizeof(double));
        }
    catch (const std::ifstream::failure &e)
        {
            return false;
        }
    return true;
}


bool Rtklib_Solver_Dump_Reader::restart()
{
    if (d_dump_file.is_open())
        {
            d_dump_file.clear();
            d_dump_file.seekg(0, std::ios::beg);
            return true;
        }
    return false;
}


int64_t Rtklib_Solver_Dump_Reader::num_epochs()
{
    std::ifstream::pos_type size;
    int epoch_size_bytes = 2 * sizeof(uint32_t) + 21 * sizeof(double) + 3 * sizeof(uint8_t) + 2 * sizeof(float);
    std::ifstream tmpfile(d_dump_filename.c_str(), std::ios::binary | std::ios::ate);
    if (tmpfile.is_open())
        {
            size = tmpfile.tellg();
            int64_t nepoch = size / epoch_size_bytes;
            return nepoch;
        }
    return 0;
}


bool Rtklib_Solver_Dump_Reader::open_obs_file(std::string out_file)
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
                    std::cout << "Problem opening rtklib_solver dump Log file: " << d_dump_filename << '\n';
                    return false;
                }
        }
    else
        {
            return false;
        }
}


Rtklib_Solver_Dump_Reader::~Rtklib_Solver_Dump_Reader()
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
            std::cerr << "Problem closing rtklib_solver dump Log file: " << d_dump_filename << '\n';
        }
    catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
}
