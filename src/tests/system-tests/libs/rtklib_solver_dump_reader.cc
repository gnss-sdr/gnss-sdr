/*!
 * \file rtklib_solver_dump_reader.cc
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

#include "rtklib_solver_dump_reader.h"

#include <iostream>

bool rtklib_solver_dump_reader::read_binary_obs()
{
    try
        {
            d_dump_file.read(reinterpret_cast<char *>(&TOW_at_current_symbol_ms), sizeof(TOW_at_current_symbol_ms));
            //            std::cout << "TOW_at_current_symbol_ms: " << TOW_at_current_symbol_ms << std::endl;
            d_dump_file.read(reinterpret_cast<char *>(&week), sizeof(week));
            //            std::cout << "week: " << week << std::endl;
            d_dump_file.read(reinterpret_cast<char *>(&RX_time), sizeof(RX_time));
            //            std::cout << "RX_time: " << RX_time << std::endl;
            d_dump_file.read(reinterpret_cast<char *>(&clk_offset_s), sizeof(clk_offset_s));
            //            std::cout << "clk_offset_s: " << clk_offset_s << std::endl;
            d_dump_file.read(reinterpret_cast<char *>(&rr[0]), sizeof(rr));
            //            for (int n = 0; n < 6; n++)
            //                {
            //                    std::cout << "rr: " << rr[n] << std::endl;
            //                }
            d_dump_file.read(reinterpret_cast<char *>(&qr[0]), sizeof(qr));
            //            for (int n = 0; n < 6; n++)
            //                {
            //                    std::cout << "qr" << qr[n] << std::endl;
            //                }
            d_dump_file.read(reinterpret_cast<char *>(&latitude), sizeof(latitude));
            //std::cout << "latitude: " << latitude << std::endl;
            d_dump_file.read(reinterpret_cast<char *>(&longitude), sizeof(longitude));
            //std::cout << "longitude: " << longitude << std::endl;
            d_dump_file.read(reinterpret_cast<char *>(&height), sizeof(height));
            //std::cout << "height: " << height << std::endl;
            d_dump_file.read(reinterpret_cast<char *>(&ns), sizeof(ns));
            //            std::cout << "ns: " << (int)ns << std::endl;
            d_dump_file.read(reinterpret_cast<char *>(&status), sizeof(status));
            //            std::cout << "status: " << (int)status << std::endl;
            d_dump_file.read(reinterpret_cast<char *>(&type), sizeof(type));
            //            std::cout << "type: " << (int)type << std::endl;
            d_dump_file.read(reinterpret_cast<char *>(&AR_ratio), sizeof(AR_ratio));
            //            std::cout << "AR_ratio: " << AR_ratio << std::endl;
            d_dump_file.read(reinterpret_cast<char *>(&AR_thres), sizeof(AR_thres));
            //            std::cout << "AR_thres: " << AR_thres << std::endl;
            d_dump_file.read(reinterpret_cast<char *>(&dop[0]), sizeof(dop));

            //            for (int n = 0; n < 4; n++)
            //                {
            //                    std::cout << "dop" << dop[n] << std::endl;
            //                }
            //            getchar();
        }
    catch (const std::ifstream::failure &e)
        {
            return false;
        }
    return true;
}


bool rtklib_solver_dump_reader::restart()
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


int64_t rtklib_solver_dump_reader::num_epochs()
{
    std::ifstream::pos_type size;
    int epoch_size_bytes = sizeof(TOW_at_current_symbol_ms) + sizeof(week) + sizeof(RX_time) + sizeof(clk_offset_s) + sizeof(rr) + sizeof(qr) + sizeof(latitude) + sizeof(longitude) + sizeof(height) + sizeof(ns) + sizeof(status) + sizeof(type) + sizeof(AR_ratio) + sizeof(AR_thres) + sizeof(dop);

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


bool rtklib_solver_dump_reader::open_obs_file(std::string out_file)
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
                    std::cout << "Problem opening rtklib_solver dump Log file: " << d_dump_filename.c_str() << std::endl;
                    return false;
                }
        }
    else
        {
            return false;
        }
}


rtklib_solver_dump_reader::~rtklib_solver_dump_reader()
{
    if (d_dump_file.is_open() == true)
        {
            d_dump_file.close();
        }
}
