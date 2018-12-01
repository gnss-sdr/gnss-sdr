/*!
 * \file spirent_motion_csv_dump_reader.cc
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

#include "spirent_motion_csv_dump_reader.h"
#include <boost/tokenizer.hpp>
#include <iostream>


spirent_motion_csv_dump_reader::spirent_motion_csv_dump_reader()
{
    header_lines = 2;
    TOW_ms = 0.0;
    Pos_X = 0.0;
    Pos_Y = 0.0;
    Pos_Z = 0.0;
    Vel_X = 0.0;
    Vel_Y = 0.0;
    Vel_Z = 0.0;
    Acc_X = 0.0;
    Acc_Y = 0.0;
    Acc_Z = 0.0;
    Jerk_X = 0.0;
    Jerk_Y = 0.0;
    Jerk_Z = 0.0;
    Lat = 0.0;
    Long = 0.0;
    Height = 0.0;
    Heading = 0.0;
    Elevation = 0.0;
    Bank = 0.0;
    Ang_vel_X = 0.0;
    Ang_vel_Y = 0.0;
    Ang_vel_Z = 0.0;
    Ang_acc_X = 0.0;
    Ang_acc_Y = 0.0;
    Ang_acc_Z = 0.0;
    Ant1_Pos_X = 0.0;
    Ant1_Pos_Y = 0.0;
    Ant1_Pos_Z = 0.0;
    Ant1_Vel_X = 0.0;
    Ant1_Vel_Y = 0.0;
    Ant1_Vel_Z = 0.0;
    Ant1_Acc_X = 0.0;
    Ant1_Acc_Y = 0.0;
    Ant1_Acc_Z = 0.0;
    Ant1_Lat = 0.0;
    Ant1_Long = 0.0;
    Ant1_Height = 0.0;
    Ant1_DOP = 0.0;
}


spirent_motion_csv_dump_reader::~spirent_motion_csv_dump_reader()
{
    if (d_dump_file.is_open() == true)
        {
            d_dump_file.close();
        }
}


bool spirent_motion_csv_dump_reader::read_csv_obs()
{
    try
        {
            std::vector<double> vec;
            std::string line;
            if (getline(d_dump_file, line))
                {
                    boost::tokenizer<boost::escaped_list_separator<char>> tk(
                        line, boost::escaped_list_separator<char>('\\', ',', '\"'));
                    for (boost::tokenizer<boost::escaped_list_separator<char>>::iterator i(
                             tk.begin());
                         i != tk.end(); ++i)
                        {
                            try
                                {
                                    vec.push_back(std::stod(*i));
                                }
                            catch (const std::exception &ex)
                                {
                                    vec.push_back(0.0);
                                }
                        }
                    parse_vector(vec);
                }
        }
    catch (const std::ifstream::failure &e)
        {
            return false;
        }
    return true;
}


bool spirent_motion_csv_dump_reader::parse_vector(std::vector<double> &vec)
{
    try
        {
            int n = 0;
            TOW_ms = vec.at(n++);
            Pos_X = vec.at(n++);
            Pos_Y = vec.at(n++);
            Pos_Z = vec.at(n++);
            Vel_X = vec.at(n++);
            Vel_Y = vec.at(n++);
            Vel_Z = vec.at(n++);
            Acc_X = vec.at(n++);
            Acc_Y = vec.at(n++);
            Acc_Z = vec.at(n++);
            Jerk_X = vec.at(n++);
            Jerk_Y = vec.at(n++);
            Jerk_Z = vec.at(n++);
            Lat = vec.at(n++);
            Long = vec.at(n++);
            Height = vec.at(n++);
            Heading = vec.at(n++);
            Elevation = vec.at(n++);
            Bank = vec.at(n++);
            Ang_vel_X = vec.at(n++);
            Ang_vel_Y = vec.at(n++);
            Ang_vel_Z = vec.at(n++);
            Ang_acc_X = vec.at(n++);
            Ang_acc_Y = vec.at(n++);
            Ang_acc_Z = vec.at(n++);
            Ant1_Pos_X = vec.at(n++);
            Ant1_Pos_Y = vec.at(n++);
            Ant1_Pos_Z = vec.at(n++);
            Ant1_Vel_X = vec.at(n++);
            Ant1_Vel_Y = vec.at(n++);
            Ant1_Vel_Z = vec.at(n++);
            Ant1_Acc_X = vec.at(n++);
            Ant1_Acc_Y = vec.at(n++);
            Ant1_Acc_Z = vec.at(n++);
            Ant1_Lat = vec.at(n++);
            Ant1_Long = vec.at(n++);
            Ant1_Height = vec.at(n++);
            Ant1_DOP = vec.at(n++);
            return true;
        }
    catch (const std::exception &ex)
        {
            return false;
        }
}


bool spirent_motion_csv_dump_reader::restart()
{
    if (d_dump_file.is_open())
        {
            d_dump_file.clear();
            d_dump_file.seekg(0, std::ios::beg);
            std::string line;
            for (int n = 0; n < header_lines; n++)
                {
                    getline(d_dump_file, line);
                }
            return true;
        }
    else
        {
            return false;
        }
}


int64_t spirent_motion_csv_dump_reader::num_epochs()
{
    int64_t nepoch = 0LL;
    std::string line;
    std::ifstream tmpfile(d_dump_filename.c_str());
    if (tmpfile.is_open())
        {
            while (std::getline(tmpfile, line))
                {
                    ++nepoch;
                }
            return nepoch - header_lines;
        }
    else
        {
            return 0;
        }
}


bool spirent_motion_csv_dump_reader::open_obs_file(std::string out_file)
{
    if (d_dump_file.is_open() == false)
        {
            try
                {
                    d_dump_filename = out_file;
                    d_dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
                    d_dump_file.open(d_dump_filename.c_str());
                    std::string line;
                    for (int n = 0; n < header_lines; n++)
                        {
                            getline(d_dump_file, line);
                        }
                    return true;
                }
            catch (const std::ifstream::failure &e)
                {
                    std::cout << "Problem opening Spirent CSV dump Log file: " << d_dump_filename.c_str() << std::endl;
                    return false;
                }
        }
    else
        {
            return false;
        }
}


void spirent_motion_csv_dump_reader::close_obs_file()
{
    if (d_dump_file.is_open() == false)
        {
            d_dump_file.close();
        }
}
