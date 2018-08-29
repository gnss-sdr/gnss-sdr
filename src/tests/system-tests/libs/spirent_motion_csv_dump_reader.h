/*!
 * \file spirent_motion_csv_dump_reader.h
 * \brief Helper file for unit testing
 * \author Javier Arribas, 2018. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_spirent_motion_csv_dump_READER_H
#define GNSS_SDR_spirent_motion_csv_dump_READER_H

#include <boost/tokenizer.hpp>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

class spirent_motion_csv_dump_reader
{
public:
    spirent_motion_csv_dump_reader();
    ~spirent_motion_csv_dump_reader();
    bool read_csv_obs();
    bool restart();
    int64_t num_epochs();
    bool open_obs_file(std::string out_file);
    void close_obs_file();

    int header_lines;
    //dump variables
    double TOW_ms;
    double Pos_X;
    double Pos_Y;
    double Pos_Z;
    double Vel_X;
    double Vel_Y;
    double Vel_Z;
    double Acc_X;
    double Acc_Y;
    double Acc_Z;
    double Jerk_X;
    double Jerk_Y;
    double Jerk_Z;
    double Lat;
    double Long;
    double Height;
    double Heading;
    double Elevation;
    double Bank;
    double Ang_vel_X;
    double Ang_vel_Y;
    double Ang_vel_Z;
    double Ang_acc_X;
    double Ang_acc_Y;
    double Ang_acc_Z;
    double Ant1_Pos_X;
    double Ant1_Pos_Y;
    double Ant1_Pos_Z;
    double Ant1_Vel_X;
    double Ant1_Vel_Y;
    double Ant1_Vel_Z;
    double Ant1_Acc_X;
    double Ant1_Acc_Y;
    double Ant1_Acc_Z;
    double Ant1_Lat;
    double Ant1_Long;
    double Ant1_Height;
    double Ant1_DOP;

private:
    std::string d_dump_filename;
    std::ifstream d_dump_file;
    bool parse_vector(std::vector<double> &vec);
};

#endif  //GNSS_SDR_spirent_motion_csv_dump_READER_H
