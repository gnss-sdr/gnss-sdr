/*!
 * \file spirent_motion_csv_dump_reader.h
 * \brief Helper file for unit testing
 * \author Javier Arribas, 2018. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_SPIRENT_MOTION_CSV_DUMP_READER_H
#define GNSS_SDR_SPIRENT_MOTION_CSV_DUMP_READER_H

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

class Spirent_Motion_Csv_Dump_Reader
{
public:
    Spirent_Motion_Csv_Dump_Reader();
    ~Spirent_Motion_Csv_Dump_Reader();
    bool read_csv_obs();
    bool restart();
    int64_t num_epochs();
    bool open_obs_file(std::string out_file);
    void close_obs_file();

    int header_lines;
    // dump variables
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

#endif  // GNSS_SDR_SPIRENT_MOTION_CSV_DUMP_READER_H
