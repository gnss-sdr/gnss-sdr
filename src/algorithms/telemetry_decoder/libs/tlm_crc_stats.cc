/*!
 * \file tlm_crc_stats.cc
 * \brief Class that computes the telemetry CRC statistics
 * \author Marc Majoral, 2021. mmajoral(at)cttc.es
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

#include "tlm_crc_stats.h"
#include "gnss_sdr_create_directory.h"
#include "gnss_sdr_filesystem.h"
#include <glog/logging.h>
#include <iomanip>   // for std::setw()
#include <iostream>  // for cerr, cout
#include <utility>   // for std::move


void Tlm_CRC_Stats::initialize(std::string dump_crc_stats_filename)
{
    d_dump_crc_stats_filename = std::move(dump_crc_stats_filename);

    d_enable_crc_stats = true;
    d_num_crc_ok = 0;
    d_num_crc_not_ok = 0;
}


bool Tlm_CRC_Stats::set_channel(int32_t channel)
{
    std::string dump_path;

    d_channel = channel;

    // Get path
    if (d_dump_crc_stats_filename.find_last_of('/') != std::string::npos)
        {
            std::string dump_filename_ = d_dump_crc_stats_filename.substr(d_dump_crc_stats_filename.find_last_of('/') + 1);
            dump_path = d_dump_crc_stats_filename.substr(0, d_dump_crc_stats_filename.find_last_of('/'));
            d_dump_crc_stats_filename = dump_filename_;
        }
    else
        {
            dump_path = std::string(".");
        }
    // remove extension if any
    if (d_dump_crc_stats_filename.substr(1).find_last_of('.') != std::string::npos)
        {
            d_dump_crc_stats_filename = d_dump_crc_stats_filename.substr(0, d_dump_crc_stats_filename.find_last_of('.'));
        }

    d_dump_crc_stats_filename.append("_ch");
    d_dump_crc_stats_filename.append(std::to_string(d_channel));
    d_dump_crc_stats_filename.append(".txt");
    d_dump_crc_stats_filename = dump_path + fs::path::preferred_separator + d_dump_crc_stats_filename;

    // create directory
    if (!gnss_sdr_create_directory(dump_path))
        {
            std::cerr << "GNSS-SDR cannot create telemetry CRC stats dump file for the Telemetry block. The telemetry CRC statistics has been disabled. Wrong permissions?\n";
            d_enable_crc_stats = false;
        }
    d_dump_file.exceptions(std::ofstream::failbit | std::ofstream::badbit);
    try
        {
            d_dump_file.open(d_dump_crc_stats_filename.c_str(), std::ios::out);
        }
    catch (const std::ofstream::failure &e)
        {
            LOG(WARNING) << "Exception opening telemetry CRC stats dump file " << e.what();
            d_enable_crc_stats = false;
        }

    return true;
}


void Tlm_CRC_Stats::update_CRC_stats(bool CRC)
{
    if (CRC)
        {
            d_num_crc_ok++;
        }
    else
        {
            d_num_crc_not_ok++;
        }
}


Tlm_CRC_Stats::~Tlm_CRC_Stats()
{
    const uint32_t num_crc_tests = d_num_crc_ok + d_num_crc_not_ok;
    float success_rate = 0.0;

    if (num_crc_tests > 0)
        {
            success_rate = static_cast<float>(d_num_crc_ok) / static_cast<float>(num_crc_tests);
        }
    try
        {
            const std::string txt_num_crc_tests("Num CRC Tests");
            const auto align_num_crc_tests = txt_num_crc_tests.length();
            const std::string txt_success_tests(" |  Successful Tests");
            const auto align_success_tests = txt_success_tests.length();
            const std::string txt_success_rate(" | Success rate");
            const auto align_success_rate = txt_success_rate.length();
            const std::string txt_delimiter(" |");
            const auto align_delimiter = txt_delimiter.length();

            if (d_enable_crc_stats && d_dump_file.is_open())
                {
                    // write results to the telemetry CRC statistics output file
                    try
                        {
                            d_dump_file << txt_num_crc_tests << txt_success_tests << txt_success_rate << std::endl;
                            d_dump_file << std::setw(align_num_crc_tests) << num_crc_tests << txt_delimiter << std::setw(align_success_tests - align_delimiter) << d_num_crc_ok << txt_delimiter << std::setw(align_success_rate - align_delimiter) << std::setprecision(4) << success_rate << std::endl;
                        }
                    catch (const std::exception &ex)
                        {
                            DLOG(INFO) << "Telemetry CRC stats cannot write on the output file " << d_dump_crc_stats_filename.c_str();
                        }

                    const auto pos = d_dump_file.tellp();
                    try
                        {
                            d_dump_file.close();
                        }
                    catch (const std::exception &ex)
                        {
                            LOG(WARNING) << "Exception in destructor closing the telemetry CRC stats dump file " << ex.what();
                        }
                    if (pos == 0)
                        {
                            errorlib::error_code ec;
                            if (!fs::remove(fs::path(d_dump_crc_stats_filename), ec))
                                {
                                    std::cerr << "Problem removing telemetry CRC stats temporary file " << d_dump_crc_stats_filename << '\n';
                                }
                        }
                }
        }
    catch (const std::exception &e)
        {
            std::cerr << "Problem in Tlm_CRC_Stats destructor: " << e.what() << '\n';
        }
}
