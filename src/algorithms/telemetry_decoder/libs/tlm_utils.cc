/*!
 * \file tlm_utils.cc
 * \brief Utilities for the telemetry decoder blocks.
 * \author Carles Fernandez, 2020. cfernandez(at)cttc.es
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

#include "tlm_utils.h"
#include "gnss_sdr_filesystem.h"
#include "matlab_writter_helper.h"
#include <array>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <vector>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif


int save_tlm_matfile(const std::string &dumpfile)
{
    std::ifstream::pos_type size;
    const int32_t number_of_double_vars = 2;
    const int32_t number_of_int_vars = 2;
    const int32_t epoch_size_bytes = sizeof(uint64_t) + sizeof(double) * number_of_double_vars +
                                     sizeof(int32_t) * number_of_int_vars;
    std::ifstream dump_file;
    const std::string &dump_filename_(dumpfile);

    std::cout << "Generating .mat file for " << std::string(dump_filename_.begin(), dump_filename_.end() - 4) << '\n';
    dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    try
        {
            dump_file.open(dump_filename_.c_str(), std::ios::binary | std::ios::ate);
        }
    catch (const std::ifstream::failure &e)
        {
            std::cerr << "Problem opening dump file:" << e.what() << '\n';
            return 1;
        }
    // count number of epochs and rewind
    int64_t num_epoch = 0;
    if (dump_file.is_open())
        {
            size = dump_file.tellg();
            num_epoch = static_cast<int64_t>(size) / static_cast<int64_t>(epoch_size_bytes);
            if (num_epoch == 0LL)
                {
                    // empty file, exit
                    return 1;
                }
            dump_file.seekg(0, std::ios::beg);
        }
    else
        {
            return 1;
        }
    auto TOW_at_current_symbol_ms = std::vector<double>(num_epoch);
    auto tracking_sample_counter = std::vector<uint64_t>(num_epoch);
    auto TOW_at_Preamble_ms = std::vector<double>(num_epoch);
    auto nav_symbol = std::vector<int32_t>(num_epoch);
    auto prn = std::vector<int32_t>(num_epoch);

    try
        {
            if (dump_file.is_open())
                {
                    for (int64_t i = 0; i < num_epoch; i++)
                        {
                            dump_file.read(reinterpret_cast<char *>(&TOW_at_current_symbol_ms[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&tracking_sample_counter[i]), sizeof(uint64_t));
                            dump_file.read(reinterpret_cast<char *>(&TOW_at_Preamble_ms[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&nav_symbol[i]), sizeof(int32_t));
                            dump_file.read(reinterpret_cast<char *>(&prn[i]), sizeof(int32_t));
                        }
                }
            dump_file.close();
        }
    catch (const std::ifstream::failure &e)
        {
            std::cerr << "Problem reading dump file:" << e.what() << '\n';
            return 1;
        }

    // WRITE MAT FILE
    mat_t *matfp;
    std::string filename = dump_filename_;
    filename.erase(filename.length() - 4, 4);
    filename.append(".mat");
    try
        {
            matfp = Mat_CreateVer(filename.c_str(), nullptr, MAT_FT_MAT73);
            if (reinterpret_cast<int64_t *>(matfp) != nullptr)
                {
                    std::array<size_t, 2> dims{1, static_cast<size_t>(num_epoch)};
                    write_matlab_var<2, double>("TOW_at_current_symbol_ms", TOW_at_current_symbol_ms.data(), matfp, dims);
                    write_matlab_var<2, uint64_t>("tracking_sample_counter", tracking_sample_counter.data(), matfp, dims);
                    write_matlab_var<2, double>("TOW_at_Preamble_ms", TOW_at_Preamble_ms.data(), matfp, dims);
                    write_matlab_var<2, int32_t>("nav_symbol", nav_symbol.data(), matfp, dims);
                    write_matlab_var<2, int32_t>("PRN", prn.data(), matfp, dims);
                }
            Mat_Close(matfp);
        }
    catch (const std::exception &ex)
        {
            std::cerr << "Error saving the .mat file: " << ex.what();
        }
    return 0;
}


bool tlm_remove_file(const std::string &file_to_remove)
{
    errorlib::error_code ec;
    return fs::remove(fs::path(file_to_remove), ec);
}


void tlm_cleanup_and_save_files(std::ofstream &dump_file, const std::string &dump_filename, bool dump, bool dump_mat, bool remove_dat)
{
    size_t pos = 0;
    if (dump_file.is_open() == true)
        {
            pos = dump_file.tellp();
            try
                {
                    dump_file.close();
                }
            catch (const std::exception &ex)
                {
                    LOG(WARNING) << "Exception closing dump file " << ex.what();
                }
            if (pos == 0)
                {
                    if (!tlm_remove_file(dump_filename))
                        {
                            LOG(WARNING) << "Error deleting temporary file";
                        }
                }
        }
    if (dump && (pos != 0) && dump_mat)
        {
            save_tlm_matfile(dump_filename);
            if (remove_dat)
                {
                    if (!tlm_remove_file(dump_filename))
                        {
                            LOG(WARNING) << "Error deleting temporary file";
                        }
                }
        }
}
