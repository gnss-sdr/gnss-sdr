/*!
 * \file has_simple_printer.cc
 * \brief Implementation of a class that prints HAS messages content in a txt
 * file.
 * \author Carles Fernandez-Prades, 2021. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "has_simple_printer.h"
#include "Galileo_CNAV.h"
#include "galileo_has_data.h"
#include "gnss_sdr_filesystem.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <glog/logging.h>
#include <algorithm>  // for std::find, std::count
#include <bitset>     // for std::bitset
#include <cstdint>    // for uint8_t, ...
#include <ctime>      // for tm
#include <exception>  // for std::exception
#include <iomanip>    // for std::setw, std::setprecision
#include <ios>        // for std::fixed
#include <iostream>   // for std::cout, std::cerr
#include <sstream>    // for std::stringstream


Has_Simple_Printer::Has_Simple_Printer(const std::string& base_path,
    const std::string& filename,
    bool time_tag_name) : d_has_base_path(base_path),
                          d_data_printed(false)

{
    fs::path full_path(fs::current_path());
    const fs::path p(d_has_base_path);
    if (!fs::exists(p))
        {
            std::string new_folder;
            for (const auto& folder : fs::path(d_has_base_path))
                {
                    new_folder += folder.string();
                    errorlib::error_code ec;
                    if (!fs::exists(new_folder))
                        {
                            if (!fs::create_directory(new_folder, ec))
                                {
                                    std::cerr << "Could not create the " << new_folder << " folder.\n";
                                    d_has_base_path = full_path.string();
                                }
                        }
                    new_folder += fs::path::preferred_separator;
                }
        }
    else
        {
            d_has_base_path = p.string();
        }
    if (d_has_base_path != ".")
        {
            std::cout << "HAS Message file will be stored at " << d_has_base_path << '\n';
        }

    d_has_base_path = d_has_base_path + fs::path::preferred_separator;

    const boost::posix_time::ptime pt = boost::posix_time::second_clock::local_time();
    const tm timeinfo = boost::posix_time::to_tm(pt);

    if (time_tag_name)
        {
            std::stringstream strm0;
            const int year = timeinfo.tm_year - 100;
            strm0 << year;
            const int month = timeinfo.tm_mon + 1;
            if (month < 10)
                {
                    strm0 << "0";
                }
            strm0 << month;
            const int day = timeinfo.tm_mday;
            if (day < 10)
                {
                    strm0 << "0";
                }
            strm0 << day << "_";
            const int hour = timeinfo.tm_hour;
            if (hour < 10)
                {
                    strm0 << "0";
                }
            strm0 << hour;
            const int min = timeinfo.tm_min;
            if (min < 10)
                {
                    strm0 << "0";
                }
            strm0 << min;
            const int sec = timeinfo.tm_sec;
            if (sec < 10)
                {
                    strm0 << "0";
                }
            strm0 << sec;

            d_has_filename = filename + "_" + strm0.str() + ".txt";
        }
    else
        {
            d_has_filename = filename + ".txt";
        }
    d_has_filename = d_has_base_path + d_has_filename;
    d_has_file.open(d_has_filename.c_str());
}


Has_Simple_Printer::~Has_Simple_Printer()
{
    DLOG(INFO) << "HAS Message printer destructor called.";
    try
        {
            close_file();
        }
    catch (const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    if (!d_data_printed)
        {
            errorlib::error_code ec;
            if (!fs::remove(fs::path(d_has_filename), ec))
                {
                    LOG(INFO) << "Error deleting temporary HAS Message file.";
                }
        }
}


bool Has_Simple_Printer::print_message(const Galileo_HAS_data* const has_data)
{
    std::lock_guard<std::mutex> guard(d_mutex);

    d_data_printed = true;
    std::string indent = "    ";

    if (d_has_file.is_open())
        {
            d_has_file << "HAS Message Type 1 Received.\n";
            d_has_file << "----------------------------\n";
            d_has_file << "HAS mode:       ";
            switch (has_data->has_status)
                {
                case 0:
                    d_has_file << "Test\n";
                    break;
                case 1:
                    d_has_file << "Operational\n";
                    break;
                case 2:
                    d_has_file << "Reserved\n";
                    break;
                case 3:
                    d_has_file << "Do not use HAS\n";
                    break;
                default:
                    d_has_file << "Unknown\n";
                }
            d_has_file << "HAS message ID: " << static_cast<float>(has_data->message_id) << "\n\n";

            d_has_file << indent << "MT1 Header\n";
            d_has_file << indent << "----------\n";
            d_has_file << indent << indent << "TOH [s]:             " << static_cast<float>(has_data->header.toh) << '\n';
            d_has_file << indent << indent << "Mask flag:           " << static_cast<float>(has_data->header.mask_flag) << '\n';
            d_has_file << indent << indent << "Orbit Corr. Flag:    " << static_cast<float>(has_data->header.orbit_correction_flag) << '\n';
            d_has_file << indent << indent << "Clock Full-set Flag: " << static_cast<float>(has_data->header.clock_fullset_flag) << '\n';
            d_has_file << indent << indent << "Clock Subset Flag:   " << static_cast<float>(has_data->header.clock_subset_flag) << '\n';
            d_has_file << indent << indent << "Code Bias Flag:      " << static_cast<float>(has_data->header.code_bias_flag) << '\n';
            d_has_file << indent << indent << "Phase Bias Flag:     " << static_cast<float>(has_data->header.phase_bias_flag) << '\n';
            d_has_file << indent << indent << "Mask ID:             " << static_cast<float>(has_data->header.mask_id) << '\n';
            d_has_file << indent << indent << "IOD Set ID:          " << static_cast<float>(has_data->header.iod_id) << "\n\n";

            d_has_file << indent << "MT1 Body\n";
            d_has_file << indent << "--------\n";
            d_has_file << indent << indent << "Mask Block\n";
            d_has_file << indent << indent << "----------\n";
            d_has_file << indent << indent << "Nsys:                        " << static_cast<float>(has_data->Nsys) << '\n';
            d_has_file << indent << indent << "GNSS ID:                     " << print_vector(has_data->gnss_id_mask) << "  (0: GPS, 2: Galileo)\n";

            int Nsat = has_data->get_nsat();
            d_has_file << indent << indent << "Satellite Mask:              " << print_vector_binary(has_data->satellite_mask, HAS_MSG_SATELLITE_MASK_LENGTH) << '\n';
            d_has_file << indent << indent << "                             Nsat: " << Nsat << '\n';
            std::vector<std::string> system_strings = has_data->get_systems_string();
            for (uint8_t i = 0; i < has_data->Nsys; i++)
                {
                    d_has_file << indent << indent << "                             PRN for " << system_strings[i] << ": " << print_vector(has_data->get_PRNs_in_mask(i)) << "  (" << has_data->get_PRNs_in_mask(i).size() << " satellites)\n";
                }

            d_has_file << indent << indent << "Signal Mask:                 " << print_vector_binary(has_data->signal_mask, HAS_MSG_SIGNAL_MASK_LENGTH) << '\n';
            for (uint8_t i = 0; i < has_data->Nsys; i++)
                {
                    d_has_file << indent << indent << "                             Bias corrections for " << system_strings[i] << " signals: " << print_vector_string(has_data->get_signals_in_mask(i)) << '\n';
                }

            d_has_file << indent << indent << "Cell Mask Availability Flag: " << print_vector(has_data->cell_mask_availability_flag) << '\n';
            for (uint8_t i = 0; i < has_data->Nsys; i++)
                {
                    if (has_data->cell_mask_availability_flag[i] == true)
                        {
                            std::string text;
                            if (has_data->gnss_id_mask[i] == 0)
                                {
                                    text = "Cell Mask for GPS:           ";
                                }
                            else if (has_data->gnss_id_mask[i] == 2)
                                {
                                    text = "Cell Mask for Galileo:       ";
                                }
                            else
                                {
                                    text = "Cell Mask for Reserved:      ";
                                }
                            d_has_file << indent << indent << text;
                            const std::string filler(indent.length() * 2 + text.length(), ' ');
                            d_has_file << print_matrix(has_data->cell_mask[i], filler);
                        }
                }
            d_has_file << indent << indent << "Nav message:                 " << print_vector(has_data->nav_message) << "  (0: GPS LNAV or Galileo I/NAV)\n";

            if (has_data->header.orbit_correction_flag == true)
                {
                    d_has_file << '\n';
                    d_has_file << indent << indent << "Orbit Corrections Block\n";
                    d_has_file << indent << indent << "-----------------------\n";
                    d_has_file << indent << indent << "Validity interval:     " << static_cast<float>(has_data->validity_interval_index_orbit_corrections) << '\n';
                    d_has_file << indent << indent << "GNSS IOD:              " << print_vector(has_data->gnss_iod) << '\n';
                    d_has_file << indent << indent << "Delta Radial [m]:      " << print_vector(has_data->delta_radial, HAS_MSG_DELTA_RADIAL_SCALE_FACTOR) << '\n';
                    d_has_file << indent << indent << "Delta Along Track [m]: " << print_vector(has_data->delta_along_track, HAS_MSG_DELTA_ALONG_TRACK_SCALE_FACTOR) << '\n';
                    d_has_file << indent << indent << "Delta Cross Track [m]: " << print_vector(has_data->delta_cross_track, HAS_MSG_DELTA_CROSS_TRACK_SCALE_FACTOR) << '\n';
                }

            if (has_data->header.clock_fullset_flag == true)
                {
                    d_has_file << '\n';
                    d_has_file << indent << indent << "Clock Full-set Corrections Block\n";
                    d_has_file << indent << indent << "--------------------------------\n";
                    d_has_file << indent << indent << "Validity interval:         " << static_cast<float>(has_data->validity_interval_index_clock_fullset_corrections) << '\n';
                    d_has_file << indent << indent << "Delta Clock C0 Multiplier: " << print_vector(has_data->delta_clock_c0_multiplier) << '\n';
                    d_has_file << indent << indent << "Delta Clock C0 [m]:        " << print_vector(has_data->delta_clock_c0, HAS_MSG_DELTA_CLOCK_SCALE_FACTOR) << '\n';
                }

            if (has_data->header.clock_subset_flag == true)
                {
                    d_has_file << '\n';
                    d_has_file << indent << indent << "Clock Subset Corrections Block\n";
                    d_has_file << indent << indent << "------------------------------\n";
                    d_has_file << indent << indent << "Validity interval:         " << static_cast<float>(has_data->validity_interval_index_clock_subset_corrections) << '\n';
                    d_has_file << indent << indent << "Nsysprime:                 " << static_cast<float>(has_data->Nsysprime) << '\n';
                    d_has_file << indent << indent << "GNSS ID:                   " << print_vector(has_data->gnss_id_clock_subset) << '\n';
                    d_has_file << indent << indent << "Delta Clock C0 Multiplier: " << print_vector(has_data->delta_clock_c0_multiplier_clock_subset) << '\n';
                    d_has_file << indent << indent << "Satellite sub-mask:        ";
                    int Nsatprime = 0;
                    for (uint8_t k = 0; k < has_data->Nsysprime; k++)
                        {
                            auto it = std::find(has_data->gnss_id_mask.begin(), has_data->gnss_id_mask.end(), has_data->gnss_id_clock_subset[k]);
                            if (it != has_data->gnss_id_mask.end())
                                {
                                    int index = it - has_data->gnss_id_mask.begin();
                                    std::string sat_mask = print_vector_binary(std::vector<uint64_t>(1, has_data->satellite_mask[index]), HAS_MSG_SATELLITE_MASK_LENGTH);
                                    int number_sats_satellite_mask = std::count(sat_mask.begin(), sat_mask.end(), '1');
                                    uint64_t mask_value = has_data->satellite_submask[index];
                                    // convert value into string
                                    std::string binary("");
                                    uint64_t mask = 1;
                                    for (int i = 0; i < number_sats_satellite_mask - 1; i++)
                                        {
                                            if ((mask & mask_value) >= 1)
                                                {
                                                    binary.insert(0, "1");
                                                }
                                            else
                                                {
                                                    binary.insert(0, "0");
                                                }
                                            mask <<= 1;
                                        }
                                    d_has_file << binary << " ";
                                    Nsatprime += std::count(binary.begin(), binary.end(), '1');
                                }
                        }
                    d_has_file << '\n';
                    d_has_file << "                           Nsat in subset = " << Nsatprime << '\n';
                    const std::string text("Delta Clock C0 [m]:        ");
                    const std::string filler(indent.length() * 2 + text.length(), ' ');
                    d_has_file << indent << indent << text << print_matrix(has_data->delta_clock_c0_clock_subset, filler, HAS_MSG_DELTA_CLOCK_SCALE_FACTOR);
                }

            if (has_data->header.code_bias_flag == true)
                {
                    d_has_file << '\n';
                    d_has_file << indent << indent << "Code Bias Block\n";
                    d_has_file << indent << indent << "---------------\n";
                    d_has_file << indent << indent << "Validity interval: " << static_cast<float>(has_data->validity_interval_index_code_bias_corrections) << '\n';
                    const std::string text("Code bias [m]:     ");
                    const std::string filler(indent.length() * 2 + text.length(), ' ');
                    d_has_file << indent << indent << text << print_matrix(has_data->code_bias, filler, HAS_MSG_CODE_BIAS_SCALE_FACTOR);
                }

            if (has_data->header.phase_bias_flag == true)
                {
                    d_has_file << '\n';
                    d_has_file << indent << indent << "Phase Bias Block\n";
                    d_has_file << indent << indent << "----------------\n";
                    d_has_file << indent << indent << "Validity interval:             " << static_cast<float>(has_data->validity_interval_index_phase_bias_corrections) << '\n';
                    const std::string text("Phase bias [cycles]:           ");
                    const std::string filler(indent.length() * 2 + text.length(), ' ');
                    d_has_file << indent << indent << text << print_matrix(has_data->phase_bias, filler, HAS_MSG_PHASE_BIAS_SCALE_FACTOR);
                    d_has_file << indent << indent << "Phase discontinuity indicator: " << print_matrix(has_data->phase_discontinuity_indicator, filler);
                }

            d_has_file << "\n\n";
            return true;
        }
    return false;
}


template <class T>
std::string Has_Simple_Printer::print_vector(const std::vector<T>& vec, float scale_factor) const
{
    std::string msg;
    std::stringstream ss;
    for (auto el : vec)
        {
            if (scale_factor == 1)
                {
                    ss << static_cast<float>(el) << " ";
                }
            else
                {
                    ss << std::setw(9) << std::setprecision(4) << std::fixed << static_cast<float>(el) * scale_factor << " ";
                }
        }
    msg += ss.str();
    return msg;
}


template <class T>
std::string Has_Simple_Printer::print_vector_binary(const std::vector<T>& vec, size_t bit_length) const
{
    std::string msg;
    std::stringstream ss;
    for (auto el : vec)
        {
            if (bit_length == HAS_MSG_SATELLITE_MASK_LENGTH)
                {
                    std::bitset<HAS_MSG_SATELLITE_MASK_LENGTH> bits(el);
                    ss << bits.to_string() << " ";
                }
            if (bit_length == HAS_MSG_SIGNAL_MASK_LENGTH)
                {
                    std::bitset<HAS_MSG_SIGNAL_MASK_LENGTH> bits(el);
                    ss << bits.to_string() << " ";
                }
        }
    msg += ss.str();
    return msg;
}


template <class T>
std::string Has_Simple_Printer::print_matrix(const std::vector<std::vector<T>>& mat, const std::string& filler, float scale_factor) const
{
    std::string msg;
    std::stringstream ss;
    bool first_row = true;

    if (!mat.empty())
        {
            for (size_t row = 0; row < mat.size(); row++)
                {
                    if (first_row)
                        {
                            first_row = false;
                        }
                    else
                        {
                            ss << filler;
                        }
                    for (size_t col = 0; col < mat[0].size(); col++)
                        {
                            if (scale_factor == 1)
                                {
                                    ss << static_cast<float>(mat[row][col]) << " ";
                                }
                            else
                                {
                                    ss << std::setw(9) << std::setprecision(2) << std::fixed << static_cast<float>(mat[row][col]) * scale_factor << " ";
                                }
                        }
                    ss << '\n';
                }
        }
    else
        {
            ss << '\n';
        }
    msg += ss.str();
    return msg;
}


std::string Has_Simple_Printer::print_vector_string(const std::vector<std::string>& vec) const
{
    std::string msg;
    bool first = true;
    for (const auto& el : vec)
        {
            if (first == true)
                {
                    msg += el;
                    first = false;
                }
            else
                {
                    msg += "  " + el;
                }
        }
    return msg;
}


bool Has_Simple_Printer::close_file()
{
    if (d_has_file.is_open())
        {
            d_has_file.close();
            return true;
        }
    return false;
}
