/*!
 * \file osnma_helper.h
 * \brief Class for auxiliary osnma functions
 * \author Carles Fernandez-Prades, 2024 cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2023  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "osnma_helper.h"
#include <bitset>
#include <chrono>
#include <iomanip>
#include <ios>
#include <sstream>


Osnma_Helper::Osnma_Helper()
{
    GST_START_EPOCH.tm_mday = 22;
    GST_START_EPOCH.tm_mon = 7;  // August (0-based)
    GST_START_EPOCH.tm_year = 1999 - 1900;
}


uint32_t Osnma_Helper::compute_gst(uint32_t WN, uint32_t TOW) const
{
    return (WN & 0x00000FFF) << 20 | (TOW & 0x000FFFFF);
}


uint32_t Osnma_Helper::compute_gst(tm& input)
{
    auto epoch_time_point = std::chrono::system_clock::from_time_t(mktime(&GST_START_EPOCH));
    auto input_time_point = std::chrono::system_clock::from_time_t(mktime(&input));

    // Get the duration from epoch in seconds
    auto duration_sec = std::chrono::duration_cast<std::chrono::seconds>(input_time_point - epoch_time_point);

    // Calculate the week number (WN) and time of week (TOW)
    const uint32_t sec_in_week = 604800;
    const uint32_t week_number = duration_sec.count() / sec_in_week;
    const uint32_t time_of_week = duration_sec.count() % sec_in_week;
    return compute_gst(week_number, time_of_week);
}


uint32_t Osnma_Helper::compute_gst_now()
{
    time_t now = time(nullptr);
    struct tm local_tm = *std::localtime(&now);
    struct tm utc_tm = *std::gmtime(&now);
    auto timezone_offset = std::mktime(&utc_tm) - std::mktime(&local_tm);
    auto epoch_time_point = std::chrono::system_clock::from_time_t(std::mktime(&GST_START_EPOCH) - timezone_offset) + std::chrono::seconds(13);
    auto duration_sec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - epoch_time_point);
    const uint32_t sec_in_week = 604800;
    const uint32_t week_number = duration_sec.count() / sec_in_week;
    const uint32_t time_of_week = duration_sec.count() % sec_in_week;
    return compute_gst(week_number, time_of_week);
}


std::vector<uint8_t> Osnma_Helper::gst_to_uint8(uint32_t GST) const
{
    std::vector<uint8_t> res;

    res.push_back(static_cast<uint8_t>((GST & 0xFF000000) >> 24));
    res.push_back(static_cast<uint8_t>((GST & 0x00FF0000) >> 16));
    res.push_back(static_cast<uint8_t>((GST & 0x0000FF00) >> 8));
    res.push_back(static_cast<uint8_t>(GST & 0x000000FF));
    return res;
}


/**
 * @brief Convert a binary string to a vector of bytes.
 *
 * This function takes a binary string and converts it into a vector of uint8_t bytes.
 * The binary string is padded with zeros if necessary to ensure that the total number
 * of bits is a multiple of a byte.
 *
 * @param binaryString The binary string to be converted.
 * @return The vector of bytes converted from the binary string.
 */
std::vector<uint8_t> Osnma_Helper::bytes(const std::string& binaryString) const
{
    std::vector<uint8_t> bytes;

    // Determine the size of the padding needed.
    size_t padding_size = binaryString.size() % 8;

    std::string padded_binary = binaryString;

    if (padding_size != 0)
        {
            padding_size = 8 - padding_size;          // Compute padding size
            padded_binary.append(padding_size, '0');  // Append zeros to the binary string
        }

    for (size_t i = 0; i < padded_binary.size(); i += 8)
        {
            uint8_t byte = std::bitset<8>(padded_binary.substr(i, 8)).to_ulong();
            bytes.push_back(byte);
        }

    return bytes;
}


std::string Osnma_Helper::verification_status_str(int status) const
{
    switch (status)
        {
        case 0:
            return "SUCCESS";
        case 1:
            return "FAIL";
        case 2:
            return "UNVERIFIED";
        default:
            return "UNKNOWN";
        }
}


std::string Osnma_Helper::convert_to_hex_string(const std::vector<uint8_t>& vector) const
{
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (auto byte : vector)
        {
            ss << std::setw(2) << static_cast<int>(byte);
        }
    return ss.str();
}


std::vector<uint8_t> Osnma_Helper::convert_from_hex_string(const std::string& hex_string) const
{
    std::vector<uint8_t> result;

    std::string adjusted_hex_string = hex_string;
    if (hex_string.length() % 2 != 0)
        {
            adjusted_hex_string = "0" + hex_string;
        }

    for (std::size_t i = 0; i < adjusted_hex_string.length(); i += 2)
        {
            std::string byte_string = adjusted_hex_string.substr(i, 2);
            auto byte = static_cast<uint8_t>(std::stoul(byte_string, nullptr, 16));
            result.push_back(byte);
        }

    return result;
}


uint32_t Osnma_Helper::get_WN(uint32_t GST) const
{
    return (GST & 0xFFF00000) >> 20;
}


uint32_t Osnma_Helper::get_TOW(uint32_t GST) const
{
    return GST & 0x000FFFFF;
}
