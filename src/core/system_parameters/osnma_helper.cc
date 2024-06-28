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
#include <iomanip>
#include <ios>
#include <sstream>

uint32_t Osnma_Helper::compute_gst(uint32_t WN, uint32_t TOW) const
{
    uint32_t GST = (WN & 0x00000FFF) << 20 | (TOW & 0x000FFFFF);
    return GST;
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
std::vector<uint8_t> Osnma_Helper::bytes(const std::string& binaryString) {
    std::vector<uint8_t> bytes;

    // Determine the size of the padding needed.
    size_t padding_size = binaryString.size() % 8;

    std::string padded_binary = binaryString;

    if (padding_size != 0) {
            padding_size = 8 - padding_size;  // Compute padding size
            padded_binary.append(padding_size, '0');  // Append zeros to the binary string
        }

    for (size_t i = 0; i < padded_binary.size(); i += 8) {
            uint8_t byte = std::bitset<8>(padded_binary.substr(i, 8)).to_ulong();
            bytes.push_back(byte);
        }

    return bytes;
}


std::string Osnma_Helper::verification_status_str(int status)
{
        switch (status) {
            case 0: return "SUCCESS";
            case 1: return "FAIL";
            case 2: return "UNVERIFIED";
            default: return "UNKNOWN";
            }
}


std::string Osnma_Helper::convert_to_hex_string(const std::vector<uint8_t>& vector)
{
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (auto byte : vector) {
            ss << std::setw(2) << static_cast<int>(byte);
        }
    return ss.str();
}
