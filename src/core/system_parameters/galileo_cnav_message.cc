/*!
 * \file galileo_cnav_message.cc
 * \brief  Implementation of a Galileo CNAV Data message as described in
 * Galileo High Accuracy Service E6-B Signal-In-Space Message Specification v1.2
 * (April 2020)
 * \author Carles Fernandez-Prades, 2020 cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "galileo_cnav_message.h"
#include <boost/crc.hpp>             // for boost::crc_basic, boost::crc_optimal
#include <boost/dynamic_bitset.hpp>  // for boost::dynamic_bitset
#include <algorithm>                 // for reverse


using CRC_Galileo_CNAV_type = boost::crc_optimal<24, 0x1864CFBU, 0x0, 0x0, false, false>;


bool Galileo_Cnav_Message::CRC_test(std::bitset<GALILEO_CNAV_BITS_FOR_CRC> bits, uint32_t checksum) const
{
    CRC_Galileo_CNAV_type CRC_Galileo;

    // Galileo CNAV frame for CRC is not an integer multiple of bytes
    // it needs to be filled with zeroes at the start of the frame.
    // This operation is done in the transformation from bits to bytes
    // using boost::dynamic_bitset.
    boost::dynamic_bitset<unsigned char> frame_bits(std::string(bits.to_string()));

    std::vector<unsigned char> bytes;
    boost::to_block_range(frame_bits, std::back_inserter(bytes));
    std::reverse(bytes.begin(), bytes.end());

    CRC_Galileo.process_bytes(bytes.data(), GALILEO_CNAV_BYTES_FOR_CRC);

    const uint32_t crc_computed = CRC_Galileo.checksum();
    if (checksum == crc_computed)
        {
            return true;
        }
    return false;
}


void Galileo_Cnav_Message::read_HAS_page(const std::string& page_string)
{
    const std::string has_page_bits = page_string.substr(0, GALILEO_CNAV_BITS_FOR_CRC);
    const std::string CRC_data = page_string.substr(GALILEO_CNAV_BITS_FOR_CRC, 24);
    const std::bitset<GALILEO_CNAV_BITS_FOR_CRC> Word_for_CRC_bits(has_page_bits);
    const std::bitset<24> checksum(CRC_data);
    if (CRC_test(Word_for_CRC_bits, checksum.to_ulong()) == true)
        {
            flag_CRC_test = true;
            // CRC correct: Read HAS page header
        }
    else
        {
            flag_CRC_test = false;
        }
}
