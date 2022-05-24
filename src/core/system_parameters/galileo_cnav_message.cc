/*!
 * \file galileo_cnav_message.cc
 * \brief  Implementation of a Galileo CNAV Data message as described in
 * Galileo High Accuracy Service Signal-In-Space Interface Control Document
 * (HAS SIS ICD) Issue 1.0, May 2022
 * \author Carles Fernandez-Prades, 2020-2022 cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "galileo_cnav_message.h"
#include <boost/crc.hpp>             // for boost::crc_basic, boost::crc_optimal
#include <boost/dynamic_bitset.hpp>  // for boost::dynamic_bitset
#include <glog/logging.h>
#include <algorithm>  // for reverse
#include <limits>
#include <vector>

using CRC_Galileo_CNAV_type = boost::crc_optimal<24, 0x1864CFBU, 0x0, 0x0, false, false>;


bool Galileo_Cnav_Message::CRC_test(const std::bitset<GALILEO_CNAV_BITS_FOR_CRC>& bits, uint32_t checksum) const
{
    CRC_Galileo_CNAV_type crc_galileo_e6b;

    // Galileo CNAV frame for CRC is not an integer multiple of bytes
    // it needs to be filled with zeroes at the start of the frame.
    // This operation is done in the transformation from bits to bytes
    // using boost::dynamic_bitset.
    boost::dynamic_bitset<uint8_t> frame_bits(bits.to_string());

    std::vector<uint8_t> bytes;
    bytes.reserve(GALILEO_CNAV_BYTES_FOR_CRC);
    boost::to_block_range(frame_bits, std::back_inserter(bytes));
    std::reverse(bytes.begin(), bytes.end());

    crc_galileo_e6b.process_bytes(bytes.data(), GALILEO_CNAV_BYTES_FOR_CRC);

    const uint32_t crc_computed = crc_galileo_e6b.checksum();
    if (checksum == crc_computed)
        {
            return true;
        }
    return false;
}


void Galileo_Cnav_Message::read_HAS_page(const std::string& page_string)
{
    const std::string has_page_bits = page_string.substr(0, GALILEO_CNAV_BITS_FOR_CRC);
    const std::string CRC_data = page_string.substr(GALILEO_CNAV_BITS_FOR_CRC, GALILEO_CNAV_CRC_LENGTH);
    const std::bitset<GALILEO_CNAV_BITS_FOR_CRC> Word_for_CRC_bits(has_page_bits);
    const std::bitset<GALILEO_CNAV_CRC_LENGTH> checksum(CRC_data);
    d_new_HAS_page = false;
    has_page = Galileo_HAS_page();
    has_page.tow = std::numeric_limits<uint32_t>::max();  // Unknown
    d_flag_CRC_test = CRC_test(Word_for_CRC_bits, checksum.to_ulong());
    if (d_flag_CRC_test == true)
        {
            // CRC correct: Read 24 bits of HAS page header
            read_HAS_page_header(page_string.substr(GALILEO_CNAV_PAGE_RESERVED_BITS, GALILEO_CNAV_PAGE_HEADER_BITS));
            bool use_has = false;
            d_test_mode = false;
            // HAS status as defined in HAS SIS ICD v1.0 Table 9 - HASS values and corresponding semantic
            if (!d_page_dummy)
                {
                    switch (d_has_page_status)
                        {
                        case 0:  // HAS is in Test Mode
                            use_has = true;
                            d_test_mode = true;
                            break;
                        case 1:  // HAS is in Operational Mode
                            use_has = true;
                            break;
                        case 2:  // HAS is in "reserved" status
                        case 3:  // Do not use HAS
                        default:
                            break;
                        }
                }
            if (use_has or d_page_dummy)
                {
                    // Store the 424 bits of encoded data (CNAV page) and the page header
                    has_page.has_message_string = page_string.substr(GALILEO_CNAV_PAGE_RESERVED_BITS + GALILEO_CNAV_PAGE_HEADER_BITS, GALILEO_CNAV_MESSAGE_BITS_PER_PAGE);
                    if (!d_page_dummy)
                        {
                            has_page.has_status = d_has_page_status;
                            has_page.reserved = d_has_reserved;
                            has_page.message_type = d_received_message_type;
                            has_page.message_id = d_received_message_id;
                            has_page.message_size = d_received_message_size;
                            has_page.message_page_id = d_received_message_page_id;
                        }
                    d_new_HAS_page = true;
                }
        }
}


void Galileo_Cnav_Message::read_HAS_page_header(const std::string& page_string)
{
    // check if dummy
    if (page_string == "101011110011101111000011")  // Equivalent to AF3BC3
        {
            d_page_dummy = true;
            DLOG(INFO) << "HAS page with dummy header received.";
        }
    else
        {
            d_page_dummy = false;
        }
    if (!d_page_dummy)
        {
            // HAS SIS ICD v1.0 Table 7: HAS page header
            const std::bitset<GALILEO_CNAV_PAGE_HEADER_BITS> has_page_header(page_string);
            d_has_page_status = read_has_page_header_parameter(has_page_header, GALILEO_HAS_STATUS);
            d_has_reserved = read_has_page_header_parameter(has_page_header, GALILEO_HAS_RESERVED);
            d_received_message_type = read_has_page_header_parameter(has_page_header, GALILEO_HAS_MESSAGE_TYPE);
            d_received_message_id = read_has_page_header_parameter(has_page_header, GALILEO_HAS_MESSAGE_ID);
            d_received_message_size = read_has_page_header_parameter(has_page_header, GALILEO_HAS_MESSAGE_SIZE) + 1;  // "0" means 1
            d_received_message_page_id = read_has_page_header_parameter(has_page_header, GALILEO_HAS_MESSAGE_PAGE_ID);

            DLOG(INFO) << "HAS page header received " << page_string << ":\n"
                       << "d_has_page_status: " << static_cast<float>(d_has_page_status) << "\n"
                       << "d_has_reserved: " << static_cast<float>(d_has_reserved) << "\n"
                       << "d_received_message_type: " << static_cast<float>(d_received_message_type) << "\n"
                       << "d_received_message_id: " << static_cast<float>(d_received_message_id) << "\n"
                       << "d_received_message_size: " << static_cast<float>(d_received_message_size) << "\n"
                       << "d_received_message_page_id: " << static_cast<float>(d_received_message_page_id);
        }
}


uint8_t Galileo_Cnav_Message::read_has_page_header_parameter(const std::bitset<GALILEO_CNAV_PAGE_HEADER_BITS>& bits, const std::pair<int32_t, int32_t>& parameter) const
{
    uint8_t value = 0U;
    for (int j = 0; j < parameter.second; j++)
        {
            value <<= 1U;  // shift left
            if (static_cast<int>(bits[GALILEO_CNAV_PAGE_HEADER_BITS - parameter.first - j]) == 1)
                {
                    value += 1;  // insert the bit
                }
        }
    return value;
}
