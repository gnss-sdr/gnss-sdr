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
#include <algorithm>                 // for reverse, find


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
    const std::string CRC_data = page_string.substr(GALILEO_CNAV_BITS_FOR_CRC, GALILEO_CNAV_CRC_LENGTH);
    const std::bitset<GALILEO_CNAV_BITS_FOR_CRC> Word_for_CRC_bits(has_page_bits);
    const std::bitset<GALILEO_CNAV_CRC_LENGTH> checksum(CRC_data);
    if (CRC_test(Word_for_CRC_bits, checksum.to_ulong()) == true)
        {
            d_flag_CRC_test = true;
            // CRC correct: Read HAS page header
            read_HAS_page_header(page_string.substr(GALILEO_CNAV_PAGE_RESERVED_BITS, GALILEO_CNAV_PAGE_HEADER_BITS));
            bool use_has = false;
            d_test_mode = false;
            switch (d_has_page_status)
                {
                case 0:  // HAS is in Test Mode
                    use_has = true;
                    d_test_mode = true;
                    break;
                case 1:  // HAS is in Operational Mode
                    use_has = true;
                    break;
                default:
                    break;
                }
            if (use_has)
                {
                    process_HAS_page(page_string.substr(GALILEO_CNAV_PAGE_RESERVED_BITS + GALILEO_CNAV_PAGE_HEADER_BITS, GALILEO_CNAV_MESSAGE_BITS_PER_PAGE));
                }
        }
    else
        {
            d_flag_CRC_test = false;
        }
}


void Galileo_Cnav_Message::read_HAS_page_header(const std::string& page_string)
{
    // check if dummy
    if (page_string == "101011110011101111000011")  // Equivalent to AF3BC3
        {
            d_page_dummy = true;
        }
    else
        {
            d_page_dummy = false;
        }
    if (!d_page_dummy)
        {
            const std::bitset<GALILEO_CNAV_PAGE_HEADER_BITS> has_page_header(page_string);
            d_has_page_status = read_has_page_header_parameter(has_page_header, GALILEO_HAS_STATUS);
            d_received_message_type = read_has_page_header_parameter(has_page_header, GALILEO_HAS_MESSAGE_TYPE);
            d_received_message_id = read_has_page_header_parameter(has_page_header, GALILEO_HAS_MESSAGE_ID);
            d_received_message_size = read_has_page_header_parameter(has_page_header, GALILEO_HAS_MESSAGE_SIZE);
            d_received_message_page_id = read_has_page_header_parameter(has_page_header, GALILEO_HAS_MESSAGE_PAGE_ID);
        }
}


void Galileo_Cnav_Message::process_HAS_page(const std::string& page_string)
{
    if (d_current_message_id == d_received_message_id)
        {
            // if receiver pid was not there, store it.
            if (std::find(d_list_pid.begin(), d_list_pid.end(), d_received_message_page_id) == d_list_pid.end())
                {
                    if (d_received_message_type == 1)  // contains satellite corrections
                        {
                            d_received_encoded_messages++;
                            d_list_pid.push_back(d_received_message_page_id);
                            // Store encoded page
                            d_encoded_message_type_1 += std::string(page_string);
                        }
                }
        }
    else
        {
            // Start new message
            d_current_message_id = d_received_message_id;
            d_received_encoded_messages = 0;
            d_new_message = false;
            d_current_message_size = d_received_message_size;
            // erase stored pages and start storing again
            d_encoded_message_type_1.clear();
            d_list_pid.clear();
            if (d_received_message_type == 1)
                {
                    d_encoded_message_type_1.reserve(GALILEO_CNAV_MAX_NUMBER_ENCODED_BLOCKS * GALILEO_CNAV_MESSAGE_BITS_PER_PAGE);
                    d_received_encoded_messages++;
                    d_list_pid.push_back(d_received_message_page_id);
                    d_encoded_message_type_1 += std::string(page_string);
                }
        }

    if (d_received_encoded_messages == d_current_message_size)
        {
            // we have a full encoded message stored in d_encoded_message_type_1
            d_received_encoded_messages = 0;
            d_current_message_id = 0;
            d_new_message = true;
            decode_message_type1();
        }
}


void Galileo_Cnav_Message::decode_message_type1()
{
    // TODO: Reed-Solomon decoding of d_encoded_message_type_1
    // TODO: reordering
    // decoded_message_type1 = ...
    // read_HAS_message_type1(decoded_message_type1);
}


void Galileo_Cnav_Message::read_HAS_message_type1(const std::string& message_string)
{
    d_HAS_data = Galileo_HAS_data();
    read_MT1_header(message_string);
    read_MT1_body(message_string);
}


void Galileo_Cnav_Message::read_MT1_header(const std::string& message_string)
{
    const std::bitset<GALILEO_CNAV_MT1_HEADER_BITS> has_mt1_header(message_string);
    d_HAS_data.header.toh = read_has_message_header_parameter_uint16(has_mt1_header, GALILEO_MT1_HEADER_TOH);
    d_HAS_data.header.mask_id = read_has_message_header_parameter_uint8(has_mt1_header, GALILEO_MT1_HEADER_MASK_ID);
    d_HAS_data.header.iod_id = read_has_message_header_parameter_uint8(has_mt1_header, GALILEO_MT1_HEADER_IOD_ID);
    d_HAS_data.header.mask_flag = read_has_message_header_parameter_bool(has_mt1_header, GALILEO_MT1_HEADER_MASK_FLAG);
    d_HAS_data.header.orbit_correction_flag = read_has_message_header_parameter_bool(has_mt1_header, GALILEO_MT1_HEADER_ORBIT_CORRECTION_FLAG);
    d_HAS_data.header.clock_fullset_flag = read_has_message_header_parameter_bool(has_mt1_header, GALILEO_MT1_HEADER_CLOCK_FULLSET_FLAG);
    d_HAS_data.header.clock_subset_flag = read_has_message_header_parameter_bool(has_mt1_header, GALILEO_MT1_HEADER_CLOCK_SUBSET_FLAG);
    d_HAS_data.header.code_bias_flag = read_has_message_header_parameter_bool(has_mt1_header, GALILEO_MT1_HEADER_CODE_BIAS_FLAG);
    d_HAS_data.header.phase_bias_flag = read_has_message_header_parameter_bool(has_mt1_header, GALILEO_MT1_HEADER_PHASE_BIAS_FLAG);
    d_HAS_data.header.ura_flag = read_has_message_header_parameter_bool(has_mt1_header, GALILEO_MT1_HEADER_URA_FLAG);
}


void Galileo_Cnav_Message::read_MT1_body(const std::string& message_string)
{
    auto message = std::string(message_string.begin() + GALILEO_CNAV_MT1_HEADER_BITS, message_string.end());  // Remove header
    if (d_HAS_data.header.mask_flag)
        {
            // read mask
            // size_t mask_flag_size = X;
            // message = std::string(message.begin() + mask_flag_size, message.end());  // Remove mask_flag
        }
    if (d_HAS_data.header.orbit_correction_flag)
        {
            // read orbit corrections
        }
    if (d_HAS_data.header.clock_fullset_flag)
        {
            // read clock full-set corrections
        }
    if (d_HAS_data.header.clock_subset_flag)
        {
            // read clock subset corrections
        }
    if (d_HAS_data.header.code_bias_flag)
        {
            // read code bias
        }
    if (d_HAS_data.header.phase_bias_flag)
        {
            // read phase bias
        }
    if (d_HAS_data.header.ura_flag)
        {
            // read URA
        }
}


uint8_t Galileo_Cnav_Message::read_has_page_header_parameter(std::bitset<GALILEO_CNAV_PAGE_HEADER_BITS> bits, const std::pair<int32_t, int32_t>& parameter) const
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


uint8_t Galileo_Cnav_Message::read_has_message_header_parameter_uint8(std::bitset<GALILEO_CNAV_MT1_HEADER_BITS> bits, const std::pair<int32_t, int32_t>& parameter) const
{
    uint8_t value = 0U;
    for (int j = 0; j < parameter.second; j++)
        {
            value <<= 1U;  // shift left
            if (static_cast<int>(bits[GALILEO_CNAV_MT1_HEADER_BITS - parameter.first - j]) == 1)
                {
                    value += 1;  // insert the bit
                }
        }
    return value;
}


uint16_t Galileo_Cnav_Message::read_has_message_header_parameter_uint16(std::bitset<GALILEO_CNAV_MT1_HEADER_BITS> bits, const std::pair<int32_t, int32_t>& parameter) const
{
    uint16_t value = 0U;
    for (int j = 0; j < parameter.second; j++)
        {
            value <<= 1U;  // shift left
            if (static_cast<int>(bits[GALILEO_CNAV_MT1_HEADER_BITS - parameter.first - j]) == 1)
                {
                    value += 1;  // insert the bit
                }
        }
    return value;
}


bool Galileo_Cnav_Message::read_has_message_header_parameter_bool(std::bitset<GALILEO_CNAV_MT1_HEADER_BITS> bits, const std::pair<int32_t, int32_t>& parameter) const
{
    bool value = false;
    if (static_cast<int>(bits[GALILEO_CNAV_MT1_HEADER_BITS - parameter.first]) == 1)
        {
            value = true;
        }
    return value;
}
