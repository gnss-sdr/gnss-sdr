/*!
 * \file galileo_cnav_message.cc
 * \brief  Implementation of a Galileo CNAV Data message as described in
 * Galileo High Accuracy Service E6-B Signal-In-Space Message Specification v1.2
 * (April 2020)
 * \author Carles Fernandez-Prades, 2020 cfernandez(at)cttc.es
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

#include "galileo_cnav_message.h"
#include <boost/crc.hpp>             // for boost::crc_basic, boost::crc_optimal
#include <boost/dynamic_bitset.hpp>  // for boost::dynamic_bitset
#include <algorithm>                 // for reverse, find
#include <numeric>                   // for accumulate


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
    int Nsat = 0;
    if (d_HAS_data.header.mask_flag)
        {
            // read mask
            d_HAS_data.Nsys = read_has_message_body_uint8(message.substr(0, HAS_MSG_NSYS_LENGTH));
            message = std::string(message.begin() + HAS_MSG_NSYS_LENGTH, message.end());
            d_HAS_data.gnss_id_mask.reserve(d_HAS_data.Nsys);
            d_HAS_data.cell_mask.reserve(d_HAS_data.Nsys);
            d_HAS_data.cell_mask_availability_flag.reserve(d_HAS_data.Nsys);
            d_HAS_data.nav_message.reserve(d_HAS_data.Nsys);
            for (uint8_t i = 0; i < d_HAS_data.Nsys; i++)
                {
                    d_HAS_data.gnss_id_mask[i] = read_has_message_body_uint8(message.substr(0, HAS_MSG_ID_MASK_LENGTH));
                    message = std::string(message.begin() + HAS_MSG_ID_MASK_LENGTH, message.end());
                    std::string msg = message.substr(0, HAS_MSG_SATELLITE_MASK_LENGTH);
                    d_HAS_data.satellite_mask[i] = read_has_message_body_uint64(msg);
                    int ones_in_satellite_mask = 0;
                    for (char c : msg)
                        {
                            if (c == '1')
                                {
                                    ones_in_satellite_mask++;
                                }
                        }
                    Nsat += ones_in_satellite_mask;
                    message = std::string(message.begin() + HAS_MSG_SATELLITE_MASK_LENGTH, message.end());

                    msg = message.substr(0, HAS_MSG_SIGNAL_MASK_LENGTH);
                    d_HAS_data.signal_mask[i] = read_has_message_body_uint16(msg);
                    int ones_in_signal_mask = 0;
                    for (char c : msg)
                        {
                            if (c == '1')
                                {
                                    ones_in_signal_mask++;
                                }
                        }
                    message = std::string(message.begin() + HAS_MSG_SIGNAL_MASK_LENGTH, message.end());

                    if (message.substr(0, 1) == "1")
                        {
                            d_HAS_data.cell_mask_availability_flag[i] = true;
                        }
                    else
                        {
                            d_HAS_data.cell_mask_availability_flag[i] = false;
                        }
                    message = std::string(message.begin() + 1, message.end());
                    int size_cell = ones_in_satellite_mask * ones_in_signal_mask;


                    d_HAS_data.cell_mask[i].reserve(ones_in_satellite_mask);
                    for (int s = 0; s < ones_in_satellite_mask; s++)
                        {
                            d_HAS_data.cell_mask[i][s].reserve(ones_in_signal_mask);
                            for (int sig = 0; sig < ones_in_signal_mask; sig++)
                                {
                                    d_HAS_data.cell_mask[i][s][sig] = (message[sig] == '1' ? true : false);
                                }
                        }
                    message = std::string(message.begin() + size_cell, message.end());

                    d_HAS_data.nav_message[i] = read_has_message_body_uint8(message.substr(0, HAS_MSG_NAV_MESSAGE_LENGTH));
                    message = std::string(message.begin() + HAS_MSG_NAV_MESSAGE_LENGTH, message.end());
                }
        }
    if (d_HAS_data.header.orbit_correction_flag)
        {
            // read orbit corrections
            d_HAS_data.validity_interval_index_orbit_corrections = read_has_message_body_uint8(message.substr(0, HAS_MSG_VALIDITY_INDEX_LENGTH));
            message = std::string(message.begin() + HAS_MSG_VALIDITY_INDEX_LENGTH, message.end());
            d_HAS_data.gnss_iod.reserve(Nsat);
            d_HAS_data.delta_radial.reserve(Nsat);
            d_HAS_data.delta_along_track.reserve(Nsat);
            d_HAS_data.delta_cross_track.reserve(Nsat);
            for (int i = 0; i < Nsat; i++)
                {
                    if (d_HAS_data.gnss_id_mask[i] == HAS_MSG_GPS_SYSTEM)
                        {
                            d_HAS_data.gnss_iod[i] = read_has_message_body_uint16(message.substr(0, HAS_MSG_IOD_GPS_LENGTH));
                            message = std::string(message.begin() + HAS_MSG_IOD_GPS_LENGTH, message.end());
                        }
                    if (d_HAS_data.gnss_id_mask[i] == HAS_MSG_GALILEO_SYSTEM)
                        {
                            d_HAS_data.gnss_iod[i] = read_has_message_body_uint16(message.substr(0, HAS_MSG_IOD_GAL_LENGTH));
                            message = std::string(message.begin() + HAS_MSG_IOD_GAL_LENGTH, message.end());
                        }
                    d_HAS_data.delta_radial[i] = read_has_message_body_int16(message.substr(0, HAS_MSG_DELTA_RADIAL_LENGTH));
                    message = std::string(message.begin() + HAS_MSG_DELTA_RADIAL_LENGTH, message.end());

                    d_HAS_data.delta_along_track[i] = read_has_message_body_int16(message.substr(0, HAS_MSG_DELTA_ALONG_TRACK_LENGTH));
                    message = std::string(message.begin() + HAS_MSG_DELTA_ALONG_TRACK_LENGTH, message.end());

                    d_HAS_data.delta_cross_track[i] = read_has_message_body_int16(message.substr(0, HAS_MSG_DELTA_CROSS_TRACK_LENGTH));
                    message = std::string(message.begin() + HAS_MSG_DELTA_CROSS_TRACK_LENGTH, message.end());
                }
        }
    if (d_HAS_data.header.clock_fullset_flag)
        {
            // read clock full-set corrections
            d_HAS_data.validity_interval_index_clock_fullset_corrections = read_has_message_body_uint8(message.substr(0, HAS_MSG_VALIDITY_INDEX_LENGTH));
            message = std::string(message.begin() + HAS_MSG_VALIDITY_INDEX_LENGTH, message.end());

            d_HAS_data.delta_clock_c0_multiplier.reserve(d_HAS_data.Nsys);
            for (uint8_t i = 0; i < d_HAS_data.Nsys; i++)
                {
                    if (d_HAS_data.gnss_id_mask[i] != HAS_MSG_GALILEO_SYSTEM)
                        {
                            d_HAS_data.delta_clock_c0_multiplier[i] = read_has_message_body_uint8(message.substr(0, HAS_MSG_DELTA_CLOCK_C0_MULTIPLIER_LENGTH));
                            message = std::string(message.begin() + HAS_MSG_DELTA_CLOCK_C0_MULTIPLIER_LENGTH, message.end());
                        }
                }
            d_HAS_data.iod_change_flag.reserve(Nsat);
            d_HAS_data.delta_clock_c0.reserve(Nsat);
            for (int i = 0; i < Nsat; i++)
                {
                    d_HAS_data.iod_change_flag[i] = (message[0] == '1' ? true : false);
                    message = std::string(message.begin() + 1, message.end());
                    d_HAS_data.delta_clock_c0[i] = read_has_message_body_int16(message.substr(0, HAS_MSG_DELTA_CLOCK_C0_LENGTH));
                    message = std::string(message.begin() + HAS_MSG_DELTA_CLOCK_C0_LENGTH, message.end());
                }
        }
    if (d_HAS_data.header.clock_subset_flag)
        {
            // read clock subset corrections
            d_HAS_data.validity_interval_index_clock_subset_corrections = read_has_message_body_uint8(message.substr(0, HAS_MSG_VALIDITY_INDEX_LENGTH));
            message = std::string(message.begin() + HAS_MSG_VALIDITY_INDEX_LENGTH, message.end());

            d_HAS_data.Nsysprime = read_has_message_body_uint8(message.substr(0, HAS_MSG_NSYSPRIME_LENGTH));
            message = std::string(message.begin() + HAS_MSG_NSYSPRIME_LENGTH, message.end());

            d_HAS_data.gnss_id_clock_subset.reserve(d_HAS_data.Nsysprime);
            d_HAS_data.delta_clock_c0_multiplier_clock_subset.reserve(d_HAS_data.Nsysprime);
            d_HAS_data.satellite_submask.reserve(d_HAS_data.Nsysprime);
            d_HAS_data.iod_change_flag_clock_subset.reserve(d_HAS_data.Nsysprime);
            d_HAS_data.delta_clock_c0_clock_subset.reserve(d_HAS_data.Nsysprime);
            for (uint8_t i = 0; i < d_HAS_data.Nsysprime; i++)
                {
                    d_HAS_data.gnss_id_clock_subset[i] = read_has_message_body_uint8(message.substr(0, HAS_MSG_ID_CLOCK_SUBSET_LENGTH));
                    message = std::string(message.begin() + HAS_MSG_ID_CLOCK_SUBSET_LENGTH, message.end());
                    if (d_HAS_data.gnss_id_clock_subset[i] != HAS_MSG_GALILEO_SYSTEM)
                        {
                            d_HAS_data.delta_clock_c0_multiplier_clock_subset[i] = read_has_message_body_uint8(message.substr(0, HAS_MSG_DELTA_CLOCK_MULTIPLIER_SUBSET_LENGTH));
                            message = std::string(message.begin() + HAS_MSG_DELTA_CLOCK_MULTIPLIER_SUBSET_LENGTH, message.end());
                        }
                    int number_sats_this_gnss_id = 0;
                    for (uint8_t j = 0; j < d_HAS_data.Nsys; j++)
                        {
                            if (d_HAS_data.gnss_id_mask[j] == d_HAS_data.gnss_id_clock_subset[i])
                                {
                                    uint64_t n = d_HAS_data.satellite_mask[j];
                                    while (n)
                                        {
                                            number_sats_this_gnss_id += n & 1;
                                            n >>= 1;
                                        }
                                    break;
                                }
                        }

                    d_HAS_data.satellite_submask[i].reserve(number_sats_this_gnss_id);
                    for (int j = 0; j < number_sats_this_gnss_id; j++)
                        {
                            d_HAS_data.satellite_submask[i][j] = read_has_message_body_uint64(message.substr(0, 1));
                            message = std::string(message.begin() + 1, message.end());
                        }
                    d_HAS_data.iod_change_flag_clock_subset[i] = (message[0] == '1' ? true : false);
                    message = std::string(message.begin() + 1, message.end());

                    d_HAS_data.delta_clock_c0_clock_subset[i] = read_has_message_body_int16(message.substr(0, HAS_MSG_DELTA_CLOCK_C0_SUBSET_LENGTH));
                    message = std::string(message.begin() + HAS_MSG_DELTA_CLOCK_C0_SUBSET_LENGTH, message.end());
                }
        }
    if (d_HAS_data.header.code_bias_flag)
        {
            // read code bias
            d_HAS_data.validity_interval_index_code_bias_corrections = read_has_message_body_uint8(message.substr(0, HAS_MSG_VALIDITY_INDEX_LENGTH));
            message = std::string(message.begin() + HAS_MSG_VALIDITY_INDEX_LENGTH, message.end());
            std::vector<int> number_sats(d_HAS_data.Nsys, 0);
            std::vector<int> number_codes(d_HAS_data.Nsys, 0);
            for (int sys = 0; sys < d_HAS_data.Nsys; sys++)
                {
                    int number_sats_this_gnss_id = 0;
                    int number_signals_this_gnss_id = 0;
                    if (d_HAS_data.cell_mask_availability_flag[sys] == true)
                        {
                            uint64_t n = d_HAS_data.satellite_mask[sys];
                            while (n)
                                {
                                    number_sats_this_gnss_id += n & 1;
                                    n >>= 1;
                                }
                            uint64_t m = d_HAS_data.signal_mask[sys];
                            while (m)
                                {
                                    number_signals_this_gnss_id += m & 1;
                                    m >>= 1;
                                }
                        }
                    else
                        {
                            number_sats_this_gnss_id = HAS_MSG_MAX_SATS;
                            number_signals_this_gnss_id = HAS_MSG_MAX_SIGNALS;
                        }
                    number_sats[sys] = number_sats_this_gnss_id;
                    number_codes[sys] = number_signals_this_gnss_id;
                }
            int Nsat_b = std::accumulate(number_sats.begin(), number_sats.end(), 0);

            d_HAS_data.code_bias.reserve(Nsat_b);
            int sat = 0;
            for (int sys = 0; sys < d_HAS_data.Nsys; sys++)
                {
                    d_HAS_data.code_bias[sat].reserve(number_codes[sys]);
                    for (int c = 0; c < number_codes[sys]; c++)
                        {
                            d_HAS_data.code_bias[sat][c] = read_has_message_body_int16(message.substr(0, HAS_MSG_CODE_BIAS_LENGTH));
                            message = std::string(message.begin() + HAS_MSG_CODE_BIAS_LENGTH, message.end());
                            sat += 1;
                        }
                }
        }
    if (d_HAS_data.header.phase_bias_flag)
        {
            // read phase bias
            d_HAS_data.validity_interval_index_phase_bias_corrections = read_has_message_body_uint8(message.substr(0, HAS_MSG_VALIDITY_INDEX_LENGTH));
            message = std::string(message.begin() + HAS_MSG_VALIDITY_INDEX_LENGTH, message.end());

            std::vector<int> number_sats(d_HAS_data.Nsys, 0);
            std::vector<int> number_phases(d_HAS_data.Nsys, 0);
            for (int sys = 0; sys < d_HAS_data.Nsys; sys++)
                {
                    int number_sats_this_gnss_id = 0;
                    int number_signals_this_gnss_id = 0;
                    if (d_HAS_data.cell_mask_availability_flag[sys] == true)
                        {
                            uint64_t n = d_HAS_data.satellite_mask[sys];
                            while (n)
                                {
                                    number_sats_this_gnss_id += n & 1;
                                    n >>= 1;
                                }
                            uint64_t m = d_HAS_data.signal_mask[sys];
                            while (m)
                                {
                                    number_signals_this_gnss_id += m & 1;
                                    m >>= 1;
                                }
                        }
                    else
                        {
                            number_sats_this_gnss_id = HAS_MSG_MAX_SATS;
                            number_signals_this_gnss_id = HAS_MSG_MAX_SIGNALS;
                        }
                    number_sats[sys] = number_sats_this_gnss_id;
                    number_phases[sys] = number_signals_this_gnss_id;
                }
            int Nsat_p = std::accumulate(number_sats.begin(), number_sats.end(), 0);

            d_HAS_data.phase_bias.reserve(Nsat_p);
            d_HAS_data.phase_discontinuity_indicator.reserve(Nsat_p);
            int sat = 0;
            for (int sys = 0; sys < d_HAS_data.Nsys; sys++)
                {
                    d_HAS_data.phase_bias[sat].reserve(number_phases[sys]);
                    d_HAS_data.phase_discontinuity_indicator[sat].reserve(number_phases[sys]);
                    for (int p = 0; p < number_phases[sys]; p++)
                        {
                            d_HAS_data.phase_bias[sat][p] = read_has_message_body_int16(message.substr(0, HAS_MSG_PHASE_BIAS_LENGTH));
                            message = std::string(message.begin() + HAS_MSG_PHASE_BIAS_LENGTH, message.end());

                            d_HAS_data.phase_discontinuity_indicator[sat][p] = read_has_message_body_uint8(message.substr(0, HAS_MSG_PHASE_DISCONTINUITY_INDICATOR_LENGTH));
                            message = std::string(message.begin() + HAS_MSG_PHASE_DISCONTINUITY_INDICATOR_LENGTH, message.end());
                            sat += 1;
                        }
                }
        }
    if (d_HAS_data.header.ura_flag)
        {
            // read URA
            d_HAS_data.validity_interval_index_ura_corrections = read_has_message_body_uint8(message.substr(0, HAS_MSG_VALIDITY_INDEX_LENGTH));
            message = std::string(message.begin() + HAS_MSG_VALIDITY_INDEX_LENGTH, message.end());
            d_HAS_data.ura.reserve(Nsat);
            for (int i = 0; i < Nsat; i++)
                {
                    d_HAS_data.ura[i] = read_has_message_body_uint8(message.substr(0, HAS_MSG_URA_LENGTH));
                    message = std::string(message.begin() + HAS_MSG_URA_LENGTH, message.end());
                }
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


uint8_t Galileo_Cnav_Message::read_has_message_body_uint8(const std::string& bits) const
{
    uint8_t value = 0U;
    size_t len = bits.length();

    for (size_t j = 0; j < len; j++)
        {
            value <<= 1U;  // shift left
            if (static_cast<int>(bits[len - 1 - j]) == 1)
                {
                    value += 1;  // insert the bit
                }
        }
    return value;
}


uint16_t Galileo_Cnav_Message::read_has_message_body_uint16(const std::string& bits) const
{
    uint16_t value = 0U;
    size_t len = bits.length();

    for (size_t j = 0; j < len; j++)
        {
            value <<= 1U;  // shift left
            if (static_cast<int>(bits[len - 1 - j]) == 1)
                {
                    value += 1;  // insert the bit
                }
        }
    return value;
}


uint64_t Galileo_Cnav_Message::read_has_message_body_uint64(const std::string& bits) const
{
    uint64_t value = 0U;
    size_t len = bits.length();

    for (size_t j = 0; j < len; j++)
        {
            value <<= 1U;  // shift left
            if (static_cast<int>(bits[len - 1 - j]) == 1)
                {
                    value += 1;  // insert the bit
                }
        }
    return value;
}


int16_t Galileo_Cnav_Message::read_has_message_body_int16(const std::string& bits) const
{
    int16_t value = 0;
    size_t len = bits.length();

    // read the MSB and perform the sign extension
    if (static_cast<int>(bits[len - 1]) == 1)
        {
            value ^= 0xFFFF;  // 16 bits variable
        }
    else
        {
            value &= 0;
        }

    for (size_t j = 0; j < len; j++)
        {
            value *= 2;       // shift left the signed integer
            value &= 0xFFFE;  // reset the corresponding bit (for the 16 bits variable)
            if (static_cast<int>(bits[len - 1 - j]) == 1)
                {
                    value += 1;  // insert the bit
                }
        }

    return value;
}
