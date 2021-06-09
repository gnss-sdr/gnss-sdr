/*!
 * \file galileo_e6_has_msg_receiver.cc
 * \brief GNU Radio block that processes Galileo HAS message pages received from
 * Galileo E6B telemetry blocks. After successful decoding, sends the content to
 * the PVT block.
 * \author Javier Arribas, 2021. jarribas(at)cttc.es
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


#include "galileo_e6_has_msg_receiver.h"
#include "display.h"
#include "galileo_has_page.h"  // for Galileo_HAS_page
#include "gnss_sdr_make_unique.h"
#include "reed_solomon.h"
#include <boost/any.hpp>
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <algorithm>  // std::find
#include <cstddef>    // size_t
#include <numeric>    // std::accumulate
#include <sstream>    // std::stringstream
#include <typeinfo>   // typeid

#if HAS_GENERIC_LAMBDA
#else
#include <boost/bind/bind.hpp>
#endif


galileo_e6_has_msg_receiver_sptr galileo_e6_has_msg_receiver_make()
{
    return galileo_e6_has_msg_receiver_sptr(new galileo_e6_has_msg_receiver());
}


galileo_e6_has_msg_receiver::galileo_e6_has_msg_receiver() : gr::block("galileo_e6_has_msg_receiver", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    // register Gal E6 HAS input message port from telemetry blocks
    this->message_port_register_in(pmt::mp("E6_HAS_from_TLM"));
    this->set_msg_handler(pmt::mp("E6_HAS_from_TLM"),
#if HAS_GENERIC_LAMBDA
        [this](auto&& PH1) { msg_handler_galileo_e6_has(PH1); });
#else
#if USE_BOOST_BIND_PLACEHOLDERS
        boost::bind(&galileo_e6_has_msg_receiver::msg_handler_galileo_e6_has, this, boost::placeholders::_1));
#else
        boost::bind(&galileo_e6_has_msg_receiver::msg_handler_galileo_e6_has, this, _1));
#endif
#endif

    // register Gal E6 processed HAS async output message port towards PVT
    this->message_port_register_out(pmt::mp("E6_HAS_to_PVT"));

    // initialize Reed-Solomon decoder
    d_rs = std::make_unique<ReedSolomon>();
}


void galileo_e6_has_msg_receiver::msg_handler_galileo_e6_has(const pmt::pmt_t& msg)
{
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with msg_handler_galileo_e6_has function called by the scheduler

    try
        {
            const size_t msg_type_hash_code = pmt::any_ref(msg).type().hash_code();
            if (msg_type_hash_code == typeid(std::shared_ptr<Galileo_HAS_page>).hash_code())
                {
                    const auto HAS_data_page = boost::any_cast<std::shared_ptr<Galileo_HAS_page>>(pmt::any_ref(msg));
                    DLOG(INFO) << "New HAS page received:  "
                               << "Status: " << static_cast<float>(HAS_data_page->has_status) << ", "
                               << "MT: " << static_cast<float>(HAS_data_page->message_type) << ", "
                               << "MID: " << static_cast<float>(HAS_data_page->message_id) << ", "
                               << "MS: " << static_cast<float>(HAS_data_page->message_size) << ", "
                               << "PID: " << static_cast<float>(HAS_data_page->message_page_id);

                    process_HAS_page(*HAS_data_page.get());
                }
            else
                {
                    LOG(WARNING) << "galileo_e6_has_msg_receiver received an unknown object type!";
                }
        }
    catch (const boost::bad_any_cast& e)
        {
            LOG(WARNING) << "galileo_e6_has_msg_receiver Bad any_cast: " << e.what();
        }

    //  Send the resulting decoded HAS data (if available) to PVT
    if (d_new_message == true)
        {
            auto has_data_ptr = std::make_shared<Galileo_HAS_data>(d_HAS_data);
            this->message_port_pub(pmt::mp("E6_HAS_to_PVT"), pmt::make_any(has_data_ptr));
            d_new_message = false;
            DLOG(INFO) << "HAS message sent to the PVT block through the E6_HAS_to_PVT async message port";
        }
}


void galileo_e6_has_msg_receiver::process_HAS_page(const Galileo_HAS_page& has_page)
{
    if (has_page.has_status == 0 || has_page.has_status == 1)
        {
            std::string page_string(has_page.has_message_string);
            if (has_page.message_page_id != 0)  // PID=0 is reserved, ignore it
                {
                    if (has_page.message_type == 1)  // contains satellite corrections
                        {
                            if (has_page.message_id < 32)  // MID range is from 0 to 31
                                {
                                    if (std::find(d_received_pids[has_page.message_id].begin(), d_received_pids[has_page.message_id].end(), has_page.message_page_id) == d_received_pids[has_page.message_id].end())
                                        {
                                            // New pid! Annotate it.
                                            d_received_pids[has_page.message_id].push_back(has_page.message_page_id);
                                            for (int k = 0; k < GALILEO_CNAV_OCTETS_IN_SUBPAGE; k++)
                                                {
                                                    std::string bits8 = page_string.substr(k * 8, 8);
                                                    std::bitset<8> bs(bits8);
                                                    d_C_matrix[has_page.message_id][has_page.message_page_id - 1][k] = static_cast<uint8_t>(bs.to_ulong());
                                                }
                                        }
                                }
                        }
                }
        }

    // If we have received for this message ID a number of pages equal to the message size
    if (d_received_pids[has_page.message_id].size() == has_page.message_size)
        {
            // Try to decode the message
            int res = decode_message_type1(has_page.message_id, has_page.message_size);

            if (res == 0)
                {
                    // Successful decoding, we have a valid HAS message stored at d_HAS_data
                    std::cout << TEXT_MAGENTA << "New Galileo HAS message type " << static_cast<float>(has_page.message_id)
                              << " received and successfully decoded" << TEXT_RESET << '\n';
                    d_new_message = true;
                }
            else
                {
                    d_new_message = false;
                }
        }
}


int galileo_e6_has_msg_receiver::decode_message_type1(uint8_t message_id, uint8_t message_size)
{
    DLOG(INFO) << "Start decoding of a HAS message";

    // Compute erasure positions
    std::vector<int> erasure_positions;
    erasure_positions.reserve(223);  // Maximum erasure positions ( = number of parity symbols in a block)

    for (uint8_t i = 1; i < message_size + 1; i++)  // we know that from message_size to 32, the value is 0
        {
            if (std::find(d_received_pids[message_id].begin(), d_received_pids[message_id].end(), i) == d_received_pids[message_id].end())
                {
                    erasure_positions.push_back(i - 1);
                }
        }
    for (int i = 33; i < 256; i++)
        {
            if (std::find(d_received_pids[message_id].begin(), d_received_pids[message_id].end(), static_cast<uint8_t>(i)) == d_received_pids[message_id].end())
                {
                    erasure_positions.push_back(i - 1);
                }
        }

    DLOG(INFO) << debug_print_vector("List of received PIDs", d_received_pids[message_id]);
    DLOG(INFO) << debug_print_vector("erasure_positions", erasure_positions);
    DLOG(INFO) << debug_print_matrix("d_C_matrix produced", d_C_matrix[message_id]);

    // Reset HAS decoded message matrix
    d_M_matrix = {GALILEO_CNAV_INFORMATION_VECTOR_LENGTH, std::vector<uint8_t>(GALILEO_CNAV_OCTETS_IN_SUBPAGE)};

    // Vertical decoding of d_C_matrix
    for (int col = 0; col < GALILEO_CNAV_OCTETS_IN_SUBPAGE; col++)
        {
            std::vector<uint8_t> C_column(GALILEO_CNAV_MAX_NUMBER_SYMBOLS_ENCODED_BLOCK, 0);
            for (auto pid : d_received_pids[message_id])
                {
                    C_column[pid - 1] = d_C_matrix[message_id][pid - 1][col];
                }

            DLOG(INFO) << debug_print_vector("C_column entering the decoder", C_column);

            int result = d_rs->decode(C_column, erasure_positions);

            if (result < 0)
                {
                    DLOG(ERROR) << "Decoding of HAS page failed";
                    return -1;
                }
            DLOG(INFO) << "Successful HAS page decoding";

            std::vector<uint8_t> M_column(C_column.begin(), C_column.begin() + GALILEO_CNAV_INFORMATION_VECTOR_LENGTH);
            for (int i = 0; i < GALILEO_CNAV_INFORMATION_VECTOR_LENGTH; i++)
                {
                    d_M_matrix[i][col] = M_column[i];
                }
        }

    DLOG(INFO) << debug_print_matrix("M_matrix", d_M_matrix);

    // Form the decoded HAS message by reading rows of d_M_matrix
    std::string decoded_message_type_1;
    decoded_message_type_1.reserve(message_size * GALILEO_CNAV_OCTETS_IN_SUBPAGE * 8);
    for (uint8_t row = 0; row < message_size; row++)
        {
            for (int col = 0; col < GALILEO_CNAV_OCTETS_IN_SUBPAGE; col++)
                {
                    std::bitset<8> bs(d_M_matrix[row][col]);
                    decoded_message_type_1 += bs.to_string();
                }
        }
    DLOG(INFO) << "Decoded message ID " << static_cast<float>(message_id)
               << " (size: " << static_cast<float>(message_size) << ") with body: "
               << std::string(decoded_message_type_1.begin() + GALILEO_CNAV_MT1_HEADER_BITS, decoded_message_type_1.end());

    // reset data for next decoding
    d_C_matrix[message_id] = {GALILEO_CNAV_MAX_NUMBER_SYMBOLS_ENCODED_BLOCK, std::vector<uint8_t>(GALILEO_CNAV_OCTETS_IN_SUBPAGE, 0)};
    d_received_pids[message_id].clear();

    // Trigger HAS message content reading and fill the d_HAS_data object
    d_HAS_data = Galileo_HAS_data();
    read_MT1_header(decoded_message_type_1.substr(0, GALILEO_CNAV_MT1_HEADER_BITS));
    read_MT1_body(std::string(decoded_message_type_1.begin() + GALILEO_CNAV_MT1_HEADER_BITS, decoded_message_type_1.end()));

    return 0;
}


void galileo_e6_has_msg_receiver::read_MT1_header(const std::string& message_header)
{
    // ICD v1.2 Table 6: MT1 Message Header
    const std::bitset<GALILEO_CNAV_MT1_HEADER_BITS> has_mt1_header(message_header);
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

    DLOG(INFO) << "MT1 header " << message_header << ":  "
               << "TOH: " << static_cast<float>(d_HAS_data.header.toh) << ", "
               << "mask ID: " << static_cast<float>(d_HAS_data.header.mask_id) << ", "
               << "iod ID: " << static_cast<float>(d_HAS_data.header.iod_id) << ", "
               << "mask_flag: " << static_cast<float>(d_HAS_data.header.mask_flag) << ", "
               << "orbit_correction_flag: " << static_cast<float>(d_HAS_data.header.orbit_correction_flag) << ", "
               << "clock_fullset_flag: " << static_cast<float>(d_HAS_data.header.clock_fullset_flag) << ", "
               << "clock_subset_flag: " << static_cast<float>(d_HAS_data.header.clock_subset_flag) << ", "
               << "code_bias_flag: " << static_cast<float>(d_HAS_data.header.code_bias_flag) << ", "
               << "phase_bias_flag: " << static_cast<float>(d_HAS_data.header.phase_bias_flag) << ", "
               << "ura_flag: " << static_cast<float>(d_HAS_data.header.ura_flag);
}


void galileo_e6_has_msg_receiver::read_MT1_body(const std::string& message_body)
{
    // ICD v1.2 Table 7: MT1 Message Body.
    auto message = std::string(message_body);
    // int Nsat = 0;
    if (d_HAS_data.header.mask_flag)
        {
            // read mask
            d_HAS_data.Nsys = read_has_message_body_uint8(message.substr(0, HAS_MSG_NSYS_LENGTH));
            DLOG(INFO) << "Nsys " << static_cast<float>(d_HAS_data.Nsys);
            message = std::string(message.begin() + HAS_MSG_NSYS_LENGTH, message.end());
            d_HAS_data.gnss_id_mask.reserve(d_HAS_data.Nsys);
            d_HAS_data.cell_mask.reserve(d_HAS_data.Nsys);
            d_HAS_data.cell_mask_availability_flag.reserve(d_HAS_data.Nsys);
            d_HAS_data.nav_message.reserve(d_HAS_data.Nsys);
            for (uint8_t i = 0; i < d_HAS_data.Nsys; i++)
                {
                    //         d_HAS_data.gnss_id_mask[i] = read_has_message_body_uint8(message.substr(0, HAS_MSG_ID_MASK_LENGTH));
                    //         // DLOG(ERROR) << "GNSS ID" << static_cast<float>(i) << ": " << static_cast<float>(d_HAS_data.gnss_id_mask[i]);
                    //         message = std::string(message.begin() + HAS_MSG_ID_MASK_LENGTH, message.end());
                    //         std::string msg = message.substr(0, HAS_MSG_SATELLITE_MASK_LENGTH);
                    //         d_HAS_data.satellite_mask[i] = read_has_message_body_uint64(msg);
                    //         int ones_in_satellite_mask = 0;
                    //         for (char c : msg)
                    //             {
                    //                 if (c == '1')
                    //                     {
                    //                         ones_in_satellite_mask++;
                    //                     }
                    //             }
                    //         Nsat += ones_in_satellite_mask;
                    //         message = std::string(message.begin() + HAS_MSG_SATELLITE_MASK_LENGTH, message.end());
                    //
                    //         msg = message.substr(0, HAS_MSG_SIGNAL_MASK_LENGTH);
                    //         d_HAS_data.signal_mask[i] = read_has_message_body_uint16(msg);
                    //         int ones_in_signal_mask = 0;
                    //         for (char c : msg)
                    //             {
                    //                 if (c == '1')
                    //                     {
                    //                         ones_in_signal_mask++;
                    //                     }
                    //             }
                    //         message = std::string(message.begin() + HAS_MSG_SIGNAL_MASK_LENGTH, message.end());
                    //
                    //         if (message.substr(0, 1) == "1")
                    //             {
                    //                 d_HAS_data.cell_mask_availability_flag[i] = true;
                    //             }
                    //         else
                    //             {
                    //                 d_HAS_data.cell_mask_availability_flag[i] = false;
                    //             }
                    //         message = std::string(message.begin() + 1, message.end());
                    //         int size_cell = ones_in_satellite_mask * ones_in_signal_mask;
                    //
                    //         d_HAS_data.cell_mask[i].reserve(ones_in_satellite_mask);
                    //         for (int s = 0; s < ones_in_satellite_mask; s++)
                    //             {
                    //                 d_HAS_data.cell_mask[i][s].reserve(ones_in_signal_mask);
                    //                 for (int sig = 0; sig < ones_in_signal_mask; sig++)
                    //                     {
                    //                         d_HAS_data.cell_mask[i][s][sig] = (message[sig] == '1' ? true : false);
                    //                     }
                    //             }
                    //         message = std::string(message.begin() + size_cell, message.end());
                    //
                    //         d_HAS_data.nav_message[i] = read_has_message_body_uint8(message.substr(0, HAS_MSG_NAV_MESSAGE_LENGTH));
                    //         message = std::string(message.begin() + HAS_MSG_NAV_MESSAGE_LENGTH, message.end());
                }
        }
    // if (d_HAS_data.header.orbit_correction_flag)
    //     {
    //         // read orbit corrections
    //         d_HAS_data.validity_interval_index_orbit_corrections = read_has_message_body_uint8(message.substr(0, HAS_MSG_VALIDITY_INDEX_LENGTH));
    //         message = std::string(message.begin() + HAS_MSG_VALIDITY_INDEX_LENGTH, message.end());
    //         d_HAS_data.gnss_iod.reserve(Nsat);
    //         d_HAS_data.delta_radial.reserve(Nsat);
    //         d_HAS_data.delta_along_track.reserve(Nsat);
    //         d_HAS_data.delta_cross_track.reserve(Nsat);
    //         for (int i = 0; i < Nsat; i++)
    //             {
    //                 if (d_HAS_data.gnss_id_mask[i] == HAS_MSG_GPS_SYSTEM)
    //                     {
    //                         d_HAS_data.gnss_iod[i] = read_has_message_body_uint16(message.substr(0, HAS_MSG_IOD_GPS_LENGTH));
    //                         message = std::string(message.begin() + HAS_MSG_IOD_GPS_LENGTH, message.end());
    //                     }
    //                 if (d_HAS_data.gnss_id_mask[i] == HAS_MSG_GALILEO_SYSTEM)
    //                     {
    //                         d_HAS_data.gnss_iod[i] = read_has_message_body_uint16(message.substr(0, HAS_MSG_IOD_GAL_LENGTH));
    //                         message = std::string(message.begin() + HAS_MSG_IOD_GAL_LENGTH, message.end());
    //                     }
    //                 d_HAS_data.delta_radial[i] = read_has_message_body_int16(message.substr(0, HAS_MSG_DELTA_RADIAL_LENGTH));
    //                 message = std::string(message.begin() + HAS_MSG_DELTA_RADIAL_LENGTH, message.end());
    //
    //                 d_HAS_data.delta_along_track[i] = read_has_message_body_int16(message.substr(0, HAS_MSG_DELTA_ALONG_TRACK_LENGTH));
    //                 message = std::string(message.begin() + HAS_MSG_DELTA_ALONG_TRACK_LENGTH, message.end());
    //
    //                 d_HAS_data.delta_cross_track[i] = read_has_message_body_int16(message.substr(0, HAS_MSG_DELTA_CROSS_TRACK_LENGTH));
    //                 message = std::string(message.begin() + HAS_MSG_DELTA_CROSS_TRACK_LENGTH, message.end());
    //             }
    //     }
    // if (d_HAS_data.header.clock_fullset_flag)
    //     {
    //         // read clock full-set corrections
    //         d_HAS_data.validity_interval_index_clock_fullset_corrections = read_has_message_body_uint8(message.substr(0, HAS_MSG_VALIDITY_INDEX_LENGTH));
    //         message = std::string(message.begin() + HAS_MSG_VALIDITY_INDEX_LENGTH, message.end());
    //
    //         d_HAS_data.delta_clock_c0_multiplier.reserve(d_HAS_data.Nsys);
    //         for (uint8_t i = 0; i < d_HAS_data.Nsys; i++)
    //             {
    //                 if (d_HAS_data.gnss_id_mask[i] != HAS_MSG_GALILEO_SYSTEM)
    //                     {
    //                         d_HAS_data.delta_clock_c0_multiplier[i] = read_has_message_body_uint8(message.substr(0, HAS_MSG_DELTA_CLOCK_C0_MULTIPLIER_LENGTH));
    //                         message = std::string(message.begin() + HAS_MSG_DELTA_CLOCK_C0_MULTIPLIER_LENGTH, message.end());
    //                     }
    //             }
    //         d_HAS_data.iod_change_flag.reserve(Nsat);
    //         d_HAS_data.delta_clock_c0.reserve(Nsat);
    //         for (int i = 0; i < Nsat; i++)
    //             {
    //                 d_HAS_data.iod_change_flag[i] = (message[0] == '1' ? true : false);
    //                 message = std::string(message.begin() + 1, message.end());
    //                 d_HAS_data.delta_clock_c0[i] = read_has_message_body_int16(message.substr(0, HAS_MSG_DELTA_CLOCK_C0_LENGTH));
    //                 message = std::string(message.begin() + HAS_MSG_DELTA_CLOCK_C0_LENGTH, message.end());
    //             }
    //     }
    // if (d_HAS_data.header.clock_subset_flag)
    //     {
    //         // read clock subset corrections
    //         d_HAS_data.validity_interval_index_clock_subset_corrections = read_has_message_body_uint8(message.substr(0, HAS_MSG_VALIDITY_INDEX_LENGTH));
    //         message = std::string(message.begin() + HAS_MSG_VALIDITY_INDEX_LENGTH, message.end());
    //
    //         d_HAS_data.Nsysprime = read_has_message_body_uint8(message.substr(0, HAS_MSG_NSYSPRIME_LENGTH));
    //         DLOG(INFO) << "Nsysprime: " << static_cast<float>(d_HAS_data.Nsysprime);
    //         message = std::string(message.begin() + HAS_MSG_NSYSPRIME_LENGTH, message.end());
    //
    //         d_HAS_data.gnss_id_clock_subset.reserve(d_HAS_data.Nsysprime);
    //         d_HAS_data.delta_clock_c0_multiplier_clock_subset.reserve(d_HAS_data.Nsysprime);
    //         d_HAS_data.satellite_submask.reserve(d_HAS_data.Nsysprime);
    //         d_HAS_data.iod_change_flag_clock_subset.reserve(d_HAS_data.Nsysprime);
    //         d_HAS_data.delta_clock_c0_clock_subset.reserve(d_HAS_data.Nsysprime);
    //         for (uint8_t i = 0; i < d_HAS_data.Nsysprime; i++)
    //             {
    //                 d_HAS_data.gnss_id_clock_subset[i] = read_has_message_body_uint8(message.substr(0, HAS_MSG_ID_CLOCK_SUBSET_LENGTH));
    //                 message = std::string(message.begin() + HAS_MSG_ID_CLOCK_SUBSET_LENGTH, message.end());
    //                 if (d_HAS_data.gnss_id_clock_subset[i] != HAS_MSG_GALILEO_SYSTEM)
    //                     {
    //                         d_HAS_data.delta_clock_c0_multiplier_clock_subset[i] = read_has_message_body_uint8(message.substr(0, HAS_MSG_DELTA_CLOCK_MULTIPLIER_SUBSET_LENGTH));
    //                         message = std::string(message.begin() + HAS_MSG_DELTA_CLOCK_MULTIPLIER_SUBSET_LENGTH, message.end());
    //                     }
    //                 uint64_t number_sats_this_gnss_id = 0;
    //                 for (uint8_t j = 0; j < d_HAS_data.Nsys; j++)
    //                     {
    //                         if (d_HAS_data.gnss_id_mask[j] == d_HAS_data.gnss_id_clock_subset[i])
    //                             {
    //                                 uint64_t n = d_HAS_data.satellite_mask[j];
    //                                 while (n)
    //                                     {
    //                                         number_sats_this_gnss_id += n & 1;
    //                                         n >>= 1;
    //                                     }
    //                                 break;
    //                             }
    //                     }
    //
    //                 d_HAS_data.satellite_submask[i].reserve(number_sats_this_gnss_id);
    //                 for (uint64_t j = 0; j < number_sats_this_gnss_id; j++)
    //                     {
    //                         d_HAS_data.satellite_submask[i][j] = read_has_message_body_uint64(message.substr(0, 1));
    //                         message = std::string(message.begin() + 1, message.end());
    //                     }
    //                 d_HAS_data.iod_change_flag_clock_subset[i] = (message[0] == '1' ? true : false);
    //                 message = std::string(message.begin() + 1, message.end());
    //
    //                 d_HAS_data.delta_clock_c0_clock_subset[i] = read_has_message_body_int16(message.substr(0, HAS_MSG_DELTA_CLOCK_C0_SUBSET_LENGTH));
    //                 message = std::string(message.begin() + HAS_MSG_DELTA_CLOCK_C0_SUBSET_LENGTH, message.end());
    //             }
    //     }
    // if (d_HAS_data.header.code_bias_flag)
    //     {
    //         // read code bias
    //         d_HAS_data.validity_interval_index_code_bias_corrections = read_has_message_body_uint8(message.substr(0, HAS_MSG_VALIDITY_INDEX_LENGTH));
    //         message = std::string(message.begin() + HAS_MSG_VALIDITY_INDEX_LENGTH, message.end());
    //         std::vector<uint64_t> number_sats(d_HAS_data.Nsys, 0);
    //         std::vector<uint64_t> number_codes(d_HAS_data.Nsys, 0);
    //         for (int sys = 0; sys < d_HAS_data.Nsys; sys++)
    //             {
    //                 uint64_t number_sats_this_gnss_id = 0;
    //                 uint64_t number_signals_this_gnss_id = 0;
    //                 if (d_HAS_data.cell_mask_availability_flag[sys] == true)
    //                     {
    //                         uint64_t n = d_HAS_data.satellite_mask[sys];
    //                         while (n)
    //                             {
    //                                 number_sats_this_gnss_id += n & 1;
    //                                 n >>= 1;
    //                             }
    //                         uint64_t m = d_HAS_data.signal_mask[sys];
    //                         while (m)
    //                             {
    //                                 number_signals_this_gnss_id += m & 1;
    //                                 m >>= 1;
    //                             }
    //                     }
    //                 else
    //                     {
    //                         number_sats_this_gnss_id = HAS_MSG_MAX_SATS;
    //                         number_signals_this_gnss_id = HAS_MSG_MAX_SIGNALS;
    //                     }
    //                 number_sats[sys] = number_sats_this_gnss_id;
    //                 number_codes[sys] = number_signals_this_gnss_id;
    //             }
    //         uint64_t Nsat_b = std::accumulate(number_sats.begin(), number_sats.end(), 0ULL);
    //
    //         d_HAS_data.code_bias.reserve(Nsat_b);
    //         int sat = 0;
    //         for (int sys = 0; sys < d_HAS_data.Nsys; sys++)
    //             {
    //                 d_HAS_data.code_bias[sat].reserve(number_codes[sys]);
    //                 for (uint64_t c = 0; c < number_codes[sys]; c++)
    //                     {
    //                         d_HAS_data.code_bias[sat][c] = read_has_message_body_int16(message.substr(0, HAS_MSG_CODE_BIAS_LENGTH));
    //                         message = std::string(message.begin() + HAS_MSG_CODE_BIAS_LENGTH, message.end());
    //                         sat += 1;
    //                     }
    //             }
    //     }
    // if (d_HAS_data.header.phase_bias_flag)
    //     {
    //         // read phase bias
    //         d_HAS_data.validity_interval_index_phase_bias_corrections = read_has_message_body_uint8(message.substr(0, HAS_MSG_VALIDITY_INDEX_LENGTH));
    //         message = std::string(message.begin() + HAS_MSG_VALIDITY_INDEX_LENGTH, message.end());
    //
    //         std::vector<uint64_t> number_sats(d_HAS_data.Nsys, 0);
    //         std::vector<uint64_t> number_phases(d_HAS_data.Nsys, 0);
    //         for (int sys = 0; sys < d_HAS_data.Nsys; sys++)
    //             {
    //                 uint64_t number_sats_this_gnss_id = 0;
    //                 uint64_t number_signals_this_gnss_id = 0;
    //                 if (d_HAS_data.cell_mask_availability_flag[sys] == true)
    //                     {
    //                         uint64_t n = d_HAS_data.satellite_mask[sys];
    //                         while (n)
    //                             {
    //                                 number_sats_this_gnss_id += n & 1;
    //                                 n >>= 1;
    //                             }
    //                         uint64_t m = d_HAS_data.signal_mask[sys];
    //                         while (m)
    //                             {
    //                                 number_signals_this_gnss_id += m & 1;
    //                                 m >>= 1;
    //                             }
    //                     }
    //                 else
    //                     {
    //                         number_sats_this_gnss_id = HAS_MSG_MAX_SATS;
    //                         number_signals_this_gnss_id = HAS_MSG_MAX_SIGNALS;
    //                     }
    //                 number_sats[sys] = number_sats_this_gnss_id;
    //                 number_phases[sys] = number_signals_this_gnss_id;
    //             }
    //         uint64_t Nsat_p = std::accumulate(number_sats.begin(), number_sats.end(), 0ULL);
    //
    //         d_HAS_data.phase_bias.reserve(Nsat_p);
    //         d_HAS_data.phase_discontinuity_indicator.reserve(Nsat_p);
    //         int sat = 0;
    //         for (int sys = 0; sys < d_HAS_data.Nsys; sys++)
    //             {
    //                 d_HAS_data.phase_bias[sat].reserve(number_phases[sys]);
    //                 d_HAS_data.phase_discontinuity_indicator[sat].reserve(number_phases[sys]);
    //                 for (uint64_t p = 0; p < number_phases[sys]; p++)
    //                     {
    //                         d_HAS_data.phase_bias[sat][p] = read_has_message_body_int16(message.substr(0, HAS_MSG_PHASE_BIAS_LENGTH));
    //                         message = std::string(message.begin() + HAS_MSG_PHASE_BIAS_LENGTH, message.end());
    //
    //                         d_HAS_data.phase_discontinuity_indicator[sat][p] = read_has_message_body_uint8(message.substr(0, HAS_MSG_PHASE_DISCONTINUITY_INDICATOR_LENGTH));
    //                         message = std::string(message.begin() + HAS_MSG_PHASE_DISCONTINUITY_INDICATOR_LENGTH, message.end());
    //                         sat += 1;
    //                     }
    //             }
    //     }
    // if (d_HAS_data.header.ura_flag)
    //     {
    //         // read URA
    //         d_HAS_data.validity_interval_index_ura_corrections = read_has_message_body_uint8(message.substr(0, HAS_MSG_VALIDITY_INDEX_LENGTH));
    //         message = std::string(message.begin() + HAS_MSG_VALIDITY_INDEX_LENGTH, message.end());
    //         d_HAS_data.ura.reserve(Nsat);
    //         for (int i = 0; i < Nsat; i++)
    //             {
    //                 d_HAS_data.ura[i] = read_has_message_body_uint8(message.substr(0, HAS_MSG_URA_LENGTH));
    //                 message = std::string(message.begin() + HAS_MSG_URA_LENGTH, message.end());
    //             }
    //     }
}


uint8_t galileo_e6_has_msg_receiver::read_has_message_header_parameter_uint8(const std::bitset<GALILEO_CNAV_MT1_HEADER_BITS>& bits, const std::pair<int32_t, int32_t>& parameter) const
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


uint16_t galileo_e6_has_msg_receiver::read_has_message_header_parameter_uint16(const std::bitset<GALILEO_CNAV_MT1_HEADER_BITS>& bits, const std::pair<int32_t, int32_t>& parameter) const
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


bool galileo_e6_has_msg_receiver::read_has_message_header_parameter_bool(const std::bitset<GALILEO_CNAV_MT1_HEADER_BITS>& bits, const std::pair<int32_t, int32_t>& parameter) const
{
    bool value = false;
    if (static_cast<int>(bits[GALILEO_CNAV_MT1_HEADER_BITS - parameter.first]) == 1)
        {
            value = true;
        }
    return value;
}


uint8_t galileo_e6_has_msg_receiver::read_has_message_body_uint8(const std::string& bits) const
{
    uint8_t value = 0U;
    const size_t len = bits.length();
    for (size_t j = 0; j < len; j++)
        {
            value <<= 1U;  // shift left
            if (bits[j] == '1')
                {
                    value += 1;  // insert the bit
                }
        }
    return value;
}


uint16_t galileo_e6_has_msg_receiver::read_has_message_body_uint16(const std::string& bits) const
{
    uint16_t value = 0U;
    const size_t len = bits.length();

    for (size_t j = 0; j < len; j++)
        {
            value <<= 1U;  // shift left
            if (bits[j] == '1')
                {
                    value += 1;  // insert the bit
                }
        }
    return value;
}


uint64_t galileo_e6_has_msg_receiver::read_has_message_body_uint64(const std::string& bits) const
{
    uint64_t value = 0U;
    const size_t len = bits.length();

    for (size_t j = 0; j < len; j++)
        {
            value <<= 1U;  // shift left
            if (bits[j] == '1')
                {
                    value += 1;  // insert the bit
                }
        }
    return value;
}


int16_t galileo_e6_has_msg_receiver::read_has_message_body_int16(const std::string& bits) const
{
    int16_t value = 0;
    const size_t len = bits.length();

    // read the MSB and perform the sign extension
    if (static_cast<int>(bits[0]) == 1)
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
            if (bits[j] == '1')
                {
                    value += 1;  // insert the bit
                }
        }

    return value;
}


template <class T>
std::string galileo_e6_has_msg_receiver::debug_print_vector(const std::string& title, const std::vector<T>& vec) const
{
    std::string msg(title);
    msg += ": \n";
    std::stringstream ss;
    for (auto el : vec)
        {
            ss << static_cast<float>(el) << " ";
        }
    msg += ss.str();
    return msg;
}


std::string galileo_e6_has_msg_receiver::debug_print_matrix(const std::string& title, const std::vector<std::vector<uint8_t>>& mat) const
{
    std::string msg(title);
    msg += ": \n";
    std::stringstream ss;

    for (size_t row = 0; row < mat.size(); row++)
        {
            for (size_t col = 0; col < mat[0].size(); col++)
                {
                    ss << static_cast<float>(mat[row][col]) << " ";
                }
            ss << '\n';
        }
    msg += ss.str();
    return msg;
}
