/*!
 * \file galileo_e6_has_msg_receiver.cc
 * \brief GNU Radio block that processes Galileo HAS message pages received from
 * Galileo E6B telemetry blocks. After successful decoding, sends the content to
 * the PVT block.
 * \author Carles Fernandez-Prades, 2021. cfernandez(at)cttc.es
 * \author Javier Arribas, 2021. jarribas(at)cttc.es
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
#include "display.h"                // for colors in terminal
#include "galileo_has_page.h"       // for Galileo_HAS_page
#include "gnss_sdr_make_unique.h"   // for std::make_unique in C++11
#include "reed_solomon.h"           // for ReedSolomon
#include <gnuradio/io_signature.h>  // for gr::io_signature::make
#include <algorithm>                // for std::find, std::count
#include <cmath>                    // for std::remainder
#include <cstddef>                  // for size_t
#include <iterator>                 // for std::back_inserter
#include <limits>                   // for std::numeric_limits
#include <sstream>                  // for std::stringstream
#include <stdexcept>                // for std::out_of_range
#include <typeinfo>                 // for typeid

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

#if HAS_GENERIC_LAMBDA
#else
#include <boost/bind/bind.hpp>
#endif

#if PMT_USES_BOOST_ANY
#include <boost/any.hpp>
namespace wht = boost;
#else
#include <any>
namespace wht = std;
#endif

constexpr int16_t HAS_CODE_BIAS_UNAVAILABLE_RAW = -1024;
constexpr int16_t HAS_PHASE_BIAS_UNAVAILABLE_RAW = -1024;
constexpr uint32_t GALILEO_HAS_SECONDS_PER_WEEK = 604800U;

galileo_e6_has_msg_receiver_sptr galileo_e6_has_msg_receiver_make()
{
    return galileo_e6_has_msg_receiver_sptr(new galileo_e6_has_msg_receiver());
}


galileo_e6_has_msg_receiver::galileo_e6_has_msg_receiver() : gr::block("galileo_e6_has_msg_receiver", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    // register Gal E6 HAS input message port from telemetry blocks
    this->message_port_register_in(pmt::mp("E6_HAS_from_TLM"));
    // register nav message monitor out
    this->message_port_register_out(pmt::mp("Nav_msg_from_TLM"));
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

    // Reserve memory for decoding matrices and received PIDs
    d_C_matrix = std::vector<std::vector<std::vector<uint8_t>>>(GALILEO_CNAV_INFORMATION_VECTOR_LENGTH, std::vector<std::vector<uint8_t>>(GALILEO_CNAV_MAX_NUMBER_SYMBOLS_ENCODED_BLOCK, std::vector<uint8_t>(GALILEO_CNAV_OCTETS_IN_SUBPAGE)));  // 32 x 255 x 53
    d_M_matrix = std::vector<std::vector<uint8_t>>(GALILEO_CNAV_INFORMATION_VECTOR_LENGTH, std::vector<uint8_t>(GALILEO_CNAV_OCTETS_IN_SUBPAGE));                                                                                                 // HAS message matrix 32 x 53
    d_received_pids = std::vector<std::vector<uint8_t>>(HAS_MSG_NUMBER_MESSAGE_IDS, std::vector<uint8_t>());
    d_received_timestamps = std::vector<std::vector<uint64_t>>(HAS_MSG_NUMBER_MESSAGE_IDS, std::vector<uint64_t>());
    d_printed_timestamps = std::vector<uint64_t>(HAS_MSG_NUMBER_MESSAGE_IDS, std::numeric_limits<uint64_t>::max());
    d_printed_mids = std::vector<bool>(HAS_MSG_NUMBER_MESSAGE_IDS);

    // Reserve memory to store masks
    d_nsat_in_mask_id = std::vector<int>(HAS_MSG_NUMBER_MASK_IDS);
    d_gnss_id_in_mask = std::vector<std::vector<uint8_t>>(HAS_MSG_NUMBER_MASK_IDS, std::vector<uint8_t>(HAS_MSG_NUMBER_GNSS_IDS));
    d_satellite_mask = std::vector<std::vector<uint64_t>>(HAS_MSG_NUMBER_MASK_IDS, std::vector<uint64_t>(HAS_MSG_NUMBER_GNSS_IDS));
    d_signal_mask = std::vector<std::vector<uint16_t>>(HAS_MSG_NUMBER_MASK_IDS, std::vector<uint16_t>(HAS_MSG_NUMBER_GNSS_IDS));
    d_cell_mask_availability_flag = std::vector<std::vector<bool>>(HAS_MSG_NUMBER_MASK_IDS, std::vector<bool>(HAS_MSG_NUMBER_GNSS_IDS));
    d_cell_mask = std::vector<std::vector<std::vector<std::vector<bool>>>>(HAS_MSG_NUMBER_MASK_IDS, std::vector<std::vector<std::vector<bool>>>(HAS_MSG_NUMBER_GNSS_IDS, std::vector<std::vector<bool>>(HAS_MSG_NUMBER_SATELLITE_IDS, std::vector<bool>(HAS_MSG_NUMBER_SIGNAL_MASKS))));
    d_nsys_in_mask = std::vector<uint8_t>(HAS_MSG_NUMBER_MASK_IDS);
    d_nav_message_mask = std::vector<std::vector<uint8_t>>(HAS_MSG_NUMBER_MASK_IDS, std::vector<uint8_t>(HAS_MSG_NUMBER_GNSS_IDS));
    d_mask_timestamps = std::vector<uint64_t>(HAS_MSG_NUMBER_MASK_IDS, std::numeric_limits<uint64_t>::max());

    // Initialize values for d_nav_msg_packet
    d_nav_msg_packet.system = std::string("E");
    d_nav_msg_packet.signal = std::string("E6");
    d_nav_msg_packet.prn = 0;
    d_nav_msg_packet.tow_at_current_symbol_ms = 0;
    d_last_valid_tow_timestamp = std::numeric_limits<uint64_t>::max();
    d_last_valid_tow = std::numeric_limits<uint32_t>::max();
}


void galileo_e6_has_msg_receiver::set_enable_navdata_monitor(bool enable)
{
    d_enable_navdata_monitor = enable;
}


std::shared_ptr<Galileo_HAS_data> galileo_e6_has_msg_receiver::process_test_page(const pmt::pmt_t& msg)
{
    uint64_t timestamp = std::numeric_limits<uint64_t>::max();
    uint32_t tow = std::numeric_limits<uint32_t>::max();
    try
        {
            const size_t msg_type_hash_code = pmt::any_ref(msg).type().hash_code();
            if (msg_type_hash_code == typeid(std::shared_ptr<Galileo_HAS_page>).hash_code())
                {
                    const auto HAS_data_page = wht::any_cast<std::shared_ptr<Galileo_HAS_page>>(pmt::any_ref(msg));
                    DLOG(INFO) << "New HAS page received:  "
                               << "Status: " << static_cast<float>(HAS_data_page->has_status) << ", "
                               << "MT: " << static_cast<float>(HAS_data_page->message_type) << ", "
                               << "MID: " << static_cast<float>(HAS_data_page->message_id) << ", "
                               << "MS: " << static_cast<float>(HAS_data_page->message_size) << ", "
                               << "PID: " << static_cast<float>(HAS_data_page->message_page_id);
                    d_current_has_status = HAS_data_page->has_status;
                    d_current_message_id = HAS_data_page->message_id;
                    timestamp = HAS_data_page->time_stamp;
                    update_TOW_cache(*HAS_data_page);
                    tow = get_TOW_for_page(*HAS_data_page);
                    process_HAS_page(*HAS_data_page.get());
                }
            else
                {
                    LOG(WARNING) << "galileo_e6_has_msg_receiver received an unknown object type!";
                }
        }
    catch (const wht::bad_any_cast& e)
        {
            LOG(WARNING) << "galileo_e6_has_msg_receiver Bad any_cast: " << e.what();
        }

    //  Return the resulting decoded HAS data (if available)
    if (d_new_message == true)
        {
            d_HAS_data.has_status = d_current_has_status;
            d_HAS_data.message_id = d_current_message_id;
            d_HAS_data.tow = tow;
            auto has_data_ptr = std::make_shared<Galileo_HAS_data>(d_HAS_data);
            d_new_message = false;
            d_printed_mids[d_current_message_id] = true;
            d_printed_timestamps[d_current_message_id] = timestamp;
            return has_data_ptr;
        }
    return nullptr;
}


void galileo_e6_has_msg_receiver::msg_handler_galileo_e6_has(const pmt::pmt_t& msg)
{
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with msg_handler_galileo_e6_has function called by the scheduler
    uint64_t timestamp = std::numeric_limits<uint64_t>::max();
    uint32_t tow = std::numeric_limits<uint32_t>::max();
    try
        {
            const size_t msg_type_hash_code = pmt::any_ref(msg).type().hash_code();
            if (msg_type_hash_code == typeid(std::shared_ptr<Galileo_HAS_page>).hash_code())
                {
                    const auto HAS_data_page = wht::any_cast<std::shared_ptr<Galileo_HAS_page>>(pmt::any_ref(msg));
                    DLOG(INFO) << "New HAS page received:  "
                               << "Status: " << static_cast<float>(HAS_data_page->has_status) << ", "
                               << "MT: " << static_cast<float>(HAS_data_page->message_type) << ", "
                               << "MID: " << static_cast<float>(HAS_data_page->message_id) << ", "
                               << "MS: " << static_cast<float>(HAS_data_page->message_size) << ", "
                               << "PID: " << static_cast<float>(HAS_data_page->message_page_id);
                    d_current_has_status = HAS_data_page->has_status;
                    d_current_message_id = HAS_data_page->message_id;
                    timestamp = HAS_data_page->time_stamp;
                    update_TOW_cache(*HAS_data_page);
                    tow = get_TOW_for_page(*HAS_data_page);
                    process_HAS_page(*HAS_data_page.get());
                }
            else
                {
                    LOG(WARNING) << "galileo_e6_has_msg_receiver received an unknown object type!";
                }
        }
    catch (const wht::bad_any_cast& e)
        {
            LOG(WARNING) << "galileo_e6_has_msg_receiver Bad any_cast: " << e.what();
        }

    //  Send the resulting decoded HAS data (if available) to PVT
    if (d_new_message == true)
        {
            d_HAS_data.has_status = d_current_has_status;
            d_HAS_data.message_id = d_current_message_id;
            d_HAS_data.tow = tow;
            d_printed_mids[d_current_message_id] = true;
            d_printed_timestamps[d_current_message_id] = timestamp;
            auto has_data_ptr = std::make_shared<Galileo_HAS_data>(d_HAS_data);
            this->message_port_pub(pmt::mp("E6_HAS_to_PVT"), pmt::make_any(has_data_ptr));
            d_new_message = false;
            DLOG(INFO) << "HAS message sent to the PVT block through the E6_HAS_to_PVT async message port";
        }
}


void galileo_e6_has_msg_receiver::process_HAS_page(const Galileo_HAS_page& has_page)
{
    d_new_message = false;
    if (has_page.has_status == 3)  // Do not use HAS
        {
            clear_HAS_state();
            d_HAS_data.has_status = has_page.has_status;
            d_HAS_data.message_id = has_page.message_id;
            d_HAS_data.tow = get_TOW_for_page(has_page);
            d_new_message = true;
            return;
        }

    if (has_page.message_id >= HAS_MSG_NUMBER_MESSAGE_IDS || has_page.message_size == 0 || has_page.message_size > GALILEO_CNAV_INFORMATION_VECTOR_LENGTH)
        {
            return;
        }
    bool valid_mt1_page = false;
    if (has_page.has_status == 0 || has_page.has_status == 1)
        {
            std::string page_string(has_page.has_message_string);
            if (page_string.size() != GALILEO_CNAV_MESSAGE_BITS_PER_PAGE)
                {
                    LOG(WARNING) << "Ignoring Galileo HAS page with invalid encoded page length "
                                 << page_string.size() << " bits";
                    return;
                }
            if (has_page.message_page_id != 0)  // PID=0 is reserved, ignore it
                {
                    if (has_page.message_type == 1)  // contains satellite corrections
                        {
                            delete_outdated_data(has_page);
                            if (d_printed_mids[has_page.message_id])
                                {
                                    return;
                                }
                            valid_mt1_page = true;
                            if (std::find(d_received_pids[has_page.message_id].begin(), d_received_pids[has_page.message_id].end(), has_page.message_page_id) == d_received_pids[has_page.message_id].end())
                                {
                                    // New pid! Annotate it.
                                    d_received_pids[has_page.message_id].push_back(has_page.message_page_id);
                                    d_received_timestamps[has_page.message_id].push_back(has_page.time_stamp);
                                    for (int k = 0; k < GALILEO_CNAV_OCTETS_IN_SUBPAGE; k++)
                                        {
                                            constexpr int bits_in_octet = 8;
                                            std::string bits8 = page_string.substr(k * bits_in_octet, bits_in_octet);
                                            std::bitset<bits_in_octet> bs(bits8);
                                            d_C_matrix[has_page.message_id][has_page.message_page_id - 1][k] = static_cast<uint8_t>(bs.to_ulong());
                                        }
                                }
                        }
                }
        }

    if (!valid_mt1_page)
        {
            return;
        }

    // If we have received for this message ID a number of pages equal to the message size
    if (d_received_pids[has_page.message_id].size() == has_page.message_size)
        {
            // Try to decode the message
            int res = decode_message_type1(has_page.message_id, has_page.message_size, has_page.time_stamp);

            if (res == 0)
                {
                    // Successful decoding, we have a valid HAS message stored at d_HAS_data
                    if (d_nsat_in_mask_id[d_HAS_data.header.mask_id] != 0)
                        {
                            // if we have the mask for that message, it's ready to be sent to PVT
                            d_new_message = true;
                            std::cout << TEXT_MAGENTA << "New Galileo HAS message ID " << std::to_string(has_page.message_id)
                                      << " received and successfully decoded" << TEXT_RESET << '\n';
                        }
                }
        }
}


void galileo_e6_has_msg_receiver::clear_HAS_state()
{
    DLOG(INFO) << "Clearing Galileo HAS cached state";

    d_HAS_data = Galileo_HAS_data();
    d_new_message = false;

    d_C_matrix = std::vector<std::vector<std::vector<uint8_t>>>(
        GALILEO_CNAV_INFORMATION_VECTOR_LENGTH,
        std::vector<std::vector<uint8_t>>(
            GALILEO_CNAV_MAX_NUMBER_SYMBOLS_ENCODED_BLOCK,
            std::vector<uint8_t>(GALILEO_CNAV_OCTETS_IN_SUBPAGE)));
    d_M_matrix = std::vector<std::vector<uint8_t>>(
        GALILEO_CNAV_INFORMATION_VECTOR_LENGTH,
        std::vector<uint8_t>(GALILEO_CNAV_OCTETS_IN_SUBPAGE));
    d_received_pids = std::vector<std::vector<uint8_t>>(HAS_MSG_NUMBER_MESSAGE_IDS, std::vector<uint8_t>());
    d_received_timestamps = std::vector<std::vector<uint64_t>>(HAS_MSG_NUMBER_MESSAGE_IDS, std::vector<uint64_t>());
    d_printed_timestamps = std::vector<uint64_t>(HAS_MSG_NUMBER_MESSAGE_IDS, std::numeric_limits<uint64_t>::max());
    d_printed_mids = std::vector<bool>(HAS_MSG_NUMBER_MESSAGE_IDS);

    d_nsat_in_mask_id = std::vector<int>(HAS_MSG_NUMBER_MASK_IDS);
    d_gnss_id_in_mask = std::vector<std::vector<uint8_t>>(HAS_MSG_NUMBER_MASK_IDS, std::vector<uint8_t>(HAS_MSG_NUMBER_GNSS_IDS));
    d_satellite_mask = std::vector<std::vector<uint64_t>>(HAS_MSG_NUMBER_MASK_IDS, std::vector<uint64_t>(HAS_MSG_NUMBER_GNSS_IDS));
    d_signal_mask = std::vector<std::vector<uint16_t>>(HAS_MSG_NUMBER_MASK_IDS, std::vector<uint16_t>(HAS_MSG_NUMBER_GNSS_IDS));
    d_cell_mask_availability_flag = std::vector<std::vector<bool>>(HAS_MSG_NUMBER_MASK_IDS, std::vector<bool>(HAS_MSG_NUMBER_GNSS_IDS));
    d_cell_mask = std::vector<std::vector<std::vector<std::vector<bool>>>>(HAS_MSG_NUMBER_MASK_IDS, std::vector<std::vector<std::vector<bool>>>(HAS_MSG_NUMBER_GNSS_IDS, std::vector<std::vector<bool>>(HAS_MSG_NUMBER_SATELLITE_IDS, std::vector<bool>(HAS_MSG_NUMBER_SIGNAL_MASKS))));
    d_nsys_in_mask = std::vector<uint8_t>(HAS_MSG_NUMBER_MASK_IDS);
    d_nav_message_mask = std::vector<std::vector<uint8_t>>(HAS_MSG_NUMBER_MASK_IDS, std::vector<uint8_t>(HAS_MSG_NUMBER_GNSS_IDS));
    d_mask_timestamps = std::vector<uint64_t>(HAS_MSG_NUMBER_MASK_IDS, std::numeric_limits<uint64_t>::max());

    d_iod_ref_map.clear();
    d_iod_ref_timestamps.clear();
}


void galileo_e6_has_msg_receiver::clear_mask_data(uint8_t mask_id)
{
    d_nsat_in_mask_id[mask_id] = 0;
    d_gnss_id_in_mask[mask_id] = std::vector<uint8_t>(HAS_MSG_NUMBER_GNSS_IDS);
    d_satellite_mask[mask_id] = std::vector<uint64_t>(HAS_MSG_NUMBER_GNSS_IDS);
    d_signal_mask[mask_id] = std::vector<uint16_t>(HAS_MSG_NUMBER_GNSS_IDS);
    d_cell_mask_availability_flag[mask_id] = std::vector<bool>(HAS_MSG_NUMBER_GNSS_IDS);
    d_cell_mask[mask_id] = std::vector<std::vector<std::vector<bool>>>(HAS_MSG_NUMBER_GNSS_IDS, std::vector<std::vector<bool>>(HAS_MSG_NUMBER_SATELLITE_IDS, std::vector<bool>(HAS_MSG_NUMBER_SIGNAL_MASKS)));
    d_nsys_in_mask[mask_id] = 0;
    d_nav_message_mask[mask_id] = std::vector<uint8_t>(HAS_MSG_NUMBER_GNSS_IDS);
    d_mask_timestamps[mask_id] = std::numeric_limits<uint64_t>::max();

    for (auto it = d_iod_ref_map.begin(); it != d_iod_ref_map.end();)
        {
            if (it->first.second == mask_id)
                {
                    d_iod_ref_timestamps.erase(it->first);
                    it = d_iod_ref_map.erase(it);
                }
            else
                {
                    ++it;
                }
        }
}


bool galileo_e6_has_msg_receiver::is_cache_timestamp_valid(uint64_t cache_timestamp, uint64_t current_timestamp) const
{
    if (cache_timestamp == std::numeric_limits<uint64_t>::max())
        {
            return false;
        }
    if (current_timestamp == std::numeric_limits<uint64_t>::max())
        {
            return true;
        }
    if (current_timestamp < cache_timestamp)
        {
            return false;
        }
    return (current_timestamp - cache_timestamp) <= MAX_SECONDS_REMEMBERING_MASK_IOD;
}


void galileo_e6_has_msg_receiver::update_TOW_cache(const Galileo_HAS_page& has_page)
{
    if ((has_page.tow < GALILEO_HAS_SECONDS_PER_WEEK) &&
        (has_page.time_stamp != std::numeric_limits<uint64_t>::max()))
        {
            d_last_valid_tow = has_page.tow;
            d_last_valid_tow_timestamp = has_page.time_stamp;
        }
}


uint32_t galileo_e6_has_msg_receiver::get_TOW_for_page(const Galileo_HAS_page& has_page) const
{
    if (has_page.tow < GALILEO_HAS_SECONDS_PER_WEEK)
        {
            return has_page.tow;
        }

    if ((d_last_valid_tow >= GALILEO_HAS_SECONDS_PER_WEEK) ||
        (has_page.time_stamp == std::numeric_limits<uint64_t>::max()) ||
        (d_last_valid_tow_timestamp == std::numeric_limits<uint64_t>::max()))
        {
            return std::numeric_limits<uint32_t>::max();
        }

    const bool current_after_reference = has_page.time_stamp >= d_last_valid_tow_timestamp;
    const uint64_t elapsed_s = current_after_reference ? has_page.time_stamp - d_last_valid_tow_timestamp : d_last_valid_tow_timestamp - has_page.time_stamp;
    if (elapsed_s > MAX_SECONDS_REMEMBERING_MID)
        {
            return std::numeric_limits<uint32_t>::max();
        }

    const uint32_t elapsed_week_s = static_cast<uint32_t>(elapsed_s % GALILEO_HAS_SECONDS_PER_WEEK);
    if (current_after_reference)
        {
            return (d_last_valid_tow + elapsed_week_s) % GALILEO_HAS_SECONDS_PER_WEEK;
        }

    if (elapsed_week_s <= d_last_valid_tow)
        {
            return d_last_valid_tow - elapsed_week_s;
        }

    return GALILEO_HAS_SECONDS_PER_WEEK - (elapsed_week_s - d_last_valid_tow);
}


bool galileo_e6_has_msg_receiver::is_supported_gnss_id(uint8_t gnss_id) const
{
    return gnss_id == HAS_MSG_GPS_SYSTEM || gnss_id == HAS_MSG_GALILEO_SYSTEM;
}


bool galileo_e6_has_msg_receiver::is_validity_interval_index_valid(uint8_t validity_interval_index) const
{
    return HAS_VALIDITY_INTERVALS.find(validity_interval_index) != HAS_VALIDITY_INTERVALS.cend();
}


void galileo_e6_has_msg_receiver::delete_outdated_data(const Galileo_HAS_page& has_page)
{
    const uint64_t current_time_stamp = has_page.time_stamp;
    for (size_t i = 0; i < d_received_pids.size(); i++)
        {
            uint64_t oldest_time_stamp = std::numeric_limits<uint64_t>::max();
            for (size_t j = 0; j < d_received_pids[i].size(); j++)
                {
                    uint64_t timestamp = d_received_timestamps[i][j];
                    if (timestamp > 0 && timestamp < oldest_time_stamp)
                        {
                            oldest_time_stamp = timestamp;
                        }
                }
            if (current_time_stamp > oldest_time_stamp && current_time_stamp - oldest_time_stamp > MAX_SECONDS_REMEMBERING_MID)
                {
                    DLOG(INFO) << "Deleting data for message ID " << i << " because it is too old: " << oldest_time_stamp << " vs " << current_time_stamp;
                    d_received_pids[i].clear();
                    d_received_timestamps[i].clear();
                    d_C_matrix[i] = std::vector<std::vector<uint8_t>>(
                        GALILEO_CNAV_MAX_NUMBER_SYMBOLS_ENCODED_BLOCK,
                        std::vector<uint8_t>(GALILEO_CNAV_OCTETS_IN_SUBPAGE));
                }
        }
    for (size_t mid = 0; mid < HAS_MSG_NUMBER_MESSAGE_IDS; mid++)
        {
            if (d_printed_mids[mid] == true && current_time_stamp > d_printed_timestamps[mid] && current_time_stamp - d_printed_timestamps[mid] > MAX_SECONDS_REMEMBERING_MID)
                {
                    d_printed_timestamps[mid] = std::numeric_limits<uint64_t>::max();
                    d_printed_mids[mid] = false;
                }
        }
}


int galileo_e6_has_msg_receiver::decode_message_type1(uint8_t message_id, uint8_t message_size, uint64_t time_stamp)
{
    DLOG(INFO) << "Start decoding of a HAS message";
    constexpr int32_t max_erasure_positions = GALILEO_CNAV_MAX_NUMBER_SYMBOLS_ENCODED_BLOCK - GALILEO_CNAV_INFORMATION_VECTOR_LENGTH;  // 223 Maximum erasure positions ( = number of parity symbols in a block)

    // Compute erasure positions
    std::vector<int> erasure_positions;
    erasure_positions.reserve(max_erasure_positions);

    for (uint8_t i = 1; i < message_size + 1; i++)  // we know that from message_size to 32, the value is 0
        {
            if (std::find(d_received_pids[message_id].begin(), d_received_pids[message_id].end(), i) == d_received_pids[message_id].end())
                {
                    erasure_positions.push_back(i - 1);
                }
        }
    for (int i = GALILEO_CNAV_INFORMATION_VECTOR_LENGTH + 1; i < GALILEO_CNAV_MAX_NUMBER_SYMBOLS_ENCODED_BLOCK + 1; i++)  // from 33 to 255
        {
            if (std::find(d_received_pids[message_id].begin(), d_received_pids[message_id].end(), static_cast<uint8_t>(i)) == d_received_pids[message_id].end())
                {
                    erasure_positions.push_back(i - 1);
                }
        }

    if (erasure_positions.size() > static_cast<size_t>(max_erasure_positions))
        {
            // This should not happen! Maybe message_size < PID < 33 ?
            // Don't even try to decode
            std::string msg("Reed Solomon decoding of HAS message is not possible. Received PIDs:");
            std::stringstream ss;
            for (auto pid : d_received_pids[message_id])
                {
                    ss << " " << static_cast<float>(pid);
                }
            ss << ", Message size: " << static_cast<float>(message_size) << "  Message ID: " << static_cast<float>(message_id);
            msg += ss.str();
            LOG(ERROR) << msg;
            d_received_pids[message_id].clear();
            d_received_timestamps[message_id].clear();
            d_C_matrix[message_id] = std::vector<std::vector<uint8_t>>(
                GALILEO_CNAV_MAX_NUMBER_SYMBOLS_ENCODED_BLOCK,
                std::vector<uint8_t>(GALILEO_CNAV_OCTETS_IN_SUBPAGE));
            return -1;
        }

    DLOG(INFO) << debug_print_vector("List of received PIDs", d_received_pids[message_id]);
    DLOG(INFO) << debug_print_vector("erasure_positions", erasure_positions);
    DLOG(INFO) << debug_print_matrix("C_matrix", d_C_matrix[message_id]);

    // Reset HAS decoded message matrix
    d_M_matrix = std::vector<std::vector<uint8_t>>(GALILEO_CNAV_INFORMATION_VECTOR_LENGTH, std::vector<uint8_t>(GALILEO_CNAV_OCTETS_IN_SUBPAGE));

    // Vertical decoding of d_C_matrix
    for (int col = 0; col < GALILEO_CNAV_OCTETS_IN_SUBPAGE; col++)
        {
            std::vector<uint8_t> C_column(GALILEO_CNAV_MAX_NUMBER_SYMBOLS_ENCODED_BLOCK);
            for (auto pid : d_received_pids[message_id])
                {
                    C_column[pid - 1] = d_C_matrix[message_id][pid - 1][col];
                }

            int result = d_rs->decode(C_column, erasure_positions);

            if (result < 0)
                {
                    DLOG(WARNING) << "Decoding of HAS page failed";
                    d_received_pids[message_id].clear();
                    d_received_timestamps[message_id].clear();
                    d_C_matrix[message_id] = std::vector<std::vector<uint8_t>>(
                        GALILEO_CNAV_MAX_NUMBER_SYMBOLS_ENCODED_BLOCK,
                        std::vector<uint8_t>(GALILEO_CNAV_OCTETS_IN_SUBPAGE));
                    return -1;
                }

            std::vector<uint8_t> M_column(C_column.begin(), C_column.begin() + GALILEO_CNAV_INFORMATION_VECTOR_LENGTH);
            for (int i = 0; i < GALILEO_CNAV_INFORMATION_VECTOR_LENGTH; i++)
                {
                    d_M_matrix[i][col] = M_column[i];
                }

            DLOG(INFO) << debug_print_vector("C_column entering the decoder", C_column);
            DLOG(INFO) << "Successful HAS page decoding";
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
               << " (size: " << static_cast<float>(message_size) << ") with header:\n"
               << std::string(decoded_message_type_1.begin(), decoded_message_type_1.begin() + GALILEO_CNAV_MT1_HEADER_BITS)
               << "\nand body:\n"
               << std::string(decoded_message_type_1.begin() + GALILEO_CNAV_MT1_HEADER_BITS, decoded_message_type_1.end());

    if (d_enable_navdata_monitor)
        {
            d_nav_msg_packet.nav_message = decoded_message_type_1;
            const std::shared_ptr<Nav_Message_Packet> tmp_obj = std::make_shared<Nav_Message_Packet>(d_nav_msg_packet);
            this->message_port_pub(pmt::mp("Nav_msg_from_TLM"), pmt::make_any(tmp_obj));
        }

    // reset data for next decoding
    d_C_matrix[message_id] = std::vector<std::vector<uint8_t>>(
        GALILEO_CNAV_MAX_NUMBER_SYMBOLS_ENCODED_BLOCK,
        std::vector<uint8_t>(GALILEO_CNAV_OCTETS_IN_SUBPAGE));
    d_received_pids[message_id].clear();
    d_received_timestamps[message_id].clear();

    // Trigger HAS message content reading and fill the d_HAS_data object
    d_HAS_data = Galileo_HAS_data();
    d_HAS_data.tow = std::numeric_limits<uint32_t>::max();  // Unknown

    read_MT1_header(decoded_message_type_1.substr(0, GALILEO_CNAV_MT1_HEADER_BITS));

    try
        {
            if (!read_MT1_body(std::string(decoded_message_type_1.begin() + GALILEO_CNAV_MT1_HEADER_BITS, decoded_message_type_1.end()), time_stamp))
                {
                    return -1;
                }
        }
    catch (const std::out_of_range& oor)
        {
            std::cerr << "Error when reading decoded HAS data. Wrong data format? The error was: " << oor.what() << '\n';
            return -1;
        }
    catch (const std::bad_alloc& e)
        {
            std::cerr << "Error when reading decoded HAS data. Wrong data format? The error was: " << e.what() << '\n';
            return -1;
        }
    catch (const std::exception& e)
        {
            std::cerr << "Error when reading decoded HAS data. Wrong data format? The error was: " << e.what() << '\n';
            return -1;
        }
    catch (...)
        {
            std::cerr << "Error when reading decoded HAS data. Wrong data format?\n";
            return -1;
        }
    return 0;
}


void galileo_e6_has_msg_receiver::read_MT1_header(const std::string& message_header)
{
    // HAS SIS ICD v1.0 Table 13: MT1 Message Header
    const std::bitset<GALILEO_CNAV_MT1_HEADER_BITS> has_mt1_header(message_header);
    d_HAS_data.header.toh = read_has_message_header_parameter_uint16(has_mt1_header, GALILEO_MT1_HEADER_TOH);
    d_HAS_data.header.mask_flag = read_has_message_header_parameter_bool(has_mt1_header, GALILEO_MT1_HEADER_MASK_FLAG);
    d_HAS_data.header.orbit_correction_flag = read_has_message_header_parameter_bool(has_mt1_header, GALILEO_MT1_HEADER_ORBIT_CORRECTION_FLAG);
    d_HAS_data.header.clock_fullset_flag = read_has_message_header_parameter_bool(has_mt1_header, GALILEO_MT1_HEADER_CLOCK_FULLSET_FLAG);
    d_HAS_data.header.clock_subset_flag = read_has_message_header_parameter_bool(has_mt1_header, GALILEO_MT1_HEADER_CLOCK_SUBSET_FLAG);
    d_HAS_data.header.code_bias_flag = read_has_message_header_parameter_bool(has_mt1_header, GALILEO_MT1_HEADER_CODE_BIAS_FLAG);
    d_HAS_data.header.phase_bias_flag = read_has_message_header_parameter_bool(has_mt1_header, GALILEO_MT1_HEADER_PHASE_BIAS_FLAG);
    d_HAS_data.header.reserved = read_has_message_header_parameter_uint8(has_mt1_header, GALILEO_MT1_HEADER_RESERVED);
    d_HAS_data.header.mask_id = read_has_message_header_parameter_uint8(has_mt1_header, GALILEO_MT1_HEADER_MASK_ID);
    d_HAS_data.header.iod_set_id = read_has_message_header_parameter_uint8(has_mt1_header, GALILEO_MT1_HEADER_IOD_SET_ID);

    DLOG(INFO) << "MT1 header " << message_header << ":  "
               << "TOH: " << static_cast<float>(d_HAS_data.header.toh) << ", "
               << "mask_flag: " << static_cast<float>(d_HAS_data.header.mask_flag) << ", "
               << "orbit_correction_flag: " << static_cast<float>(d_HAS_data.header.orbit_correction_flag) << ", "
               << "clock_fullset_flag: " << static_cast<float>(d_HAS_data.header.clock_fullset_flag) << ", "
               << "clock_subset_flag: " << static_cast<float>(d_HAS_data.header.clock_subset_flag) << ", "
               << "code_bias_flag: " << static_cast<float>(d_HAS_data.header.code_bias_flag) << ", "
               << "phase_bias_flag: " << static_cast<float>(d_HAS_data.header.phase_bias_flag) << ", "
               << "reserved: " << static_cast<float>(d_HAS_data.header.reserved) << ", "
               << "mask ID: " << static_cast<float>(d_HAS_data.header.mask_id) << ", "
               << "iod ID: " << static_cast<float>(d_HAS_data.header.iod_set_id);
}


bool galileo_e6_has_msg_receiver::read_MT1_body(const std::string& message_body, uint64_t time_stamp)
{
    // HAS SIS ICD v1.0 Table 17: MT1 Message Body
    auto message = std::string(message_body);
    const auto clear_current_mask_if_defined = [this]() {
        if (d_HAS_data.header.mask_flag)
            {
                clear_mask_data(d_HAS_data.header.mask_id);
            }
    };
    const auto read_bits = [&message, &clear_current_mask_if_defined](size_t bits, const std::string& field_name, std::string& field_bits) -> bool {
        if (message.size() < bits)
            {
                LOG(WARNING) << "HAS MT1 body too short while reading " << field_name << ": expected "
                             << bits << " bits, available " << message.size() << " bits";
                clear_current_mask_if_defined();
                return false;
            }
        field_bits = message.substr(0, bits);
        message.erase(0, bits);
        return true;
    };
    const auto signal_mask_has_reserved_bits = [](uint8_t gnss_id, uint16_t signal_mask) -> bool {
        for (uint8_t signal_index = 0; signal_index < HAS_MSG_NUMBER_SIGNAL_MASKS; signal_index++)
            {
                const bool signal_is_present = (signal_mask & (1U << (HAS_MSG_NUMBER_SIGNAL_MASKS - signal_index - 1))) != 0;
                bool reserved_signal = false;
                if (gnss_id == HAS_MSG_GPS_SYSTEM)
                    {
                        reserved_signal = signal_index == 1 || signal_index == 2 || signal_index == 10 || signal_index == 14 || signal_index == 15;
                    }
                else if (gnss_id == HAS_MSG_GALILEO_SYSTEM)
                    {
                        reserved_signal = signal_index == 15;
                    }
                if (signal_is_present && reserved_signal)
                    {
                        return true;
                    }
            }
        return false;
    };
    int Nsat = 0;
    bool have_mask = false;
    std::string bits;

    if (d_HAS_data.header.mask_flag)
        {
            // read mask
            if (!read_bits(HAS_MSG_NSYS_LENGTH, "Nsys", bits))
                {
                    return false;
                }
            d_HAS_data.Nsys = read_has_message_body_uint8(bits);
            d_nsys_in_mask[d_HAS_data.header.mask_id] = d_HAS_data.Nsys;
            if (d_HAS_data.Nsys == 0)
                {
                    LOG(WARNING) << "HAS MT1 Mask block contains reserved Nsys value 0";
                    clear_mask_data(d_HAS_data.header.mask_id);
                    return false;
                }
            if (d_HAS_data.Nsys != 0)
                {
                    d_HAS_data.gnss_id_mask = std::vector<uint8_t>(d_HAS_data.Nsys);
                    d_HAS_data.cell_mask = std::vector<std::vector<std::vector<bool>>>(d_HAS_data.Nsys, std::vector<std::vector<bool>>(HAS_MSG_NUMBER_SATELLITE_IDS, std::vector<bool>(HAS_MSG_NUMBER_SIGNAL_MASKS)));
                    d_HAS_data.cell_mask_availability_flag = std::vector<bool>(d_HAS_data.Nsys);
                    d_HAS_data.nav_message = std::vector<uint8_t>(d_HAS_data.Nsys);
                    d_HAS_data.satellite_mask = std::vector<uint64_t>(d_HAS_data.Nsys);
                    d_HAS_data.signal_mask = std::vector<uint16_t>(d_HAS_data.Nsys);
                    std::vector<bool> gnss_id_present(HAS_MSG_NUMBER_GNSS_IDS);

                    for (uint8_t i = 0; i < d_HAS_data.Nsys; i++)
                        {
                            if (!read_bits(HAS_MSG_ID_MASK_LENGTH, "GNSS ID", bits))
                                {
                                    return false;
                                }
                            d_HAS_data.gnss_id_mask[i] = read_has_message_body_uint8(bits);
                            if (!is_supported_gnss_id(d_HAS_data.gnss_id_mask[i]))
                                {
                                    LOG(WARNING) << "HAS MT1 Mask block contains unsupported or reserved GNSS ID " << std::to_string(d_HAS_data.gnss_id_mask[i]);
                                    clear_mask_data(d_HAS_data.header.mask_id);
                                    return false;
                                }
                            if (gnss_id_present[d_HAS_data.gnss_id_mask[i]])
                                {
                                    LOG(WARNING) << "HAS MT1 Mask block contains duplicate GNSS ID " << std::to_string(d_HAS_data.gnss_id_mask[i]);
                                    clear_mask_data(d_HAS_data.header.mask_id);
                                    return false;
                                }
                            gnss_id_present[d_HAS_data.gnss_id_mask[i]] = true;
                            d_gnss_id_in_mask[d_HAS_data.header.mask_id][i] = d_HAS_data.gnss_id_mask[i];

                            std::string msg;
                            if (!read_bits(HAS_MSG_SATELLITE_MASK_LENGTH, "Satellite Mask", msg))
                                {
                                    return false;
                                }
                            d_HAS_data.satellite_mask[i] = read_has_message_body_uint64(msg);
                            d_satellite_mask[d_HAS_data.header.mask_id][i] = d_HAS_data.satellite_mask[i];
                            int ones_in_satellite_mask = std::count(msg.begin(), msg.end(), '1');
                            Nsat += ones_in_satellite_mask;

                            if (!read_bits(HAS_MSG_SIGNAL_MASK_LENGTH, "Signal Mask", msg))
                                {
                                    return false;
                                }
                            d_HAS_data.signal_mask[i] = read_has_message_body_uint16(msg);
                            if (signal_mask_has_reserved_bits(d_HAS_data.gnss_id_mask[i], d_HAS_data.signal_mask[i]))
                                {
                                    LOG(WARNING) << "HAS MT1 Mask block contains reserved Signal Mask bits for GNSS ID " << std::to_string(d_HAS_data.gnss_id_mask[i]);
                                    clear_mask_data(d_HAS_data.header.mask_id);
                                    return false;
                                }
                            d_signal_mask[d_HAS_data.header.mask_id][i] = d_HAS_data.signal_mask[i];
                            int ones_in_signal_mask = std::count(msg.begin(), msg.end(), '1');

                            d_HAS_data.cell_mask[i] = std::vector<std::vector<bool>>(ones_in_satellite_mask, std::vector<bool>(ones_in_signal_mask, false));

                            if (!read_bits(1, "Cell Mask Availability Flag", bits))
                                {
                                    return false;
                                }
                            if (bits == "1")
                                {
                                    d_HAS_data.cell_mask_availability_flag[i] = true;
                                }
                            else
                                {
                                    d_HAS_data.cell_mask_availability_flag[i] = false;
                                }
                            d_cell_mask_availability_flag[d_HAS_data.header.mask_id][i] = d_HAS_data.cell_mask_availability_flag[i];

                            if (d_HAS_data.cell_mask_availability_flag[i] == true)
                                {
                                    int size_cell = ones_in_satellite_mask * ones_in_signal_mask;
                                    if (!read_bits(size_cell, "Cell Mask", msg))
                                        {
                                            return false;
                                        }
                                    int pos = 0;
                                    for (int s = 0; s < ones_in_satellite_mask; s++)
                                        {
                                            for (int sig = 0; sig < ones_in_signal_mask; sig++)
                                                {
                                                    d_HAS_data.cell_mask[i][s][sig] = (msg[pos] == '1' ? true : false);
                                                    pos++;
                                                }
                                        }
                                }

                            if (!read_bits(HAS_MSG_NAV_MESSAGE_LENGTH, "Navigation Message", bits))
                                {
                                    return false;
                                }
                            d_HAS_data.nav_message[i] = read_has_message_body_uint8(bits);
                            if (d_HAS_data.nav_message[i] != 0)
                                {
                                    LOG(WARNING) << "HAS MT1 Mask block contains reserved Navigation Message Index " << std::to_string(d_HAS_data.nav_message[i]);
                                    clear_mask_data(d_HAS_data.header.mask_id);
                                    return false;
                                }
                            d_nav_message_mask[d_HAS_data.header.mask_id][i] = d_HAS_data.nav_message[i];
                        }
                    d_nsat_in_mask_id[d_HAS_data.header.mask_id] = Nsat;
                    d_cell_mask[d_HAS_data.header.mask_id] = d_HAS_data.cell_mask;
                    d_mask_timestamps[d_HAS_data.header.mask_id] = time_stamp;
                    if (!read_bits(HAS_MSG_MASK_RESERVED_LENGTH, "Mask reserved bits", bits))
                        {
                            return false;
                        }
                }
            if (Nsat != 0)
                {
                    have_mask = true;
                }

            DLOG(INFO) << "Nsys: " << static_cast<float>(d_HAS_data.Nsys);
            DLOG(INFO) << debug_print_vector("GNSS ID", d_HAS_data.gnss_id_mask);
            DLOG(INFO) << debug_print_vector("cell_mask_availability_flag", d_HAS_data.cell_mask_availability_flag);
            DLOG(INFO) << debug_print_vector("nav_message", d_HAS_data.nav_message);
        }
    else
        {
            // Take data from a previously received mask with the same mask_id
            Nsat = d_nsat_in_mask_id[d_HAS_data.header.mask_id];
            if (Nsat != 0)
                {
                    if (is_cache_timestamp_valid(d_mask_timestamps[d_HAS_data.header.mask_id], time_stamp))
                        {
                            d_HAS_data.Nsys = d_nsys_in_mask[d_HAS_data.header.mask_id];
                            std::copy(d_gnss_id_in_mask[d_HAS_data.header.mask_id].begin(), d_gnss_id_in_mask[d_HAS_data.header.mask_id].begin() + d_HAS_data.Nsys, std::back_inserter(d_HAS_data.gnss_id_mask));
                            std::copy(d_satellite_mask[d_HAS_data.header.mask_id].begin(), d_satellite_mask[d_HAS_data.header.mask_id].begin() + d_HAS_data.Nsys, std::back_inserter(d_HAS_data.satellite_mask));
                            std::copy(d_signal_mask[d_HAS_data.header.mask_id].begin(), d_signal_mask[d_HAS_data.header.mask_id].begin() + d_HAS_data.Nsys, std::back_inserter(d_HAS_data.signal_mask));
                            std::copy(d_cell_mask_availability_flag[d_HAS_data.header.mask_id].begin(), d_cell_mask_availability_flag[d_HAS_data.header.mask_id].begin() + d_HAS_data.Nsys, std::back_inserter(d_HAS_data.cell_mask_availability_flag));
                            d_HAS_data.cell_mask = d_cell_mask[d_HAS_data.header.mask_id];
                            std::copy(d_nav_message_mask[d_HAS_data.header.mask_id].begin(), d_nav_message_mask[d_HAS_data.header.mask_id].begin() + d_HAS_data.Nsys, std::back_inserter(d_HAS_data.nav_message));

                            have_mask = true;

                            DLOG(INFO) << "Nsys: " << static_cast<float>(d_HAS_data.Nsys);
                            DLOG(INFO) << debug_print_vector("GNSS ID", d_HAS_data.gnss_id_mask);
                            DLOG(INFO) << debug_print_vector("cell_mask_availability_flag", d_HAS_data.cell_mask_availability_flag);
                            DLOG(INFO) << debug_print_vector("nav_message", d_HAS_data.nav_message);
                        }
                    else
                        {
                            DLOG(INFO) << "Discarding stale Galileo HAS mask ID " << static_cast<float>(d_HAS_data.header.mask_id);
                            clear_mask_data(d_HAS_data.header.mask_id);
                            Nsat = 0;
                        }
                }
        }

    // discard data if crazy Values
    if (d_HAS_data.header.toh > HAS_MSG_NUMBER_MAX_TOH)
        {
            have_mask = false;
            clear_mask_data(d_HAS_data.header.mask_id);
        }

    if (d_HAS_data.header.orbit_correction_flag && have_mask)
        {
            // read orbit corrections
            if (!read_bits(HAS_MSG_VALIDITY_INDEX_LENGTH, "Orbit Corrections Validity Interval Index", bits))
                {
                    return false;
                }
            d_HAS_data.validity_interval_index_orbit_corrections = read_has_message_body_uint8(bits);
            if (!is_validity_interval_index_valid(d_HAS_data.validity_interval_index_orbit_corrections))
                {
                    LOG(WARNING) << "Orbit Corrections block contains reserved Validity Interval Index " << std::to_string(d_HAS_data.validity_interval_index_orbit_corrections);
                    if (d_HAS_data.header.mask_flag)
                        {
                            clear_mask_data(d_HAS_data.header.mask_id);
                        }
                    return false;
                }
            d_HAS_data.gnss_iod = std::vector<uint16_t>(Nsat);
            d_HAS_data.delta_radial = std::vector<int16_t>(Nsat);
            d_HAS_data.delta_in_track = std::vector<int16_t>(Nsat);
            d_HAS_data.delta_cross_track = std::vector<int16_t>(Nsat);
            for (int i = 0; i < Nsat; i++)
                {
                    if (d_HAS_data.get_gnss_id(i) == HAS_MSG_GPS_SYSTEM)
                        {
                            if (!read_bits(HAS_MSG_IOD_GPS_LENGTH, "GPS IOD", bits))
                                {
                                    return false;
                                }
                            d_HAS_data.gnss_iod[i] = read_has_message_body_uint16(bits);
                        }
                    else if (d_HAS_data.get_gnss_id(i) == HAS_MSG_GALILEO_SYSTEM)
                        {
                            if (!read_bits(HAS_MSG_IOD_GAL_LENGTH, "Galileo IOD", bits))
                                {
                                    return false;
                                }
                            d_HAS_data.gnss_iod[i] = read_has_message_body_uint16(bits);
                        }
                    else if (d_HAS_data.get_gnss_id(i) == HAS_MSG_WRONG_SYSTEM)
                        {
                            // wrong data format, aborting
                            have_mask = false;
                        }
                    else
                        {
                            // reserved system, not defined what to do, aborting
                            LOG(WARNING) << "Is the HAS message transmitting data belonging to a new system? System identifier was: " << std::to_string(d_HAS_data.get_gnss_id(i));
                            have_mask = false;
                        }

                    if (have_mask)
                        {
                            if (!read_bits(HAS_MSG_DELTA_RADIAL_LENGTH, "Delta Radial", bits))
                                {
                                    return false;
                                }
                            d_HAS_data.delta_radial[i] = read_has_message_body_int16(bits);

                            if (!read_bits(HAS_MSG_DELTA_IN_TRACK_LENGTH, "Delta In-Track", bits))
                                {
                                    return false;
                                }
                            d_HAS_data.delta_in_track[i] = read_has_message_body_int16(bits);

                            if (!read_bits(HAS_MSG_DELTA_CROSS_TRACK_LENGTH, "Delta Cross-Track", bits))
                                {
                                    return false;
                                }
                            d_HAS_data.delta_cross_track[i] = read_has_message_body_int16(bits);
                        }
                }

            if (have_mask)
                {
                    const auto iod_ref_key = std::make_pair(d_HAS_data.header.iod_set_id, d_HAS_data.header.mask_id);
                    d_iod_ref_map[iod_ref_key] = d_HAS_data.gnss_iod;
                    d_iod_ref_timestamps[iod_ref_key] = time_stamp;
                }
            DLOG(INFO) << debug_print_vector("gnss_iod", d_HAS_data.gnss_iod);
            DLOG(INFO) << debug_print_vector("delta_radial", d_HAS_data.delta_radial);
            DLOG(INFO) << debug_print_vector("delta_in_track", d_HAS_data.delta_in_track);
            DLOG(INFO) << debug_print_vector("delta_cross_track", d_HAS_data.delta_cross_track);
        }

    const bool has_corrections_without_orbit = !d_HAS_data.header.orbit_correction_flag &&
                                               (d_HAS_data.header.clock_fullset_flag || d_HAS_data.header.clock_subset_flag ||
                                                   d_HAS_data.header.code_bias_flag || d_HAS_data.header.phase_bias_flag);
    if (has_corrections_without_orbit && have_mask)
        {
            const auto iod_ref_key = std::make_pair(d_HAS_data.header.iod_set_id, d_HAS_data.header.mask_id);
            auto ref_it = d_iod_ref_map.find(iod_ref_key);
            auto ref_timestamp_it = d_iod_ref_timestamps.find(iod_ref_key);
            if (ref_it != d_iod_ref_map.end() && ref_timestamp_it != d_iod_ref_timestamps.end() && is_cache_timestamp_valid(ref_timestamp_it->second, time_stamp))
                {
                    d_HAS_data.gnss_iod = ref_it->second;
                }
            else
                {
                    LOG(WARNING) << "MT1 corrections refer to unavailable or stale Reference IODs for Mask ID "
                                 << std::to_string(d_HAS_data.header.mask_id)
                                 << " and IOD Set ID " << std::to_string(d_HAS_data.header.iod_set_id);
                    have_mask = false;
                    if (ref_it != d_iod_ref_map.end())
                        {
                            d_iod_ref_map.erase(ref_it);
                        }
                    if (ref_timestamp_it != d_iod_ref_timestamps.end())
                        {
                            d_iod_ref_timestamps.erase(ref_timestamp_it);
                        }
                }
        }

    if (d_HAS_data.header.clock_fullset_flag && have_mask)
        {
            // read clock full-set corrections
            if (!read_bits(HAS_MSG_VALIDITY_INDEX_LENGTH, "Clock Full-Set Corrections Validity Interval Index", bits))
                {
                    return false;
                }
            d_HAS_data.validity_interval_index_clock_fullset_corrections = read_has_message_body_uint8(bits);
            if (!is_validity_interval_index_valid(d_HAS_data.validity_interval_index_clock_fullset_corrections))
                {
                    LOG(WARNING) << "Clock Full-Set Corrections block contains reserved Validity Interval Index " << std::to_string(d_HAS_data.validity_interval_index_clock_fullset_corrections);
                    if (d_HAS_data.header.mask_flag)
                        {
                            clear_mask_data(d_HAS_data.header.mask_id);
                        }
                    return false;
                }

            d_HAS_data.delta_clock_multiplier = std::vector<uint8_t>(d_HAS_data.Nsys);
            for (uint8_t i = 0; i < d_HAS_data.Nsys; i++)
                {
                    if (!read_bits(HAS_MSG_DELTA_CLOCK_MULTIPLIER_LENGTH, "Delta Clock Multiplier", bits))
                        {
                            return false;
                        }
                    d_HAS_data.delta_clock_multiplier[i] = read_has_message_body_uint8(bits) + 1;  // b00 means x1, b01 means x2, etc
                }

            d_HAS_data.delta_clock_correction = std::vector<int16_t>(Nsat);
            for (int i = 0; i < Nsat; i++)
                {
                    if (!read_bits(HAS_MSG_DELTA_CLOCK_CORRECTION_LENGTH, "Delta Clock Correction", bits))
                        {
                            return false;
                        }
                    d_HAS_data.delta_clock_correction[i] = read_has_message_body_int16(bits);
                }

            DLOG(INFO) << debug_print_vector("delta_clock_multiplier", d_HAS_data.delta_clock_multiplier);
            DLOG(INFO) << debug_print_vector("delta_clock_correction", d_HAS_data.delta_clock_correction);
        }

    if (d_HAS_data.header.clock_subset_flag && have_mask)
        {
            // read clock subset corrections
            if (!read_bits(HAS_MSG_VALIDITY_INDEX_LENGTH, "Clock Subset Corrections Validity Interval Index", bits))
                {
                    return false;
                }
            d_HAS_data.validity_interval_index_clock_subset_corrections = read_has_message_body_uint8(bits);
            if (!is_validity_interval_index_valid(d_HAS_data.validity_interval_index_clock_subset_corrections))
                {
                    LOG(WARNING) << "Clock Subset Corrections block contains reserved Validity Interval Index " << std::to_string(d_HAS_data.validity_interval_index_clock_subset_corrections);
                    if (d_HAS_data.header.mask_flag)
                        {
                            clear_mask_data(d_HAS_data.header.mask_id);
                        }
                    return false;
                }

            if (!read_bits(HAS_MSG_NSYS_SUB_LENGTH, "Nsys_sub", bits))
                {
                    return false;
                }
            d_HAS_data.Nsys_sub = read_has_message_body_uint8(bits);

            if (d_HAS_data.Nsys_sub == 0)
                {
                    // wrong data format, aborting
                    have_mask = false;
                    clear_mask_data(d_HAS_data.header.mask_id);
                }

            if (have_mask)
                {
                    d_HAS_data.gnss_id_clock_subset = std::vector<uint8_t>(d_HAS_data.Nsys_sub);
                    d_HAS_data.delta_clock_multiplier_clock_subset = std::vector<uint8_t>(d_HAS_data.Nsys_sub);
                    d_HAS_data.satellite_submask = std::vector<uint64_t>(d_HAS_data.Nsys_sub);
                    d_HAS_data.delta_clock_correction_clock_subset = std::vector<std::vector<int16_t>>(d_HAS_data.Nsys_sub, std::vector<int16_t>());

                    for (uint8_t i = 0; i < d_HAS_data.Nsys_sub; i++)
                        {
                            if (!read_bits(HAS_MSG_ID_CLOCK_SUBSET_LENGTH, "Clock Subset GNSS ID", bits))
                                {
                                    return false;
                                }
                            d_HAS_data.gnss_id_clock_subset[i] = read_has_message_body_uint8(bits);

                            if (!read_bits(HAS_MSG_DELTA_CLOCK_MULTIPLIER_SUBSET_LENGTH, "Clock Subset Delta Clock Multiplier", bits))
                                {
                                    return false;
                                }
                            uint8_t clock_multiplier = read_has_message_body_uint8(bits);
                            d_HAS_data.delta_clock_multiplier_clock_subset[i] = clock_multiplier + 1;  // b00 means x1, b01 means x2, etc

                            // find the satellite mask corresponding to this GNSS ID
                            auto it = std::find(d_HAS_data.gnss_id_mask.begin(), d_HAS_data.gnss_id_mask.end(), d_HAS_data.gnss_id_clock_subset[i]);
                            if (it == d_HAS_data.gnss_id_mask.end())
                                {
                                    LOG(WARNING) << "Clock Subset Corrections block refers to a GNSS ID that is not present in the mask: " << std::to_string(d_HAS_data.gnss_id_clock_subset[i]);
                                    have_mask = false;
                                    clear_mask_data(d_HAS_data.header.mask_id);
                                    break;
                                }
                            int index = it - d_HAS_data.gnss_id_mask.begin();
                            uint64_t satellite_mask = d_HAS_data.satellite_mask[index];

                            // SatMsub length is Nsat, the number of satellites marked in SatM for this GNSS.
                            std::bitset<HAS_MSG_SATELLITE_MASK_LENGTH> satellite_mask_bits(satellite_mask);
                            const auto number_sats_this_gnss_id = static_cast<size_t>(satellite_mask_bits.count());

                            std::string satellite_submask_bits;
                            if (!read_bits(number_sats_this_gnss_id, "Satellite Subset Mask", satellite_submask_bits))
                                {
                                    return false;
                                }
                            d_HAS_data.satellite_submask[i] = read_has_message_body_uint64(satellite_submask_bits);

                            const auto Nsat_sub = static_cast<size_t>(std::count(satellite_submask_bits.cbegin(), satellite_submask_bits.cend(), '1'));
                            d_HAS_data.delta_clock_correction_clock_subset[i].resize(Nsat_sub);

                            // Read Nsat_sub values of delta_clock_correction_clock_subset
                            for (size_t j = 0; j < Nsat_sub; j++)
                                {
                                    if (!read_bits(HAS_MSG_DELTA_CLOCK_CORRECTION_SUBSET_LENGTH, "Clock Subset Delta Clock Correction", bits))
                                        {
                                            return false;
                                        }
                                    d_HAS_data.delta_clock_correction_clock_subset[i][j] = read_has_message_body_int16(bits);
                                }
                        }
                }

            DLOG(INFO) << "Nsys_sub: " << static_cast<float>(d_HAS_data.Nsys_sub);
            DLOG(INFO) << (d_HAS_data.Nsys_sub == 0 ? "" : debug_print_vector("gnss_id_clock_subset", d_HAS_data.gnss_id_clock_subset));
            DLOG(INFO) << (d_HAS_data.Nsys_sub == 0 ? "" : debug_print_vector("delta_clock_multiplier_clock_subset", d_HAS_data.delta_clock_multiplier_clock_subset));
            DLOG(INFO) << (d_HAS_data.Nsys_sub == 0 ? "" : debug_print_vector("satellite_submask", d_HAS_data.satellite_submask));
            DLOG(INFO) << (d_HAS_data.Nsys_sub == 0 ? "" : debug_print_matrix("delta_clock_correction_clock_subset", d_HAS_data.delta_clock_correction_clock_subset));
        }

    if (d_HAS_data.header.code_bias_flag && have_mask)
        {
            // read code bias
            if (!read_bits(HAS_MSG_VALIDITY_INDEX_LENGTH, "Code Bias Validity Interval Index", bits))
                {
                    return false;
                }
            d_HAS_data.validity_interval_index_code_bias_corrections = read_has_message_body_uint8(bits);
            if (!is_validity_interval_index_valid(d_HAS_data.validity_interval_index_code_bias_corrections))
                {
                    LOG(WARNING) << "Code Bias block contains reserved Validity Interval Index " << std::to_string(d_HAS_data.validity_interval_index_code_bias_corrections);
                    if (d_HAS_data.header.mask_flag)
                        {
                            clear_mask_data(d_HAS_data.header.mask_id);
                        }
                    return false;
                }

            std::vector<uint64_t> number_sats(d_HAS_data.Nsys);
            std::vector<uint64_t> number_codes(d_HAS_data.Nsys);
            uint64_t max_signals = 0ULL;
            for (int sys = 0; sys < d_HAS_data.Nsys; sys++)
                {
                    uint64_t number_sats_this_gnss_id = 0;
                    uint64_t number_signals_this_gnss_id = 0;
                    if (d_HAS_data.cell_mask_availability_flag[sys] == true)
                        {
                            // cell mask is provided
                            number_sats_this_gnss_id = d_HAS_data.cell_mask[sys].size();
                            number_signals_this_gnss_id = d_HAS_data.cell_mask[sys][0].size();

                            if (number_signals_this_gnss_id > max_signals)
                                {
                                    max_signals = number_signals_this_gnss_id;
                                }
                        }
                    else
                        {
                            // corrections for all satellites in satellite_mask
                            // and all signals in signal mask
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
                            if (number_signals_this_gnss_id > max_signals)
                                {
                                    max_signals = number_signals_this_gnss_id;
                                }
                        }
                    number_sats[sys] = number_sats_this_gnss_id;
                    number_codes[sys] = number_signals_this_gnss_id;
                }

            d_HAS_data.code_bias = std::vector<std::vector<int16_t>>(Nsat, std::vector<int16_t>(max_signals, HAS_CODE_BIAS_UNAVAILABLE_RAW));

            int sat = 0;
            for (int sys = 0; sys < d_HAS_data.Nsys; sys++)
                {
                    for (uint64_t s = 0; s < number_sats[sys]; s++)
                        {
                            for (uint64_t c = 0; c < number_codes[sys]; c++)
                                {
                                    if ((d_HAS_data.cell_mask_availability_flag[sys] == false) || ((d_HAS_data.cell_mask_availability_flag[sys] == true) && (d_HAS_data.cell_mask[sys][s][c])))
                                        {
                                            if (!read_bits(HAS_MSG_CODE_BIAS_LENGTH, "Code Bias", bits))
                                                {
                                                    return false;
                                                }
                                            d_HAS_data.code_bias[sat][c] = read_has_message_body_int16(bits);
                                        }
                                }
                            sat += 1;
                        }
                }

            DLOG(INFO) << debug_print_matrix("code bias", d_HAS_data.code_bias);
        }

    if (d_HAS_data.header.phase_bias_flag && have_mask)
        {
            // read phase bias
            if (!read_bits(HAS_MSG_VALIDITY_INDEX_LENGTH, "Phase Bias Validity Interval Index", bits))
                {
                    return false;
                }
            d_HAS_data.validity_interval_index_phase_bias_corrections = read_has_message_body_uint8(bits);
            if (!is_validity_interval_index_valid(d_HAS_data.validity_interval_index_phase_bias_corrections))
                {
                    LOG(WARNING) << "Phase Bias block contains reserved Validity Interval Index " << std::to_string(d_HAS_data.validity_interval_index_phase_bias_corrections);
                    if (d_HAS_data.header.mask_flag)
                        {
                            clear_mask_data(d_HAS_data.header.mask_id);
                        }
                    return false;
                }

            std::vector<uint64_t> number_sats(d_HAS_data.Nsys);
            std::vector<uint64_t> number_phases(d_HAS_data.Nsys);
            uint64_t max_signals = 0ULL;
            for (int sys = 0; sys < d_HAS_data.Nsys; sys++)
                {
                    uint64_t number_sats_this_gnss_id = 0;
                    uint64_t number_signals_this_gnss_id = 0;
                    if (d_HAS_data.cell_mask_availability_flag[sys] == true)
                        {
                            // cell mask is provided
                            number_sats_this_gnss_id = d_HAS_data.cell_mask[sys].size();
                            number_signals_this_gnss_id = d_HAS_data.cell_mask[sys][0].size();

                            if (number_signals_this_gnss_id > max_signals)
                                {
                                    max_signals = number_signals_this_gnss_id;
                                }
                        }
                    else
                        {
                            // corrections for all satellites in satellite_mask
                            // and all signals in signal mask
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
                            if (number_signals_this_gnss_id > max_signals)
                                {
                                    max_signals = number_signals_this_gnss_id;
                                }
                        }
                    number_sats[sys] = number_sats_this_gnss_id;
                    number_phases[sys] = number_signals_this_gnss_id;
                }

            d_HAS_data.phase_bias = std::vector<std::vector<int16_t>>(Nsat, std::vector<int16_t>(max_signals, HAS_PHASE_BIAS_UNAVAILABLE_RAW));
            d_HAS_data.phase_discontinuity_indicator = std::vector<std::vector<uint8_t>>(Nsat, std::vector<uint8_t>(max_signals));

            int sat = 0;
            for (int sys = 0; sys < d_HAS_data.Nsys; sys++)
                {
                    for (uint64_t s = 0; s < number_sats[sys]; s++)
                        {
                            for (uint64_t p = 0; p < number_phases[sys]; p++)
                                {
                                    if ((d_HAS_data.cell_mask_availability_flag[sys] == false) || ((d_HAS_data.cell_mask_availability_flag[sys] == true) && (d_HAS_data.cell_mask[sys][s][p])))
                                        {
                                            if (!read_bits(HAS_MSG_PHASE_BIAS_LENGTH, "Phase Bias", bits))
                                                {
                                                    return false;
                                                }
                                            d_HAS_data.phase_bias[sat][p] = read_has_message_body_int16(bits);

                                            if (!read_bits(HAS_MSG_PHASE_DISCONTINUITY_INDICATOR_LENGTH, "Phase Discontinuity Indicator", bits))
                                                {
                                                    return false;
                                                }
                                            d_HAS_data.phase_discontinuity_indicator[sat][p] = read_has_message_body_uint8(bits);
                                        }
                                }
                            sat += 1;
                        }
                }

            DLOG(INFO) << debug_print_matrix("phase bias", d_HAS_data.phase_bias);
            DLOG(INFO) << debug_print_matrix("phase discontinuity indicator", d_HAS_data.phase_discontinuity_indicator);
        }
    return have_mask;
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


bool galileo_e6_has_msg_receiver::read_has_message_header_parameter_bool(const std::bitset<GALILEO_CNAV_MT1_HEADER_BITS>& bits, const std::pair<int32_t, int32_t>& parameter) const
{
    bool value = false;
    if (static_cast<int>(bits[GALILEO_CNAV_MT1_HEADER_BITS - parameter.first]) == 1)
        {
            value = true;
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


int16_t galileo_e6_has_msg_receiver::read_has_message_body_int16(const std::string& bits) const
{
    const size_t len = bits.length();
    if (bits.empty() || len > 16)
        {
            LOG(WARNING) << "Invalid binary string length for int16_t: " << len;
            return 0;
        }

    uint16_t value = 0;
    for (const char c : bits)
        {
            if (c != '0' && c != '1')
                {
                    LOG(WARNING) << "Invalid character in binary string: " << c;
                    return 0;
                }
            value = static_cast<uint16_t>((value << 1) | (c - '0'));
        }

    // If sign bit is set, sign-extend to 16-bit using unsigned arithmetic.
    if (bits[0] == '1' && len < 16)
        {
            value = static_cast<uint16_t>(value | static_cast<uint16_t>(0xFFFFU << len));
        }
    return static_cast<int16_t>(value);
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


template <class T>
std::string galileo_e6_has_msg_receiver::debug_print_matrix(const std::string& title, const std::vector<std::vector<T>>& mat) const
{
    std::string msg(title);
    msg += ": \n";
    std::stringstream ss;

    if (!mat.empty())
        {
            for (size_t row = 0; row < mat.size(); row++)
                {
                    for (size_t col = 0; col < mat[0].size(); col++)
                        {
                            ss << static_cast<float>(mat[row][col]) << " ";
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
