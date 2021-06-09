/*!
 * \file galileo_e6_has_msg_receiver.h
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

#ifndef GNSS_SDR_GALILEO_E6_HAS_MSG_RECEIVER_H
#define GNSS_SDR_GALILEO_E6_HAS_MSG_RECEIVER_H

#include "Galileo_CNAV.h"
#include "galileo_has_data.h"
#include "gnss_block_interface.h"
#include <gnuradio/block.h>
#include <pmt/pmt.h>
#include <bitset>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>  // std::pair
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver_Library
 * \{ */

class Galileo_HAS_page;
class ReedSolomon;
class galileo_e6_has_msg_receiver;

using galileo_e6_has_msg_receiver_sptr = gnss_shared_ptr<galileo_e6_has_msg_receiver>;

galileo_e6_has_msg_receiver_sptr galileo_e6_has_msg_receiver_make();

/*!
 * \brief GNU Radio block that receives asynchronous Galileo HAS message pages
 * from the telemetry blocks, stores them in memory, and decodes HAS messages
 * when enough data have been received.
 * The decoded HAS message is sent to the PVT block.
 */
class galileo_e6_has_msg_receiver : public gr::block
{
public:
    ~galileo_e6_has_msg_receiver() = default;  //!< Default destructor

private:
    friend galileo_e6_has_msg_receiver_sptr galileo_e6_has_msg_receiver_make();
    galileo_e6_has_msg_receiver();
    void msg_handler_galileo_e6_has(const pmt::pmt_t& msg);
    void process_HAS_page(const Galileo_HAS_page& has_page);
    void read_MT1_header(const std::string& message_header);
    void read_MT1_body(const std::string& message_body);

    int decode_message_type1(uint8_t message_id, uint8_t message_size);
    uint8_t read_has_message_header_parameter_uint8(const std::bitset<GALILEO_CNAV_MT1_HEADER_BITS>& bits, const std::pair<int32_t, int32_t>& parameter) const;
    uint16_t read_has_message_header_parameter_uint16(const std::bitset<GALILEO_CNAV_MT1_HEADER_BITS>& bits, const std::pair<int32_t, int32_t>& parameter) const;
    bool read_has_message_header_parameter_bool(const std::bitset<GALILEO_CNAV_MT1_HEADER_BITS>& bits, const std::pair<int32_t, int32_t>& parameter) const;

    uint8_t read_has_message_body_uint8(const std::string& bits) const;
    uint16_t read_has_message_body_uint16(const std::string& bits) const;
    uint64_t read_has_message_body_uint64(const std::string& bits) const;
    int16_t read_has_message_body_int16(const std::string& bits) const;

    template <class T>
    std::string debug_print_vector(const std::string& title, const std::vector<T>& vec) const;                     // only for debug purposes
    std::string debug_print_matrix(const std::string& title, const std::vector<std::vector<uint8_t>>& mat) const;  // only for debug purposes

    std::unique_ptr<ReedSolomon> d_rs;
    Galileo_HAS_data d_HAS_data{};
    std::vector<std::vector<std::vector<uint8_t>>> d_C_matrix{32, std::vector<std::vector<uint8_t>>(GALILEO_CNAV_MAX_NUMBER_SYMBOLS_ENCODED_BLOCK, std::vector<uint8_t>(GALILEO_CNAV_OCTETS_IN_SUBPAGE, 0))};  // 32 x 255 x 53
    std::vector<std::vector<uint8_t>> d_M_matrix{GALILEO_CNAV_INFORMATION_VECTOR_LENGTH, std::vector<uint8_t>(GALILEO_CNAV_OCTETS_IN_SUBPAGE, 0)};                                                             // HAS message matrix 32 x 53
    std::vector<std::vector<uint8_t>> d_received_pids{32, std::vector<uint8_t>()};
    bool d_new_message{};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_E6_HAS_MSG_RECEIVER_H
