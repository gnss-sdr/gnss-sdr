/*!
 * \file osnma_msg_receiver.h
 * \brief GNU Radio block that processes Galileo OSNMA data received from
 * Galileo E1B telemetry blocks. After successful decoding, sends the content to
 * the PVT block.
 * \author Carles Fernandez-Prades, 2023. cfernandez(at)cttc.es
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

#ifndef GNSS_SDR_OSNMA_MSG_RECEIVER_H
#define GNSS_SDR_OSNMA_MSG_RECEIVER_H

#include "galileo_inav_message.h"  // for OSNMA_msg
#include "gnss_block_interface.h"  // for gnss_shared_ptr
#include "gnss_sdr_make_unique.h"  // for std::make:unique in C++11
#include "osnma_data.h"            // for OSNMA_data
#include <boost/circular_buffer.hpp>
#include <gnuradio/block.h>  // for gr::block
#include <pmt/pmt.h>         // for pmt::pmt_t
#include <array>             // for std::array
#include <memory>            // for std::shared_ptr
#include <string>
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver_Library
 * \{ */

class OSNMA_DSM_Reader;
class Gnss_Crypto;
class osnma_msg_receiver;

using osnma_msg_receiver_sptr = gnss_shared_ptr<osnma_msg_receiver>;

osnma_msg_receiver_sptr osnma_msg_receiver_make(const std::string& pemFilePath, const std::string& merkleFilePath);

/*!
 * \brief GNU Radio block that receives asynchronous OSNMA messages
 * from the telemetry blocks, stores them in memory, and decodes OSNMA info
 * when enough data have been received.
 * The decoded OSNMA data is sent to the PVT block.
 */
class osnma_msg_receiver : public gr::block
{
public:
    ~osnma_msg_receiver() = default;  //!< Default destructor

private:
    friend osnma_msg_receiver_sptr osnma_msg_receiver_make(const std::string& pemFilePath, const std::string& merkleFilePath);
    osnma_msg_receiver(const std::string& pemFilePath, const std::string& merkleFilePath);

    void msg_handler_osnma(const pmt::pmt_t& msg);
    void process_osnma_message(const std::shared_ptr<OSNMA_msg>& osnma_msg);
    void read_nma_header(uint8_t nma_header);
    void read_dsm_header(uint8_t dsm_header);
    void read_dsm_block(const std::shared_ptr<OSNMA_msg>& osnma_msg);
    void local_time_verification(const std::shared_ptr<OSNMA_msg>& osnma_msg);
    void process_dsm_block(const std::shared_ptr<OSNMA_msg>& osnma_msg);
    void process_dsm_message(const std::vector<uint8_t>& dsm_msg, const std::shared_ptr<OSNMA_msg>& osnma_msg);
    bool verify_dsm_pkr(DSM_PKR_message message);
    void read_and_process_mack_block(const std::shared_ptr<OSNMA_msg>& osnma_msg);
    void read_mack_header();
    void read_mack_body();
    void process_mack_message(const std::shared_ptr<OSNMA_msg>& osnma_msg);
    bool verify_tag(MACK_tag_and_info tag_and_info, OSNMA_data applicable_OSNMA, uint8_t tag_position, const std::vector<uint8_t>& applicable_key, NavData applicable_NavData);

    //boost::circular_buffer<MACK_message> d_old_mack_message;
    boost::circular_buffer<OSNMA_data> d_old_OSNMA_buffer; // buffer that holds last 12 received OSNMA messages, including current one at back()
    std::unique_ptr<OSNMA_DSM_Reader> d_dsm_reader;
    std::unique_ptr<Gnss_Crypto> d_crypto;

    std::array<std::array<uint8_t, 256>, 16> d_dsm_message{}; // C: each dsm[0-15] has 2048 bits
    std::array<std::array<uint8_t, 16>, 16> d_dsm_id_received{};
    std::array<uint16_t, 16> d_number_of_blocks{};
    std::array<uint8_t, 60> d_mack_message{}; // C: 480 b

    OSNMA_data d_osnma_data{};
    bool d_new_data{false};
    bool d_public_key_verified{false};
    bool d_kroot_verified{false};
    bool d_tesla_key_verified{false};
    bool d_flag_debug{false};
    uint32_t d_GST_Sf {}; // C: used for MACSEQ and Tesla Key verification
    uint32_t d_old_GST_SIS{0};
    uint8_t d_Lt_min {}; // minimum equivalent tag length
    uint8_t d_Lt_verified_eph {0}; // verified tag bits - ephemeris
    uint8_t d_Lt_verified_utc {0}; // verified tag bits - timing
    uint8_t const d_T_L{30}; // s RG Section 2.1
    uint8_t const d_delta_COP{30}; // s SIS ICD Table 14
    uint32_t d_GST_0 {};
    uint32_t d_GST_SIS {};
    std::time_t d_receiver_time {0};
    enum tags_to_verify{all,utc,slow_eph, eph, none}; // TODO is this safe? I hope so
    tags_to_verify d_tags_allowed{tags_to_verify::all};
    std::vector<uint8_t> d_tags_to_verify{0,4,12};
    bool is_next_subframe();
};


/** \} */
/** \} */
#endif  // GNSS_SDR_OSNMA_MSG_RECEIVER_H
