/*!
 * \file osnma_msg_receiver.cc
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


#include "osnma_msg_receiver.h"
#include "gnss_sdr_make_unique.h"   // for std::make_unique in C++11
#include <glog/logging.h>           // for DLOG
#include <gnuradio/io_signature.h>  // for gr::io_signature::make
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <iterator>
#include <numeric>
#include <typeinfo>  // for typeid
#include <unordered_map>
#include <utility>

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


// OSNMA SIS ICD v1.0 Table 3
const std::unordered_map<uint8_t, std::pair<uint16_t, uint16_t>> OSNMA_TABLE_3 = {
    {0, {0, 0}},
    {1, {0, 0}},
    {2, {0, 0}},
    {3, {0, 0}},
    {4, {0, 0}},
    {5, {0, 0}},
    {6, {0, 0}},
    {7, {13, 1352}},
    {8, {14, 1456}},
    {9, {15, 1560}},
    {10, {16, 1664}},
    {11, {0, 0}},
    {12, {0, 0}},
    {13, {0, 0}},
    {14, {0, 0}},
    {15, {0, 0}}};  // key: nb_dp, value: {num_blocks, l_dp_bits}

const std::unordered_map<uint8_t, std::string> OSNMA_TABLE_5 = {
    {0, std::string("Reserved")},
    {1, std::string("ECDSA P-256")},
    {2, std::string("Reserved")},
    {3, std::string("ECDSA P-521")},
    {4, std::string("OAM")},
    {5, std::string("Reserved")},
    {6, std::string("Reserved")},
    {7, std::string("Reserved")},
    {8, std::string("Reserved")},
    {9, std::string("Reserved")},
    {10, std::string("Reserved")},
    {11, std::string("Reserved")},
    {12, std::string("Reserved")},
    {13, std::string("Reserved")},
    {14, std::string("Reserved")},
    {15, std::string("Reserved")}};  // key: nptk, value: message

const std::unordered_map<std::string, uint16_t> OSNMA_TABLE_6 = {
    {std::string("ECDSA P-256"), 264},
    {std::string("ECDSA P-521"), 536}};

// OSNMA SIS ICD v1.0 Table 7
const std::unordered_map<uint8_t, std::pair<uint16_t, uint16_t>> OSNMA_TABLE_7 = {
    {0, {0, 0}},
    {1, {7, 728}},
    {2, {8, 832}},
    {3, {9, 936}},
    {4, {10, 1040}},
    {5, {11, 1144}},
    {6, {12, 1248}},
    {7, {13, 1352}},
    {8, {14, 1456}},
    {9, {0, 0}},
    {10, {0, 0}},
    {11, {0, 0}},
    {12, {0, 0}},
    {13, {0, 0}},
    {14, {0, 0}},
    {15, {0, 0}}};  // key: nb_dk, value: {num_blocks, l_dk_bits}

const std::unordered_map<uint8_t, std::string> OSNMA_TABLE_8 = {
    {0, std::string("SHA-256")},
    {1, std::string("Reserved")},
    {2, std::string("SHA3-256")},
    {3, std::string("Reserved")}};  // key: hs, value: hash_function

const std::unordered_map<uint8_t, uint16_t> OSNMA_TABLE_10 = {
    {0, 96},
    {1, 104},
    {2, 112},
    {3, 120},
    {4, 128},
    {5, 160},
    {6, 192},
    {7, 224},
    {8, 256},
    {9, 0},
    {10, 0},
    {11, 0},
    {12, 0},
    {13, 0},
    {15, 0},
    {15, 0}};  // key: ks, value: lk_bits

const std::unordered_map<std::string, std::pair<uint16_t, uint16_t>> OSNMA_TABLE_15 = {
    {std::string("ECDSA P-256"), {512, 256}},
    {std::string("ECDSA P-521"), {1059, 521}}};  // key: ECDSA Curve and hash function, value: {l_ds_bits, key_lenght_bits}


osnma_msg_receiver_sptr osnma_msg_receiver_make()
{
    return osnma_msg_receiver_sptr(new osnma_msg_receiver());
}


osnma_msg_receiver::osnma_msg_receiver() : gr::block("osnma_msg_receiver",
                                               gr::io_signature::make(0, 0, 0),
                                               gr::io_signature::make(0, 0, 0))
{
    // register OSNMA input message port from telemetry blocks
    this->message_port_register_in(pmt::mp("OSNMA_from_TLM"));
    // register OSNMA output message port to PVT block
    this->message_port_register_out(pmt::mp("OSNMA_to_PVT"));

    this->set_msg_handler(pmt::mp("OSNMA_from_TLM"),
#if HAS_GENERIC_LAMBDA
        [this](auto&& PH1) { msg_handler_osnma(PH1); });
#else
#if USE_BOOST_BIND_PLACEHOLDERS
        boost::bind(&osnma_msg_receiver::msg_handler_osnma, this, boost::placeholders::_1));
#else
        boost::bind(&osnma_msg_receiver::msg_handler_osnma, this, _1));
#endif
#endif
}


void osnma_msg_receiver::msg_handler_osnma(const pmt::pmt_t& msg)
{
    // requires mutex with msg_handler_osnma function called by the scheduler
    gr::thread::scoped_lock lock(d_setlock);
    try
        {
            const size_t msg_type_hash_code = pmt::any_ref(msg).type().hash_code();
            if (msg_type_hash_code == typeid(std::shared_ptr<OSNMA_msg>).hash_code())
                {
                    const auto nma_msg = wht::any_cast<std::shared_ptr<OSNMA_msg>>(pmt::any_ref(msg));
                    process_osnma_message(nma_msg);
                }
            else
                {
                    LOG(WARNING) << "osnma_msg_receiver received an unknown object type!";
                }
        }
    catch (const wht::bad_any_cast& e)
        {
            LOG(WARNING) << "osnma_msg_receiver Bad any_cast: " << e.what();
        }

    //  Send the resulting decoded NMA data (if available) to PVT
    if (d_new_data == true)
        {
            auto osnma_data_ptr = std::make_shared<OSNMA_data>(d_osnma_data);
            this->message_port_pub(pmt::mp("OSNMA_to_PVT"), pmt::make_any(osnma_data_ptr));
            d_new_data = false;
            DLOG(INFO) << "NMA info sent to the PVT block through the OSNMA_to_PVT async message port";
        }
}


void osnma_msg_receiver::process_osnma_message(const std::shared_ptr<OSNMA_msg>& osnma_msg)
{
    const auto hkroot_msg = osnma_msg->hkroot;
    read_nma_header(hkroot_msg[0]);
    read_dsm_header(hkroot_msg[1]);
    read_dsm_block(osnma_msg);
}


void osnma_msg_receiver::read_nma_header(uint8_t nma_header)
{
    d_osnma_data.d_nma_header.nmas = (nma_header & 0b11000000) >> 6;
    d_osnma_data.d_nma_header.cid = (nma_header & 0b00110000) >> 4;
    d_osnma_data.d_nma_header.cpks = (nma_header & 0b00001110) >> 1;
    d_osnma_data.d_nma_header.reserved = ((nma_header & 0b00000001) ? true : false);
}


void osnma_msg_receiver::read_dsm_header(uint8_t dsm_header)
{
    d_osnma_data.d_dsm_header.dsm_id = (dsm_header & 0b11110000) >> 4;
    d_osnma_data.d_dsm_header.dsm_block_id = dsm_header & 0b00001111;  // BID
}


void osnma_msg_receiver::read_dsm_block(const std::shared_ptr<OSNMA_msg>& osnma_msg)
{
    size_t i = 0;
    for (auto it = osnma_msg->hkroot.cbegin() + 2; it != osnma_msg->hkroot.cend(); ++it)
        {
            d_dsm_message[d_osnma_data.d_dsm_header.dsm_id][13 * d_osnma_data.d_dsm_header.dsm_block_id + i] = *it;
            i++;
        }
    if (d_osnma_data.d_dsm_header.dsm_block_id == 0)
        {
            // Get number of blocks in message
            uint8_t nb = (osnma_msg->hkroot[2] & 0b11110000) >> 4;
            uint16_t number_of_blocks = 0;
            if (d_osnma_data.d_dsm_header.dsm_id < 12)
                {
                    // Table 3
                    const auto it = OSNMA_TABLE_3.find(nb);
                    if (it != OSNMA_TABLE_3.cend())
                        {
                            number_of_blocks = it->second.first;
                        }
                }
            else if (d_osnma_data.d_dsm_header.dsm_id >= 12 && d_osnma_data.d_dsm_header.dsm_id < 16)
                {
                    // Table 7
                    const auto it = OSNMA_TABLE_7.find(nb);
                    if (it != OSNMA_TABLE_7.cend())
                        {
                            number_of_blocks = it->second.first;
                        }
                }

            d_number_of_blocks[d_osnma_data.d_dsm_header.dsm_id] = number_of_blocks;
        }
    // Annotate bid
    d_dsm_id_received[d_osnma_data.d_dsm_header.dsm_id][d_osnma_data.d_dsm_header.dsm_block_id] = 1;
    // is message complete? -> process_dsm_message(osnma_msg)
    if ((d_number_of_blocks[d_osnma_data.d_dsm_header.dsm_id] != 0) && (d_number_of_blocks[d_osnma_data.d_dsm_header.dsm_id] == std::accumulate(d_dsm_id_received[d_osnma_data.d_dsm_header.dsm_id].begin(), d_dsm_id_received[d_osnma_data.d_dsm_header.dsm_id].end(), 0)))
        {
            std::vector<uint8_t> dsm_msg(std::size_t(d_number_of_blocks[d_osnma_data.d_dsm_header.dsm_id]) * 13, 0);
            for (uint32_t i = 0; i < d_number_of_blocks[d_osnma_data.d_dsm_header.dsm_id]; i++)
                {
                    for (uint32_t j = 0; j < 14; j++)
                        {
                            dsm_msg[i * 13 + j] = d_dsm_message[d_osnma_data.d_dsm_header.dsm_id][i * 13 + j];
                        }
                }
            process_dsm_message(osnma_msg, dsm_msg);
        }
}


void osnma_msg_receiver::process_dsm_message(const std::shared_ptr<OSNMA_msg>& osnma_msg, const std::vector<uint8_t>& dsm_msg)
{
    if (d_osnma_data.d_dsm_header.dsm_id < 12)
        {
            // DSM-KROOT message
            d_osnma_data.d_dsm_kroot_message.nb_dk = (dsm_msg[0] & 0b11110000) >> 4;
            d_osnma_data.d_dsm_kroot_message.pkid = (dsm_msg[0] & 0b00001111);
            d_osnma_data.d_dsm_kroot_message.cidkr = (dsm_msg[1] & 0b11000000) >> 6;
            d_osnma_data.d_dsm_kroot_message.reserved1 = (dsm_msg[1] & 0b00110000) >> 4;
            d_osnma_data.d_dsm_kroot_message.hf = (dsm_msg[1] & 0b00001100) >> 2;

            d_osnma_data.d_dsm_kroot_message.mf = (dsm_msg[1] & 0b00000011);
            d_osnma_data.d_dsm_kroot_message.ks = (dsm_msg[2] & 0b11110000) >> 4;
            d_osnma_data.d_dsm_kroot_message.ts = (dsm_msg[2] & 0b00001111);
            d_osnma_data.d_dsm_kroot_message.maclt = dsm_msg[3];
            d_osnma_data.d_dsm_kroot_message.reserved = (dsm_msg[4] & 0b11110000) >> 4;
            d_osnma_data.d_dsm_kroot_message.wn_k = static_cast<uint16_t>((dsm_msg[4] & 0b00001111) << 8) +
                                                    static_cast<uint16_t>(dsm_msg[5]);
            d_osnma_data.d_dsm_kroot_message.towh_k = dsm_msg[6];

            uint32_t lk = 0;
            const auto it = OSNMA_TABLE_10.find(d_osnma_data.d_dsm_kroot_message.ks);
            if (it != OSNMA_TABLE_10.cend())
                {
                    lk = it->second;
                }

            d_osnma_data.d_dsm_kroot_message.alpha = (static_cast<uint64_t>(dsm_msg[7]) << 40) +
                                                     (static_cast<uint64_t>(dsm_msg[8]) << 32) +
                                                     (static_cast<uint64_t>(dsm_msg[9]) << 24) +
                                                     (static_cast<uint64_t>(dsm_msg[10]) << 16) +
                                                     (static_cast<uint64_t>(dsm_msg[11]) << 8) +
                                                     static_cast<uint64_t>(dsm_msg[12]);
            uint32_t bytes_lk = lk / 8;
            d_osnma_data.d_dsm_kroot_message.kroot = std::vector<uint8_t>(bytes_lk, 0);
            for (uint32_t k = 0; k < bytes_lk; k++)
                {
                    d_osnma_data.d_dsm_kroot_message.kroot[k] = dsm_msg[13 + k];
                }
            // uint32_t l_ld = 0;
            //  const auto it2 = OSNMA_TABLE_5.find()
            //   d_osnma_data.d_dsm_kroot_message.ds = "0";
            //  d_osnma_data.d_dsm_kroot_message.p_dk;
        }
    else if (d_osnma_data.d_dsm_header.dsm_id >= 12 && d_osnma_data.d_dsm_header.dsm_id < 16)
        {
            // DSM-PKR message
            d_osnma_data.d_dsm_pkr_message.nb_dp = (dsm_msg[0] & 0b11110000) >> 4;
            d_osnma_data.d_dsm_pkr_message.mid = (dsm_msg[0] & 0b00001111);
            for (int k = 0; k > 128; k++)
                {
                    d_osnma_data.d_dsm_pkr_message.itn[k] = dsm_msg[k + 1];
                }

            d_osnma_data.d_dsm_pkr_message.npkt = (dsm_msg[129] & 0b11110000) >> 4;
            d_osnma_data.d_dsm_pkr_message.npktid = (dsm_msg[129] & 0b00001111);

            // Table 5
            uint32_t l_npk = 0;
            const auto it = OSNMA_TABLE_5.find(d_osnma_data.d_dsm_pkr_message.npkt);
            if (it != OSNMA_TABLE_5.cend())
                {
                    const auto it2 = OSNMA_TABLE_6.find(it->second);
                    if (it2 != OSNMA_TABLE_6.cend())
                        {
                            l_npk = it2->second / 8;
                        }
                }

            if (d_osnma_data.d_dsm_pkr_message.npkt == 4)
                {
                    // OAM
                    l_npk = 0;  // ?
                }

            d_osnma_data.d_dsm_pkr_message.npk = std::vector<uint8_t>(l_npk, 0);
            for (uint32_t k = 0; k > l_npk; k++)
                {
                    d_osnma_data.d_dsm_pkr_message.npk[k] = dsm_msg[k + 130];
                }
            uint32_t l_dp = dsm_msg.size();
            uint32_t l_pd = l_dp - 130 - l_npk;
            d_osnma_data.d_dsm_pkr_message.p_dp = std::vector<uint8_t>(l_pd, 0);
            for (uint32_t k = 0; k > l_pd; k++)
                {
                    d_osnma_data.d_dsm_pkr_message.p_dp[k] = dsm_msg[l_dp - l_pd + k];
                }
        }
}
