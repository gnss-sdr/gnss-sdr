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
#include "Galileo_OSNMA.h"
#include "gnss_crypto.h"
#include "gnss_satellite.h"
#include "osnma_dsm_reader.h"       // for OSNMA_DSM_Reader
#include <glog/logging.h>           // for DLOG
#include <gnuradio/io_signature.h>  // for gr::io_signature::make
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <numeric>
#include <typeinfo>  // for typeid

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


osnma_msg_receiver_sptr osnma_msg_receiver_make(const std::string& pemFilePath, const std::string& merkleFilePath)
{
    return osnma_msg_receiver_sptr(new osnma_msg_receiver(pemFilePath, merkleFilePath));
}


osnma_msg_receiver::osnma_msg_receiver(
    const std::string& pemFilePath,
    const std::string& merkleFilePath) : gr::block("osnma_msg_receiver",
                                             gr::io_signature::make(0, 0, 0),
                                             gr::io_signature::make(0, 0, 0))
{
    d_dsm_reader = std::make_unique<OSNMA_DSM_Reader>();
    d_crypto = std::make_unique<Gnss_Crypto>(pemFilePath, merkleFilePath);
    d_old_mack_message.set_capacity(10);
    //  register OSNMA input message port from telemetry blocks
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
                    const auto sat = Gnss_Satellite(std::string("Galileo"), nma_msg->PRN);
                    std::cout << "Galileo OSNMA: Subframe received starting at "
                              << "WN="
                              << nma_msg->WN_sf0
                              << ", TOW="
                              << nma_msg->TOW_sf0
                              << ", from satellite "
                              << sat
                              << std::endl;
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
    if (d_new_data == true) // TODO where is it set to true?
        {
            auto osnma_data_ptr = std::make_shared<OSNMA_data>(d_osnma_data); // C: why create new object and pass it empty to Pvt?
            this->message_port_pub(pmt::mp("OSNMA_to_PVT"), pmt::make_any(osnma_data_ptr));
            d_new_data = false;
            // d_osnma_data = OSNMA_data();
            DLOG(INFO) << "NMA info sent to the PVT block through the OSNMA_to_PVT async message port";
        }
}


void osnma_msg_receiver::process_osnma_message(const std::shared_ptr<OSNMA_msg>& osnma_msg)
{
    read_nma_header(osnma_msg->hkroot[0]);
    read_dsm_header(osnma_msg->hkroot[1]);
    read_dsm_block(osnma_msg);
    read_mack_block(osnma_msg);
}


void osnma_msg_receiver::read_nma_header(uint8_t nma_header)
{
    d_osnma_data.d_nma_header.nmas = d_dsm_reader->get_nmas(nma_header);
    d_osnma_data.d_nma_header.cid = d_dsm_reader->get_cid(nma_header);
    d_osnma_data.d_nma_header.cpks = d_dsm_reader->get_cpks(nma_header);
    d_osnma_data.d_nma_header.reserved = d_dsm_reader->get_nma_header_reserved(nma_header);
}


void osnma_msg_receiver::read_dsm_header(uint8_t dsm_header)
{
    d_osnma_data.d_dsm_header.dsm_id = d_dsm_reader->get_dsm_id(dsm_header);
    d_osnma_data.d_dsm_header.dsm_block_id = d_dsm_reader->get_dsm_block_id(dsm_header);  // BID
    LOG(WARNING) << "OSNMA: DSM_ID=" << static_cast<uint32_t>(d_osnma_data.d_dsm_header.dsm_id);
    LOG(WARNING) << "OSNMA: DSM_BID=" << static_cast<uint32_t>(d_osnma_data.d_dsm_header.dsm_block_id);
    std::cout << "Galileo OSNMA: Received block " << static_cast<uint32_t>(d_osnma_data.d_dsm_header.dsm_block_id)
              << " from DSM_ID " << static_cast<uint32_t>(d_osnma_data.d_dsm_header.dsm_id)
              << std::endl;
}


void osnma_msg_receiver::read_dsm_block(const std::shared_ptr<OSNMA_msg>& osnma_msg)
{
    size_t index = 0;
    for (const auto* it = osnma_msg->hkroot.cbegin() + 2; it != osnma_msg->hkroot.cend(); ++it)
        {
            d_dsm_message[d_osnma_data.d_dsm_header.dsm_id][SIZE_DSM_BLOCKS_BYTES * d_osnma_data.d_dsm_header.dsm_block_id + index] = *it;
            index++;
        }
    if (d_osnma_data.d_dsm_header.dsm_block_id == 0)
        {
            // Get number of blocks in message
            uint8_t nb = d_dsm_reader->get_number_blocks_index(d_dsm_message[d_osnma_data.d_dsm_header.dsm_id][0]);
            uint16_t number_of_blocks = 0;
            if (d_osnma_data.d_dsm_header.dsm_id < 12)
                {
                    // DSM-KROOT Table 7
                    const auto it = OSNMA_TABLE_7.find(nb);
                    if (it != OSNMA_TABLE_7.cend())
                        {
                            number_of_blocks = it->second.first;
                        }
                }
            else if (d_osnma_data.d_dsm_header.dsm_id >= 12 && d_osnma_data.d_dsm_header.dsm_id < 16)
                {
                    // DSM-PKR Table 3
                    const auto it = OSNMA_TABLE_3.find(nb);
                    if (it != OSNMA_TABLE_3.cend())
                        {
                            number_of_blocks = it->second.first;
                        }
                }

            d_number_of_blocks[d_osnma_data.d_dsm_header.dsm_id] = number_of_blocks;
            LOG(WARNING) << "OSNMA: number_of_blocks=" << static_cast<uint32_t>(number_of_blocks);
            if (number_of_blocks == 0)
                {
                    // Something is wrong, start over
                    LOG(WARNING) << "OSNMA: Wrong number of blocks, start over";
                    d_dsm_message[d_osnma_data.d_dsm_header.dsm_id] = std::array<uint8_t, 256>{};
                    d_dsm_id_received[d_osnma_data.d_dsm_header.dsm_id] = std::array<uint8_t, 16>{};
                }
        }
    // Annotate bid
    d_dsm_id_received[d_osnma_data.d_dsm_header.dsm_id][d_osnma_data.d_dsm_header.dsm_block_id] = 1;

    std::cout << "Galileo OSNMA: Available blocks for DSM_ID " << static_cast<uint32_t>(d_osnma_data.d_dsm_header.dsm_id) << ": [ ";
    if (d_number_of_blocks[d_osnma_data.d_dsm_header.dsm_id] == 0)
        {
            for (auto id_received : d_dsm_id_received[d_osnma_data.d_dsm_header.dsm_id])
                {
                    if (id_received == 0)
                        {
                            std::cout << "- ";
                        }
                    else
                        {
                            std::cout << "X ";
                        }
                }
        }
    else
        {
            for (uint8_t k = 0; k < d_number_of_blocks[d_osnma_data.d_dsm_header.dsm_id]; k++)
                {
                    if (d_dsm_id_received[d_osnma_data.d_dsm_header.dsm_id][k] == 0)
                        {
                            std::cout << "- ";
                        }
                    else
                        {
                            std::cout << "X ";
                        }
                }
        }
    std::cout << "]" << std::endl;

    // is message complete? -> Process DSM message
    if ((d_number_of_blocks[d_osnma_data.d_dsm_header.dsm_id] != 0) &&
        (d_number_of_blocks[d_osnma_data.d_dsm_header.dsm_id] == std::accumulate(d_dsm_id_received[d_osnma_data.d_dsm_header.dsm_id].cbegin(), d_dsm_id_received[d_osnma_data.d_dsm_header.dsm_id].cend(), 0)))
        {
            std::vector<uint8_t> dsm_msg(std::size_t(d_number_of_blocks[d_osnma_data.d_dsm_header.dsm_id]) * SIZE_DSM_BLOCKS_BYTES, 0);
            for (uint32_t i = 0; i < d_number_of_blocks[d_osnma_data.d_dsm_header.dsm_id]; i++)
                {
                    for (size_t j = 0; j < SIZE_DSM_BLOCKS_BYTES; j++)
                        {
                            dsm_msg[i * SIZE_DSM_BLOCKS_BYTES + j] = d_dsm_message[d_osnma_data.d_dsm_header.dsm_id][i * SIZE_DSM_BLOCKS_BYTES + j];
                        }
                }
            d_dsm_message[d_osnma_data.d_dsm_header.dsm_id] = std::array<uint8_t, 256>{};
            d_dsm_id_received[d_osnma_data.d_dsm_header.dsm_id] = std::array<uint8_t, 16>{};
            process_dsm_message(dsm_msg, osnma_msg);
        }
}


void osnma_msg_receiver::process_dsm_message(const std::vector<uint8_t>& dsm_msg, const std::shared_ptr<OSNMA_msg>& osnma_msg)
{
    if (d_osnma_data.d_dsm_header.dsm_id < 12)
        {
            LOG(WARNING) << "OSNMA: DSM-KROOT message received.";
            // DSM-KROOT message
            d_osnma_data.d_dsm_kroot_message.nb_dk = d_dsm_reader->get_number_blocks_index(dsm_msg[0]);
            d_osnma_data.d_dsm_kroot_message.pkid = d_dsm_reader->get_pkid(dsm_msg);
            d_osnma_data.d_dsm_kroot_message.cidkr = d_dsm_reader->get_cidkr(dsm_msg);
            d_osnma_data.d_dsm_kroot_message.reserved1 = d_dsm_reader->get_dsm_reserved1(dsm_msg);
            d_osnma_data.d_dsm_kroot_message.hf = d_dsm_reader->get_hf(dsm_msg);
            d_osnma_data.d_dsm_kroot_message.mf = d_dsm_reader->get_mf(dsm_msg);
            d_osnma_data.d_dsm_kroot_message.ks = d_dsm_reader->get_ks(dsm_msg);
            d_osnma_data.d_dsm_kroot_message.ts = d_dsm_reader->get_ts(dsm_msg);
            d_osnma_data.d_dsm_kroot_message.maclt = d_dsm_reader->get_maclt(dsm_msg);
            d_osnma_data.d_dsm_kroot_message.reserved = d_dsm_reader->get_dsm_reserved(dsm_msg);
            d_osnma_data.d_dsm_kroot_message.wn_k = d_dsm_reader->get_wn_k(dsm_msg);
            d_osnma_data.d_dsm_kroot_message.towh_k = d_dsm_reader->get_towh_k(dsm_msg);
            d_osnma_data.d_dsm_kroot_message.alpha = d_dsm_reader->get_alpha(dsm_msg);

            const uint16_t l_lk_bytes = d_dsm_reader->get_lk_bits(d_osnma_data.d_dsm_kroot_message.ks) / 8;
            d_osnma_data.d_dsm_kroot_message.kroot = d_dsm_reader->get_kroot(dsm_msg, l_lk_bytes);

            std::string hash_function = d_dsm_reader->get_hash_function(d_osnma_data.d_dsm_kroot_message.hf);
            uint16_t l_ds_bits = 0;
            const auto it = OSNMA_TABLE_15.find(hash_function);
            if (it != OSNMA_TABLE_15.cend())
                {
                    l_ds_bits = it->second;
                }
            const uint16_t l_ds_bytes = l_ds_bits / 8;
            d_osnma_data.d_dsm_kroot_message.ds = std::vector<uint8_t>(l_ds_bytes, 0);
            for (uint16_t k = 0; k < l_ds_bytes; k++)
                {
                    d_osnma_data.d_dsm_kroot_message.ds[k] = dsm_msg[13 + l_lk_bytes + k];
                }
            const uint16_t l_dk_bits = d_dsm_reader->get_l_dk_bits(d_osnma_data.d_dsm_kroot_message.nb_dk);
            const uint16_t l_dk_bytes = l_dk_bits / 8;
            const uint16_t l_pdk_bytes = (l_dk_bytes - 13 - l_lk_bytes - l_ds_bytes);
            d_osnma_data.d_dsm_kroot_message.p_dk = std::vector<uint8_t>(l_pdk_bytes, 0);
            for (uint16_t k = 0; k < l_pdk_bytes; k++)
                {
                    d_osnma_data.d_dsm_kroot_message.p_dk[k] = dsm_msg[13 + l_lk_bytes + l_ds_bytes + k];
                }

            const uint16_t check_l_dk = 104 * std::ceil(1.0 + static_cast<float>((l_lk_bytes * 8.0) + l_ds_bits) / 104.0);
            if (l_dk_bits != check_l_dk)
                {
                    std::cout << "Galileo OSNMA: Failed length reading" << std::endl;
                }
            else
                {
                    // validation of padding
                    const uint16_t size_m = 13 + l_lk_bytes;
                    std::vector<uint8_t> MSG;
                    MSG.reserve(size_m + l_ds_bytes + 1);
                    MSG.push_back(osnma_msg->hkroot[0]); // C: NMA header
                    for (uint16_t i = 1; i < size_m; i++)
                        {
                            MSG.push_back(dsm_msg[i]);
                        }
                    std::vector<uint8_t> message = MSG; // C: MSG == M || DS from ICD. Eq. 7
                    for (uint16_t k = 0; k < l_ds_bytes; k++)
                        {
                            MSG.push_back(d_osnma_data.d_dsm_kroot_message.ds[k]);
                        }

                    std::vector<uint8_t> hash;
                    if (d_osnma_data.d_dsm_kroot_message.hf == 0)  // Table 8.
                        {
                            hash = d_crypto->computeSHA256(MSG);
                        }
                    else if (d_osnma_data.d_dsm_kroot_message.hf == 2)
                        {
                            hash = d_crypto->computeSHA3_256(MSG);
                        }
                    else
                        {
                            hash = std::vector<uint8_t>(32);
                        }
                    // truncate hash
                    std::vector<uint8_t> p_dk_truncated;
                    p_dk_truncated.reserve(l_pdk_bytes);
                    for (uint16_t i = 0; i < l_pdk_bytes; i++)
                        {
                            p_dk_truncated.push_back(hash[i]);
                        }
                    // check DS signature
                    if (d_osnma_data.d_dsm_kroot_message.p_dk == p_dk_truncated)
                        {
                            bool authenticated = d_crypto->verify_signature(message, d_osnma_data.d_dsm_kroot_message.ds);
                            LOG(WARNING) << "OSNMA: DSM-KROOT message received ok.";
                            std::cout << "Galileo OSNMA: KROOT with CID=" << static_cast<uint32_t>(d_osnma_data.d_nma_header.cid)
                                      << ", PKID=" << static_cast<uint32_t>(d_osnma_data.d_dsm_kroot_message.pkid)
                                      << ", WN=" << static_cast<uint32_t>(d_osnma_data.d_dsm_kroot_message.wn_k)
                                      << ", TOW=" << static_cast<uint32_t>(d_osnma_data.d_dsm_kroot_message.towh_k) * 3600;
                            if (authenticated)
                                {
                                    std::cout << " authenticated" << std::endl; // C: proceed with Tesla chain key verification.
                                }
                            else
                                {
                                    std::cout << " validated" << std::endl; // C: Kroot not verified => retrieve it again
                                }
                            std::cout << "Galileo OSNMA: NMA Status is " << d_dsm_reader->get_nmas_status(d_osnma_data.d_nma_header.nmas) << ", "
                                      << "Chain in force is " << static_cast<uint32_t>(d_osnma_data.d_nma_header.cid) << ", "
                                      << "Chain and Public Key Status is " << d_dsm_reader->get_cpks_status(d_osnma_data.d_nma_header.cpks) << std::endl;
                        }
                    else
                        {
                            std::cout << "Galileo OSNMA: Error computing padding bits." << std::endl;
                        }
                }
        }
    else if (d_osnma_data.d_dsm_header.dsm_id >= 12 && d_osnma_data.d_dsm_header.dsm_id < 16)
        {
            LOG(WARNING) << "OSNMA: DSM-PKR message received.";
            // DSM-PKR message
            d_osnma_data.d_dsm_pkr_message.nb_dp = d_dsm_reader->get_number_blocks_index(dsm_msg[0]);
            d_osnma_data.d_dsm_pkr_message.mid = d_dsm_reader->get_mid(dsm_msg);
            for (int k = 0; k > 128; k++)
                {
                    d_osnma_data.d_dsm_pkr_message.itn[k] = dsm_msg[k + 1];
                }
            d_osnma_data.d_dsm_pkr_message.npkt = d_dsm_reader->get_npkt(dsm_msg);
            d_osnma_data.d_dsm_pkr_message.npktid = d_dsm_reader->get_npktid(dsm_msg);

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
            uint32_t l_dp = dsm_msg.size();
            if (d_osnma_data.d_dsm_pkr_message.npkt == 4)
                {
                    LOG(WARNING) << "OSNMA: OAM received";
                    l_npk = l_dp - 130;  // bytes
                }

            d_osnma_data.d_dsm_pkr_message.npk = std::vector<uint8_t>(l_npk, 0);  // ECDSA Public Key
            for (uint32_t k = 0; k > l_npk; k++)
                {
                    d_osnma_data.d_dsm_pkr_message.npk[k] = dsm_msg[k + 130];
                }

            uint32_t l_pd = l_dp - 130 - l_npk;
            uint32_t check_l_dp = 104 * std::ceil(static_cast<float>(1040.0 + l_npk * 8.0) / 104.0);
            if (l_dp != check_l_dp)
                {
                    std::cout << "Galileo OSNMA: Failed length reading" << std::endl;
                }
            else
                {
                    d_osnma_data.d_dsm_pkr_message.p_dp = std::vector<uint8_t>(l_pd, 0);
                    for (uint32_t k = 0; k < l_pd; k++)
                        {
                            d_osnma_data.d_dsm_pkr_message.p_dp[k] = dsm_msg[l_dp - l_pd + k];
                        }
                    // std::vector<uint8_t> mi;  //  (NPKT + NPKID + NPK)
                    std::cout << "Galileo OSNMA: DSM-PKR with CID=" << static_cast<uint32_t>(d_osnma_data.d_nma_header.cid)
                              << ", PKID=" << static_cast<uint32_t>(d_osnma_data.d_dsm_kroot_message.pkid)
                              << ", WN=" << static_cast<uint32_t>(d_osnma_data.d_dsm_kroot_message.wn_k)
                              << ", TOW=" << static_cast<uint32_t>(d_osnma_data.d_dsm_kroot_message.towh_k) * 3600
                              << " received" << std::endl;
                    // C: NPK verification against Merkle tree root.
                    d_public_key_verified = verify_dsm_pkr(d_osnma_data.d_dsm_pkr_message);
                }
        }
    else
        {
            // Reserved message?
            LOG(WARNING) << "OSNMA Reserved message received";
            // d_osnma_data = OSNMA_data();
        }
    d_number_of_blocks[d_osnma_data.d_dsm_header.dsm_id] = 0;
}


void osnma_msg_receiver::read_mack_block(const std::shared_ptr<OSNMA_msg>& osnma_msg)
{
    uint32_t index = 0;
    for (uint32_t value : osnma_msg->mack)
        {
            d_mack_message[index] = static_cast<uint8_t>((value & 0xFF000000) >> 24);
            d_mack_message[index + 1] = static_cast<uint8_t>((value & 0x00FF0000) >> 16);
            d_mack_message[index + 2] = static_cast<uint8_t>((value & 0x0000FF00) >> 8);
            d_mack_message[index + 3] = static_cast<uint8_t>(value & 0x000000FF);
            index = index + 4;
        }
    if (d_osnma_data.d_dsm_kroot_message.ts != 0) // C: 4 ts <  ts < 10
        {
            read_mack_header();
            read_mack_body();
            process_mack_message(osnma_msg);
        }
}


void osnma_msg_receiver::read_mack_header()
{ // C: still to review computations.
    uint8_t lt_bits = 0;
    const auto it = OSNMA_TABLE_11.find(d_osnma_data.d_dsm_kroot_message.ts);
    if (it != OSNMA_TABLE_11.cend())
        {
            lt_bits = it->second;
        }
    if (lt_bits == 0)
        {
            return; // C: TODO if Tag length is 0, what is the action? no verification possible of NavData for sure.
        }
    uint16_t macseq = 0;
    uint8_t cop = 0;
    uint64_t first_lt_bits = static_cast<uint64_t>(d_mack_message[0]) << (lt_bits - 8);
    first_lt_bits += (static_cast<uint64_t>(d_mack_message[1]) << (lt_bits - 16));
    if (lt_bits == 20)
        {
            first_lt_bits += (static_cast<uint64_t>(d_mack_message[1] & 0xF0) >> 4);
            macseq += (static_cast<uint16_t>(d_mack_message[1] & 0x0F) << 8);
            macseq += static_cast<uint16_t>(d_mack_message[2]);
            cop += ((d_mack_message[3] & 0xF0) >> 4);
        }
    else if (lt_bits == 24)
        {
            first_lt_bits += static_cast<uint64_t>(d_mack_message[2]);
            macseq += (static_cast<uint16_t>(d_mack_message[3]) << 4);
            macseq += (static_cast<uint16_t>(d_mack_message[4] & 0xF0) >> 4);
            cop += (d_mack_message[4] & 0x0F);
        }
    else if (lt_bits == 28)
        {
            first_lt_bits += (static_cast<uint64_t>(d_mack_message[2]) << 4);
            first_lt_bits += (static_cast<uint64_t>(d_mack_message[3] & 0xF0) >> 4);
            macseq += (static_cast<uint16_t>(d_mack_message[3] & 0x0F) << 8);
            macseq += (static_cast<uint16_t>(d_mack_message[4]));
            cop += ((d_mack_message[5] & 0xF0) >> 4);
        }
    else if (lt_bits == 32)
        {
            first_lt_bits += (static_cast<uint64_t>(d_mack_message[2]) << 8);
            first_lt_bits += static_cast<uint64_t>(d_mack_message[3]);
            macseq += (static_cast<uint16_t>(d_mack_message[4]) << 4);
            macseq += (static_cast<uint16_t>(d_mack_message[5] & 0xF0) >> 4);
            cop += (d_mack_message[5] & 0x0F);
        }
    else if (lt_bits == 40)
        {
            first_lt_bits += (static_cast<uint64_t>(d_mack_message[2]) << 16);
            first_lt_bits += (static_cast<uint64_t>(d_mack_message[3]) << 8);
            first_lt_bits += static_cast<uint64_t>(d_mack_message[4]);
            macseq += (static_cast<uint16_t>(d_mack_message[5]) << 4);
            macseq += (static_cast<uint16_t>(d_mack_message[6] & 0xF0) >> 4);
            cop += (d_mack_message[6] & 0x0F);
        }
    d_osnma_data.d_mack_message.header.tag0 = first_lt_bits;
    d_osnma_data.d_mack_message.header.macseq = macseq;
    d_osnma_data.d_mack_message.header.cop = cop;
}


void osnma_msg_receiver::read_mack_body()
{

    uint8_t lt_bits = 0;
    const auto it = OSNMA_TABLE_11.find(d_osnma_data.d_dsm_kroot_message.ts);
    if (it != OSNMA_TABLE_11.cend())
        {
            lt_bits = it->second;
        }
    if (lt_bits == 0)
        {
            return;
        }
    uint16_t lk_bits = 0;
    const auto it2 = OSNMA_TABLE_10.find(d_osnma_data.d_dsm_kroot_message.ks);
    if (it2 != OSNMA_TABLE_10.cend())
        {
            lk_bits = it2->second;
        }
    if (lk_bits == 0)
        {
            return;
        }
    uint16_t nt = std::floor((480.0 - float(lk_bits)) / (float(lt_bits) + 16.0)); // C: compute number of tags
    d_osnma_data.d_mack_message.tag_and_info = std::vector<MACK_tag_and_info>(nt - 1);
    for (uint16_t k = 0; k < (nt - 1); k++) // C: retrieve Tag&Info
        {
            uint64_t tag = 0;
            uint8_t PRN_d = 0;
            uint8_t ADKD = 0;
            uint8_t cop = 0;
            if (lt_bits == 20)
                {
                    const uint16_t step = std::ceil(4.5 * k);
                    if (k % 2 == 0)
                        {
                            tag += (static_cast<uint64_t>((d_mack_message[3 + step] & 0x0F)) << 16);
                            tag += (static_cast<uint64_t>(d_mack_message[4 + step]) << 8);
                            tag += static_cast<uint64_t>(d_mack_message[5 + step]);
                            PRN_d += d_mack_message[6 + step];
                            ADKD += ((d_mack_message[7 + step] & 0xF0) >> 4);
                            cop += (d_mack_message[7 + step] & 0x0F);
                            if (k == (nt - 2))
                                {
                                    d_osnma_data.d_mack_message.key = std::vector<uint8_t>(d_osnma_data.d_dsm_kroot_message.kroot.size());
                                    for (size_t j = 0; j < d_osnma_data.d_dsm_kroot_message.kroot.size(); j++)
                                        {
                                            d_osnma_data.d_mack_message.key[j] = d_mack_message[8 + step + j];
                                        }
                                }
                        }
                    else
                        {
                            tag += (static_cast<uint64_t>(d_mack_message[3 + step]) << 12);
                            tag += (static_cast<uint64_t>(d_mack_message[4 + step]) << 4);
                            tag += (static_cast<uint64_t>((d_mack_message[5 + step] & 0xF0)) >> 4);
                            PRN_d += (d_mack_message[5 + step] & 0x0F) << 4;
                            PRN_d += (d_mack_message[6 + step] & 0xF0) >> 4;
                            ADKD += (d_mack_message[6 + step] & 0x0F);
                            cop += (d_mack_message[7 + step] & 0xF0) >> 4;
                            if (k == (nt - 2))
                                {
                                    d_osnma_data.d_mack_message.key = std::vector<uint8_t>(d_osnma_data.d_dsm_kroot_message.kroot.size());
                                    for (size_t j = 0; j < d_osnma_data.d_dsm_kroot_message.kroot.size(); j++)
                                        {
                                            d_osnma_data.d_mack_message.key[j] = ((d_mack_message[7 + step + j] & 0x0F) << 4) + ((d_mack_message[8 + step + j] & 0xF0) >> 4);
                                        }
                                }
                        }
                }
            else if (lt_bits == 24)
                {
                    tag += (static_cast<uint64_t>((d_mack_message[5 + k * 5])) << 16);
                    tag += (static_cast<uint64_t>((d_mack_message[6 + k * 5])) << 8);
                    tag += static_cast<uint64_t>(d_mack_message[7 + k * 5]);
                    PRN_d += d_mack_message[8 + k * 5];
                    ADKD += ((d_mack_message[9 + k * 5] & 0xF0) >> 4);
                    cop += (d_mack_message[9 + k * 5] & 0x0F);
                    if (k == (nt - 2))
                        {
                            d_osnma_data.d_mack_message.key = std::vector<uint8_t>(d_osnma_data.d_dsm_kroot_message.kroot.size());
                            for (size_t j = 0; j < d_osnma_data.d_dsm_kroot_message.kroot.size(); j++)
                                {
                                    d_osnma_data.d_mack_message.key[j] = d_mack_message[10 + k * 5 + j];
                                }
                        }
                }
            else if (lt_bits == 28)
                {
                    const uint16_t step = std::ceil(5.5 * k);
                    if (k % 2 == 0)
                        {
                            tag += (static_cast<uint64_t>((d_mack_message[5 + step] & 0x0F)) << 24);
                            tag += (static_cast<uint64_t>(d_mack_message[6 + step]) << 16);
                            tag += (static_cast<uint64_t>(d_mack_message[7 + step]) << 8);
                            tag += static_cast<uint64_t>(d_mack_message[8 + step]);
                            PRN_d += d_mack_message[9 + step];
                            ADKD += ((d_mack_message[10 + step] & 0xF0) >> 4);
                            cop += (d_mack_message[10 + step] & 0x0F);
                            if (k == (nt - 2))
                                {
                                    d_osnma_data.d_mack_message.key = std::vector<uint8_t>(d_osnma_data.d_dsm_kroot_message.kroot.size());
                                    for (size_t j = 0; j < d_osnma_data.d_dsm_kroot_message.kroot.size(); j++)
                                        {
                                            d_osnma_data.d_mack_message.key[j] = d_mack_message[11 + step + j];
                                        }
                                }
                        }
                    else
                        {
                            tag += (static_cast<uint64_t>((d_mack_message[5 + step])) << 20);
                            tag += (static_cast<uint64_t>((d_mack_message[6 + step])) << 12);
                            tag += (static_cast<uint64_t>((d_mack_message[7 + step])) << 4);
                            tag += (static_cast<uint64_t>((d_mack_message[8 + step] & 0xF0)) >> 4);
                            PRN_d += ((d_mack_message[8 + step] & 0x0F) << 4);
                            PRN_d += ((d_mack_message[9 + step] & 0xF0) >> 4);
                            ADKD += (d_mack_message[9 + step] & 0x0F);
                            cop += ((d_mack_message[10 + step] & 0xF0) >> 4);
                            if (k == (nt - 2))
                                {
                                    d_osnma_data.d_mack_message.key = std::vector<uint8_t>(d_osnma_data.d_dsm_kroot_message.kroot.size());
                                    for (size_t j = 0; j < d_osnma_data.d_dsm_kroot_message.kroot.size(); j++)
                                        {
                                            d_osnma_data.d_mack_message.key[j] = ((d_mack_message[10 + step + j] & 0x0F) << 4) + ((d_mack_message[11 + step + j] & 0xF0) >> 4);
                                        }
                                }
                        }
                }
            else if (lt_bits == 32)
                {
                    tag += (static_cast<uint64_t>((d_mack_message[6 + k * 6])) << 24);
                    tag += (static_cast<uint64_t>((d_mack_message[7 + k * 6])) << 16);
                    tag += (static_cast<uint64_t>((d_mack_message[8 + k * 6])) << 8);
                    tag += static_cast<uint64_t>(d_mack_message[9 + k * 6]);
                    PRN_d += d_mack_message[10 + k * 6];
                    ADKD += ((d_mack_message[11 + k * 6] & 0xF0) >> 4);
                    cop += (d_mack_message[11 + k * 6] & 0x0F);
                    if (k == (nt - 2))
                        {
                            d_osnma_data.d_mack_message.key = std::vector<uint8_t>(d_osnma_data.d_dsm_kroot_message.kroot.size());
                            for (size_t j = 0; j < d_osnma_data.d_dsm_kroot_message.kroot.size(); j++)
                                {
                                    d_osnma_data.d_mack_message.key[j] = d_mack_message[12 + k * 6 + j];
                                }
                        }
                }
            else if (lt_bits == 40)
                {
                    tag += (static_cast<uint64_t>((d_mack_message[7 + k * 7])) << 32);
                    tag += (static_cast<uint64_t>((d_mack_message[8 + k * 7])) << 24);
                    tag += (static_cast<uint64_t>((d_mack_message[9 + k * 7])) << 16);
                    tag += (static_cast<uint64_t>((d_mack_message[10 + k * 7])) << 8);
                    tag += static_cast<uint64_t>(d_mack_message[11 + k * 7]);
                    PRN_d += d_mack_message[12 + k * 7];
                    ADKD += ((d_mack_message[13 + k * 7] & 0xF0) >> 4);
                    cop += (d_mack_message[13 + k * 7] & 0x0F);
                    if (k == (nt - 2))
                        {
                            d_osnma_data.d_mack_message.key = std::vector<uint8_t>(d_osnma_data.d_dsm_kroot_message.kroot.size());
                            for (size_t j = 0; j < d_osnma_data.d_dsm_kroot_message.kroot.size(); j++)
                                {
                                    d_osnma_data.d_mack_message.key[j] = d_mack_message[14 + k * 7 + j];
                                }
                        }
                }
            d_osnma_data.d_mack_message.tag_and_info[k].tag = tag;
            d_osnma_data.d_mack_message.tag_and_info[k].tag_info.PRN_d = PRN_d;
            d_osnma_data.d_mack_message.tag_and_info[k].tag_info.ADKD = ADKD;
            d_osnma_data.d_mack_message.tag_and_info[k].tag_info.cop = cop;
        }
    // TODO - [TAS-110]
    //      retrieve TESLA key: index i is [479-MH(l_t)-(nt-1)*(16+l_t), i+l_k]
    //      compute I (GST_SFi,GST_Kroot)
    //      perform recursive hash calls until base case: i = 10...1

    // retrieve tesla key
    uint8_t start_index_bytes = ( 480 - (lt_bits + 16) - (nt - 1) * ( lt_bits + 16 ) - 1 ) / 8; // includes -1 to start at [i-1]
    uint8_t last_index_bytes = ( start_index_bytes * 8 + lk_bits ) / 8;
    uint8_t key_index_bytes = 0;
    for (uint8_t i = start_index_bytes; i < last_index_bytes ; i++, key_index_bytes++)
        {
            d_osnma_data.d_mack_message.key[key_index_bytes] = d_mack_message[i];
        }

}


void osnma_msg_receiver::process_mack_message(const std::shared_ptr<OSNMA_msg>& osnma_msg)
{
    d_old_mack_message.push_back(d_osnma_data.d_mack_message); // C: old mack message is needed  for

    // MACSEQ validation - case no FLX Tags
    uint32_t GST_SF = osnma_msg->TOW_sf0;

    // C: TODO check ADKD-MACLT for match, also identify which tags are FLX
    // C: TODO if Tag_x FLX => MACSEQ, otherwise ___ ?
    // Are there flexible tags?
    // d_osnma_data.d_dsm_kroot_message.maclt
    // d_osnma_data.d_mack_message.tag_and_info[k].tag

    // retrieve data to verify MACK tags
    uint8_t msg {0};
    uint8_t nt {0};
    std::vector<std::string> sq1{};
    std::vector<std::string> sq2{};
    const auto it = OSNMA_TABLE_16.find(d_osnma_data.d_dsm_kroot_message.maclt);
    if (it != OSNMA_TABLE_16.cend())
        {
            auto msg = it->second.msg;
            auto nt = it->second.nt;
            auto sq1 = it->second.sequence1;
            auto sq2 = it->second.sequence2;
        }
    if (msg == 0)
        {
          return;
        }
    // compare ADKD of Mack tags with MACLT defined ADKDs
    // TODO - "When it is equal to 2, the sequence starts with the MACK message transmitted in the first 30 seconds of a GST minute."
    if(d_osnma_data.d_mack_message.tag_and_info.size() != sq1.size()) // TODO - which sequence is retrieved in this Mack?
        {
          std::cout << "Galileo OSNMA: Number of retrieved tags does not match MACLT sequence size!" << std::endl;
          return;
        }
    bool allOk = true;
    std::string selfAutenticated {};
    for (uint8_t i = 0; i < d_osnma_data.d_mack_message.tag_and_info.size(); i++)
        {
            if(d_osnma_data.d_mack_message.tag_and_info[i].tag_info.ADKD != std::stoi(sq1[i])) // C: undefined if format is not "00S"
                {
                    allOk = false;
                    break;
                }
            selfAutenticated = sq1[i][sq1[i].size()-1];
        }


    std::vector<uint8_t> m(5); // C: ICD - Eq. 22
    m[0] = static_cast<uint8_t>(osnma_msg->PRN);  // PRN_A
    m[1] = ((GST_SF & 0xF000) >> 24);
    m[2] = ((GST_SF & 0x0F00) >> 16);
    m[3] = ((GST_SF & 0x00F0) >> 8);
    m[4] = (GST_SF & 0x000F);

    std::vector<uint8_t> applicable_key;
    // if ADKD=12, pick d_old_mack_message.front() if d_old_mack_message[10] is full
    // otherwise pick d_old_mack_message.back()
    applicable_key = d_old_mack_message.back().key;
    std::vector<uint8_t> mac;
    if (d_osnma_data.d_dsm_kroot_message.mf == 0) // C: HMAC-SHA-256
        {
            mac = d_crypto->computeHMAC_SHA_256(applicable_key, m);
        }
    else if (d_osnma_data.d_dsm_kroot_message.mf == 1) // C: CMAC-AES
        {
            mac = d_crypto->computeCMAC_AES(applicable_key, m);
        }
    uint16_t mac_msb = 0;
    if (!mac.empty())
        {
            mac_msb = (mac[0] << 8) + mac[1];
        }
    uint16_t computed_macseq = (mac_msb & 0x0FFF);
    int num_tags_added = 0;
    if (computed_macseq == d_osnma_data.d_mack_message.header.macseq)
        {
            std::cout << "OSNMA: MACSEQ authenticated for PRN_A "
                      << osnma_msg->PRN << " with WN="
                      << osnma_msg->WN_sf0 << ", TOW="
                      << osnma_msg->TOW_sf0 << ". Tags added: "
                      << num_tags_added << std::endl;
        }

    // C: TODO - for each tag in tag_and_info[] && until l_t_verified <= L_t_min
    // C:  d_osnma_data.d_dsm_kroot_message.ts gives l_t of each tag for this mack
    // C:  tag = trunc(l_t, mac(applicable_key,m))
    // C:  where m = (PRNd || PRNa || GSTsf || CTR || NMAS || NavData || P)
    // C: si l_t_verified >= L_t_min d_new_data = true
}

bool osnma_msg_receiver::verify_dsm_pkr(DSM_PKR_message message)
{
    // TODO create leafe base message m_i
    // TODO create function for recursively apply hash

    // build base leaf m_i
//    auto leaf = message.mid;
    std::vector<uint8_t> m_i;
    m_i.reserve(2 + message.npk.size());
    m_i[0] = message.npkt;
    m_i[1] = message.npktid;
    for (uint8_t i = 2; i < m_i.size(); i++)
        {
            m_i.push_back(message.npk[i]);
        }

    // compute intermediate leafs' values
    std::vector<uint8_t> x_0,x_1,x_2,x_3,x_4;
//    uint8_t  k = 0;
    x_0 = d_crypto->computeSHA256(m_i);
    x_0.insert(x_0.end(),message.itn.begin(),&message.itn[31]);
    x_1 = d_crypto->computeSHA256(x_0);
    x_1.insert(x_1.end(),&message.itn[32],&message.itn[63]);
    x_2 = d_crypto->computeSHA256(x_1);
    x_2.insert(x_2.end(),&message.itn[64],&message.itn[95]);
    x_3 = d_crypto->computeSHA256(x_2);
    x_3.insert(x_3.end(),&message.itn[96],&message.itn[127]);
    // root leaf computation
    x_4 = d_crypto->computeSHA256(x_3);

    // C: d_crypto->getMerkleRoot([m_0:m_15]) I realised I could have done this...
    // C: ... but why computing all the possible results?  I have only one leaf in each osnma message...
    // verify that computed root matches merkle root

    if(x_4 == d_crypto->getMerkleRoot())
        {
            std::cout << "Galileo OSNMA: DSM-PKR verified successfully! " << std::endl;
            return true;
            // C: NPK verification against Merkle tree root.
        }
    else
        {
            std::cout << "Galileo OSNMA: DSM-PKR verification unsuccessful !" << std::endl;
            return false;
        }
}
