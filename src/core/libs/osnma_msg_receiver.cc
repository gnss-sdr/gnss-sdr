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
#include "gnss_sdr_make_unique.h"   // for std::make_unique in C++11
#include <glog/logging.h>           // for DLOG
#include <gnuradio/io_signature.h>  // for gr::io_signature::make
#include <bitset>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <iterator>
#include <numeric>
#include <string>
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

#if USE_OPENSSL_FALLBACK
#include <openssl/sha.h>
#else
#include <gnutls/crypto.h>
#include <gnutls/gnutls.h>
#endif

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
            d_osnma_data = OSNMA_data();
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
    d_osnma_data.d_nma_header.nmas = get_nmas(nma_header);
    d_osnma_data.d_nma_header.cid = get_cid(nma_header);
    d_osnma_data.d_nma_header.cpks = get_cpks(nma_header);
    d_osnma_data.d_nma_header.reserved = get_nma_header_reserved(nma_header);
}


void osnma_msg_receiver::read_dsm_header(uint8_t dsm_header)
{
    d_osnma_data.d_dsm_header.dsm_id = get_dsm_id(dsm_header);
    d_osnma_data.d_dsm_header.dsm_block_id = get_dsm_block_id(dsm_header);  // BID
    LOG(WARNING) << "OSNMA: DSM_ID=" << static_cast<uint32_t>(d_osnma_data.d_dsm_header.dsm_id);
    LOG(WARNING) << "OSNMA: DSM_BID=" << static_cast<uint32_t>(d_osnma_data.d_dsm_header.dsm_block_id);
}


void osnma_msg_receiver::read_dsm_block(const std::shared_ptr<OSNMA_msg>& osnma_msg)
{
    size_t index = 0;
    for (const auto* it = osnma_msg->hkroot.cbegin() + 2; it != osnma_msg->hkroot.cend(); ++it)
        {
            d_dsm_message[d_osnma_data.d_dsm_header.dsm_id][13 * d_osnma_data.d_dsm_header.dsm_block_id + index] = *it;
            index++;
        }

    if (d_osnma_data.d_dsm_header.dsm_block_id == 0)
        {
            // Get number of blocks in message
            uint8_t nb = get_number_blocks_index(d_dsm_message[d_osnma_data.d_dsm_header.dsm_id][0]);
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

    // is message complete? -> Process DSM message
    if ((d_number_of_blocks[d_osnma_data.d_dsm_header.dsm_id] != 0) &&
        (d_number_of_blocks[d_osnma_data.d_dsm_header.dsm_id] == std::accumulate(d_dsm_id_received[d_osnma_data.d_dsm_header.dsm_id].cbegin(), d_dsm_id_received[d_osnma_data.d_dsm_header.dsm_id].cend(), 0)))
        {
            std::vector<uint8_t> dsm_msg(std::size_t(d_number_of_blocks[d_osnma_data.d_dsm_header.dsm_id]) * 13, 0);
            for (uint32_t i = 0; i < d_number_of_blocks[d_osnma_data.d_dsm_header.dsm_id]; i++)
                {
                    for (uint32_t j = 0; j < 14; j++)
                        {
                            dsm_msg[i * 13 + j] = d_dsm_message[d_osnma_data.d_dsm_header.dsm_id][i * 13 + j];
                        }
                }
            d_dsm_message[d_osnma_data.d_dsm_header.dsm_id] = std::array<uint8_t, 256>{};
            d_dsm_id_received[d_osnma_data.d_dsm_header.dsm_id] = std::array<uint8_t, 16>{};
            process_dsm_message(dsm_msg, osnma_msg->hkroot[0]);
        }
}


void osnma_msg_receiver::process_dsm_message(const std::vector<uint8_t>& dsm_msg, uint8_t nma_header)
{
    if (d_osnma_data.d_dsm_header.dsm_id < 12)
        {
            LOG(WARNING) << "OSNMA: DSM-KROOT message received.";
            // DSM-KROOT message
            d_osnma_data.d_dsm_kroot_message.nb_dk = get_number_blocks_index(dsm_msg[0]);
            d_osnma_data.d_dsm_kroot_message.pkid = get_pkid(dsm_msg);
            d_osnma_data.d_dsm_kroot_message.cidkr = get_cidkr(dsm_msg);
            d_osnma_data.d_dsm_kroot_message.reserved1 = get_dsm_reserved1(dsm_msg);
            d_osnma_data.d_dsm_kroot_message.hf = get_hf(dsm_msg);
            d_osnma_data.d_dsm_kroot_message.mf = get_mf(dsm_msg);
            d_osnma_data.d_dsm_kroot_message.ks = get_ks(dsm_msg);
            d_osnma_data.d_dsm_kroot_message.ts = get_ts(dsm_msg);
            d_osnma_data.d_dsm_kroot_message.maclt = get_maclt(dsm_msg);
            d_osnma_data.d_dsm_kroot_message.reserved = get_dsm_reserved(dsm_msg);
            d_osnma_data.d_dsm_kroot_message.wn_k = get_wn_k(dsm_msg);
            d_osnma_data.d_dsm_kroot_message.towh_k = get_towh_k(dsm_msg);
            d_osnma_data.d_dsm_kroot_message.alpha = get_alpha(dsm_msg);

            LOG(WARNING) << "nb_dk=" << static_cast<uint32_t>(d_osnma_data.d_dsm_kroot_message.nb_dk);
            LOG(WARNING) << "pkid=" << static_cast<uint32_t>(d_osnma_data.d_dsm_kroot_message.pkid);
            LOG(WARNING) << "cidkr=" << static_cast<uint32_t>(d_osnma_data.d_dsm_kroot_message.cidkr);
            LOG(WARNING) << "reserved1=" << static_cast<uint32_t>(d_osnma_data.d_dsm_kroot_message.reserved1);
            LOG(WARNING) << "hf=" << static_cast<uint32_t>(d_osnma_data.d_dsm_kroot_message.hf);
            LOG(WARNING) << "mf=" << static_cast<uint32_t>(d_osnma_data.d_dsm_kroot_message.mf);
            LOG(WARNING) << "ks=" << static_cast<uint32_t>(d_osnma_data.d_dsm_kroot_message.ks);
            LOG(WARNING) << "ts=" << static_cast<uint32_t>(d_osnma_data.d_dsm_kroot_message.ts);
            LOG(WARNING) << "maclt=" << static_cast<uint32_t>(d_osnma_data.d_dsm_kroot_message.maclt);
            LOG(WARNING) << "reserved=" << static_cast<uint32_t>(d_osnma_data.d_dsm_kroot_message.reserved);
            LOG(WARNING) << "wn_k=" << static_cast<uint32_t>(d_osnma_data.d_dsm_kroot_message.wn_k);
            LOG(WARNING) << "towh_k=" << static_cast<uint32_t>(d_osnma_data.d_dsm_kroot_message.towh_k);
            LOG(WARNING) << "alpha=" << d_osnma_data.d_dsm_kroot_message.alpha;

            uint16_t bytes_lk = get_lk_bits(d_osnma_data.d_dsm_kroot_message.ks) / 8;
            d_osnma_data.d_dsm_kroot_message.kroot = get_kroot(dsm_msg, bytes_lk);
            LOG(WARNING) << "lk_bits=" << static_cast<uint32_t>(get_lk_bits(d_osnma_data.d_dsm_kroot_message.ks));
            LOG(WARNING) << "lk_bytes=" << static_cast<uint32_t>(bytes_lk);

            std::string hash_function = get_hash_function(d_osnma_data.d_dsm_kroot_message.hf);

            uint16_t l_ds_bits = 0;
            const auto it4 = OSNMA_TABLE_15.find(hash_function);
            if (it4 != OSNMA_TABLE_15.cend())
                {
                    l_ds_bits = it4->second;
                }
            uint16_t l_ds_bytes = l_ds_bits / 8;
            LOG(WARNING) << "ds_bits=" << static_cast<uint32_t>(l_ds_bits);
            LOG(WARNING) << "ds_bytes=" << static_cast<uint32_t>(l_ds_bytes);
            d_osnma_data.d_dsm_kroot_message.ds = std::vector<uint8_t>(l_ds_bytes, 0);
            for (uint16_t k = 0; k < l_ds_bytes; k++)
                {
                    d_osnma_data.d_dsm_kroot_message.ds[k] = dsm_msg[13 + bytes_lk + k];
                }
            uint16_t l_dk_bits = get_l_dk_bits(d_osnma_data.d_dsm_kroot_message.nb_dk);
            uint16_t l_dk_bytes = l_dk_bits / 8;
            LOG(WARNING) << "dk_bits=" << static_cast<uint32_t>(l_dk_bits);
            LOG(WARNING) << "dk_bytes=" << static_cast<uint32_t>(l_dk_bytes);

            uint16_t l_pdk_bytes = (l_dk_bytes - 13 - bytes_lk - l_ds_bytes);
            LOG(WARNING) << "pdk_bytes=" << static_cast<uint32_t>(l_pdk_bytes);
            d_osnma_data.d_dsm_kroot_message.p_dk = std::vector<uint8_t>(l_pdk_bytes, 0);
            for (uint16_t k = 0; k < l_pdk_bytes; k++)
                {
                    d_osnma_data.d_dsm_kroot_message.p_dk[k] = dsm_msg[13 + bytes_lk + l_ds_bytes + k];
                }

            uint16_t check_l_dk = 104 * std::ceil(1.0 + static_cast<float>((bytes_lk * 8.0) + l_ds_bits) / 104.0);
            LOG(WARNING) << "check_l_dk_bits=" << static_cast<uint32_t>(check_l_dk);
            if (l_dk_bits != check_l_dk)
                {
                    std::cout << "OSNMA: Failed length reading" << std::endl;
                }
            else
                {
                    // validation of padding
                    std::vector<uint8_t> MSG;
                    MSG.push_back(nma_header);
                    for (int i = 1; i < 13; i++)
                        {
                            MSG.push_back(dsm_msg[i]);
                        }
                    for (uint16_t i = 0; i < bytes_lk; i++)
                        {
                            MSG.push_back(dsm_msg[13 + i]);
                        }
                    for (uint16_t k = 0; k < l_ds_bytes; k++)
                        {
                            MSG.push_back(dsm_msg[13 + bytes_lk + k]);
                        }

                    std::vector<uint8_t> hash = computeSHA256(MSG);
                    std::cout << "hash: ";
                    for (auto c : hash)
                        {
                            std::cout << static_cast<uint32_t>(c);
                        }
                    std::cout << std::endl;

                    std::cout << "pdk: ";
                    for (auto c : d_osnma_data.d_dsm_kroot_message.p_dk)
                        {
                            std::cout << static_cast<uint32_t>(c);
                        }
                    std::cout << std::endl;
                    // truncate hash
                    std::vector<uint8_t> p_dk_computed;
                    for (uint16_t i = 0; i < l_pdk_bytes; i++)
                        {
                            p_dk_computed.push_back(hash[i]);
                        }
                    if (d_osnma_data.d_dsm_kroot_message.p_dk == p_dk_computed)
                        {
                            std::cout << "OSNMA: DSM-KROOT message validated" << std::endl;
                        }
                    // Validate signature
                }
        }
    else if (d_osnma_data.d_dsm_header.dsm_id >= 12 && d_osnma_data.d_dsm_header.dsm_id < 16)
        {
            LOG(WARNING) << "OSNMA: DSM-PKR message received.";
            // DSM-PKR message
            d_osnma_data.d_dsm_pkr_message.nb_dp = get_number_blocks_index(dsm_msg[0]);
            d_osnma_data.d_dsm_pkr_message.mid = get_mid(dsm_msg);
            for (int k = 0; k > 128; k++)
                {
                    d_osnma_data.d_dsm_pkr_message.itn[k] = dsm_msg[k + 1];
                }
            d_osnma_data.d_dsm_pkr_message.npkt = get_npkt(dsm_msg);
            d_osnma_data.d_dsm_pkr_message.npktid = get_npktid(dsm_msg);

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
                    LOG(WARNING) << "OSNMA: OAM received";
                    l_npk = 0;  // ?
                }

            d_osnma_data.d_dsm_pkr_message.npk = std::vector<uint8_t>(l_npk, 0);  // ECDSA Public Key
            for (uint32_t k = 0; k > l_npk; k++)
                {
                    d_osnma_data.d_dsm_pkr_message.npk[k] = dsm_msg[k + 130];
                }
            uint32_t l_dp = dsm_msg.size();
            uint32_t l_pd = l_dp - 130 - l_npk;

            uint32_t check_l_dp = 104 * std::ceil(static_cast<float>(1040.0 + l_npk * 8.0) / 104.0);

            if (l_dp != check_l_dp)
                {
                    std::cout << "OSNMA: Failed length reading" << std::endl;
                }
            else
                {
                    d_osnma_data.d_dsm_pkr_message.p_dp = std::vector<uint8_t>(l_pd, 0);
                    for (uint32_t k = 0; k < l_pd; k++)
                        {
                            d_osnma_data.d_dsm_pkr_message.p_dp[k] = dsm_msg[l_dp - l_pd + k];
                        }
                }
        }
    else
        {
            // Reserved message?
            LOG(WARNING) << "OSNMA Reserved message received";
            d_osnma_data = OSNMA_data();
        }
    d_number_of_blocks[d_osnma_data.d_dsm_header.dsm_id] = 0;
}


void osnma_msg_receiver::read_mack_block(const std::shared_ptr<OSNMA_msg>& osnma_msg)
{
    uint32_t index = 0;
    for (size_t i = 0; i < osnma_msg->mack.size(); i++)
        {
            uint32_t value = osnma_msg->mack[i];
            d_mack_message[index] = static_cast<uint8_t>((value & 0xFF000000) >> 6);
            d_mack_message[index + 1] = static_cast<uint8_t>((value & 0x00FF0000) >> 4);
            d_mack_message[index + 2] = static_cast<uint8_t>((value & 0x0000FF00) >> 2);
            d_mack_message[index + 3] = static_cast<uint8_t>(value & 0x000000FF);
            index = index + 4;
        }
    read_mack_header();
    read_mack_info_and_tags();
    read_mack_key();
    read_mack_padding();
}


void osnma_msg_receiver::read_mack_header()
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
}


void osnma_msg_receiver::read_mack_info_and_tags()
{
}


void osnma_msg_receiver::read_mack_key()
{
}


void osnma_msg_receiver::read_mack_padding()
{
}


std::vector<uint8_t> osnma_msg_receiver::computeSHA256(const std::vector<uint8_t>& input)
{
    std::vector<uint8_t> output(32);  // SHA256 hash size
#if USE_OPENSSL_FALLBACK
    SHA256_CTX sha256Context;
    SHA256_Init(&sha256Context);
    SHA256_Update(&sha256Context, input.data(), input.size());
    SHA256_Final(output.data(), &sha256Context);
#else
    gnutls_hash_hd_t hashHandle;
    gnutls_hash_init(&hashHandle, GNUTLS_DIG_SHA256);
    gnutls_hash(hashHandle, input.data(), input.size());
    gnutls_hash_output(hashHandle, output.data());
    gnutls_hash_deinit(hashHandle, output.data());
#endif
    return output;
}

// bool signature(const std::vector<uint8_t>& publicKey, const std::vector<uint8_t>& publicKey)
// {
//     bool success = false;
// #if USE_OPENSSL_FALLBACK
// #else
//     gnutls_global_init();
//     int result = gnutls_pubkey_verify_data(publicKey, GNUTLS_SIGN_ECDSA_SHA256, digest, sizeof(digest), signature, signatureSize);
//     success = (result == GNUTLS_E_SUCCESS);
// #endif
//     return success;
// }
