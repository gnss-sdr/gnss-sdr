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
    //d_old_mack_message.set_capacity(10);
    d_old_OSNMA_buffer.set_capacity(25);
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
            auto osnma_data_ptr = std::make_shared<OSNMA_data>(d_osnma_data);
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
    local_time_verification(osnma_msg);
    process_dsm_block(osnma_msg); // will process dsm block if received a complete one, then will call mack processing upon re-setting the dsm block to 0
    read_and_process_mack_block(osnma_msg); // only process them if a least 3 available.
}


/**
 * @brief Reads the NMA header from the given input and stores the values in the d_osnma_data structure.
 *
 * The NMA header consists of several fields: d_nma_header.nmas, d_nma_header.cid, d_nma_header.cpks, and d_nma_header.reserved.
 * Each field is retrieved using the corresponding getter functions from the d_dsm_reader auxiliary object.
 *
 * @param nma_header The input containing the NMA header.
 */
void osnma_msg_receiver::read_nma_header(uint8_t nma_header)
{
    d_osnma_data.d_nma_header.nmas = d_dsm_reader->get_nmas(nma_header);
    d_osnma_data.d_nma_header.cid = d_dsm_reader->get_cid(nma_header);
    d_osnma_data.d_nma_header.cpks = d_dsm_reader->get_cpks(nma_header);
    d_osnma_data.d_nma_header.reserved = d_dsm_reader->get_nma_header_reserved(nma_header);
}


/**
 * @brief Read the DSM header from the given dsm_header and populate the d_osnma_data structure.
 *
 * @param dsm_header The DSM header.
 */
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

/*
 * accumulates dsm messages until completeness, then calls process_dsm_message
 * */
void osnma_msg_receiver::read_dsm_block(const std::shared_ptr<OSNMA_msg>& osnma_msg)
{
    // Fill d_dsm_message
    size_t index = 0;
    for (const auto* it = osnma_msg->hkroot.cbegin() + 2; it != osnma_msg->hkroot.cend(); ++it)
        {
            d_dsm_message[d_osnma_data.d_dsm_header.dsm_id][SIZE_DSM_BLOCKS_BYTES * d_osnma_data.d_dsm_header.dsm_block_id + index] = *it;
            index++;
        }
    // First block indicates number of blocks in DSM message
    if (d_osnma_data.d_dsm_header.dsm_block_id == 0)
        {

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
    // TODO FIXME   Galileo OSNMA: Available blocks for DSM_ID 6: [ - - - - - X - - - - - - - - - - ] in the first received Sf.. size 16?
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
    std::cout << "]" << std::endl; // TODO update documentation
}

/**
 * @brief Function to verify the local time based on GST_SIS and GST_0
 *
 * @param osnma_msg Shared pointer to OSNMA message structure
 */
void osnma_msg_receiver::local_time_verification(const std::shared_ptr<OSNMA_msg>& osnma_msg)
{
    // compute local time based on GST_SIS and GST_0
    d_GST_SIS = (osnma_msg->WN_sf0 & 0x00000FFF) << 20 | osnma_msg->TOW_sf0 & 0x000FFFFF;
    //std::cout << "Galileo OSNMA: d_GST_SIS: " << d_GST_SIS << std::endl;
    //d_GST_0 = d_osnma_data.d_dsm_kroot_message.towh_k + 604800 * d_osnma_data.d_dsm_kroot_message.wn_k + 30;
    d_GST_0 = ((d_osnma_data.d_dsm_kroot_message.wn_k  & 0x00000FFF) << 20 | d_osnma_data.d_dsm_kroot_message.towh_k & 0x000FFFFF) + 30;
    //d_GST_0 = d_osnma_data.d_dsm_kroot_message.towh_k + 604800 * d_osnma_data.d_dsm_kroot_message.wn_k + 30;
    // TODO store list of SVs sending OSNMA and if received ID matches one stored, then just increment time 30s for that ID.
    if(d_receiver_time != 0)
        {

            d_receiver_time = d_GST_0 + 30 * std::floor((d_GST_SIS - d_GST_0) / 30); // Eq. 3 R.G.
//            d_receiver_time += 30;
            //std::cout << "Galileo OSNMA: d_receiver_time: " << d_receiver_time << std::endl;

        }
    else
        {// local time not initialised -> compute it.
            d_receiver_time = d_GST_0 + 30 * std::floor((d_GST_SIS - d_GST_0) / 30); // Eq. 3 R.G.
            //std::cout << "Galileo OSNMA: d_receiver_time: " << d_receiver_time << std::endl;
        }
    // verify time constraint
    std::time_t delta_T = abs(d_receiver_time - d_GST_SIS);
    if(  delta_T <= d_T_L )
        {
            d_tags_allowed = tags_to_verify::all;
            d_tags_to_verify = {0,4,12};
            std::cout << "Galileo OSNMA: time constraint OK \n";
            std::cout << "Galileo OSNMA: d_receiver_time: " << d_receiver_time << " d_GST_SIS: " << d_GST_SIS << "\n";
            //std::cout << "( |local_t - GST_SIS| < T_L ) [ |" << static_cast<int>(d_receiver_time - d_GST_SIS)<< " | < " << static_cast<int>(d_T_L) << " ]" << std::endl;

            // TODO set flag to false to avoid processing dsm and MACK messages
        }
    else if( delta_T > d_T_L && delta_T <= 10* delta_T  )
        {
            d_tags_allowed = tags_to_verify::slow_eph;
            d_tags_to_verify = {12};
            std::cout << "Galileo OSNMA: time constraint allows only slow MACs to be verified\n";
            std::cout << "Galileo OSNMA: d_receiver_time: " << d_receiver_time << " d_GST_SIS: " << d_GST_SIS << "\n";
            std::cout << "( |local_t - GST_SIS| < T_L ) [ |" << static_cast<int>(d_receiver_time - d_GST_SIS) << " | < " << static_cast<int>(d_T_L) << " ]" << std::endl;

        }
    else
        {
            d_tags_allowed = tags_to_verify::none;
            d_tags_to_verify = {};
            std::cerr << "Galileo OSNMA: time constraint violation\n";
            std::cout << "Galileo OSNMA: d_receiver_time: " << d_receiver_time << " d_GST_SIS: " << d_GST_SIS << "\n";
            std::cout << "( |local_t - GST_SIS| < T_L ) [ |" << static_cast<int>(d_receiver_time - d_GST_SIS) << " | < " << static_cast<int>(d_T_L) << " ]" << std::endl;

        }

}

/**
 * @brief Process DSM block of an OSNMA message.
 *
 * \details This function checks if all inner blocks of the DSM message are available and if so, calls process_dsm_message().
 * \post It creates a vector to hold the DSM message data, copies the data from the inner blocks into the vector,
 * resets the inner block arrays to empty
 *
 * @param osnma_msg The OSNMA message.
 */
void osnma_msg_receiver::process_dsm_block(const std::shared_ptr<OSNMA_msg>& osnma_msg)
{
    // if all inner blocks available -> Process DSM message
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

/*
 * case DSM-Kroot:
 * - computes the padding and compares with received message
 * - if successful, tries to verify the digital signature
 * case DSM-PKR:
 * - calls verify_dsm_pkr to verify the public key
 * */
void osnma_msg_receiver::process_dsm_message(const std::vector<uint8_t>& dsm_msg, const std::shared_ptr<OSNMA_msg>& osnma_msg)
{
    // DSM-KROOT message
    if (d_osnma_data.d_dsm_header.dsm_id < 12)
        {
            LOG(WARNING) << "OSNMA: DSM-KROOT message received.";
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
            d_osnma_data.d_dsm_kroot_message.ds = std::vector<uint8_t>(l_ds_bytes, 0); // C: this accounts for padding in case needed.
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
                    MSG.reserve(size_m + l_ds_bytes + 1); // C: message will get too many zeroes? ((12+1)+16) + 64 + 1? => in theory not, allocating is not assigning
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
                    // Check that the padding bits received match the computed values
                    if (d_osnma_data.d_dsm_kroot_message.p_dk == p_dk_truncated)
                        {

                            LOG(WARNING) << "OSNMA: DSM-KROOT message received ok.";
                            std::cout << "Galileo OSNMA: KROOT with CID=" << static_cast<uint32_t>(d_osnma_data.d_nma_header.cid)
                                      << ", PKID=" << static_cast<uint32_t>(d_osnma_data.d_dsm_kroot_message.pkid)
                                      << ", WN=" << static_cast<uint32_t>(d_osnma_data.d_dsm_kroot_message.wn_k)
                                      << ", TOW=" << static_cast<uint32_t>(d_osnma_data.d_dsm_kroot_message.towh_k) * 3600;
                            d_kroot_verified = d_crypto->verify_signature(message, d_osnma_data.d_dsm_kroot_message.ds);
                            if (d_kroot_verified)
                                {
                                    std::cout << "Galileo OSNMA: KROOT authentication successful !" << std::endl;
                                    std::cout << "Galileo OSNMA: NMA Status is " << d_dsm_reader->get_nmas_status(d_osnma_data.d_nma_header.nmas) << ", "
                                              << "Chain in force is " << static_cast<uint32_t>(d_osnma_data.d_nma_header.cid) << ", "
                                              << "Chain and Public Key Status is " << d_dsm_reader->get_cpks_status(d_osnma_data.d_nma_header.cpks) << std::endl;
                                }
                            else
                                {
                                    std::cout << " Galileo OSNMA: KROOT authentication failed. " << std::endl;
                                }

                        }
                    else
                        {
                            std::cout << "Galileo OSNMA: Error computing padding bits." << std::endl;
                            // TODO - here will have to decide if perform the verification or not. Since this step is not mandatory, one could as well have skipped it.
                        }
                }
        }
    else if (d_osnma_data.d_dsm_header.dsm_id >= 12 && d_osnma_data.d_dsm_header.dsm_id < 16)
        {
            LOG(WARNING) << "OSNMA: DSM-PKR message received.";
            // Save DSM-PKR message
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
                    if (!d_public_key_verified)
                        {
                            bool verification = verify_dsm_pkr(d_osnma_data.d_dsm_pkr_message);
                            if (verification)
                                {
                                    d_public_key_verified = true;
                                    d_crypto->set_public_key(d_osnma_data.d_dsm_pkr_message.npk);
                                }
                        }

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


/**
 * @brief Reads the Mack message from the given OSNMA_msg object.
 *
 * @param osnma_msg The OSNMA_msg object containing the Mack message.
 */
void osnma_msg_receiver::read_and_process_mack_block(const std::shared_ptr<OSNMA_msg>& osnma_msg)
{
    // Retrieve Mack message
    uint32_t index = 0;
    for (uint32_t value : osnma_msg->mack)
        {
            d_mack_message[index] = static_cast<uint8_t>((value & 0xFF000000) >> 24);
            d_mack_message[index + 1] = static_cast<uint8_t>((value & 0x00FF0000) >> 16);
            d_mack_message[index + 2] = static_cast<uint8_t>((value & 0x0000FF00) >> 8);
            d_mack_message[index + 3] = static_cast<uint8_t>(value & 0x000000FF);
            index = index + 4;
        }


    if (d_osnma_data.d_dsm_kroot_message.ts != 0 && is_next_subframe()/* && time_constraint && */) // C: 4 ts <  ts < 10
        { // TODO - correct? with this, MACK would not be processed unless a Kroot is available
            read_mack_header();
            read_mack_body();
            process_mack_message(osnma_msg);
            // TODO - shorten the MACK processing for the cases where no TK verified or no Kroot verified (warm and cold start)
            // still, for instance the NAvData and Mack storage (within process_mack_message) makes sense.
        }
}


/**
 * \brief Reads the MACk header from the d_mack_message array and updates the d_osnma_data structure.
 * \details This function reads the message MACK header from the d_mack_message array and updates the d_osnma_data structure with the parsed data. The header consists of three fields
*: tag0, macseq, and cop. The size of the fields is determined by the number of tag length (lt) bits specified in OSNMA_TABLE_11 for the corresponding tag size in d_osnma_data.d_dsm_k
*root_message.ts. The lt_bits value is used to calculate tag0, MACSEQ, and COP.
 * \pre The d_mack_message array and d_osnma_data.d_dsm_kroot_message.ts field must be properly populated.
 * \post The d_osnma_data.d_mack_message.header.tag0, d_osnma_data.d_mack_message.header.macseq, and d_osnma_data.d_mack_message.header.cop fields are updated with the parsed values
*.
 * \returns None.
 */
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
            return; // C: TODO if Tag length is 0, what is the action? no verification possible of NavData for sure.
        }
    uint16_t macseq = 0;
    uint8_t cop = 0;
    uint64_t first_lt_bits = static_cast<uint64_t>(d_mack_message[0]) << (lt_bits - 8);
    first_lt_bits += (static_cast<uint64_t>(d_mack_message[1]) << (lt_bits - 16));
    if (lt_bits == 20)
        {
            first_lt_bits += (static_cast<uint64_t>(d_mack_message[2] & 0xF0) >> 4);
            macseq += (static_cast<uint16_t>(d_mack_message[2] & 0x0F) << 8);
            macseq += static_cast<uint16_t>(d_mack_message[3]);
            cop += ((d_mack_message[4] & 0xF0) >> 4);
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

/**
 * @brief Reads the MACK message body
 *
 * \details It retrieves all the tags and tag-info associated, as well as the TESLA key.
 * \post populates d_osnma_data.d_mack_message with all tags and tag_info associated of MACK message, as well as the TESLA key into d_osnma_data.d_mack_message.key
 * @return None
 */
void osnma_msg_receiver::read_mack_body()
{
    // retrieve tag length
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
    // retrieve key length
    const uint16_t lk_bits = d_dsm_reader->get_lk_bits(d_osnma_data.d_dsm_kroot_message.ks);
    // compute number  of tags in the given Mack message as per Eq. 8 ICD
    uint16_t nt = std::floor((480.0 - float(lk_bits)) / (float(lt_bits) + 16.0));
    d_osnma_data.d_mack_message.tag_and_info = std::vector<MACK_tag_and_info>(nt - 1);
    // retrieve tags and tag-info associated with the tags
    for (uint16_t k = 0; k < (nt - 1); k++)
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
                            tag += (static_cast<uint64_t>((d_mack_message[4 + step] & 0x0F)) << 16);
                            tag += (static_cast<uint64_t>(d_mack_message[5 + step]) << 8);
                            tag += static_cast<uint64_t>(d_mack_message[6 + step]);
                            PRN_d += d_mack_message[7 + step];
                            ADKD += ((d_mack_message[8 + step] & 0xF0) >> 4);
                            cop += (d_mack_message[8 + step] & 0x0F);
                            if (k == (nt - 2))
                                {
                                    d_osnma_data.d_mack_message.key = std::vector<uint8_t>(d_osnma_data.d_dsm_kroot_message.kroot.size());
                                    for (size_t j = 0; j < d_osnma_data.d_dsm_kroot_message.kroot.size(); j++)
                                        {
                                            d_osnma_data.d_mack_message.key[j] = d_mack_message[9 + step + j];
                                        }
                                }
                        }
                    else
                        {
                            tag += (static_cast<uint64_t>(d_mack_message[4 + step]) << 12);
                            tag += (static_cast<uint64_t>(d_mack_message[5 + step]) << 4);
                            tag += (static_cast<uint64_t>((d_mack_message[6 + step] & 0xF0)) >> 4);
                            PRN_d += (d_mack_message[6 + step] & 0x0F) << 4;
                            PRN_d += (d_mack_message[7 + step] & 0xF0) >> 4;
                            ADKD += (d_mack_message[7 + step] & 0x0F);
                            cop += (d_mack_message[8 + step] & 0xF0) >> 4;
                            if (k == (nt - 2))
                                {
                                    d_osnma_data.d_mack_message.key = std::vector<uint8_t>(d_osnma_data.d_dsm_kroot_message.kroot.size());
                                    for (size_t j = 0; j < d_osnma_data.d_dsm_kroot_message.kroot.size(); j++)
                                        {
                                            d_osnma_data.d_mack_message.key[j] = ((d_mack_message[8 + step + j] & 0x0F) << 4) + ((d_mack_message[9 + step + j] & 0xF0) >> 4);
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
                    tag += (static_cast<uint64_t>((d_mack_message[7/* bytes of MACK header */ + k * 7 /* offset of k-th tag */])) << 32);
                    tag += (static_cast<uint64_t>((d_mack_message[8 + k * 7])) << 24);
                    tag += (static_cast<uint64_t>((d_mack_message[9 + k * 7])) << 16);
                    tag += (static_cast<uint64_t>((d_mack_message[10 + k * 7])) << 8);
                    tag += static_cast<uint64_t>(d_mack_message[11 + k * 7]);
                    PRN_d += d_mack_message[12 + k * 7];
                    ADKD += ((d_mack_message[13 + k * 7] & 0xF0) >> 4);
                    cop += (d_mack_message[13 + k * 7] & 0x0F);
                    if (k == (nt - 2)) // end of Tag&Info
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
    // rest are padding bits, used for anything ?
}


/**
 * @brief Verifies the tags transmitted in the past.
 *
 * \details This function is responsible for processing the MACK message received (480 bits) at time SF(i).
 * It stores the last 10 MACK messages and the last 11 NavData messages.
 * Then attempts to verify the Tesla Key by computing the number of hashes of distance between the key-to-verify and the
 * Kroot and iteratively hashing the result, until the required number of hashes is achieved.
 * The result is then compared with the Kroot. If the two values match, the Tesla key is verified.
 *  It also performs MACSEQ validation and compares the ADKD of Mack tags with MACLT defined ADKDs.
 *  Finally, it verifies the tags.
 * \pre Kroot or already a TESLA key shall be available. Depending on the ADKD of the tag, NavData of SF(i-2)...SF(i-11)
 * \post Number of tags bits verified for each ADKD. MACSEQ verification success
 * @param osnma_msg A reference to OSNMA_msg containing the MACK message to be processed.
 */
void osnma_msg_receiver::process_mack_message(const std::shared_ptr<OSNMA_msg>& osnma_msg)
{
    d_flag_debug = true;
    // populate d_nav_data with needed data from subframe
    d_osnma_data.d_nav_data.init(osnma_msg);
    // store MACK, KROOT and NavData needed.
    d_old_OSNMA_buffer.push_back(d_osnma_data);
    if(d_old_OSNMA_buffer.size() < 3)
        {
            std::cerr << "Galileo OSNMA: MACK cannot be processed. "<< ", "
                                  << "Not enough OSNMA messages available"
                       << "buffer size: "<< d_old_OSNMA_buffer.size() << std::endl;
            return;
        }
    if(d_kroot_verified == false && d_tesla_key_verified == false)
        {
            std::cerr << "Galileo OSNMA: MACK cannot be processed. "<< ", "
                     << "No Kroot nor TESLA key available" << std::endl;
            if(!d_flag_debug)
                return; // early return, cannot proceed further without one of the two verified.
        }

    // Verify tesla key
    if(d_tesla_key_verified || d_flag_debug)
        {
            // TODO - find out I bt. both tesla keys, then hash until then, then compare.
            // retrieve latest tesla key
            // compute hashes needed
            // hash current key until num_hashes  and compare
        }
    else
        {// have to go until Kroot
            uint32_t num_of_hashes_needed = (d_receiver_time - d_GST_0) / 30 + 1; // Eq. 19 ICD
            std::cout << "Galileo OSNMA: TESLA verification ("<< num_of_hashes_needed << " hashes) need to be performed. " << std::endl;
            auto start = std::chrono::high_resolution_clock::now();
            uint32_t GST_SFi = d_receiver_time;
            std::vector<uint8_t> K_II = d_osnma_data.d_mack_message.key;
            std::vector<uint8_t> K_I; // result of the recursive hash operations
            const uint8_t lk_bytes = d_dsm_reader->get_lk_bits(d_osnma_data.d_dsm_kroot_message.ks)/8;
            // compute the tesla key for current SF (GST_SFi and K_II change in each iteration)
            for (uint32_t i = 1; i < num_of_hashes_needed ; i++)
                {
                    // build message digest m = (K_I+1 || GST_SFi || alpha)
                    // TODO sizeof() wrong.
                    std::vector<uint8_t> msg(sizeof(K_II) + sizeof(GST_SFi) + sizeof(d_osnma_data.d_dsm_kroot_message.alpha));
                    std::copy(K_II.begin(),K_II.end(),msg.begin());

                    msg.push_back((d_GST_Sf & 0xFF000000) >> 24);
                    msg.push_back((d_GST_Sf & 0x00FF0000) >> 16);
                    msg.push_back((d_GST_Sf & 0x0000FF00) >> 8);
                    msg.push_back(d_GST_Sf & 0x000000FF);
                    // extract alpha
                    for (int k = 5; k >= 0;k--)
                        {
                            // TODO: static extracts the MSB in case from larger to shorter int?
                            msg.push_back(static_cast<uint8_t>((d_osnma_data.d_dsm_kroot_message.alpha >> (i * 8)) & 0xFF)); // extract first 6 bytes of alpha.
                        }
                    // compute hash
                    std::vector<uint8_t> hash;
                    if (d_osnma_data.d_dsm_kroot_message.hf == 0)  // Table 8.
                        {
                            hash = d_crypto->computeSHA256(msg);
                        }
                    else if (d_osnma_data.d_dsm_kroot_message.hf == 2)
                        {
                            hash = d_crypto->computeSHA3_256(msg);
                        }
                    else
                        {
                            hash = std::vector<uint8_t>(32);
                        }
                    // truncate hash
                    K_I.reserve(lk_bytes); // TODO - case hash function has 512 bits
                    for (uint16_t i = 0; i < lk_bytes; i++)
                        {
                            K_I.push_back(hash[i]);
                        }

                    // set parameters for next iteration
                    GST_SFi -= 30; // next SF time is the actual minus 30 seconds
                    K_II = K_I; // next key is the actual one
                    K_I.clear(); // empty the actual one for a new computation
                }
            // compare computed current key against received key
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            std::cout << "Galileo OSNMA: TESLA verification ("<< num_of_hashes_needed << " hashes) took " << elapsed.count() << " seconds.\n";

            if(K_II.size() != d_osnma_data.d_mack_message.key.size())
                {
                    std::cout << "Galileo OSNMA: Error during tesla key verification. " << std::endl;
                    return;
                }
            if (K_II == d_osnma_data.d_mack_message.key)
                {
                    std::cout << "Galileo OSNMA: tesla key verified successfully " << std::endl;
                    d_tesla_key_verified = true;
                    // TODO - propagate result
                    // TODO - save current tesla key as latest one? propose a map with <GST_Sf, TeslaKey>
                    // TODO - Tags Sequence Verification: check ADKD[i] follows MACLT sequence
                }
            else

                {
                    std::cerr << "Galileo OSNMA: Error during tesla key verification. " << std::endl;
                    if(!d_flag_debug)
                        return;
                }
        }

    // verify MACK tags - MACSEQ
    OSNMA_data applicable_OSNMA = d_old_OSNMA_buffer[d_old_OSNMA_buffer.size() - 2]; // former subframe
    d_GST_Sf = d_GST_SIS - 30; // time of the start of SF containing MACSEQ // TODO buffer with times? since out of debug not every 30 s a Sf is necessarily received..
    std::vector<uint8_t> applicable_key =  d_old_OSNMA_buffer.back().d_mack_message.key; // current tesla key ie transmitted in the next subframe
    std::vector<std::string> sq1{};
    std::vector<std::string> sq2{};
    std::vector<std::string> applicable_sequence;
    const auto it = OSNMA_TABLE_16.find(applicable_OSNMA.d_dsm_kroot_message.maclt);
    // TODO as per RG example appears that the seq. q shall also be validated ageints either next or former Sf (depending on GST)
    if (it != OSNMA_TABLE_16.cend())
        {
            sq1 = it->second.sequence1;
            sq2 = it->second.sequence2;
        }

    // Assign relevant sequence based on subframe time
    if (applicable_OSNMA.d_nav_data.TOW_sf0 % 60 < 30) // tried GST_Sf and it does not support the data present.
        {
            applicable_sequence = sq1;
        }
    else if (applicable_OSNMA.d_nav_data.TOW_sf0 % 60 >= 30)
        {
            applicable_sequence = sq2;
        }
    else
        {
            std::cout << "Galileo OSNMA: Mismatch in the GST verification. " << std::endl;
        }
    // compare ADKD of Mack tags with MACLT defined ADKDs
    if(applicable_OSNMA.d_mack_message.tag_and_info.size() != applicable_sequence.size()-1)
        {
          std::cout << "Galileo OSNMA: Number of retrieved tags does not match MACLT sequence size!" << std::endl;
          return;
        }
    std::vector<uint8_t> flxTags {};
    std::string tempADKD;
    for (uint8_t i = 0; i < applicable_OSNMA.d_mack_message.tag_and_info.size(); i++)
        {
            tempADKD = applicable_sequence[i+1];
            if(tempADKD == "FLX")
                {
                    flxTags.push_back(i); // C: just need to save the index in the sequence
                }
            else if(applicable_OSNMA.d_mack_message.tag_and_info[i].tag_info.ADKD != std::stoi(applicable_sequence[i+1]))
                {                    std::cout << "Galileo OSNMA: Unsuccessful verification of MACSEQ - received ADKD against MAC Look-up table. " << std::endl;
                    return; // C: suffices one incorrect to abort and not process the rest of the tags
                }
        }

    // MACSEQ verification

    // Fixed as well as  FLX Tags share first part - Eq. 22 ICD
    std::vector<uint8_t> m(5 + 2 * flxTags.size()); // each flx tag brings two bytes
    m[0] = static_cast<uint8_t>(applicable_OSNMA.d_nav_data.PRNa);  // PRN_A - SVID of the satellite transmiting the tag
    m[1] = static_cast<uint8_t>((d_GST_Sf & 0xFF000000) >> 24);
    m[2] = static_cast<uint8_t>((d_GST_Sf & 0x00FF0000) >> 16);
    m[3] = static_cast<uint8_t>((d_GST_Sf & 0x0000FF00) >> 8);
    m[4] = static_cast<uint8_t>(d_GST_Sf & 0x000000FF);

    // Case tags flexible - Eq. 21 ICD
    for (uint8_t i = 0; i < flxTags.size() ; i++)
        {
            m[2*i + 5] = applicable_OSNMA.d_mack_message.tag_and_info[flxTags[i]].tag_info.PRN_d;
            m[2*i + 6] = applicable_OSNMA.d_mack_message.tag_and_info[flxTags[i]].tag_info.ADKD << 4 |
                       applicable_OSNMA.d_mack_message.tag_and_info[flxTags[i]].tag_info.cop;
        }
//    m = {0x18, 0x4f, 0x93, 0x53, 0x04, 0x05, 0x0f, 0x1f, 0x0f};
//    applicable_key = {0x11, 0x26, 0x47, 0x3b, 0x0e, 0x05, 0x05, 0x35,
//        0xb0, 0xf2, 0xa7, 0x24, 0x00, 0x22, 0xba, 0x8f};
//    applicable_OSNMA.d_mack_message.header.macseq = 0xbb8;
    // compute mac
    std::vector<uint8_t> mac;
    if (applicable_OSNMA.d_dsm_kroot_message.mf == 0) // C: HMAC-SHA-256
        {
            mac = d_crypto->computeHMAC_SHA_256(applicable_key, m);
        }
    else if (applicable_OSNMA.d_dsm_kroot_message.mf == 1) // C: CMAC-AES
        {
            mac = d_crypto->computeCMAC_AES(applicable_key, m);
        }
    // Truncate the twelve MSBits and compare with received MACSEQ
    uint16_t mac_msb = 0;
    if (!mac.empty())
        {
            mac_msb = (mac[0] << 8) + mac[1];
        }
    uint16_t computed_macseq = (mac_msb & 0xFFF0) >> 4;
    // Verify tags if MACSEQ is authenticated
    if (computed_macseq == applicable_OSNMA.d_mack_message.header.macseq)
        { // TODO  this shall affect only flx tags verification - and currently all tags of a MACK are affected which is undesired
            std::cout << "OSNMA: MACSEQ authenticated for PRN_A "
                      << osnma_msg->PRN << " with WN="
                      << osnma_msg->WN_sf0 << ", TOW="
                      << osnma_msg->TOW_sf0 << ". Verifying tags. "
                      << std::endl;
            uint8_t lt_bits = 0;
            const auto it2 = OSNMA_TABLE_11.find(d_osnma_data.d_dsm_kroot_message.ts);
            if (it2 != OSNMA_TABLE_11.cend())
                {
                    lt_bits = it2->second;
                }
            if (lt_bits == 0)
                {
                    return; // C: TODO if Tag length is 0, what is the action? no verification possible of NavData for sure.
                }

            // Tag verification
            // tag[i-1]:
                // adkd = 4/0 : use TK[i], NavData[i-2] to validate Tag[i-1]
                // adkd = 12 : ignore it -> not possible to verify yet
            // tag[i-10]
                // adkd = 4/0 : use TK[i-9], NavData[i-11] to validate Tag[i-10] would already be done by tag[i-1]
                // adkd = 12 : use TK[i], NavData[i-11] to validate Tag[i-10] TODO - pending better logic for not repeating this while twice.
            int t = d_old_OSNMA_buffer.size() - 2;
            applicable_OSNMA = d_old_OSNMA_buffer[t]; // former subframe
            d_GST_Sf = d_receiver_time - 30; // time of the start of SF containing MACSEQ

            size_t i = 0;
            while (i < applicable_OSNMA.d_mack_message.tag_and_info.size() && // loop over all tags in MACK message
                std::find(d_tags_to_verify.begin(),d_tags_to_verify.end(), // ADKD[i] is within allowed ADKDs
                       applicable_OSNMA.d_mack_message.tag_and_info[i].tag_info.ADKD)
                       != d_tags_to_verify.end())
                {
                    // TODO - if a  subsequent tag was already part of the verification (inner loop), this while is going to ignore that and try to validate it anyway.

                    // Take tag_k and check its ADKD, COP, PRN_d, this will be the reference for the iteration and search of other Tags
                    uint8_t Nt = d_Lt_min / applicable_OSNMA.d_dsm_kroot_message.ts; // Tags needed to be verified
                    uint8_t applicable_ADKD = applicable_OSNMA.d_mack_message.tag_and_info[i].tag_info.ADKD;
                    uint8_t applicable_COP = applicable_OSNMA.d_mack_message.tag_and_info[i].tag_info.cop; // * d_delta_COP;
                    uint8_t counter_COP = 1;
                    uint8_t applicable_PRNd = applicable_OSNMA.d_mack_message.tag_and_info[i].tag_info.PRN_d;
                    // ADKD=12 or ADKD = 4/0 => pick d_old_OSNMA_buffer.back() or [size-1]
                    applicable_key = d_old_OSNMA_buffer[t+1].d_mack_message.key; // current subframe
                    NavData applicable_NavData{};
                    if((applicable_ADKD == 0 || applicable_ADKD == 4) && d_old_OSNMA_buffer.size() > 3)
                        {
                            applicable_NavData = d_old_OSNMA_buffer[t-1].d_nav_data;
                        }
                    else if(applicable_ADKD == 12 && d_old_OSNMA_buffer.size() > 11)
                        {
                            applicable_NavData = d_old_OSNMA_buffer[t - 11].d_nav_data;
                        }
                    else
                        {
                            std::cout << "Galileo OSNMA: MACK message buffer elements not enough. Cannot verify tags. " << std::endl;
                        }


                    int k = i + 1;
                    uint8_t nt = 0;
                    bool flag_cancel_tag_verification = false; // if a tag fails, cancel whole NavData verification set
                    // Look for tags relative to reference NavData until Nt achieved,
                    // this may require going back in time, as long as COP is valid
                    while (nt <= Nt && counter_COP <= applicable_COP && !flag_cancel_tag_verification)
                    {
                        auto start_it = std::next(applicable_OSNMA.d_mack_message.tag_and_info.begin(), k);

                        // check the vector of tags of aplicable OSNMA for a match against the chosen
                        for (auto it = start_it; it != applicable_OSNMA.d_mack_message.tag_and_info.end() && nt <= Nt; ++it)
                        {
                                // Check if ADKD, COP, and PRN_d match
                                if(it->tag_info.ADKD == applicable_ADKD
                                    // && it->tag_info.cop == applicable_COP // TODO - I think this may be skipped as the relevant is the COP distance.
                                    && it->tag_info.PRN_d == applicable_PRNd)
                                {
                                    if(verify_tag(it.operator*(), applicable_OSNMA, k,applicable_key,applicable_NavData))
                                        {
                                            nt++;
                                        }
                                    else
                                        {
                                           // failure, discard this k-th tag
                                           flag_cancel_tag_verification = true;
                                           std::cout << "Galileo OSNMA: tag verification failed for PRN_a "
                                                     << applicable_OSNMA.d_nav_data.PRNa << " with WN="
                                                     << applicable_OSNMA.d_nav_data.WN_sf0 << ", TOW="
                                                     << applicable_OSNMA.d_nav_data.TOW_sf0 << ". "
                                                     << std::endl;
                                        }

                                }
                                if(flag_cancel_tag_verification)
                                    break;
                        }
                        // Check if Nt is achieved, if not, switch to older frame
                        if(nt < Nt && t > 0 /*not end of buffer*/ && counter_COP <= applicable_COP && !flag_cancel_tag_verification)
                            {
                                t--;
                                applicable_OSNMA = d_old_OSNMA_buffer[t];
                                applicable_key = d_old_OSNMA_buffer[t+1].d_mack_message.key;
                                applicable_NavData = d_old_OSNMA_buffer[t-1].d_nav_data;
                                d_GST_Sf -= 30;
                                counter_COP++;
                                k = 0;
                            }
                    }

                    if (nt >= Nt)
                        {
                            nt = 0;
                            std::cout << "Galileo OSNMA: tag verification accumulation succesful for PRN_a "
                                      << applicable_OSNMA.d_nav_data.PRNa << " with WN="
                                      << applicable_OSNMA.d_nav_data.WN_sf0 << ", TOW="
                                      << applicable_OSNMA.d_nav_data.TOW_sf0 << ". "
                                      << std::endl;
                        }

                }


        }
}

bool osnma_msg_receiver::verify_dsm_pkr(DSM_PKR_message message)
{
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
        }
    else
        {
            std::cout << "Galileo OSNMA: DSM-PKR verification unsuccessful !" << std::endl;
            return false;
        }
}
bool osnma_msg_receiver::verify_tag(MACK_tag_and_info tag_and_info,
    OSNMA_data applicable_OSNMA, uint8_t tag_position,
    const std::vector<uint8_t>& applicable_key,
    NavData applicable_NavData)
{
    bool verified = false;
    auto CTR = tag_position + 2; // CTR, first tag is CTR(tag0)=1 + 1 == 2
    // NAvData, tag_and_info[i]

    // check if enough osnma messages stored in the buffer.
    if (tag_and_info.tag_info.ADKD == 0
        || tag_and_info.tag_info.ADKD == 4)
        {
            if (d_old_OSNMA_buffer.size() < 3)
                {
                    std::cout << "Galileo OSNMA: MACK message buffer empty. Cannot verify tags. " << std::endl;
                    return verified;
                }
        }
    else if (tag_and_info.tag_info.ADKD == 12)
        {

            if (d_old_OSNMA_buffer.size() < 10+15)
                {
                    std::cout << "Galileo OSNMA: Tesla key not yet available. Cannot verify slow mac. at " <<
                        d_receiver_time << "s. "<< std::endl;
                    return verified;
                }
        }
    else
        {
            std::cout << "Galileo OSNMA: Unknown ADKD. " << std::endl;
            return verified;
        }

    // compute m
    std::vector<uint8_t> m;
    m.push_back(tag_and_info.tag_info.PRN_d);
    for(int i = 24; i >= 0; i -= 8)
        {
            m.push_back((applicable_NavData.PRNa >> i) & 0xFF);
        }
    m.push_back(static_cast<uint8_t>((d_GST_Sf & 0xFF000000) >> 24));
    m.push_back(static_cast<uint8_t>((d_GST_Sf & 0x00FF0000) >> 16));
    m.push_back(static_cast<uint8_t>((d_GST_Sf & 0x0000FF00) >> 8));
    m.push_back(static_cast<uint8_t>(d_GST_Sf & 0x000000FF));
    m.push_back(CTR);
    m.push_back(applicable_OSNMA.d_nma_header.nmas);
    if(tag_and_info.tag_info.ADKD == 0)
        {
            m.insert(m.end(),
                applicable_NavData.ephemeris_iono_vector.begin(),
                applicable_NavData.ephemeris_iono_vector.end()) ;
        }
    else if(tag_and_info.tag_info.ADKD == 4)
        {
            m.insert(m.end(),applicable_NavData.utc_vector.begin(),applicable_NavData.utc_vector.end()) ;
        }
    else
        {
            std::cout << "Galileo OSNMA: Unknown ADKD. " << std::endl;
        }

    // check that m has an integer number of bytes, if not, add padding zeroes
    // padding zeroes until size of vector is an integer number of bytes.
    // I think not needed, if bytes of m correctly formatted (i.e. added in big-endianness) -> the unused bits will be zero
    // and the vector has an integer number of uint8_t elements.

    // compute mac
    std::vector<uint8_t> mac;
    if (applicable_OSNMA.d_dsm_kroot_message.mf == 0) // C: HMAC-SHA-256
        {
            mac = d_crypto->computeHMAC_SHA_256(applicable_key, m);
        }
    else if (applicable_OSNMA.d_dsm_kroot_message.mf == 1) // C: CMAC-AES
        {
            mac = d_crypto->computeCMAC_AES(applicable_key, m);
        }

    // truncate the computed mac: trunc(l_t, mac(K,m)) Eq. 23 ICD
    uint8_t lt_bits = 0; // TODO - remove this duplication of code.
    const auto it2 = OSNMA_TABLE_11.find(applicable_OSNMA.d_dsm_kroot_message.ts);
    if (it2 != OSNMA_TABLE_11.cend())
        {
            lt_bits = it2->second;
        }
    if (lt_bits == 0)
        {
            return verified;
        }
    uint64_t computed_mac = static_cast<uint64_t>(mac[0]) << (lt_bits - 8);
    computed_mac += (static_cast<uint64_t>(mac[1]) << (lt_bits - 16));
    if (lt_bits == 20)
        {
            computed_mac += (static_cast<uint64_t>(mac[1] & 0xF0) >> 4);
        }
    else if (lt_bits == 24)
        {
            computed_mac += static_cast<uint64_t>(mac[2]);
        }
    else if (lt_bits == 28)
        {
            computed_mac += (static_cast<uint64_t>(mac[2]) << 4);
            computed_mac += (static_cast<uint64_t>(mac[3] & 0xF0) >> 4);
        }
    else if (lt_bits == 32)
        {
            computed_mac += (static_cast<uint64_t>(mac[2]) << 8);
            computed_mac += static_cast<uint64_t>(mac[3]);
        }
    else if (lt_bits == 40)
        {
            computed_mac += (static_cast<uint64_t>(mac[2]) << 16);
            computed_mac += (static_cast<uint64_t>(mac[3]) << 8);
            computed_mac += static_cast<uint64_t>(mac[4]);
        }

    // Compare computed tag with received one truncated
    if (tag_and_info.tag == computed_mac)
        {
            verified = true;
            if(tag_and_info.tag_info.ADKD == 0 || tag_and_info.tag_info.ADKD == 12)
                {
                    std::cout << "Galileo OSNMA: tag verification successful for PRN_a "
                              << applicable_NavData.PRNa << " with WN="
                              << applicable_NavData.WN_sf0 << ", TOW="
                              << applicable_NavData.TOW_sf0 << "NavData= "
                              << "Ephemeris, Clock and Ionospheric data" << ". "
                              << std::endl;
                }
            else if(tag_and_info.tag_info.ADKD == 4)
                {
                    std::cout << "Galileo OSNMA: tag verification successful for PRN_a "
                              << applicable_NavData.PRNa << " with WN="
                              << applicable_NavData.WN_sf0 << ", TOW="
                              << applicable_NavData.TOW_sf0 << "NavData= "
                              << "Timing data" << ". "
                              << std::endl;
                }

        }
    else
        {
            std::cout << "Galileo OSNMA: Tag verification failed for PRN_a "
                      << applicable_NavData.PRNa << " with WN="
                      << applicable_NavData.WN_sf0 << ", TOW="
                      << applicable_NavData.TOW_sf0 << ". "
                      << std::endl;
        }
    return verified;
}
/**
 * @brief Checks if the current subframe time is bigger than last one received.
 *
 * \details It compares the current GST value with the previous one and updates the old value with the new one.
 *
 * @param sharedPtr A shared pointer to an instance of OSNMA_msg.
 * @return True if the current subframe is the next subframe, False otherwise.
 */
bool osnma_msg_receiver::is_next_subframe()
{
    bool is_bigger = d_GST_SIS > d_old_GST_SIS;
    if(d_GST_SIS != d_old_GST_SIS + 30 && d_old_GST_SIS != 0){
            std::cout << "Galileo OSNMA:: Mack processing - skip " << std::endl;
        }

    d_old_GST_SIS = d_GST_SIS;

    return is_bigger;
}
