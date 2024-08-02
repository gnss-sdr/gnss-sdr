/*!
 * \file osnma_msg_receiver.cc
 * \brief GNU Radio block that processes Galileo OSNMA data received from
 * Galileo E1B telemetry blocks. After successful decoding, sends the content to
 * the PVT block.
 * \author Carles Fernandez-Prades, 2023-2024. cfernandez(at)cttc.es
 * Cesare Ghionoiu Martinez, 2023-2024. c.ghionoiu-martinez@tu-braunschweig.de
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2024  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "osnma_msg_receiver.h"
#include "Galileo_OSNMA.h"
#include "gnss_crypto.h"
#include "gnss_satellite.h"
#include "osnma_dsm_reader.h"  // for OSNMA_DSM_Reader
#include "osnma_helper.h"
#include "osnma_nav_data_manager.h"
#include <gnuradio/io_signature.h>  // for gr::io_signature::make
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>  // for std::ifstream and std::ofstream
#include <iomanip>  // for std::setfill
#include <ios>      // for std::hex, std::uppercase
#include <iostream>
#include <numeric>  // for std::accumulate
#include <sstream>  // std::stringstream
#include <tuple>
#include <typeinfo>  // for typeid
#include <utility>


#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>  // for DLOG
#else
#include <absl/log/log.h>
#endif

#if HAS_GENERIC_LAMBDA
#else
#include <boost/bind/bind.hpp>
#endif

#if PMT_USES_BOOST_ANY
#include <boost/any.hpp>
#include <iomanip>
namespace wht = boost;
#else
#include <any>
namespace wht = std;
#endif


osnma_msg_receiver_sptr osnma_msg_receiver_make(const std::string& pemFilePath, const std::string& merkleFilePath)
{
    return osnma_msg_receiver_sptr(new osnma_msg_receiver(pemFilePath, merkleFilePath));
}


osnma_msg_receiver::osnma_msg_receiver(const std::string& crtFilePath, const std::string& merkleFilePath) : gr::block("osnma_msg_receiver",
                                                                                                                gr::io_signature::make(0, 0, 0),
                                                                                                                gr::io_signature::make(0, 0, 0))
{
    d_dsm_reader = std::make_unique<OSNMA_DSM_Reader>();
    d_crypto = std::make_unique<Gnss_Crypto>(crtFilePath, merkleFilePath);
    d_helper = std::make_unique<Osnma_Helper>();
    d_nav_data_manager = std::make_unique<OSNMA_nav_data_Manager>();

    if (d_crypto->have_public_key())
        {  // Hot start is enabled
            LOG(WARNING) << "OSNMA Public Key available, trying to find DSM-KROOT saved";
            std::cout << "OSNMA Public Key available, trying to find DSM-KROOT saved" << std::endl;
            d_public_key_verified = true;

            auto dsm_nmah = parse_dsm_kroot();
            if (!dsm_nmah.first.empty())
                {
                    LOG(WARNING) << "OSNMA DSM-KROOT and NMA Header successfully read from file " << KROOTFILE_DEFAULT;
                    std::cout << "OSNMA DSM-KROOT and NMA Header successfully read from file " << KROOTFILE_DEFAULT << std::endl;
                    d_flag_hot_start = true;
                    process_dsm_message(dsm_nmah.first, dsm_nmah.second);
                    LOG(WARNING) << "OSNMA DSM-KROOT available :: HOT START";
                    std::cout << "OSNMA DSM-KROOT available :: HOT START" << std::endl;
                }
            else
                {
                    LOG(WARNING) << "OSNMA DSM-KROOT not available :: WARM START";
                    std::cout << "OSNMA DSM-KROOT not available :: WARM START" << std::endl;
                }
        }

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
    std::chrono::time_point<std::chrono::system_clock> now;
    if (d_flag_debug)
        {
            // d_GST_Rx = d_helper->compute_gst(d_initial_debug_time);
            LOG(WARNING) << "Galileo OSNMA: Debug mode, time artificially set up.";
            std::cout << "Galileo OSNMA: Debug mode, time artificially set up." << std::endl;
            // TODO - need to synchronize time lapse with Gnss_Synchro?
        }
    else
        {
            d_GST_Rx = d_helper->compute_gst_now();
        }

    d_WN = d_helper->get_WN(d_GST_Rx);
    d_TOW = d_helper->get_TOW(d_GST_Rx);
    LOG(WARNING) << "Galileo OSNMA: initial receiver time GST=[" << d_WN << " " << d_TOW << "]";
    std::cout << "Galileo OSNMA: initial receiver time GST=[" << d_WN << " " << d_TOW << "]" << std::endl;
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
                    const auto sat = Gnss_Satellite(std::string("Galileo"), nma_msg->PRN);  // TODO remove if unneeded

                    std::ostringstream output_message;
                    output_message << "Galileo OSNMA: data received starting at "
                                   << "WN="
                                   << nma_msg->WN_sf0
                                   << ", TOW="
                                   << nma_msg->TOW_sf0
                                   << ", from satellite "
                                   << sat;
                    LOG(INFO) << output_message.str();
                    std::cout << output_message.str() << std::endl;

                    // Receiver time update
                    d_GST_SIS = d_helper->compute_gst(nma_msg->WN_sf0, nma_msg->TOW_sf0);
                    if (d_last_verified_key_GST == 0)
                        {
                            d_last_received_GST = d_GST_SIS;
                        }
                    else if (d_GST_SIS > d_last_received_GST)
                        {
                            d_last_received_GST = d_GST_SIS;
                        }
                    if (d_flag_debug)
                        {
                            d_GST_Rx = d_last_received_GST;
                        }
                    else
                        {
                            d_GST_Rx = d_helper->compute_gst_now();
                        }
                    LOG(INFO) << "Galileo OSNMA: Receiver Time GST=[" << d_helper->get_WN(d_GST_Rx) << " " << d_helper->get_TOW(d_GST_Rx) << "]";
                    std::cout << "Galileo OSNMA: Receiver Time GST=[" << d_helper->get_WN(d_GST_Rx) << " " << d_helper->get_TOW(d_GST_Rx) << "]" << std::endl;

                    // time constraint verification
                    std::time_t delta_T = std::abs(static_cast<int64_t>(d_GST_Rx - d_GST_SIS));
                    if (delta_T <= d_T_L)
                        {
                            d_tags_to_verify = {0, 4, 12};
                            LOG(INFO) << "Galileo OSNMA: time constraint OK ( delta_T=" << delta_T << " s)";
                            std::cout << "Galileo OSNMA: time constraint OK ( delta_T=" << delta_T << " s)" << std::endl;
                        }
                    else if (delta_T > d_T_L && delta_T <= 10 * d_T_L)
                        {
                            d_tags_to_verify = {12};
                            LOG(WARNING) << "Galileo OSNMA: time constraint allows only slow MACs to be verified";
                            std::cout << "Galileo OSNMA: |local_t - GST_SIS| < T_L  [ |" << static_cast<int>(d_GST_Rx - d_GST_SIS) << " | < " << static_cast<int>(d_T_L) << " ]" << std::endl;
                            LOG(WARNING) << "Galileo OSNMA: d_receiver_time: " << d_GST_Rx << " d_GST_SIS: " << d_GST_SIS;
                            LOG(WARNING) << "Galileo OSNMA: |local_t - GST_SIS| < T_L [ |" << static_cast<int>(d_GST_Rx - d_GST_SIS) << " | < " << static_cast<int>(d_T_L) << " ]";
                        }
                    else
                        {
                            d_tags_to_verify = {};
                            LOG(WARNING) << "Galileo OSNMA: time constraint violation";
                            std::cerr << "Galileo OSNMA: time constraint violation" << std::endl;
                            std::cerr << "Galileo OSNMA: | local_t - GST_SIS | < T_L  [ | " << static_cast<int>(d_GST_Rx - d_GST_SIS) << " | < " << static_cast<int>(d_T_L) << " ]" << std::endl;
                            LOG(WARNING) << "Galileo OSNMA: d_receiver_time: " << d_GST_Rx << " d_GST_SIS: " << d_GST_SIS;
                            LOG(WARNING) << "Galileo OSNMA: | local_t - GST_SIS | < T_L  [ | " << static_cast<int>(d_GST_Rx - d_GST_SIS) << " | < " << static_cast<int>(d_T_L) << " ]";
                            return;
                        }

                    process_osnma_message(nma_msg);
                }                                                                                                             // OSNMA frame received
            else if (msg_type_hash_code == typeid(std::shared_ptr<std::tuple<uint32_t, std::string, uint32_t>>).hash_code())  // Navigation data bits for OSNMA received
                {
                    const auto inav_data = wht::any_cast<std::shared_ptr<std::tuple<uint32_t, std::string, uint32_t>>>(pmt::any_ref(msg));
                    uint32_t PRNd = std::get<0>(*inav_data);
                    std::string nav_data = std::get<1>(*inav_data);
                    uint32_t TOW = std::get<2>(*inav_data);
                    d_nav_data_manager->add_navigation_data(nav_data, PRNd, TOW);
                }
            else
                {
                    LOG(WARNING) << "Galileo OSNMA: osnma_msg_receiver received an unknown object type!";
                }
        }
    catch (const wht::bad_any_cast& e)
        {
            LOG(WARNING) << "Galileo OSNMA: osnma_msg_receiver Bad any_cast: " << e.what();
        }

    //  Send the resulting decoded NMA data (if available) to PVT
    if (d_new_data)
        {
            auto osnma_data_ptr = std::make_shared<OSNMA_data>(d_osnma_data);
            this->message_port_pub(pmt::mp("OSNMA_to_PVT"), pmt::make_any(osnma_data_ptr));
            d_new_data = false;
            // d_osnma_data = OSNMA_data();
            DLOG(INFO) << "Galileo OSNMA: NMA info sent to the PVT block through the OSNMA_to_PVT async message port";
        }
}


void osnma_msg_receiver::process_osnma_message(const std::shared_ptr<OSNMA_msg>& osnma_msg)
{
    if (d_flag_alert_message && (d_public_key_verified || d_kroot_verified))
        {
            return;
        }
    read_nma_header(osnma_msg->hkroot[0]);

    // Check for corner cases: renewal, revocation, alert message
    if (d_osnma_data.d_nma_header.nmas == 0 /* RES */)
        {
            LOG(WARNING) << "Galileo OSNMA: NMAS invalid (RES), skipping osnma message";
            return;
        }
    // TODO - trusting the NMAS and CPKS shall be done upon PKR verification or Tag verification.
    //  It's ok to activate the flags, but the final decision should happen after verifying it.
    // For OAM is solved, but NPK and PKREV I think not yet
    if (d_osnma_data.d_nma_header.nmas == 2 /* OP */ && d_osnma_data.d_nma_header.cpks == 4 /* NPK */ && d_GST_PKR_PKREV_start == 0)
        {
            d_flag_PK_renewal = true;
            d_GST_PKR_PKREV_start = d_helper->compute_gst(osnma_msg->WN_sf0, osnma_msg->TOW_sf0);
            LOG(INFO) << "Galileo OSNMA: Public Key Renewal :: Start at GST=[" << osnma_msg->WN_sf0 << " " << osnma_msg->TOW_sf0 << "]";
            std::cout << "Galileo OSNMA: Public Key Renewal :: Start at GST=[" << osnma_msg->WN_sf0 << " " << osnma_msg->TOW_sf0 << "]" << std::endl;
        }
    if (d_flag_PK_renewal && d_osnma_data.d_nma_header.nmas == 2 /* OP */ && d_osnma_data.d_nma_header.cpks == 1 /* Nominal */)
        {
            d_flag_PK_renewal = false;
            uint32_t final_GST = d_helper->compute_gst(osnma_msg->WN_sf0, osnma_msg->TOW_sf0);
            double duration_hours = (final_GST - d_GST_PKR_PKREV_start) / 3600;
            LOG(INFO) << "Galileo OSNMA: Public Key Renewal :: Finished at GST=" << duration_hours << ", Duration=" << duration_hours << " h";
            std::cout << "Galileo OSNMA: Public Key Renewal :: Finished at GST=" << duration_hours << ", Duration=" << duration_hours << " h" << std::endl;
        }

    if (d_osnma_data.d_nma_header.nmas == 3 /* DU */ && d_osnma_data.d_nma_header.cpks == 5 /* PKREV */ && d_GST_PKR_PKREV_start == 0)
        {
            d_flag_PK_revocation = true;
            d_number_of_blocks[d_osnma_data.d_dsm_header.dsm_id] = 0;
            d_public_key_verified = false;
            d_kroot_verified = false;
            d_tesla_key_verified = false;
            d_GST_PKR_PKREV_start = d_helper->compute_gst(osnma_msg->WN_sf0, osnma_msg->TOW_sf0);
            LOG(INFO) << "Galileo OSNMA: Public Key Revocation :: Start at GST=[" << osnma_msg->WN_sf0 << " " << osnma_msg->TOW_sf0 << "]";
            std::cout << "Galileo OSNMA: Public Key Revocation :: Start at GST=[" << osnma_msg->WN_sf0 << " " << osnma_msg->TOW_sf0 << "]" << std::endl;
        }
    if (d_flag_PK_revocation && d_osnma_data.d_nma_header.nmas == 2 /* OP */ && d_osnma_data.d_nma_header.cpks == 1 /* Nominal */)
        {
            // step 2 , start using new chain
            d_flag_PK_revocation = false;
            uint32_t final_GST = d_helper->compute_gst(osnma_msg->WN_sf0, osnma_msg->TOW_sf0);
            double duration_hours = (final_GST - d_GST_PKR_PKREV_start) / 3600;
            LOG(INFO) << "Galileo OSNMA: Public Key Revocation :: Finished at GST=[" << osnma_msg->WN_sf0 << " " << osnma_msg->TOW_sf0 << "]"
                      << ", Duration=" << duration_hours << "h";
            std::cout << "Galileo OSNMA: Public Key Revocation :: Finished at GST=[" << osnma_msg->WN_sf0 << " " << osnma_msg->TOW_sf0 << "]"
                      << ", Duration=" << duration_hours << "h" << std::endl;
        }

    if (d_osnma_data.d_nma_header.nmas == 3 /* DU */ && d_osnma_data.d_nma_header.cpks == 7 /* AM */ && d_GST_PKR_AM_start == 0)
        {
            d_flag_alert_message = true;
            d_GST_PKR_AM_start = d_helper->compute_gst(osnma_msg->WN_sf0, osnma_msg->TOW_sf0);
            d_public_key_verified = false;
            d_kroot_verified = false;
            LOG(INFO) << "Galileo OSNMA: Alert message :: Start at GST=[" << osnma_msg->WN_sf0 << " " << osnma_msg->TOW_sf0 << "]";
            std::cout << "Galileo OSNMA: Alert message :: Start at GST=[" << osnma_msg->WN_sf0 << " " << osnma_msg->TOW_sf0 << "]" << std::endl;
        }

    read_dsm_header(osnma_msg->hkroot[1]);
    read_dsm_block(osnma_msg);
    process_dsm_block(osnma_msg);  // will process dsm block if received a complete one, then will call mack processing upon re-setting the dsm block to 0
    if (d_osnma_data.d_dsm_kroot_message.towh_k != 0)
        {
            d_GST_0 = d_helper->compute_gst(d_osnma_data.d_dsm_kroot_message.wn_k, d_osnma_data.d_dsm_kroot_message.towh_k * 3600);
            d_GST_Sf = d_GST_0 + 30 * std::floor((d_GST_SIS - d_GST_0) / 30);  // Eq. 3 R.G.
        }
    read_and_process_mack_block(osnma_msg);  // only process them if at least 3 available.
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
    LOG(INFO) << "Galileo OSNMA: Received block DSM_BID=" << static_cast<uint32_t>(d_osnma_data.d_dsm_header.dsm_block_id)
              << " with DSM_ID " << static_cast<uint32_t>(d_osnma_data.d_dsm_header.dsm_id);
}

/*
 * accumulates dsm messages
 * */
void osnma_msg_receiver::read_dsm_block(const std::shared_ptr<OSNMA_msg>& osnma_msg)
{
    // Fill d_dsm_message. dsm_block_id provides the offset within the dsm message.
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
            LOG(INFO) << "Galileo OSNMA: number of blocks in this message: " << static_cast<uint32_t>(number_of_blocks);
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
    std::stringstream available_blocks;
    available_blocks << "Galileo OSNMA: Available blocks for DSM_ID " << static_cast<uint32_t>(d_osnma_data.d_dsm_header.dsm_id) << ": [ ";
    if (d_number_of_blocks[d_osnma_data.d_dsm_header.dsm_id] == 0)  // block 0 not received yet
        {
            for (auto id_received : d_dsm_id_received[d_osnma_data.d_dsm_header.dsm_id])
                {
                    if (id_received == 0)
                        {
                            available_blocks << "- ";
                        }
                    else
                        {
                            available_blocks << "X ";
                        }
                }
        }
    else
        {
            for (uint16_t k = 0; k < d_number_of_blocks[d_osnma_data.d_dsm_header.dsm_id]; k++)
                {
                    if (d_dsm_id_received[d_osnma_data.d_dsm_header.dsm_id][k] == 0)
                        {
                            available_blocks << "- ";
                        }
                    else
                        {
                            available_blocks << "X ";
                        }
                }
        }
    available_blocks << "]";
    LOG(INFO) << available_blocks.str();
    std::cout << available_blocks.str() << std::endl;
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
            size_t len = std::size_t(d_number_of_blocks[d_osnma_data.d_dsm_header.dsm_id]) * SIZE_DSM_BLOCKS_BYTES;
            std::vector<uint8_t> dsm_msg(len, 0);
            for (uint32_t i = 0; i < d_number_of_blocks[d_osnma_data.d_dsm_header.dsm_id]; i++)
                {
                    for (size_t j = 0; j < SIZE_DSM_BLOCKS_BYTES; j++)
                        {
                            dsm_msg[i * SIZE_DSM_BLOCKS_BYTES + j] = d_dsm_message[d_osnma_data.d_dsm_header.dsm_id][i * SIZE_DSM_BLOCKS_BYTES + j];
                        }
                }
            d_dsm_message[d_osnma_data.d_dsm_header.dsm_id] = std::array<uint8_t, 256>{};
            d_dsm_id_received[d_osnma_data.d_dsm_header.dsm_id] = std::array<uint8_t, 16>{};
            LOG(INFO) << "Galileo OSNMA: DSM message completed :: start processing, GST=[" << osnma_msg->WN_sf0 << " " << osnma_msg->TOW_sf0 << "]";
            process_dsm_message(dsm_msg, osnma_msg->hkroot[0]);
        }
}


/*
 * case DSM-Kroot:
 * - computes the padding and compares with received message
 * - if successful, tries to verify the digital signature
 * case DSM-PKR:
 * - calls verify_dsm_pkr to verify the public key
 * */
void osnma_msg_receiver::process_dsm_message(const std::vector<uint8_t>& dsm_msg, const uint8_t& nma_header)
{
    // DSM-KROOT message
    if ((d_osnma_data.d_dsm_header.dsm_id < 12 || d_flag_hot_start) && d_public_key_verified)
        {
            // Parse Kroot message
            LOG(INFO) << "Galileo OSNMA: DSM-KROOT message received.";
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
            // Kroot field
            const uint16_t l_lk_bytes = d_dsm_reader->get_lk_bits(d_osnma_data.d_dsm_kroot_message.ks) / 8;
            d_osnma_data.d_dsm_kroot_message.kroot = d_dsm_reader->get_kroot(dsm_msg, l_lk_bytes);
            // DS field
            uint16_t l_ds_bits = 0;
            const auto it = OSNMA_TABLE_15.find(d_crypto->get_public_key_type());
            if (it != OSNMA_TABLE_15.cend())
                {
                    l_ds_bits = it->second;
                }
            const uint16_t l_ds_bytes = l_ds_bits / 8;
            d_osnma_data.d_dsm_kroot_message.ds = std::vector<uint8_t>(l_ds_bytes, 0);  // C: this accounts for padding in case needed.
            for (uint16_t k = 0; k < l_ds_bytes; k++)
                {
                    d_osnma_data.d_dsm_kroot_message.ds[k] = dsm_msg[13 + l_lk_bytes + k];
                }
            // Padding
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
                    LOG(WARNING) << "Galileo OSNMA: Failed length reading of DSM-KROOT message";
                    d_count_failed_Kroot++;
                }
            else
                {
                    // validation of padding
                    const uint16_t size_m = 13 + l_lk_bytes;
                    std::vector<uint8_t> MSG;
                    MSG.reserve(size_m + l_ds_bytes + 1);
                    MSG.push_back(nma_header);  // NMA header
                    for (uint16_t i = 1; i < size_m; i++)
                        {
                            MSG.push_back(dsm_msg[i]);
                        }
                    std::vector<uint8_t> message = MSG;  // MSG = (M | DS) from ICD. Eq. 7
                    for (uint16_t k = 0; k < l_ds_bytes; k++)
                        {
                            MSG.push_back(d_osnma_data.d_dsm_kroot_message.ds[k]);
                        }

                    std::vector<uint8_t> hash;
                    if (d_osnma_data.d_dsm_kroot_message.hf == 0)  // Table 8.
                        {
                            hash = d_crypto->compute_SHA_256(MSG);
                        }
                    else if (d_osnma_data.d_dsm_kroot_message.hf == 2)
                        {
                            hash = d_crypto->compute_SHA3_256(MSG);
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
                            LOG(INFO) << "Galileo OSNMA: DSM-KROOT message received ok.";
                            LOG(INFO) << "Galileo OSNMA: DSM-KROOT with CID=" << static_cast<uint32_t>(d_osnma_data.d_nma_header.cid)
                                      << ", PKID=" << static_cast<uint32_t>(d_osnma_data.d_dsm_kroot_message.pkid)
                                      << ", WN=" << static_cast<uint32_t>(d_osnma_data.d_dsm_kroot_message.wn_k)
                                      << ", TOW=" << static_cast<uint32_t>(d_osnma_data.d_dsm_kroot_message.towh_k) * 3600;
                            // local_time_verification(osnma_msg);  // FIXME TODO: real time verification needed

                            // If new PK verified and the new KROOT arrived, set the new PK before attempting verification
                            if (d_flag_PK_renewal && d_osnma_data.d_dsm_kroot_message.pkid == d_new_public_key_id && d_flag_NPK_set == false)
                                {
                                    d_crypto->set_public_key(d_new_public_key);
                                    d_crypto->store_public_key(PEMFILE_DEFAULT);
                                    d_flag_NPK_set = true;
                                }
                            if (l_ds_bits == 512)
                                {
                                    d_kroot_verified = d_crypto->verify_signature_ecdsa_p256(message, d_osnma_data.d_dsm_kroot_message.ds);
                                }
                            else if (l_ds_bits == 1056)
                                {
                                    d_kroot_verified = d_crypto->verify_signature_ecdsa_p521(message, d_osnma_data.d_dsm_kroot_message.ds);
                                }
                            if (d_kroot_verified)
                                {
                                    std::cout << "Galileo OSNMA: DSM-KROOT authentication successful!" << std::endl;
                                    LOG(INFO) << "Galileo OSNMA: DSM-KROOT authentication successful!";
                                    if (d_flag_alert_message)
                                        {
                                            LOG(WARNING) << "Galileo OSNMA: DSM-KROOT :: Alert message verification :: SUCCESS. ";
                                        }
                                    else
                                        {
                                            LOG(INFO) << "Galileo OSNMA: NMA Status is " << d_dsm_reader->get_nmas_status(d_osnma_data.d_nma_header.nmas) << ", "
                                                      << "Chain in force is " << static_cast<uint32_t>(d_osnma_data.d_nma_header.cid) << ", "
                                                      << "Chain and Public Key Status is " << d_dsm_reader->get_cpks_status(d_osnma_data.d_nma_header.cpks);
                                        }
                                    // Save DSM-Kroot and NMA header into a permanent storage
                                    if (d_flag_hot_start)
                                        {
                                            d_flag_hot_start = false;
                                            return;
                                        }
                                    store_dsm_kroot(dsm_msg, nma_header);  // TODO - store it only if DSM-KROOT is new
                                }
                            else
                                {
                                    LOG(WARNING) << "Galileo OSNMA: DSM-KROOT authentication failed.";
                                    std::cerr << "Galileo OSNMA: DSM-KROOT authentication failed." << std::endl;
                                    if (d_flag_alert_message)
                                        {
                                            d_flag_alert_message = false;
                                        }
                                    d_count_failed_Kroot++;
                                }
                        }
                    else
                        {
                            LOG(WARNING) << "Galileo OSNMA: Error computing padding bits.";
                            // TODO - here will have to decide if perform the verification or not. Since this step is not mandatory, one could as well have skipped it.
                            d_count_failed_Kroot++;
                        }
                }
        }
    // DSM-PKR message
    else if (d_osnma_data.d_dsm_header.dsm_id >= 12 && d_osnma_data.d_dsm_header.dsm_id < 16)
        {
            LOG(INFO) << "Galileo OSNMA: DSM-PKR message received";
            // Save DSM-PKR message
            d_osnma_data.d_dsm_pkr_message.nb_dp = d_dsm_reader->get_number_blocks_index(dsm_msg[0]);
            d_osnma_data.d_dsm_pkr_message.mid = d_dsm_reader->get_mid(dsm_msg);
            for (int k = 0; k < 128; k++)
                {
                    d_osnma_data.d_dsm_pkr_message.itn[k] = dsm_msg[k + 1];
                }
            d_osnma_data.d_dsm_pkr_message.npkt = d_dsm_reader->get_npkt(dsm_msg);
            uint8_t npktid = d_dsm_reader->get_npktid(dsm_msg);
            if (d_flag_PK_renewal && npktid > d_osnma_data.d_dsm_pkr_message.npktid)
                {
                    d_new_public_key_id = npktid;
                }
            d_osnma_data.d_dsm_pkr_message.npktid = npktid;

            uint32_t l_npk_bytes = 0;
            std::string PKT;
            const auto it = OSNMA_TABLE_5.find(d_osnma_data.d_dsm_pkr_message.npkt);
            if (it != OSNMA_TABLE_5.cend())
                {
                    PKT = it->second;
                    const auto it2 = OSNMA_TABLE_6.find(it->second);
                    if (it2 != OSNMA_TABLE_6.cend())
                        {
                            l_npk_bytes = it2->second / 8;
                        }
                }
            uint32_t l_dp_bytes = dsm_msg.size();
            if (d_osnma_data.d_dsm_pkr_message.npkt == 4 && d_osnma_data.d_dsm_pkr_message.npktid == 0)
                {
                    LOG(WARNING) << "Galileo OSNMA: DSM-PKR :: Alert message received. Verifying it.";
                    std::cout << "Galileo OSNMA: DSM-PKR :: Alert message received. Verifying it." << std::endl;
                    l_npk_bytes = l_dp_bytes - 130;  // bytes
                }

            d_osnma_data.d_dsm_pkr_message.npk = std::vector<uint8_t>(l_npk_bytes, 0);  // ECDSA Public Key
            for (uint32_t k = 0; k < l_npk_bytes; k++)
                {
                    d_osnma_data.d_dsm_pkr_message.npk[k] = dsm_msg[k + 130];
                }

            uint32_t l_pd_bytes = l_dp_bytes - 130 - l_npk_bytes;
            uint32_t check_l_dp_bytes = 104 * std::ceil(static_cast<float>(1040.0 + l_npk_bytes * 8.0) / 104.0) / 8;
            if (l_dp_bytes != check_l_dp_bytes)
                {
                    LOG(WARNING) << "Galileo OSNMA: Failed length reading of DSM-PKR message";
                    d_flag_alert_message = false;
                }
            else
                {
                    d_osnma_data.d_dsm_pkr_message.p_dp = std::vector<uint8_t>(l_pd_bytes, 0);
                    for (uint32_t k = 0; k < l_pd_bytes; k++)
                        {
                            d_osnma_data.d_dsm_pkr_message.p_dp[k] = dsm_msg[l_dp_bytes - l_pd_bytes + k];
                        }
                    // TODO: kroot fields are 0 in case no DSM-KROOT received yet, need to take this into account.
                    // std::vector<uint8_t> mi;  //  (NPKT + NPKID + NPK)
                    LOG(INFO) << "Galileo OSNMA: DSM-PKR with CID=" << static_cast<uint32_t>(d_osnma_data.d_nma_header.cid)
                              << ", PKID=" << static_cast<uint32_t>(d_osnma_data.d_dsm_pkr_message.npktid) << " received";
                    // Public key verification against Merkle tree root.
                    bool verification = verify_dsm_pkr(d_osnma_data.d_dsm_pkr_message);
                    if (verification)
                        {
                            LOG(INFO) << "Galileo OSNMA: DSM-PKR verification :: SUCCESS";
                            d_public_key_verified = true;
                            if (d_flag_PK_renewal)
                                {
                                    d_new_public_key = d_osnma_data.d_dsm_pkr_message.npk;
                                }
                            else if (d_flag_alert_message)
                                {
                                    LOG(WARNING) << "Galileo OSNMA: DSM-PKR verification :: Alert message verification :: SUCCESS. OSNMA disabled. Contact Galileo Service Centre";
                                    std::cout << "Galileo OSNMA: DSM-PKR verification :: Alert message verification :: SUCCESS. OSNMA disabled. Contact Galileo Service Centre" << std::endl;
                                }
                            else
                                {
                                    d_crypto->set_public_key_type(PKT);
                                    d_crypto->set_public_key(d_osnma_data.d_dsm_pkr_message.npk);
                                    d_crypto->store_public_key(PEMFILE_DEFAULT);
                                }
                        }
                    else
                        {
                            LOG(ERROR) << "Galileo OSNMA: DSM-PKR verification :: FAILURE";
                            d_public_key_verified = false;
                            d_count_failed_pubKey++;
                            if (d_flag_alert_message)
                                {
                                    d_flag_alert_message = false;  // disregard message as its authenticity could not be verified.
                                }
                        }
                }
        }
    else
        {
            // Reserved message?
            LOG(WARNING) << "Galileo OSNMA: Reserved message received";
            std::cerr << "Galileo OSNMA: Reserved message received" << std::endl;
        }
    d_number_of_blocks[d_osnma_data.d_dsm_header.dsm_id] = 0;  // TODO - reset during header parsing in PKREV?
}


/**
 * @brief Reads the Mack message from the given OSNMA_msg object
 *
 * @details Conditions for MACK processing:
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

    d_osnma_data.d_nav_data.set_tow_sf0(osnma_msg->TOW_sf0);
    bool can_process_mack_block = (d_osnma_data.d_nma_header.nmas != 3 && d_kroot_verified) ||  // NMAS different than DU
                                  (d_osnma_data.d_nma_header.nmas == 3 && !d_kroot_verified);   // NMAS is DU, but must be disregarded
    bool can_verify_tesla_key = d_kroot_verified || d_tesla_key_verified;                       // Either of those suffices for verifying the incoming TESLA key
    bool can_parse_tag_fields = d_osnma_data.d_dsm_kroot_message.ts != 0;                       // calculating the number of tags is based on the TS of the DSM-KROOT.
    if (can_verify_tesla_key && can_parse_tag_fields && can_process_mack_block)
        {
            read_mack_header();
            d_osnma_data.d_mack_message.PRNa = osnma_msg->PRN;  // FIXME this is ugly.
            d_osnma_data.d_mack_message.TOW = osnma_msg->TOW_sf0;
            d_osnma_data.d_mack_message.WN = osnma_msg->WN_sf0;
            read_mack_body();
            process_mack_message();
            // TODO - shorten the MACK processing for the cases where no TK verified or no Kroot verified (warm and cold start)
            // still, for instance the OSNMA_NavData and Mack storage (within process_mack_message) makes sense.
        }
    else
        {
            // TODO - MACKs should be saved because once Kroot available, they could be verified.
            LOG(WARNING) << "Galileo OSNMA: Cannot process MACK block. Skipping it.";
            std::cout << "Galileo OSNMA: Cannot process MACK block. Skipping it." << std::endl;
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
            return;  // C: TODO if Tag length is 0, what is the action? no verification possible of NavData for sure.
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
                    tag += (static_cast<uint64_t>((d_mack_message[7 /* bytes of MACK header */ + k * 7 /* offset of k-th tag */])) << 32);
                    tag += (static_cast<uint64_t>((d_mack_message[8 + k * 7])) << 24);
                    tag += (static_cast<uint64_t>((d_mack_message[9 + k * 7])) << 16);
                    tag += (static_cast<uint64_t>((d_mack_message[10 + k * 7])) << 8);
                    tag += static_cast<uint64_t>(d_mack_message[11 + k * 7]);
                    PRN_d += d_mack_message[12 + k * 7];
                    ADKD += ((d_mack_message[13 + k * 7] & 0xF0) >> 4);
                    cop += (d_mack_message[13 + k * 7] & 0x0F);
                    if (k == (nt - 2))  // end of Tag&Info
                        {
                            d_osnma_data.d_mack_message.key = std::vector<uint8_t>(d_osnma_data.d_dsm_kroot_message.kroot.size());
                            for (size_t j = 0; j < d_osnma_data.d_dsm_kroot_message.kroot.size(); j++)
                                {
                                    d_osnma_data.d_mack_message.key[j] = d_mack_message[14 + k * 7 + j];
                                }
                        }
                }
            d_osnma_data.d_mack_message.tag_and_info[k].tag = tag;
            d_osnma_data.d_mack_message.tag_and_info[k].counter = k + 2;  // CTR==1 for Tag0, increases subsequently for all other tags.
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
 * It stores the last 10 MACK messages and the last 11 OSNMA_NavData messages.
 * Then attempts to verify the Tesla Key by computing the number of hashes of distance between the key-to-verify and the
 * Kroot and iteratively hashing the result, until the required number of hashes is achieved.
 * The result is then compared with the Kroot. If the two values match, the Tesla key is verified.
 *  It also performs MACSEQ validation and compares the ADKD of Mack tags with MACLT defined ADKDs.
 *  Finally, it verifies the tags.
 * \pre Kroot or already a TESLA key shall be available. Depending on the ADKD of the tag, OSNMA_NavData of SF(i-2)...SF(i-11)
 * \post Number of tags bits verified for each ADKD. MACSEQ verification success
 * @param osnma_msg A reference to OSNMA_msg containing the MACK message to be processed.
 */
void osnma_msg_receiver::process_mack_message()
{
    if (!d_kroot_verified && !d_tesla_key_verified)
        {
            LOG(WARNING) << "Galileo OSNMA: MACK cannot be processed, "
                         << "no Kroot nor TESLA key available.";
            return;  // early return, cannot proceed further without one of the two verified. this equals to having Kroot but no TESLa key yet.
        }
    // verify tesla key and add it to the container of verified keys if successful
    if (d_tesla_keys.find(d_osnma_data.d_nav_data.get_tow_sf0()) == d_tesla_keys.end())  // check if already available => no need to verify
        {
            bool retV = verify_tesla_key(d_osnma_data.d_mack_message.key, d_osnma_data.d_nav_data.get_tow_sf0());
            if (retV)
                {
                    d_tesla_keys.insert(std::pair<uint32_t, std::vector<uint8_t>>(d_osnma_data.d_nav_data.get_tow_sf0(), d_osnma_data.d_mack_message.key));
                }
        }

    // MACSEQ - verify current macks, then add current retrieved mack to the end.
    auto mack = d_macks_awaiting_MACSEQ_verification.begin();
    while (mack != d_macks_awaiting_MACSEQ_verification.end())
        {
            if (d_tesla_keys.find(mack->TOW + 30) != d_tesla_keys.end())
                {
                    // add tag0 first
                    Tag tag0(*mack);
                    d_tags_awaiting_verify.insert(std::pair<uint32_t, Tag>(mack->TOW, tag0));
                    //                    bool ret = verify_macseq(*mack);
                    std::vector<MACK_tag_and_info> macseq_verified_tags = verify_macseq_new(*mack);
                    for (auto& tag_and_info : macseq_verified_tags)
                        {
                            // add tags of current mack to the verification queue
                            Tag t(tag_and_info, mack->TOW, mack->WN, mack->PRNa, tag_and_info.counter);
                            d_tags_awaiting_verify.insert(std::pair<uint32_t, Tag>(mack->TOW, t));
                            LOG(INFO) << "Galileo OSNMA: Add Tag Id= "
                                      << t.tag_id
                                      << ", value=0x" << std::setfill('0') << std::setw(10) << std::hex << std::uppercase
                                      << t.received_tag << std::dec
                                      << ", TOW="
                                      << t.TOW
                                      << ", ADKD="
                                      << static_cast<unsigned>(t.ADKD)
                                      << ", PRNa="
                                      << static_cast<unsigned>(t.PRNa)
                                      << ", PRNd="
                                      << static_cast<unsigned>(t.PRN_d);
                        }
                    LOG(INFO) << "Galileo OSNMA: d_tags_awaiting_verify :: size: " << d_tags_awaiting_verify.size();
                    mack = d_macks_awaiting_MACSEQ_verification.erase(mack);
                }
            else
                {
                    // key not yet available - keep in container until then -- might be deleted if container size exceeds max allowed
                    ++mack;
                }
        }
    // add current received MACK to the container to be verified in the next iteration (on this one no key available)
    d_macks_awaiting_MACSEQ_verification.push_back(d_osnma_data.d_mack_message);

    // Tag verification
    for (auto& it : d_tags_awaiting_verify)
        {
            bool ret;
            if (tag_has_key_available(it.second) && d_nav_data_manager->have_nav_data(it.second))  // tag_has_nav_data_available(it.second))
                {
                    ret = verify_tag(it.second);
                    /* TODO - take into account:
                     * - COP: if
                     * - ADKD type
                     * - OSNMA_NavData the tag verifies (min. number of bits verified to consider OSNMA_NavData OK)
                     * */
                    if (ret)
                        {
                            d_count_successful_tags++;
                            it.second.status = Tag::SUCCESS;
                            LOG(INFO) << "Galileo OSNMA: Tag verification :: SUCCESS for tag Id="
                                      << it.second.tag_id
                                      << ", value=0x" << std::setfill('0') << std::setw(10) << std::hex << std::uppercase
                                      << it.second.received_tag << std::dec
                                      << ", TOW="
                                      << it.second.TOW
                                      << ", ADKD="
                                      << static_cast<unsigned>(it.second.ADKD)
                                      << ", PRNa="
                                      << static_cast<unsigned>(it.second.PRNa)
                                      << ", PRNd="
                                      << static_cast<unsigned>(it.second.PRN_d);
                            std::cout << "Galileo OSNMA: Tag verification :: SUCCESS for tag ADKD="
                                      << static_cast<unsigned>(it.second.ADKD)
                                      << ", PRNa="
                                      << static_cast<unsigned>(it.second.PRNa)
                                      << ", PRNd="
                                      << static_cast<unsigned>(it.second.PRN_d) << std::endl;
                        }
                    /* TODO notify PVT via pmt
                     * have_new_data() true
                     * signal which one is verified
                     * communicate to PVT*/
                    else
                        {
                            d_count_failed_tags++;
                            it.second.status = Tag::FAIL;
                            LOG(WARNING) << "Galileo OSNMA: Tag verification :: FAILURE for tag Id="
                                         << it.second.tag_id
                                         << ", value=0x" << std::setfill('0') << std::setw(10) << std::hex << std::uppercase
                                         << it.second.received_tag << std::dec
                                         << ", TOW="
                                         << it.second.TOW
                                         << ", ADKD="
                                         << static_cast<unsigned>(it.second.ADKD)
                                         << ", PRNa="
                                         << static_cast<unsigned>(it.second.PRNa)
                                         << ", PRNd="
                                         << static_cast<unsigned>(it.second.PRN_d);
                            std::cerr << "Galileo OSNMA: Tag verification :: FAILURE for tag ADKD="
                                      << static_cast<unsigned>(it.second.ADKD)
                                      << ", PRNa="
                                      << static_cast<unsigned>(it.second.PRNa)
                                      << ", PRNd="
                                      << static_cast<unsigned>(it.second.PRN_d) << std::endl;
                        }
                }
            else if (it.second.TOW > d_osnma_data.d_nav_data.get_tow_sf0())
                {
                    // TODO - I dont understand logic. This needs to be reviewed.
                    // case 1: adkd=12 and t.Tow + 300 < current TOW
                    // case 2: adkd=0/4 and t.Tow + 30 < current TOW
                    // case 3: any adkd and t.Tow > current TOW
                    it.second.skipped++;
                    LOG(WARNING) << "Galileo OSNMA: Tag verification :: SKIPPED (x" << it.second.skipped << ")for Tag Id= "
                                 << it.second.tag_id
                                 << ", value=0x" << std::setfill('0') << std::setw(10) << std::hex << std::uppercase
                                 << it.second.received_tag << std::dec
                                 << ", TOW="
                                 << it.second.TOW
                                 << ", ADKD="
                                 << static_cast<unsigned>(it.second.ADKD)
                                 << ", PRNa="
                                 << static_cast<unsigned>(it.second.PRNa)
                                 << ", PRNd="
                                 << static_cast<unsigned>(it.second.PRN_d)
                                 << ". Key available (" << tag_has_key_available(it.second) << "),  navData (" << tag_has_nav_data_available(it.second) << "). ";
                }
        }
    uint8_t tag_size = 0;
    const auto it = OSNMA_TABLE_11.find(d_osnma_data.d_dsm_kroot_message.ts);
    if (it != OSNMA_TABLE_11.cend())
        {
            tag_size = it->second;
        }
    d_nav_data_manager->update_nav_data(d_tags_awaiting_verify, tag_size);
    auto data_to_send = d_nav_data_manager->get_verified_data();
    d_nav_data_manager->print_status();
    send_data_to_pvt(data_to_send);

    remove_verified_tags();

    control_tags_awaiting_verify_size();  // remove the oldest tags if size is too big.
}


/**
 * @brief Verify received DSM-PKR message
 *
 * \details This method provides the functionality to verify the DSM-PKR message. The verification includes generating the base leaf
 * and intermediate leafs, and comparing the computed merkle root leaf with the received one.
 * \pre DSM_PKR_message correctly filled in, especially the 1024-bit intermediate tree nodes fields
 * \returns true if computed merkle root matches received one, false otherwise
 */
bool osnma_msg_receiver::verify_dsm_pkr(const DSM_PKR_message& message) const
{
    const auto base_leaf = get_merkle_tree_leaves(message);                     // m_i
    const auto computed_merkle_root = compute_merkle_root(message, base_leaf);  // x_4_0
    const auto msg_id = static_cast<int>(message.mid);
    LOG(INFO) << "Galileo OSNMA: DSM-PKR verification :: leaf provided for Message ID " << msg_id;

    if (computed_merkle_root == d_crypto->get_merkle_root())
        {
            LOG(INFO) << "Galileo OSNMA: DSM-PKR verification for Message ID " << msg_id << " :: SUCCESS. PKID=" << static_cast<unsigned>(message.npktid);
            std::cout << "Galileo OSNMA: DSM-PKR verification for Message ID " << msg_id << " :: SUCCESS. PKID=" << static_cast<unsigned>(message.npktid) << std::endl;
            return true;
        }
    else
        {
            LOG(WARNING) << "Galileo OSNMA: DSM-PKR verification for Message ID " << msg_id << " :: FAILURE.";
            std::cerr << "Galileo OSNMA: DSM-PKR verification for Message ID " << msg_id << " :: FAILURE." << std::endl;
            return false;
        }
}


std::vector<uint8_t> osnma_msg_receiver::compute_merkle_root(const DSM_PKR_message& dsm_pkr_message, const std::vector<uint8_t>& m_i) const
{
    std::vector<uint8_t> x_next;
    std::vector<uint8_t> x_current = d_crypto->compute_SHA_256(m_i);
    for (size_t i = 0; i < 4; i++)
        {
            x_next.clear();
            bool leaf_is_on_right = ((dsm_pkr_message.mid / (1 << (i))) % 2) == 1;

            if (leaf_is_on_right)
                {
                    // Leaf is on the right -> first the itn, then concatenate the leaf
                    x_next.insert(x_next.end(), &dsm_pkr_message.itn[32 * i], &dsm_pkr_message.itn[32 * i + 32]);
                    x_next.insert(x_next.end(), x_current.begin(), x_current.end());
                }
            else
                {
                    // Leaf is on the left -> first the leaf, then concatenate the itn
                    x_next.insert(x_next.end(), x_current.begin(), x_current.end());
                    x_next.insert(x_next.end(), &dsm_pkr_message.itn[32 * i], &dsm_pkr_message.itn[32 * i + 32]);
                }

            // Compute the next node.
            x_current = d_crypto->compute_SHA_256(x_next);
        }
    return x_current;
}


std::vector<uint8_t> osnma_msg_receiver::get_merkle_tree_leaves(const DSM_PKR_message& dsm_pkr_message) const
{
    // build base leaf m_i according to OSNMA SIS ICD v1.1, section 6.2 DSM-PKR Verification
    std::vector<uint8_t> m_i;
    const size_t size_npk = dsm_pkr_message.npk.size();
    m_i.reserve(1 + size_npk);
    m_i.push_back((dsm_pkr_message.npkt << 4) + dsm_pkr_message.npktid);
    for (size_t i = 0; i < size_npk; i++)
        {
            m_i.push_back(dsm_pkr_message.npk[i]);
        }
    return m_i;
}


bool osnma_msg_receiver::verify_tag(Tag& tag) const
{
    // Debug
    //    LOG(INFO) << "Galileo OSNMA: Tag verification :: Start for tag Id= "
    //              << tag.tag_id
    //              << ", value=0x" << std::setfill('0') << std::setw(10) << std::hex << std::uppercase
    //              << tag.received_tag << std::dec;
    // build message
    std::vector<uint8_t> m = build_message(tag);

    std::vector<uint8_t> mac;
    std::vector<uint8_t> applicable_key;
    if (tag.ADKD == 0 || tag.ADKD == 4)
        {
            const auto it = d_tesla_keys.find(tag.TOW + 30);
            if (it != d_tesla_keys.cend())
                {
                    applicable_key = it->second;
                }
            else
                {
                    return false;
                }
            // LOG(INFO) << "|---> Galileo OSNMA :: applicable key: 0x" << d_helper->convert_to_hex_string(applicable_key) << "TOW="<<static_cast<int>(tag.TOW + 30);
        }
    else  // ADKD 12
        {
            const auto it = d_tesla_keys.find(tag.TOW + 330);
            if (it != d_tesla_keys.cend())
                {
                    applicable_key = it->second;
                }
            else
                {
                    return false;
                }
            // LOG(INFO) << "|---> Galileo OSNMA :: applicable key: 0x" << d_helper->convert_to_hex_string(applicable_key) << "TOW="<<static_cast<int>(tag.TOW + 330);
        }

    if (d_osnma_data.d_dsm_kroot_message.mf == 0)  // C: HMAC-SHA-256
        {
            mac = d_crypto->compute_HMAC_SHA_256(applicable_key, m);
        }
    else if (d_osnma_data.d_dsm_kroot_message.mf == 1)  // C: CMAC-AES
        {
            mac = d_crypto->compute_CMAC_AES(applicable_key, m);
        }

    // truncate the computed mac: trunc(l_t, mac(K,m)) Eq. 23 ICD
    uint8_t lt_bits = 0;  // TODO - remove this duplication of code.
    const auto it2 = OSNMA_TABLE_11.find(d_osnma_data.d_dsm_kroot_message.ts);
    if (it2 != OSNMA_TABLE_11.cend())
        {
            lt_bits = it2->second;
        }
    if (lt_bits == 0)
        {
            return false;
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

    tag.computed_tag = computed_mac;  // update with computed value
    // Compare computed tag with received one truncated
    if (tag.received_tag == computed_mac)
        {
            return true;
        }
    return false;
}


/**
 * \brief generates the message for computing the tag
 * \remarks It also sets some parameters to the Tag object, based on the verification process.
 *
 * \param tag The tag containing the information to be included in the message.
 *
 * \return The built OSNMA message as a vector of uint8_t.
 */
std::vector<uint8_t> osnma_msg_receiver::build_message(Tag& tag) const
{
    std::vector<uint8_t> m;
    if (tag.CTR != 1)
        {
            m.push_back(static_cast<uint8_t>(tag.PRN_d));
        }
    m.push_back(static_cast<uint8_t>(tag.PRNa));
    // TODO: maybe here I have to use d_receiver_time instead of d_GST_SIS which is what I am computing
    uint32_t GST = d_helper->compute_gst(tag.WN, tag.TOW);
    std::vector<uint8_t> GST_uint8 = d_helper->gst_to_uint8(GST);
    m.insert(m.end(), GST_uint8.begin(), GST_uint8.end());
    m.push_back(tag.CTR);
    // Extracts only two bits from d_osnma_data.d_nma_header.nmas
    uint8_t two_bits_nmas = d_osnma_data.d_nma_header.nmas & 0b00000011;
    two_bits_nmas = two_bits_nmas << 6;
    m.push_back(two_bits_nmas);

    // Add applicable NavData bits to message
    std::string applicable_nav_data = d_nav_data_manager->get_navigation_data(tag);
    std::vector<uint8_t> applicable_nav_data_bytes = d_helper->bytes(applicable_nav_data);
    tag.nav_data = applicable_nav_data;  // update tag with applicable data

    // Convert and add OSNMA_NavData bytes into the message, taking care of that NMAS has only 2 bits
    for (uint8_t byte : applicable_nav_data_bytes)
        {
            m.back() |= (byte >> 2);  // First take the 6 MSB bits of byte and add to m
            m.push_back(byte << 6);   // Then take the last 2 bits of byte, shift them to MSB position and insert the new element into m
        }
    if (m.back() == 0)
        {
            m.pop_back();  // Remove the last element if its value is 0 (only padding was added)
        }
    else
        {
            // Pad with zeros if the last element wasn't full
            for (int bits = 2; bits < 8; bits += 2)
                {
                    // Check if the last element in the vector has 2 '00' bits in its LSB position
                    if ((m.back() & 0b00000011) == 0)
                        {
                            m.back() <<= 2;  // Shift the existing bits  to make room for new 2 bits
                        }
                    else
                        {
                            break;  // If it does not have 2 '00' bits in its LSB position, then the padding is complete
                        }
                }
        }
    return m;
}

void osnma_msg_receiver::display_data()
{
    //    if(d_satellite_nav_data.empty())
    //        return;
    //
    //    for(const auto& satellite : d_satellite_nav_data) {
    //            std::cout << "SV_ID: " << satellite.first << std::endl;
    //            for(const auto& towData : satellite.second) {
    //                    std::cout << "\tTOW: " << towData.first << " key: ";
    //                    for(size_t i = 0; i < towData.second.d_mack_message.key.size(); i++) {
    //                        std::cout << std::hex << std::setfill('0') << std::setw(2)
    //                              << static_cast<int>(towData.second.d_mack_message.key[i]) << " ";
    //                    }
    //                }
    //        }
}


bool osnma_msg_receiver::verify_tesla_key(std::vector<uint8_t>& key, uint32_t TOW)
{
    uint32_t num_of_hashes_needed;
    uint32_t GST_SFi = d_GST_Sf - 30;  // GST of target key is to be used.
    std::vector<uint8_t> hash;
    const uint8_t lk_bytes = d_dsm_reader->get_lk_bits(d_osnma_data.d_dsm_kroot_message.ks) / 8;
    std::vector<uint8_t> validated_key;
    if (d_tesla_key_verified)
        {  // have to go up to last verified key
            validated_key = d_tesla_keys.rbegin()->second;
            num_of_hashes_needed = (d_GST_Sf - d_last_verified_key_GST) / 30;  // Eq. 19 ICD modified
            LOG(INFO) << "Galileo OSNMA: TESLA verification (" << num_of_hashes_needed << " hashes) need to be performed up to closest verified TESLA key";

            hash = hash_chain(num_of_hashes_needed, key, GST_SFi, lk_bytes);
        }
    else
        {  // have to go until Kroot
            validated_key = d_osnma_data.d_dsm_kroot_message.kroot;
            num_of_hashes_needed = (d_GST_Sf - d_GST_0) / 30 + 1;  // Eq. 19 IC
            LOG(INFO) << "Galileo OSNMA: TESLA verification (" << num_of_hashes_needed << " hashes) need to be performed up to Kroot";

            hash = hash_chain(num_of_hashes_needed, key, GST_SFi, lk_bytes);
        }
    // truncate hash
    std::vector<uint8_t> computed_key;
    computed_key.reserve(key.size());
    for (size_t i = 0; i < key.size(); i++)
        {
            computed_key.push_back(hash[i]);
        }
    if (computed_key == validated_key && num_of_hashes_needed > 0)
        {
            LOG(INFO) << "Galileo OSNMA: TESLA key verification :: SUCCESS!";
            std::cout << "Galileo OSNMA: TESLA key verification :: SUCCESS!" << std::endl;
            d_tesla_keys.insert(std::pair<uint32_t, std::vector<uint8_t>>(TOW, key));
            d_tesla_key_verified = true;
            d_last_verified_key_GST = d_GST_Sf;
        }
    else if (num_of_hashes_needed > 0)
        {
            LOG(WARNING) << "Galileo OSNMA: TESLA key verification :: FAILED";
            std::cerr << "Galileo OSNMA: TESLA key verification :: FAILED" << std::endl;
        }
    return d_tesla_key_verified;
}

/**
 * @brief Removes the tags that have been verified from the multimap d_tags_awaiting_verify.
 *
 * This function iterates through the multimap d_tags_awaiting_verify, and removes the tags that have a status of SUCCESS or FAIL.
 * \remarks it also prints the current unverified tags
 */
void osnma_msg_receiver::remove_verified_tags()
{
    for (auto it = d_tags_awaiting_verify.begin(); it != d_tags_awaiting_verify.end();)
        {
            if (it->second.status == Tag::SUCCESS || it->second.status == Tag::FAIL)
                {
                    LOG(INFO) << "Galileo OSNMA: Tag verification :: DELETE tag Id="
                              << it->second.tag_id
                              << ", value=0x" << std::setfill('0') << std::setw(10) << std::hex << std::uppercase
                              << it->second.received_tag << std::dec
                              << ", TOW="
                              << it->second.TOW
                              << ", ADKD="
                              << static_cast<unsigned>(it->second.ADKD)
                              << ", PRNa="
                              << static_cast<unsigned>(it->second.PRNa)
                              << ", PRNd="
                              << static_cast<unsigned>(it->second.PRN_d)
                              << ", status= "
                              << d_helper->verification_status_str(it->second.status);
                    it = d_tags_awaiting_verify.erase(it);
                }
            else if (it->second.skipped >= 20)
                {
                    LOG(INFO) << "Galileo OSNMA: Tag verification :: DELETE tag Id="
                              << it->second.tag_id
                              << ", value=0x" << std::setfill('0') << std::setw(10) << std::hex << std::uppercase
                              << it->second.received_tag << std::dec
                              << ", TOW="
                              << it->second.TOW
                              << ", ADKD="
                              << static_cast<unsigned>(it->second.ADKD)
                              << ", PRNa="
                              << static_cast<unsigned>(it->second.PRNa)
                              << ", PRNd="
                              << static_cast<unsigned>(it->second.PRN_d)
                              << ", status= "
                              << d_helper->verification_status_str(it->second.status);
                    it = d_tags_awaiting_verify.erase(it);
                }
            else
                {
                    ++it;
                }
        }
    LOG(INFO) << "Galileo OSNMA: d_tags_awaiting_verify :: size: " << d_tags_awaiting_verify.size();
    for (const auto& it : d_tags_awaiting_verify)
        {
            LOG(INFO) << "Galileo OSNMA: Tag verification :: status tag Id="
                      << it.second.tag_id
                      << ", value=0x" << std::setfill('0') << std::setw(10) << std::hex << std::uppercase
                      << it.second.received_tag << std::dec
                      << ", TOW="
                      << it.second.TOW
                      << ", ADKD="
                      << static_cast<unsigned>(it.second.ADKD)
                      << ", PRNa="
                      << static_cast<unsigned>(it.second.PRNa)
                      << ", PRNd="
                      << static_cast<unsigned>(it.second.PRN_d)
                      << ", status= "
                      << d_helper->verification_status_str(it.second.status);
        }
}

/**
 * @brief Control the size of the tags awaiting verification multimap.
 *
 * This function checks the size of the multimap `d_tags_awaiting_verify` and removes
 * elements from the beginning until the size is no longer greater than 60.
 * The purpose is to limit the size of the multimap and prevent it from consuming
 * excessive memory.
 */
void osnma_msg_receiver::control_tags_awaiting_verify_size()
{
    while (d_tags_awaiting_verify.size() > 500)
        {
            auto it = d_tags_awaiting_verify.begin();
            LOG(INFO) << "Galileo OSNMA: Tag verification :: DELETED tag due to exceeding buffer size. "
                      << "Tag Id= " << it->second.tag_id
                      << ", TOW=" << it->first
                      << ", ADKD=" << static_cast<unsigned>(it->second.ADKD)
                      << ", from satellite " << it->second.PRNa;
            d_tags_awaiting_verify.erase(it);
        }
}


// TODO - remove this method
/**
 * @brief Verifies the MACSEQ of a received MACK_message.
 *
 * \details checks for each tag in the retrieved mack message if its flexible (MACSEQ) or not (MACSEQ/MACLT depending on configuration param, and
 * verifies according to Eqs. 20, 21 SIS ICD.
 * @param message The MACK_message to verify.
 * @return True if the MACSEQ is valid, false otherwise.
 */
bool osnma_msg_receiver::verify_macseq(const MACK_message& mack)
{
    // MACSEQ verification
    uint32_t GST_SFi = d_GST_Sf - 30;                                   // time of the start of SF containing MACSEQ
    std::vector<uint8_t> applicable_key = d_tesla_keys[mack.TOW + 30];  // current tesla key ie transmitted in the next subframe
    std::vector<std::string> sq1{};
    std::vector<std::string> sq2{};
    std::vector<std::string> applicable_sequence;
    const auto it = OSNMA_TABLE_16.find(d_osnma_data.d_dsm_kroot_message.maclt);
    // TODO as per RG example appears that the seq. q shall also be validated against either next or former Sf (depending on GST)
    if (it != OSNMA_TABLE_16.cend())
        {
            sq1 = it->second.sequence1;
            sq2 = it->second.sequence2;
        }
    // Assign relevant sequence based on subframe time
    if (mack.TOW % 60 < 30)  // tried GST_Sf and it does not support the data present.
        {
            applicable_sequence = sq1;
        }
    else if (mack.TOW % 60 >= 30)
        {
            applicable_sequence = sq2;
        }
    if (mack.tag_and_info.size() != applicable_sequence.size() - 1)
        {
            LOG(WARNING) << "Galileo OSNMA: Number of retrieved tags does not match MACLT sequence size!";
            return false;
        }
    std::vector<uint8_t> flxTags{};
    std::string tempADKD;
    // MACLT verification
    for (size_t i = 0; i < mack.tag_and_info.size(); i++)
        {
            tempADKD = applicable_sequence[i + 1];
            if (tempADKD == "FLX")
                {
                    flxTags.push_back(i);  // C: just need to save the index in the sequence
                }
            else if (mack.tag_and_info[i].tag_info.ADKD != std::stoi(applicable_sequence[i + 1]))
                {
                    // fill index of tags failed
                    LOG(WARNING) << "Galileo OSNMA: MACSEQ verification :: FAILURE :: ADKD mismatch against MAC Look-up table.";
                    return false;  // TODO macseq shall be individual to each tag, a wrongly verified macseq should not discard the whole MACK tags
                }
        }

    if (flxTags.empty())
        {
            LOG(INFO) << "Galileo OSNMA: MACSEQ verification :: SUCCESS :: ADKD matches MAC Look-up table.";
            return true;
        }
    // Fixed as well as  FLX Tags share first part - Eq. 22 ICD
    std::vector<uint8_t> m(5 + 2 * flxTags.size());  // each flx tag brings two bytes
    m[0] = static_cast<uint8_t>(mack.PRNa);          // PRN_A - SVID of the satellite transmiting the tag
    m[1] = static_cast<uint8_t>((GST_SFi & 0xFF000000) >> 24);
    m[2] = static_cast<uint8_t>((GST_SFi & 0x00FF0000) >> 16);
    m[3] = static_cast<uint8_t>((GST_SFi & 0x0000FF00) >> 8);
    m[4] = static_cast<uint8_t>(GST_SFi & 0x000000FF);
    // Case tags flexible - Eq. 21 ICD
    for (size_t i = 0; i < flxTags.size(); i++)
        {
            m[2 * i + 5] = mack.tag_and_info[flxTags[i]].tag_info.PRN_d;
            m[2 * i + 6] = mack.tag_and_info[flxTags[i]].tag_info.ADKD << 4 |
                           mack.tag_and_info[flxTags[i]].tag_info.cop;
        }
    // compute mac
    std::vector<uint8_t> mac;
    if (d_osnma_data.d_dsm_kroot_message.mf == 0)  // C: HMAC-SHA-256
        {
            mac = d_crypto->compute_HMAC_SHA_256(applicable_key, m);
        }
    else if (d_osnma_data.d_dsm_kroot_message.mf == 1)  // C: CMAC-AES
        {
            mac = d_crypto->compute_CMAC_AES(applicable_key, m);
        }
    // Truncate the twelve MSBits and compare with received MACSEQ
    uint16_t mac_msb = 0;
    if (!mac.empty())
        {
            mac_msb = (mac[0] << 8) + mac[1];
        }
    uint16_t computed_macseq = (mac_msb & 0xFFF0) >> 4;
    if (computed_macseq == mack.header.macseq)
        {
            LOG(INFO) << "Galileo OSNMA: MACSEQ verification :: SUCCESS :: FLX tags verification OK";
            return true;
        }
    else
        {
            LOG(WARNING) << "Galileo OSNMA: MACSEQ verification :: FAILURE :: FLX tags verification failed";
            return false;
        }
}


bool osnma_msg_receiver::tag_has_nav_data_available(const Tag& t) const
{
    auto prn_it = d_satellite_nav_data.find(t.PRN_d);
    if (prn_it != d_satellite_nav_data.end())
        {
            // PRN was found, check if TOW exists in inner map
            // LOG(INFO) << "Galileo OSNMA: hasData = true " << std::endl;
            std::map<uint32_t, OSNMA_NavData> tow_map = prn_it->second;
            auto tow_it = tow_map.find(t.TOW - 30);
            if (tow_it != tow_map.end())
                {
                    return true;
                }
            else
                {
                    // TOW not found
                    return false;
                }
        }
    else
        {
            // PRN was not found
            // LOG(INFO) << "Galileo OSNMA: hasData = false " << std::endl;
            return false;
        }
    return false;
}


bool osnma_msg_receiver::tag_has_key_available(const Tag& t) const
{
    // check adkd of tag
    // if adkd = 0 or 4 => look for d_tesla_keys[t.TOW+30]
    // if adkd = 12 => look for d_tesla_keys[t.TOW+300]
    // return true if available, otherwise false

    if (t.ADKD == 0 || t.ADKD == 4)
        {
            auto it = d_tesla_keys.find(t.TOW + 30);
            if (it != d_tesla_keys.end())
                {
                    // LOG(INFO) << "Galileo OSNMA: hasKey = true " << std::endl;
                    return true;
                }
        }
    else if (t.ADKD == 12)
        {
            auto it = d_tesla_keys.find(t.TOW + 330);
            if (it != d_tesla_keys.end())
                {
                    // LOG(INFO) << "Galileo OSNMA: hasKey = true " << std::endl;
                    return true;
                }
        }
    // LOG(INFO) << "Galileo OSNMA: hasKey = false ";
    return false;
}


std::vector<uint8_t> osnma_msg_receiver::hash_chain(uint32_t num_of_hashes_needed, const std::vector<uint8_t>& key, uint32_t GST_SFi, const uint8_t lk_bytes) const
{
    std::vector<uint8_t> K_II = key;
    std::vector<uint8_t> K_I;  // result of the recursive hash operations
    std::vector<uint8_t> msg;
    // compute the tesla key for current SF (GST_SFi and K_II change in each iteration)
    for (uint32_t i = 1; i <= num_of_hashes_needed; i++)
        {
            // build message digest m = (K_I+1 || GST_SFi || alpha)
            msg.reserve(K_II.size() + sizeof(GST_SFi) + sizeof(d_osnma_data.d_dsm_kroot_message.alpha));
            std::copy(K_II.begin(), K_II.end(), std::back_inserter(msg));

            msg.push_back((GST_SFi & 0xFF000000) >> 24);
            msg.push_back((GST_SFi & 0x00FF0000) >> 16);
            msg.push_back((GST_SFi & 0x0000FF00) >> 8);
            msg.push_back(GST_SFi & 0x000000FF);
            // extract alpha
            //            d_osnma_data.d_dsm_kroot_message.alpha = 0xa06221261ad9;
            for (int k = 5; k >= 0; k--)
                {
                    // TODO: static extracts the MSB in case from larger to shorter int?
                    msg.push_back(static_cast<uint8_t>((d_osnma_data.d_dsm_kroot_message.alpha >> (k * 8)) & 0xFF));  // extract first 6 bytes of alpha.
                }
            // compute hash
            std::vector<uint8_t> hash;
            if (d_osnma_data.d_dsm_kroot_message.hf == 0)  // Table 8.
                {
                    hash = d_crypto->compute_SHA_256(msg);
                }
            else if (d_osnma_data.d_dsm_kroot_message.hf == 2)
                {
                    hash = d_crypto->compute_SHA3_256(msg);
                }
            else
                {
                    hash = std::vector<uint8_t>(32);
                }
            // truncate hash
            K_I.reserve(lk_bytes);  // TODO - case hash function has 512 bits
            for (int k = 0; k < lk_bytes; k++)
                {
                    K_I.push_back(hash[k]);
                }
            // set parameters for next iteration
            GST_SFi -= 30;  // next SF time is the actual minus 30 seconds
            K_II = K_I;     // next key is the actual one
            K_I.clear();    // empty the actual one for a new computation
            msg.clear();
        }

    // check that the final time matches the Kroot time
    bool check;
    if (!d_tesla_key_verified)
        {
            check = GST_SFi + 30 == d_GST_0 - 30;
        }
    else
        {
            check = GST_SFi + 30 == d_last_verified_key_GST;
        }
    if (!check)
        {
            LOG(WARNING) << "Galileo OSNMA: TESLA key chain verification error: KROOT time mismatch!";  // ICD. Eq. 18
            std::cerr << "Galileo OSNMA: TESLA key chain verification error: KROOT time mismatch!" << std::endl;
        }
    else
        {
            LOG(INFO) << "Galileo OSNMA: TESLA key chain verification: KROOT time matches.";  // ICD. Eq. 18
        }
    return K_II;
}


/**
 * @brief Verifies the MAC sequence of a received MACK message.
 *
 * This function is responsible for verifying the MAC sequence of a received MACK message.
 * It takes a reference to a constant MACK_message object as input and returns a vector containing
 * the tags for which the MACSEQ verification was successful
 *
 * @param mack The MACK message object to verify the MAC sequence for.
 * @return vector MACK_tag_and_info for which the MACSEQ was successful
 */
std::vector<MACK_tag_and_info> osnma_msg_receiver::verify_macseq_new(const MACK_message& mack)
{
    std::vector<MACK_tag_and_info> verified_tags{};

    // MACSEQ verification
    uint32_t GST_Sfi = d_GST_Sf - 30;  // time of the start of SF containing MACSEQ
    std::vector<uint8_t> applicable_key;
    const auto key_it = d_tesla_keys.find(mack.TOW + 30);  // current tesla key ie transmitted in the next subframe
    if (key_it != d_tesla_keys.cend())
        {
            applicable_key = key_it->second;
        }
    std::vector<std::string> sq1{};
    std::vector<std::string> sq2{};
    std::vector<std::string> applicable_sequence;
    const auto it = OSNMA_TABLE_16.find(d_osnma_data.d_dsm_kroot_message.maclt);
    if (it != OSNMA_TABLE_16.cend())
        {
            sq1 = it->second.sequence1;
            sq2 = it->second.sequence2;
        }

    // Assign relevant sequence based on subframe time
    if (mack.TOW % 60 < 30)  // tried GST_Sf and it does not support the data present.
        {
            applicable_sequence = sq1;
        }
    else if (mack.TOW % 60 >= 30)
        {
            applicable_sequence = sq2;
        }
    if (mack.tag_and_info.size() != applicable_sequence.size() - 1)
        {
            LOG(WARNING) << "Galileo OSNMA: Number of retrieved tags does not match MACLT sequence size!";
            d_count_failed_macseq += mack.tag_and_info.size();
            return verified_tags;
        }
    std::vector<uint8_t> flxTags{};
    std::string tempADKD;
    // MACLT verification
    for (size_t i = 0; i < mack.tag_and_info.size(); i++)
        {
            tempADKD = applicable_sequence[i + 1];
            if (tempADKD == "FLX")
                {
                    flxTags.push_back(i);  // C: just need to save the index in the sequence
                }
            else if (mack.tag_and_info[i].tag_info.ADKD == std::stoi(applicable_sequence[i + 1]))
                {
                    // fill index of tags failed
                    LOG(INFO) << "Galileo OSNMA: MACSEQ verification :: SUCCESS :: ADKD match against MAC Look-up table for Tag=0x"
                              << std::setfill('0') << std::setw(10) << std::hex << std::uppercase
                              << mack.tag_and_info[i].tag << std::dec;
                    verified_tags.push_back(mack.tag_and_info[i]);
                }
            else
                {
                    // discard tag
                    LOG(WARNING) << "Galileo OSNMA: MACSEQ verification :: FAILURE :: ADKD mismatch against MAC Look-up table for Tag=0x"
                                 << std::setfill('0') << std::setw(10) << std::hex << std::uppercase
                                 << mack.tag_and_info[i].tag << std::dec;
                    d_count_failed_macseq++;
                }
        }

    if (flxTags.empty() /*TODO add check d_flag_check_mackseq_fixed_tags*/)
        {
            LOG(INFO) << "Galileo OSNMA: MACSEQ verification :: No FLX tags to verify.";
            return verified_tags;
        }
    // Fixed as well as  FLX Tags share first part - Eq. 22 ICD
    std::vector<uint8_t> m(5 + 2 * flxTags.size());  // each flx tag brings two bytes
    m[0] = static_cast<uint8_t>(mack.PRNa);          // PRN_A - SVID of the satellite transmiting the tag
    m[1] = static_cast<uint8_t>((GST_Sfi & 0xFF000000) >> 24);
    m[2] = static_cast<uint8_t>((GST_Sfi & 0x00FF0000) >> 16);
    m[3] = static_cast<uint8_t>((GST_Sfi & 0x0000FF00) >> 8);
    m[4] = static_cast<uint8_t>(GST_Sfi & 0x000000FF);
    // Case tags flexible - Eq. 21 ICD
    for (size_t i = 0; i < flxTags.size(); i++)
        {
            m[2 * i + 5] = mack.tag_and_info[flxTags[i]].tag_info.PRN_d;
            m[2 * i + 6] = mack.tag_and_info[flxTags[i]].tag_info.ADKD << 4 |
                           mack.tag_and_info[flxTags[i]].tag_info.cop;
        }
    // compute mac
    std::vector<uint8_t> mac;
    if (d_osnma_data.d_dsm_kroot_message.mf == 0)  // C: HMAC-SHA-256
        {
            mac = d_crypto->compute_HMAC_SHA_256(applicable_key, m);
        }
    else if (d_osnma_data.d_dsm_kroot_message.mf == 1)  // C: CMAC-AES
        {
            mac = d_crypto->compute_CMAC_AES(applicable_key, m);
        }
    // Truncate the twelve MSBits and compare with received MACSEQ
    uint16_t mac_msb = 0;
    if (!mac.empty())
        {
            mac_msb = (mac[0] << 8) + mac[1];
        }
    uint16_t computed_macseq = (mac_msb & 0xFFF0) >> 4;
    if (computed_macseq == mack.header.macseq)
        {
            LOG(INFO) << "Galileo OSNMA: MACSEQ verification :: SUCCESS :: FLX tags verification OK";
            for (uint8_t flxTag : flxTags)
                {
                    verified_tags.push_back(mack.tag_and_info[flxTag]);
                }
            return verified_tags;
        }
    else
        {
            LOG(WARNING) << "Galileo OSNMA: MACSEQ verification :: FAILURE :: FLX tags verification failed";
            d_count_failed_macseq += flxTags.size();
            return verified_tags;
        }
}

void osnma_msg_receiver::send_data_to_pvt(const std::vector<OSNMA_NavData>& data)
{
    if (!data.empty())
        {
            for (auto& i : data)
                {
                    const auto tmp_obj = std::make_shared<OSNMA_NavData>(i);
                    this->message_port_pub(pmt::mp("OSNMA_to_PVT"), pmt::make_any(tmp_obj));
                }
        }
}

bool osnma_msg_receiver::store_dsm_kroot(const std::vector<uint8_t>& dsm, const uint8_t nma_header) const
{
    std::ofstream file(KROOTFILE_DEFAULT, std::ios::binary | std::ios::out);

    if (!file.is_open())
        {
            return false;
        }

    // NMA header
    file.write(reinterpret_cast<const char*>(&nma_header), 1);

    // Then writing the entire dsm_msg vector to the file
    file.write(reinterpret_cast<const char*>(dsm.data()), dsm.size());

    return file.good();
}

std::pair<std::vector<uint8_t>, uint8_t> osnma_msg_receiver::parse_dsm_kroot() const
{
    std::ifstream file(KROOTFILE_DEFAULT, std::ios::binary | std::ios::in);
    if (!file)
        {
            return {std::vector<uint8_t>(), 0};
        }

    // Read the first byte into hkroot[0]
    uint8_t nma_header;
    file.read(reinterpret_cast<char*>(&nma_header), 1);

    // Read the remaining file content into dsm_msg
    std::vector<uint8_t> dsm_msg((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    file.close();

    if (file.bad())
        {
            return {std::vector<uint8_t>(), 0};
        }

    return {dsm_msg, nma_header};
}
