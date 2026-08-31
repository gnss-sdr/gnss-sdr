/*!
 * \file beidou_b2b_telemetry_decoder_gs.cc
 * \brief BeiDou B2b B-CNAV3 telemetry decoder
 * \author Chandoss, 2026. huangchh37(at)mail2.sysu.edu.cn
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "beidou_b2b_telemetry_decoder_gs.h"
#include "Beidou_B2b.h"
#include "Beidou_DNAV.h"
#include "beidou_cnav1_ephemeris.h"
#include "gnss_satellite.h"
#include "gnss_synchro.h"
#include "tlm_crc_stats.h"
#include "tlm_utils.h"
#include "tow_to_trk.h"
#include <pmt/pmt.h>
#include <pmt/pmt_sugar.h>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

namespace
{
constexpr int32_t CRC_ERROR_LIMIT = 8;
}  // namespace


beidou_b2b_telemetry_decoder_gs_sptr beidou_b2b_make_telemetry_decoder_gs(
    const Gnss_Satellite& satellite, const Tlm_Conf& conf)
{
    return beidou_b2b_telemetry_decoder_gs_sptr(new beidou_b2b_telemetry_decoder_gs(satellite, conf));
}


beidou_b2b_telemetry_decoder_gs::beidou_b2b_telemetry_decoder_gs(const Gnss_Satellite& satellite, const Tlm_Conf& conf)
    : telemetry_impl_interface("beidou_b2b_telemetry_decoder_gs",
          gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
          gr::io_signature::make(1, 1, sizeof(Gnss_Synchro))),
      d_satellite(satellite),
      d_dump_filename(conf.dump_filename),
      d_dump(conf.dump),
      d_dump_mat(conf.dump_mat),
      d_remove_dat(conf.remove_dat),
      d_enable_navdata_monitor(conf.enable_navdata_monitor),
      d_dump_crc_stats(conf.dump_crc_stats),
      d_tow_to_trk(conf.tow_to_trk)
{
    configure_basic_outputs();
    if (d_enable_navdata_monitor)
        {
            this->message_port_register_out(pmt::mp("Nav_msg_from_TLM"));
            d_nav_msg_packet.system = std::string("C");
            d_nav_msg_packet.signal = std::string("B2");
        }
}


beidou_b2b_telemetry_decoder_gs::~beidou_b2b_telemetry_decoder_gs()
{
    tlm_cleanup_and_save_files(d_dump_file, d_dump_filename, d_dump, d_dump_mat, d_remove_dat);
}


void beidou_b2b_telemetry_decoder_gs::set_satellite(const Gnss_Satellite& satellite)
{
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    d_nav_msg_packet.prn = static_cast<int32_t>(d_satellite.get_PRN());
}


void beidou_b2b_telemetry_decoder_gs::set_channel(int channel)
{
    d_channel = channel;
    configure_dump_file(d_channel, d_dump, d_dump_filename, d_dump_file);
    configure_crc_stats_channel(d_channel, d_dump_crc_stats, d_Tlm_CRC_Stats);
}


void beidou_b2b_telemetry_decoder_gs::reset()
{
    d_symbol_history.clear();
    d_flag_frame_sync = false;
    d_flag_valid_word = false;
    d_CRC_error_counter = 0;
    d_sample_counter = 0;
    d_preamble_index = 0;
}


void beidou_b2b_telemetry_decoder_gs::forecast(int noutput_items, gr_vector_int& ninput_items_required)
{
    ninput_items_required[0] = noutput_items;
}


void beidou_b2b_telemetry_decoder_gs::publish_navigation(double cn0_db_hz)
{
    if (d_nav.have_new_ephemeris())
        {
            auto eph = std::make_shared<Beidou_Cnav1_Ephemeris>(d_nav.get_ephemeris());
            eph->PRN = d_satellite.get_PRN();
            message_port_pub(pmt::mp("telemetry"), pmt::make_any(eph));
            std::cout << "New BeiDou B-CNAV3 ephemeris in channel " << d_channel
                      << " from satellite " << d_satellite
                      << " with CN0=" << std::setprecision(2) << cn0_db_hz << " dB-Hz\n";
            LOG(INFO) << "New BeiDou B-CNAV3 ephemeris from PRN " << d_satellite.get_PRN();
        }
    if (d_enable_navdata_monitor && !d_nav.get_last_nav_bits().empty())
        {
            d_nav_msg_packet.prn = static_cast<int32_t>(d_satellite.get_PRN());
            d_nav_msg_packet.tow_at_current_symbol_ms = static_cast<int32_t>(d_TOW_at_current_symbol_ms);
            d_nav_msg_packet.nav_message = d_nav.get_last_nav_bits();
            message_port_pub(pmt::mp("Nav_msg_from_TLM"), pmt::make_any(std::make_shared<Nav_Message_Packet>(d_nav_msg_packet)));
        }
    d_nav.clear_flags();
}


bool beidou_b2b_telemetry_decoder_gs::try_decode_frame()
{
    if (d_symbol_history.size() < static_cast<size_t>(BEIDOU_CNAV3_FRAME_SYMBOLS))
        {
            return false;
        }
    std::vector<float> frame(static_cast<size_t>(BEIDOU_CNAV3_FRAME_SYMBOLS));
    auto it = d_symbol_history.end() - BEIDOU_CNAV3_FRAME_SYMBOLS;
    for (int32_t i = 0; i < BEIDOU_CNAV3_FRAME_SYMBOLS; i++)
        {
            frame[static_cast<size_t>(i)] = *it++;
        }
    return d_nav.decode_frame_symbols(frame.data(), BEIDOU_CNAV3_FRAME_SYMBOLS, d_satellite.get_PRN());
}


int beidou_b2b_telemetry_decoder_gs::general_work(
    int noutput_items,
    gr_vector_int& ninput_items,
    gr_vector_const_void_star& input_items,
    gr_vector_void_star& output_items)
{
    if ((noutput_items < 1) || (ninput_items[0] < 1))
        {
            return 0;
        }
    const auto* in = reinterpret_cast<const Gnss_Synchro*>(input_items[0]);
    auto* out = reinterpret_cast<Gnss_Synchro*>(output_items[0]);
    Gnss_Synchro current = in[0];
    current.PRN = d_satellite.get_PRN();
    current.Flag_valid_word = false;
    bool frame_decoded = false;

    if (current.Flag_valid_symbol_output)
        {
            d_sample_counter++;
            d_symbol_history.push_back(static_cast<float>(current.Prompt_I));
            if (d_symbol_history.size() > static_cast<size_t>(BEIDOU_CNAV3_FRAME_SYMBOLS * 2))
                {
                    d_symbol_history.pop_front();
                }

            const bool history_ready =
                d_symbol_history.size() >= static_cast<size_t>(BEIDOU_CNAV3_FRAME_SYMBOLS);
            bool try_now = false;
            if (history_ready)
                {
                    if (!d_flag_frame_sync)
                        {
                            // Sliding 1 ms search until the first preamble+CRC lock.
                            try_now = true;
                        }
                    else if ((d_sample_counter > d_preamble_index) &&
                             ((d_sample_counter - d_preamble_index) %
                                 static_cast<uint64_t>(BEIDOU_CNAV3_FRAME_SYMBOLS)) == 0U)
                        {
                            try_now = true;
                        }
                }

            if (try_now)
                {
                    if (try_decode_frame())
                        {
                            frame_decoded = true;
                            d_flag_frame_sync = true;
                            d_CRC_error_counter = 0;
                            d_flag_valid_word = true;
                            d_preamble_index = d_sample_counter;
                            const int32_t sow = d_nav.last_sow();
                            std::cout << "B-CNAV3 CRC ok ch " << d_channel
                                      << " " << d_satellite
                                      << " MT" << d_nav.last_mes_type()
                                      << " SOW=" << sow
                                      << " eph=" << (d_nav.have_new_ephemeris() ? "yes" : "no")
                                      << '\n';
                            if (sow >= 0)
                                {
                                    // SOW is the start of this 1 s frame. Current symbol is the last.
                                    const double tow_gpst_s = static_cast<double>(sow) +
                                                              static_cast<double>(BEIDOU_DNAV_BDT2GPST_LEAP_SEC_OFFSET);
                                    d_TOW_at_current_symbol_ms = static_cast<uint32_t>(std::fmod(
                                        tow_gpst_s * 1000.0 + static_cast<double>(BEIDOU_CNAV3_FRAME_SYMBOLS),
                                        604800000.0));
                                }
                            if (d_Tlm_CRC_Stats)
                                {
                                    d_Tlm_CRC_Stats->update_CRC_stats(true);
                                }
                            publish_navigation(current.CN0_dB_hz);
                        }
                    else if (d_flag_frame_sync)
                        {
                            d_CRC_error_counter++;
                            if (d_Tlm_CRC_Stats)
                                {
                                    d_Tlm_CRC_Stats->update_CRC_stats(false);
                                }
                            if (d_CRC_error_counter > CRC_ERROR_LIMIT)
                                {
                                    d_flag_frame_sync = false;
                                    d_flag_valid_word = false;
                                    d_CRC_error_counter = 0;
                                }
                        }
                }
        }

    if (d_flag_valid_word && !frame_decoded)
        {
            if (current.Flag_valid_symbol_output)
                {
                    d_TOW_at_current_symbol_ms = static_cast<uint32_t>(
                        (static_cast<uint64_t>(d_TOW_at_current_symbol_ms) + 1U) % 604800000ULL);
                }
            else
                {
                    d_flag_valid_word = false;
                }
        }

    if (d_flag_valid_word)
        {
            current.Flag_valid_word = true;
            current.TOW_at_current_symbol_ms = d_TOW_at_current_symbol_ms;
            if (d_tow_to_trk)
                {
                    const auto tow_obj = std::make_shared<TOW_to_trk>(TOW_to_trk(
                        std::string("B2"),
                        d_channel,
                        d_TOW_at_current_symbol_ms,
                        current.Tracking_sample_counter,
                        d_nav.get_ephemeris().WN,
                        d_satellite.get_PRN()));
                    message_port_pub(pmt::mp("telemetry_to_trk"), pmt::make_any(tow_obj));
                }
        }

    *out = current;
    consume_each(1);
    return 1;
}
