/*!
 * \file beidou_b2b_telemetry_decoder_gs.h
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
#ifndef GNSS_SDR_BEIDOU_B2B_TELEMETRY_DECODER_GS_H
#define GNSS_SDR_BEIDOU_B2B_TELEMETRY_DECODER_GS_H

#include "Beidou_CNAV3.h"
#include "beidou_cnav3_navigation_message.h"
#include "nav_message_packet.h"
#include "telemetry_impl_interface.h"
#include "tlm_conf.h"
#include <deque>
#include <vector>

class Gnss_Satellite;
class beidou_b2b_telemetry_decoder_gs;
using beidou_b2b_telemetry_decoder_gs_sptr = gnss_shared_ptr<beidou_b2b_telemetry_decoder_gs>;

beidou_b2b_telemetry_decoder_gs_sptr beidou_b2b_make_telemetry_decoder_gs(
    const Gnss_Satellite& satellite,
    const Tlm_Conf& conf);

class beidou_b2b_telemetry_decoder_gs : public telemetry_impl_interface
{
public:
    ~beidou_b2b_telemetry_decoder_gs() override;
    void set_satellite(const Gnss_Satellite& satellite) override;
    void set_channel(int channel) override;
    void reset() override;
    void forecast(int noutput_items, gr_vector_int& ninput_items_required) override;
    int general_work(int noutput_items, gr_vector_int& ninput_items,
        gr_vector_const_void_star& input_items, gr_vector_void_star& output_items) override;

private:
    friend beidou_b2b_telemetry_decoder_gs_sptr beidou_b2b_make_telemetry_decoder_gs(
        const Gnss_Satellite& satellite, const Tlm_Conf& conf);
    beidou_b2b_telemetry_decoder_gs(const Gnss_Satellite& satellite, const Tlm_Conf& conf);
    void publish_navigation(double cn0_db_hz);
    bool try_decode_frame();

    Beidou_Cnav3_Navigation_Message d_nav;
    Nav_Message_Packet d_nav_msg_packet;
    Gnss_Satellite d_satellite;
    std::deque<float> d_symbol_history;
    std::string d_dump_filename;
    std::ofstream d_dump_file;
    std::unique_ptr<Tlm_CRC_Stats> d_Tlm_CRC_Stats;
    uint64_t d_sample_counter{};
    uint64_t d_preamble_index{};
    int32_t d_channel{};
    int32_t d_CRC_error_counter{};
    uint32_t d_TOW_at_current_symbol_ms{};
    bool d_flag_frame_sync{false};
    bool d_flag_valid_word{false};
    bool d_dump{false};
    bool d_dump_mat{false};
    bool d_remove_dat{false};
    bool d_enable_navdata_monitor{false};
    bool d_dump_crc_stats{false};
    bool d_tow_to_trk{false};
};

#endif  // GNSS_SDR_BEIDOU_B2B_TELEMETRY_DECODER_GS_H
