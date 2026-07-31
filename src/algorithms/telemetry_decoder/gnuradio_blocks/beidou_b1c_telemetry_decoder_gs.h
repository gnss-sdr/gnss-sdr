/*!
 * \file beidou_b1c_telemetry_decoder_gs.h
 * \brief BeiDou B1C B-CNAV1 telemetry decoder block
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef GNSS_SDR_BEIDOU_B1C_TELEMETRY_DECODER_GS_H
#define GNSS_SDR_BEIDOU_B1C_TELEMETRY_DECODER_GS_H

#include "beidou_cnav1_navigation_message.h"
#include "nav_message_packet.h"
#include "telemetry_impl_interface.h"
#include "tlm_conf.h"
#include <boost/circular_buffer.hpp>
#include <vector>

class beidou_b1c_telemetry_decoder_gs;
using beidou_b1c_telemetry_decoder_gs_sptr = gnss_shared_ptr<beidou_b1c_telemetry_decoder_gs>;

beidou_b1c_telemetry_decoder_gs_sptr beidou_b1c_make_telemetry_decoder_gs(
    const Gnss_Satellite& satellite,
    const Tlm_Conf& conf);

class beidou_b1c_telemetry_decoder_gs : public telemetry_impl_interface
{
public:
    ~beidou_b1c_telemetry_decoder_gs() override;
    void set_satellite(const Gnss_Satellite& satellite) override;
    void set_channel(int channel) override;
    void reset() override;
    int general_work(int noutput_items, gr_vector_int& ninput_items,
        gr_vector_const_void_star& input_items, gr_vector_void_star& output_items) override;

private:
    friend beidou_b1c_telemetry_decoder_gs_sptr beidou_b1c_make_telemetry_decoder_gs(
        const Gnss_Satellite& satellite, const Tlm_Conf& conf);

    beidou_b1c_telemetry_decoder_gs(const Gnss_Satellite& satellite, const Tlm_Conf& conf);
    bool decode_frame_from_history(bool invert);
    bool decode_frame_from_history_with_offset(int32_t start_offset, bool invert);
    bool probe_frame_from_history_with_offset(int32_t start_offset, bool invert) const;
    void publish_navigation(double cn0_db_hz);

    Beidou_Cnav1_Navigation_Message d_nav;
    Nav_Message_Packet d_nav_msg_packet;
    Gnss_Satellite d_satellite;
    boost::circular_buffer<float> d_symbol_history;
    boost::circular_buffer<float> d_symbol_history_q;
    std::string d_dump_filename;
    std::ofstream d_dump_file;
    std::unique_ptr<Tlm_CRC_Stats> d_Tlm_CRC_Stats;
    uint64_t d_sample_counter{};
    uint64_t d_frame_sync_index{};
    int32_t d_channel{};
    int32_t d_stat{};
    int32_t d_CRC_error_counter{};
    uint32_t d_TOW_at_current_symbol_ms{};
    uint32_t d_symbol_duration_ms{};
    bool d_flag_valid_word{false};
    bool d_flag_frame_sync{false};
    bool d_flag_PLL_180_deg_phase_locked{false};
    bool d_dump{false};
    bool d_dump_mat{false};
    bool d_remove_dat{false};
    bool d_enable_navdata_monitor{false};
    bool d_dump_crc_stats{false};
    bool d_tow_to_trk{false};
    bool d_prev_valid_symbol_output{false};
    bool d_await_post_lock_frame_decode{false};
    uint32_t d_post_lock_valid_symbols{0U};
    std::vector<float> d_prev_candidate_frame;
    int32_t d_prev_candidate_offset{-1};
    bool d_have_prev_candidate_frame{false};
};

#endif
