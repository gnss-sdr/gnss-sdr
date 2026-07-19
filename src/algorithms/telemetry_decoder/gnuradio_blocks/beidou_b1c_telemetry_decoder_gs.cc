/*!
 * \file beidou_b1c_telemetry_decoder_gs.cc
 * \brief BeiDou B1C B-CNAV1 telemetry decoder block
 * \author Wenhao Ou, 2026. ouwh(at)mail2.sysu.edu.cn
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
#include "beidou_b1c_telemetry_decoder_gs.h"
#include "Beidou_B1C.h"
#include "Beidou_B1C_codes.h"
#include "Beidou_CNAV1.h"
#include "Beidou_DNAV.h"
#include "MATH_CONSTANTS.h"
#include "beidou_cnav1_ephemeris.h"
#include "beidou_cnav1_iono.h"
#include "beidou_cnav1_utc_model.h"
#include "display.h"
#include "gnss_sdr_make_unique.h"
#include "gnss_synchro.h"
#include "tlm_crc_stats.h"
#include "tlm_utils.h"
#include "tow_to_trk.h"
#include <pmt/pmt.h>
#include <pmt/pmt_sugar.h>
#include <algorithm>
#include <cmath>
#include <complex>
#include <iomanip>
#include <iostream>
#include <utility>
#include <vector>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

#define CRC_ERROR_LIMIT 8

namespace
{
// ICD §7.3: SOH marks subframe-1 start. Symbols from SOH to current = N - offset.
int32_t resolve_b1c_frame_soh_offset(int32_t matched_offset, int32_t prev_candidate_offset)
{
    if (matched_offset >= 0)
        {
            return matched_offset;
        }
    if (matched_offset == -2 && prev_candidate_offset >= 0)
        {
            return prev_candidate_offset;
        }
    return 0;
}


uint32_t compute_b1c_tow_ms_at_current_symbol(
    double tow_soh_bdt_s,
    int32_t frame_soh_offset,
    int32_t symbol_duration_ms)
{
    const int32_t clamped_offset = std::max(0, std::min(frame_soh_offset, BEIDOU_CNAV1_FRAME_SYMBOLS - 1));
    const int32_t symbols_since_soh = BEIDOU_CNAV1_FRAME_SYMBOLS - clamped_offset;
    const double tow_gpst_s = tow_soh_bdt_s + static_cast<double>(BEIDOU_DNAV_BDT2GPST_LEAP_SEC_OFFSET) +
                              static_cast<double>(symbols_since_soh) * static_cast<double>(symbol_duration_ms) / 1000.0;
    const double tow_ms = std::fmod(tow_gpst_s * 1000.0, 604800000.0);
    if (tow_ms < 0.0)
        {
            return static_cast<uint32_t>(tow_ms + 604800000.0);
        }
    return static_cast<uint32_t>(tow_ms);
}


void extract_prompt_window(const Gnss_Synchro* in, std::vector<float>& i_out, std::vector<float>& q_out)
{
    const int32_t n = BEIDOU_CNAV1_FRAME_SYMBOLS;
    i_out.resize(static_cast<size_t>(n));
    q_out.resize(static_cast<size_t>(n));
    for (int32_t i = 0; i < n; i++)
        {
            i_out[static_cast<size_t>(i)] = in[i].Prompt_I;
            q_out[static_cast<size_t>(i)] = in[i].Prompt_Q;
        }
}


bool build_rotated_bpsk_frame_from_iq(
    const std::vector<float>& history_i,
    const std::vector<float>& history_q,
    int32_t start_offset,
    bool invert,
    std::vector<float>& frame_out)
{
    if (history_i.size() < static_cast<size_t>(BEIDOU_CNAV1_FRAME_SYMBOLS) ||
        history_q.size() < static_cast<size_t>(BEIDOU_CNAV1_FRAME_SYMBOLS))
        {
            return false;
        }

    const int32_t n = BEIDOU_CNAV1_FRAME_SYMBOLS;
    std::vector<std::complex<float> > z(static_cast<size_t>(n));
    std::complex<float> sum_z2(0.0F, 0.0F);
    for (int32_t i = 0; i < n; i++)
        {
            const auto idx = static_cast<size_t>((start_offset + i) % n);
            const std::complex<float> zi(history_i[idx], history_q[idx]);
            z[static_cast<size_t>(i)] = zi;
            sum_z2 += zi * zi;
        }

    const float theta = 0.5F * std::arg(sum_z2);
    const std::complex<float> rot(std::cos(theta), -std::sin(theta));
    frame_out.resize(static_cast<size_t>(n));
    for (int32_t i = 0; i < n; i++)
        {
            float sample = (z[static_cast<size_t>(i)] * rot).real();
            if (invert)
                {
                    sample = -sample;
                }
            frame_out[static_cast<size_t>(i)] = sample;
        }
    return true;
}


bool apply_llr_scale_and_decode(
    std::vector<float>& frame,
    Beidou_Cnav1_Navigation_Message& nav,
    int32_t expected_prn,
    float cn0_db_hz,
    int32_t* fail_stage)
{
    const int32_t n = BEIDOU_CNAV1_FRAME_SYMBOLS;
    double rms_acc = 0.0;
    for (float sample : frame)
        {
            const auto v = static_cast<double>(sample);
            rms_acc += v * v;
        }
    const double rms = std::sqrt(rms_acc / static_cast<double>(n)) + 1e-6;
    const double cn0_linear = std::pow(10.0, static_cast<double>(cn0_db_hz) / 10.0);
    const double snr_sym = std::max(cn0_linear * (BEIDOU_B1C_CODE_PERIOD_MS * 1e-3), 1e-4);
    const double llr_scale = std::min(std::max(std::sqrt(snr_sym) / rms, 0.05), 50.0);
    for (float& sample : frame)
        {
            sample = static_cast<float>(static_cast<double>(sample) * llr_scale);
        }
    return nav.decode_frame_symbols(frame.data(), BEIDOU_CNAV1_FRAME_SYMBOLS, expected_prn, fail_stage);
}


bool probe_frame_from_iq_window(
    const std::vector<float>& history_i,
    const std::vector<float>& history_q,
    const Beidou_Cnav1_Navigation_Message& nav,
    int32_t expected_prn,
    int32_t start_offset,
    bool invert)
{
    std::vector<float> frame;
    if (!build_rotated_bpsk_frame_from_iq(history_i, history_q, start_offset, invert, frame))
        {
            return false;
        }
    return nav.probe_subframe1_prn(frame.data(), BEIDOU_CNAV1_FRAME_SYMBOLS, expected_prn);
}


bool decode_frame_from_iq_window(
    const std::vector<float>& history_i,
    const std::vector<float>& history_q,
    Beidou_Cnav1_Navigation_Message& nav,
    int32_t expected_prn,
    int32_t start_offset,
    bool invert,
    float cn0_db_hz,
    int32_t* fail_stage = nullptr)
{
    std::vector<float> frame;
    if (!build_rotated_bpsk_frame_from_iq(history_i, history_q, start_offset, invert, frame))
        {
            return false;
        }
    return apply_llr_scale_and_decode(frame, nav, expected_prn, cn0_db_hz, fail_stage);
}


std::pair<int32_t, std::vector<int32_t> > find_secondary_code_candidates(
    const std::vector<float>& pilot_symbols,
    int32_t prn)
{
    if (pilot_symbols.size() < static_cast<size_t>(BEIDOU_CNAV1_FRAME_SYMBOLS) ||
        prn < 1 || prn > static_cast<int32_t>(BEIDOU_B1C_NUMBER_OF_PRNS))
        {
            return std::make_pair(0, std::vector<int32_t>());
        }

    const char* sec_code = BEIDOU_B1C_PILOT_SECONDARY_CODE[static_cast<size_t>(prn - 1)];
    const int32_t n = BEIDOU_CNAV1_FRAME_SYMBOLS;
    int32_t best_corr = 0;
    int32_t best_lag = 0;
    std::vector<int32_t> candidates;
    for (int32_t lag = 0; lag < n; lag++)
        {
            int32_t corr = 0;
            for (int32_t i = 0; i < n; i++)
                {
                    const float sym = pilot_symbols[static_cast<size_t>((lag + i) % n)];
                    const int32_t sym_sign = (sym >= 0.0F) ? 1 : -1;
                    const int32_t sec_sign = (sec_code[static_cast<size_t>(i)] == '0') ? 1 : -1;
                    corr += sym_sign * sec_sign;
                }
            if (std::abs(corr) > std::abs(best_corr))
                {
                    best_corr = corr;
                    best_lag = lag;
                }
            if (std::abs(corr) >= 1799)
                {
                    candidates.push_back(lag);
                }
        }

    if (candidates.empty() && std::abs(best_corr) >= 1650)
        {
            candidates.push_back(best_lag);
        }
    return std::make_pair(best_corr, candidates);
}
}  // namespace


beidou_b1c_telemetry_decoder_gs_sptr beidou_b1c_make_telemetry_decoder_gs(
    const Gnss_Satellite& satellite,
    const Tlm_Conf& conf)
{
    return beidou_b1c_telemetry_decoder_gs_sptr(new beidou_b1c_telemetry_decoder_gs(satellite, conf));
}


beidou_b1c_telemetry_decoder_gs::beidou_b1c_telemetry_decoder_gs(
    const Gnss_Satellite& satellite,
    const Tlm_Conf& conf)
    : telemetry_impl_interface("beidou_b1c_telemetry_decoder_gs",
          gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
          gr::io_signature::make(1, 1, sizeof(Gnss_Synchro))),
      d_dump_filename(conf.dump_filename),

      d_symbol_duration_ms(BEIDOU_B1C_CODE_PERIOD_MS),
      d_dump(conf.dump),
      d_dump_mat(conf.dump_mat),
      d_remove_dat(conf.remove_dat),
      d_enable_navdata_monitor(conf.enable_navdata_monitor),
      d_dump_crc_stats(conf.dump_crc_stats),
      d_tow_to_trk(conf.tow_to_trk)
{
    set_history(static_cast<unsigned int>(BEIDOU_CNAV1_FRAME_SYMBOLS));
    set_output_multiple(1);
    configure_basic_outputs();
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << "Initializing BeiDou B1C B-CNAV1 telemetry decoder for satellite " << d_satellite;
    if (d_enable_navdata_monitor)
        {
            message_port_register_out(pmt::mp("Nav_msg_from_TLM"));
            d_nav_msg_packet.system = std::string("C");
            d_nav_msg_packet.signal = std::string("1D");
        }
    configure_dump_file(d_channel, d_dump, d_dump_filename, d_dump_file);
    configure_crc_stats_channel(d_channel, d_dump_crc_stats, d_Tlm_CRC_Stats);
}


beidou_b1c_telemetry_decoder_gs::~beidou_b1c_telemetry_decoder_gs()
{
    if (d_dump_file.is_open())
        {
            d_dump_file.close();
        }
}


void beidou_b1c_telemetry_decoder_gs::set_satellite(const Gnss_Satellite& satellite)
{
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << "BeiDou B1C telemetry decoder set to satellite " << d_satellite;
}


void beidou_b1c_telemetry_decoder_gs::set_channel(int channel)
{
    d_channel = channel;
    DLOG(INFO) << "BeiDou B1C B-CNAV1 telemetry channel set to " << channel;
    configure_dump_file(d_channel, d_dump, d_dump_filename, d_dump_file);
    configure_crc_stats_channel(d_channel, d_dump_crc_stats, d_Tlm_CRC_Stats);
}


void beidou_b1c_telemetry_decoder_gs::reset()
{
    d_sample_counter = 0;
    d_frame_sync_index = 0;
    d_TOW_at_current_symbol_ms = 0;
    d_flag_valid_word = false;
    d_flag_frame_sync = false;
    d_flag_PLL_180_deg_phase_locked = false;
    d_stat = 0;
    d_CRC_error_counter = 0;
    d_frame_soh_offset = 0;
    d_prev_candidate_offset = -1;
    d_prev_valid_symbol_output = false;
    d_await_post_lock_frame_decode = false;
    d_post_lock_valid_symbols = 0U;
    d_nav.clear_flags();
    DLOG(INFO) << "BeiDou B1C telemetry decoder reset for satellite " << d_satellite;
}


void beidou_b1c_telemetry_decoder_gs::forecast(int noutput_items, gr_vector_int& ninput_items_required)
{
    ninput_items_required[0] = BEIDOU_CNAV1_FRAME_SYMBOLS + std::max(noutput_items, 1) - 1;
}


bool beidou_b1c_telemetry_decoder_gs::decode_frame_from_window(
    const Gnss_Synchro* window,
    int32_t start_offset,
    bool invert)
{
    std::vector<float> frame(static_cast<size_t>(BEIDOU_CNAV1_FRAME_SYMBOLS));
    const int32_t n = BEIDOU_CNAV1_FRAME_SYMBOLS;
    const int32_t offset = ((start_offset % n) + n) % n;
    for (int32_t i = 0; i < n; i++)
        {
            float sample = window[(offset + i) % n].Prompt_I;
            if (invert)
                {
                    sample = -sample;
                }
            frame[static_cast<size_t>(i)] = sample;
        }
    return d_nav.decode_frame_symbols(frame.data(), BEIDOU_CNAV1_FRAME_SYMBOLS, d_satellite.get_PRN());
}


void beidou_b1c_telemetry_decoder_gs::publish_navigation(double cn0_db_hz)
{
    if (d_nav.have_new_ephemeris())
        {
            auto eph = std::make_shared<Beidou_Cnav1_Ephemeris>(d_nav.get_ephemeris());
            eph->PRN = d_satellite.get_PRN();
            message_port_pub(pmt::mp("telemetry"), pmt::make_any(eph));
#if __cplusplus == 201103L
            const int default_precision = std::cout.precision();
#else
            const auto default_precision{std::cout.precision()};
#endif
            std::cout << TEXT_BLUE << "New BeiDou B-CNAV1 ephemeris in channel " << d_channel
                      << " from satellite " << d_satellite
                      << " with CN0=" << std::setprecision(2) << cn0_db_hz << std::setprecision(default_precision)
                      << " dB-Hz" << TEXT_RESET << std::endl;
            LOG(INFO) << "New BeiDou B-CNAV1 ephemeris from PRN " << d_satellite.get_PRN()
                      << " in channel " << d_channel << " with CN0=" << cn0_db_hz << " dB-Hz";
        }
    if (d_nav.have_new_iono())
        {
            auto iono = std::make_shared<Beidou_Cnav1_Iono>(d_nav.get_iono());
            iono->valid = true;
            message_port_pub(pmt::mp("telemetry"), pmt::make_any(iono));
            DLOG(INFO) << "New BeiDou B-CNAV1 iono model from PRN " << d_satellite.get_PRN()
                       << " in channel " << d_channel;
        }
    if (d_nav.have_new_utc_model())
        {
            auto utc = std::make_shared<Beidou_Cnav1_Utc_Model>(d_nav.get_utc_model());
            utc->valid = true;
            message_port_pub(pmt::mp("telemetry"), pmt::make_any(utc));
            DLOG(INFO) << "New BeiDou B-CNAV1 UTC model from PRN " << d_satellite.get_PRN()
                       << " in channel " << d_channel;
        }
    if (d_nav.have_new_page_data())
        {
            Beidou_Cnav1_PageData_Message page_msg{};
            page_msg.PRN = d_satellite.get_PRN();
            page_msg.page_data = d_nav.get_page_data();
            message_port_pub(pmt::mp("telemetry"), pmt::make_any(std::make_shared<Beidou_Cnav1_PageData_Message>(page_msg)));
            DLOG(INFO) << "New BeiDou B-CNAV1 page data from PRN " << d_satellite.get_PRN()
                       << " in channel " << d_channel
                       << " PageID=" << d_nav.get_page_data().common.page_id;
        }
    d_nav.clear_flags();
    d_prev_candidate_offset = -1;
}


int beidou_b1c_telemetry_decoder_gs::general_work(
    int noutput_items,
    gr_vector_int& ninput_items,
    gr_vector_const_void_star& input_items,
    gr_vector_void_star& output_items)
{
    const auto* in = reinterpret_cast<const Gnss_Synchro*>(input_items[0]);
    auto* out = reinterpret_cast<Gnss_Synchro*>(output_items[0]);
    if (ninput_items[0] < BEIDOU_CNAV1_FRAME_SYMBOLS || noutput_items <= 0)
        {
            return 0;
        }

    const int32_t hist = BEIDOU_CNAV1_FRAME_SYMBOLS;
    const Gnss_Synchro* frame_window = in;
    Gnss_Synchro current_symbol = in[hist - 1];
    current_symbol.PRN = d_satellite.get_PRN();
    d_sample_counter++;

    const bool just_locked_event = current_symbol.Flag_valid_symbol_output && !d_prev_valid_symbol_output;
    d_prev_valid_symbol_output = current_symbol.Flag_valid_symbol_output;

    if (current_symbol.Flag_valid_symbol_output && d_await_post_lock_frame_decode && !just_locked_event)
        {
            d_post_lock_valid_symbols++;
        }

    // Decode on lock (or when 18 s history is ready), then only on frame boundaries.
    const bool at_frame_boundary =
        d_flag_frame_sync &&
        (d_sample_counter > d_frame_sync_index) &&
        ((d_sample_counter - d_frame_sync_index) % static_cast<uint64_t>(BEIDOU_CNAV1_FRAME_SYMBOLS) == 0U);
    const bool post_lock_frame_ready =
        d_await_post_lock_frame_decode &&
        d_post_lock_valid_symbols >= static_cast<uint32_t>(BEIDOU_CNAV1_FRAME_SYMBOLS);
    bool frame_decoded = false;

    const bool history_ready = (d_sample_counter >= static_cast<uint64_t>(BEIDOU_CNAV1_FRAME_SYMBOLS));
    if (history_ready)
        {
            const bool should_try_decode =
                current_symbol.Flag_valid_symbol_output &&
                ((d_stat == 0 && just_locked_event) ||
                    (d_stat == 0 && post_lock_frame_ready && !just_locked_event) ||
                    (d_stat == 2 && at_frame_boundary &&
                        d_sample_counter >= d_frame_sync_index + static_cast<uint64_t>(BEIDOU_CNAV1_FRAME_SYMBOLS)));

            if (should_try_decode)
                {
                    bool invert = false;
                    int32_t matched_offset = -1;
                    if (d_stat == 0 && just_locked_event)
                        {
                            const int32_t expected_prn_scan = d_satellite.get_PRN();
                            std::vector<float> history_i;
                            std::vector<float> history_q;
                            extract_prompt_window(frame_window, history_i, history_q);
                            const auto sec_candidates =
                                find_secondary_code_candidates(history_q, expected_prn_scan);
                            const auto& candidate_offsets = sec_candidates.second;
                            constexpr int32_t local_offset_span = 32;
                            const auto cn0_db_hz = static_cast<float>(current_symbol.CN0_dB_hz);

                            if (!candidate_offsets.empty())
                                {
                                    for (int32_t candidate_offset : candidate_offsets)
                                        {
                                            for (int32_t delta = -local_offset_span; delta <= local_offset_span; delta++)
                                                {
                                                    const int32_t start_offset =
                                                        (candidate_offset + delta + BEIDOU_CNAV1_FRAME_SYMBOLS) % BEIDOU_CNAV1_FRAME_SYMBOLS;
                                                    for (int inv_i = 0; inv_i < 2; ++inv_i)
                                                        {
                                                            const bool inv = (inv_i != 0);
                                                            if (!probe_frame_from_iq_window(
                                                                    history_i, history_q, d_nav, expected_prn_scan, start_offset, inv))
                                                                {
                                                                    continue;
                                                                }
                                                            int32_t fail_stage = -1;
                                                            if (decode_frame_from_iq_window(
                                                                    history_i, history_q, d_nav, expected_prn_scan, start_offset, inv,
                                                                    cn0_db_hz, &fail_stage))
                                                                {
                                                                    frame_decoded = true;
                                                                    invert = inv;
                                                                    matched_offset = start_offset;
                                                                    break;
                                                                }
                                                        }
                                                    if (frame_decoded)
                                                        {
                                                            break;
                                                        }
                                                }
                                            if (frame_decoded)
                                                {
                                                    break;
                                                }
                                        }
                                }

                            if (frame_decoded)
                                {
                                    d_await_post_lock_frame_decode = false;
                                    d_post_lock_valid_symbols = 0U;
                                }
                            else
                                {
                                    d_await_post_lock_frame_decode = true;
                                    d_post_lock_valid_symbols = 1U;
                                }
                        }
                    else if (d_stat == 0 && post_lock_frame_ready)
                        {
                            frame_decoded = decode_frame_from_window(frame_window, 0, false);
                            if (!frame_decoded)
                                {
                                    frame_decoded = decode_frame_from_window(frame_window, 0, true);
                                }
                            if (frame_decoded)
                                {
                                    matched_offset = 0;
                                    d_await_post_lock_frame_decode = false;
                                    d_post_lock_valid_symbols = 0U;
                                }
                        }
                    else
                        {
                            frame_decoded = decode_frame_from_window(frame_window, d_frame_soh_offset, false);
                            if (!frame_decoded)
                                {
                                    frame_decoded = decode_frame_from_window(frame_window, d_frame_soh_offset, true);
                                    invert = frame_decoded;
                                }
                        }

                    if (frame_decoded)
                        {
                            d_CRC_error_counter = 0;
                            d_flag_PLL_180_deg_phase_locked = invert;
                            d_frame_sync_index = d_sample_counter;
                            const int32_t frame_soh_offset = resolve_b1c_frame_soh_offset(matched_offset, d_prev_candidate_offset);
                            d_TOW_at_current_symbol_ms = compute_b1c_tow_ms_at_current_symbol(
                                d_nav.get_tow_s(), frame_soh_offset, d_symbol_duration_ms);
                            d_flag_valid_word = true;
                            publish_navigation(current_symbol.CN0_dB_hz);

                            if (!d_flag_frame_sync)
                                {
                                    d_flag_frame_sync = true;
                                    d_stat = 2;
                                    d_frame_soh_offset = (matched_offset >= 0) ? matched_offset : 0;
                                    LOG(INFO) << "Successful frame synchronization in channel " << d_channel
                                              << " for satellite " << d_satellite
                                              << " with CN0=" << current_symbol.CN0_dB_hz << " dB-Hz";
#if __cplusplus == 201103L
                                    const int default_precision = std::cout.precision();
#else
                                    const auto default_precision{std::cout.precision()};
#endif
                                    std::cout << TEXT_GREEN << "B-CNAV1 frame sync on channel " << d_channel
                                              << ", satellite " << d_satellite
                                              << ", TOW=" << std::setprecision(3) << d_nav.get_tow_s()
                                              << std::setprecision(default_precision) << " s" << TEXT_RESET << std::endl;
                                }
                        }
                    else if (d_stat == 2)
                        {
                            d_CRC_error_counter++;
                            if (d_CRC_error_counter > CRC_ERROR_LIMIT)
                                {
                                    DLOG(INFO) << "B-CNAV1 frame sync lost for satellite " << d_satellite
                                               << " in channel " << d_channel;
                                    d_flag_frame_sync = false;
                                    d_flag_valid_word = false;
                                    d_frame_soh_offset = 0;
                                    d_stat = 0;
                                    d_CRC_error_counter = 0;
                                }
                        }
                }
        }

    if (d_flag_valid_word && !frame_decoded)
        {
            d_TOW_at_current_symbol_ms += d_symbol_duration_ms;
            if (!current_symbol.Flag_valid_symbol_output)
                {
                    d_flag_valid_word = false;
                }
        }

    if (d_flag_PLL_180_deg_phase_locked)
        {
            current_symbol.Carrier_phase_rads += GNSS_PI;
            current_symbol.Flag_PLL_180_deg_phase_locked = true;
        }
    else
        {
            current_symbol.Flag_PLL_180_deg_phase_locked = false;
        }

    consume_each(1);
    if (d_flag_frame_sync && d_frame_soh_offset > 0)
        {
            d_frame_soh_offset--;
        }

    if (d_flag_valid_word)
        {
            current_symbol.TOW_at_current_symbol_ms = d_TOW_at_current_symbol_ms;
            current_symbol.Flag_valid_word = true;
            if (d_tow_to_trk)
                {
                    const auto tow_obj = std::make_shared<TOW_to_trk>(TOW_to_trk(
                        std::string(current_symbol.Signal),
                        d_channel,
                        d_TOW_at_current_symbol_ms,
                        current_symbol.Tracking_sample_counter,
                        d_nav.get_ephemeris().WN,
                        d_satellite.get_PRN()));
                    message_port_pub(pmt::mp("telemetry_to_trk"), pmt::make_any(tow_obj));
                }
            out[0] = std::move(current_symbol);
            return 1;
        }
    return 0;
}
