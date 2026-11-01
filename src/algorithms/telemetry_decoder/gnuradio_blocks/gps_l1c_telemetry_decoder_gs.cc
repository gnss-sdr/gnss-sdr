/*!
 * \file gps_l1c_telemetry_decoder_gs.cc
 * \brief GNU Radio block that decodes the GPS L1C CNAV-2 navigation message
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 * \author José Antonio Mayo, 2026. contact(at)tatjam.eu
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

#include "gps_l1c_telemetry_decoder_gs.h"
#include "GPS_L1C.h"
#include "MATH_CONSTANTS.h"
#include "deterministic_clock.h"
#include "display.h"  // for colours in terminal: TEXT_RED, TEXT_RESET
#include "dump_logger_helper.h"
#include "gnss_sdr_make_unique.h"  // for std::make_unique in C++11
#include "gnss_synchro.h"
#include "telemetry_message_stamp.h"
#include "tlm_conf.h"
#include "tlm_crc_stats.h"
#include "tlm_utils.h"
#include "tow_to_trk.h"
#include "tow_utils.h"
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>        // for make_any
#include <pmt/pmt_sugar.h>  // for mp
#include <iomanip>          // for std::setprecision
#include <iostream>         // for std::cout
#include <mutex>            // for std::mutex, std::lock_guard
#include <set>              // for std::set
#include <sstream>          // for std::ostringstream
#include <utility>          // for std::move

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

namespace
{
// Consecutive frames without a passing CRC nor the expected TOI before the
// frame synchronization is dropped
constexpr int32_t MAX_FRAME_ERRORS = 3;

// Frames without any valid one before telling the tracking block that the
// telemetry fails
constexpr uint64_t MAX_SYMBOLS_WITHOUT_VALID_FRAME = 5 * GPS_L1C_FRAME_SYMBOLS;

// Consecutive frames with subframe 2 but without time of week before telling
// the user. A compliant signal has TOI = 0, which does not give the time of
// week, in a single frame every two hours (IS-GPS-800J, 3.5.2)
constexpr int32_t MAX_FRAMES_WITHOUT_TOW = 3;

// True the first time it is called for a given PRN, so that the user is told
// once per satellite and receiver run, whatever the channel
bool first_unusable_notice(uint32_t prn)
{
    static std::mutex mutex;
    static std::set<uint32_t> notified;
    const std::lock_guard<std::mutex> lock(mutex);
    return notified.insert(prn).second;
}
}  // namespace


gps_l1c_telemetry_decoder_gs_sptr gps_l1c_make_telemetry_decoder_gs(const Tlm_Conf &conf)
{
    return gps_l1c_telemetry_decoder_gs_sptr(new gps_l1c_telemetry_decoder_gs(conf));
}


gps_l1c_telemetry_decoder_gs::gps_l1c_telemetry_decoder_gs(const Tlm_Conf &conf)
    : telemetry_impl_interface("gps_l1c_telemetry_decoder_gs",
          gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
          gr::io_signature::make(1, 1, sizeof(Gnss_Synchro))),
      d_symbol_history(GPS_L1C_FRAME_SYMBOLS),
      d_frame(GPS_L1C_FRAME_SYMBOLS),
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
            // register nav message monitor out
            this->message_port_register_out(pmt::mp("Nav_msg_from_TLM"));
            d_nav_msg_packet.system = std::string("G");
            d_nav_msg_packet.signal = std::string("1S");
        }

    if (d_dump_crc_stats)
        {
            // initialize the telemetry CRC statistics class
            d_Tlm_CRC_Stats = std::make_unique<Tlm_CRC_Stats>();
            d_Tlm_CRC_Stats->initialize(conf.dump_crc_stats_filename);
        }
    else
        {
            d_Tlm_CRC_Stats = nullptr;
        }
}


gps_l1c_telemetry_decoder_gs::~gps_l1c_telemetry_decoder_gs()
{
    DLOG(INFO) << "GPS L1C Telemetry decoder block (channel " << d_channel << ") destructor called.";
    tlm_cleanup_and_save_files(d_dump_file, d_dump_filename, d_dump, d_dump_mat, d_remove_dat);
}


void gps_l1c_telemetry_decoder_gs::set_satellite(const Gnss_Satellite &satellite)
{
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    d_nav.set_prn(d_satellite.get_PRN());
    d_nav.reset();
    DLOG(INFO) << "GPS L1C CNAV-2 telemetry decoder in channel " << d_channel << " set to satellite " << d_satellite;
}


void gps_l1c_telemetry_decoder_gs::set_channel(int32_t channel)
{
    d_channel = channel;
    DLOG(INFO) << "GPS L1C CNAV-2 channel set to " << channel;
    configure_dump_file(d_channel, d_dump, d_dump_filename, d_dump_file);
    configure_crc_stats_channel(d_channel, d_dump_crc_stats, d_Tlm_CRC_Stats);
}


void gps_l1c_telemetry_decoder_gs::reset()
{
    clear_sync();
    d_last_valid_frame = d_sample_counter;
    d_sent_tlm_failed_msg = false;
    DLOG(INFO) << "Telemetry decoder reset for satellite " << d_satellite;
}


void gps_l1c_telemetry_decoder_gs::clear_sync()
{
    d_symbol_history.clear();
    d_symbols_in_frame = 0;
    d_expected_toi = -1;
    d_frame_errors = 0;
    d_TOW_at_current_symbol_ms = 0;
    d_TOW_at_frame_start_ms = 0;
    d_frames_without_tow = 0;
    d_flag_frame_sync = false;
    d_flag_valid_word = false;
}


bool gps_l1c_telemetry_decoder_gs::decode_last_frame(bool inverted)
{
    const float sign = inverted ? -1.0F : 1.0F;
    for (int32_t i = 0; i < GPS_L1C_FRAME_SYMBOLS; i++)
        {
            d_frame[i] = sign * d_symbol_history[i];
        }
    return d_nav.decode_frame(d_frame.data());
}


bool gps_l1c_telemetry_decoder_gs::search_frame(bool tracking_inverted)
{
    // The TOI correlation magnitude does not depend on the polarity
    for (int32_t i = 0; i < GPS_L1C_TOI_SYMBOLS; i++)
        {
            d_frame[i] = d_symbol_history[i];
        }
    float toi_quality = 0.0F;
    Gps_CNAV2_Navigation_Message::decode_toi(d_frame.data(), toi_quality);
    if (toi_quality < GPS_L1C_TOI_MIN_QUALITY)
        {
            return false;
        }
    // Try first the polarity that the pilot suggests, if any
    for (const bool inverted : {tracking_inverted, !tracking_inverted})
        {
            const float sign = inverted ? -1.0F : 1.0F;
            for (int32_t i = 0; i < GPS_L1C_FRAME_SYMBOLS; i++)
                {
                    d_frame[i] = sign * d_symbol_history[i];
                }
            if (d_nav.is_frame_candidate(d_frame.data()) && d_nav.decode_frame(d_frame.data()) && d_nav.get_toi() <= GPS_L1C_TOI_MAX)
                {
                    d_inverted = inverted;
                    return true;
                }
        }
    return false;
}


void gps_l1c_telemetry_decoder_gs::update_time_of_week(bool in_sync)
{
    // The current symbol is the last one of the decoded frame, and the time
    // tag of a symbol refers to its end, that is, to the start of the next frame
    const int32_t frame_end_tow_s = d_nav.get_frame_end_tow();
    if (frame_end_tow_s < 0)
        {
            return;
        }
    const uint32_t decoded_tow_ms = gnss_tow::seconds_to_ms(frame_end_tow_s);
    if (in_sync && d_flag_valid_word && d_TOW_at_current_symbol_ms != decoded_tow_ms)
        {
            LOG(WARNING) << "GPS L1C TOW in channel " << d_channel << " for satellite " << d_satellite
                         << " jumped from " << d_TOW_at_current_symbol_ms << " ms to " << decoded_tow_ms << " ms";
        }
    if (!d_flag_valid_word)
        {
            LOG(INFO) << "GPS L1C TOW set in channel " << d_channel << " for satellite " << d_satellite
                      << ": " << frame_end_tow_s << " s";
        }
    d_TOW_at_current_symbol_ms = decoded_tow_ms;
    d_flag_valid_word = true;
}


void gps_l1c_telemetry_decoder_gs::check_navigation_usability()
{
    if (!d_nav.sf2_crc_ok())
        {
            return;
        }
    std::string reason;
    if (d_nav.sf2_has_no_data())
        {
            reason = "carries no navigation data (default message or all zeros)";
        }
    else if (d_nav.get_toi_quality() < GPS_L1C_TOI_MIN_QUALITY)
        {
            return;
        }
    else if (d_nav.get_frame_end_tow() >= 0)
        {
            d_frames_without_tow = 0;
        }
    else if (++d_frames_without_tow >= MAX_FRAMES_WITHOUT_TOW)
        {
            reason = "gives no time of week (TOI = " + std::to_string(d_nav.get_toi()) +
                     ", ITOW = " + std::to_string(d_nav.get_ephemeris().ITOW) + " in " +
                     std::to_string(d_frames_without_tow) + " consecutive frames)";
        }
    if (!reason.empty() && first_unusable_notice(d_satellite.get_PRN()))
        {
            std::ostringstream text;
            text << "The GPS L1C CNAV-2 message from satellite " << d_satellite << " in channel " << d_channel
                 << " " << reason << ": its L1C observations cannot be used for PVT";
            LOG(WARNING) << text.str();
            std::cout << TEXT_RED << text.str() << TEXT_RESET << '\n';
        }
}


void gps_l1c_telemetry_decoder_gs::publish_navigation(const Gnss_Synchro &symbol)
{
    if (d_enable_navdata_monitor)
        {
            // one packet per decoded subframe
            d_nav_msg_packet.prn = static_cast<int32_t>(d_satellite.get_PRN());
            d_nav_msg_packet.tow_at_current_symbol_ms = static_cast<int32_t>(d_TOW_at_current_symbol_ms);
            if (d_nav.sf2_crc_ok())
                {
                    d_nav_msg_packet.nav_message = d_nav.get_sf2_bits();
                    this->message_port_pub(pmt::mp("Nav_msg_from_TLM"), pmt::make_any(std::make_shared<Nav_Message_Packet>(d_nav_msg_packet)));
                }
            if (d_nav.sf3_crc_ok())
                {
                    d_nav_msg_packet.nav_message = d_nav.get_sf3_bits();
                    this->message_port_pub(pmt::mp("Nav_msg_from_TLM"), pmt::make_any(std::make_shared<Nav_Message_Packet>(d_nav_msg_packet)));
                }
            d_nav_msg_packet.nav_message.clear();
        }

    const auto default_precision = std::cout.precision();
    if (d_nav.have_new_ephemeris())
        {
            const auto tmp_obj = std::make_shared<Gps_CNAV2_Ephemeris>(d_nav.get_ephemeris());
            this->message_port_pub(pmt::mp("telemetry"), stamp_telemetry_message(pmt::make_any(tmp_obj), d_channel));
            std::cout << "New GPS L1C CNAV-2 message received in channel " << d_channel
                      << ": ephemeris from satellite " << d_satellite
                      << " with CN0=" << std::setprecision(2) << symbol.CN0_dB_hz
                      << std::setprecision(default_precision) << " dB-Hz" << std::endl;
        }
    if (d_nav.have_new_iono())
        {
            const auto tmp_obj = std::make_shared<Gps_CNAV_Iono>(d_nav.get_iono());
            this->message_port_pub(pmt::mp("telemetry"), stamp_telemetry_message(pmt::make_any(tmp_obj), d_channel));
            std::cout << "New GPS L1C CNAV-2 message received in channel " << d_channel
                      << ": iono model parameters from satellite " << d_satellite
                      << " with CN0=" << std::setprecision(2) << symbol.CN0_dB_hz
                      << std::setprecision(default_precision) << " dB-Hz" << std::endl;
        }
    if (d_nav.have_new_utc_model())
        {
            const auto tmp_obj = std::make_shared<Gps_CNAV_Utc_Model>(d_nav.get_utc_model());
            this->message_port_pub(pmt::mp("telemetry"), stamp_telemetry_message(pmt::make_any(tmp_obj), d_channel));
            std::cout << "New GPS L1C CNAV-2 message received in channel " << d_channel
                      << ": UTC model parameters from satellite " << d_satellite
                      << " with CN0=" << std::setprecision(2) << symbol.CN0_dB_hz
                      << std::setprecision(default_precision) << " dB-Hz" << std::endl;
        }
    if (d_nav.have_new_eop())
        {
            const auto tmp_obj = std::make_shared<Gps_CNAV_Eop>(d_nav.get_eop());
            this->message_port_pub(pmt::mp("telemetry"), stamp_telemetry_message(pmt::make_any(tmp_obj), d_channel));
        }
}


int gps_l1c_telemetry_decoder_gs::general_work(int noutput_items __attribute__((unused)),
    gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    auto *out = reinterpret_cast<Gnss_Synchro *>(output_items[0]);
    const auto *in = reinterpret_cast<const Gnss_Synchro *>(input_items[0]);

    Gnss_Synchro current_symbol = in[0];
    // Navigation data sent to the PVT is stamped with this symbol
    const TelemetryWorkScope symbol_stamp(this, current_symbol.Tracking_sample_counter);
    consume_each(1);
    d_sample_counter++;

    // check if there is a problem with the telemetry of the current satellite
    if (!d_sent_tlm_failed_msg && (d_sample_counter - d_last_valid_frame) > MAX_SYMBOLS_WITHOUT_VALID_FRAME)
        {
            const int message = 1;  // bad telemetry
            if (DeterministicClock::instance().enabled())
                {
                    // Record before releasing progress; tracking reads this in its work call
                    DeterministicClock::instance().report_telemetry_fault(d_channel, current_symbol.Tracking_sample_counter);
                }
            else
                {
                    this->message_port_pub(pmt::mp("telemetry_to_trk"), pmt::make_any(message));
                }
            d_sent_tlm_failed_msg = true;
        }

    if (!current_symbol.Flag_valid_symbol_output)
        {
            // loss of lock: the symbol stream restarts at an unknown position
            clear_sync();
        }
    else
        {
            d_symbol_history.push_back(static_cast<float>(current_symbol.Prompt_I));
            if (d_flag_valid_word)
                {
                    d_TOW_at_current_symbol_ms = gnss_tow::add_ms(d_TOW_at_current_symbol_ms, GPS_L1C_CODE_PERIOD_MS);
                }
            if (d_flag_frame_sync)
                {
                    d_symbols_in_frame++;
                    if (d_symbols_in_frame == GPS_L1C_FRAME_SYMBOLS)
                        {
                            d_symbols_in_frame = 0;
                            bool crc_ok = decode_last_frame(d_inverted);
                            int32_t toi = d_nav.get_toi();
                            bool toi_reliable = d_nav.get_toi_quality() >= GPS_L1C_TOI_MIN_QUALITY;
                            if (!crc_ok && decode_last_frame(!d_inverted))
                                {
                                    // half-cycle slip of the carrier loop
                                    LOG(INFO) << "GPS L1C symbol polarity changed in channel " << d_channel
                                              << " for satellite " << d_satellite;
                                    d_inverted = !d_inverted;
                                    crc_ok = true;
                                    toi = d_nav.get_toi();
                                    toi_reliable = d_nav.get_toi_quality() >= GPS_L1C_TOI_MIN_QUALITY;
                                }
                            if (d_dump_crc_stats)
                                {
                                    d_Tlm_CRC_Stats->update_CRC_stats(d_nav.sf2_crc_ok());
                                    d_Tlm_CRC_Stats->update_CRC_stats(d_nav.sf3_crc_ok());
                                }
                            LOG(INFO) << "GPS L1C frame in channel " << d_channel << " for satellite " << d_satellite
                                      << ": TOI=" << toi << " (expected " << d_expected_toi << ")"
                                      << ", subframe 2 " << (d_nav.sf2_crc_ok() ? (d_nav.sf2_has_no_data() ? "OK, no data" : "OK") : "failed")
                                      << ", subframe 3 " << (d_nav.sf3_crc_ok() ? (d_nav.sf3_has_no_data() ? std::string("OK, no data") : "OK, page " + std::to_string(d_nav.get_sf3_page())) : std::string("failed"));
                            const bool toi_ok = toi_reliable && toi == d_expected_toi;
                            if (crc_ok || toi_ok)
                                {
                                    d_frame_errors = 0;
                                    d_last_valid_frame = d_sample_counter;
                                    // The subframe CRCs do not protect TOI. An unreliable
                                    // TOI must neither replace the symbol-derived time
                                    // nor initialize TOW after subframe-3-only sync.
                                    if (crc_ok && toi_reliable && toi <= GPS_L1C_TOI_MAX)
                                        {
                                            d_expected_toi = toi;
                                            update_time_of_week(true);
                                        }
                                    d_expected_toi = (d_expected_toi + 1) % (GPS_L1C_TOI_MAX + 1);
                                    publish_navigation(current_symbol);
                                    check_navigation_usability();
                                }
                            else
                                {
                                    d_frame_errors++;
                                    d_expected_toi = (d_expected_toi + 1) % (GPS_L1C_TOI_MAX + 1);
                                    if (d_frame_errors > MAX_FRAME_ERRORS)
                                        {
                                            LOG(INFO) << "GPS L1C frame synchronization lost in channel " << d_channel
                                                      << " for satellite " << d_satellite;
                                            clear_sync();
                                        }
                                }
                            if (d_flag_valid_word)
                                {
                                    d_TOW_at_frame_start_ms = d_TOW_at_current_symbol_ms;
                                }
                        }
                }
            else if (d_symbol_history.full() && search_frame(current_symbol.Flag_PLL_180_deg_phase_locked))
                {
                    d_flag_frame_sync = true;
                    d_symbols_in_frame = 0;
                    d_frame_errors = 0;
                    d_last_valid_frame = d_sample_counter;
                    d_expected_toi = (d_nav.get_toi() + 1) % (GPS_L1C_TOI_MAX + 1);
                    LOG(INFO) << "GPS L1C frame synchronization in channel " << d_channel << " for satellite " << d_satellite
                              << " with CN0=" << current_symbol.CN0_dB_hz << " dB-Hz, TOI=" << d_nav.get_toi()
                              << (d_inverted ? ", inverted symbols" : "");
                    LOG(INFO) << "GPS L1C subframes at frame synchronization in channel " << d_channel
                              << ": subframe 2 " << (d_nav.sf2_crc_ok() ? d_nav.get_sf2_bits() : std::string("failed"))
                              << ", subframe 3 " << (d_nav.sf3_crc_ok() ? d_nav.get_sf3_bits() : std::string("failed"));
                    if (d_dump_crc_stats)
                        {
                            d_Tlm_CRC_Stats->update_CRC_stats(d_nav.sf2_crc_ok());
                            d_Tlm_CRC_Stats->update_CRC_stats(d_nav.sf3_crc_ok());
                        }
                    update_time_of_week(false);
                    if (d_flag_valid_word)
                        {
                            d_TOW_at_frame_start_ms = d_TOW_at_current_symbol_ms;
                        }
                    publish_navigation(current_symbol);
                    check_navigation_usability();
                }
        }

    if (d_flag_valid_word)
        {
            if (d_inverted)
                {
                    // correct the accumulated phase for the 180 deg PLL lock
                    current_symbol.Carrier_phase_rads += GNSS_PI;
                    current_symbol.Flag_PLL_180_deg_phase_locked = true;
                }
            else
                {
                    current_symbol.Flag_PLL_180_deg_phase_locked = false;
                }
            current_symbol.TOW_at_current_symbol_ms = d_TOW_at_current_symbol_ms;
            current_symbol.Flag_valid_word = true;

            if (d_dump)
                {
                    // MULTIPLEXED FILE RECORDING - Record results to file
                    try
                        {
                            write_value(d_dump_file, static_cast<double>(d_TOW_at_current_symbol_ms) / 1000.0);
                            write_value(d_dump_file, current_symbol.Tracking_sample_counter);
                            write_value(d_dump_file, static_cast<double>(d_TOW_at_frame_start_ms) / 1000.0);
                            write_value(d_dump_file, ((current_symbol.Prompt_I > 0.0) != d_inverted) ? 1 : -1);
                            write_value(d_dump_file, static_cast<int32_t>(current_symbol.PRN));
                        }
                    catch (const std::ofstream::failure &e)
                        {
                            LOG(WARNING) << "Exception writing Telemetry GPS L1C dump file " << e.what();
                        }
                }

            // SEND TOW TO THE TRACKING BLOCK
            if (d_tow_to_trk)
                {
                    const auto tmp_tow_obj = std::make_shared<TOW_to_trk>(
                        std::string("1S"),
                        d_channel,
                        d_TOW_at_current_symbol_ms,
                        current_symbol.Tracking_sample_counter,
                        d_nav.get_wn(), d_satellite.get_PRN());
                    this->message_port_pub(pmt::mp("telemetry_to_trk"), pmt::make_any(tmp_tow_obj));
                }

            out[0] = std::move(current_symbol);
            return 1;
        }
    // No valid TOW yet: forward the tracking data flagged as not valid for
    // pseudorange computation, so that the Monitor and the channel status see it
    current_symbol.Flag_valid_word = false;
    out[0] = std::move(current_symbol);
    return 1;
}
