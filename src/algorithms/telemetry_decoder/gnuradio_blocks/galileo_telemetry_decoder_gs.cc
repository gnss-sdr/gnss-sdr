/*!
 * \file galileo_telemetry_decoder_gs.cc
 * \brief Implementation of a Galileo unified INAV and FNAV message demodulator
 * block
 * \author Javier Arribas 2018. jarribas(at)cttc.es
 * \author Carles Fernandez, 2021-2022. cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "galileo_telemetry_decoder_gs.h"
#include "Galileo_E1.h"   // for GALILEO_E1_CODE_PERIOD_MS
#include "Galileo_E5a.h"  // for GALILEO_E5A_CODE_PERIOD_MS
#include "Galileo_E6.h"   // for GALILEO_E6_CODE_PERIOD_MS
#include "display.h"      // for colours in terminal: TEXT_BLUE, TEXT_RESET, ...
#include "dump_logger_helper.h"
#include "galileo_almanac_helper.h"  // for Galileo_Almanac_Helper
#include "galileo_ephemeris.h"       // for Galileo_Ephemeris
#include "galileo_has_page.h"        // For Galileo_HAS_page
#include "galileo_iono.h"            // for Galileo_Iono
#include "galileo_tow_map.h"         // for GalileoTowMapEntry
#include "galileo_tow_utils.h"       // for Galileo TOW arithmetic helpers
#include "galileo_utc_model.h"       // for Galileo_Utc_Model
#include "gnss_sdr_make_unique.h"    // for std::make_unique in C++11
#include "gnss_synchro.h"            // for Gnss_Synchro
#include "osnma_data.h"              // for osnma::galileo_week_tow_with_offset
#include "tlm_conf.h"
#include "tlm_crc_stats.h"    // for Tlm_CRC_Stats
#include "tlm_utils.h"        // for save_tlm_matfile, tlm_remove_file
#include "tow_to_trk.h"       // for TOW_to_trk
#include "viterbi_decoder.h"  // for Viterbi_Decoder
#include <pmt/pmt_sugar.h>    // for pmt::mp
#include <array>              // for std::array
#include <cmath>              // for std::fmod, std::abs
#include <cstddef>            // for size_t
#include <iomanip>            // for std::setprecision
#include <iostream>           // for std::cout
#include <limits>             // for std::numeric_limits
#include <stdexcept>          // for std::out_of_range
#include <tuple>              // for std::tuple
#include <typeinfo>           // for typeid

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

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

#define CRC_ERROR_LIMIT 6

galileo_telemetry_decoder_gs_sptr
galileo_make_telemetry_decoder_gs(const Tlm_Conf &conf, int frame_type)
{
    return galileo_telemetry_decoder_gs_sptr(new galileo_telemetry_decoder_gs(conf, frame_type));
}


galileo_telemetry_decoder_gs::galileo_telemetry_decoder_gs(const Tlm_Conf &conf, int frame_type)
    : telemetry_impl_interface("galileo_telemetry_decoder_gs",
          gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
          gr::io_signature::make(1, 1, sizeof(Gnss_Synchro))),
      d_dump_filename(conf.dump_filename),
      d_delta_t(0),
      d_symbol_counter(0ULL),
      d_preamble_index(0ULL),
      d_last_valid_preamble(0ULL),
      d_received_sample_counter(0),
      d_pending_reduced_ced_start_symbol(0ULL),
      d_pending_reduced_ced_cn0(0.0),
      d_frame_type(frame_type),
      d_CRC_error_counter(0),
      d_channel(0),
      d_flag_even_word_arrived(0),
      d_stat(0),
      d_TOW_at_Preamble_ms(0),
      d_TOW_at_current_symbol_ms(0),
      d_received_week(GALILEO_TOW_MAP_INVALID_WEEK),
      d_received_tow_ms(GALILEO_TOW_MAP_INVALID_TOW_MS),
      d_TOW_week(GALILEO_TOW_MAP_INVALID_WEEK),
      d_galileo_week(GALILEO_TOW_MAP_INVALID_WEEK),
      d_band('1'),
      d_sent_tlm_failed_msg(false),
      d_flag_frame_sync(false),
      d_flag_PLL_180_deg_phase_locked(false),
      d_flag_preamble(false),
      d_dump(conf.dump),
      d_dump_mat(conf.dump_mat),
      d_remove_dat(conf.remove_dat),
      d_first_eph_sent(false),
      d_pending_reduced_ced(false),
      d_cnav_dummy_page(false),
      d_print_cnav_page(true),
      d_enable_navdata_monitor(conf.enable_navdata_monitor),
      d_dump_crc_stats(conf.dump_crc_stats),
      d_enable_reed_solomon_inav(false),
      d_valid_timetag(false),
      d_E6_TOW_set(false),
      d_galileo_week_valid(false),
      d_there_are_e1_channels(conf.there_are_e1_channels),
      d_there_are_e6_channels(conf.there_are_e6_channels),
      d_use_ced(conf.use_ced),
      d_tow_to_trk(conf.tow_to_trk)
{
    configure_basic_outputs();

    if (d_there_are_e1_channels)
        {
            // register OSM out
            this->message_port_register_out(pmt::mp("OSNMA_from_TLM"));
        }

    if (d_there_are_e6_channels)
        {
            // register Gal E6 messages HAS out
            this->message_port_register_out(pmt::mp("E6_HAS_from_TLM"));
            // register TOW from map out
            this->message_port_register_out(pmt::mp("TOW_from_TLM"));

            // register TOW to TLM input
            this->message_port_register_in(pmt::mp("TOW_to_TLM"));
            // handler for input port
            this->set_msg_handler(pmt::mp("TOW_to_TLM"),
#if HAS_GENERIC_LAMBDA
                [this](auto &&PH1) { msg_handler_read_galileo_tow_map(PH1); });
#else
#if USE_BOOST_BIND_PLACEHOLDERS
                boost::bind(&galileo_telemetry_decoder_gs::msg_handler_read_galileo_tow_map, this, boost::placeholders::_1));
#else
                boost::bind(&galileo_telemetry_decoder_gs::msg_handler_read_galileo_tow_map, this, _1));
#endif
#endif
        }

    if (d_enable_navdata_monitor)
        {
            // register nav message monitor out
            this->message_port_register_out(pmt::mp("Nav_msg_from_TLM"));
        }

    // Viterbi decoder vars
    const int32_t nn = 2;                               // Coding rate 1/n
    const int32_t KK = 7;                               // Constraint Length
    const std::array<int32_t, 2> g_encoder{{121, 91}};  // Polynomial G1 and G2
    d_mm = KK - 1;

    DLOG(INFO) << "Initializing GALILEO UNIFIED TELEMETRY DECODER";

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

    switch (d_frame_type)
        {
        case 1:  // INAV
            d_bits_per_preamble = GALILEO_INAV_PREAMBLE_LENGTH_BITS;
            d_PRN_code_period_ms = GALILEO_E1_CODE_PERIOD_MS;  // for Galileo E5b is also 4 ms
            // set the preamble
            d_samples_per_preamble = GALILEO_INAV_PREAMBLE_LENGTH_BITS;
            d_preamble_period_symbols = GALILEO_INAV_PREAMBLE_PERIOD_SYMBOLS;
            d_required_symbols = GALILEO_INAV_PAGE_SYMBOLS + d_samples_per_preamble;
            // preamble bits to sampled symbols
            d_preamble_samples = std::vector<int32_t>(d_samples_per_preamble);
            d_frame_length_symbols = GALILEO_INAV_PAGE_PART_SYMBOLS - GALILEO_INAV_PREAMBLE_LENGTH_BITS;
            d_codelength = static_cast<int32_t>(d_frame_length_symbols);
            d_datalength = (d_codelength / nn) - d_mm;
            d_max_symbols_without_valid_frame = GALILEO_INAV_PAGE_SYMBOLS * 30;  // rise alarm 60 seconds without valid tlm
            if (conf.enable_reed_solomon == true)
                {
                    d_enable_reed_solomon_inav = true;
                    d_inav_nav.enable_reed_solomon();
                }
            break;
        case 2:  // FNAV
            d_PRN_code_period_ms = static_cast<uint32_t>(GALILEO_E5A_CODE_PERIOD_MS * GALILEO_E5A_I_SECONDARY_CODE_LENGTH);
            d_bits_per_preamble = GALILEO_FNAV_PREAMBLE_LENGTH_BITS;
            // set the preamble
            d_samples_per_preamble = GALILEO_FNAV_PREAMBLE_LENGTH_BITS;
            d_preamble_period_symbols = GALILEO_FNAV_SYMBOLS_PER_PAGE;
            d_required_symbols = static_cast<uint32_t>(GALILEO_FNAV_SYMBOLS_PER_PAGE) + d_samples_per_preamble;
            // preamble bits to sampled symbols
            d_preamble_samples = std::vector<int32_t>(d_samples_per_preamble);
            d_frame_length_symbols = GALILEO_FNAV_SYMBOLS_PER_PAGE - GALILEO_FNAV_PREAMBLE_LENGTH_BITS;
            d_codelength = static_cast<int32_t>(d_frame_length_symbols);
            d_datalength = (d_codelength / nn) - d_mm;
            d_max_symbols_without_valid_frame = GALILEO_FNAV_SYMBOLS_PER_PAGE * 5;  // rise alarm 100 seconds without valid tlm
            break;
        case 3:  // CNAV
            d_PRN_code_period_ms = GALILEO_E6_CODE_PERIOD_MS;
            d_bits_per_preamble = GALILEO_CNAV_PREAMBLE_LENGTH_BITS;
            d_samples_per_preamble = GALILEO_CNAV_PREAMBLE_LENGTH_BITS;
            d_preamble_period_symbols = GALILEO_CNAV_SYMBOLS_PER_PAGE;
            d_required_symbols = static_cast<uint32_t>(GALILEO_CNAV_SYMBOLS_PER_PAGE) + d_samples_per_preamble;
            d_preamble_samples = std::vector<int32_t>(d_samples_per_preamble);
            d_frame_length_symbols = GALILEO_CNAV_SYMBOLS_PER_PAGE - GALILEO_CNAV_PREAMBLE_LENGTH_BITS;
            d_codelength = static_cast<int32_t>(d_frame_length_symbols);
            d_datalength = (d_codelength / nn) - d_mm;
            d_max_symbols_without_valid_frame = GALILEO_CNAV_SYMBOLS_PER_PAGE * 60;
            break;
        default:
            d_bits_per_preamble = 0;
            d_samples_per_preamble = 0;
            d_preamble_period_symbols = 0;
            d_PRN_code_period_ms = 0U;
            d_required_symbols = 0U;
            d_frame_length_symbols = 0U;
            d_codelength = 0;
            d_datalength = 0;
            d_max_symbols_without_valid_frame = 0;
            std::cout << "Galileo unified telemetry decoder error: Unknown frame type\n";
        }

    d_page_part_symbols = std::vector<float>(d_frame_length_symbols);

    for (int32_t i = 0; i < d_bits_per_preamble; i++)
        {
            switch (d_frame_type)
                {
                case 1:  // INAV
                    if (GALILEO_INAV_PREAMBLE[i] == '1')
                        {
                            d_preamble_samples[i] = 1;
                        }
                    else
                        {
                            d_preamble_samples[i] = -1;
                        }
                    break;
                case 2:  // FNAV for E5a-I
                    if (GALILEO_FNAV_PREAMBLE[i] == '1')
                        {
                            d_preamble_samples[i] = 1;
                        }
                    else
                        {
                            d_preamble_samples[i] = -1;
                        }
                    break;
                case 3:  // CNAV for E6
                    if (GALILEO_CNAV_PREAMBLE[i] == '1')
                        {
                            d_preamble_samples[i] = 1;
                        }
                    else
                        {
                            d_preamble_samples[i] = -1;
                        }
                    break;
                }
        }

    d_symbol_history.set_capacity(d_required_symbols + 1);

    d_inav_nav.init_PRN(d_satellite.get_PRN());

    // Instantiate the Viterbi decoder
    d_viterbi = std::make_unique<Viterbi_Decoder>(KK, nn, d_datalength, g_encoder);
}


galileo_telemetry_decoder_gs::~galileo_telemetry_decoder_gs()
{
    DLOG(INFO) << "Galileo Telemetry decoder block (channel " << d_channel << ") destructor called.";
    tlm_cleanup_and_save_files(d_dump_file, d_dump_filename, d_dump, d_dump_mat, d_remove_dat);
}


void galileo_telemetry_decoder_gs::msg_handler_read_galileo_tow_map(const pmt::pmt_t &msg)
{
    if (d_frame_type == 3)
        {
            try
                {
                    const size_t msg_type_hash_code = pmt::any_ref(msg).type().hash_code();
                    if (msg_type_hash_code == typeid(std::shared_ptr<GalileoTowMap>).hash_code())
                        {
                            const auto received_tow_map = wht::any_cast<std::shared_ptr<GalileoTowMap>>(pmt::any_ref(msg));
                            const GalileoTowMapEntry received_tow_sample = received_tow_map->at(d_satellite.get_PRN());
                            if (received_tow_sample.week != GALILEO_TOW_MAP_INVALID_WEEK && received_tow_sample.tow_ms < galileo_tow::WEEK_MS)
                                {
                                    d_received_week = received_tow_sample.week;
                                    d_received_tow_ms = received_tow_sample.tow_ms;
                                    d_received_sample_counter = received_tow_sample.sample_counter;
                                }
                            else
                                {
                                    d_received_week = GALILEO_TOW_MAP_INVALID_WEEK;
                                    d_received_tow_ms = GALILEO_TOW_MAP_INVALID_TOW_MS;
                                    d_received_sample_counter = GALILEO_TOW_MAP_INVALID_SAMPLE_COUNTER;
                                }
                        }
                }
            catch (const wht::bad_any_cast &e)
                {
                    LOG(WARNING) << "msg_handler_read_galileo_tow_map Bad any_cast: " << e.what();
                }
            catch (const std::out_of_range &oor)
                {
                    LOG(WARNING) << "msg_handler_read_galileo_tow_map Out of Range error: " << oor.what();
                }
        }
}


void galileo_telemetry_decoder_gs::deinterleaver(int32_t rows, int32_t cols, const float *in, float *out)
{
    for (int32_t r = 0; r < rows; r++)
        {
            for (int32_t c = 0; c < cols; c++)
                {
                    out[c * rows + r] = in[r * cols + c];
                }
        }
}


void galileo_telemetry_decoder_gs::clear_galileo_tow_map_entry()
{
    publish_galileo_tow_map_entry(GALILEO_TOW_MAP_INVALID_WEEK, GALILEO_TOW_MAP_INVALID_TOW_MS, GALILEO_TOW_MAP_INVALID_SAMPLE_COUNTER);
}


void galileo_telemetry_decoder_gs::publish_galileo_tow_map_entry(uint32_t week, uint32_t tow_ms, uint64_t sample_counter)
{
    if (!d_there_are_e6_channels)
        {
            return;
        }

    const GalileoTowMapEntry tow_entry{week, tow_ms, sample_counter};
    const auto tmp_obj = std::make_shared<GalileoTowMapMessage>(d_satellite.get_PRN(), tow_entry);
    this->message_port_pub(pmt::mp("TOW_from_TLM"), pmt::make_any(tmp_obj));
}


void galileo_telemetry_decoder_gs::publish_current_galileo_tow_map_entry(uint64_t sample_counter)
{
    if (d_TOW_week != GALILEO_TOW_MAP_INVALID_WEEK)
        {
            publish_galileo_tow_map_entry(d_TOW_week, d_TOW_at_current_symbol_ms, sample_counter);
        }
}


bool galileo_telemetry_decoder_gs::update_known_galileo_week(int32_t week)
{
    if (week < 0)
        {
            return false;
        }

    d_galileo_week = static_cast<uint32_t>(week);
    d_galileo_week_valid = true;
    return true;
}


bool galileo_telemetry_decoder_gs::set_current_tow_from_preamble(uint32_t preamble_week, uint32_t preamble_tow_ms, int64_t delay_ms)
{
    d_TOW_at_current_symbol_ms = galileo_tow::add_ms(preamble_tow_ms, delay_ms);
    if (preamble_week == GALILEO_TOW_MAP_INVALID_WEEK)
        {
            d_TOW_week = GALILEO_TOW_MAP_INVALID_WEEK;
            return false;
        }

    uint32_t current_week = GALILEO_TOW_MAP_INVALID_WEEK;
    if (!galileo_tow::week_after_delta(preamble_week, preamble_tow_ms, delay_ms, current_week))
        {
            d_TOW_week = GALILEO_TOW_MAP_INVALID_WEEK;
            return false;
        }

    d_TOW_week = current_week;
    return true;
}


void galileo_telemetry_decoder_gs::advance_current_tow(int64_t delta_ms)
{
    if (d_TOW_week != GALILEO_TOW_MAP_INVALID_WEEK)
        {
            uint32_t current_week = GALILEO_TOW_MAP_INVALID_WEEK;
            if (galileo_tow::week_after_delta(d_TOW_week, d_TOW_at_current_symbol_ms, delta_ms, current_week))
                {
                    d_TOW_week = current_week;
                }
            else
                {
                    d_TOW_week = GALILEO_TOW_MAP_INVALID_WEEK;
                }
        }
    d_TOW_at_current_symbol_ms = galileo_tow::add_ms(d_TOW_at_current_symbol_ms, delta_ms);
}


void galileo_telemetry_decoder_gs::capture_pending_reduced_ced(double cn0)
{
    if (d_band != '1' || !d_use_ced || d_first_eph_sent)
        {
            return;
        }

    const int64_t decoder_delay_ms = galileo_tow::inav_current_symbol_delay_ms(d_required_symbols, d_PRN_code_period_ms);
    const uint64_t decoder_delay_symbols = static_cast<uint64_t>(decoder_delay_ms / static_cast<int64_t>(d_PRN_code_period_ms));
    if (d_symbol_counter < decoder_delay_symbols)
        {
            return;
        }

    d_pending_reduced_ced_start_symbol = d_symbol_counter - decoder_delay_symbols;
    d_pending_reduced_ced_cn0 = cn0;
    d_pending_reduced_ced = true;
}


void galileo_telemetry_decoder_gs::publish_pending_reduced_ced()
{
    if (!d_pending_reduced_ced || d_first_eph_sent || !d_inav_nav.has_valid_word_5() ||
        d_TOW_week == GALILEO_TOW_MAP_INVALID_WEEK)
        {
            return;
        }

    const int64_t symbol_delta = galileo_tow::sample_counter_delta(d_pending_reduced_ced_start_symbol, d_symbol_counter);
    const int64_t code_period_ms = static_cast<int64_t>(d_PRN_code_period_ms);
    if (symbol_delta > std::numeric_limits<int64_t>::max() / code_period_ms ||
        symbol_delta < std::numeric_limits<int64_t>::min() / code_period_ms)
        {
            d_pending_reduced_ced = false;
            return;
        }
    const int64_t tow_delta_ms = symbol_delta * code_period_ms;
    uint32_t reduced_ced_week = GALILEO_TOW_MAP_INVALID_WEEK;
    if (!galileo_tow::week_after_delta(d_TOW_week, d_TOW_at_current_symbol_ms, tow_delta_ms, reduced_ced_week))
        {
            d_pending_reduced_ced = false;
            return;
        }

    Galileo_Reduced_CED reduced_ced = d_inav_nav.get_reduced_ced();
    reduced_ced.WN = reduced_ced_week;
    reduced_ced.TOTRedCED = galileo_tow::add_ms(d_TOW_at_current_symbol_ms, tow_delta_ms) / 1000U;
    const auto tmp_obj = std::make_shared<Galileo_Reduced_CED>(reduced_ced);
    this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));

    const auto default_precision = std::cout.precision();
    std::cout << "New Galileo E1 I/NAV reduced CED message received in channel "
              << d_channel << " from satellite " << d_satellite << " with CN0="
              << std::setprecision(2) << d_pending_reduced_ced_cn0 << std::setprecision(default_precision)
              << " dB-Hz" << std::endl;
    d_pending_reduced_ced = false;
}


galileo_telemetry_decoder_gs::CnavPageReceptionTime galileo_telemetry_decoder_gs::get_cnav_page_reception_time(const Gnss_Synchro &current_symbol) const
{
    CnavPageReceptionTime page_reception_time{GALILEO_TOW_MAP_INVALID_WEEK, GALILEO_TOW_MAP_INVALID_TOW_MS};
    if (d_valid_timetag == true)
        {
            const uint32_t predicted_tow_at_preamble_ms =
                galileo_tow::floor_to_second_ms(galileo_tow::wrap_ms(d_current_timetag.tow_ms));
            const int64_t decoder_delay_ms = (static_cast<int64_t>(d_required_symbols) + 1LL) * static_cast<int64_t>(d_PRN_code_period_ms);
            page_reception_time.tow_ms = galileo_tow::add_ms(predicted_tow_at_preamble_ms, decoder_delay_ms);
            if (d_current_timetag.week >= 0)
                {
                    galileo_tow::week_after_delta(static_cast<uint32_t>(d_current_timetag.week), predicted_tow_at_preamble_ms, decoder_delay_ms, page_reception_time.week);
                }
            return page_reception_time;
        }

    if (d_received_week != GALILEO_TOW_MAP_INVALID_WEEK &&
        d_received_tow_ms < galileo_tow::WEEK_MS &&
        d_received_sample_counter != GALILEO_TOW_MAP_INVALID_SAMPLE_COUNTER &&
        current_symbol.fs > 0LL)
        {
            const int64_t diff = galileo_tow::sample_counter_delta(current_symbol.Tracking_sample_counter, d_received_sample_counter);
            const auto time_since_reference_ms = static_cast<int64_t>(static_cast<double>(diff) * 1000.0 / static_cast<double>(current_symbol.fs));
            const int64_t tow_delta_ms = time_since_reference_ms + static_cast<int64_t>(GALILEO_E6_CODE_PERIOD_MS);
            page_reception_time.tow_ms = galileo_tow::add_ms(d_received_tow_ms, tow_delta_ms);
            galileo_tow::week_after_delta(d_received_week, d_received_tow_ms, tow_delta_ms, page_reception_time.week);
            return page_reception_time;
        }

    if (d_E6_TOW_set == true)
        {
            page_reception_time.tow_ms = galileo_tow::add_ms(d_TOW_at_current_symbol_ms, d_PRN_code_period_ms);
            if (d_TOW_week != GALILEO_TOW_MAP_INVALID_WEEK)
                {
                    galileo_tow::week_after_delta(d_TOW_week, d_TOW_at_current_symbol_ms, d_PRN_code_period_ms, page_reception_time.week);
                }
            return page_reception_time;
        }

    return page_reception_time;
}


void galileo_telemetry_decoder_gs::decode_INAV_word(float *page_part_symbols, int32_t frame_length, double cn0)
{
    // 1. De-interleave
    std::vector<float> page_part_symbols_soft_value(frame_length);
    deinterleaver(GALILEO_INAV_INTERLEAVER_ROWS, GALILEO_INAV_INTERLEAVER_COLS, page_part_symbols, page_part_symbols_soft_value.data());
    // 2. Viterbi decoder
    // 2.1 Take into account the NOT gate in G2 polynomial (Galileo ICD Figure 13, FEC encoder)
    for (int32_t i = 0; i < frame_length; i++)
        {
            if ((i + 1) % 2 == 0)
                {
                    page_part_symbols_soft_value[i] = -page_part_symbols_soft_value[i];
                }
        }
    const int32_t decoded_length = frame_length / 2;
    std::vector<int32_t> page_part_bits(decoded_length);
    d_viterbi->decode(page_part_bits, page_part_symbols_soft_value);

    // 3. Call the Galileo page decoder
    std::string page_String;
    page_String.reserve(decoded_length);
    for (int32_t i = 0; i < decoded_length; i++)
        {
            if (page_part_bits[i] > 0)
                {
                    page_String.push_back('1');
                }
            else
                {
                    page_String.push_back('0');
                }
        }

    if (d_enable_navdata_monitor)
        {
            d_nav_msg_packet.nav_message = page_String;
        }

    if (page_part_bits[0] == 1)
        {
            // DECODE COMPLETE WORD (even + odd) and TEST CRC
            d_inav_nav.split_page(std::move(page_String), d_flag_even_word_arrived);
            if (d_inav_nav.get_flag_CRC_test() == true)
                {
                    if (d_band == '1')
                        {
                            DLOG(INFO) << "Galileo E1 CRC correct in channel " << d_channel << " from satellite " << d_satellite;
                        }
                    else if (d_band == '7')
                        {
                            DLOG(INFO) << "Galileo E5b CRC correct in channel " << d_channel << " from satellite " << d_satellite;
                        }
                }
            else
                {
                    if (d_band == '1')
                        {
                            DLOG(INFO) << "Galileo E1 CRC error in channel " << d_channel << " from satellite " << d_satellite;
                        }
                    else if (d_band == '7')
                        {
                            DLOG(INFO) << "Galileo E5b CRC error in channel " << d_channel << " from satellite " << d_satellite;
                        }
                }
            d_flag_even_word_arrived = 0;
        }
    else
        {
            // STORE HALF WORD (even page)
            d_inav_nav.split_page(std::move(page_String), d_flag_even_word_arrived);
            d_flag_even_word_arrived = 1;
        }

    // 4. Push the new navigation data to the queues
    // extract OSNMA bits, reset container.
    if (d_inav_nav.get_osnma_adkd_0_12_nav_bits().size() == 549 && d_band == '1')
        {
            const auto osnma_nav_time = osnma::galileo_week_tow_with_offset(d_inav_nav.get_Galileo_week(), d_inav_nav.get_TOW5(), -25);
            DLOG(INFO) << "Galileo OSNMA: new ADKD=0/12 navData from " << d_satellite << " at WN=" << osnma_nav_time.first << ", TOW_sf=" << osnma_nav_time.second;
            const auto tmp_obj_osnma = std::make_shared<std::tuple<uint32_t, std::string, uint32_t, uint32_t>>(  // < PRNd , navDataBits, WN, TOW_Sosf>
                d_satellite.get_PRN(),
                d_inav_nav.get_osnma_adkd_0_12_nav_bits(),
                osnma_nav_time.first,
                osnma_nav_time.second);
            this->message_port_pub(pmt::mp("OSNMA_from_TLM"), pmt::make_any(tmp_obj_osnma));
            d_inav_nav.reset_osnma_nav_bits_adkd0_12();
        }
    if (d_inav_nav.get_osnma_adkd_4_nav_bits().size() == 141 && d_band == '1')
        {
            const auto osnma_nav_time = osnma::galileo_week_tow_with_offset(d_inav_nav.get_Galileo_week(), d_inav_nav.get_TOW6(), -5);
            DLOG(INFO) << "Galileo OSNMA: new ADKD=4 navData from " << d_satellite << " at WN=" << osnma_nav_time.first << ", TOW_sf=" << osnma_nav_time.second;
            const auto tmp_obj = std::make_shared<std::tuple<uint32_t, std::string, uint32_t, uint32_t>>(  // < PRNd , navDataBits, WN, TOW_Sosf> // TODO conversion from W6 to W_Start_of_subframe
                d_satellite.get_PRN(),
                d_inav_nav.get_osnma_adkd_4_nav_bits(),
                osnma_nav_time.first,
                osnma_nav_time.second);
            this->message_port_pub(pmt::mp("OSNMA_from_TLM"), pmt::make_any(tmp_obj));
            d_inav_nav.reset_osnma_nav_bits_adkd4();
        }

    if (d_inav_nav.have_new_reduced_ced())
        {
            capture_pending_reduced_ced(cn0);
        }

    if (d_inav_nav.have_new_ephemeris() == true)  // C: tells if W1-->W4 available from same block (and W5!)
        {
            // get object for this SV (mandatory)
            const std::shared_ptr<Galileo_Ephemeris> tmp_obj = std::make_shared<Galileo_Ephemeris>(d_inav_nav.get_ephemeris());
            if (d_band == '1')
                {
                    const auto default_precision = std::cout.precision();
                    std::cout << "New Galileo E1 I/NAV message received in channel " << d_channel
                              << ": ephemeris from satellite " << d_satellite << " with CN0="
                              << std::setprecision(2) << cn0 << std::setprecision(default_precision)
                              << " dB-Hz" << std::endl;
                }
            else if (d_band == '7')
                {
                    const auto default_precision = std::cout.precision();
                    std::cout << TEXT_BLUE
                              << "New Galileo E5b I/NAV message received in channel " << d_channel
                              << ": ephemeris from satellite " << d_satellite << " with CN0="
                              << std::setprecision(2) << cn0 << std::setprecision(default_precision)
                              << " dB-Hz" << TEXT_RESET << std::endl;
                }
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            d_first_eph_sent = true;  // do not send reduced CED anymore, since we have the full ephemeris set
            d_pending_reduced_ced = false;
        }

    if (d_inav_nav.have_new_iono_and_GST() == true)  // C: W5
        {
            // get object for this SV (mandatory)
            const std::shared_ptr<Galileo_Iono> tmp_obj = std::make_shared<Galileo_Iono>(d_inav_nav.get_iono());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            if (d_band == '1')
                {
                    const auto default_precision = std::cout.precision();
                    std::cout << "New Galileo E1 I/NAV message received in channel " << d_channel
                              << ": iono/GST model parameters from satellite " << d_satellite
                              << " with CN0=" << std::setprecision(2) << cn0 << std::setprecision(default_precision)
                              << " dB-Hz" << std::endl;
                }
            else if (d_band == '7')
                {
                    const auto default_precision = std::cout.precision();
                    std::cout << TEXT_BLUE << "New Galileo E5b I/NAV message received in channel "
                              << d_channel << ": iono/GST model parameters from satellite "
                              << d_satellite << " with CN0=" << std::setprecision(2) << cn0 << std::setprecision(default_precision)
                              << " dB-Hz" << TEXT_RESET << std::endl;
                }
        }

    if (d_inav_nav.have_new_utc_model() == true)  // C: tells if W6 is available
        {
            // get object for this SV (mandatory)
            const std::shared_ptr<Galileo_Utc_Model> tmp_obj = std::make_shared<Galileo_Utc_Model>(d_inav_nav.get_utc_model());
            LOG(INFO) << "Galileo leap second Delta_tLS=" << tmp_obj->Delta_tLS;
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            if (d_band == '1')
                {
                    const auto default_precision = std::cout.precision();
                    std::cout << "New Galileo E1 I/NAV message received in channel " << d_channel
                              << ": UTC model parameters from satellite " << d_satellite
                              << " with CN0=" << std::setprecision(2) << cn0 << std::setprecision(default_precision)
                              << " dB-Hz" << std::endl;
                }
            else if (d_band == '7')
                {
                    const auto default_precision = std::cout.precision();
                    std::cout << TEXT_BLUE << "New Galileo E5b I/NAV message received in channel "
                              << d_channel
                              << ": UTC model parameters from satellite " << d_satellite
                              << " with CN0=" << std::setprecision(2) << cn0 << std::setprecision(default_precision)
                              << " dB-Hz" << TEXT_RESET << std::endl;
                }

            if (tmp_obj->flag_GGTO)
                {
                    d_delta_t = tmp_obj->A_0G + tmp_obj->A_1G * (static_cast<double>(d_TOW_at_current_symbol_ms) / 1000.0 - tmp_obj->t_0G + 604800 * (std::fmod(static_cast<float>(d_inav_nav.get_Galileo_week() - tmp_obj->WN_0G), 64.0)));
                    DLOG(INFO) << "delta_t=" << d_delta_t << "[s]";
                }
        }

    if (d_inav_nav.have_new_almanac() == true)  // flag_almanac_4 tells if W10 available.
        {
            const std::shared_ptr<Galileo_Almanac_Helper> tmp_obj = std::make_shared<Galileo_Almanac_Helper>(d_inav_nav.get_almanac());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            // debug
            if (d_band == '1')
                {
                    const auto default_precision = std::cout.precision();
                    std::cout << "Galileo E1 I/NAV almanac received in channel " << d_channel
                              << " from satellite " << d_satellite << " with CN0="
                              << std::setprecision(2) << cn0 << std::setprecision(default_precision) << " dB-Hz" << std::endl;
                }
            else if (d_band == '7')
                {
                    const auto default_precision = std::cout.precision();
                    std::cout << TEXT_BLUE << "Galileo E5b I/NAV almanac received in channel "
                              << d_channel << " from satellite " << d_satellite << " with CN0="
                              << std::setprecision(2) << cn0 << std::setprecision(default_precision)
                              << " dB-Hz" << TEXT_RESET << std::endl;
                }
            DLOG(INFO) << "Current parameters:";
            DLOG(INFO) << "d_TOW_at_current_symbol_ms=" << d_TOW_at_current_symbol_ms;
            DLOG(INFO) << "d_nav.WN_0=" << d_inav_nav.get_Galileo_week();
        }
    auto newOSNMA = d_inav_nav.have_new_nma();
    if (d_band == '1' && newOSNMA)
        {
            const std::shared_ptr<OSNMA_msg> tmp_obj = std::make_shared<OSNMA_msg>(d_inav_nav.get_osnma_msg());
            this->message_port_pub(pmt::mp("OSNMA_from_TLM"), pmt::make_any(tmp_obj));
        }
}


void galileo_telemetry_decoder_gs::decode_FNAV_word(float *page_symbols, int32_t frame_length, double cn0)
{
    // 1. De-interleave
    std::vector<float> page_symbols_soft_value(frame_length);
    deinterleaver(GALILEO_FNAV_INTERLEAVER_ROWS, GALILEO_FNAV_INTERLEAVER_COLS, page_symbols, page_symbols_soft_value.data());

    // 2. Viterbi decoder
    // 2.1 Take into account the NOT gate in G2 polynomial (Galileo ICD Figure 13, FEC encoder)
    for (int32_t i = 0; i < frame_length; i++)
        {
            if ((i + 1) % 2 == 0)
                {
                    page_symbols_soft_value[i] = -page_symbols_soft_value[i];
                }
        }

    const int32_t decoded_length = frame_length / 2;
    std::vector<int32_t> page_bits(decoded_length);
    d_viterbi->decode(page_bits, page_symbols_soft_value);

    // 3. Call the Galileo page decoder
    std::string page_String;
    page_String.reserve(decoded_length);
    for (int32_t i = 0; i < decoded_length; i++)
        {
            if (page_bits[i] > 0)
                {
                    page_String.push_back('1');
                }
            else
                {
                    page_String.push_back('0');
                }
        }

    if (d_enable_navdata_monitor)
        {
            d_nav_msg_packet.nav_message = page_String;
        }

    // DECODE COMPLETE WORD (even + odd) and TEST CRC
    d_fnav_nav.split_page(page_String);
    if (d_fnav_nav.get_flag_CRC_test() == true)
        {
            DLOG(INFO) << "Galileo E5a CRC correct in channel " << d_channel << " from satellite " << d_satellite << " with CN0=" << cn0 << " dB-Hz";
        }
    else
        {
            DLOG(INFO) << "Galileo E5a CRC error in channel " << d_channel << " from satellite " << d_satellite << " with CN0=" << cn0 << " dB-Hz";
        }

    // 4. Push the new navigation data to the queues
    if (d_fnav_nav.have_new_ephemeris() == true)
        {
            const std::shared_ptr<Galileo_Ephemeris> tmp_obj = std::make_shared<Galileo_Ephemeris>(d_fnav_nav.get_ephemeris());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            const auto default_precision = std::cout.precision();
            std::cout << TEXT_MAGENTA << "New Galileo E5a F/NAV message received in channel "
                      << d_channel << ": ephemeris from satellite " << d_satellite << " with CN0="
                      << std::setprecision(2) << cn0 << std::setprecision(default_precision)
                      << " dB-Hz" << TEXT_RESET << std::endl;
        }

    if (d_fnav_nav.have_new_iono_and_GST() == true)
        {
            const std::shared_ptr<Galileo_Iono> tmp_obj = std::make_shared<Galileo_Iono>(d_fnav_nav.get_iono());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            const auto default_precision = std::cout.precision();
            std::cout << TEXT_MAGENTA << "New Galileo E5a F/NAV message received in channel "
                      << d_channel << ": iono/GST model parameters from satellite " << d_satellite
                      << " with CN0=" << std::setprecision(2) << cn0 << std::setprecision(default_precision)
                      << " dB-Hz" << TEXT_RESET << std::endl;
        }

    if (d_fnav_nav.have_new_utc_model() == true)
        {
            const std::shared_ptr<Galileo_Utc_Model> tmp_obj = std::make_shared<Galileo_Utc_Model>(d_fnav_nav.get_utc_model());
            LOG(INFO) << "Galileo leap second Delta_tLS=" << tmp_obj->Delta_tLS;
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            const auto default_precision = std::cout.precision();
            std::cout << TEXT_MAGENTA << "New Galileo E5a F/NAV message received in channel "
                      << d_channel << ": UTC model parameters from satellite " << d_satellite
                      << " with CN0=" << std::setprecision(2) << cn0 << std::setprecision(default_precision)
                      << " dB-Hz" << TEXT_RESET << std::endl;
        }
}


void galileo_telemetry_decoder_gs::decode_CNAV_word(uint64_t time_stamp, CnavPageReceptionTime page_reception_time, float *page_symbols, int32_t page_length, double cn0)
{
    // 1. De-interleave
    std::vector<float> page_symbols_soft_value(page_length);
    deinterleaver(GALILEO_CNAV_INTERLEAVER_ROWS, GALILEO_CNAV_INTERLEAVER_COLS, page_symbols, page_symbols_soft_value.data());

    // 2. Viterbi decoder
    // 2.1 Take into account the NOT gate in G2 polynomial (Galileo ICD Figure 13, FEC encoder)
    for (int32_t i = 0; i < page_length; i++)
        {
            if ((i + 1) % 2 == 0)
                {
                    page_symbols_soft_value[i] = -page_symbols_soft_value[i];
                }
        }
    const int32_t decoded_length = page_length / 2;
    std::vector<int32_t> page_bits(decoded_length);
    d_viterbi->decode(page_bits, page_symbols_soft_value);

    // 3. Call the Galileo page decoder
    std::string page_String;
    page_String.reserve(decoded_length);
    for (int32_t i = 0; i < decoded_length; i++)
        {
            if (page_bits[i] > 0)
                {
                    page_String.push_back('1');
                }
            else
                {
                    page_String.push_back('0');
                }
        }

    if (d_enable_navdata_monitor)
        {
            d_nav_msg_packet.nav_message = page_String;
        }

    d_cnav_nav.read_HAS_page(page_String);
    d_cnav_nav.set_time_stamp(time_stamp);
    // 4. If we have a new HAS page, read it
    if (d_cnav_nav.have_new_HAS_page() == true)
        {
            bool is_page_dummy = d_cnav_nav.is_HAS_page_dummy();
            if (is_page_dummy == true)
                {
                    d_print_cnav_page = true;
                    // Only print the message once
                    if (is_page_dummy != d_cnav_dummy_page)
                        {
                            d_cnav_dummy_page = is_page_dummy;
                            const auto default_precision = std::cout.precision();
                            std::cout << TEXT_MAGENTA << "Receiving Galileo E6 CNAV dummy pages in channel "
                                      << d_channel << " from satellite "
                                      << d_satellite << " with CN0="
                                      << std::setprecision(2) << cn0 << std::setprecision(default_precision) << " dB-Hz"
                                      << TEXT_RESET << std::endl;
                        }
                }
            else
                {
                    // HAS ICD Section 7.7 uses the time when the HAS message has been fully received.
                    if (page_reception_time.tow_ms < galileo_tow::WEEK_MS)
                        {
                            d_cnav_nav.set_tow(page_reception_time.tow_ms / 1000U);
                            d_cnav_nav.set_week(page_reception_time.week);
                        }
                    const std::shared_ptr<Galileo_HAS_page> tmp_obj = std::make_shared<Galileo_HAS_page>(d_cnav_nav.get_HAS_encoded_page());
                    this->message_port_pub(pmt::mp("E6_HAS_from_TLM"), pmt::make_any(tmp_obj));
                    if (d_print_cnav_page == true)
                        {
                            d_print_cnav_page = false;  // only print the first page
                            const auto default_precision = std::cout.precision();
                            std::cout << TEXT_MAGENTA << "Receiving Galileo E6 HAS pages"
                                      << (d_cnav_nav.is_HAS_in_test_mode() == true ? " (test mode) " : " ")
                                      << "in channel " << d_channel << " from satellite " << d_satellite
                                      << " with CN0=" << std::setprecision(2) << cn0 << std::setprecision(default_precision) << " dB-Hz"
                                      << TEXT_RESET << std::endl;
                        }
                }
        }
}


void galileo_telemetry_decoder_gs::set_satellite(const Gnss_Satellite &satellite)
{
    gr::thread::scoped_lock lock(d_setlock);
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    d_last_valid_preamble = d_symbol_counter;
    d_sent_tlm_failed_msg = false;
    d_received_week = GALILEO_TOW_MAP_INVALID_WEEK;
    d_received_tow_ms = GALILEO_TOW_MAP_INVALID_TOW_MS;
    d_received_sample_counter = GALILEO_TOW_MAP_INVALID_SAMPLE_COUNTER;
    d_TOW_week = GALILEO_TOW_MAP_INVALID_WEEK;
    d_galileo_week = GALILEO_TOW_MAP_INVALID_WEEK;
    d_galileo_week_valid = false;
    d_E6_TOW_set = false;
    d_valid_timetag = false;
    d_first_eph_sent = false;
    d_pending_reduced_ced = false;
    d_inav_nav.init_PRN(d_satellite.get_PRN());
    d_symbol_history.clear();
    clear_galileo_tow_map_entry();
    DLOG(INFO) << "Setting decoder Finite State Machine to satellite " << d_satellite;
    DLOG(INFO) << "Navigation Satellite set to " << d_satellite;
}


void galileo_telemetry_decoder_gs::reset()
{
    gr::thread::scoped_lock lock(d_setlock);
    d_flag_frame_sync = false;
    d_TOW_at_current_symbol_ms = 0;
    d_TOW_at_Preamble_ms = 0;
    d_TOW_week = GALILEO_TOW_MAP_INVALID_WEEK;
    d_galileo_week = GALILEO_TOW_MAP_INVALID_WEEK;
    d_galileo_week_valid = false;
    d_fnav_nav.set_flag_TOW_set(false);
    d_inav_nav.set_flag_TOW_set(false);
    d_inav_nav.set_TOW0_flag(false);
    d_inav_nav.init_PRN(d_satellite.get_PRN());
    d_last_valid_preamble = d_symbol_counter;
    d_sent_tlm_failed_msg = false;
    d_E6_TOW_set = false;
    d_stat = 0;
    d_received_week = GALILEO_TOW_MAP_INVALID_WEEK;
    d_received_tow_ms = GALILEO_TOW_MAP_INVALID_TOW_MS;
    d_received_sample_counter = GALILEO_TOW_MAP_INVALID_SAMPLE_COUNTER;
    d_viterbi->reset();
    d_valid_timetag = false;
    d_first_eph_sent = false;
    d_pending_reduced_ced = false;
    d_symbol_history.clear();
    clear_galileo_tow_map_entry();
    if (d_enable_reed_solomon_inav == true)
        {
            d_inav_nav.enable_reed_solomon();
        }
    DLOG(INFO) << "Telemetry decoder reset for satellite " << d_satellite;
}


void galileo_telemetry_decoder_gs::set_channel(int32_t channel)
{
    d_channel = channel;
    DLOG(INFO) << "Navigation channel set to " << channel;

    configure_dump_file(d_channel, d_dump, d_dump_filename, d_dump_file);
    configure_crc_stats_channel(d_channel, d_dump_crc_stats, d_Tlm_CRC_Stats);
}


void galileo_telemetry_decoder_gs::check_tlm_separation()
{
    gr::thread::scoped_lock lock(d_setlock);
    if (d_sent_tlm_failed_msg == false)
        {
            if ((d_symbol_counter - d_last_valid_preamble) > d_max_symbols_without_valid_frame)
                {
                    const int message = 1;  // bad telemetry
                    DLOG(INFO) << "Wrong tlm sync in sat " << this->d_satellite;
                    this->message_port_pub(pmt::mp("telemetry_to_trk"), pmt::make_any(message));
                    d_sent_tlm_failed_msg = true;
                }
        }
}


int galileo_telemetry_decoder_gs::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    auto **out = reinterpret_cast<Gnss_Synchro **>(&output_items[0]);            // Get the output buffer pointer
    const auto **in = reinterpret_cast<const Gnss_Synchro **>(&input_items[0]);  // Get the input buffer pointer

    Gnss_Synchro current_symbol{};  // structure to save the synchronization information and send the output object to the next block
    // 1. Copy the current tracking output
    current_symbol = in[0][0];
    d_band = current_symbol.Signal[0];

    // add new symbol to the symbol queue
    d_symbol_history.push_back(current_symbol.Prompt_I);

    d_symbol_counter++;  // counter for the processed symbols

    // Time Tags from signal source (optional feature)
    std::vector<gr::tag_t> tags_vec;
    this->get_tags_in_range(tags_vec, 0, this->nitems_read(0), this->nitems_read(0) + 1);  // telemetry decoder consumes symbols one-by-one
    if (!tags_vec.empty())
        {
            for (const auto &it : tags_vec)
                {
                    try
                        {
                            if (pmt::any_ref(it.value).type().hash_code() == typeid(const std::shared_ptr<GnssTime>).hash_code())
                                {
                                    const auto timetag = wht::any_cast<const std::shared_ptr<GnssTime>>(pmt::any_ref(it.value));
                                    // std::cout << "Old tow: " << d_current_timetag.tow_ms << " new tow: " << timetag->tow_ms << "\n";
                                    d_current_timetag = *timetag;
                                    d_valid_timetag = true;
                                }
                            else
                                {
                                    std::cout << "hash code not match\n";
                                }
                        }
                    catch (const wht::bad_any_cast &e)
                        {
                            std::cout << "msg Bad any_cast: " << e.what();
                        }
                }
        }
    else
        {
            if (d_valid_timetag == true)
                {
                    // propagate timetag to current symbol
                    // todo: tag rx_time is set only in the time channel. The tracking tag does not have valid rx_time (it is not required since it is associated to the current symbol)
                    // d_current_timetag.rx_time+=d_PRN_code_period_ms
                    const int64_t next_tow_ms = static_cast<int64_t>(d_current_timetag.tow_ms) + static_cast<int64_t>(d_PRN_code_period_ms);
                    if (next_tow_ms >= static_cast<int64_t>(galileo_tow::WEEK_MS))
                        {
                            d_current_timetag.week += static_cast<int>(next_tow_ms / static_cast<int64_t>(galileo_tow::WEEK_MS));
                        }
                    d_current_timetag.tow_ms = static_cast<int>(galileo_tow::wrap_ms(next_tow_ms));
                }
        }

    consume_each(1);
    d_flag_preamble = false;

    // check if there is a problem with the telemetry of the current satellite
    check_tlm_separation();

    // ******* frame sync ******************
    int32_t corr_value = 0;
    switch (d_stat)
        {
        case 0:  // no preamble information
            // correlate with preamble
            if (d_symbol_history.size() > d_required_symbols)
                {
                    // ******* preamble correlation ********
                    for (int32_t i = 0; i < d_samples_per_preamble; i++)
                        {
                            if (d_symbol_history[i] < 0.0)  // symbols clipping
                                {
                                    corr_value -= d_preamble_samples[i];
                                }
                            else
                                {
                                    corr_value += d_preamble_samples[i];
                                }
                        }
                    if (std::abs(corr_value) >= d_samples_per_preamble)
                        {
                            d_preamble_index = d_symbol_counter;  // record the preamble sample stamp
                            LOG(INFO) << "Preamble detection for Galileo satellite " << this->d_satellite << " in channel " << this->d_channel;
                            d_stat = 1;  // enter into frame pre-detection status
                        }
                }
            break;
        case 1:  // possible preamble lock
            // correlate with preamble
            if (d_symbol_history.size() > d_required_symbols)
                {
                    // ******* preamble correlation ********
                    for (int32_t i = 0; i < d_samples_per_preamble; i++)
                        {
                            if (d_symbol_history[i] < 0.0)  // symbols clipping
                                {
                                    corr_value -= d_preamble_samples[i];
                                }
                            else
                                {
                                    corr_value += d_preamble_samples[i];
                                }
                        }
                    if (std::abs(corr_value) >= d_samples_per_preamble)
                        {
                            // check preamble separation
                            const auto preamble_diff = static_cast<int32_t>(d_symbol_counter - d_preamble_index);
                            if (std::abs(preamble_diff - d_preamble_period_symbols) == 0)
                                {
                                    // try to decode frame
                                    DLOG(INFO) << "Starting page decoder for Galileo satellite " << this->d_satellite;
                                    d_preamble_index = d_symbol_counter;  // record the preamble sample stamp
                                    d_CRC_error_counter = 0;
                                    if (corr_value < 0)
                                        {
                                            d_flag_PLL_180_deg_phase_locked = true;
                                        }
                                    else
                                        {
                                            d_flag_PLL_180_deg_phase_locked = false;
                                        }
                                    d_stat = 2;
                                }
                            else
                                {
                                    if (preamble_diff > d_preamble_period_symbols)
                                        {
                                            d_stat = 0;  // start again
                                        }
                                }
                        }
                }
            break;
        case 2:  // preamble acquired
            if (d_symbol_counter == d_preamble_index + static_cast<uint64_t>(d_preamble_period_symbols))
                {
                    // call the decoder
                    // NEW Galileo page part is received
                    // 0. fetch the symbols into an array
                    if (d_flag_PLL_180_deg_phase_locked == false)  // normal PLL lock
                        {
                            for (uint32_t i = 0; i < d_frame_length_symbols; i++)
                                {
                                    d_page_part_symbols[i] = d_symbol_history[i + d_samples_per_preamble];  // because last symbol of the preamble is just received now!
                                }
                        }
                    else  // 180 deg. inverted carrier phase PLL lock
                        {
                            for (uint32_t i = 0; i < d_frame_length_symbols; i++)
                                {
                                    d_page_part_symbols[i] = -d_symbol_history[i + d_samples_per_preamble];  // because last symbol of the preamble is just received now!
                                }
                        }
                    switch (d_frame_type)
                        {
                        case 1:  // INAV
                            decode_INAV_word(d_page_part_symbols.data(), d_frame_length_symbols, current_symbol.CN0_dB_hz);
                            break;
                        case 2:  // FNAV
                            decode_FNAV_word(d_page_part_symbols.data(), d_frame_length_symbols, current_symbol.CN0_dB_hz);
                            break;
                        case 3:  // CNAV
                            if (current_symbol.fs != 0LL)
                                {
                                    const CnavPageReceptionTime page_reception_time = get_cnav_page_reception_time(current_symbol);
                                    decode_CNAV_word(current_symbol.Tracking_sample_counter / static_cast<uint64_t>(current_symbol.fs),
                                        page_reception_time,
                                        d_page_part_symbols.data(),
                                        d_frame_length_symbols,
                                        current_symbol.CN0_dB_hz);
                                }
                            break;
                        default:
                            return -1;
                            break;
                        }
                    bool crc_ok = (d_inav_nav.get_flag_CRC_test() || d_fnav_nav.get_flag_CRC_test() || d_cnav_nav.get_flag_CRC_test());
                    if (d_dump_crc_stats)
                        {
                            // update CRC statistics
                            d_Tlm_CRC_Stats->update_CRC_stats(crc_ok);
                        }

                    d_preamble_index = d_symbol_counter;  // record the preamble sample stamp (t_P)
                    if (crc_ok)
                        {
                            d_CRC_error_counter = 0;
                            d_flag_preamble = true;  // valid preamble indicator (initialized to false every work())
                            gr::thread::scoped_lock lock(d_setlock);
                            d_last_valid_preamble = d_symbol_counter;
                            if (!d_flag_frame_sync)
                                {
                                    d_flag_frame_sync = true;
                                    LOG(INFO) << "Successful frame synchronization in channel " << d_channel << " for satellite " << this->d_satellite
                                              << " at sample_counter=" << d_received_sample_counter;
                                }
                        }
                    else
                        {
                            d_CRC_error_counter++;
                            if (d_CRC_error_counter > CRC_ERROR_LIMIT)
                                {
                                    DLOG(INFO) << "Lost of frame sync SAT " << this->d_satellite;
                                    gr::thread::scoped_lock lock(d_setlock);
                                    d_flag_frame_sync = false;
                                    d_stat = 0;
                                    d_TOW_at_current_symbol_ms = 0;
                                    d_TOW_at_Preamble_ms = 0;
                                    d_TOW_week = GALILEO_TOW_MAP_INVALID_WEEK;
                                    d_galileo_week = GALILEO_TOW_MAP_INVALID_WEEK;
                                    d_galileo_week_valid = false;
                                    d_received_week = GALILEO_TOW_MAP_INVALID_WEEK;
                                    d_received_tow_ms = GALILEO_TOW_MAP_INVALID_TOW_MS;
                                    d_received_sample_counter = GALILEO_TOW_MAP_INVALID_SAMPLE_COUNTER;
                                    d_E6_TOW_set = false;
                                    d_valid_timetag = false;
                                    clear_galileo_tow_map_entry();
                                    d_fnav_nav.set_flag_TOW_set(false);
                                    d_inav_nav.set_flag_TOW_set(false);
                                }
                        }
                }
            break;
        }

    // UPDATE GNSS SYNCHRO DATA
    // 2. Add the telemetry decoder information
    if (this->d_flag_preamble == true)
        // update TOW at the preamble instant
        {
            switch (d_frame_type)
                {
                case 1:  // INAV
                    if (d_inav_nav.get_flag_TOW_set() == true)
                        {
                            if (d_inav_nav.is_TOW5_set() == true)  // page 5 arrived and decoded, so we are in the odd page (since Tow refers to the even page, we have to add 1 sec)
                                {
                                    // TOW_5 refers to the even preamble, but when we decode it we are in the odd part, so 1 second later plus the decoding delay
                                    d_TOW_at_Preamble_ms = galileo_tow::seconds_to_ms(d_inav_nav.get_TOW5());
                                    const int64_t decoder_delay_ms = galileo_tow::inav_current_symbol_delay_ms(d_required_symbols, d_PRN_code_period_ms);
                                    const uint32_t preamble_week = update_known_galileo_week(d_inav_nav.get_Galileo_week()) ? d_galileo_week : GALILEO_TOW_MAP_INVALID_WEEK;
                                    set_current_tow_from_preamble(preamble_week, d_TOW_at_Preamble_ms, decoder_delay_ms);
                                    d_inav_nav.set_TOW5_flag(false);
                                    if (d_there_are_e6_channels && !d_valid_timetag)
                                        {
                                            publish_current_galileo_tow_map_entry(current_symbol.Tracking_sample_counter);
                                        }
                                    // timetag debug
                                    if (d_valid_timetag == true)
                                        {
                                            uint32_t predicted_tow_at_preamble_ms = galileo_tow::floor_to_second_ms(galileo_tow::wrap_ms(static_cast<int64_t>(d_current_timetag.tow_ms) - decoder_delay_ms));
                                            std::cout << "TOW at PREAMBLE: " << d_TOW_at_Preamble_ms << " predicted TOW at preamble: " << predicted_tow_at_preamble_ms << " [ms]\n";
                                        }
                                }

                            else if (d_inav_nav.is_TOW6_set() == true)  // page 6 arrived and decoded, so we are in the odd page (since Tow refers to the even page, we have to add 1 sec)
                                {
                                    // TOW_6 refers to the even preamble, but when we decode it we are in the odd part, so 1 second later plus the decoding delay
                                    d_TOW_at_Preamble_ms = galileo_tow::seconds_to_ms(d_inav_nav.get_TOW6());
                                    const int64_t decoder_delay_ms = galileo_tow::inav_current_symbol_delay_ms(d_required_symbols, d_PRN_code_period_ms);
                                    set_current_tow_from_preamble(d_galileo_week_valid ? d_galileo_week : GALILEO_TOW_MAP_INVALID_WEEK, d_TOW_at_Preamble_ms, decoder_delay_ms);
                                    d_inav_nav.set_TOW6_flag(false);
                                    // timetag debug
                                    if (d_valid_timetag == true)
                                        {
                                            uint32_t predicted_tow_at_preamble_ms = galileo_tow::floor_to_second_ms(galileo_tow::wrap_ms(static_cast<int64_t>(d_current_timetag.tow_ms) - decoder_delay_ms));
                                            std::cout << "TOW at PREAMBLE: " << d_TOW_at_Preamble_ms << " predicted TOW at preamble: " << predicted_tow_at_preamble_ms << " [ms]\n";
                                        }
                                }
                            else if (d_inav_nav.is_TOW0_set() == true)  // page 0 arrived and decoded
                                {
                                    // TOW_0 refers to the even preamble, but when we decode it we are in the odd part, so 1 second later plus the decoding delay
                                    d_TOW_at_Preamble_ms = galileo_tow::seconds_to_ms(d_inav_nav.get_TOW0());
                                    const int64_t decoder_delay_ms = galileo_tow::inav_current_symbol_delay_ms(d_required_symbols, d_PRN_code_period_ms);
                                    const uint32_t preamble_week = update_known_galileo_week(d_inav_nav.get_Galileo_week()) ? d_galileo_week : GALILEO_TOW_MAP_INVALID_WEEK;
                                    set_current_tow_from_preamble(preamble_week, d_TOW_at_Preamble_ms, decoder_delay_ms);
                                    d_inav_nav.set_TOW0_flag(false);
                                    if (d_there_are_e6_channels && !d_valid_timetag)
                                        {
                                            publish_current_galileo_tow_map_entry(current_symbol.Tracking_sample_counter);
                                        }
                                    // timetag debug
                                    if (d_valid_timetag == true)
                                        {
                                            uint32_t predicted_tow_at_preamble_ms = galileo_tow::floor_to_second_ms(galileo_tow::wrap_ms(static_cast<int64_t>(d_current_timetag.tow_ms) - decoder_delay_ms));
                                            std::cout << "TOW at PREAMBLE: " << d_TOW_at_Preamble_ms << " predicted TOW at preamble: " << predicted_tow_at_preamble_ms << " [ms]\n";
                                        }
                                }
                            else
                                {
                                    // this page has no timing information
                                    advance_current_tow(d_PRN_code_period_ms);
                                }
                        }
                    publish_pending_reduced_ced();
                    if (d_enable_navdata_monitor && !d_nav_msg_packet.nav_message.empty())
                        {
                            d_nav_msg_packet.system = std::string(1, current_symbol.System);
                            d_nav_msg_packet.signal = std::string(current_symbol.Signal);
                            d_nav_msg_packet.prn = static_cast<int32_t>(current_symbol.PRN);
                            d_nav_msg_packet.tow_at_current_symbol_ms = static_cast<int32_t>(d_TOW_at_current_symbol_ms);
                            const std::shared_ptr<Nav_Message_Packet> tmp_obj = std::make_shared<Nav_Message_Packet>(d_nav_msg_packet);
                            this->message_port_pub(pmt::mp("Nav_msg_from_TLM"), pmt::make_any(tmp_obj));
                            d_nav_msg_packet.nav_message = "";
                        }
                    break;
                case 2:  // FNAV
                    if (d_fnav_nav.get_flag_TOW_set() == true)
                        {
                            if (d_fnav_nav.is_TOW1_set() == true)
                                {
                                    d_TOW_at_Preamble_ms = galileo_tow::seconds_to_ms(d_fnav_nav.get_TOW1());
                                    const int64_t decoder_delay_ms = galileo_tow::fnav_current_symbol_delay_ms(d_required_symbols);
                                    const uint32_t preamble_week = update_known_galileo_week(d_fnav_nav.get_WN1()) ? d_galileo_week : GALILEO_TOW_MAP_INVALID_WEEK;
                                    set_current_tow_from_preamble(preamble_week, d_TOW_at_Preamble_ms, decoder_delay_ms);
                                    d_fnav_nav.set_TOW1_flag(false);
                                    if (d_there_are_e6_channels && !d_valid_timetag)
                                        {
                                            publish_current_galileo_tow_map_entry(current_symbol.Tracking_sample_counter);
                                        }
                                }
                            else if (d_fnav_nav.is_TOW2_set() == true)
                                {
                                    d_TOW_at_Preamble_ms = galileo_tow::seconds_to_ms(d_fnav_nav.get_TOW2());
                                    const int64_t decoder_delay_ms = galileo_tow::fnav_current_symbol_delay_ms(d_required_symbols);
                                    const uint32_t preamble_week = update_known_galileo_week(d_fnav_nav.get_WN2()) ? d_galileo_week : GALILEO_TOW_MAP_INVALID_WEEK;
                                    set_current_tow_from_preamble(preamble_week, d_TOW_at_Preamble_ms, decoder_delay_ms);
                                    d_fnav_nav.set_TOW2_flag(false);
                                    if (d_there_are_e6_channels && !d_valid_timetag)
                                        {
                                            publish_current_galileo_tow_map_entry(current_symbol.Tracking_sample_counter);
                                        }
                                }
                            else if (d_fnav_nav.is_TOW3_set() == true)
                                {
                                    d_TOW_at_Preamble_ms = galileo_tow::seconds_to_ms(d_fnav_nav.get_TOW3());
                                    const int64_t decoder_delay_ms = galileo_tow::fnav_current_symbol_delay_ms(d_required_symbols);
                                    const uint32_t preamble_week = update_known_galileo_week(d_fnav_nav.get_WN3()) ? d_galileo_week : GALILEO_TOW_MAP_INVALID_WEEK;
                                    set_current_tow_from_preamble(preamble_week, d_TOW_at_Preamble_ms, decoder_delay_ms);
                                    d_fnav_nav.set_TOW3_flag(false);
                                    if (d_there_are_e6_channels && !d_valid_timetag)
                                        {
                                            publish_current_galileo_tow_map_entry(current_symbol.Tracking_sample_counter);
                                        }
                                }
                            else if (d_fnav_nav.is_TOW4_set() == true)
                                {
                                    d_TOW_at_Preamble_ms = galileo_tow::seconds_to_ms(d_fnav_nav.get_TOW4());
                                    const int64_t decoder_delay_ms = galileo_tow::fnav_current_symbol_delay_ms(d_required_symbols);
                                    set_current_tow_from_preamble(d_galileo_week_valid ? d_galileo_week : GALILEO_TOW_MAP_INVALID_WEEK, d_TOW_at_Preamble_ms, decoder_delay_ms);
                                    d_fnav_nav.set_TOW4_flag(false);
                                }
                            else
                                {
                                    advance_current_tow(static_cast<int64_t>(GALILEO_FNAV_CODES_PER_SYMBOL) * static_cast<int64_t>(GALILEO_E5A_CODE_PERIOD_MS));
                                }
                        }
                    if (d_enable_navdata_monitor && !d_nav_msg_packet.nav_message.empty())
                        {
                            d_nav_msg_packet.system = std::string(1, current_symbol.System);
                            d_nav_msg_packet.signal = std::string(current_symbol.Signal);
                            d_nav_msg_packet.prn = static_cast<int32_t>(current_symbol.PRN);
                            d_nav_msg_packet.tow_at_current_symbol_ms = static_cast<int32_t>(d_TOW_at_current_symbol_ms);
                            const std::shared_ptr<Nav_Message_Packet> tmp_obj = std::make_shared<Nav_Message_Packet>(d_nav_msg_packet);
                            this->message_port_pub(pmt::mp("Nav_msg_from_TLM"), pmt::make_any(tmp_obj));
                            d_nav_msg_packet.nav_message = "";
                        }
                    break;
                case 3:  // CNAV
                    if (d_valid_timetag == true)
                        {
                            uint32_t predicted_tow_at_preamble_ms = galileo_tow::floor_to_second_ms(galileo_tow::wrap_ms(d_current_timetag.tow_ms));
                            const int64_t decoder_delay_ms = (static_cast<int64_t>(d_required_symbols) + 1LL) * static_cast<int64_t>(d_PRN_code_period_ms);
                            d_TOW_at_Preamble_ms = predicted_tow_at_preamble_ms;
                            set_current_tow_from_preamble(d_current_timetag.week >= 0 ? static_cast<uint32_t>(d_current_timetag.week) : GALILEO_TOW_MAP_INVALID_WEEK, predicted_tow_at_preamble_ms, decoder_delay_ms);
                            if (d_E6_TOW_set == false)
                                {
                                    std::cout << " Sat PRN " << d_satellite.get_PRN() << " E6 TimeTag TOW at preamble: " << predicted_tow_at_preamble_ms
                                              << " [ms] d_TOW_at_current_symbol_ms: " << d_TOW_at_current_symbol_ms << " [ms]\n";
                                    d_E6_TOW_set = true;
                                }
                        }
                    else
                        {
                            if (d_received_week != GALILEO_TOW_MAP_INVALID_WEEK && d_received_tow_ms < galileo_tow::WEEK_MS && current_symbol.fs > 0LL)
                                {
                                    const int64_t diff = galileo_tow::sample_counter_delta(current_symbol.Tracking_sample_counter, d_received_sample_counter);
                                    const auto time_since_reference_ms = static_cast<int64_t>(static_cast<double>(diff) * 1000.0 / static_cast<double>(current_symbol.fs));
                                    const int64_t tow_delta_ms = time_since_reference_ms + static_cast<int64_t>(GALILEO_E6_CODE_PERIOD_MS);
                                    uint32_t projected_week = GALILEO_TOW_MAP_INVALID_WEEK;
                                    if (galileo_tow::week_after_delta(d_received_week, d_received_tow_ms, tow_delta_ms, projected_week))
                                        {
                                            d_TOW_week = projected_week;
                                            d_TOW_at_current_symbol_ms = galileo_tow::add_ms(d_received_tow_ms, tow_delta_ms);
                                            d_TOW_at_Preamble_ms = galileo_tow::floor_to_second_ms(d_TOW_at_current_symbol_ms);
                                            d_E6_TOW_set = true;
                                        }
                                }
                        }
                    if (d_enable_navdata_monitor && d_E6_TOW_set && !d_nav_msg_packet.nav_message.empty())
                        {
                            d_nav_msg_packet.system = std::string(1, current_symbol.System);
                            d_nav_msg_packet.signal = std::string(current_symbol.Signal);
                            d_nav_msg_packet.prn = static_cast<int32_t>(current_symbol.PRN);
                            d_nav_msg_packet.tow_at_current_symbol_ms = static_cast<int32_t>(d_TOW_at_current_symbol_ms);
                            const std::shared_ptr<Nav_Message_Packet> tmp_obj = std::make_shared<Nav_Message_Packet>(d_nav_msg_packet);
                            this->message_port_pub(pmt::mp("Nav_msg_from_TLM"), pmt::make_any(tmp_obj));
                            d_nav_msg_packet.nav_message = "";
                        }
                }
        }
    else  // if there is not a new preamble, we define the TOW of the current symbol
        {
            switch (d_frame_type)
                {
                case 1:  // INAV
                    if (d_inav_nav.get_flag_TOW_set() == true)
                        {
                            advance_current_tow(d_PRN_code_period_ms);
                        }
                    break;
                case 2:  // FNAV
                    if (d_fnav_nav.get_flag_TOW_set() == true)
                        {
                            advance_current_tow(d_PRN_code_period_ms);
                        }
                    break;
                case 3:  // CNAV
                    if (d_E6_TOW_set == true)
                        {
                            advance_current_tow(d_PRN_code_period_ms);
                        }
                    break;
                }
        }

    switch (d_frame_type)
        {
        case 1:  // INAV
            if (d_inav_nav.get_flag_TOW_set() == true)
                {
                    if (d_inav_nav.get_flag_GGTO() == true)  // all GGTO parameters arrived
                        {
                            d_delta_t = d_inav_nav.get_A0G() + d_inav_nav.get_A1G() * (static_cast<double>(d_TOW_at_current_symbol_ms) / 1000.0 - d_inav_nav.get_t0G() + 604800.0 * (std::fmod(static_cast<float>(d_inav_nav.get_Galileo_week() - d_inav_nav.get_WN0G()), 64.0)));
                        }
                    current_symbol.Flag_valid_word = true;
                }
            break;
        case 2:  // FNAV
            if (d_fnav_nav.get_flag_TOW_set() == true)
                {
                    current_symbol.Flag_valid_word = true;
                }
            break;
        case 3:  // CNAV
            if (d_E6_TOW_set == true)
                {
                    current_symbol.Flag_valid_word = true;
                }
            break;
        }

    if (current_symbol.Flag_valid_word == true)
        {
            current_symbol.TOW_at_current_symbol_ms = d_TOW_at_current_symbol_ms;
            // todo: Galileo to GPS time conversion should be moved to observable block.
            // current_symbol.TOW_at_current_symbol_ms -= d_delta_t;  // Galileo to GPS TOW

            if (d_flag_PLL_180_deg_phase_locked == true)
                {
                    // correct the accumulated phase for the Costas loop phase shift, if required
                    current_symbol.Carrier_phase_rads += GNSS_PI;
                    current_symbol.Flag_PLL_180_deg_phase_locked = true;
                }
            else
                {
                    current_symbol.Flag_PLL_180_deg_phase_locked = false;
                }

            if (d_dump == true)
                {
                    // MULTIPLEXED FILE RECORDING - Record results to file
                    try
                        {
                            write_value(d_dump_file, static_cast<double>(d_TOW_at_current_symbol_ms) / 1000.0);
                            write_value(d_dump_file, current_symbol.Tracking_sample_counter);
                            write_value(d_dump_file, static_cast<double>(d_TOW_at_Preamble_ms) / 1000.0);

                            switch (d_frame_type)
                                {
                                case 1:
                                    write_value(d_dump_file, current_symbol.Prompt_I > 0.0 ? 1 : -1);
                                    break;
                                case 2:
                                    write_value(d_dump_file, current_symbol.Prompt_Q > 0.0 ? 1 : -1);
                                    break;
                                case 3:
                                    write_value(d_dump_file, current_symbol.Prompt_I > 0.0 ? 1 : -1);
                                    break;
                                default:
                                    write_value(d_dump_file, 0);
                                    break;
                                }

                            write_value(d_dump_file, static_cast<int32_t>(current_symbol.PRN));
                        }
                    catch (const std::ofstream::failure &e)
                        {
                            LOG(WARNING) << "Exception writing navigation data dump file " << e.what();
                        }
                }

            // SEND TOW TO THE TRACKING BLOCK
            if (d_tow_to_trk)
                {
                    int32_t gal_week = 0;
                    if (d_TOW_week != GALILEO_TOW_MAP_INVALID_WEEK && d_TOW_week <= static_cast<uint32_t>(std::numeric_limits<int32_t>::max()))
                        {
                            gal_week = static_cast<int32_t>(d_TOW_week);
                        }
                    else
                        {
                            switch (d_frame_type)
                                {
                                case 1:
                                    gal_week = d_inav_nav.get_Galileo_week();
                                    break;
                                case 2:
                                    gal_week = d_fnav_nav.get_ephemeris().WN;
                                    break;
                                default:
                                    break;
                                }
                        }
                    const std::shared_ptr<TOW_to_trk> tmp_tow_obj = std::make_shared<TOW_to_trk>(
                        std::string(current_symbol.Signal),
                        d_channel,
                        d_TOW_at_current_symbol_ms,
                        current_symbol.Tracking_sample_counter,
                        gal_week, d_satellite.get_PRN());
                    this->message_port_pub(pmt::mp("telemetry_to_trk"), pmt::make_any(tmp_tow_obj));
                }

            // 3. Make the output (move the object contents to the GNURadio reserved memory)
            *out[0] = std::move(current_symbol);
            return 1;
        }
    return 0;
}
