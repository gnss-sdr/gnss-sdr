/*!
 * \file navic_l5_telemetry_decoder_gs.cc
 * \brief Implementation of a NavIC L5 LNAV data decoder block
 * \author Pradyumna Krishna, 2026. pradyumnakrishna(at)gmail.com
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
#include "navic_l5_telemetry_decoder_gs.h"
#include "NAVIC_L5.h"
#include "NAVIC_LNAV.h"
#include "display.h"
#include "gnss_sdr_make_unique.h"  // for std::make_unique in C++11
#include "gnss_synchro.h"
#include "navic_lnav_almanac.h"
#include "navic_lnav_ephemeris.h"
#include "navic_lnav_iono.h"
#include "navic_lnav_utc_model.h"
#include "tlm_utils.h"
#include "viterbi_decoder.h"
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>        // for make_any
#include <pmt/pmt_sugar.h>  // for mp
#include <cstddef>           // for size_t
#include <cstdlib>           // for abs
#include <exception>         // for exception
#include <iomanip>           // for setprecision
#include <iostream>          // for cout
#include <memory>            // for shared_ptr, make_shared
#include <utility>           // for std::move
#include <vector>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

#define CRC_ERROR_LIMIT 8


navic_l5_telemetry_decoder_gs_sptr
navic_l5_make_telemetry_decoder_gs(const Gnss_Satellite &satellite, const Tlm_Conf &conf)
{
    return navic_l5_telemetry_decoder_gs_sptr(new navic_l5_telemetry_decoder_gs(satellite, conf));
}


navic_l5_telemetry_decoder_gs::navic_l5_telemetry_decoder_gs(
    const Gnss_Satellite &satellite,
    const Tlm_Conf &conf) : gr::block("navic_l5_telemetry_decoder_gs",
                                gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
                                gr::io_signature::make(1, 1, sizeof(Gnss_Synchro))),
                            d_dump_filename(conf.dump_filename),
                            d_sample_counter(0),
                            d_preamble_index(0),
                            d_symbol_accumulator(0.0),
                            d_symbol_accumulator_counter(0),
                            d_raw_prompt_write_idx(0),
                            d_raw_prompt_count(0),
                            d_bit_phase_locked(false),
                            d_code_period_counter(0),
                            d_channel(0),
                            d_symbols_per_preamble(NAVIC_L5_PREAMBLE_LENGTH_SYMBOLS),
                            d_samples_per_preamble(NAVIC_L5_PREAMBLE_LENGTH_SYMBOLS),
                            d_preamble_period_samples(NAVIC_L5_PREAMBLE_PERIOD_SYMBOLS),
                            d_CRC_error_counter(0),
                            d_required_symbols(NAVIC_L5_SUBFRAME_SYMBOLS + NAVIC_L5_PREAMBLE_LENGTH_SYMBOLS),
                            d_stat(0),
                            d_last_valid_preamble(0),
                            d_symbol_duration_ms(NAVIC_L5_SYMBOL_PERIOD_MS),
                            d_TOW_at_Preamble_ms(0U),
                            d_TOW_at_current_symbol_ms(0U),
                            d_flag_TOW_set(false),
                            d_flag_frame_sync(false),
                            d_flag_preamble(false),
                            d_flag_valid_word(false),
                            d_flag_pll_180_deg_locked(false),
                            d_flag_bit_sync_detected(false),
                            d_sent_tlm_failed_msg(false),
                            d_dump(conf.dump),
                            d_dump_mat(conf.dump_mat),
                            d_remove_dat(conf.remove_dat),
                            d_enable_navdata_monitor(conf.enable_navdata_monitor),
                            d_dump_crc_stats(conf.dump_crc_stats)
{
    // prevent telemetry symbols accumulation in output buffers
    this->set_max_noutput_items(1);
    // Ephemeris data port out
    this->message_port_register_out(pmt::mp("telemetry"));
    // Control messages to tracking block
    this->message_port_register_out(pmt::mp("telemetry_to_trk"));

    if (d_enable_navdata_monitor)
        {
            // register nav message monitor out
            this->message_port_register_out(pmt::mp("Nav_msg_from_TLM"));
            d_nav_msg_packet.system = std::string("I");
            d_nav_msg_packet.signal = std::string("L5");
        }

    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    LOG(INFO) << "Initializing NavIC L5 Telemetry Decoding for satellite " << this->d_satellite;

    // Setting samples of preamble code
    for (int32_t i = 0; i < d_symbols_per_preamble; i++)
        {
            if (NAVIC_L5_PREAMBLE[i] == '1')
                {
                    d_preamble_samples[i] = 1;
                }
            else
                {
                    d_preamble_samples[i] = -1;
                }
        }

    d_symbol_history.set_capacity(d_required_symbols);

    // Raw prompt buffer: store enough 1ms samples for bit boundary search
    // Need at least (subframe + preamble) * 20 = (600+16)*20 = 12320 samples
    d_raw_prompt_buffer.resize(25000, 0.0F);  // Must hold 2+ subframes (24320 = 1216*20)

    // Instantiate the Viterbi decoder
    // NavIC L5: rate 1/2, K=7, G1=171o (=121 decimal), G2=133o (=91 decimal)
    const int32_t nn = 2;                               // Coding rate 1/n
    const int32_t KK = 7;                               // Constraint Length
    const int32_t mm = KK - 1;                          // Memory depth
    const std::array<int32_t, 2> g_encoder{{121, 91}};  // Polynomial G1=171o, G2=133o
    const int32_t d_datalength = (NAVIC_L5_SUBFRAME_DATA_SYMBOLS / nn) - mm;
    d_viterbi = std::make_unique<Viterbi_Decoder>(KK, nn, d_datalength, g_encoder);

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


navic_l5_telemetry_decoder_gs::~navic_l5_telemetry_decoder_gs()
{
    DLOG(INFO) << "NavIC L5 Telemetry decoder block (channel " << d_channel << ") destructor called.";
    size_t pos = 0;
    if (d_dump_file.is_open() == true)
        {
            pos = d_dump_file.tellp();
            try
                {
                    d_dump_file.close();
                }
            catch (const std::exception &ex)
                {
                    LOG(WARNING) << "Exception in destructor closing the dump file " << ex.what();
                }
            if (pos == 0)
                {
                    if (!tlm_remove_file(d_dump_filename))
                        {
                            LOG(WARNING) << "Error deleting temporary file";
                        }
                }
        }
    if (d_dump && (pos != 0) && d_dump_mat)
        {
            save_tlm_matfile(d_dump_filename);
            if (d_remove_dat)
                {
                    if (!tlm_remove_file(d_dump_filename))
                        {
                            LOG(WARNING) << "Error deleting temporary file";
                        }
                }
        }
}


void navic_l5_telemetry_decoder_gs::deinterleaver(int32_t rows, int32_t cols, const float *in, float *out)
{
    // NavIC ICD Section 5.3: "Data is written in columns and then, read in rows."
    // ENCODING at transmitter:
    //   FEC output (sequential) is written into cols (column-major): seq[k] -> matrix[k%rows][k/rows]
    //   Then read out in rows (row-major): transmitted[r*cols+c] = matrix[r][c]
    // DECODING at receiver:
    //   Received (row-major) -> fill matrix row by row: matrix[r][c] = received[r*cols+c]
    //   Read column by column: original[k] = matrix[k%rows][k/rows]
    //   i.e. original[k] = received[(k%rows)*cols + k/rows]
    for (int32_t k = 0; k < rows * cols; k++)
        {
            int32_t r = k % rows;   // row in the interleaver matrix
            int32_t c = k / rows;   // column in the interleaver matrix
            out[k] = in[r * cols + c];
        }
}


void navic_l5_telemetry_decoder_gs::decode_subframe(float *frame_symbols, double cn0)
{
    // frame_symbols contains 600 symbols: 16 preamble + 584 FEC-encoded data
    // Step 1: Extract the 584 FEC-encoded symbols (after preamble)
    float *data_symbols = &frame_symbols[NAVIC_L5_PREAMBLE_LENGTH_SYMBOLS];
    const int32_t data_length = NAVIC_L5_SUBFRAME_DATA_SYMBOLS;  // 584

    // Step 2: De-interleave (73 columns x 8 rows)
    std::vector<float> deinterleaved(data_length);
    deinterleaver(NAVIC_L5_INTERLEAVER_ROWS, NAVIC_L5_INTERLEAVER_COLS,
        data_symbols, deinterleaved.data());

    // Step 3: Viterbi decoding (rate 1/2, K=7)
    // Sign convention: test both polarities - if CRC fails consistently,
    // the sign may need to be flipped.
    // Current: pass symbols as-is (positive = bit 0 likely for the decoder)

    const int32_t decoded_length = data_length / 2;  // 292 bits (including 6 tail bits)
    std::vector<int32_t> decoded_bits(decoded_length);
    d_viterbi->decode(decoded_bits, deinterleaved);

    // Step 4: Convert decoded bits to string
    std::string data_bits;
    data_bits.reserve(decoded_length);
    for (int32_t i = 0; i < decoded_length; i++)
        {
            if (decoded_bits[i] > 0)
                {
                    data_bits.push_back('1');
                }
            else
                {
                    data_bits.push_back('0');
                }
        }

    if (d_enable_navdata_monitor)
        {
            d_nav_msg_packet.nav_message = data_bits;
        }

    // Debug: show first bits of decoded data and raw symbol signs
    std::string raw_signs;
    raw_signs.reserve(20);
    for (int j = 0; j < 20 && j < NAVIC_L5_PREAMBLE_LENGTH_SYMBOLS + 4; j++)
        {
            raw_signs.push_back(frame_symbols[j] >= 0 ? '+' : '-');
        }
    LOG(INFO) << "NavIC decode_subframe PRN " << d_satellite.get_PRN()
              << " decoded[0:40]=" << data_bits.substr(0, 40)
              << " raw_signs=" << raw_signs
              << " CN0=" << cn0;

    // Step 5: Pass decoded subframe to navigation message decoder
    // CRC-24Q check is performed inside decode_subframe
    d_nav.decode_subframe(data_bits);

    // Step 6: Check CRC result
    bool crc_ok = d_nav.get_flag_CRC_test();
    LOG(INFO) << "NavIC LNAV CRC " << (crc_ok ? "PASS" : "FAIL")
              << " ch" << d_channel << " PRN " << d_satellite.get_PRN();
    if (d_dump_crc_stats)
        {
            // update CRC statistics
            d_Tlm_CRC_Stats->update_CRC_stats(crc_ok);
        }

    // Step 7: Push the new navigation data to the queues
    if (d_nav.have_new_ephemeris() == true)
        {
            const std::shared_ptr<Navic_Lnav_Ephemeris> tmp_obj = std::make_shared<Navic_Lnav_Ephemeris>(d_nav.get_ephemeris());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            LOG(INFO) << "NAVIC LNAV Ephemeris have been received in channel " << d_channel << " from satellite " << d_satellite << " with CN0=" << cn0 << " dB-Hz";
#if __cplusplus == 201103L
            const int default_precision = std::cout.precision();
#else
            const auto default_precision{std::cout.precision()};
#endif
            std::cout << "New NavIC L5 LNAV message received in channel " << d_channel
                      << ": ephemeris from satellite " << d_satellite
                      << " with CN0=" << std::setprecision(2) << cn0 << std::setprecision(default_precision) << " dB-Hz" << std::endl;
            // ephemeris dump removed (was debug code)
        }
    if (d_nav.have_new_utc_model() == true)
        {
            const std::shared_ptr<Navic_Lnav_Utc_Model> tmp_obj = std::make_shared<Navic_Lnav_Utc_Model>(d_nav.get_utc_model());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            LOG(INFO) << "NAVIC LNAV UTC Model data have been received in channel " << d_channel << " from satellite " << d_satellite << " with CN0=" << cn0 << " dB-Hz";
#if __cplusplus == 201103L
            const int default_precision = std::cout.precision();
#else
            const auto default_precision{std::cout.precision()};
#endif
            std::cout << "New NavIC L5 LNAV UTC model message received in channel "
                      << d_channel
                      << ": UTC model parameters from satellite " << d_satellite
                      << " with CN0=" << std::setprecision(2) << cn0 << std::setprecision(default_precision)
                      << " dB-Hz" << std::endl;
        }
    if (d_nav.have_new_iono() == true)
        {
            const std::shared_ptr<Navic_Lnav_Iono> tmp_obj = std::make_shared<Navic_Lnav_Iono>(d_nav.get_iono());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            LOG(INFO) << "NAVIC LNAV Iono data have been received in channel " << d_channel << " from satellite " << d_satellite << " with CN0=" << cn0 << " dB-Hz";
#if __cplusplus == 201103L
            const int default_precision = std::cout.precision();
#else
            const auto default_precision{std::cout.precision()};
#endif
            std::cout << "New NavIC L5 LNAV Iono message received in channel " << d_channel
                      << ": Iono model parameters from satellite " << d_satellite
                      << " with CN0=" << std::setprecision(2) << cn0 << std::setprecision(default_precision)
                      << " dB-Hz" << std::endl;
        }
    if (d_nav.have_new_almanac() == true)
        {
            const std::shared_ptr<Navic_Lnav_Almanac> tmp_obj = std::make_shared<Navic_Lnav_Almanac>(d_nav.get_almanac());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            LOG(INFO) << "NAVIC LNAV Almanac data have been received in channel " << d_channel << " from satellite " << d_satellite << " with CN0=" << cn0 << " dB-Hz";
#if __cplusplus == 201103L
            const int default_precision = std::cout.precision();
#else
            const auto default_precision{std::cout.precision()};
#endif
            std::cout << "New NavIC L5 LNAV almanac received in channel " << d_channel
                      << " from satellite " << d_satellite
                      << " with CN0=" << std::setprecision(2) << cn0 << std::setprecision(default_precision)
                      << " dB-Hz" << std::endl;
        }
}


void navic_l5_telemetry_decoder_gs::set_satellite(const Gnss_Satellite &satellite)
{
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << "Setting decoder Finite State Machine to satellite " << d_satellite;
    DLOG(INFO) << "Navigation Satellite set to " << d_satellite;

    // Update satellite information for LNAV decoder
    d_nav.set_satellite_PRN(d_satellite.get_PRN());
}


void navic_l5_telemetry_decoder_gs::set_channel(int32_t channel)
{
    d_channel = channel;
    d_nav.i_channel_ID = channel;
    LOG(INFO) << "Navigation channel set to " << channel;
    // ############# ENABLE DATA FILE LOG #################
    if (d_dump == true)
        {
            if (d_dump_file.is_open() == false)
                {
                    try
                        {
                            d_dump_filename.append(std::to_string(d_channel));
                            d_dump_filename.append(".dat");
                            d_dump_file.exceptions(std::ofstream::failbit | std::ofstream::badbit);
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "Telemetry decoder dump enabled on channel " << d_channel << " Log file: " << d_dump_filename.c_str();
                        }
                    catch (const std::ofstream::failure &e)
                        {
                            LOG(WARNING) << "channel " << d_channel << ": exception opening NavIC TLM dump file. " << e.what();
                        }
                }
        }
    if (d_dump_crc_stats)
        {
            // set the channel number for the telemetry CRC statistics
            // disable the telemetry CRC statistics if there is a problem opening the output file
            d_dump_crc_stats = d_Tlm_CRC_Stats->set_channel(d_channel);
        }
}


void navic_l5_telemetry_decoder_gs::reset()
{
    d_last_valid_preamble = d_sample_counter;
    d_TOW_at_current_symbol_ms = 0;
    d_sent_tlm_failed_msg = false;
    d_flag_valid_word = false;
    d_flag_bit_sync_detected = false;
    d_symbol_accumulator = 0.0;
    d_symbol_accumulator_counter = 0;
    DLOG(INFO) << "NavIC L5 Telemetry decoder reset for satellite " << d_satellite;
}


int navic_l5_telemetry_decoder_gs::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    auto **out = reinterpret_cast<Gnss_Synchro **>(&output_items[0]);            // Get the output buffer pointer
    const auto **in = reinterpret_cast<const Gnss_Synchro **>(&input_items[0]);  // Get the input buffer pointer

    Gnss_Synchro current_symbol{};  // structure to save the synchronization information and send the output object to the next block
    // 1. Copy the current tracking output
    current_symbol = in[0][0];
    consume_each(1);
    d_flag_preamble = false;

    // 2. Symbol handling — tracker outputs 20ms symbols after preamble-based bit sync.
    d_code_period_counter++;

    if (!current_symbol.Flag_valid_symbol_output)
        {
            // Tracker hasn't achieved bit sync yet — pass through
            current_symbol.TOW_at_current_symbol_ms = d_TOW_at_current_symbol_ms;
            current_symbol.Flag_valid_word = false;
            current_symbol.Flag_PLL_180_deg_phase_locked = d_flag_pll_180_deg_locked;
            **out = current_symbol;
            return 1;
        }

    // Pre-seed preamble into empty buffer (mirrors GPS L1 CA)
    if (d_symbol_history.empty())
        {
            for (int32_t i = 0; i < d_samples_per_preamble; i++)
                {
                    if (current_symbol.Flag_PLL_180_deg_phase_locked == true)
                        {
                            d_symbol_history.push_back(static_cast<float>(-d_preamble_samples[i]));
                        }
                    else
                        {
                            d_symbol_history.push_back(static_cast<float>(d_preamble_samples[i]));
                        }
                    d_sample_counter++;
                }
        }
    d_symbol_history.push_back(current_symbol.Prompt_I);
    d_sample_counter++;

    // 3. Preamble search and frame sync (like GPS L1 CA)
    if (d_symbol_history.size() >= d_required_symbols)
        {
            int32_t corr_value = 0;
            int32_t preamble_diff = 0;

            // ******* preamble correlation ********
            // Check position [0] of the circular buffer (oldest entry)
            for (int32_t i = 0; i < d_samples_per_preamble; i++)
                {
                    if (d_symbol_history[i] < 0)
                        {
                            corr_value -= d_preamble_samples[i];
                        }
                    else
                        {
                            corr_value += d_preamble_samples[i];
                        }
                }

            // ******* frame sync (GPS L1 CA style: 2-state with CRC validation) ********
            const int32_t preamble_threshold = d_samples_per_preamble - 2;  // allow 1 mismatch
            switch (d_stat)
                {
                case 0:  // no preamble information — search
                    {
                        if (abs(corr_value) >= preamble_threshold)
                            {
                                d_preamble_index = d_sample_counter;
                                d_flag_pll_180_deg_locked = (corr_value < 0);

                                // Immediately try to decode — CRC validates
                                if (corr_value > 0)
                                    {
                                        for (uint32_t i = 0; i < NAVIC_L5_PREAMBLE_PERIOD_SYMBOLS; i++)
                                            d_subframe_symbols[i] = d_symbol_history[i];
                                    }
                                else
                                    {
                                        for (uint32_t i = 0; i < NAVIC_L5_PREAMBLE_PERIOD_SYMBOLS; i++)
                                            d_subframe_symbols[i] = -d_symbol_history[i];
                                    }

                                decode_subframe(d_subframe_symbols.data(), current_symbol.CN0_dB_hz);

                                if (d_nav.get_flag_CRC_test())
                                    {
                                        d_CRC_error_counter = 0;
                                        d_flag_preamble = true;
                                        if (!d_flag_frame_sync)
                                            {
                                                d_flag_frame_sync = true;
                                                DLOG(INFO) << "NavIC LNAV frame sync found for SAT " << this->d_satellite;
                                            }
                                        d_stat = 1;  // preamble acquired
                                    }
                            }
                        break;
                    }
                case 1:  // preamble acquired — decode every 600 symbols
                    {
                        if (d_sample_counter >= d_preamble_index + static_cast<uint64_t>(d_preamble_period_samples))
                            {
                                d_preamble_index = d_sample_counter;

                                if (corr_value > 0)
                                    {
                                        d_flag_pll_180_deg_locked = false;
                                        for (uint32_t i = 0; i < NAVIC_L5_PREAMBLE_PERIOD_SYMBOLS; i++)
                                            d_subframe_symbols[i] = d_symbol_history[i];
                                    }
                                else
                                    {
                                        d_flag_pll_180_deg_locked = true;
                                        for (uint32_t i = 0; i < NAVIC_L5_PREAMBLE_PERIOD_SYMBOLS; i++)
                                            d_subframe_symbols[i] = -d_symbol_history[i];
                                    }

                                decode_subframe(d_subframe_symbols.data(), current_symbol.CN0_dB_hz);

                                if (d_nav.get_flag_CRC_test())
                                    {
                                        d_CRC_error_counter = 0;
                                        d_flag_preamble = true;
                                        if (!d_flag_frame_sync)
                                            {
                                                d_flag_frame_sync = true;
                                                DLOG(INFO) << "NavIC LNAV frame sync found for SAT " << this->d_satellite;
                                            }
                                    }
                                else
                                    {
                                        d_CRC_error_counter++;
                                        if (d_CRC_error_counter > CRC_ERROR_LIMIT)
                                            {
                                                DLOG(INFO) << "NavIC LNAV frame sync lost for SAT " << this->d_satellite;
                                                d_flag_frame_sync = false;
                                                d_stat = 0;
                                                d_TOW_at_current_symbol_ms = 0;
                                                d_TOW_at_Preamble_ms = 0;
                                                d_CRC_error_counter = 0;
                                                d_flag_TOW_set = false;
                                            }
                                    }
                            }
                        break;
                    }
                }
        }

    // UPDATE GNSS SYNCHRO DATA
    // 4. Add the telemetry decoder information
    if (d_flag_preamble == true)
        {
            const Navic_Lnav_Ephemeris eph = d_nav.get_ephemeris();
            if (eph.tow != 0)
                {
                    // TOWC * 12 = start of NEXT subframe (same convention as GPS HOW TOW)
                    d_TOW_at_current_symbol_ms = static_cast<uint32_t>(eph.tow * 1000.0);
                    d_TOW_at_Preamble_ms = d_TOW_at_current_symbol_ms;
                    d_last_valid_preamble = d_sample_counter;
                    d_flag_TOW_set = true;
                    d_flag_valid_word = true;
                }
        }
    else
        {
            if (d_flag_TOW_set == true)
                {
                    d_TOW_at_current_symbol_ms += d_symbol_duration_ms;  // +20ms per symbol
                }
        }

    // 5. Output: Flag_valid_word=true for every 1ms output once TOW is set.
    // The observables block interpolates TOW at its 20ms epoch boundaries.
    if (d_flag_TOW_set == true)
        {
            current_symbol.TOW_at_current_symbol_ms = d_TOW_at_current_symbol_ms;
            current_symbol.Flag_valid_word = true;
        }
    current_symbol.Flag_PLL_180_deg_phase_locked = d_flag_pll_180_deg_locked;

    if (d_flag_valid_word == true && d_dump == true)
        {
            // MULTIPLEXED FILE RECORDING - Record results to file
            try
                {
                    double tmp_double;
                    uint64_t tmp_ulong_int;
                    int32_t tmp_int;
                    tmp_double = static_cast<double>(d_TOW_at_current_symbol_ms) / 1000.0;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                    tmp_ulong_int = current_symbol.Tracking_sample_counter;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_ulong_int), sizeof(uint64_t));
                    tmp_double = static_cast<double>(d_TOW_at_Preamble_ms) / 1000.0;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                    tmp_int = (current_symbol.Prompt_I > 0.0 ? 1 : -1);
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_int), sizeof(int32_t));
                    tmp_int = static_cast<int32_t>(current_symbol.PRN);
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_int), sizeof(int32_t));
                }
            catch (const std::ofstream::failure &e)
                {
                    LOG(WARNING) << "Exception writing Telemetry NavIC L5 dump file " << e.what();
                }
        }

    // 6. Make the output (move the object contents to the GNURadio reserved memory)
    *out[0] = std::move(current_symbol);
    return 1;
}
