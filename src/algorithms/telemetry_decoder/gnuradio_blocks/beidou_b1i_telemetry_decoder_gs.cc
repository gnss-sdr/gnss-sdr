/*!
 * \file beidou_b1i_telemetry_decoder_gs.cc
 * \brief Implementation of a BEIDOU BI1 DNAV data decoder block
 * \note Code added as part of GSoC 2018 program
 * \author Damian Miralles, 2018. dmiralles2009(at)gmail.com
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */
#include "beidou_b1i_telemetry_decoder_gs.h"
#include "Beidou_B1I.h"
#include "Beidou_DNAV.h"
#include "beidou_dnav_almanac.h"
#include "beidou_dnav_ephemeris.h"
#include "beidou_dnav_iono.h"
#include "beidou_dnav_utc_model.h"
#include "display.h"
#include "gnss_synchro.h"
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>        // for make_any
#include <pmt/pmt_sugar.h>  // for mp
#include <cstdlib>          // for abs
#include <exception>        // for exception
#include <iostream>         // for cout
#include <memory>           // for shared_ptr, make_shared

#define CRC_ERROR_LIMIT 8


beidou_b1i_telemetry_decoder_gs_sptr
beidou_b1i_make_telemetry_decoder_gs(const Gnss_Satellite &satellite, bool dump)
{
    return beidou_b1i_telemetry_decoder_gs_sptr(new beidou_b1i_telemetry_decoder_gs(satellite, dump));
}

beidou_b1i_telemetry_decoder_gs::beidou_b1i_telemetry_decoder_gs(
    const Gnss_Satellite &satellite,
    bool dump) : gr::block("beidou_b1i_telemetry_decoder_gs",
                     gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
                     gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    // prevent telemetry symbols accumulation in output buffers
    this->set_max_noutput_items(1);
    // Ephemeris data port out
    this->message_port_register_out(pmt::mp("telemetry"));
    // Control messages to tracking block
    this->message_port_register_out(pmt::mp("telemetry_to_trk"));
    // initialize internal vars
    d_dump = dump;
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    LOG(INFO) << "Initializing BeiDou B1I Telemetry Decoding for satellite " << this->d_satellite;

    d_symbol_duration_ms = BEIDOU_B1I_TELEMETRY_SYMBOLS_PER_BIT * BEIDOU_B1I_CODE_PERIOD_MS;
    d_symbols_per_preamble = BEIDOU_DNAV_PREAMBLE_LENGTH_SYMBOLS;
    d_samples_per_preamble = BEIDOU_DNAV_PREAMBLE_LENGTH_SYMBOLS;
    d_preamble_period_samples = BEIDOU_DNAV_PREAMBLE_PERIOD_SYMBOLS;

    // Setting samples of preamble code
    for (int32_t i = 0; i < d_symbols_per_preamble; i++)
        {
            if (BEIDOU_DNAV_PREAMBLE.at(i) == '1')
                {
                    d_preamble_samples[i] = 1;
                }
            else
                {
                    d_preamble_samples[i] = -1;
                }
        }

    d_required_symbols = BEIDOU_DNAV_SUBFRAME_SYMBOLS + d_samples_per_preamble;
    d_symbol_history.set_capacity(d_required_symbols);

    d_last_valid_preamble = 0;
    d_sent_tlm_failed_msg = false;
    d_flag_valid_word = false;
    // Generic settings
    d_sample_counter = 0;
    d_stat = 0;
    d_preamble_index = 0;
    d_flag_frame_sync = false;
    d_TOW_at_current_symbol_ms = 0U;
    d_TOW_at_Preamble_ms = 0U;
    Flag_valid_word = false;
    d_CRC_error_counter = 0;
    d_flag_preamble = false;
    d_channel = 0;
    flag_SOW_set = false;
}


beidou_b1i_telemetry_decoder_gs::~beidou_b1i_telemetry_decoder_gs()
{
    if (d_dump_file.is_open() == true)
        {
            try
                {
                    d_dump_file.close();
                }
            catch (const std::exception &ex)
                {
                    LOG(WARNING) << "Exception in destructor closing the dump file " << ex.what();
                }
        }
}


void beidou_b1i_telemetry_decoder_gs::decode_bch15_11_01(const int32_t *bits, std::array<int32_t, 15> &decbits)
{
    int32_t bit, err;
    std::array<int32_t, 4> reg{1, 1, 1, 1};
    const std::array<int32_t, 15> errind{14, 13, 10, 12, 6, 9, 4, 11, 0, 5, 7, 8, 1, 3, 2};

    for (uint32_t i = 0; i < 15; i++)
        {
            decbits[i] = bits[i];
        }

    for (uint32_t i = 0; i < 15; i++)
        {
            bit = reg[3];
            reg[3] = reg[2];
            reg[2] = reg[1];
            reg[1] = reg[0];
            reg[0] = bits[i] * bit;
            reg[1] *= bit;
        }

    err = errind[reg[0] + reg[1] * 2 + reg[2] * 4 + reg[3] * 8];

    if (err > 0 and err < 16)
        {
            decbits[err - 1] *= -1;
        }
}


void beidou_b1i_telemetry_decoder_gs::decode_word(
    int32_t word_counter,
    const float *enc_word_symbols,
    int32_t *dec_word_symbols)
{
    std::array<int32_t, 30> bitsbch{};
    std::array<int32_t, 15> first_branch{};
    std::array<int32_t, 15> second_branch{};

    if (word_counter == 1)
        {
            for (uint32_t j = 0; j < 30; j++)
                {
                    dec_word_symbols[j] = static_cast<int32_t>(enc_word_symbols[j] > 0) ? (1) : (-1);
                }
        }
    else
        {
            for (uint32_t r = 0; r < 2; r++)
                {
                    for (uint32_t c = 0; c < 15; c++)
                        {
                            bitsbch[r * 15 + c] = static_cast<int32_t>(enc_word_symbols[c * 2 + r] > 0) ? (1) : (-1);
                        }
                }

            decode_bch15_11_01(&bitsbch[0], first_branch);
            decode_bch15_11_01(&bitsbch[15], second_branch);

            for (uint32_t j = 0; j < 11; j++)
                {
                    dec_word_symbols[j] = first_branch[j];
                    dec_word_symbols[j + 11] = second_branch[j];
                }

            for (uint32_t j = 0; j < 4; j++)
                {
                    dec_word_symbols[j + 22] = first_branch[11 + j];
                    dec_word_symbols[j + 26] = second_branch[11 + j];
                }
        }
}


void beidou_b1i_telemetry_decoder_gs::decode_subframe(float *frame_symbols)
{
    // 1. Transform from symbols to bits
    std::string data_bits;
    std::array<int32_t, 30> dec_word_bits{};

    // Decode each word in subframe
    for (uint32_t ii = 0; ii < BEIDOU_DNAV_WORDS_SUBFRAME; ii++)
        {
            // decode the word
            decode_word((ii + 1), &frame_symbols[ii * 30], dec_word_bits.data());

            // Save word to string format
            for (uint32_t jj = 0; jj < (BEIDOU_DNAV_WORD_LENGTH_BITS); jj++)
                {
                    data_bits.push_back((dec_word_bits[jj] > 0) ? ('1') : ('0'));
                }
        }

    if (d_satellite.get_PRN() > 0 and d_satellite.get_PRN() < 6)
        {
            d_nav.d2_subframe_decoder(data_bits);
        }
    else
        {
            d_nav.d1_subframe_decoder(data_bits);
        }

    // 3. Check operation executed correctly
    if (d_nav.flag_crc_test == true)
        {
            DLOG(INFO) << "BeiDou DNAV CRC correct in channel " << d_channel
                       << " from satellite " << d_satellite;
        }
    else
        {
            DLOG(INFO) << "BeiDou DNAV CRC error in channel " << d_channel
                       << " from satellite " << d_satellite;
        }
    // 4. Push the new navigation data to the queues
    if (d_nav.have_new_ephemeris() == true)
        {
            // get object for this SV (mandatory)
            std::shared_ptr<Beidou_Dnav_Ephemeris> tmp_obj = std::make_shared<Beidou_Dnav_Ephemeris>(d_nav.get_ephemeris());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            LOG(INFO) << "BEIDOU DNAV Ephemeris have been received in channel" << d_channel << " from satellite " << d_satellite;
            std::cout << TEXT_YELLOW << "New BEIDOU B1I DNAV message received in channel " << d_channel << ": ephemeris from satellite " << d_satellite << TEXT_RESET << std::endl;
        }
    if (d_nav.have_new_utc_model() == true)
        {
            // get object for this SV (mandatory)
            std::shared_ptr<Beidou_Dnav_Utc_Model> tmp_obj = std::make_shared<Beidou_Dnav_Utc_Model>(d_nav.get_utc_model());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            LOG(INFO) << "BEIDOU DNAV UTC Model have been received in channel" << d_channel << " from satellite " << d_satellite;
            std::cout << TEXT_YELLOW << "New BEIDOU B1I DNAV utc model message received in channel " << d_channel << ": UTC model parameters from satellite " << d_satellite << TEXT_RESET << std::endl;
        }
    if (d_nav.have_new_iono() == true)
        {
            // get object for this SV (mandatory)
            std::shared_ptr<Beidou_Dnav_Iono> tmp_obj = std::make_shared<Beidou_Dnav_Iono>(d_nav.get_iono());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            LOG(INFO) << "BEIDOU DNAV Iono have been received in channel" << d_channel << " from satellite " << d_satellite;
            std::cout << TEXT_YELLOW << "New BEIDOU B1I DNAV Iono message received in channel " << d_channel << ": Iono model parameters from satellite " << d_satellite << TEXT_RESET << std::endl;
        }
    if (d_nav.have_new_almanac() == true)
        {
            // uint32_t slot_nbr = d_nav.i_alm_satellite_PRN;
            // std::shared_ptr<Beidou_Dnav_Almanac> tmp_obj = std::make_shared<Beidou_Dnav_Almanac>(d_nav.get_almanac(slot_nbr));
            // this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            LOG(INFO) << "BEIDOU DNAV Almanac have been received in channel" << d_channel << " from satellite " << d_satellite << std::endl;
            std::cout << TEXT_YELLOW << "New BEIDOU B1I DNAV almanac received in channel " << d_channel << " from satellite " << d_satellite << TEXT_RESET << std::endl;
        }
}


void beidou_b1i_telemetry_decoder_gs::set_satellite(const Gnss_Satellite &satellite)
{
    uint32_t sat_prn = 0;
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << "Setting decoder Finite State Machine to satellite " << d_satellite;
    DLOG(INFO) << "Navigation Satellite set to " << d_satellite;

    // Update satellite information for DNAV decoder
    sat_prn = d_satellite.get_PRN();
    d_nav.i_satellite_PRN = sat_prn;
    d_nav.i_signal_type = 1;  // BDS: data source (0:unknown,1:B1I,2:B1Q,3:B2I,4:B2Q,5:B3I,6:B3Q)

    // Update tel dec parameters for D2 NAV Messages
    if (sat_prn > 0 and sat_prn < 6)
        {
            d_symbols_per_preamble = BEIDOU_DNAV_PREAMBLE_LENGTH_SYMBOLS;
            d_samples_per_preamble = BEIDOU_DNAV_PREAMBLE_LENGTH_SYMBOLS;
            d_preamble_period_samples = BEIDOU_DNAV_PREAMBLE_PERIOD_SYMBOLS;

            // Setting samples of preamble code
            for (int32_t i = 0; i < d_symbols_per_preamble; i++)
                {
                    if (BEIDOU_DNAV_PREAMBLE.at(i) == '1')
                        {
                            d_preamble_samples[i] = 1;
                        }
                    else
                        {
                            d_preamble_samples[i] = -1;
                        }
                }

            d_symbol_duration_ms = BEIDOU_B1I_GEO_TELEMETRY_SYMBOLS_PER_BIT * BEIDOU_B1I_CODE_PERIOD_MS;
            d_required_symbols = BEIDOU_DNAV_SUBFRAME_SYMBOLS + d_samples_per_preamble;
            d_symbol_history.set_capacity(d_required_symbols);
        }
    else
        {
            // back to normal satellites
            d_symbol_duration_ms = BEIDOU_B1I_TELEMETRY_SYMBOLS_PER_BIT * BEIDOU_B1I_CODE_PERIOD_MS;
            d_symbols_per_preamble = BEIDOU_DNAV_PREAMBLE_LENGTH_SYMBOLS;
            d_samples_per_preamble = BEIDOU_DNAV_PREAMBLE_LENGTH_SYMBOLS;
            d_preamble_period_samples = BEIDOU_DNAV_PREAMBLE_PERIOD_SYMBOLS;

            // Setting samples of preamble code
            for (int32_t i = 0; i < d_symbols_per_preamble; i++)
                {
                    if (BEIDOU_DNAV_PREAMBLE.at(i) == '1')
                        {
                            d_preamble_samples[i] = 1;
                        }
                    else
                        {
                            d_preamble_samples[i] = -1;
                        }
                }

            d_required_symbols = BEIDOU_DNAV_SUBFRAME_SYMBOLS + d_samples_per_preamble;
            d_symbol_history.set_capacity(d_required_symbols);
        }
}


void beidou_b1i_telemetry_decoder_gs::set_channel(int32_t channel)
{
    d_channel = channel;
    LOG(INFO) << "Navigation channel set to " << channel;
    // ############# ENABLE DATA FILE LOG #################
    if (d_dump == true)
        {
            if (d_dump_file.is_open() == false)
                {
                    try
                        {
                            d_dump_filename = "telemetry";
                            d_dump_filename.append(std::to_string(d_channel));
                            d_dump_filename.append(".dat");
                            d_dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "Telemetry decoder dump enabled on channel " << d_channel << " Log file: " << d_dump_filename.c_str();
                        }
                    catch (const std::ifstream::failure &e)
                        {
                            LOG(WARNING) << "channel " << d_channel << ": exception opening Beidou TLM dump file. " << e.what();
                        }
                }
        }
}

void beidou_b1i_telemetry_decoder_gs::reset()
{
    d_last_valid_preamble = d_sample_counter;
    d_TOW_at_current_symbol_ms = 0;
    d_sent_tlm_failed_msg = false;
    d_flag_valid_word = false;
    DLOG(INFO) << "Beidou B1I Telemetry decoder reset for satellite " << d_satellite;
    return;
}

int beidou_b1i_telemetry_decoder_gs::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    int32_t corr_value = 0;
    int32_t preamble_diff = 0;

    auto **out = reinterpret_cast<Gnss_Synchro **>(&output_items[0]);            // Get the output buffer pointer
    const auto **in = reinterpret_cast<const Gnss_Synchro **>(&input_items[0]);  // Get the input buffer pointer

    Gnss_Synchro current_symbol{};  // structure to save the synchronization information and send the output object to the next block
    // 1. Copy the current tracking output
    current_symbol = in[0][0];
    d_symbol_history.push_back(current_symbol.Prompt_I);  // add new symbol to the symbol queue
    d_sample_counter++;                                   // count for the processed samples
    consume_each(1);
    d_flag_preamble = false;

    if (d_symbol_history.size() >= d_required_symbols)
        {
            // ******* preamble correlation ********
            for (int32_t i = 0; i < d_samples_per_preamble; i++)
                {
                    if (d_symbol_history[i] < 0)  // symbols clipping
                        {
                            corr_value -= d_preamble_samples[i];
                        }
                    else
                        {
                            corr_value += d_preamble_samples[i];
                        }
                }
        }
    // ******* frame sync ******************
    if (d_stat == 0)  // no preamble information
        {
            if (abs(corr_value) >= d_samples_per_preamble)
                {
                    // Record the preamble sample stamp
                    d_preamble_index = d_sample_counter;
                    DLOG(INFO) << "Preamble detection for BEIDOU B1I SAT " << this->d_satellite;
                    // Enter into frame pre-detection status
                    d_stat = 1;
                }
        }
    else if (d_stat == 1)  // possible preamble lock
        {
            if (abs(corr_value) >= d_samples_per_preamble)
                {
                    // check preamble separation
                    preamble_diff = static_cast<int32_t>(d_sample_counter - d_preamble_index);
                    if (abs(preamble_diff - d_preamble_period_samples) == 0)
                        {
                            // try to decode frame
                            DLOG(INFO) << "Starting BeiDou DNAV frame decoding for BeiDou B1I SAT " << this->d_satellite;
                            d_preamble_index = d_sample_counter;  // record the preamble sample stamp


                            d_stat = 2;

                            // ******* SAMPLES TO SYMBOLS *******
                            if (corr_value > 0)  // normal PLL lock
                                {
                                    for (uint32_t i = 0; i < BEIDOU_DNAV_PREAMBLE_PERIOD_SYMBOLS; i++)
                                        {
                                            d_subframe_symbols[i] = d_symbol_history.at(i);
                                        }
                                }
                            else  // 180 deg. inverted carrier phase PLL lock
                                {
                                    for (uint32_t i = 0; i < BEIDOU_DNAV_PREAMBLE_PERIOD_SYMBOLS; i++)
                                        {
                                            d_subframe_symbols[i] = -d_symbol_history.at(i);
                                        }
                                }

                            // call the decoder
                            decode_subframe(d_subframe_symbols.data());

                            if (d_nav.flag_crc_test == true)
                                {
                                    d_CRC_error_counter = 0;
                                    d_flag_preamble = true;               // valid preamble indicator (initialized to false every work())
                                    d_preamble_index = d_sample_counter;  // record the preamble sample stamp (t_P)
                                    if (!d_flag_frame_sync)
                                        {
                                            d_flag_frame_sync = true;
                                            DLOG(INFO) << "BeiDou DNAV frame sync found for SAT " << this->d_satellite;
                                        }
                                }
                            else
                                {
                                    d_CRC_error_counter++;
                                    d_preamble_index = d_sample_counter;  // record the preamble sample stamp
                                    if (d_CRC_error_counter > CRC_ERROR_LIMIT)
                                        {
                                            DLOG(INFO) << "BeiDou DNAV frame sync lost for SAT " << this->d_satellite;
                                            d_flag_frame_sync = false;
                                            d_stat = 0;
                                            flag_SOW_set = false;
                                        }
                                }
                        }
                    else
                        {
                            if (preamble_diff > d_preamble_period_samples)
                                {
                                    d_stat = 0;  // start again
                                }
                            DLOG(INFO) << "Failed BeiDou DNAV frame decoding for BeiDou B1I SAT " << this->d_satellite;
                        }
                }
        }
    else if (d_stat == 2)  // preamble acquired
        {
            if (d_sample_counter == d_preamble_index + static_cast<uint64_t>(d_preamble_period_samples))
                {
                    // ******* SAMPLES TO SYMBOLS *******
                    if (corr_value > 0)  // normal PLL lock
                        {
                            for (uint32_t i = 0; i < BEIDOU_DNAV_PREAMBLE_PERIOD_SYMBOLS; i++)
                                {
                                    d_subframe_symbols[i] = d_symbol_history.at(i);
                                }
                        }
                    else  // 180 deg. inverted carrier phase PLL lock
                        {
                            for (uint32_t i = 0; i < BEIDOU_DNAV_PREAMBLE_PERIOD_SYMBOLS; i++)
                                {
                                    d_subframe_symbols[i] = -d_symbol_history.at(i);
                                }
                        }

                    // call the decoder
                    decode_subframe(d_subframe_symbols.data());

                    if (d_nav.flag_crc_test == true)
                        {
                            d_CRC_error_counter = 0;
                            d_flag_preamble = true;               // valid preamble indicator (initialized to false every work())
                            d_preamble_index = d_sample_counter;  // record the preamble sample stamp (t_P)
                            if (!d_flag_frame_sync)
                                {
                                    d_flag_frame_sync = true;
                                    DLOG(INFO) << "BeiDou DNAV frame sync found for SAT " << this->d_satellite;
                                }
                        }
                    else
                        {
                            d_CRC_error_counter++;
                            d_preamble_index = d_sample_counter;  // record the preamble sample stamp
                            if (d_CRC_error_counter > CRC_ERROR_LIMIT)
                                {
                                    DLOG(INFO) << "BeiDou DNAV frame sync lost for SAT " << this->d_satellite;
                                    d_flag_frame_sync = false;
                                    d_stat = 0;
                                    flag_SOW_set = false;
                                }
                        }
                }
        }
    // UPDATE GNSS SYNCHRO DATA
    // 2. Add the telemetry decoder information
    if (this->d_flag_preamble == true and d_nav.flag_new_SOW_available == true)
        // update TOW at the preamble instant
        {
            // Reporting sow as gps time of week
            d_TOW_at_Preamble_ms = static_cast<uint32_t>((d_nav.d_SOW + BEIDOU_DNAV_BDT2GPST_LEAP_SEC_OFFSET) * 1000.0);
            // check TOW update consistency
            uint32_t last_d_TOW_at_current_symbol_ms = d_TOW_at_current_symbol_ms;
            // compute new TOW
            d_TOW_at_current_symbol_ms = d_TOW_at_Preamble_ms + d_required_symbols * d_symbol_duration_ms;
            flag_SOW_set = true;
            d_nav.flag_new_SOW_available = false;

            if (last_d_TOW_at_current_symbol_ms != 0 and abs(static_cast<int64_t>(d_TOW_at_current_symbol_ms) - int64_t(last_d_TOW_at_current_symbol_ms)) > static_cast<int64_t>(d_symbol_duration_ms))
                {
                    LOG(INFO) << "Warning: BEIDOU B1I TOW update in ch " << d_channel
                              << " does not match the TLM TOW counter " << static_cast<int64_t>(d_TOW_at_current_symbol_ms) - int64_t(last_d_TOW_at_current_symbol_ms) << " ms \n";

                    d_TOW_at_current_symbol_ms = 0;
                    d_flag_valid_word = false;
                }
            else
                {
                    d_last_valid_preamble = d_sample_counter;
                    d_flag_valid_word = true;
                }
        }
    else
        {
            if (d_flag_valid_word)
                {
                    d_TOW_at_current_symbol_ms += d_symbol_duration_ms;
                    if (current_symbol.Flag_valid_symbol_output == false)
                        {
                            d_flag_valid_word = false;
                        }
                }
        }

    if (d_flag_valid_word == true)
        {
            current_symbol.TOW_at_current_symbol_ms = d_TOW_at_current_symbol_ms;
            current_symbol.Flag_valid_word = d_flag_valid_word;

            if (d_dump == true)
                {
                    // MULTIPLEXED FILE RECORDING - Record results to file
                    try
                        {
                            double tmp_double;
                            uint64_t tmp_ulong_int;
                            tmp_double = static_cast<double>(d_TOW_at_current_symbol_ms) / 1000.0;
                            d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                            tmp_ulong_int = current_symbol.Tracking_sample_counter;
                            d_dump_file.write(reinterpret_cast<char *>(&tmp_ulong_int), sizeof(uint64_t));
                            tmp_double = static_cast<double>(d_TOW_at_Preamble_ms) / 1000.0;
                            d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                        }
                    catch (const std::ifstream::failure &e)
                        {
                            LOG(WARNING) << "Exception writing Telemetry GPS L5 dump file " << e.what();
                        }
                }

            // 3. Make the output (copy the object contents to the GNURadio reserved memory)
            *out[0] = current_symbol;
            return 1;
        }
    return 0;
}
