/*!
 * \file galileo_telemetry_decoder_gs.cc
 * \brief Implementation of a Galileo unified INAV and FNAV message demodulator block
 * \author Javier Arribas 2018. jarribas(at)cttc.es
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#include "galileo_telemetry_decoder_gs.h"
#include "Galileo_E1.h"   // for GALILEO_E1_CODE_PERIOD_MS
#include "Galileo_E5a.h"  // for GALILEO_E5A_CODE_PERIO...
#include "convolutional.h"
#include "display.h"
#include "galileo_almanac_helper.h"  // for Galileo_Almanac_Helper
#include "galileo_ephemeris.h"       // for Galileo_Ephemeris
#include "galileo_iono.h"            // for Galileo_Iono
#include "galileo_utc_model.h"       // for Galileo_Utc_Model
#include "gnss_synchro.h"
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>        // for make_any
#include <pmt/pmt_sugar.h>  // for mp
#include <cmath>            // for fmod
#include <cstdlib>          // for abs
#include <exception>        // for exception
#include <iostream>         // for cout
#include <memory>           // for shared_ptr, make_shared


#define CRC_ERROR_LIMIT 6


galileo_telemetry_decoder_gs_sptr
galileo_make_telemetry_decoder_gs(const Gnss_Satellite &satellite, int frame_type, bool dump)
{
    return galileo_telemetry_decoder_gs_sptr(new galileo_telemetry_decoder_gs(satellite, frame_type, dump));
}


galileo_telemetry_decoder_gs::galileo_telemetry_decoder_gs(
    const Gnss_Satellite &satellite, int frame_type,
    bool dump) : gr::block("galileo_telemetry_decoder_gs", gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
                     gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    // prevent telemetry symbols accumulation in output buffers
    this->set_max_noutput_items(1);
    // Ephemeris data port out
    this->message_port_register_out(pmt::mp("telemetry"));
    // Control messages to tracking block
    this->message_port_register_out(pmt::mp("telemetry_to_trk"));
    d_last_valid_preamble = 0;
    d_sent_tlm_failed_msg = false;

    // initialize internal vars
    d_dump = dump;
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    d_frame_type = frame_type;
    DLOG(INFO) << "Initializing GALILEO UNIFIED TELEMETRY DECODER";

    switch (d_frame_type)
        {
        case 1:  // INAV
            {
                d_PRN_code_period_ms = static_cast<uint32_t>(GALILEO_E1_CODE_PERIOD_MS);
                d_bits_per_preamble = GALILEO_INAV_PREAMBLE_LENGTH_BITS;
                // set the preamble
                d_samples_per_preamble = GALILEO_INAV_PREAMBLE_LENGTH_BITS;
                d_preamble_period_symbols = GALILEO_INAV_PREAMBLE_PERIOD_SYMBOLS;
                d_required_symbols = static_cast<uint32_t>(GALILEO_INAV_PAGE_SYMBOLS) + d_samples_per_preamble;
                // preamble bits to sampled symbols
                d_preamble_samples.reserve(d_samples_per_preamble);
                d_frame_length_symbols = GALILEO_INAV_PAGE_PART_SYMBOLS - GALILEO_INAV_PREAMBLE_LENGTH_BITS;
                CodeLength = GALILEO_INAV_PAGE_PART_SYMBOLS - GALILEO_INAV_PREAMBLE_LENGTH_BITS;
                DataLength = (CodeLength / nn) - mm;
                d_max_symbols_without_valid_frame = GALILEO_INAV_PAGE_SYMBOLS * 30;  // rise alarm 60 seconds without valid tlm

                break;
            }
        case 2:  // FNAV
            {
                d_PRN_code_period_ms = static_cast<uint32_t>(GALILEO_E5A_CODE_PERIOD_MS * GALILEO_E5A_I_SECONDARY_CODE_LENGTH);
                d_bits_per_preamble = GALILEO_FNAV_PREAMBLE_LENGTH_BITS;
                // set the preamble
                d_samples_per_preamble = GALILEO_FNAV_PREAMBLE_LENGTH_BITS;
                d_preamble_period_symbols = GALILEO_FNAV_SYMBOLS_PER_PAGE;
                d_required_symbols = static_cast<uint32_t>(GALILEO_FNAV_SYMBOLS_PER_PAGE) + d_samples_per_preamble;
                // preamble bits to sampled symbols
                d_preamble_samples.reserve(d_samples_per_preamble);
                d_frame_length_symbols = GALILEO_FNAV_SYMBOLS_PER_PAGE - GALILEO_FNAV_PREAMBLE_LENGTH_BITS;
                CodeLength = GALILEO_FNAV_SYMBOLS_PER_PAGE - GALILEO_FNAV_PREAMBLE_LENGTH_BITS;
                DataLength = (CodeLength / nn) - mm;
                d_max_symbols_without_valid_frame = GALILEO_FNAV_SYMBOLS_PER_PAGE * 5;  // rise alarm 100 seconds without valid tlm
                break;
            }
        default:
            d_bits_per_preamble = 0;
            d_samples_per_preamble = 0;
            d_preamble_period_symbols = 0;
            d_PRN_code_period_ms = 0U;
            d_required_symbols = 0U;
            d_frame_length_symbols = 0U;
            CodeLength = 0;
            DataLength = 0;
            d_max_symbols_without_valid_frame = 0;
            std::cout << "Galileo unified telemetry decoder error: Unknown frame type " << std::endl;
        }

    d_page_part_symbols.reserve(d_frame_length_symbols);
    for (int32_t i = 0; i < d_bits_per_preamble; i++)
        {
            switch (d_frame_type)
                {
                case 1:  // INAV
                    {
                        if (GALILEO_INAV_PREAMBLE.at(i) == '1')
                            {
                                d_preamble_samples[i] = 1;
                            }
                        else
                            {
                                d_preamble_samples[i] = -1;
                            }
                        break;
                    }
                case 2:  // FNAV for E5a-I
                    {
                        if (GALILEO_FNAV_PREAMBLE.at(i) == '1')
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
        }
    d_sample_counter = 0ULL;
    d_stat = 0;
    d_preamble_index = 0ULL;

    d_flag_frame_sync = false;

    d_flag_parity = false;
    d_TOW_at_current_symbol_ms = 0;
    d_TOW_at_Preamble_ms = 0;
    delta_t = 0;
    d_CRC_error_counter = 0;
    flag_even_word_arrived = 0;
    d_flag_preamble = false;
    d_channel = 0;
    flag_TOW_set = false;
    flag_PLL_180_deg_phase_locked = false;
    d_symbol_history.set_capacity(d_required_symbols + 1);

    // vars for Viterbi decoder
    int32_t max_states = 1U << static_cast<uint32_t>(mm);  // 2^mm
    g_encoder[0] = 121;                                    // Polynomial G1
    g_encoder[1] = 91;                                     // Polynomial G2
    out0.reserve(max_states);
    out1.reserve(max_states);
    state0.reserve(max_states);
    state1.reserve(max_states);
    // create appropriate transition matrices
    nsc_transit(out0.data(), state0.data(), 0, g_encoder.data(), KK, nn);
    nsc_transit(out1.data(), state1.data(), 1, g_encoder.data(), KK, nn);
}


galileo_telemetry_decoder_gs::~galileo_telemetry_decoder_gs()
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


void galileo_telemetry_decoder_gs::viterbi_decoder(double *page_part_symbols, int32_t *page_part_bits)
{
    Viterbi(page_part_bits, out0.data(), state0.data(), out1.data(), state1.data(),
        page_part_symbols, KK, nn, DataLength);
}


void galileo_telemetry_decoder_gs::deinterleaver(int32_t rows, int32_t cols, const double *in, double *out)
{
    for (int32_t r = 0; r < rows; r++)
        {
            for (int32_t c = 0; c < cols; c++)
                {
                    out[c * rows + r] = in[r * cols + c];
                }
        }
}


void galileo_telemetry_decoder_gs::decode_INAV_word(double *page_part_symbols, int32_t frame_length)
{
    // 1. De-interleave
    std::vector<double> page_part_symbols_deint(frame_length);
    deinterleaver(GALILEO_INAV_INTERLEAVER_ROWS, GALILEO_INAV_INTERLEAVER_COLS, page_part_symbols, page_part_symbols_deint.data());

    // 2. Viterbi decoder
    // 2.1 Take into account the NOT gate in G2 polynomial (Galileo ICD Figure 13, FEC encoder)
    // 2.2 Take into account the possible inversion of the polarity due to PLL lock at 180º
    for (int32_t i = 0; i < frame_length; i++)
        {
            if ((i + 1) % 2 == 0)
                {
                    page_part_symbols_deint[i] = -page_part_symbols_deint[i];
                }
        }

    std::vector<int32_t> page_part_bits(frame_length / 2);
    viterbi_decoder(page_part_symbols_deint.data(), page_part_bits.data());

    // 3. Call the Galileo page decoder
    std::string page_String;
    for (int32_t i = 0; i < (frame_length / 2); i++)
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

    if (page_part_bits[0] == 1)
        {
            // DECODE COMPLETE WORD (even + odd) and TEST CRC
            d_inav_nav.split_page(page_String, flag_even_word_arrived);
            if (d_inav_nav.flag_CRC_test == true)
                {
                    DLOG(INFO) << "Galileo E1 CRC correct in channel " << d_channel << " from satellite " << d_satellite;
                }
            else
                {
                    DLOG(INFO) << "Galileo E1 CRC error in channel " << d_channel << " from satellite " << d_satellite;
                }
            flag_even_word_arrived = 0;
        }
    else
        {
            // STORE HALF WORD (even page)
            d_inav_nav.split_page(page_String, flag_even_word_arrived);
            flag_even_word_arrived = 1;
        }

    // 4. Push the new navigation data to the queues
    if (d_inav_nav.have_new_ephemeris() == true)
        {
            // get object for this SV (mandatory)
            std::shared_ptr<Galileo_Ephemeris> tmp_obj = std::make_shared<Galileo_Ephemeris>(d_inav_nav.get_ephemeris());
            std::cout << "New Galileo E1 I/NAV message received in channel " << d_channel << ": ephemeris from satellite " << d_satellite << std::endl;
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
        }
    if (d_inav_nav.have_new_iono_and_GST() == true)
        {
            // get object for this SV (mandatory)
            std::shared_ptr<Galileo_Iono> tmp_obj = std::make_shared<Galileo_Iono>(d_inav_nav.get_iono());
            std::cout << "New Galileo E1 I/NAV message received in channel " << d_channel << ": iono/GST model parameters from satellite " << d_satellite << std::endl;
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
        }
    if (d_inav_nav.have_new_utc_model() == true)
        {
            // get object for this SV (mandatory)
            std::shared_ptr<Galileo_Utc_Model> tmp_obj = std::make_shared<Galileo_Utc_Model>(d_inav_nav.get_utc_model());
            std::cout << "New Galileo E1 I/NAV message received in channel " << d_channel << ": UTC model parameters from satellite " << d_satellite << std::endl;
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            delta_t = tmp_obj->A_0G_10 + tmp_obj->A_1G_10 * (static_cast<double>(d_TOW_at_current_symbol_ms) / 1000.0 - tmp_obj->t_0G_10 + 604800 * (fmod((d_inav_nav.WN_0 - tmp_obj->WN_0G_10), 64)));
            DLOG(INFO) << "delta_t=" << delta_t << "[s]";
        }
    if (d_inav_nav.have_new_almanac() == true)
        {
            std::shared_ptr<Galileo_Almanac_Helper> tmp_obj = std::make_shared<Galileo_Almanac_Helper>(d_inav_nav.get_almanac());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            // debug
            std::cout << "Galileo E1 I/NAV almanac received in channel " << d_channel << " from satellite " << d_satellite << std::endl;
            DLOG(INFO) << "Current parameters:";
            DLOG(INFO) << "d_TOW_at_current_symbol_ms=" << d_TOW_at_current_symbol_ms;
            DLOG(INFO) << "d_nav.WN_0=" << d_inav_nav.WN_0;
        }
}


void galileo_telemetry_decoder_gs::decode_FNAV_word(double *page_symbols, int32_t frame_length)
{
    // 1. De-interleave
    std::vector<double> page_symbols_deint(frame_length);
    deinterleaver(GALILEO_FNAV_INTERLEAVER_ROWS, GALILEO_FNAV_INTERLEAVER_COLS, page_symbols, page_symbols_deint.data());

    // 2. Viterbi decoder
    // 2.1 Take into account the NOT gate in G2 polynomial (Galileo ICD Figure 13, FEC encoder)
    // 2.2 Take into account the possible inversion of the polarity due to PLL lock at 180�
    for (int32_t i = 0; i < frame_length; i++)
        {
            if ((i + 1) % 2 == 0)
                {
                    page_symbols_deint[i] = -page_symbols_deint[i];
                }
        }
    std::vector<int32_t> page_bits(frame_length / 2);
    viterbi_decoder(page_symbols_deint.data(), page_bits.data());

    // 3. Call the Galileo page decoder
    std::string page_String;
    for (int32_t i = 0; i < frame_length; i++)
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

    // DECODE COMPLETE WORD (even + odd) and TEST CRC
    d_fnav_nav.split_page(page_String);
    if (d_fnav_nav.flag_CRC_test == true)
        {
            DLOG(INFO) << "Galileo E5a CRC correct in channel " << d_channel << " from satellite " << d_satellite;
        }
    else
        {
            DLOG(INFO) << "Galileo E5a CRC error in channel " << d_channel << " from satellite " << d_satellite;
        }

    // 4. Push the new navigation data to the queues
    if (d_fnav_nav.have_new_ephemeris() == true)
        {
            std::shared_ptr<Galileo_Ephemeris> tmp_obj = std::make_shared<Galileo_Ephemeris>(d_fnav_nav.get_ephemeris());
            std::cout << TEXT_MAGENTA << "New Galileo E5a F/NAV message received in channel " << d_channel << ": ephemeris from satellite " << d_satellite << TEXT_RESET << std::endl;
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
        }
    if (d_fnav_nav.have_new_iono_and_GST() == true)
        {
            std::shared_ptr<Galileo_Iono> tmp_obj = std::make_shared<Galileo_Iono>(d_fnav_nav.get_iono());
            std::cout << TEXT_MAGENTA << "New Galileo E5a F/NAV message received in channel " << d_channel << ": iono/GST model parameters from satellite " << d_satellite << TEXT_RESET << std::endl;
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
        }
    if (d_fnav_nav.have_new_utc_model() == true)
        {
            std::shared_ptr<Galileo_Utc_Model> tmp_obj = std::make_shared<Galileo_Utc_Model>(d_fnav_nav.get_utc_model());
            std::cout << TEXT_MAGENTA << "New Galileo E5a F/NAV message received in channel " << d_channel << ": UTC model parameters from satellite " << d_satellite << TEXT_RESET << std::endl;
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
        }
}


void galileo_telemetry_decoder_gs::set_satellite(const Gnss_Satellite &satellite)
{
    gr::thread::scoped_lock lock(d_setlock);
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    d_last_valid_preamble = d_sample_counter;
    d_sent_tlm_failed_msg = false;
    DLOG(INFO) << "Setting decoder Finite State Machine to satellite " << d_satellite;
    DLOG(INFO) << "Navigation Satellite set to " << d_satellite;
}


void galileo_telemetry_decoder_gs::reset()
{
    gr::thread::scoped_lock lock(d_setlock);
    d_last_valid_preamble = d_sample_counter;
    d_sent_tlm_failed_msg = false;
    d_stat = 0;
    DLOG(INFO) << "Telemetry decoder reset for satellite " << d_satellite;
}


void galileo_telemetry_decoder_gs::set_channel(int32_t channel)
{
    d_channel = channel;
    DLOG(INFO) << "Navigation channel set to " << channel;
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
                            LOG(WARNING) << "channel " << d_channel << " Exception opening trk dump file " << e.what();
                        }
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
    // add new symbol to the symbol queue
    switch (d_frame_type)
        {
        case 1:  // INAV
            {
                d_symbol_history.push_back(current_symbol.Prompt_I);
                break;
            }
        case 2:  // FNAV
            {
                d_symbol_history.push_back(current_symbol.Prompt_Q);
                break;
            }
        default:
            {
                d_symbol_history.push_back(current_symbol.Prompt_I);
                break;
            }
        }
    d_sample_counter++;  // count for the processed symbols
    consume_each(1);
    d_flag_preamble = false;

    // check if there is a problem with the telemetry of the current satellite
    if (d_sent_tlm_failed_msg == false)
        {
            if ((d_sample_counter - d_last_valid_preamble) > d_max_symbols_without_valid_frame)
                {
                    int message = 1;  // bad telemetry
                    DLOG(INFO) << "sent msg sat " << this->d_satellite;
                    this->message_port_pub(pmt::mp("telemetry_to_trk"), pmt::make_any(message));
                    d_sent_tlm_failed_msg = true;
                }
        }

    // ******* frame sync ******************
    switch (d_stat)
        {
        case 0:  // no preamble information
            {
                // correlate with preamble
                int32_t corr_value = 0;
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
                        if (abs(corr_value) >= d_samples_per_preamble)
                            {
                                d_preamble_index = d_sample_counter;  // record the preamble sample stamp
                                DLOG(INFO) << "Preamble detection for Galileo satellite " << this->d_satellite;
                                d_stat = 1;  // enter into frame pre-detection status
                            }
                    }

                break;
            }
        case 1:  // possible preamble lock
            {
                // correlate with preamble
                int32_t corr_value = 0;
                int32_t preamble_diff = 0;
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
                        if (abs(corr_value) >= d_samples_per_preamble)
                            {
                                // check preamble separation
                                preamble_diff = static_cast<int32_t>(d_sample_counter - d_preamble_index);
                                if (abs(preamble_diff - d_preamble_period_symbols) == 0)
                                    {
                                        // try to decode frame
                                        DLOG(INFO) << "Starting page decoder for Galileo satellite " << this->d_satellite;
                                        d_preamble_index = d_sample_counter;  // record the preamble sample stamp
                                        d_CRC_error_counter = 0;
                                        if (corr_value < 0)
                                            {
                                                flag_PLL_180_deg_phase_locked = true;
                                            }
                                        else
                                            {
                                                flag_PLL_180_deg_phase_locked = false;
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
            }
        case 2:  // preamble acquired
            {
                if (d_sample_counter == d_preamble_index + static_cast<uint64_t>(d_preamble_period_symbols))
                    {
                        // call the decoder
                        switch (d_frame_type)
                            {
                            case 1:  // INAV
                                     // NEW Galileo page part is received
                                // 0. fetch the symbols into an array
                                if (flag_PLL_180_deg_phase_locked == false)  // normal PLL lock
                                    {
                                        for (uint32_t i = 0; i < d_frame_length_symbols; i++)
                                            {
                                                d_page_part_symbols[i] = d_symbol_history.at(i + d_samples_per_preamble);  // because last symbol of the preamble is just received now!
                                            }
                                    }
                                else  // 180 deg. inverted carrier phase PLL lock
                                    {
                                        for (uint32_t i = 0; i < d_frame_length_symbols; i++)
                                            {
                                                d_page_part_symbols[i] = -d_symbol_history.at(i + d_samples_per_preamble);  // because last symbol of the preamble is just received now!
                                            }
                                    }
                                decode_INAV_word(d_page_part_symbols.data(), d_frame_length_symbols);
                                break;
                            case 2:  // FNAV
                                     // NEW Galileo page part is received
                                // 0. fetch the symbols into an array
                                if (flag_PLL_180_deg_phase_locked == false)  // normal PLL lock
                                    {
                                        for (uint32_t i = 0; i < d_frame_length_symbols; i++)
                                            {
                                                for (uint32_t i = 0; i < d_frame_length_symbols; i++)
                                                    {
                                                        d_page_part_symbols[i] = d_symbol_history.at(i + d_samples_per_preamble);  // because last symbol of the preamble is just received now!
                                                    }
                                            }
                                    }
                                else  // 180 deg. inverted carrier phase PLL lock
                                    {
                                        for (uint32_t i = 0; i < d_frame_length_symbols; i++)
                                            {
                                                for (uint32_t i = 0; i < d_frame_length_symbols; i++)
                                                    {
                                                        d_page_part_symbols[i] = -d_symbol_history.at(i + d_samples_per_preamble);  // because last symbol of the preamble is just received now!
                                                    }
                                            }
                                    }
                                decode_FNAV_word(d_page_part_symbols.data(), d_frame_length_symbols);
                                break;
                            default:
                                return -1;
                                break;
                            }
                        d_preamble_index = d_sample_counter;  // record the preamble sample stamp (t_P)
                        if (d_inav_nav.flag_CRC_test == true or d_fnav_nav.flag_CRC_test == true)
                            {
                                d_CRC_error_counter = 0;
                                d_flag_preamble = true;  // valid preamble indicator (initialized to false every work())
                                gr::thread::scoped_lock lock(d_setlock);
                                d_last_valid_preamble = d_sample_counter;
                                if (!d_flag_frame_sync)
                                    {
                                        d_flag_frame_sync = true;
                                        DLOG(INFO) << " Frame sync SAT " << this->d_satellite;
                                    }
                            }
                        else
                            {
                                d_CRC_error_counter++;
                                if (d_CRC_error_counter > CRC_ERROR_LIMIT)
                                    {
                                        DLOG(INFO) << "Lost of frame sync SAT " << this->d_satellite;
                                        d_flag_frame_sync = false;
                                        d_stat = 0;
                                        d_TOW_at_current_symbol_ms = 0;
                                        d_TOW_at_Preamble_ms = 0;
                                        d_fnav_nav.flag_TOW_set = false;
                                        d_inav_nav.flag_TOW_set = false;
                                    }
                            }
                    }
                break;
            }
        }

    // UPDATE GNSS SYNCHRO DATA
    // 2. Add the telemetry decoder information
    if (this->d_flag_preamble == true)
        // update TOW at the preamble instant
        {
            switch (d_frame_type)
                {
                case 1:  // INAV
                    {
                        if (d_inav_nav.flag_TOW_set == true)
                            {
                                if (d_inav_nav.flag_TOW_5 == true)  // page 5 arrived and decoded, so we are in the odd page (since Tow refers to the even page, we have to add 1 sec)
                                    {
                                        // TOW_5 refers to the even preamble, but when we decode it we are in the odd part, so 1 second later plus the decoding delay
                                        d_TOW_at_Preamble_ms = static_cast<uint32_t>(d_inav_nav.TOW_5 * 1000.0);
                                        d_TOW_at_current_symbol_ms = d_TOW_at_Preamble_ms + static_cast<uint32_t>(GALILEO_INAV_PAGE_PART_MS + (d_required_symbols + 1) * GALILEO_E1_CODE_PERIOD_MS);
                                        d_inav_nav.flag_TOW_5 = false;
                                    }

                                else if (d_inav_nav.flag_TOW_6 == true)  // page 6 arrived and decoded, so we are in the odd page (since Tow refers to the even page, we have to add 1 sec)
                                    {
                                        // TOW_6 refers to the even preamble, but when we decode it we are in the odd part, so 1 second later plus the decoding delay
                                        d_TOW_at_Preamble_ms = static_cast<uint32_t>(d_inav_nav.TOW_6 * 1000.0);
                                        d_TOW_at_current_symbol_ms = d_TOW_at_Preamble_ms + static_cast<uint32_t>(GALILEO_INAV_PAGE_PART_MS + (d_required_symbols + 1) * GALILEO_E1_CODE_PERIOD_MS);
                                        d_inav_nav.flag_TOW_6 = false;
                                    }
                                else
                                    {
                                        // this page has no timing information
                                        d_TOW_at_current_symbol_ms += static_cast<uint32_t>(GALILEO_E1_CODE_PERIOD_MS);  // + GALILEO_INAV_PAGE_PART_SYMBOLS*GALILEO_E1_CODE_PERIOD;
                                    }
                            }
                        break;
                    }
                case 2:  // FNAV
                    {
                        if (d_fnav_nav.flag_TOW_set == true)
                            {
                                if (d_fnav_nav.flag_TOW_1 == true)
                                    {
                                        d_TOW_at_Preamble_ms = static_cast<uint32_t>(d_fnav_nav.FNAV_TOW_1 * 1000.0);
                                        d_TOW_at_current_symbol_ms = d_TOW_at_Preamble_ms + static_cast<uint32_t>((d_required_symbols + 1) * GALILEO_FNAV_CODES_PER_SYMBOL * GALILEO_E5A_CODE_PERIOD_MS);
                                        //d_TOW_at_current_symbol_ms = d_TOW_at_Preamble_ms + static_cast<uint32_t>((GALILEO_FNAV_CODES_PER_PAGE + GALILEO_FNAV_CODES_PER_PREAMBLE) * GALILEO_E5a_CODE_PERIOD_MS);
                                        d_fnav_nav.flag_TOW_1 = false;
                                    }
                                else if (d_fnav_nav.flag_TOW_2 == true)
                                    {
                                        d_TOW_at_Preamble_ms = static_cast<uint32_t>(d_fnav_nav.FNAV_TOW_2 * 1000.0);
                                        //d_TOW_at_current_symbol_ms = d_TOW_at_Preamble_ms + static_cast<uint32_t>((GALILEO_FNAV_CODES_PER_PAGE + GALILEO_FNAV_CODES_PER_PREAMBLE) * GALILEO_E5a_CODE_PERIOD_MS);
                                        d_TOW_at_current_symbol_ms = d_TOW_at_Preamble_ms + static_cast<uint32_t>((d_required_symbols + 1) * GALILEO_FNAV_CODES_PER_SYMBOL * GALILEO_E5A_CODE_PERIOD_MS);
                                        d_fnav_nav.flag_TOW_2 = false;
                                    }
                                else if (d_fnav_nav.flag_TOW_3 == true)
                                    {
                                        d_TOW_at_Preamble_ms = static_cast<uint32_t>(d_fnav_nav.FNAV_TOW_3 * 1000.0);
                                        //d_TOW_at_current_symbol_ms = d_TOW_at_Preamble_ms + static_cast<uint32_t>((GALILEO_FNAV_CODES_PER_PAGE + GALILEO_FNAV_CODES_PER_PREAMBLE) * GALILEO_E5a_CODE_PERIOD_MS);
                                        d_TOW_at_current_symbol_ms = d_TOW_at_Preamble_ms + static_cast<uint32_t>((d_required_symbols + 1) * GALILEO_FNAV_CODES_PER_SYMBOL * GALILEO_E5A_CODE_PERIOD_MS);
                                        d_fnav_nav.flag_TOW_3 = false;
                                    }
                                else if (d_fnav_nav.flag_TOW_4 == true)
                                    {
                                        d_TOW_at_Preamble_ms = static_cast<uint32_t>(d_fnav_nav.FNAV_TOW_4 * 1000.0);
                                        //d_TOW_at_current_symbol_ms = d_TOW_at_Preamble_ms + static_cast<uint32_t>((GALILEO_FNAV_CODES_PER_PAGE + GALILEO_FNAV_CODES_PER_PREAMBLE) * GALILEO_E5a_CODE_PERIOD_MS);
                                        d_TOW_at_current_symbol_ms = d_TOW_at_Preamble_ms + static_cast<uint32_t>((d_required_symbols + 1) * GALILEO_FNAV_CODES_PER_SYMBOL * GALILEO_E5A_CODE_PERIOD_MS);
                                        d_fnav_nav.flag_TOW_4 = false;
                                    }
                                else
                                    {
                                        d_TOW_at_current_symbol_ms += static_cast<uint32_t>(GALILEO_FNAV_CODES_PER_SYMBOL * GALILEO_E5A_CODE_PERIOD_MS);
                                    }
                                break;
                            }
                    }
                }
        }
    else  // if there is not a new preamble, we define the TOW of the current symbol
        {
            switch (d_frame_type)
                {
                case 1:  // INAV
                    {
                        if (d_inav_nav.flag_TOW_set == true)
                            {
                                d_TOW_at_current_symbol_ms += d_PRN_code_period_ms;
                            }
                        break;
                    }
                case 2:  // FNAV
                    {
                        if (d_fnav_nav.flag_TOW_set == true)
                            {
                                d_TOW_at_current_symbol_ms += d_PRN_code_period_ms;
                            }
                        break;
                    }
                }
        }

    switch (d_frame_type)
        {
        case 1:  // INAV
            {
                if (d_inav_nav.flag_TOW_set)
                    {
                        if (d_inav_nav.flag_GGTO_1 == true and d_inav_nav.flag_GGTO_2 == true and d_inav_nav.flag_GGTO_3 == true and d_inav_nav.flag_GGTO_4 == true)  // all GGTO parameters arrived
                            {
                                delta_t = d_inav_nav.A_0G_10 + d_inav_nav.A_1G_10 * (static_cast<double>(d_TOW_at_current_symbol_ms) / 1000.0 - d_inav_nav.t_0G_10 + 604800.0 * (fmod((d_inav_nav.WN_0 - d_inav_nav.WN_0G_10), 64.0)));
                            }

                        current_symbol.Flag_valid_word = true;
                    }
                break;
            }

        case 2:  // FNAV
            {
                if (d_fnav_nav.flag_TOW_set)
                    {
                        current_symbol.Flag_valid_word = true;
                    }
                break;
            }
        }

    if (d_inav_nav.flag_TOW_set or d_fnav_nav.flag_TOW_set)
        {
            current_symbol.TOW_at_current_symbol_ms = d_TOW_at_current_symbol_ms;
            // todo: Galileo to GPS time conversion should be moved to observable block.
            // current_symbol.TOW_at_current_symbol_ms -= delta_t;  // Galileo to GPS TOW

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
                            LOG(WARNING) << "Exception writing observables dump file " << e.what();
                        }
                }
            // 3. Make the output (copy the object contents to the GNURadio reserved memory)
            *out[0] = current_symbol;
            return 1;
        }
    return 0;
}
