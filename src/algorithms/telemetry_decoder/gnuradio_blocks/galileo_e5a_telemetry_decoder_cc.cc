/*!
 * \file galileo_e5a_telemetry_decoder_cc.cc
 * \brief Implementation of a Galileo FNAV message demodulator block
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 * 		   Javier Arribas, 2017. jarribas(at)cttc.es
 * \based on work from:
 *          <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          </ul>
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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

#include "galileo_e5a_telemetry_decoder_cc.h"
#include "control_message_factory.h"
#include "convolutional.h"
#include <boost/lexical_cast.hpp>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include <cmath>
#include <iostream>


#define GALILEO_E5a_CRC_ERROR_LIMIT 6

using google::LogMessage;


galileo_e5a_telemetry_decoder_cc_sptr
galileo_e5a_make_telemetry_decoder_cc(const Gnss_Satellite &satellite, bool dump)
{
    return galileo_e5a_telemetry_decoder_cc_sptr(new galileo_e5a_telemetry_decoder_cc(satellite, dump));
}


void galileo_e5a_telemetry_decoder_cc::viterbi_decoder(double *page_part_symbols, int *page_part_bits)
{
    //    int CodeLength = 240;
    int CodeLength = 488;
    int DataLength;
    int nn, KK, mm, max_states;
    int g_encoder[2];

    nn = 2;              // Coding rate 1/n
    KK = 7;              // Constraint Length
    g_encoder[0] = 121;  // Polynomial G1
    g_encoder[1] = 91;   // Polynomial G2
    //    g_encoder[0] = 171; // Polynomial G1
    //    g_encoder[1] = 133;  // Polynomial G2

    mm = KK - 1;
    max_states = 1 << mm;  // 2^mm
    DataLength = (CodeLength / nn) - mm;

    //create appropriate transition matrices

    int *out0, *out1, *state0, *state1;
    out0 = static_cast<int *>(calloc(max_states, sizeof(int)));
    out1 = static_cast<int *>(calloc(max_states, sizeof(int)));
    state0 = static_cast<int *>(calloc(max_states, sizeof(int)));
    state1 = static_cast<int *>(calloc(max_states, sizeof(int)));

    nsc_transit(out0, state0, 0, g_encoder, KK, nn);
    nsc_transit(out1, state1, 1, g_encoder, KK, nn);

    Viterbi(page_part_bits, out0, state0, out1, state1,
        page_part_symbols, KK, nn, DataLength);

    //Clean up memory
    free(out0);
    free(out1);
    free(state0);
    free(state1);
}


void galileo_e5a_telemetry_decoder_cc::deinterleaver(int rows, int cols, double *in, double *out)
{
    for (int r = 0; r < rows; r++)
        {
            for (int c = 0; c < cols; c++)
                {
                    out[c * rows + r] = in[r * cols + c];
                }
        }
}


void galileo_e5a_telemetry_decoder_cc::decode_word(double *page_symbols, int frame_length)
{
    double page_symbols_deint[frame_length];
    // 1. De-interleave

    deinterleaver(GALILEO_FNAV_INTERLEAVER_ROWS, GALILEO_FNAV_INTERLEAVER_COLS, page_symbols, page_symbols_deint);

    // 2. Viterbi decoder
    // 2.1 Take into account the NOT gate in G2 polynomial (Galileo ICD Figure 13, FEC encoder)
    // 2.2 Take into account the possible inversion of the polarity due to PLL lock at 180�
    for (int i = 0; i < frame_length; i++)
        {
            if ((i + 1) % 2 == 0)
                {
                    page_symbols_deint[i] = -page_symbols_deint[i];
                }
        }
    int page_bits[frame_length / 2];
    viterbi_decoder(page_symbols_deint, page_bits);

    // 3. Call the Galileo page decoder
    std::string page_String;
    for (int i = 0; i < frame_length; i++)
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
    d_nav.split_page(page_String);
    if (d_nav.flag_CRC_test == true)
        {
            LOG(INFO) << "Galileo E5a CRC correct on channel " << d_channel << " from satellite " << d_satellite;
            //std::cout << "Galileo E5a CRC correct on channel " << d_channel << " from satellite " << d_satellite << std::endl;
        }
    else
        {
            std::cout << "Galileo E5a CRC error on channel " << d_channel << " from satellite " << d_satellite << std::endl;
            LOG(INFO) << "Galileo E5a CRC error on channel " << d_channel << " from satellite " << d_satellite;
        }

    // 4. Push the new navigation data to the queues
    if (d_nav.have_new_ephemeris() == true)
        {
            std::shared_ptr<Galileo_Ephemeris> tmp_obj = std::make_shared<Galileo_Ephemeris>(d_nav.get_ephemeris());
            std::cout << "New Galileo E5a F/NAV message received: ephemeris from satellite " << d_satellite << std::endl;
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
        }
    if (d_nav.have_new_iono_and_GST() == true)
        {
            std::shared_ptr<Galileo_Iono> tmp_obj = std::make_shared<Galileo_Iono>(d_nav.get_iono());
            std::cout << "New Galileo E5a F/NAV message received: iono/GST model parameters from satellite " << d_satellite << std::endl;
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
        }
    if (d_nav.have_new_utc_model() == true)
        {
            std::shared_ptr<Galileo_Utc_Model> tmp_obj = std::make_shared<Galileo_Utc_Model>(d_nav.get_utc_model());
            std::cout << "New Galileo E5a F/NAV message received: UTC model parameters from satellite " << d_satellite << std::endl;
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
        }
}


galileo_e5a_telemetry_decoder_cc::galileo_e5a_telemetry_decoder_cc(
    const Gnss_Satellite &satellite, bool dump) : gr::block("galileo_e5a_telemetry_decoder_cc",
                                                      gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
                                                      gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    // Ephemeris data port out
    this->message_port_register_out(pmt::mp("telemetry"));
    // initialize internal vars
    d_dump = dump;
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    LOG(INFO) << "GALILEO E5A TELEMETRY PROCESSING: satellite " << d_satellite;

    // set the preamble
    for (int i = 0; i < GALILEO_FNAV_PREAMBLE_LENGTH_BITS; i++)
        {
            if (GALILEO_FNAV_PREAMBLE.at(i) == '0')
                {
                    d_preambles_bits[i] = 1;
                }
            else
                {
                    d_preambles_bits[i] = -1;
                }
        }
    for (int i = 0; i < GALILEO_FNAV_PREAMBLE_LENGTH_BITS; i++)
        {
            for (int k = 0; k < GALILEO_FNAV_CODES_PER_SYMBOL; k++)
                {
                    d_preamble_samples[(i * GALILEO_FNAV_CODES_PER_SYMBOL) + k] = d_preambles_bits[i];
                }
        }

    d_sample_counter = 0;
    d_stat = 0;
    corr_value = 0;
    d_flag_preamble = false;
    d_preamble_index = 0;
    d_flag_frame_sync = false;
    d_TOW_at_current_symbol = 0.0;
    flag_TOW_set = false;
    d_CRC_error_counter = 0;
    d_channel = 0;
    delta_t = 0.0;
    d_symbol_counter = 0;
    d_prompt_acum = 0.0;
    flag_bit_start = false;
    new_symbol = false;
    required_symbols = GALILEO_FNAV_SYMBOLS_PER_PAGE + GALILEO_FNAV_PREAMBLE_LENGTH_BITS;
}


galileo_e5a_telemetry_decoder_cc::~galileo_e5a_telemetry_decoder_cc()
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


void galileo_e5a_telemetry_decoder_cc::set_satellite(const Gnss_Satellite &satellite)
{
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << "Setting decoder Finite State Machine to satellite " << d_satellite;
    DLOG(INFO) << "Navigation Satellite set to " << d_satellite;
}


void galileo_e5a_telemetry_decoder_cc::set_channel(int channel)
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
                            d_dump_filename.append(boost::lexical_cast<std::string>(d_channel));
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


int galileo_e5a_telemetry_decoder_cc::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    int preamble_diff = 0;

    Gnss_Synchro *out = reinterpret_cast<Gnss_Synchro *>(output_items[0]);            // Get the output buffer pointer
    const Gnss_Synchro *in = reinterpret_cast<const Gnss_Synchro *>(input_items[0]);  // Get the input buffer pointer

    //1. Copy the current tracking output
    Gnss_Synchro current_sample = in[0];
    d_symbol_counter++;
    if (flag_bit_start)
        {
            d_prompt_acum += current_sample.Prompt_I;
            if (d_symbol_counter == GALILEO_FNAV_CODES_PER_SYMBOL)
                {
                    current_sample.Prompt_I = d_prompt_acum / static_cast<double>(GALILEO_FNAV_CODES_PER_SYMBOL);
                    d_symbol_history.push_back(current_sample);  //add new symbol to the symbol queue
                    d_prompt_acum = 0.0;
                    d_symbol_counter = 0;
                    new_symbol = true;
                }
        }
    else
        {
            if (current_sample.Prompt_I < 0.0)
                {
                    d_preamble_init.push_back(1);
                }
            else
                {
                    d_preamble_init.push_back(-1);
                }

            if (d_preamble_init.size() == GALILEO_FNAV_CODES_PER_PREAMBLE)
                {
                    std::deque<int>::iterator iter;
                    int k = 0;
                    corr_value = 0;
                    for (iter = d_preamble_init.begin(); iter != d_preamble_init.end(); iter++)
                        {
                            corr_value += *iter * d_preamble_samples[k];
                            k++;
                        }
                    if (abs(corr_value) == GALILEO_FNAV_CODES_PER_PREAMBLE)
                        {
                            d_symbol_counter = 0;
                            flag_bit_start = true;
                            corr_value = 0;
                            d_preamble_init.clear();
                            d_symbol_history.clear();
                            LOG(INFO) << "Bit start sync for Galileo E5a satellite " << d_satellite;
                        }
                    else
                        {
                            d_preamble_init.pop_front();
                        }
                }
        }
    d_sample_counter++;  //count for the processed samples
    consume_each(1);

    d_flag_preamble = false;

    if ((d_symbol_history.size() > required_symbols) && new_symbol)
        {
            //******* preamble correlation ********
            corr_value = 0;
            for (int i = 0; i < GALILEO_FNAV_PREAMBLE_LENGTH_BITS; i++)
                {
                    if (d_symbol_history.at(i).Prompt_I < 0.0)  // symbols clipping
                        {
                            corr_value -= d_preambles_bits[i];
                        }
                    else
                        {
                            corr_value += d_preambles_bits[i];
                        }
                }
        }

    //******* frame sync ******************
    if ((d_stat == 0) && new_symbol)  //no preamble information
        {
            if (abs(corr_value) >= GALILEO_FNAV_PREAMBLE_LENGTH_BITS)
                {
                    d_preamble_index = d_sample_counter;  //record the preamble sample stamp
                    LOG(INFO) << "Preamble detection for Galileo E5a satellite " << d_satellite;
                    d_stat = 1;  // enter into frame pre-detection status
                }
        }
    else if ((d_stat == 1) && new_symbol)  // possible preamble lock
        {
            if (abs(corr_value) >= GALILEO_FNAV_PREAMBLE_LENGTH_BITS)
                {
                    //check preamble separation
                    preamble_diff = d_sample_counter - d_preamble_index;
                    if (preamble_diff == GALILEO_FNAV_CODES_PER_PAGE)
                        {
                            //try to decode frame
                            LOG(INFO) << "Starting page decoder for Galileo E5a satellite " << d_satellite;
                            d_preamble_index = d_sample_counter;  //record the preamble sample stamp
                            d_stat = 2;
                        }
                    else if (preamble_diff > GALILEO_FNAV_CODES_PER_PAGE)
                        {
                            d_stat = 0;  // start again
                            flag_bit_start = false;
                            LOG(INFO) << "Preamble diff = " << preamble_diff;
                        }
                }
        }
    else if ((d_stat == 2) && new_symbol)
        {
            if (d_sample_counter == (d_preamble_index + GALILEO_FNAV_CODES_PER_PAGE))
                {
                    // NEW Galileo page part is received
                    // 0. fetch the symbols into an array
                    int frame_length = GALILEO_FNAV_SYMBOLS_PER_PAGE - GALILEO_FNAV_PREAMBLE_LENGTH_BITS;
                    double corr_sign = 0.0;
                    if (corr_value > 0)
                        {
                            corr_sign = -1.0;
                        }
                    else
                        {
                            corr_sign = 1.0;
                        }
                    for (int i = 0; i < frame_length; i++)
                        {
                            page_symbols[i] = corr_sign * d_symbol_history.at(i + GALILEO_FNAV_PREAMBLE_LENGTH_BITS).Prompt_I;  // because last symbol of the preamble is just received now!
                        }

                    //call the decoder
                    decode_word(page_symbols, frame_length);
                    if (d_nav.flag_CRC_test == true)
                        {
                            d_CRC_error_counter = 0;
                            d_flag_preamble = true;               //valid preamble indicator (initialized to false every work())
                            d_preamble_index = d_sample_counter;  //record the preamble sample stamp (t_P)
                            if (!d_flag_frame_sync)
                                {
                                    d_flag_frame_sync = true;
                                    DLOG(INFO) << " Frame sync SAT " << this->d_satellite << " with preamble start at "
                                               << d_symbol_history.at(0).Tracking_sample_counter << " [samples]";
                                }
                        }
                    else
                        {
                            d_CRC_error_counter++;
                            d_preamble_index = d_sample_counter;  //record the preamble sample stamp
                            if (d_CRC_error_counter > GALILEO_E5A_CRC_ERROR_LIMIT)
                                {
                                    LOG(INFO) << "Lost of frame sync SAT " << this->d_satellite;
                                    d_flag_frame_sync = false;
                                    d_stat = 0;
                                    flag_bit_start = false;
                                }
                        }
                }
        }
    new_symbol = false;

    // UPDATE GNSS SYNCHRO DATA
    //Add the telemetry decoder information
    if (d_flag_preamble and d_nav.flag_TOW_set)
        //update TOW at the preamble instant
        //We expect a preamble each 10 seconds (FNAV page period)
        {
            if (d_nav.flag_TOW_1 == true)
                {
                    d_TOW_at_current_symbol = d_nav.FNAV_TOW_1 + (static_cast<double>(GALILEO_FNAV_CODES_PER_PAGE + GALILEO_FNAV_CODES_PER_PREAMBLE) * GALILEO_E5a_CODE_PERIOD);
                    d_nav.flag_TOW_1 = false;
                }
            else if (d_nav.flag_TOW_2 == true)
                {
                    d_TOW_at_current_symbol = d_nav.FNAV_TOW_2 + (static_cast<double>(GALILEO_FNAV_CODES_PER_PAGE + GALILEO_FNAV_CODES_PER_PREAMBLE) * GALILEO_E5a_CODE_PERIOD);
                    d_nav.flag_TOW_2 = false;
                }
            else if (d_nav.flag_TOW_3 == true)
                {
                    d_TOW_at_current_symbol = d_nav.FNAV_TOW_3 + (static_cast<double>(GALILEO_FNAV_CODES_PER_PAGE + GALILEO_FNAV_CODES_PER_PREAMBLE) * GALILEO_E5a_CODE_PERIOD);
                    d_nav.flag_TOW_3 = false;
                }
            else if (d_nav.flag_TOW_4 == true)
                {
                    d_TOW_at_current_symbol = d_nav.FNAV_TOW_4 + (static_cast<double>(GALILEO_FNAV_CODES_PER_PAGE + GALILEO_FNAV_CODES_PER_PREAMBLE) * GALILEO_E5a_CODE_PERIOD);
                    d_nav.flag_TOW_4 = false;
                }
            else
                {
                    d_TOW_at_current_symbol += GALILEO_E5a_CODE_PERIOD;
                }
        }
    else  //if there is not a new preamble, we define the TOW of the current symbol
        {
            d_TOW_at_current_symbol += GALILEO_E5a_CODE_PERIOD;
        }

    //if (d_flag_frame_sync == true and d_nav.flag_TOW_set==true and d_nav.flag_CRC_test == true)
    if (d_flag_frame_sync and d_nav.flag_TOW_set)
        {
            current_sample.Flag_valid_word = true;
        }
    else
        {
            current_sample.Flag_valid_word = false;
        }

    current_sample.TOW_at_current_symbol_s = floor(d_TOW_at_current_symbol * 1000.0) / 1000.0;

    if (d_dump)
        {
            // MULTIPLEXED FILE RECORDING - Record results to file
            try
                {
                    double tmp_double;
                    unsigned long int tmp_ulong_int;
                    tmp_double = d_TOW_at_current_symbol;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                    tmp_ulong_int = current_sample.Tracking_sample_counter;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_ulong_int), sizeof(unsigned long int));
                    tmp_double = 0.0;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                }
            catch (const std::ifstream::failure &e)
                {
                    LOG(WARNING) << "Exception writing Galileo E5a Telemetry Decoder dump file " << e.what();
                }
        }
    // remove used symbols from history
    while (d_symbol_history.size() > required_symbols)
        {
            d_symbol_history.pop_front();
        }
    //3. Make the output
    if (current_sample.Flag_valid_word)
        {
            out[0] = current_sample;
            return 1;
        }
    else
        {
            return 0;
        }
}
