/*!
 * \file galileo_e5a_telemetry_decoder_cc.cc
 * \brief Implementation of a Galileo FNAV message demodulator block
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
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
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include "control_message_factory.h"
#include "gnss_synchro.h"
#include "convolutional.h"


#define CRC_ERROR_LIMIT 6

using google::LogMessage;


galileo_e5a_telemetry_decoder_cc_sptr
galileo_e5a_make_telemetry_decoder_cc(Gnss_Satellite satellite, bool dump)
{
    return galileo_e5a_telemetry_decoder_cc_sptr(new galileo_e5a_telemetry_decoder_cc(satellite, dump));
}


void galileo_e5a_telemetry_decoder_cc::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
    //ninput_items_required[0] = GALILEO_FNAV_SAMPLES_PER_PAGE; // set the required sample history
    if (noutput_items != 0)
        {
            ninput_items_required[0] = GALILEO_FNAV_CODES_PER_PREAMBLE;
        }
}


void galileo_e5a_telemetry_decoder_cc::viterbi_decoder(double *page_part_symbols, int *page_part_bits)
{
    //    int CodeLength = 240;
    int CodeLength = 488;
    int DataLength;
    int nn, KK, mm, max_states;
    int g_encoder[2];

    nn = 2;             // Coding rate 1/n
    KK = 7;             // Constraint Length
    g_encoder[0] = 121; // Polynomial G1
    g_encoder[1] = 91;  // Polynomial G2
    //    g_encoder[0] = 171; // Polynomial G1
    //    g_encoder[1] = 133;  // Polynomial G2

    mm = KK - 1;
    max_states = 1 << mm; // 2^mm
    DataLength = (CodeLength/nn) - mm;

    //create appropriate transition matrices

    int *out0, *out1, *state0, *state1;
    out0 = (int*)calloc( max_states, sizeof(int) );
    out1 = (int*)calloc( max_states, sizeof(int) );
    state0 = (int*)calloc( max_states, sizeof(int) );
    state1 = (int*)calloc( max_states, sizeof(int) );

    nsc_transit( out0, state0, 0, g_encoder, KK, nn );
    nsc_transit( out1, state1, 1, g_encoder, KK, nn );

    Viterbi(page_part_bits, out0, state0, out1, state1,
            page_part_symbols, KK, nn, DataLength );

    //Clean up memory
    free( out0 );
    free( out1 );
    free( state0 );
    free( state1 );
}


void galileo_e5a_telemetry_decoder_cc::deinterleaver(int rows, int cols, double *in, double *out)
{
    for (int r = 0; r < rows; r++)
        {
            for(int c = 0; c < cols; c++)
                {
                    out[c*rows + r] = in[r*cols + c];
                }
        }
}


void galileo_e5a_telemetry_decoder_cc::decode_word(double *page_symbols,int frame_length)
{
    double page_symbols_deint[frame_length];
    // 1. De-interleave

    galileo_e5a_telemetry_decoder_cc::deinterleaver(GALILEO_FNAV_INTERLEAVER_ROWS, GALILEO_FNAV_INTERLEAVER_COLS, page_symbols, page_symbols_deint);

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
    int page_bits[frame_length/2];
    galileo_e5a_telemetry_decoder_cc::viterbi_decoder(page_symbols_deint, page_bits);

    // 3. Call the Galileo page decoder
    std::string page_String;
    for(int i = 0; i < frame_length; i++)
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
    if(d_nav.flag_CRC_test == true)
        {
            LOG(INFO) << "Galileo CRC correct on channel " << d_channel << " from satellite " << d_satellite;
            std::cout << "Galileo CRC correct on channel " << d_channel << " from satellite " << d_satellite << std::endl;
        }
    else
        {
            std::cout << "Galileo CRC error on channel " << d_channel << " from satellite " << d_satellite << std::endl;
            LOG(INFO)<< "Galileo CRC error on channel " << d_channel << " from satellite " << d_satellite;
        }

    // 4. Push the new navigation data to the queues
    if (d_nav.have_new_ephemeris() == true)
        {
            std::shared_ptr<Galileo_Ephemeris> tmp_obj= std::make_shared<Galileo_Ephemeris>(d_nav.get_ephemeris());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
        }
    if (d_nav.have_new_iono_and_GST() == true)
        {
            std::shared_ptr<Galileo_Iono> tmp_obj= std::make_shared<Galileo_Iono>(d_nav.get_iono());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
        }
    if (d_nav.have_new_utc_model() == true)
        {
            std::shared_ptr<Galileo_Utc_Model> tmp_obj= std::make_shared<Galileo_Utc_Model>(d_nav.get_utc_model());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
        }

}

galileo_e5a_telemetry_decoder_cc::galileo_e5a_telemetry_decoder_cc(
        Gnss_Satellite satellite,
        bool dump) :
                   gr::block("galileo_e5a_telemetry_decoder_cc", gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
                           gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    // Telemetry Bit transition synchronization port out
    this->message_port_register_out(pmt::mp("preamble_timestamp_s"));
    // Ephemeris data port out
    this->message_port_register_out(pmt::mp("telemetry"));
    // initialize internal vars
    d_dump = dump;
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    LOG(INFO) << "GALILEO E5A TELEMETRY PROCESSING: satellite " << d_satellite;
    //d_samples_per_symbol = ( Galileo_E5a_CODE_CHIP_RATE_HZ / Galileo_E5a_CODE_LENGTH_CHIPS ) / Galileo_E1_B_SYMBOL_RATE_BPS;

    // set the preamble
    //unsigned short int preambles_bits[GALILEO_FNAV_PREAMBLE_LENGTH_BITS] = GALILEO_FNAV_PREAMBLE;
    for (int i = 0; i < GALILEO_FNAV_PREAMBLE_LENGTH_BITS; i++)
        {
            if (GALILEO_FNAV_PREAMBLE.at(i) == '0')
                {
                    d_preamble_bits[i] = 1;
                }
            else
                {
                    d_preamble_bits[i] = -1;
                }
        }

    //    memcpy((unsigned short int*)this->d_preambles_bits, (unsigned short int*)preambles_bits, GALILEO_FNAV_PREAMBLE_LENGTH_BITS*sizeof(unsigned short int));

    //    // preamble bits to sampled symbols
    //    d_preambles_symbols = (signed int*)malloc(sizeof(signed int) * GALILEO_FNAV_SAMPLES_PER_PREAMBLE);
    //    int n = 0;
    //    for (int i = 0; i < GALILEO_FNAV_PREAMBLE_LENGTH_BITS; i++)
    //        {
    //            for (unsigned int j = 0; j < GALILEO_FNAV_SAMPLES_PER_SYMBOL; j++)
    //                {
    //                    if (d_preambles_bits[i] == 1)
    //                        {
    //                            d_preambles_symbols[n] = 1;
    //                        }
    //                    else
    //                        {
    //                            d_preambles_symbols[n] = -1;
    //                        }
    //                    n++;
    //                }
    //        }
    //
    d_sample_counter = 0;
    d_state = 0;
    d_preamble_lock = false;
    d_preamble_index = 0;
    d_preamble_time_seconds = 0;
    d_flag_frame_sync = false;
    d_current_symbol = 0;
    d_prompt_counter = 0;
    d_symbol_counter = 0;

    d_TOW_at_Preamble = 0;
    d_TOW_at_current_symbol = 0;

    d_CRC_error_counter = 0;
    d_sign_init = 0;

    d_flag_preamble = false;
    d_channel = 0;
    Prn_timestamp_at_preamble_ms = 0;
    flag_TOW_set = false;
}


galileo_e5a_telemetry_decoder_cc::~galileo_e5a_telemetry_decoder_cc()
{
    d_dump_file.close();
}


int galileo_e5a_telemetry_decoder_cc::general_work (int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
        gr_vector_const_void_star &input_items,    gr_vector_void_star &output_items)
{
    //
    const Gnss_Synchro **in = (const Gnss_Synchro **)  &input_items[0]; //Get the input samples pointer
    Gnss_Synchro **out = (Gnss_Synchro **) &output_items[0];

    /* Terminology:     Prompt: output from tracking Prompt correlator (Prompt samples)
     *             Symbol: encoded navigation bits. 1 symbol = 20 samples in E5a
     *             Bit: decoded navigation bits forming words as described in Galileo ICD
     * States:     0 Receiving dummy samples.
     *         1 Preamble not locked
     *         3 Preamble lock
     */
    switch (d_state)
    {
    case 0:
        {
            if (in[0][0].Prompt_I != 0)
                {
                    d_current_symbol += in[0][0].Prompt_I;
                    if (d_prompt_counter == GALILEO_FNAV_CODES_PER_SYMBOL - 1)
                        {
                            if (d_current_symbol > 0)
                                {
                                    d_page_symbols[d_symbol_counter] = 1;
                                }
                            else
                                {
                                    d_page_symbols[d_symbol_counter] = -1;
                                }
                            d_current_symbol = 0;
                            d_symbol_counter++;
                            d_prompt_counter = 0;
                            if (d_symbol_counter == GALILEO_FNAV_PREAMBLE_LENGTH_BITS - 1)
                                {
                                    d_state = 1;
                                }
                        }
                    else
                        {
                            d_prompt_counter++;
                        }
                }
            break;
        }
    case 1:
        {
            d_current_symbol += in[0][0].Prompt_I;
            if (d_prompt_counter == GALILEO_FNAV_CODES_PER_SYMBOL - 1)
                {
                    if (d_current_symbol > 0)
                        {
                            d_page_symbols[d_symbol_counter] = 1;
                        }
                    else
                        {
                            d_page_symbols[d_symbol_counter] = -1;
                        }
                    //            d_page_symbols[d_symbol_counter] = d_current_symbol_float/(float)GALILEO_FNAV_CODES_PER_SYMBOL;
                    d_current_symbol = 0;
                    d_symbol_counter++;
                    d_prompt_counter = 0;
                    // **** Attempt Preamble correlation ****
                    bool corr_flag=true;
                    int corr_sign = 0; // sequence can be found inverted
                    // check if the preamble starts positive correlated or negative correlated
                    if (d_page_symbols[d_symbol_counter - GALILEO_FNAV_PREAMBLE_LENGTH_BITS] < 0)    // symbols clipping
                        {
                            corr_sign=-d_preamble_bits[0];
                        }
                    else
                        {
                            corr_sign=d_preamble_bits[0];
                        }
                    // the preamble is fully correlated only if maintains corr_sign along the whole sequence
                    for (int i = 1; i < GALILEO_FNAV_PREAMBLE_LENGTH_BITS; i++)
                        {
                            if (d_page_symbols[d_symbol_counter - GALILEO_FNAV_PREAMBLE_LENGTH_BITS + i] < 0 && d_preamble_bits[i]+corr_sign != 0)
                                {
                                    //exit for
                                    corr_flag=false;
                                    break;
                                }
                            if (d_page_symbols[d_symbol_counter - GALILEO_FNAV_PREAMBLE_LENGTH_BITS + i] > 0 && d_preamble_bits[i]+corr_sign == 0)
                                {
                                    //exit for
                                    corr_flag=false;
                                    break;
                                }
                        }
                    //
                    if (corr_flag==true) // preamble fully correlates
                        {
                            d_preamble_index = d_sample_counter - GALILEO_FNAV_CODES_PER_PREAMBLE;//record the preamble sample stamp. Remember correlation appears at the end of the preamble in this design
                            LOG(INFO) << "Preamble detection for Galileo SAT " << this->d_satellite << std::endl;
                            d_symbol_counter = 0; // d_page_symbols start right after preamble and finish at the end of next preamble.
                            d_state = 2; // preamble lock
                        }
                    if (d_symbol_counter >= GALILEO_FNAV_SYMBOLS_PER_PAGE + GALILEO_FNAV_PREAMBLE_LENGTH_BITS)
                        {
                            d_symbol_counter = GALILEO_FNAV_PREAMBLE_LENGTH_BITS; // prevents overflow
                        }
                }
            else
                {
                    d_prompt_counter++;
                }
            break;
        }
    case 2:
        {
            d_current_symbol += in[0][0].Prompt_I;
            if (d_prompt_counter == GALILEO_FNAV_CODES_PER_SYMBOL - 1)
                {
                    if (d_current_symbol > 0)
                        {
                            d_page_symbols[d_symbol_counter] = 1;
                        }
                    else
                        {
                            d_page_symbols[d_symbol_counter] = -1;
                        }
                    // d_page_symbols[d_symbol_counter] = d_current_symbol_float/(float)GALILEO_FNAV_CODES_PER_SYMBOL;
                    d_current_symbol = 0;
                    d_symbol_counter++;
                    d_prompt_counter = 0;
                    // At the right sample stamp, check preamble synchro
                    if (d_sample_counter == d_preamble_index + GALILEO_FNAV_CODES_PER_PAGE + GALILEO_FNAV_CODES_PER_PREAMBLE)
                        {
                            // **** Attempt Preamble correlation ****
                            bool corr_flag = true;
                            int corr_sign = 0; // sequence can be found inverted
                            //    corr_sign = d_preamble_bits[0] * d_page_symbols[d_symbol_counter - GALILEO_FNAV_PREAMBLE_LENGTH_BITS];
                            //    for (int i = 1; i < GALILEO_FNAV_PREAMBLE_LENGTH_BITS; i++)
                            //       {
                            //          if ((d_preamble_bits[i] * d_page_symbols[i + d_symbol_counter - GALILEO_FNAV_PREAMBLE_LENGTH_BITS]) != corr_sign)
                            //              {
                            //                  //exit for if one bit doesn't correlate
                            //                corr_flag = false;
                            //                  break;
                            //              }
                            //     }
                            // check if the preamble starts positive correlated or negative correlated
                            if (d_page_symbols[d_symbol_counter - GALILEO_FNAV_PREAMBLE_LENGTH_BITS] < 0)    // symbols clipping
                                {
                                    corr_sign=-d_preamble_bits[0];
                                }
                            else
                                {
                                    corr_sign=d_preamble_bits[0];
                                }
                            // the preamble is fully correlated only if maintains corr_sign along the whole sequence
                            for (int i = 1; i < GALILEO_FNAV_PREAMBLE_LENGTH_BITS; i++)
                                {
                                    if (d_page_symbols[d_symbol_counter - GALILEO_FNAV_PREAMBLE_LENGTH_BITS + i] < 0 && d_preamble_bits[i]+corr_sign != 0)
                                        {
                                            //exit for
                                            corr_flag=false;
                                            break;
                                        }
                                    if (d_page_symbols[d_symbol_counter - GALILEO_FNAV_PREAMBLE_LENGTH_BITS + i] > 0 && d_preamble_bits[i]+corr_sign == 0)
                                        {
                                            //exit for
                                            corr_flag=false;
                                            break;
                                        }
                                }

                            if (corr_flag==true) // NEW PREAMBLE RECEIVED. DECODE PAGE
                                {
                                    d_preamble_index = d_sample_counter - GALILEO_FNAV_CODES_PER_PREAMBLE;//record the preamble sample stamp
                                    // DECODE WORD
                                    decode_word(d_page_symbols, GALILEO_FNAV_SYMBOLS_PER_PAGE - GALILEO_FNAV_PREAMBLE_LENGTH_BITS);
                                    // CHECK CRC
                                    if (d_nav.flag_CRC_test == true)
                                        {
                                            d_CRC_error_counter = 0;
                                            d_flag_preamble = true; //valid preamble indicator (initialized to false every work())
                                            d_preamble_time_seconds = in[0][0].Tracking_timestamp_secs - (static_cast<double>(GALILEO_FNAV_CODES_PER_PAGE+GALILEO_FNAV_CODES_PER_PREAMBLE) * GALILEO_E5a_CODE_PERIOD); //record the PRN start sample index associated to the preamble start.
                                            if (!d_flag_frame_sync)
                                                {
                                                    d_flag_frame_sync = true;
                                                    LOG(INFO) <<" Frame sync SAT " << this->d_satellite << " with preamble start at " << d_preamble_time_seconds << " [s]";
                                                }
                                            d_symbol_counter = 0; // d_page_symbols start right after preamble and finish at the end of next preamble.
                                        }
                                    else
                                        {
                                            d_CRC_error_counter++;
                                            if (d_CRC_error_counter > CRC_ERROR_LIMIT)
                                                {
                                                    LOG(INFO) << "Lost of frame sync SAT " << this->d_satellite;
                                                    d_state = 1;
                                                    d_symbol_counter = GALILEO_FNAV_PREAMBLE_LENGTH_BITS; // prevents overflow
                                                    d_flag_frame_sync = false;
                                                }
                                            else
                                                {
                                                    d_symbol_counter = 0; // d_page_symbols start right after preamble and finish at the end of next preamble.
                                                }
                                        }
                                }
                        }
                }
            else
                {
                    d_prompt_counter++;
                }
            break;
        }
    }
    consume_each(1);

    // UPDATE GNSS SYNCHRO DATA
    Gnss_Synchro current_synchro_data; //structure to save the synchronization information and send the output object to the next block
    //1. Copy the current tracking output
    current_synchro_data = in[0][0];
    //2. Add the telemetry decoder information
    if (this->d_flag_preamble == true and d_nav.flag_TOW_set == true)
        //update TOW at the preamble instant
        //We expect a preamble each 10 seconds (FNAV page period)
        {
            Prn_timestamp_at_preamble_ms = d_preamble_time_seconds * 1000;
            //Prn_timestamp_at_preamble_ms = in[0][0].Tracking_timestamp_secs * 1000.0;
            if (d_nav.flag_TOW_1 == true)
                {
                    d_TOW_at_Preamble = d_nav.FNAV_TOW_1;
                    d_TOW_at_current_symbol = d_TOW_at_Preamble + (static_cast<double>(GALILEO_FNAV_CODES_PER_PAGE+GALILEO_FNAV_CODES_PER_PREAMBLE) * GALILEO_E5a_CODE_PERIOD);
                    d_nav.flag_TOW_1 = false;
                }
            if (d_nav.flag_TOW_2 == true)
                {
                    d_TOW_at_Preamble = d_nav.FNAV_TOW_2;
                    d_TOW_at_current_symbol = d_TOW_at_Preamble + (static_cast<double>(GALILEO_FNAV_CODES_PER_PAGE+GALILEO_FNAV_CODES_PER_PREAMBLE) * GALILEO_E5a_CODE_PERIOD);
                    d_nav.flag_TOW_2 = false;
                }
            if (d_nav.flag_TOW_3 == true)
                {
                    d_TOW_at_Preamble = d_nav.FNAV_TOW_3;
                    d_TOW_at_current_symbol = d_TOW_at_Preamble + (static_cast<double>(GALILEO_FNAV_CODES_PER_PAGE+GALILEO_FNAV_CODES_PER_PREAMBLE) * GALILEO_E5a_CODE_PERIOD);
                    d_nav.flag_TOW_3 = false;
                }
            if (d_nav.flag_TOW_4 == true)
                {
                    d_TOW_at_Preamble = d_nav.FNAV_TOW_4;
                    d_TOW_at_current_symbol = d_TOW_at_Preamble + (static_cast<double>(GALILEO_FNAV_CODES_PER_PAGE+GALILEO_FNAV_CODES_PER_PREAMBLE) * GALILEO_E5a_CODE_PERIOD);
                    d_nav.flag_TOW_4 = false;
                }
            else
                {
                    //this page has no timming information
                    d_TOW_at_Preamble = d_TOW_at_Preamble + GALILEO_FNAV_SECONDS_PER_PAGE;
                    d_TOW_at_current_symbol =  d_TOW_at_current_symbol + GALILEO_E5a_CODE_PERIOD;
                }

        }
    else //if there is not a new preamble, we define the TOW of the current symbol
        {
            d_TOW_at_current_symbol = d_TOW_at_current_symbol + GALILEO_E5a_CODE_PERIOD;
        }

    //if (d_flag_frame_sync == true and d_nav.flag_TOW_set==true and d_nav.flag_CRC_test == true)
    if (d_flag_frame_sync == true and d_nav.flag_TOW_set == true)
        {
            current_synchro_data.Flag_valid_word = true;
        }
    else
        {
            current_synchro_data.Flag_valid_word = false;
        }

    current_synchro_data.d_TOW = d_TOW_at_Preamble;
    current_synchro_data.d_TOW_at_current_symbol = d_TOW_at_current_symbol;
    current_synchro_data.Flag_preamble = d_flag_preamble;
    current_synchro_data.Prn_timestamp_ms = in[0][0].Tracking_timestamp_secs * 1000.0;
    current_synchro_data.Prn_timestamp_at_preamble_ms = Prn_timestamp_at_preamble_ms;

    if(d_dump == true)
        {
            // MULTIPLEXED FILE RECORDING - Record results to file
            try
            {
                    double tmp_double;
                    tmp_double = d_TOW_at_current_symbol;
                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                    tmp_double = current_synchro_data.Prn_timestamp_ms;
                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                    tmp_double = d_TOW_at_Preamble;
                    d_dump_file.write((char*)&tmp_double, sizeof(double));
            }
            catch (const std::ifstream::failure& e)
            {
                    LOG(WARNING) << "Exception writing observables dump file " << e.what();
            }
        }
    d_sample_counter++; //count for the processed samples
    //3. Make the output (copy the object contents to the GNURadio reserved memory)
    *out[0] = current_synchro_data;
    return 1;
}


void galileo_e5a_telemetry_decoder_cc::set_satellite(Gnss_Satellite satellite)
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
                            d_dump_file.exceptions ( std::ifstream::failbit | std::ifstream::badbit );
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "Telemetry decoder dump enabled on channel " << d_channel << " Log file: " << d_dump_filename.c_str();
                    }
                    catch (const std::ifstream::failure& e)
                    {
                            LOG(WARNING) << "channel " << d_channel << " Exception opening trk dump file " << e.what();
                    }
                }
        }
}
