/*!
 * \file galileo_e1b_telemetry_decoder_cc.cc
 * \brief Implementation of a Galileo INAV message demodulator block
 * \author Mara Branzanti 2013. mara.branzanti(at)gmail.com
 * \author Javier Arribas 2013. jarribas(at)cttc.es
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


#include "galileo_e1b_telemetry_decoder_cc.h"
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


galileo_e1b_telemetry_decoder_cc_sptr
galileo_e1b_make_telemetry_decoder_cc(Gnss_Satellite satellite, bool dump)
{
    return galileo_e1b_telemetry_decoder_cc_sptr(new galileo_e1b_telemetry_decoder_cc(satellite, dump));
}



void galileo_e1b_telemetry_decoder_cc::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
    if(noutput_items != 0)
        {
            ninput_items_required[0] = GALILEO_INAV_PAGE_SYMBOLS; // set the required sample history
        }
}


void galileo_e1b_telemetry_decoder_cc::viterbi_decoder(double *page_part_symbols, int *page_part_bits)
{
    int CodeLength = 240;
    int DataLength;
    int nn, KK, mm, max_states;
    int g_encoder[2];

    nn = 2;             // Coding rate 1/n
    KK = 7;             // Constraint Length
    g_encoder[0] = 121; // Polynomial G1
    g_encoder[1] = 91;  // Polynomial G2

    mm = KK - 1;
    max_states = 1 << mm; /* 2^mm */
    DataLength = (CodeLength/nn) - mm;

    /* create appropriate transition matrices */
    int *out0, *out1, *state0, *state1;
    out0 = (int*)calloc( max_states, sizeof(int) );
    out1 = (int*)calloc( max_states, sizeof(int) );
    state0 = (int*)calloc( max_states, sizeof(int) );
    state1 = (int*)calloc( max_states, sizeof(int) );

    nsc_transit( out0, state0, 0, g_encoder, KK, nn );
    nsc_transit( out1, state1, 1, g_encoder, KK, nn );

    Viterbi(page_part_bits, out0, state0, out1, state1,
            page_part_symbols, KK, nn, DataLength );

    /* Clean up memory */
    free( out0 );
    free( out1 );
    free( state0 );
    free( state1 );
}


void galileo_e1b_telemetry_decoder_cc::deinterleaver(int rows, int cols, double *in, double *out)
{
    for (int r = 0; r < rows; r++)
        {
            for(int c = 0; c < cols; c++)
                {
                    out[c*rows + r] = in[r*cols + c];
                }
        }
}


galileo_e1b_telemetry_decoder_cc::galileo_e1b_telemetry_decoder_cc(
        Gnss_Satellite satellite,
        bool dump) :
                   gr::block("galileo_e1b_telemetry_decoder_cc", gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
                           gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    // Telemetry Bit transition synchronization port out
    this->message_port_register_out(pmt::mp("preamble_timestamp_s"));
    // Ephemeris data port out
    this->message_port_register_out(pmt::mp("telemetry"));
    // initialize internal vars
    d_dump = dump;
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    LOG(INFO) << "Initializing GALILEO E1B TELEMETRY PROCESSING";
    d_samples_per_symbol = ( Galileo_E1_CODE_CHIP_RATE_HZ / Galileo_E1_B_CODE_LENGTH_CHIPS ) / Galileo_E1_B_SYMBOL_RATE_BPS;

    // set the preamble
    unsigned short int preambles_bits[GALILEO_INAV_PREAMBLE_LENGTH_BITS] = GALILEO_INAV_PREAMBLE;

    d_symbols_per_preamble = GALILEO_INAV_PREAMBLE_LENGTH_BITS * d_samples_per_symbol;

    memcpy((unsigned short int*)this->d_preambles_bits, (unsigned short int*)preambles_bits, GALILEO_INAV_PREAMBLE_LENGTH_BITS*sizeof(unsigned short int));

    // preamble bits to sampled symbols
    d_preambles_symbols = (signed int*)malloc(sizeof(signed int) * d_symbols_per_preamble);
    int n = 0;
    for (int i = 0; i < GALILEO_INAV_PREAMBLE_LENGTH_BITS; i++)
        {
            for (unsigned int j = 0; j < d_samples_per_symbol; j++)
                {
                    if (d_preambles_bits[i] == 1)
                        {
                            d_preambles_symbols[n] = 1;
                        }
                    else
                        {
                            d_preambles_symbols[n] = -1;
                        }
                    n++;
                }
        }
    d_sample_counter = 0;
    d_stat = 0;
    d_preamble_index = 0;

    d_preamble_time_seconds = 0;
    d_flag_frame_sync = false;

    d_flag_parity = false;
    d_TOW_at_Preamble = 0;
    d_TOW_at_current_symbol = 0;
    delta_t = 0;
    d_CRC_error_counter = 0;
    flag_even_word_arrived = 0;
    d_flag_preamble = false;
    d_channel = 0;
    Prn_timestamp_at_preamble_ms = 0.0;
    flag_TOW_set = false;
    d_average_count = 0;
    d_decimation_output_factor = 1;
}


galileo_e1b_telemetry_decoder_cc::~galileo_e1b_telemetry_decoder_cc()
{
    delete d_preambles_symbols;
    d_dump_file.close();
}


void galileo_e1b_telemetry_decoder_cc::decode_word(double *page_part_symbols,int frame_length)
{
    double page_part_symbols_deint[frame_length];
    // 1. De-interleave
    deinterleaver(GALILEO_INAV_INTERLEAVER_ROWS, GALILEO_INAV_INTERLEAVER_COLS, page_part_symbols, page_part_symbols_deint);

    // 2. Viterbi decoder
    // 2.1 Take into account the NOT gate in G2 polynomial (Galileo ICD Figure 13, FEC encoder)
    // 2.2 Take into account the possible inversion of the polarity due to PLL lock at 180�
    for (int i = 0; i < frame_length; i++)
        {
            if ((i + 1) % 2 == 0)
                {
                    page_part_symbols_deint[i] = -page_part_symbols_deint[i];
                }
        }

    int page_part_bits[frame_length/2];
    viterbi_decoder(page_part_symbols_deint, page_part_bits);

    // 3. Call the Galileo page decoder
    std::string page_String;
    for(int i = 0; i < (frame_length/2); i++)
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
            d_nav.split_page(page_String, flag_even_word_arrived);
            if(d_nav.flag_CRC_test == true)
                {
                    LOG(INFO) << "Galileo CRC correct on channel " << d_channel << " from satellite " << d_satellite;
                    std::cout << "Galileo CRC correct on channel " << d_channel << " from satellite " << d_satellite << std::endl;
                }
            else
                {
                    std::cout << "Galileo CRC error on channel " << d_channel <<  " from satellite " << d_satellite << std::endl;
                    LOG(INFO) << "Galileo CRC error on channel " << d_channel <<  " from satellite " << d_satellite;
                }
            flag_even_word_arrived = 0;
        }
    else
        {
            // STORE HALF WORD (even page)
            d_nav.split_page(page_String.c_str(), flag_even_word_arrived);
            flag_even_word_arrived = 1;
        }

    // 4. Push the new navigation data to the queues
    if (d_nav.have_new_ephemeris() == true)
        {
            // get object for this SV (mandatory)
            std::shared_ptr<Galileo_Ephemeris> tmp_obj = std::make_shared<Galileo_Ephemeris>(d_nav.get_ephemeris());

            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));

        }
    if (d_nav.have_new_iono_and_GST() == true)
        {
            // get object for this SV (mandatory)
            std::shared_ptr<Galileo_Iono> tmp_obj = std::make_shared<Galileo_Iono>(d_nav.get_iono());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
        }
    if (d_nav.have_new_utc_model() == true)
        {
            // get object for this SV (mandatory)
            std::shared_ptr<Galileo_Utc_Model> tmp_obj = std::make_shared<Galileo_Utc_Model>(d_nav.get_utc_model());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
        }
    if (d_nav.have_new_almanac() == true)
        {
            std::shared_ptr<Galileo_Almanac> tmp_obj= std::make_shared<Galileo_Almanac>(d_nav.get_almanac());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            //debug
            std::cout << "Galileo almanac received!" << std::endl;
            LOG(INFO) << "GPS_to_Galileo time conversion:";
            LOG(INFO) << "A0G=" << tmp_obj->A_0G_10;
            LOG(INFO) << "A1G=" << tmp_obj->A_1G_10;
            LOG(INFO) << "T0G=" << tmp_obj->t_0G_10;
            LOG(INFO) << "WN_0G_10=" << tmp_obj->WN_0G_10;
            LOG(INFO) << "Current parameters:";
            LOG(INFO) << "d_TOW_at_current_symbol=" << d_TOW_at_current_symbol;
            LOG(INFO) << "d_nav.WN_0=" << d_nav.WN_0;
            delta_t = tmp_obj->A_0G_10 + tmp_obj->A_1G_10 * (d_TOW_at_current_symbol - tmp_obj->t_0G_10 + 604800 * (fmod((d_nav.WN_0 - tmp_obj->WN_0G_10), 64)));
            LOG(INFO) << "delta_t=" << delta_t << "[s]";
        }
}




int galileo_e1b_telemetry_decoder_cc::general_work (int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    int corr_value = 0;
    int preamble_diff = 0;

    Gnss_Synchro **out = (Gnss_Synchro **) &output_items[0];
    d_sample_counter++; //count for the processed samples

    // ########### Output the tracking data to navigation and PVT ##########
    const Gnss_Synchro **in = (const Gnss_Synchro **)  &input_items[0]; //Get the input samples pointer

    // TODO Optimize me!
    //******* preamble correlation ********
    for (int i = 0; i < d_symbols_per_preamble; i++)
        {
            if (in[0][i].Prompt_I < 0)    // symbols clipping
                {
                    corr_value -= d_preambles_symbols[i];
                }
            else
                {
                    corr_value += d_preambles_symbols[i];
                }
        }
    d_flag_preamble = false;

    //******* frame sync ******************
    if (d_stat == 0) //no preamble information
        {
            if (abs(corr_value) >= d_symbols_per_preamble)
                {
                    d_preamble_index = d_sample_counter;//record the preamble sample stamp
                    LOG(INFO) << "Preamble detection for Galileo SAT " << this->d_satellite;
                    d_stat = 1; // enter into frame pre-detection status
                }
        }
    else if (d_stat == 1) // posible preamble lock
        {
            if (abs(corr_value) >= d_symbols_per_preamble)
                {
                    //check preamble separation
                    preamble_diff = d_sample_counter - d_preamble_index;
                    if (abs(preamble_diff - GALILEO_INAV_PREAMBLE_PERIOD_SYMBOLS) == 0)
                        {
                            //try to decode frame
                            LOG(INFO) << "Starting page decoder for Galileo SAT " << this->d_satellite;
                            d_preamble_index = d_sample_counter; //record the preamble sample stamp
                            d_stat = 2;
                        }
                    else
                        {
                            if (preamble_diff > GALILEO_INAV_PREAMBLE_PERIOD_SYMBOLS)
                                {
                                    d_stat = 0; // start again
                                }
                        }
                }
        }
    else if (d_stat == 2)
        {
            if (d_sample_counter == d_preamble_index + GALILEO_INAV_PREAMBLE_PERIOD_SYMBOLS)
                {
                    // NEW Galileo page part is received
                    // 0. fetch the symbols into an array
                    int frame_length = GALILEO_INAV_PAGE_PART_SYMBOLS - d_symbols_per_preamble;
                    double page_part_symbols[frame_length];

                    for (int i = 0; i < frame_length; i++)
                        {
                            if (corr_value > 0)
                                {
                                    page_part_symbols[i] = in[0][i + d_symbols_per_preamble].Prompt_I; // because last symbol of the preamble is just received now!

                                }
                            else
                                {
                                    page_part_symbols[i] = -in[0][i + d_symbols_per_preamble].Prompt_I; // because last symbol of the preamble is just received now!
                                }
                        }

                    //call the decoder
                    decode_word(page_part_symbols, frame_length);
                    if (d_nav.flag_CRC_test == true)
                        {
                            d_CRC_error_counter = 0;
                            d_flag_preamble = true; //valid preamble indicator (initialized to false every work())
                            d_preamble_index = d_sample_counter;  //record the preamble sample stamp (t_P)
                            d_preamble_time_seconds = in[0][0].Tracking_timestamp_secs; // - d_preamble_duration_seconds; //record the PRN start sample index associated to the preamble
                            if (!d_flag_frame_sync)
                                {
                                    d_flag_frame_sync = true;
                                    LOG(INFO) << " Frame sync SAT " << this->d_satellite << " with preamble start at " << d_preamble_time_seconds << " [s]";
                                }
                        }
                    else
                        {
                            d_CRC_error_counter++;
                            d_preamble_index = d_sample_counter;  //record the preamble sample stamp
                            if (d_CRC_error_counter > CRC_ERROR_LIMIT)
                                {
                                    LOG(INFO) << "Lost of frame sync SAT " << this->d_satellite;
                                    d_flag_frame_sync = false;
                                    d_stat = 0;
                                }
                        }
                }
        }

    consume_each(1); //one by one

    // UPDATE GNSS SYNCHRO DATA
    Gnss_Synchro current_synchro_data; //structure to save the synchronization information and send the output object to the next block
    //1. Copy the current tracking output
    current_synchro_data = in[0][0];
    //2. Add the telemetry decoder information
    if (this->d_flag_preamble == true and d_nav.flag_TOW_set == true)
        //update TOW at the preamble instant
        // JAVI: 30/06/2014
        // TOW, in Galileo, is referred to the START of the PAGE PART, that is, THE FIRST SYMBOL OF THAT PAGE, NOT THE PREAMBLE.
        // thus, no correction should be done. d_TOW_at_Preamble should be renamed to d_TOW_at_page_start.
        // Since we detected the preamble, then, we are in the last symbol of that preamble, or just at the start of the first page symbol.
        //flag preamble is true after the all page (even and odd) is received. I/NAV page period is 2 SECONDS
        {
            Prn_timestamp_at_preamble_ms = in[0][0].Tracking_timestamp_secs * 1000.0;
            if(d_nav.flag_TOW_5 == true) //page 5 arrived and decoded, so we are in the odd page (since Tow refers to the even page, we have to add 1 sec)
                {
                    //std::cout<< "Using TOW_5 for timestamping" << std::endl;
                    d_TOW_at_Preamble = d_nav.TOW_5 + GALILEO_INAV_PAGE_PART_SECONDS; //TOW_5 refers to the even preamble, but when we decode it we are in the odd part, so 1 second later
                    /* 1  sec (GALILEO_INAV_PAGE_PART_SYMBOLS*GALILEO_E1_CODE_PERIOD) is added because
                     * if we have a TOW value it means that we are at the beginning of the last page part
                     * (GNU Radio history keeps in a buffer the rest of the incoming frame part)*/
                    d_TOW_at_current_symbol = d_TOW_at_Preamble;//-GALILEO_E1_CODE_PERIOD;//+ (double)GALILEO_INAV_PREAMBLE_LENGTH_BITS/(double)GALILEO_TELEMETRY_RATE_BITS_SECOND;
                    d_nav.flag_TOW_5 = false;
                }

            else if(d_nav.flag_TOW_6 == true) //page 6 arrived and decoded, so we are in the odd page (since Tow refers to the even page, we have to add 1 sec)
                {
                    //std::cout<< "Using TOW_6 for timestamping" << std::endl;
                    d_TOW_at_Preamble = d_nav.TOW_6 + GALILEO_INAV_PAGE_PART_SECONDS;
                    //TOW_6 refers to the even preamble, but when we decode it we are in the odd part, so 1 second later
                    /* 1  sec (GALILEO_INAV_PAGE_PART_SYMBOLS*GALILEO_E1_CODE_PERIOD) is added because
                     * if we have a TOW value it means that we are at the beginning of the last page part
                     * (GNU Radio history keeps in a buffer the rest of the incoming frame part)*/
                    d_TOW_at_current_symbol = d_TOW_at_Preamble;//-GALILEO_E1_CODE_PERIOD;//+ (double)GALILEO_INAV_PREAMBLE_LENGTH_BITS/(double)GALILEO_TELEMETRY_RATE_BITS_SECOND;
                    d_nav.flag_TOW_6 = false;
                }
            else
                {
                    //this page has no timing information
                    d_TOW_at_Preamble = d_TOW_at_Preamble + GALILEO_INAV_PAGE_SECONDS;
                    d_TOW_at_current_symbol = d_TOW_at_current_symbol + GALILEO_E1_CODE_PERIOD;// + GALILEO_INAV_PAGE_PART_SYMBOLS*GALILEO_E1_CODE_PERIOD;
                }
        }
    else //if there is not a new preamble, we define the TOW of the current symbol
        {
            d_TOW_at_current_symbol = d_TOW_at_current_symbol + GALILEO_E1_CODE_PERIOD;
        }

    //if (d_flag_frame_sync == true and d_nav.flag_TOW_set==true and d_nav.flag_CRC_test == true)

    if(d_nav.flag_GGTO_1 == true  and  d_nav.flag_GGTO_2 == true and  d_nav.flag_GGTO_3 == true and  d_nav.flag_GGTO_4 == true) //all GGTO parameters arrived
        {
            delta_t = d_nav.A_0G_10 + d_nav.A_1G_10 * (d_TOW_at_current_symbol - d_nav.t_0G_10 + 604800.0 * (fmod((d_nav.WN_0 - d_nav.WN_0G_10), 64)));
        }

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
    current_synchro_data.d_TOW_hybrid_at_current_symbol = current_synchro_data.d_TOW_at_current_symbol - delta_t; //delta_t = t_gal - t_gps  ---->  t_gps = t_gal -delta_t
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
    //todo: implement averaging
    d_average_count++;

    if (d_average_count == d_decimation_output_factor)
        {
            d_average_count = 0;
            //3. Make the output (copy the object contents to the GNURadio reserved memory)
            *out[0] = current_synchro_data;
            //std::cout<<"GPS L1 TLM output on CH="<<this->d_channel << " SAMPLE STAMP="<<d_sample_counter/d_decimation_output_factor<<std::endl;
            return 1;
        }
    else
        {
            return 0;
        }

}


void galileo_e1b_telemetry_decoder_cc::set_decimation(int decimation)
{
    d_decimation_output_factor = decimation;
}


void galileo_e1b_telemetry_decoder_cc::set_satellite(Gnss_Satellite satellite)
{
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << "Setting decoder Finite State Machine to satellite " << d_satellite;
    DLOG(INFO) << "Navigation Satellite set to " << d_satellite;
}


void galileo_e1b_telemetry_decoder_cc::set_channel(int channel)
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



