/*!
 * \file galileo_e5a_telemetry_decoder_cc.cc
 * \brief Implementation of a Galileo FNAV message demodulator block
 * \author Marc Sales 2014. marcsales92(at)gmail.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2014  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include "control_message_factory.h"
#include "galileo_navigation_message.h"
#include "gnss_synchro.h"
#include "convolutional.h"

#define CRC_ERROR_LIMIT 6

using google::LogMessage;


galileo_e5a_telemetry_decoder_cc_sptr
galileo_e5a_make_telemetry_decoder_cc(Gnss_Satellite satellite, long if_freq, long fs_in, unsigned
		int vector_length, boost::shared_ptr<gr::msg_queue> queue, bool dump)
{
    return galileo_e5a_telemetry_decoder_cc_sptr(new galileo_e5a_telemetry_decoder_cc(satellite, if_freq,
            fs_in, vector_length, queue, dump));
}

void galileo_e5a_telemetry_decoder_cc::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
    ninput_items_required[0] = GALILEO_FNAV_SYMBOLS_PER_PAGE; // set the required sample history
}

void galileo_e5a_telemetry_decoder_cc::viterbi_decoder(double *page_part_symbols, int *page_part_bits)
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
    int page_part_bits[frame_length];
    viterbi_decoder(page_symbols_deint, page_part_bits);

    // 3. Call the Galileo page decoder
    std::string page_String;
    for(int i = 0; i < frame_length; i++)
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

    // DECODE COMPLETE WORD (even + odd) and TEST CRC
    d_nav.split_page(page_String);
    if(d_nav.flag_CRC_test == true)
        {
            LOG(INFO) << "Galileo CRC correct on channel " << d_channel;
            std::cout << "Galileo CRC correct on channel " << d_channel << std::endl;
        }
    else
        {
            std::cout << "Galileo CRC error on channel " << d_channel << std::endl;
            LOG(INFO)<< "Galileo CRC error on channel " << d_channel;
        }

    // 4. Push the new navigation data to the queues
     if (d_nav.have_new_ephemeris() == true)
         {
             // get ephemeris object for this SV
             Galileo_Ephemeris ephemeris = d_nav.get_ephemeris();//notice that the read operation will clear the valid flag
             //std::cout<<"New Galileo Ephemeris received for SV "<<d_satellite.get_PRN()<<std::endl;
             d_ephemeris_queue->push(ephemeris);
         }
     if (d_nav.have_new_iono_and_GST() == true)
         {
             Galileo_Iono iono = d_nav.get_iono(); //notice that the read operation will clear the valid flag
             //std::cout<<"New Galileo IONO model (and UTC) received for SV "<<d_satellite.get_PRN()<<std::endl;
             d_iono_queue->push(iono);
         }
     if (d_nav.have_new_utc_model() == true)
         {
             Galileo_Utc_Model utc_model = d_nav.get_utc_model(); //notice that the read operation will clear the valid flag
             //std::cout<<"New Galileo UTC model received for SV "<<d_satellite.get_PRN()<<std::endl;
             d_utc_model_queue->push(utc_model);
         }

}

galileo_e5a_telemetry_decoder_cc::galileo_e5a_telemetry_decoder_cc(
        Gnss_Satellite satellite,
        long if_freq,
        long fs_in,
        unsigned
        int vector_length,
        boost::shared_ptr<gr::msg_queue> queue,
        bool dump) :
           gr::block("galileo_e5a_telemetry_decoder_cc", gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
	   gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    // initialize internal vars
    d_queue = queue;
    d_dump = dump;
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    LOG(INFO) << "GALILEO E5A TELEMETRY PROCESSING: satellite " << d_satellite;
    d_vector_length = vector_length;
    //d_samples_per_symbol = ( Galileo_E5a_CODE_CHIP_RATE_HZ / Galileo_E5a_CODE_LENGTH_CHIPS ) / Galileo_E1_B_SYMBOL_RATE_BPS;
    d_fs_in = fs_in;

    // set the preamble
    unsigned short int preambles_bits[GALILEO_INAV_PREAMBLE_LENGTH_BITS] = GALILEO_INAV_PREAMBLE;

    //d_symbols_per_preamble = GALILEO_INAV_PREAMBLE_LENGTH_BITS * d_samples_per_symbol;

    memcpy((unsigned short int*)this->d_preambles_bits, (unsigned short int*)preambles_bits, GALILEO_INAV_PREAMBLE_LENGTH_BITS*sizeof(unsigned short int));

    // preamble bits to sampled symbols
    d_preambles_symbols = (signed int*)malloc(sizeof(signed int) * GALILEO_FNAV_SAMPLES_PER_PREAMBLE);
    int n = 0;
    for (int i = 0; i < GALILEO_INAV_PREAMBLE_LENGTH_BITS; i++)
        {
            for (unsigned int j = 0; j < GALILEO_FNAV_SAMPLES_PER_SYMBOL; j++)
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
    //d_stat = 0;
    d_preamble_lock=false;
    d_preamble_index = 0;
    d_preamble_time_seconds = 0;
    d_flag_frame_sync = false;

    d_TOW_at_Preamble = 0;
    d_TOW_at_current_symbol = 0;

    d_CRC_error_counter = 0;
}

galileo_e5a_telemetry_decoder_cc::~galileo_e5a_telemetry_decoder_cc()
{
	delete d_preambles_symbols;
	d_dump_file.close();
}

int galileo_e5a_telemetry_decoder_cc::general_work (int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,	gr_vector_void_star &output_items)
{
    int corr_value = 0;
    int preamble_diff = 0;
    int corr_sign=0;
    bool corr_flag=true;

    Gnss_Synchro **out = (Gnss_Synchro **) &output_items[0];
    d_sample_counter++; //count for the processed samples

    // ########### Output the tracking data to navigation and PVT ##########
    const Gnss_Synchro **in = (const Gnss_Synchro **)  &input_items[0]; //Get the input samples pointer

    d_flag_preamble = false;
    //******* frame sync ******************
    if (d_preamble_lock == false)
        {
    	// d_preamble_lock tells if we have received a valid preamble and we are waiting
    	// for the next one. Doesn't ensure frame sync yet.
    	//******* preamble correlation ********
    	    // check if the preamble starts positive correlated or negative correlated
    	    if (in[0][0].Prompt_I < 0)	// symbols clipping
    	        {
    	    	    corr_sign=d_preambles_symbols[0];
    	        }
    	    else
    	        {
    	    	    corr_sign=-d_preambles_symbols[0];
    	        }
    	    // the preamble is fully correlated only if maintains corr_sign along the whole sequence
    	    for (int i = 1; i < GALILEO_FNAV_SAMPLES_PER_PREAMBLE; i++)
    	        {
    	    	    if (in[0][i].Prompt_I < 0 && d_preambles_symbols[i]+corr_sign != 0)
    	    	        {
    	    	    	    //exit for
    	    	    	    corr_flag=false;
    	    	    	    break;
    	    	        }
    	    	    if (in[0][i].Prompt_I > 0 && d_preambles_symbols[i]+corr_sign == 0)
    	    	        {
    	    	    	    //exit for
    	    	    	    corr_flag=false;
    	    	    	    break;
    	    	        }
    	        }
    	    if (corr_flag==true)
    	        {
    	    	    d_preamble_index = d_sample_counter;//record the preamble sample stamp
    	    	    LOG(INFO) << "Preamble detection for Galileo SAT " << this->d_satellite << std::endl;
    	    	    d_preamble_lock=true;
    	        }
        }
    // else, preamble_lock == true , we are waiting for the next preamble at a specific time
    else if (d_sample_counter == d_preamble_index + GALILEO_FNAV_SAMPLES_PER_PAGE)
	{
	    // only correlate preamble at the right time
	    //******* preamble correlation ********
	    // check if the preamble starts positive correlated or negative correlated
	    if (in[0][0].Prompt_I < 0)	// symbols clipping
		{
		    corr_sign = d_preambles_symbols[0];
		}
	    else
		{
		    corr_sign = -d_preambles_symbols[0];
		}
	    // the preamble is fully correlated only if maintains corr_sign along the whole sequence
	    for (int i = 1; i < GALILEO_FNAV_SAMPLES_PER_PREAMBLE; i++)
		{
		    if (in[0][i].Prompt_I < 0 && d_preambles_symbols[i] + corr_sign != 0)
			{
			    //exit for
			    corr_flag = false;
			    break;
			}
		    if (in[0][i].Prompt_I > 0
			    && d_preambles_symbols[i] + corr_sign == 0)
			{
			    //exit for
			    corr_flag = false;
			    break;
			}
		}
	    if (corr_flag == false) // lost preamble sync
		{
		    d_preamble_lock = false;
		    LOG(INFO)<< "Lost of frame sync SAT " << this->d_satellite << " preamble_diff= " << preamble_diff;
		    d_flag_frame_sync = false;
		    flag_TOW_set = false;
		}
	    else // NEW PAGE RECEIVED
		{
		    // 1 - Obtain message symbols averaging samples (20 samples / symbol)
		    int frame_length_symbols = GALILEO_FNAV_SYMBOLS_PER_PAGE
			    - GALILEO_FNAV_PREAMBLE_LENGTH_BITS;
		    //int page_symbols[frame_length_symbols];
		    double samples_to_bit_accumulator[frame_length_symbols];
		    //int samples_to_bit_accumulator = 0;
		    for (int i = 0; i < frame_length_symbols; i++)
			{
			    samples_to_bit_accumulator[i]=0.0;
			    for (int k = 0; k < GALILEO_FNAV_SAMPLES_PER_SYMBOL; k++)
				{
				    samples_to_bit_accumulator[i] += in[0][k + i*GALILEO_FNAV_SAMPLES_PER_SYMBOL + GALILEO_FNAV_SAMPLES_PER_PREAMBLE].Prompt_I;
				    // Reminder: corr_sign is negative if phase lock is at 180º
//				    if (in[0][k + i*GALILEO_FNAV_SAMPLES_PER_SYMBOL + GALILEO_FNAV_SAMPLES_PER_PREAMBLE].Prompt_I
//				              > 0)
//					{
//					    samples_to_bit_accumulator += corr_sign;
//					}
//				    else
//					{
//					    samples_to_bit_accumulator -= corr_sign;
//					}

				}
			    samples_to_bit_accumulator[i] /= corr_sign*GALILEO_FNAV_SAMPLES_PER_SYMBOL;
//			    if (samples_to_bit_accumulator > 0)
//				{
//				    page_symbols[i] = 1; // because last symbol of the preamble is just received now!
//
//				}
//			    else
//				{
//				    page_symbols[i] = -1; // because last symbol of the preamble is just received now!
//				}
			}
		    // DECODE WORD
		    decode_word(samples_to_bit_accumulator, frame_length_symbols);
		    // CHECK CRC
                    if (d_nav.flag_CRC_test == true)
                        {
                            d_CRC_error_counter = 0;
                            d_flag_preamble = true; //valid preamble indicator (initialized to false every work())
                            d_preamble_index = d_sample_counter;  //record the preamble sample stamp (t_P)
                            d_preamble_time_seconds = in[0][0].Tracking_timestamp_secs; // - d_preamble_duration_seconds; //record the PRN start sample index associated to the preamble
                            if (!d_flag_frame_sync)
                                {
                                    d_flag_frame_sync = true;
                                    LOG(INFO) <<" Frame sync SAT " << this->d_satellite << " with preamble start at " << d_preamble_time_seconds << " [s]";
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
                                    d_preamble_lock = false;
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
        //We expect a preamble each 10 seconds (FNAV page period)
        {
            Prn_timestamp_at_preamble_ms = in[0][0].Tracking_timestamp_secs * 1000.0;
            if (d_nav.flag_TOW_1 == true)
        	{
        	    d_TOW_at_Preamble = d_nav.FNAV_TOW_1;
        	    d_TOW_at_current_symbol = d_TOW_at_Preamble;
        	    d_nav.flag_TOW_1 = false;
        	}
            if (d_nav.flag_TOW_2 == true)
        	{
        	    d_TOW_at_Preamble = d_nav.FNAV_TOW_2;
        	    d_TOW_at_current_symbol = d_TOW_at_Preamble;
        	    d_nav.flag_TOW_2 = false;
        	}
            if (d_nav.flag_TOW_3 == true)
        	{
        	    d_TOW_at_Preamble = d_nav.FNAV_TOW_3;
        	    d_TOW_at_current_symbol = d_TOW_at_Preamble;
        	    d_nav.flag_TOW_3 = false;
        	}
            if (d_nav.flag_TOW_4 == true)
        	{
        	    d_TOW_at_Preamble = d_nav.FNAV_TOW_4;
        	    d_TOW_at_current_symbol = d_TOW_at_Preamble;
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

void galileo_e5a_telemetry_decoder_cc::set_ephemeris_queue(concurrent_queue<Galileo_Ephemeris> *ephemeris_queue)
{
    d_ephemeris_queue = ephemeris_queue;
}


void galileo_e5a_telemetry_decoder_cc::set_iono_queue(concurrent_queue<Galileo_Iono> *iono_queue)
{
    d_iono_queue = iono_queue;
}


void galileo_e5a_telemetry_decoder_cc::set_almanac_queue(concurrent_queue<Galileo_Almanac> *almanac_queue)
{
    d_almanac_queue = almanac_queue;
}


void galileo_e5a_telemetry_decoder_cc::set_utc_model_queue(concurrent_queue<Galileo_Utc_Model> *utc_model_queue)
{
    d_utc_model_queue = utc_model_queue;
}
