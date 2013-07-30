/*!
 * \file galileo_e1b_telemetry_decoder_cc.cc
 * \brief Implementation of a Galileo INAV message demodulator block
 * \author Mara Branzanti 2013. mara.branzanti(at)gmail.com
 * \author Javier Arribas 2013. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
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

#include "gnss_synchro.h"
#include "galileo_e1b_telemetry_decoder_cc.h"
#include <iostream>
#include <sstream>
#include <bitset>
#include <gnuradio/io_signature.h>
#include <glog/log_severity.h>
#include <glog/logging.h>
#include <boost/lexical_cast.hpp>
#include "control_message_factory.h"

#include "gnss_synchro.h"

#include "convolutional.h"

using google::LogMessage;


galileo_e1b_telemetry_decoder_cc_sptr
galileo_e1b_make_telemetry_decoder_cc(Gnss_Satellite satellite, long if_freq, long fs_in, unsigned
        int vector_length, boost::shared_ptr<gr::msg_queue> queue, bool dump)
{
    return galileo_e1b_telemetry_decoder_cc_sptr(new galileo_e1b_telemetry_decoder_cc(satellite, if_freq,
            fs_in, vector_length, queue, dump));
}



void galileo_e1b_telemetry_decoder_cc::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
    for (unsigned i = 0; i < 3; i++)
        {
            ninput_items_required[i] = GALILEO_INAV_PAGE_SYMBOLS; //set the required sample history
        }
}


void galileo_e1b_telemetry_decoder_cc::viterbi_decoder(double *page_part_symbols, int *page_part_bits)
{

	int CodeLength=240;
	int DataLength;
	int	nn, KK, mm, max_states;
	int g_encoder[2];

	nn = 2; //Coding rate 1/n
	KK = 7; //Constraint Length
	g_encoder[0]=121; // Polynomial G1
	g_encoder[1]=91; // Polinomial G2

	mm = KK - 1;
	max_states = 1 << mm;			/* 2^mm */
	DataLength = (CodeLength/nn)-mm;

	/* create appropriate transition matrices */
	int     *out0, *out1, *state0, *state1;
	out0 = (int*)calloc( max_states, sizeof(int) );
	out1 = (int*)calloc( max_states, sizeof(int) );
	state0 = (int*)calloc( max_states, sizeof(int) );
	state1 = (int*)calloc( max_states, sizeof(int) );

	nsc_transit( out0, state0, 0, g_encoder, KK, nn );
	nsc_transit( out1, state1, 1, g_encoder, KK, nn );

	Viterbi( page_part_bits, out0, state0, out1, state1,
			page_part_symbols, KK, nn, DataLength );


	/* Clean up memory */
	free( out0 );
	free( out1 );
	free( state0 );
	free( state1 );

}


void galileo_e1b_telemetry_decoder_cc::deinterleaver(int rows, int cols, double *in, double *out)
{
	for (int r=0;r<rows;r++)
	{
		for(int c=0;c<cols;c++)
		{
			out[c*rows+r]=in[r*cols+c];
		}
	}
}


galileo_e1b_telemetry_decoder_cc::galileo_e1b_telemetry_decoder_cc(
        Gnss_Satellite satellite,
        long if_freq,
        long fs_in,
        unsigned
        int vector_length,
        boost::shared_ptr<gr::msg_queue> queue,
        bool dump) :
        gr::block("galileo_e1b_telemetry_decoder_cc", gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
        gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    // initialize internal vars
    d_queue = queue;
    d_dump = dump;
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << "GALILEO E1B TELEMETRY PROCESSING: satellite " << d_satellite;
    d_vector_length = vector_length;
    d_samples_per_symbol = ( Galileo_E1_CODE_CHIP_RATE_HZ / Galileo_E1_B_CODE_LENGTH_CHIPS ) / Galileo_E1_B_SYMBOL_RATE_BPS;
    d_fs_in = fs_in;

    // set the preamble
    unsigned short int preambles_bits[GALILEO_INAV_PREAMBLE_LENGTH_BITS]=GALILEO_INAV_PREAMBLE;

    d_symbols_per_preamble=GALILEO_INAV_PREAMBLE_LENGTH_BITS * d_samples_per_symbol;

    memcpy((unsigned short int*)this->d_preambles_bits, (unsigned short int*)preambles_bits, GALILEO_INAV_PREAMBLE_LENGTH_BITS*sizeof(unsigned short int));

    // preamble bits to sampled symbols
    d_preambles_symbols = (signed int*)malloc(sizeof(signed int) * d_symbols_per_preamble);
    int n = 0;
    for (int i=0; i<GALILEO_INAV_PREAMBLE_LENGTH_BITS; i++)
        {
            for (unsigned int j=0; j<d_samples_per_symbol; j++)
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
    flag_TOW_set = false;

}


galileo_e1b_telemetry_decoder_cc::~galileo_e1b_telemetry_decoder_cc()
{
    delete d_preambles_symbols;
    d_dump_file.close();
}


int galileo_e1b_telemetry_decoder_cc::general_work (int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,	gr_vector_void_star &output_items)
{
    int corr_value = 0;
    int preamble_diff = 0;

    Gnss_Synchro **out = (Gnss_Synchro **) &output_items[0];
    d_sample_counter++; //count for the processed samples

    // ########### Output the tracking data to navigation and PVT ##########
    const Gnss_Synchro **in = (const Gnss_Synchro **)  &input_items[0]; //Get the input samples pointer

    // TODO Optimize me!
    //******* preamble correlation ********
    for (int i=0; i<d_symbols_per_preamble; i++)
        {
            if (in[0][i].Prompt_I < 0)	// symbols clipping
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
    if (abs(corr_value) >= d_symbols_per_preamble)
        {
    		//std::cout << "Positive preamble correlation for Galileo SAT " << this->d_satellite << std::endl;
            if (d_stat == 0)
                {
                    d_preamble_index = d_sample_counter;//record the preamble sample stamp
                    std::cout << "Preamble detection for Galileo SAT " << this->d_satellite << std::endl;
                    d_stat = 1; // enter into frame pre-detection status
                }
            else if (d_stat == 1) //check preamble separation
                {
                    preamble_diff = abs(d_sample_counter - d_preamble_index);
                    //std::cout << "preamble_diff="<< preamble_diff <<" for Galileo SAT " << this->d_satellite << std::endl;
                    if (abs(preamble_diff - GALILEO_INAV_PREAMBLE_PERIOD_SYMBOLS) < 1)
                        {

                    		std::cout<<"d_sample_counter="<<d_sample_counter<<std::endl;
                    		std::cout<<"corr_value="<<corr_value<<std::endl;
                    		// NEW Galileo page part is received
                    	    // 0. fetch the symbols into an array
                    	    int frame_length=GALILEO_INAV_PAGE_PART_SYMBOLS-d_symbols_per_preamble;
                    	    double page_part_symbols[frame_length];
                    	    double page_part_symbols_deint[frame_length];

                    	    for (int i=0;i<frame_length;i++)
                    	    {
                    	    	if (corr_value>0)
                    	    	{
                    	    		page_part_symbols[i]=in[0][i+d_symbols_per_preamble].Prompt_I; // because last symbol of the preamble is just received now!

                    	    	}else{
                    	    		page_part_symbols[i]=-in[0][i+d_symbols_per_preamble].Prompt_I; // because last symbol of the preamble is just received now!
                    	    	}
                    	    }
                    		// 1. De-interleave
                    	    deinterleaver(GALILEO_INAV_INTERLEAVER_ROWS,GALILEO_INAV_INTERLEAVER_COLS,page_part_symbols, page_part_symbols_deint);

                    		// 2. Viterbi decoder
                    	    // 2.1 Take into account the NOT gate in G2 polynomial (Galileo ICD Figure 13, FEC encoder)
                    	    // 2.2 Take into account the possible inversion of the polarity due to PLL lock at 180º
                    	    for (int i=0;i<frame_length;i++)
                    	    {
                    	    	if ((i+1)%2==0)
                    	    	{
                    	    		page_part_symbols_deint[i]=-page_part_symbols_deint[i];
                    	    	}
                    	    }

                    	    int page_part_bits[frame_length/2];
                    	    viterbi_decoder(page_part_symbols_deint, page_part_bits);

                    		// 3. Call the Galileo page decoder

                     	    std::string page_String;

                      	    //std::cout<<"frame_bits=[";
                     	    for(int i=0; i < (frame_length/2); i++)
                     	    {
                     	    	if (page_part_bits[i]>0)
                     	    	{
                     	    		page_String.push_back('1');
                    	    		//std::cout<<",1";
                     	    	}else{
                     	    		page_String.push_back('0');
                    	    		//std::cout<<",0";
                     	    	}

                     	    	//sprintf(&page_String[i], "%d", page_part_bits[i]); // this produces a memory core dumped...
                     	    }
                    	    //std::cout<<"]"<<std::endl;

                     	    Galileo_Navigation_Message decode_page;

                     	    std::cout<<"page_string="<<page_String<<std::endl; //correctly transformed to char


             	             if (page_part_bits[0]==1)
                              {
                             	 std::cout<<"Page Odd"<<std::endl;
                             	 decode_page.split_page(page_String.c_str(), flag_even_word_arrived);
                             	 flag_even_word_arrived=0;
                             	 std::cout<<"Page type ="<< page_part_bits[1]<<std::endl;
                               }
                              else
                              {
                             	 std::cout<<"Page Even"<<std::endl;
                             	 decode_page.split_page(page_String.c_str(), flag_even_word_arrived);
                             	 flag_even_word_arrived=1;
                             	 std::cout<<"Page type ="<< page_part_bits[1]<<std::endl;
                              }



                            d_flag_preamble = true;
                            d_preamble_index = d_sample_counter;  //record the preamble sample stamp (t_P)
                            d_preamble_time_seconds = in[0][0].Tracking_timestamp_secs;// - d_preamble_duration_seconds; //record the PRN start sample index associated to the preamble

                            if (!d_flag_frame_sync)
                                {
                                    d_flag_frame_sync = true;
                                    std::cout <<" Frame sync SAT " << this->d_satellite << " with preamble start at " << d_preamble_time_seconds << " [s]" << std::endl;
                                }
                        }
                }
        }
    else
        {
            if (d_stat == 1)
                {
                    preamble_diff = d_sample_counter - d_preamble_index;
                    if (preamble_diff > GALILEO_INAV_PREAMBLE_PERIOD_SYMBOLS)
                        {
                            std::cout << "Lost of frame sync SAT " << this->d_satellite << " preamble_diff= " << preamble_diff << std::endl;
                            d_stat = 0; //lost of frame sync
                            d_flag_frame_sync = false;
                            //flag_TOW_set=false;
                        }
                }
        }

    consume_each(1); //one by one
    // UPDATE GNSS SYNCHRO DATA
    Gnss_Synchro current_synchro_data; //structure to save the synchronization information and send the output object to the next block
    //1. Copy the current tracking output
    current_synchro_data = in[0][0];
    //2. Add the telemetry decoder information
    //if (this->d_flag_preamble==true and d_GPS_FSM.d_nav.d_TOW>0) //update TOW at the preamble instant (todo: check for valid d_TOW)
    if (this->d_flag_preamble==true) //update TOW at the preamble instant (todo: check for valid d_TOW)
        {
            //d_TOW_at_Preamble = d_GPS_FSM.d_nav.d_TOW + GPS_SUBFRAME_SECONDS; //we decoded the current TOW when the last word of the subframe arrive, so, we have a lag of ONE SUBFRAME
            //d_TOW_at_current_symbol = d_TOW_at_Preamble + GALILEO_INAV_PREAMBLE_LENGTH_BITS/Galileo_E1_B_SYMBOL_RATE_BPS;
            Prn_timestamp_at_preamble_ms = in[0][0].Tracking_timestamp_secs * 1000.0;
            ///if (flag_TOW_set==false)
                //{
                //    flag_TOW_set = true;
                //}
        }
    else
        {
            //d_TOW_at_current_symbol = d_TOW_at_current_symbol + Galileo_E1_CODE_PERIOD;
        }


    current_synchro_data.d_TOW = d_TOW_at_Preamble;
    current_synchro_data.d_TOW_at_current_symbol = d_TOW_at_current_symbol;
    current_synchro_data.Flag_valid_word = (d_flag_frame_sync == true and d_flag_parity == true and flag_TOW_set==true);
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
            catch (std::ifstream::failure e)
            {
                    std::cout << "Exception writing observables dump file " << e.what() << std::endl;
            }
        }
    //3. Make the output (copy the object contents to the GNURadio reserved memory)
    *out[0] = current_synchro_data;
    return 1;
}


void galileo_e1b_telemetry_decoder_cc::set_satellite(Gnss_Satellite satellite)
{
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << "Setting decoder Finite State Machine to satellite "  << d_satellite;
    //d_GPS_FSM.i_satellite_PRN = d_satellite.get_PRN();
    DLOG(INFO) << "Navigation Satellite set to " << d_satellite;
}


void galileo_e1b_telemetry_decoder_cc::set_channel(int channel)
{
    d_channel = channel;
    //d_GPS_FSM.i_channel_ID = channel;
    DLOG(INFO) << "Navigation channel set to " << channel;
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
                            std::cout << "Telemetry decoder dump enabled on channel " << d_channel << " Log file: " << d_dump_filename.c_str() << std::endl;
                    }
                    catch (std::ifstream::failure e)
                    {
                            std::cout << "channel " << d_channel << " Exception opening trk dump file " << e.what() << std::endl;
                    }
                }
        }
}

void galileo_e1b_telemetry_decoder_cc::set_ephemeris_queue(concurrent_queue<Galileo_Ephemeris> *ephemeris_queue)
{
	//d_Galileo_INAV_FSM.d_ephemeris_queue = ephemeris_queue;
}
void galileo_e1b_telemetry_decoder_cc::set_iono_queue(concurrent_queue<Galileo_Iono> *iono_queue)
{
	//d_Galileo_INAV_FSM.d_iono_queue = iono_queue;
}
void galileo_e1b_telemetry_decoder_cc::set_almanac_queue(concurrent_queue<Galileo_Almanac> *almanac_queue)
{
	//d_Galileo_INAV_FSM.d_almanac_queue = almanac_queue;
}
void galileo_e1b_telemetry_decoder_cc::set_utc_model_queue(concurrent_queue<Galileo_Utc_Model> *utc_model_queue)
{
	//d_Galileo_INAV_FSM.d_utc_model_queue = utc_model_queue;
}


