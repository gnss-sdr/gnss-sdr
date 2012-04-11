/*!
 * \file gps_l1_ca_observables_cc.cc
 * \brief Implementation of the pseudorange computation block for GPS L1 C/A
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
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

#include "gps_l1_ca_observables_cc.h"
#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>
#include <bitset>
#include <cmath>
#include "math.h"
#include "control_message_factory.h"
#include <gnuradio/gr_io_signature.h>
#include <glog/log_severity.h>
#include <glog/logging.h>

#include "gnss_synchro.h"


using google::LogMessage;



gps_l1_ca_observables_cc_sptr
gps_l1_ca_make_observables_cc(unsigned int nchannels, gr_msg_queue_sptr queue, bool dump, std::string dump_filename, int output_rate_ms, bool flag_averaging)
{

    return gps_l1_ca_observables_cc_sptr(new gps_l1_ca_observables_cc(nchannels, queue, dump, dump_filename, output_rate_ms, flag_averaging));
}


gps_l1_ca_observables_cc::gps_l1_ca_observables_cc(unsigned int nchannels, gr_msg_queue_sptr queue, bool dump, std::string dump_filename, int output_rate_ms, bool flag_averaging) :
		                gr_block ("gps_l1_ca_observables_cc", gr_make_io_signature (nchannels, nchannels,  sizeof(Gnss_Synchro)),
		                        gr_make_io_signature(nchannels, nchannels, sizeof(Gnss_Synchro)))
{
    // initialize internal vars
    d_queue = queue;
    d_dump = dump;
    d_nchannels = nchannels;
    d_output_rate_ms = output_rate_ms;
    d_history_gnss_synchro_deque = new std::deque<Gnss_Synchro>[d_nchannels];
    d_dump_filename = dump_filename;
    d_flag_averaging = flag_averaging;

    // ############# ENABLE DATA FILE LOG #################
    if (d_dump == true)
        {
            if (d_dump_file.is_open() == false)
                {
                    try
                    {
                            d_dump_file.exceptions (std::ifstream::failbit | std::ifstream::badbit );
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            std::cout << "Observables dump enabled Log file: " << d_dump_filename.c_str() << std::endl;
                    }
                    catch (std::ifstream::failure e) {
                            std::cout << "Exception opening observables dump file " << e.what() << std::endl;
                    }
                }
        }
}

gps_l1_ca_observables_cc::~gps_l1_ca_observables_cc()
{
    d_dump_file.close();
    delete[] d_history_gnss_synchro_deque;
}


bool pairCompare_gnss_synchro_Prn_delay_ms( std::pair<int,Gnss_Synchro> a, std::pair<int,Gnss_Synchro> b)
{
    return (a.second.Prn_timestamp_ms) < (b.second.Prn_timestamp_ms);
}

bool pairCompare_gnss_synchro_preamble_symbol_count( std::pair<int,Gnss_Synchro> a, std::pair<int,Gnss_Synchro> b)
{
    return (a.second.Preamble_symbol_counter) < (b.second.Preamble_symbol_counter);
}

bool pairCompare_gnss_synchro_preamble_delay_ms( std::pair<int,Gnss_Synchro> a, std::pair<int,Gnss_Synchro> b)
{
    return (a.second.Preamble_timestamp_ms) < (b.second.Preamble_timestamp_ms);
}


bool pairCompare_double( std::pair<int,double> a, std::pair<int,double> b)
{
    return (a.second) < (b.second);
}


void clearQueue( std::deque<double> &q )
{
    std::deque<double> empty;
    std::swap(q, empty);
}


int gps_l1_ca_observables_cc::general_work (int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,	gr_vector_void_star &output_items) {

    Gnss_Synchro **in = (Gnss_Synchro **)  &input_items[0]; //Get the input pointer
    Gnss_Synchro **out = (Gnss_Synchro **)  &output_items[0]; //Get the output pointer

    Gnss_Synchro current_gnss_synchro[d_nchannels];

    std::map<int,Gnss_Synchro> current_gnss_synchro_map;
    std::map<int,Gnss_Synchro> gnss_synchro_aligned_map;

    std::map<int,Gnss_Synchro>::iterator gnss_synchro_iter;


    double traveltime_ms;
    double pseudorange_m;

    d_sample_counter++; //count for the processed samples

    bool flag_history_ok = true; //flag to indicate that all the queues have filled their GNSS SYNCHRO history
    /*
     * 1. Read the GNSS SYNCHRO objects from available channels
     */
    for (unsigned int i=0; i<d_nchannels ; i++)
        {
        //Copy the telemetry decoder data to local copy
    	current_gnss_synchro[i]=in[i][0];

            if (current_gnss_synchro[i].Flag_valid_word) //if this channel have valid word
                {
            	    current_gnss_synchro_map.insert(std::pair<int,Gnss_Synchro>(current_gnss_synchro[i].Channel_ID, current_gnss_synchro[i])); //record the word structure in a map for pseudoranges
                    // RECORD PRN start timestamps history
                    if (d_history_gnss_synchro_deque[i].size()<MAX_TOA_DELAY_MS)
                        {
                    	d_history_gnss_synchro_deque[i].push_front(current_gnss_synchro[i]);
                            flag_history_ok = false; // at least one channel need more samples
                        }
                    else
                        {
                                //clearQueue(d_history_prn_delay_ms[i]); //clear the queue as the preamble arrives
							d_history_gnss_synchro_deque[i].pop_back();
							d_history_gnss_synchro_deque[i].push_front(current_gnss_synchro[i]);
                        }
                }
        }

    /*
     * 1.2 Assume no satellites in tracking
     */
    for (unsigned int i=0; i<d_nchannels ; i++)
        {
			current_gnss_synchro[i].Flag_valid_pseudorange = false;
			current_gnss_synchro[i].Pseudorange_m = 0.0;
			current_gnss_synchro[i].Pseudorange_symbol_shift = 0.0;
        }
    /*
     * 2. Compute RAW pseudorranges: Use only the valid channels (channels that are tracking a satellite)
     */
    if(current_gnss_synchro_map.size() > 0 and flag_history_ok == true)
        {

        /*
         *  2.1 Find the correct symbol timestamp in the gnss_synchro history: we have to compare timestamps between channels on the SAME symbol
		 *  (common TX time algorithm)
		 */

	    double min_preamble_delay_ms;
	    double max_preamble_delay_ms;
	    int current_symbol=0;
	    int reference_channel;
	    int history_shift;
	    Gnss_Synchro tmp_gnss_synchro;

	    gnss_synchro_iter = min_element(current_gnss_synchro_map.begin(), current_gnss_synchro_map.end(), pairCompare_gnss_synchro_preamble_delay_ms);
	    min_preamble_delay_ms = gnss_synchro_iter->second.Preamble_timestamp_ms; //[ms]

	    gnss_synchro_iter = max_element(current_gnss_synchro_map.begin(), current_gnss_synchro_map.end(), pairCompare_gnss_synchro_preamble_delay_ms);
	    max_preamble_delay_ms = gnss_synchro_iter->second.Preamble_timestamp_ms; //[ms]

	    if ((max_preamble_delay_ms-min_preamble_delay_ms)< MAX_TOA_DELAY_MS)
	    {
	    	// we have a valid information set. Its time to align the symbols information
	    	// what is the most delayed symbol in the current set? -> this will be the reference symbol
	    	gnss_synchro_iter=min_element(current_gnss_synchro_map.begin(), current_gnss_synchro_map.end(), pairCompare_gnss_synchro_preamble_symbol_count);
	    	current_symbol=gnss_synchro_iter->second.Preamble_symbol_counter;
	    	reference_channel=gnss_synchro_iter->second.Channel_ID;
	    	// save it in the aligned symbols map
	    	gnss_synchro_aligned_map.insert(std::pair<int,Gnss_Synchro>(gnss_synchro_iter->second.Channel_ID, gnss_synchro_iter->second));

	    	// Now find where the same symbols were in the rest of the channels searching in the symbol history
			for(gnss_synchro_iter = current_gnss_synchro_map.begin(); gnss_synchro_iter != current_gnss_synchro_map.end(); gnss_synchro_iter++)
			{
				//TODO: Replace the loop using current current_symbol-Preamble_symbol_counter
				if (reference_channel!=gnss_synchro_iter->second.Channel_ID)
				{
					// compute the required symbol history shift in order to match the reference symbol
					history_shift=gnss_synchro_iter->second.Preamble_symbol_counter-current_symbol;
					if (history_shift<(int)MAX_TOA_DELAY_MS)// and history_shift>=0)
					{
						tmp_gnss_synchro= d_history_gnss_synchro_deque[gnss_synchro_iter->second.Channel_ID][history_shift];
						gnss_synchro_aligned_map.insert(std::pair<int,Gnss_Synchro>(gnss_synchro_iter->second.Channel_ID,tmp_gnss_synchro));
					}
				}
			}
	    }

	    /*
	     * 3 Compute the pseudorranges using the aligned data map
	     */
	    double min_symbol_timestamp_ms;
	    double max_symbol_timestamp_ms;
	    gnss_synchro_iter = min_element(gnss_synchro_aligned_map.begin(), gnss_synchro_aligned_map.end(), pairCompare_gnss_synchro_Prn_delay_ms);
	    min_symbol_timestamp_ms = gnss_synchro_iter->second.Prn_timestamp_ms; //[ms]

	    gnss_synchro_iter = max_element(gnss_synchro_aligned_map.begin(), gnss_synchro_aligned_map.end(), pairCompare_gnss_synchro_Prn_delay_ms);
	    max_symbol_timestamp_ms = gnss_synchro_iter->second.Prn_timestamp_ms; //[ms]


		// check again if this is a valid set of observations
		if ((max_symbol_timestamp_ms - min_symbol_timestamp_ms) < MAX_TOA_DELAY_MS)
			/*
			 * 2.3 compute the pseudoranges
			 */
			{
				for(gnss_synchro_iter = gnss_synchro_aligned_map.begin(); gnss_synchro_iter != gnss_synchro_aligned_map.end(); gnss_synchro_iter++)
					{
					traveltime_ms = gnss_synchro_iter->second.Prn_timestamp_ms - min_symbol_timestamp_ms + GPS_STARTOFFSET_ms; //[ms]
					pseudorange_m = traveltime_ms*GPS_C_m_ms; // [m]
					// update the pseudorange object
					current_gnss_synchro[gnss_synchro_iter->second.Channel_ID]=gnss_synchro_iter->second;
					current_gnss_synchro[gnss_synchro_iter->second.Channel_ID].Pseudorange_m = pseudorange_m;
					current_gnss_synchro[gnss_synchro_iter->second.Channel_ID].Pseudorange_symbol_shift = (double)current_symbol; // number of symbols shifted from preamble start symbol
					current_gnss_synchro[gnss_synchro_iter->second.Channel_ID].Flag_valid_pseudorange = true;
					current_gnss_synchro[gnss_synchro_iter->second.Channel_ID].Pseudorange_timestamp_ms=max_symbol_timestamp_ms;
					}
			}
        }


    if(d_dump == true)
        {
            // MULTIPLEXED FILE RECORDING - Record results to file
            try
            {
                    double tmp_double;
                    for (unsigned int i=0; i<d_nchannels ; i++)
                        {
                            tmp_double = current_gnss_synchro[i].Preamble_timestamp_ms;
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            tmp_double = current_gnss_synchro[i].Prn_timestamp_ms;
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            tmp_double = current_gnss_synchro[i].Pseudorange_m;
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            tmp_double = current_gnss_synchro[i].Pseudorange_symbol_shift;
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            tmp_double = current_gnss_synchro[i].PRN;
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                        }
            }
            catch (std::ifstream::failure e)
            {
                    std::cout << "Exception writing observables dump file " << e.what() << std::endl;
            }
        }

    consume_each(1); //one by one
    // mod 8/4/2012: always make the observables output
    //if ((d_sample_counter % d_output_rate_ms) == 0)
    //    {
			for (unsigned int i=0; i<d_nchannels ; i++)
				{
					*out[i] = current_gnss_synchro[i];
				}
            return 1; //Output the observables
    //    }
    //else
    //    {
    //        return 0; //hold on
    //    }
}


