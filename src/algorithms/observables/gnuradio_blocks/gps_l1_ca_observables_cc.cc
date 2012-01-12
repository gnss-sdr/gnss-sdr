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

using google::LogMessage;



gps_l1_ca_observables_cc_sptr
gps_l1_ca_make_observables_cc(unsigned int nchannels, gr_msg_queue_sptr queue, bool dump, std::string dump_filename, int output_rate_ms, bool flag_averaging)
{

    return gps_l1_ca_observables_cc_sptr(new gps_l1_ca_observables_cc(nchannels, queue, dump, dump_filename, output_rate_ms, flag_averaging));
}


gps_l1_ca_observables_cc::gps_l1_ca_observables_cc(unsigned int nchannels, gr_msg_queue_sptr queue, bool dump, std::string dump_filename, int output_rate_ms, bool flag_averaging) :
		                gr_block ("gps_l1_ca_observables_cc", gr_make_io_signature (nchannels, nchannels,  sizeof(gnss_synchro)),
		                        gr_make_io_signature(nchannels, nchannels, sizeof(gnss_pseudorange)))
{
    // initialize internal vars
    d_queue = queue;
    d_dump = dump;
    d_nchannels = nchannels;
    d_output_rate_ms = output_rate_ms;
    d_history_prn_delay_ms = new std::deque<double>[d_nchannels];
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
    delete[] d_history_prn_delay_ms;
}





bool pairCompare_gnss_synchro( std::pair<int,gnss_synchro> a, std::pair<int,gnss_synchro> b)
{
    return (a.second.preamble_delay_ms) < (b.second.preamble_delay_ms);
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

    gnss_synchro **in = (gnss_synchro **)  &input_items[0]; //Get the input pointer
    gnss_pseudorange **out = (gnss_pseudorange **)  &output_items[0]; //Get the output pointer

    gnss_pseudorange current_gnss_pseudorange;

    std::map<int,gnss_synchro> gps_words;
    std::map<int,gnss_synchro>::iterator gps_words_iter;
    std::map<int,double>::iterator current_prn_timestamps_ms_iter;
    std::map<int,double> current_prn_timestamps_ms;

    double min_preamble_delay_ms;
    double max_preamble_delay_ms;
    double pseudoranges_timestamp_ms;
    double traveltime_ms;
    double pseudorange_m;
    double delta_timestamp_ms;
    double min_delta_timestamp_ms;
    double actual_min_prn_delay_ms;
    double current_prn_delay_ms;

    int history_shift = 0;
    int pseudoranges_reference_sat_ID = 0;
    unsigned int pseudoranges_reference_sat_channel_ID = 0;

    d_sample_counter++; //count for the processed samples

    bool flag_history_ok = true; //flag to indicate that all the queues have filled their timestamp history
    /*
     * 1. Read the GNSS SYNCHRO objects from available channels to obtain the preamble timestamp, current PRN start time and accumulated carrier phase
     */
    for (unsigned int i=0; i<d_nchannels ; i++)
        {
            if (in[i][0].valid_word) //if this channel have valid word
                {
                    gps_words.insert(std::pair<int,gnss_synchro>(in[i][0].channel_ID, in[i][0])); //record the word structure in a map for pseudoranges
                    // RECORD PRN start timestamps history
                    if (d_history_prn_delay_ms[i].size()<MAX_TOA_DELAY_MS)
                        {
                            d_history_prn_delay_ms[i].push_front(in[i][0].prn_delay_ms);
                            flag_history_ok = false; // at least one channel need more samples
                        }
                    else
                        {
                                //clearQueue(d_history_prn_delay_ms[i]); //clear the queue as the preamble arrives
                                d_history_prn_delay_ms[i].pop_back();
                                d_history_prn_delay_ms[i].push_front(in[i][0].prn_delay_ms);
                        }
                }
        }

    /*
     * 1.2 Assume no satellites in tracking
     */
    for (unsigned int i=0; i<d_nchannels ; i++)
        {
            current_gnss_pseudorange.valid = false;
            current_gnss_pseudorange.SV_ID = 0;
            current_gnss_pseudorange.pseudorange_m = 0;
            current_gnss_pseudorange.timestamp_ms = 0;
            *out[i] = current_gnss_pseudorange;
        }
    /*
     * 2. Compute RAW pseudorranges: Use only the valid channels (channels that are tracking a satellite)
     */
    if(gps_words.size() > 0 and flag_history_ok == true)
        {
            /*
             *  2.1 find the minimum preamble timestamp (nearest satellite, will be the reference)
             */
            // The nearest satellite, first preamble to arrive
            gps_words_iter = min_element(gps_words.begin(), gps_words.end(), pairCompare_gnss_synchro);
            min_preamble_delay_ms = gps_words_iter->second.preamble_delay_ms; //[ms]

            pseudoranges_reference_sat_ID = gps_words_iter->second.satellite_PRN; // it is the reference!
            pseudoranges_reference_sat_channel_ID = gps_words_iter->second.channel_ID;

            // The farthest satellite, last preamble to arrive
            gps_words_iter = max_element(gps_words.begin(), gps_words.end(), pairCompare_gnss_synchro);
            max_preamble_delay_ms = gps_words_iter->second.preamble_delay_ms;
            min_delta_timestamp_ms = gps_words_iter->second.prn_delay_ms - max_preamble_delay_ms; //[ms]

            // check if this is a valid set of observations
            if ((max_preamble_delay_ms - min_preamble_delay_ms) < MAX_TOA_DELAY_MS)
                {
                    // Now we have to determine were we are in time, compared with the last preamble! -> we select the corresponding history
                    /*!
                     * \todo Explain this better!
                     */
                    //bool flag_preamble_navigation_now=true;
                    // find again the minimum CURRENT minimum preamble time, taking into account the preamble timeshift
                    for(gps_words_iter = gps_words.begin(); gps_words_iter != gps_words.end(); gps_words_iter++)
                        {
                            delta_timestamp_ms = (gps_words_iter->second.prn_delay_ms - gps_words_iter->second.preamble_delay_ms) - min_delta_timestamp_ms;
                            history_shift = round(delta_timestamp_ms);
                            //std::cout<<"history_shift="<<history_shift<<"\r\n";
                            current_prn_timestamps_ms.insert(std::pair<int,double>(gps_words_iter->second.channel_ID, d_history_prn_delay_ms[gps_words_iter->second.channel_ID][history_shift]));
                            // debug: preamble position test
                            //if ((d_history_prn_delay_ms[gps_words_iter->second.channel_ID][history_shift]-gps_words_iter->second.preamble_delay_ms)<0.1)
                            //{std::cout<<"ch "<<gps_words_iter->second.channel_ID<<" current_prn_time-last_preamble_prn_time="<<
                            //  d_history_prn_delay_ms[gps_words_iter->second.channel_ID][history_shift]-gps_words_iter->second.preamble_delay_ms<<"\r\n";
                            //}else{
                            //  flag_preamble_navigation_now=false;
                            //}
                        }

                    //if (flag_preamble_navigation_now==true)
                    //{
                    //std::cout<<"PREAMBLE NAVIGATION NOW!\r\n";
                    //d_sample_counter=0;
                    //}
                    current_prn_timestamps_ms_iter = min_element(current_prn_timestamps_ms.begin(), current_prn_timestamps_ms.end(), pairCompare_double);

                    actual_min_prn_delay_ms = current_prn_timestamps_ms_iter->second;

                    pseudoranges_timestamp_ms = actual_min_prn_delay_ms; //save the shortest pseudorange timestamp to compute the current GNSS timestamp
                    /*
                     * 2.2 compute the pseudoranges
                     */

                    for(gps_words_iter = gps_words.begin(); gps_words_iter != gps_words.end(); gps_words_iter++)
                        {
                            // #### compute the pseudorange for this satellite ###

                            current_prn_delay_ms = current_prn_timestamps_ms.at(gps_words_iter->second.channel_ID);
                            traveltime_ms = current_prn_delay_ms - actual_min_prn_delay_ms + GPS_STARTOFFSET_ms; //[ms]
                            //std::cout<<"delta_time_ms="<<current_prn_delay_ms-actual_min_prn_delay_ms<<"\r\n";
                            pseudorange_m = traveltime_ms*GPS_C_m_ms; // [m]

                            // update the pseudorange object
                            current_gnss_pseudorange.pseudorange_m = pseudorange_m;
                            current_gnss_pseudorange.timestamp_ms = pseudoranges_timestamp_ms;
                            current_gnss_pseudorange.SV_ID = gps_words_iter->second.satellite_PRN;
                            current_gnss_pseudorange.valid = true;
                            // #### write the pseudorrange block output for this satellite ###
                            *out[gps_words_iter->second.channel_ID] = current_gnss_pseudorange;
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
                            tmp_double = in[i][0].preamble_delay_ms;
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            tmp_double = in[i][0].prn_delay_ms;
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            tmp_double = out[i][0].pseudorange_m;
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            tmp_double = out[i][0].timestamp_ms;
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            tmp_double = out[i][0].SV_ID;
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                        }
            }
            catch (std::ifstream::failure e)
            {
                    std::cout << "Exception writing observables dump file " << e.what() << std::endl;
            }
        }

    consume_each(1); //one by one

    if ((d_sample_counter % d_output_rate_ms) == 0)
        {
            return 1; //Output the observables
        }
    else
        {
            return 0; //hold on
        }
}


