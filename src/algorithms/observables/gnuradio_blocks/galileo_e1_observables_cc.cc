/*!
 * \file galileo_e1_observables_cc.cc
 * \brief Implementation of the pseudorange computation block for Galileo E1
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

#include "galileo_e1_observables_cc.h"
#include <algorithm>
#include <bitset>
#include <cmath>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include "control_message_factory.h"
#include "gnss_synchro.h"


using google::LogMessage;


galileo_e1_observables_cc_sptr
galileo_e1_make_observables_cc(unsigned int nchannels, boost::shared_ptr<gr::msg_queue> queue, bool dump, std::string dump_filename, int output_rate_ms, bool flag_averaging)
{
    return galileo_e1_observables_cc_sptr(new galileo_e1_observables_cc(nchannels, queue, dump, dump_filename, output_rate_ms, flag_averaging));
}


galileo_e1_observables_cc::galileo_e1_observables_cc(unsigned int nchannels, boost::shared_ptr<gr::msg_queue> queue, bool dump, std::string dump_filename, int output_rate_ms, bool flag_averaging) :
		                        gr::block("galileo_e1_observables_cc", gr::io_signature::make(nchannels, nchannels, sizeof(Gnss_Synchro)),
		                        gr::io_signature::make(nchannels, nchannels, sizeof(Gnss_Synchro)))
{
    // initialize internal vars
    d_queue = queue;
    d_dump = dump;
    d_nchannels = nchannels;
    d_output_rate_ms = output_rate_ms;
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
                            LOG(INFO) << "Observables dump enabled Log file: " << d_dump_filename.c_str();
                    }
                    catch (std::ifstream::failure e)
                    {
                            LOG(WARNING) << "Exception opening observables dump file " << e.what();
                    }
                }
        }
}



galileo_e1_observables_cc::~galileo_e1_observables_cc()
{
    d_dump_file.close();
}



bool Galileo_pairCompare_gnss_synchro_Prn_delay_ms(const std::pair<int,Gnss_Synchro>& a, const std::pair<int,Gnss_Synchro>& b)
{
    return (a.second.Prn_timestamp_ms) < (b.second.Prn_timestamp_ms);
}



bool Galileo_pairCompare_gnss_synchro_d_TOW_at_current_symbol(const std::pair<int,Gnss_Synchro>& a, const std::pair<int,Gnss_Synchro>& b)
{
    return (a.second.d_TOW_at_current_symbol) < (b.second.d_TOW_at_current_symbol);
}



int galileo_e1_observables_cc::general_work (int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,	gr_vector_void_star &output_items)
{
    Gnss_Synchro **in = (Gnss_Synchro **)  &input_items[0];   // Get the input pointer
    Gnss_Synchro **out = (Gnss_Synchro **)  &output_items[0]; // Get the output pointer

    Gnss_Synchro current_gnss_synchro[d_nchannels];
    std::map<int,Gnss_Synchro> current_gnss_synchro_map;
    std::map<int,Gnss_Synchro>::iterator gnss_synchro_iter;

    /*
     * 1. Read the GNSS SYNCHRO objects from available channels
     */
    for (unsigned int i = 0; i < d_nchannels; i++)
        {
            //Copy the telemetry decoder data to local copy
            current_gnss_synchro[i] = in[i][0];
            /*
             * 1.2 Assume no valid pseudoranges
             */
            current_gnss_synchro[i].Flag_valid_pseudorange = false;
            current_gnss_synchro[i].Pseudorange_m = 0.0;
            if (current_gnss_synchro[i].Flag_valid_word)
                {
                    //record the word structure in a map for pseudorange computation
                    current_gnss_synchro_map.insert(std::pair<int, Gnss_Synchro>(current_gnss_synchro[i].Channel_ID, current_gnss_synchro[i]));
                }
        }

    /*
     * 2. Compute RAW pseudoranges using COMMON RECEPTION TIME algorithm. Use only the valid channels (channels that are tracking a satellite)
     */
    if(current_gnss_synchro_map.size() > 0)
        {
            /*
             *  2.1 Use CURRENT set of measurements and find the nearest satellite
             *  common RX time algorithm
             */
            // what is the most recent symbol TOW in the current set? -> this will be the reference symbol
            gnss_synchro_iter = max_element(current_gnss_synchro_map.begin(), current_gnss_synchro_map.end(), Galileo_pairCompare_gnss_synchro_d_TOW_at_current_symbol);
            double d_TOW_reference = gnss_synchro_iter->second.d_TOW_at_current_symbol;
            double d_ref_PRN_rx_time_ms = gnss_synchro_iter->second.Prn_timestamp_ms;
            //int reference_channel= gnss_synchro_iter->second.Channel_ID;

            // Now compute RX time differences due to the PRN alignment in the correlators
            double traveltime_ms;
            double pseudorange_m;
            double delta_rx_time_ms;
            for(gnss_synchro_iter = current_gnss_synchro_map.begin(); gnss_synchro_iter != current_gnss_synchro_map.end(); gnss_synchro_iter++)
                {
                    // compute the required symbol history shift in order to match the reference symbol
                    delta_rx_time_ms = gnss_synchro_iter->second.Prn_timestamp_ms-d_ref_PRN_rx_time_ms;
                    //compute the pseudorange
                    traveltime_ms = (d_TOW_reference - gnss_synchro_iter->second.d_TOW_at_current_symbol)*1000.0 + delta_rx_time_ms + GALILEO_STARTOFFSET_ms;
                    pseudorange_m = traveltime_ms * GALILEO_C_m_ms; // [m]
                    // update the pseudorange object
                    //current_gnss_synchro[gnss_synchro_iter->second.Channel_ID] = gnss_synchro_iter->second;
                    current_gnss_synchro[gnss_synchro_iter->second.Channel_ID].Pseudorange_m = pseudorange_m;
                    current_gnss_synchro[gnss_synchro_iter->second.Channel_ID].Flag_valid_pseudorange = true;
                    current_gnss_synchro[gnss_synchro_iter->second.Channel_ID].d_TOW_at_current_symbol = round(d_TOW_reference*1000)/1000 + GALILEO_STARTOFFSET_ms/1000.0;
                }
        }

      if(d_dump == true)
        {
            // MULTIPLEXED FILE RECORDING - Record results to file
            try
            {
                    double tmp_double;
                    for (unsigned int i = 0; i < d_nchannels ; i++)
                        {
                            tmp_double = current_gnss_synchro[i].d_TOW_at_current_symbol;
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            tmp_double = current_gnss_synchro[i].Prn_timestamp_ms;
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            tmp_double = current_gnss_synchro[i].Pseudorange_m;
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            tmp_double = (double)(current_gnss_synchro[i].Flag_valid_pseudorange==true);
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                            tmp_double = current_gnss_synchro[i].PRN;
                            d_dump_file.write((char*)&tmp_double, sizeof(double));
                        }
            }
            catch (const std::ifstream::failure& e)
            {
                    LOG(WARNING) << "Exception writing observables dump file " << e.what();
            }
        }

    consume_each(1); //one by one
    for (unsigned int i = 0; i < d_nchannels ; i++)
        {
            *out[i] = current_gnss_synchro[i];
        }
    return 1; //Output the observables
}

