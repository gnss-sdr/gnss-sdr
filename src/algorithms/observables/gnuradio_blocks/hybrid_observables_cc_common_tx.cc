/*!
 * \file hybrid_observables_cc.cc
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

#include "hybrid_observables_cc.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <vector>
#include <utility>
#include <armadillo>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include "Galileo_E1.h"
#include "GPS_L1_CA.h"



using google::LogMessage;


hybrid_observables_cc_sptr
hybrid_make_observables_cc(unsigned int nchannels, bool dump, std::string dump_filename, unsigned int deep_history)
{
    return hybrid_observables_cc_sptr(new hybrid_observables_cc(nchannels, dump, dump_filename, deep_history));
}


hybrid_observables_cc::hybrid_observables_cc(unsigned int nchannels, bool dump, std::string dump_filename, unsigned int deep_history) :
                                                gr::block("hybrid_observables_cc", gr::io_signature::make(nchannels, nchannels, sizeof(Gnss_Synchro)),
                                                        gr::io_signature::make(nchannels, nchannels, sizeof(Gnss_Synchro)))
{
    // initialize internal vars
    d_dump = dump;
    d_nchannels = nchannels;
    d_dump_filename = dump_filename;
    history_deep = deep_history;
    d_last_ref_TOW=0;
    for (unsigned int i = 0; i < d_nchannels; i++)
    {
        d_gnss_synchro_history_queue.push_back(std::deque<Gnss_Synchro>());
    }

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
            catch (const std::ifstream::failure & e)
            {
                LOG(WARNING) << "Exception opening observables dump file " << e.what();
            }
        }
    }
}


hybrid_observables_cc::~hybrid_observables_cc()
{
    d_dump_file.close();
}


bool Hybrid_pairCompare_gnss_synchro_sample_counter(const std::pair<int,Gnss_Synchro>& a, const std::pair<int,Gnss_Synchro>& b)
{
    return (a.second.Tracking_sample_counter) < (b.second.Tracking_sample_counter);
}

bool Hybrid_pairCompare_gnss_synchro_d_TOW(const std::pair<int,Gnss_Synchro>& a, const std::pair<int,Gnss_Synchro>& b)
{
    return (a.second.TOW_at_current_symbol_s) < (b.second.TOW_at_current_symbol_s);
}
bool Hybrid_valueCompare_gnss_synchro_d_TOW(const Gnss_Synchro& a, double b)
{
    return (a.TOW_at_current_symbol_s) < (b);
}

int hybrid_observables_cc::general_work (int noutput_items,
        gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
{
    Gnss_Synchro **in = (Gnss_Synchro **)  &input_items[0];   // Get the input pointer
    Gnss_Synchro **out = (Gnss_Synchro **)  &output_items[0]; // Get the output pointer

    Gnss_Synchro current_gnss_synchro[d_nchannels];

    if (d_nchannels != ninput_items.size())
    {
        LOG(WARNING) << "The Observables block is not well connected";
    }

    bool valid_observables=false;
    /*
     * 1. Read the GNSS SYNCHRO objects from available channels.
     *  Multi-rate GNURADIO Block. Read how many input items are avaliable in each channel
     *  Record all synchronization data into queues
     */
    for (unsigned int i = 0; i < d_nchannels; i++)
    {

        //TODO: optimize this: Copy the telemetry decoder data to local copy
        current_gnss_synchro[i] = in[i][0];
        /*
         * 1.2 Assume no valid pseudoranges
         */
        current_gnss_synchro[i].Flag_valid_pseudorange = false;
        current_gnss_synchro[i].Pseudorange_m = 0.0;


        for (int j=0;j<ninput_items[i];j++)
        {
            /*
             * 1.2 Assume no valid pseudoranges
             */
            if (in[i][j].Flag_valid_word)
            {
                valid_observables=true;
                d_gnss_synchro_history_queue[i].push_back(in[i][j]);
                if (d_gnss_synchro_history_queue[i].size() > history_deep)
                {
                    d_gnss_synchro_history_queue[i].pop_front();
                }
            }
            else
            {
                // Clear the observables history for this channel
                if (d_gnss_synchro_history_queue[i].size() > 0)
                {
                    d_gnss_synchro_history_queue[i].clear();
                }

            }
        }
    }

    /*
     * 2. Compute RAW pseudoranges using COMMON TRANSMISSION TIME algorithm. Use only the valid channels (channels that are tracking a satellite)
     */
    if(valid_observables==true)
    {
        std::map<int,Gnss_Synchro> current_gnss_synchro_map;
        for (unsigned int i = 0; i < d_nchannels; i++)
        {
            if (d_gnss_synchro_history_queue[i].size() > 0)
            {
                //record the word structure in a map for pseudorange computation
                current_gnss_synchro_map.insert(std::pair<int, Gnss_Synchro>(
                        d_gnss_synchro_history_queue[i].back().Channel_ID,
                        d_gnss_synchro_history_queue[i].back()));
            }

        }

        std::map<int,Gnss_Synchro>::iterator gnss_synchro_map_iter;
        std::deque<Gnss_Synchro>::iterator gnss_synchro_deque_iter;
        //find the most distant satellite (minimum tow) to be the common TX time
        gnss_synchro_map_iter = min_element(current_gnss_synchro_map.begin(), current_gnss_synchro_map.end(), Hybrid_pairCompare_gnss_synchro_d_TOW);
        double TOW_reference_s = gnss_synchro_map_iter->second.TOW_at_current_symbol_s;
        //todo: Use also the Week number to avoid week rollover problems!
        if (TOW_reference_s!=d_last_ref_TOW)
        {
            d_last_ref_TOW=TOW_reference_s;
            //shift channels history to match the reference TOW
            current_gnss_synchro_map.clear();
            for (unsigned int i = 0; i < d_nchannels; i++)
            {
                if (d_gnss_synchro_history_queue[i].size() > 0)
                {
                    gnss_synchro_deque_iter = std::lower_bound(d_gnss_synchro_history_queue[i].begin(),
                            d_gnss_synchro_history_queue[i].end(),
                            TOW_reference_s,
                            Hybrid_valueCompare_gnss_synchro_d_TOW);
                    //check TOW difference less than a threshold
                    if (fabs(gnss_synchro_deque_iter->TOW_at_current_symbol_s-TOW_reference_s)<1e-4)
                    {
                        //record the word structure in a map for pseudorange computation
                        current_gnss_synchro_map.insert(std::pair<int, Gnss_Synchro>(gnss_synchro_deque_iter->Channel_ID,*gnss_synchro_deque_iter));

                        //discard other elements
                        int distance=std::distance(d_gnss_synchro_history_queue[i].begin(), gnss_synchro_deque_iter);


                    }else{
                        std::cout<<"not found valid TOW in history for SV "
                                <<gnss_synchro_deque_iter->Signal
                                <<" "<<gnss_synchro_deque_iter->PRN
                                <<" Diff tow: "<<gnss_synchro_deque_iter->TOW_at_current_symbol_s-TOW_reference_s
                                <<std::endl;

                        int n=0;
                        for(std::deque<Gnss_Synchro>::iterator tmp_iter = d_gnss_synchro_history_queue[i].begin(); tmp_iter != d_gnss_synchro_history_queue[i].end(); tmp_iter++)
                        {

                            std::cout<<"TOW History difference ["<<n
                                    <<"]="<<tmp_iter->TOW_at_current_symbol_s-TOW_reference_s<<std::endl;
                            n++;
                        }
                    }
                }
            }

            // Find the nearest satellite at common transmission time: the one who has the minimum PRN timestamp
            gnss_synchro_map_iter = min_element(current_gnss_synchro_map.begin(), current_gnss_synchro_map.end(), Hybrid_pairCompare_gnss_synchro_sample_counter);
            //std::cout<<"OBS SV REF SAT: "<<gnss_synchro_map_iter->second.Signal<< " "<<gnss_synchro_map_iter->second.PRN<<" TOW Ref: "<<TOW_reference_s<<std::endl;
            unsigned long int ref_sample_counter = gnss_synchro_map_iter->second.Tracking_sample_counter;
            double ref_code_phase_samples = gnss_synchro_map_iter->second.Code_phase_samples;
            // Now compute RX time differences due to the PRN alignment in the correlators
            int delta_rx_time_samples;

            double delta_rx_time_s;
            double traveltime_ms;
            double pseudorange_m;

            for(gnss_synchro_map_iter = current_gnss_synchro_map.begin(); gnss_synchro_map_iter != current_gnss_synchro_map.end(); gnss_synchro_map_iter++)
            {

                //compute the pseudorange (no rx time offset correction)

                delta_rx_time_samples = gnss_synchro_map_iter->second.Tracking_sample_counter-ref_sample_counter;
                delta_rx_time_s = ((double)delta_rx_time_samples + gnss_synchro_map_iter->second.Code_phase_samples-ref_code_phase_samples)/((double)gnss_synchro_map_iter->second.fs);

                traveltime_ms = delta_rx_time_s*1000.0 + GPS_STARTOFFSET_ms;

                //convert to meters
                pseudorange_m = traveltime_ms * GPS_C_m_ms; // [m]

                // update the pseudorange object
                current_gnss_synchro[gnss_synchro_map_iter->second.Channel_ID] = gnss_synchro_map_iter->second;
                current_gnss_synchro[gnss_synchro_map_iter->second.Channel_ID].Pseudorange_m = pseudorange_m;
                current_gnss_synchro[gnss_synchro_map_iter->second.Channel_ID].Flag_valid_pseudorange = true;
                // Save the estimated RX time (no RX clock offset correction yet!)
                current_gnss_synchro[gnss_synchro_map_iter->second.Channel_ID].RX_time = TOW_reference_s + GPS_STARTOFFSET_ms / 1000.0;

                //std::cout<<gnss_synchro_map_iter->second.Signal<<" ["<<gnss_synchro_map_iter->second.PRN
                //        <<"] delta_TOW: "<<(gnss_synchro_map_iter->second.TOW_at_current_symbol_s-TOW_reference_s)*1000.0
                //        <<" [ms] delta_Prn_timestamp : "<<delta_rx_time_s*1000.0
                //       <<" [ms] Pr: "<<pseudorange_m<<" [m]"
                //       <<std::endl;
            }
        }
    }

    //            if (d_Prn_timestamp_queue_s[gnss_synchro_iter->second.Channel_ID].size() >= history_deep)
    //            {
    //                arma::vec d_Prn_timestamp_vec_s;
    //                arma::vec symbol_TOW_vec_s;
    //                arma::vec dopper_vec_hz;
    //                arma::vec dopper_vec_interp_hz;
    //                arma::vec acc_phase_vec_rads;
    //                arma::vec acc_phase_vec_interp_rads;
    //                arma::vec desired_Prn_timestamp_s(1);
    //                arma::vec TOW_at_rx_time_interp_s;
    //
    //                // compute interpolated observation values
    //                d_Prn_timestamp_vec_s = arma::vec(std::vector<double>(d_Prn_timestamp_queue_s[gnss_synchro_iter->second.Channel_ID].begin(), d_Prn_timestamp_queue_s[gnss_synchro_iter->second.Channel_ID].end()));
    //                symbol_TOW_vec_s = arma::vec(std::vector<double>(d_symbol_TOW_queue_s[gnss_synchro_iter->second.Channel_ID].begin(), d_symbol_TOW_queue_s[gnss_synchro_iter->second.Channel_ID].end()));
    //                acc_phase_vec_rads = arma::vec(std::vector<double>(d_acc_carrier_phase_queue_rads[gnss_synchro_iter->second.Channel_ID].begin(), d_acc_carrier_phase_queue_rads[gnss_synchro_iter->second.Channel_ID].end()));
    //                dopper_vec_hz = arma::vec(std::vector<double>(d_carrier_doppler_queue_hz[gnss_synchro_iter->second.Channel_ID].begin(), d_carrier_doppler_queue_hz[gnss_synchro_iter->second.Channel_ID].end()));
    //
    //                desired_Prn_timestamp_s[0]=d_ref_PRN_rx_time_s;
    //
    //                if (ref_channel_id != gnss_synchro_iter->second.Channel_ID)
    //                {
    //                    // Interpolatio/Extrapoladion using Curve fitting to quadratic function
    //                    //arma::interp1(d_Prn_timestamp_vec_s,symbol_TOW_vec_s,desired_Prn_timestamp_s,TOW_at_rx_time_interp_s); //no extrapolation support !
    //
    //                    arma::vec p1 = arma::polyfit(d_Prn_timestamp_vec_s,symbol_TOW_vec_s,1);
    //
    //                    TOW_at_rx_time_interp_s = arma::polyval(p1,desired_Prn_timestamp_s);
    //
    //                    //compute the pseudorange (no rx time offset correction)
    //                    traveltime_ms = (TOW_reference_s - TOW_at_rx_time_interp_s[0]) * 1000.0
    //                            + GPS_STARTOFFSET_ms;
    //
    //                    //std::cout<<gnss_synchro_iter->second.Signal<<"["<<gnss_synchro_iter->second.PRN<<"] TOW_at_rx_time_interp_s: "<<TOW_at_rx_time_interp_s[0]
    //                    //          <<" [s] delta_TOW_s: "<<(TOW_reference_s - gnss_synchro_iter->second.d_TOW_at_current_symbol_s)
    //                    //          <<" [s] delta_TOW_at_rx_time: "<<(TOW_reference_s - TOW_at_rx_time_interp_s[0])
    //                    //          <<" Pr: "<<pseudorange_m<<" [m]"
    //                    //         <<std::endl;
    //                }else{
    //                    //std::cout<<"REF channel "<<ref_channel_id<<" PRN"<<gnss_synchro_iter->second.PRN<<std::endl;
    //                    traveltime_ms = GPS_STARTOFFSET_ms;
    //                }
    //            }


    if(d_dump == true)
    {
        // MULTIPLEXED FILE RECORDING - Record results to file
        try
        {
            double tmp_double;
            for (unsigned int i = 0; i < d_nchannels; i++)
            {
                tmp_double = current_gnss_synchro[i].RX_time;
                d_dump_file.write((char*)&tmp_double, sizeof(double));
                tmp_double = current_gnss_synchro[i].TOW_at_current_symbol_s;
                d_dump_file.write((char*)&tmp_double, sizeof(double));
                tmp_double = current_gnss_synchro[i].Carrier_Doppler_hz;
                d_dump_file.write((char*)&tmp_double, sizeof(double));
                tmp_double = current_gnss_synchro[i].Carrier_phase_rads/GPS_TWO_PI;
                d_dump_file.write((char*)&tmp_double, sizeof(double));
                tmp_double = current_gnss_synchro[i].Pseudorange_m;
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

    for (unsigned int i = 0; i < d_nchannels; i++)
    {
        *out[i] = current_gnss_synchro[i];
    }
    if (noutput_items == 0)
    {
        LOG(WARNING) << "noutput_items = 0";
    }

    //Multi-rate consume!
    for (unsigned int i=0; i<d_nchannels;i++)
    {
        consume(i,ninput_items[i]); //which input, how many items
    }
    return 1;
}
