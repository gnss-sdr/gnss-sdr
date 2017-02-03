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
#include <utility>
#include <vector>
#include <armadillo>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include "gnss_synchro.h"
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

    for (unsigned int i = 0; i < d_nchannels; i++)
        {
            d_acc_carrier_phase_queue_rads.push_back(std::deque<double>(d_nchannels));
            d_carrier_doppler_queue_hz.push_back(std::deque<double>(d_nchannels));
            d_symbol_TOW_queue_s.push_back(std::deque<double>(d_nchannels));
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


bool Hybrid_pairCompare_gnss_synchro_d_TOW_hybrid_at_current_symbol(const std::pair<int,Gnss_Synchro>& a, const std::pair<int,Gnss_Synchro>& b)
{
    return (a.second.d_TOW_hybrid_at_current_symbol) < (b.second.d_TOW_hybrid_at_current_symbol);
}


int hybrid_observables_cc::general_work (int noutput_items,
    gr_vector_int &ninput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    Gnss_Synchro **in = (Gnss_Synchro **)  &input_items[0];   // Get the input pointer
    Gnss_Synchro **out = (Gnss_Synchro **)  &output_items[0]; // Get the output pointer

    Gnss_Synchro current_gnss_synchro[d_nchannels];
    std::map<int,Gnss_Synchro> current_gnss_synchro_map;
    std::map<int,Gnss_Synchro>::iterator gnss_synchro_iter;

    if (d_nchannels != ninput_items.size())
        {
            LOG(WARNING) << "The Observables block is not well connected";
        }

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
                    //################### SAVE DOPPLER AND ACC CARRIER PHASE HISTORIC DATA FOR INTERPOLATION IN OBSERVABLE MODULE #######
                    d_carrier_doppler_queue_hz[i].push_back(current_gnss_synchro[i].Carrier_Doppler_hz);
                    d_acc_carrier_phase_queue_rads[i].push_back(current_gnss_synchro[i].Carrier_phase_rads);
                    // save TOW history
                    d_symbol_TOW_queue_s[i].push_back(current_gnss_synchro[i].d_TOW_at_current_symbol);
                    if (d_carrier_doppler_queue_hz[i].size() > history_deep)
                        {
                            d_carrier_doppler_queue_hz[i].pop_front();
                        }
                    if (d_acc_carrier_phase_queue_rads[i].size() > history_deep)
                        {
                            d_acc_carrier_phase_queue_rads[i].pop_front();
                        }
                    if (d_symbol_TOW_queue_s[i].size() > history_deep)
                        {
                            d_symbol_TOW_queue_s[i].pop_front();
                        }
                }
            else
                {
                    // Clear the observables history for this channel
                    if (d_symbol_TOW_queue_s[i].size() > 0)
                        {
                            d_symbol_TOW_queue_s[i].clear();
                            d_carrier_doppler_queue_hz[i].clear();
                            d_acc_carrier_phase_queue_rads[i].clear();
                        }
                }
        }

    /*
     * 2. Compute RAW pseudoranges using COMMON RECEPTION TIME algorithm. Use only the valid channels (channels that are tracking a satellite)
     */
    DLOG(INFO) << "gnss_synchro set size=" << current_gnss_synchro_map.size();
    double traveltime_ms;
    double pseudorange_m;
    double delta_rx_time_ms;
    double delta_TOW_ms;
    arma::vec symbol_TOW_vec_s;
    arma::vec dopper_vec_hz;
    arma::vec dopper_vec_interp_hz;
    arma::vec acc_phase_vec_rads;
    arma::vec acc_phase_vec_interp_rads;
    arma::vec desired_symbol_TOW(1);
    double start_offset_ms = 0.0;

    if(current_gnss_synchro_map.size() > 0)
        {
            /*
             *  2.1 Use CURRENT set of measurements and find the nearest satellite
             *  common RX time algorithm
             */
            // what is the most recent symbol TOW in the current set? -> this will be the reference symbol
            gnss_synchro_iter = max_element(current_gnss_synchro_map.begin(), current_gnss_synchro_map.end(), Hybrid_pairCompare_gnss_synchro_d_TOW_hybrid_at_current_symbol);
            //gnss_synchro_iter = max_element(current_gnss_synchro_map_gps_only.begin(), current_gnss_synchro_map_gps_only.end(), Hybrid_pairCompare_gnss_synchro_d_TOW_hybrid_at_current_symbol);
            double d_TOW_reference = gnss_synchro_iter->second.d_TOW_hybrid_at_current_symbol;
            DLOG(INFO) << "d_TOW_hybrid_reference [ms] = " << d_TOW_reference * 1000;
            double d_ref_PRN_rx_time_ms = gnss_synchro_iter->second.Prn_timestamp_ms;
            DLOG(INFO) << "ref_PRN_rx_time_ms [ms] = " << d_ref_PRN_rx_time_ms;

            // Now compute RX time differences due to the PRN alignment in the correlators
            for(gnss_synchro_iter = current_gnss_synchro_map.begin(); gnss_synchro_iter != current_gnss_synchro_map.end(); gnss_synchro_iter++)
                {
                    // check and correct synchronization in cross-system pseudoranges!
                    delta_rx_time_ms = gnss_synchro_iter->second.Prn_timestamp_ms - d_ref_PRN_rx_time_ms;
                    delta_TOW_ms = (d_TOW_reference - gnss_synchro_iter->second.d_TOW_hybrid_at_current_symbol) * 1000.0;
                    if(gnss_synchro_iter->second.System == 'E')
                        {
                            start_offset_ms = GALILEO_STARTOFFSET_ms;
                        }
                    if(gnss_synchro_iter->second.System == 'G')
                        {
                            start_offset_ms = GPS_STARTOFFSET_ms;
                        }
                    //compute the pseudorange
                    traveltime_ms = delta_TOW_ms + delta_rx_time_ms + start_offset_ms;
                    pseudorange_m = traveltime_ms * GALILEO_C_m_ms; // [m]
                    DLOG(INFO) << "CH " << gnss_synchro_iter->second.Channel_ID << " tracking GNSS System "
                               << gnss_synchro_iter->second.System << " has PRN start at= " << gnss_synchro_iter->second.Prn_timestamp_ms
                               << " [ms], d_TOW_at_current_symbol = " << (gnss_synchro_iter->second.d_TOW_at_current_symbol) * 1000
                               << " [ms], d_TOW_hybrid_at_current_symbol = "<< (gnss_synchro_iter->second.d_TOW_hybrid_at_current_symbol) * 1000
                               << "[ms], delta_rx_time_ms = " << delta_rx_time_ms << "[ms], travel_time = " << traveltime_ms
                               << ", pseudorange[m] = "<< pseudorange_m;

                    // update the pseudorange object
                    //current_gnss_synchro[gnss_synchro_iter->second.Channel_ID] = gnss_synchro_iter->second;
                    current_gnss_synchro[gnss_synchro_iter->second.Channel_ID].Pseudorange_m = pseudorange_m;
                    current_gnss_synchro[gnss_synchro_iter->second.Channel_ID].Flag_valid_pseudorange = true;
                    current_gnss_synchro[gnss_synchro_iter->second.Channel_ID].d_TOW_hybrid_at_current_symbol = round(d_TOW_reference * 1000) / 1000 + start_offset_ms / 1000.0;
                    if (d_symbol_TOW_queue_s[gnss_synchro_iter->second.Channel_ID].size() >= history_deep)
                        {
                            // compute interpolated observation values for Doppler and Accumulate carrier phase
                            symbol_TOW_vec_s = arma::vec(std::vector<double>(d_symbol_TOW_queue_s[gnss_synchro_iter->second.Channel_ID].begin(), d_symbol_TOW_queue_s[gnss_synchro_iter->second.Channel_ID].end()));
                            acc_phase_vec_rads = arma::vec(std::vector<double>(d_acc_carrier_phase_queue_rads[gnss_synchro_iter->second.Channel_ID].begin(), d_acc_carrier_phase_queue_rads[gnss_synchro_iter->second.Channel_ID].end()));
                            dopper_vec_hz = arma::vec(std::vector<double>(d_carrier_doppler_queue_hz[gnss_synchro_iter->second.Channel_ID].begin(), d_carrier_doppler_queue_hz[gnss_synchro_iter->second.Channel_ID].end()));

                            desired_symbol_TOW[0] = symbol_TOW_vec_s[history_deep - 1] + delta_rx_time_ms / 1000.0;

                            // Curve fitting to cuadratic function
                            arma::mat A = arma::ones<arma::mat> (history_deep, 2);
                            A.col(1) = symbol_TOW_vec_s;

                            arma::mat coef_acc_phase(1,3);
                            arma::mat pinv_A = arma::pinv(A.t() * A) * A.t();
                            coef_acc_phase = pinv_A * acc_phase_vec_rads;
                            arma::mat coef_doppler(1,3);
                            coef_doppler = pinv_A * dopper_vec_hz;
                            arma::vec acc_phase_lin;
                            arma::vec carrier_doppler_lin;
                            acc_phase_lin = coef_acc_phase[0] + coef_acc_phase[1] * desired_symbol_TOW[0];
                            carrier_doppler_lin = coef_doppler[0] + coef_doppler[1] * desired_symbol_TOW[0];
                            current_gnss_synchro[gnss_synchro_iter->second.Channel_ID].Carrier_phase_rads = acc_phase_lin[0];
                            current_gnss_synchro[gnss_synchro_iter->second.Channel_ID].Carrier_Doppler_hz = carrier_doppler_lin[0];
                        }
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
                            tmp_double = current_gnss_synchro[i].d_TOW_hybrid_at_current_symbol;
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

    consume_each(1); //consume one by one

    for (unsigned int i = 0; i < d_nchannels ; i++)
        {
            *out[i] = current_gnss_synchro[i];
        }

    if (noutput_items == 0)
        {
            LOG(WARNING) << "noutput_items = 0";
        }
    return 1;
}
