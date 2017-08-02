/*!
 * \file hybrid_observables_cc.cc
 * \brief Implementation of the pseudorange computation block for Galileo E1
 * \author Javier Arribas 2017. jarribas(at)cttc.es
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


hybrid_observables_cc_sptr hybrid_make_observables_cc(unsigned int nchannels, bool dump, std::string dump_filename, unsigned int deep_history)
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
  T_rx_s = 0.0;
  T_rx_step_s = 1e-3;// todo: move to gnss-sdr config
  for (unsigned int i = 0; i < d_nchannels; i++)
    {
      d_gnss_synchro_history_queue.push_back(std::deque<Gnss_Synchro>());
    }
  //todo: this is a gnuradio scheduler hack.
  // Migrate the queues to gnuradio set_history to see if the scheduler can handle
  // the multiple output flow
  d_max_noutputs = 100;
  this->set_min_noutput_items(100);

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
  if (d_dump_file.is_open() == true)
    {
      try
      {
          d_dump_file.close();
      }
      catch(const std::exception & ex)
      {
          LOG(WARNING) << "Exception in destructor closing the dump file " << ex.what();
      }
    }
}


bool Hybrid_pairCompare_gnss_synchro_sample_counter(const std::pair<int,Gnss_Synchro>& a, const std::pair<int,Gnss_Synchro>& b)
{
  return (a.second.Tracking_sample_counter) < (b.second.Tracking_sample_counter);
}


bool Hybrid_valueCompare_gnss_synchro_sample_counter(const Gnss_Synchro& a, unsigned long int b)
{
  return (a.Tracking_sample_counter) < (b);
}


bool Hybrid_valueCompare_gnss_synchro_receiver_time(const Gnss_Synchro& a, double b)
{
  return (((double)a.Tracking_sample_counter+a.Code_phase_samples)/(double)a.fs) < (b);
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
  int n_outputs = 0;
  int n_consume[d_nchannels];
  double past_history_s = 100e-3;

  Gnss_Synchro current_gnss_synchro[d_nchannels];

  /*
   * 1. Read the GNSS SYNCHRO objects from available channels.
   *  Multi-rate GNURADIO Block. Read how many input items are avaliable in each channel
   *  Record all synchronization data into queues
   */
  for (unsigned int i = 0; i < d_nchannels; i++)
    {
      n_consume[i] = ninput_items[i];// full throttle
      for (int j = 0; j < n_consume[i]; j++)
        {
          d_gnss_synchro_history_queue[i].push_back(in[i][j]);
        }
      //std::cout<<"push["<<i<<"] items "<<n_consume[i]
      ///         <<" latest T_rx: "<<(double)in[i][ninput_items[i]-1].Tracking_sample_counter/(double)in[i][ninput_items[i]-1].fs
      //         <<" [s] q size: "
      //         <<d_gnss_synchro_history_queue[i].size()
      //         <<std::endl;
    }

  bool channel_history_ok;
  do
    {
      channel_history_ok = true;
      for (unsigned int i = 0; i < d_nchannels; i++)
        {
          if (d_gnss_synchro_history_queue[i].size() < history_deep)
            {
              channel_history_ok = false;
            }

        }
      if (channel_history_ok == true)
        {
          std::map<int,Gnss_Synchro>::iterator gnss_synchro_map_iter;
          std::deque<Gnss_Synchro>::iterator gnss_synchro_deque_iter;

          //1. If the RX time is not set, set the Rx time
          if (T_rx_s == 0)
            {
              //0. Read a gnss_synchro snapshot from the queue and store it in a map
              std::map<int,Gnss_Synchro> gnss_synchro_map;
              for (unsigned int i = 0; i < d_nchannels; i++)
                {
                  gnss_synchro_map.insert(std::pair<int, Gnss_Synchro>(
                      d_gnss_synchro_history_queue[i].front().Channel_ID,
                      d_gnss_synchro_history_queue[i].front()));
                }
              gnss_synchro_map_iter = min_element(gnss_synchro_map.begin(),
                                                  gnss_synchro_map.end(),
                                                  Hybrid_pairCompare_gnss_synchro_sample_counter);
              T_rx_s = (double)gnss_synchro_map_iter->second.Tracking_sample_counter / (double)gnss_synchro_map_iter->second.fs;
              T_rx_s = floor(T_rx_s * 1000.0) / 1000.0; // truncate to ms
              T_rx_s += past_history_s; // increase T_rx to have a minimum past history to interpolate
            }

          //2. Realign RX time in all valid channels
          std::map<int,Gnss_Synchro> realigned_gnss_synchro_map; //container for the aligned set of observables for the selected T_rx
          std::map<int,Gnss_Synchro> adjacent_gnss_synchro_map;  //container for the previous observable values to interpolate
          //shift channels history to match the reference TOW
          for (unsigned int i = 0; i < d_nchannels; i++)
            {
              gnss_synchro_deque_iter = std::lower_bound(d_gnss_synchro_history_queue[i].begin(),
                                                         d_gnss_synchro_history_queue[i].end(),
                                                         T_rx_s,
                                                         Hybrid_valueCompare_gnss_synchro_receiver_time);
              if (gnss_synchro_deque_iter != d_gnss_synchro_history_queue[i].end())
                {
                  if (gnss_synchro_deque_iter->Flag_valid_word == true)
                    {
                      double T_rx_channel = (double)gnss_synchro_deque_iter->Tracking_sample_counter / (double)gnss_synchro_deque_iter->fs;
                      double delta_T_rx_s = T_rx_channel - T_rx_s;

                      //check that T_rx difference is less than a threshold (the correlation interval)
                      if (delta_T_rx_s * 1000.0 < (double)gnss_synchro_deque_iter->correlation_length_ms)
                        {
                          //record the word structure in a map for pseudorange computation
                          //save the previous observable
                          int distance = std::distance(d_gnss_synchro_history_queue[i].begin(), gnss_synchro_deque_iter);
                          if (distance > 0)
                            {
                              if (d_gnss_synchro_history_queue[i].at(distance-1).Flag_valid_word)
                                {
                                  double T_rx_channel_prev = (double)d_gnss_synchro_history_queue[i].at(distance - 1).Tracking_sample_counter / (double)gnss_synchro_deque_iter->fs;
                                  double delta_T_rx_s_prev = T_rx_channel_prev - T_rx_s;
                                  if (fabs(delta_T_rx_s_prev) < fabs(delta_T_rx_s))
                                    {
                                      realigned_gnss_synchro_map.insert(std::pair<int, Gnss_Synchro>(
                                          d_gnss_synchro_history_queue[i].at(distance-1).Channel_ID,
                                          d_gnss_synchro_history_queue[i].at(distance-1)));
                                      adjacent_gnss_synchro_map.insert(std::pair<int, Gnss_Synchro>(gnss_synchro_deque_iter->Channel_ID, *gnss_synchro_deque_iter));
                                    }
                                  else
                                    {
                                      realigned_gnss_synchro_map.insert(std::pair<int, Gnss_Synchro>(gnss_synchro_deque_iter->Channel_ID, *gnss_synchro_deque_iter));
                                      adjacent_gnss_synchro_map.insert(std::pair<int, Gnss_Synchro>(
                                          d_gnss_synchro_history_queue[i].at(distance-1).Channel_ID,
                                          d_gnss_synchro_history_queue[i].at(distance-1)));
                                    }
                                }
                            }
                          else
                            {
                              realigned_gnss_synchro_map.insert(std::pair<int, Gnss_Synchro>(gnss_synchro_deque_iter->Channel_ID, *gnss_synchro_deque_iter));
                            }

                        }
                      else
                        {
                          //std::cout<<"ch["<<i<<"] delta_T_rx:"<<delta_T_rx_s*1000.0<<std::endl;
                        }
                    }
                }
            }

          if(!realigned_gnss_synchro_map.empty())
            {
              /*
               *  2.1 Use CURRENT set of measurements and find the nearest satellite
               *  common RX time algorithm
               */
              // what is the most recent symbol TOW in the current set? -> this will be the reference symbol
              gnss_synchro_map_iter = max_element(realigned_gnss_synchro_map.begin(),
                                                  realigned_gnss_synchro_map.end(),
                                                  Hybrid_pairCompare_gnss_synchro_d_TOW);
              double ref_fs_hz = (double)gnss_synchro_map_iter->second.fs;

              // compute interpolated TOW value at T_rx_s
              int ref_channel_key = gnss_synchro_map_iter->second.Channel_ID;
              Gnss_Synchro adj_obs = adjacent_gnss_synchro_map.at(ref_channel_key);
              double ref_adj_T_rx_s = (double)adj_obs.Tracking_sample_counter / ref_fs_hz + adj_obs.Code_phase_samples / ref_fs_hz;

              double d_TOW_reference = gnss_synchro_map_iter->second.TOW_at_current_symbol_s;
              double d_ref_T_rx_s = (double)gnss_synchro_map_iter->second.Tracking_sample_counter / ref_fs_hz + gnss_synchro_map_iter->second.Code_phase_samples / ref_fs_hz;

              double selected_T_rx_s = T_rx_s;
              // two points linear interpolation using adjacent (adj) values: y=y1+(x-x1)*(y2-y1)/(x2-x1)
              double ref_TOW_at_T_rx_s = adj_obs.TOW_at_current_symbol_s + (selected_T_rx_s - ref_adj_T_rx_s)
                                	    * (d_TOW_reference - adj_obs.TOW_at_current_symbol_s) / (d_ref_T_rx_s - ref_adj_T_rx_s);

              // Now compute RX time differences due to the PRN alignment in the correlators
              double traveltime_ms;
              double pseudorange_m;
              double channel_T_rx_s;
              double channel_fs_hz;
              double channel_TOW_s;
              double delta_T_rx_s;
              for(gnss_synchro_map_iter = realigned_gnss_synchro_map.begin(); gnss_synchro_map_iter != realigned_gnss_synchro_map.end(); gnss_synchro_map_iter++)
                {
                  channel_fs_hz = (double)gnss_synchro_map_iter->second.fs;
                  channel_TOW_s = gnss_synchro_map_iter->second.TOW_at_current_symbol_s;
                  channel_T_rx_s = (double)gnss_synchro_map_iter->second.Tracking_sample_counter / channel_fs_hz + gnss_synchro_map_iter->second.Code_phase_samples / channel_fs_hz;
                  // compute interpolated observation values
                  // two points linear interpolation using adjacent (adj) values: y=y1+(x-x1)*(y2-y1)/(x2-x1)
                  // TOW at the selected receiver time T_rx_s
                  int element_key = gnss_synchro_map_iter->second.Channel_ID;
                  adj_obs = adjacent_gnss_synchro_map.at(element_key);

                  double adj_T_rx_s = (double)adj_obs.Tracking_sample_counter / channel_fs_hz + adj_obs.Code_phase_samples / channel_fs_hz;

                  double channel_TOW_at_T_rx_s = adj_obs.TOW_at_current_symbol_s + (selected_T_rx_s - adj_T_rx_s) * (channel_TOW_s - adj_obs.TOW_at_current_symbol_s) / (channel_T_rx_s - adj_T_rx_s);

                  //Doppler and Accumulated carrier phase
                  double Carrier_phase_lin_rads = adj_obs.Carrier_phase_rads + (selected_T_rx_s - adj_T_rx_s) * (gnss_synchro_map_iter->second.Carrier_phase_rads - adj_obs.Carrier_phase_rads) / (channel_T_rx_s - adj_T_rx_s);
                  double Carrier_Doppler_lin_hz = adj_obs.Carrier_Doppler_hz + (selected_T_rx_s - adj_T_rx_s) * (gnss_synchro_map_iter->second.Carrier_Doppler_hz - adj_obs.Carrier_Doppler_hz) / (channel_T_rx_s - adj_T_rx_s);

                  //compute the pseudorange (no rx time offset correction)
                  traveltime_ms = (ref_TOW_at_T_rx_s - channel_TOW_at_T_rx_s) * 1000.0 + GPS_STARTOFFSET_ms;
                  //convert to meters
                  pseudorange_m = traveltime_ms * GPS_C_m_ms; // [m]
                  // update the pseudorange object
                  current_gnss_synchro[gnss_synchro_map_iter->second.Channel_ID] = gnss_synchro_map_iter->second;
                  current_gnss_synchro[gnss_synchro_map_iter->second.Channel_ID].Pseudorange_m = pseudorange_m;
                  current_gnss_synchro[gnss_synchro_map_iter->second.Channel_ID].Flag_valid_pseudorange = true;
                  // Save the estimated RX time (no RX clock offset correction yet!)
                  current_gnss_synchro[gnss_synchro_map_iter->second.Channel_ID].RX_time = ref_TOW_at_T_rx_s + GPS_STARTOFFSET_ms / 1000.0;

                  current_gnss_synchro[gnss_synchro_map_iter->second.Channel_ID].Carrier_phase_rads = Carrier_phase_lin_rads;
                  current_gnss_synchro[gnss_synchro_map_iter->second.Channel_ID].Carrier_Doppler_hz = Carrier_Doppler_lin_hz;
                }

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
                          tmp_double = current_gnss_synchro[i].Flag_valid_pseudorange;
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
                  out[i][n_outputs] = current_gnss_synchro[i];
                }

              n_outputs++;
            }

          //Move RX time
          T_rx_s = T_rx_s + T_rx_step_s;
          //pop old elements from queue
          for (unsigned int i = 0; i < d_nchannels; i++)
            {
              while (d_gnss_synchro_history_queue[i].front().Tracking_sample_counter / (double)d_gnss_synchro_history_queue[i].front().fs < (T_rx_s - past_history_s))
                {
                  d_gnss_synchro_history_queue[i].pop_front();
                }
            }
        }
    }while(channel_history_ok == true && d_max_noutputs>n_outputs);

  //Multi-rate consume!
  for (unsigned int i = 0; i < d_nchannels; i++)
    {
      consume(i, n_consume[i]); //which input, how many items
    }

  return n_outputs;
}
