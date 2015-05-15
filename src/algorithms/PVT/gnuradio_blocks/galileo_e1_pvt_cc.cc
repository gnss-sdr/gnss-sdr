/*!
 * \file galileo_e1_pvt_cc.cc
 * \brief Implementation of a Position Velocity and Time computation block for GPS L1 C/A
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
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

#include "galileo_e1_pvt_cc.h"
#include <algorithm>
#include <bitset>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <gnuradio/gr_complex.h>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include "control_message_factory.h"
#include "gnss_synchro.h"
#include "concurrent_map.h"

using google::LogMessage;

extern concurrent_map<Galileo_Ephemeris> global_galileo_ephemeris_map;
extern concurrent_map<Galileo_Iono> global_galileo_iono_map;
extern concurrent_map<Galileo_Utc_Model> global_galileo_utc_model_map;
extern concurrent_map<Galileo_Almanac> global_galileo_almanac_map;

galileo_e1_pvt_cc_sptr
galileo_e1_make_pvt_cc(unsigned int nchannels, boost::shared_ptr<gr::msg_queue> queue, bool dump, std::string dump_filename, int averaging_depth, bool flag_averaging, int output_rate_ms, int display_rate_ms, bool flag_nmea_tty_port, std::string nmea_dump_filename, std::string nmea_dump_devname)
{
    return galileo_e1_pvt_cc_sptr(new galileo_e1_pvt_cc(nchannels, queue, dump, dump_filename, averaging_depth, flag_averaging, output_rate_ms, display_rate_ms, flag_nmea_tty_port, nmea_dump_filename, nmea_dump_devname));
}


galileo_e1_pvt_cc::galileo_e1_pvt_cc(unsigned int nchannels, boost::shared_ptr<gr::msg_queue> queue, bool dump, std::string dump_filename, int averaging_depth, bool flag_averaging, int output_rate_ms, int display_rate_ms, bool flag_nmea_tty_port, std::string nmea_dump_filename, std::string nmea_dump_devname) :
		                		                gr::block("galileo_e1_pvt_cc", gr::io_signature::make(nchannels, nchannels,  sizeof(Gnss_Synchro)),
		                		                        gr::io_signature::make(1, 1, sizeof(gr_complex)))
{

    d_output_rate_ms = output_rate_ms;
    d_display_rate_ms = display_rate_ms;
    d_queue = queue;
    d_dump = dump;
    d_nchannels = nchannels;
    d_dump_filename = dump_filename;
    std::string dump_ls_pvt_filename = dump_filename;

    //initialize kml_printer
    std::string kml_dump_filename;
    kml_dump_filename = d_dump_filename;
    kml_dump_filename.append(".kml");
    d_kml_dump = std::make_shared<Kml_Printer>();
    d_kml_dump->set_headers(kml_dump_filename);

    //initialize nmea_printer
    d_nmea_printer = std::make_shared<Nmea_Printer>(nmea_dump_filename, flag_nmea_tty_port, nmea_dump_devname);
    d_dump_filename.append("_raw.dat");
    dump_ls_pvt_filename.append("_ls_pvt.dat");
    d_averaging_depth = averaging_depth;
    d_flag_averaging = flag_averaging;

    d_ls_pvt = std::make_shared<galileo_e1_ls_pvt>(nchannels, dump_ls_pvt_filename, d_dump);
    d_ls_pvt->set_averaging_depth(d_averaging_depth);

    d_sample_counter = 0;
    d_last_sample_nav_output = 0;
    d_rx_time = 0.0;

    b_rinex_header_writen = false;
    b_rinex_header_updated = false;
    rp = std::make_shared<Rinex_Printer>();

    // ############# ENABLE DATA FILE LOG #################
    if (d_dump == true)
        {
            if (d_dump_file.is_open() == false)
                {
                    try
                    {
                            d_dump_file.exceptions (std::ifstream::failbit | std::ifstream::badbit );
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "PVT dump enabled Log file: " << d_dump_filename.c_str();
                    }
                    catch (const std::ifstream::failure& e)
                    {
                            LOG(WARNING) << "Exception opening PVT dump file " << e.what();
                    }
                }
        }
}



galileo_e1_pvt_cc::~galileo_e1_pvt_cc()
{}



bool galileo_e1_pvt_cc::pseudoranges_pairCompare_min(const std::pair<int,Gnss_Synchro>& a, const std::pair<int,Gnss_Synchro>& b)
{
    return (a.second.Pseudorange_m) < (b.second.Pseudorange_m);
}



int galileo_e1_pvt_cc::general_work (int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,	gr_vector_void_star &output_items)
{
    d_sample_counter++;

    std::map<int,Gnss_Synchro> gnss_pseudoranges_map;

    Gnss_Synchro **in = (Gnss_Synchro **)  &input_items[0]; //Get the input pointer

    for (unsigned int i = 0; i < d_nchannels; i++)
        {
            if (in[i][0].Flag_valid_pseudorange == true)
                {
                    gnss_pseudoranges_map.insert(std::pair<int,Gnss_Synchro>(in[i][0].PRN, in[i][0])); // store valid pseudoranges in a map
                    d_rx_time = in[i][0].d_TOW_at_current_symbol; // all the channels have the same RX timestamp (common RX time pseudoranges)
                }
        }

    // ############ 1. READ EPHEMERIS/UTC_MODE/IONO FROM GLOBAL MAPS ####

    if (global_galileo_ephemeris_map.size() > 0)
        {
            d_ls_pvt->galileo_ephemeris_map = global_galileo_ephemeris_map.get_map_copy();
        }

    if (global_galileo_utc_model_map.size() > 0)
        {
            // UTC MODEL data is shared for all the Galileo satellites. Read always at a locked channel
            signed int i = 0;
            while(true)
                {
                    if (in[i][0].Flag_valid_pseudorange == true)
                        {
                            global_galileo_utc_model_map.read(i, d_ls_pvt->galileo_utc_model);
                            break;
                        }
                    i++;
                    if (i == (signed int)d_nchannels - 1)
                        {
                            break;
                        }
                }
        }

    if (global_galileo_iono_map.size() > 0)
        {
            // IONO data is shared for all Galileo satellites. Read always at a locked channel
            signed int i = 0;
            while(true)
                {
                    if (in[i][0].Flag_valid_pseudorange == true)
                        {
                            global_galileo_iono_map.read(i, d_ls_pvt->galileo_iono);
                            break;
                        }
                    i++;
                    if (i == (signed int)d_nchannels - 1)
                        {
                            break;
                        }
                }
        }

    if (global_galileo_almanac_map.size() > 0)
        {
            // ALMANAC data is shared for all Galileo satellites. Read always at a locked channel
            signed int i = 0;
            while(true)
                {
                    if (in[i][0].Flag_valid_pseudorange == true)
                        {
                            global_galileo_almanac_map.read(i, d_ls_pvt->galileo_almanac);
                            break;
                        }
                    i++;
                    if (i == (signed int)d_nchannels - 1)
                        {
                            break;
                        }
                }
        }


    // ############ 2 COMPUTE THE PVT ################################
    if (gnss_pseudoranges_map.size() > 0 and d_ls_pvt->galileo_ephemeris_map.size() > 0)
        {
            // compute on the fly PVT solution
            if ((d_sample_counter % d_output_rate_ms) == 0)
                {
                    bool pvt_result;
                    pvt_result = d_ls_pvt->get_PVT(gnss_pseudoranges_map, d_rx_time, d_flag_averaging);

                    if (pvt_result == true)
                        {
                            d_kml_dump->print_position_galileo(d_ls_pvt, d_flag_averaging);
                            //ToDo: Implement Galileo RINEX and Galileo NMEA outputs
                            //   d_nmea_printer->Print_Nmea_Line(d_ls_pvt, d_flag_averaging);
                            //
                            if (!b_rinex_header_writen)
                                {
                                    std::map<int,Galileo_Ephemeris>::iterator galileo_ephemeris_iter;
                                    galileo_ephemeris_iter = d_ls_pvt->galileo_ephemeris_map.begin();
                                    if (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.end())
                                        {
                                            rp->rinex_obs_header(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time);
                                            rp->rinex_nav_header(rp->navGalFile, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model, d_ls_pvt->galileo_almanac);
                                            b_rinex_header_writen = true; // do not write header anymore
                                        }
                                }
                            if(b_rinex_header_writen) // Put here another condition to separate annotations (e.g 30 s)
                                {
                                    // Limit the RINEX navigation output rate to 1/6 seg
                                    // Notice that d_sample_counter period is 4ms (for Galileo correlators)
                                    if ((d_sample_counter - d_last_sample_nav_output) >= 6000)
                                        {
                                            rp->log_rinex_nav(rp->navGalFile, d_ls_pvt->galileo_ephemeris_map);
                                            d_last_sample_nav_output = d_sample_counter;
                                        }
                                    std::map<int, Galileo_Ephemeris>::iterator galileo_ephemeris_iter;
                                    galileo_ephemeris_iter = d_ls_pvt->galileo_ephemeris_map.begin();
                                    if (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.end())
                                        {
                                            rp->log_rinex_obs(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, gnss_pseudoranges_map);
                                        }
                                    if (!b_rinex_header_updated && (d_ls_pvt->galileo_utc_model.A0_6 != 0))
                                        {
                                            rp->update_nav_header(rp->navGalFile, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model, d_ls_pvt->galileo_almanac);
                                            rp->update_obs_header(rp->obsFile, d_ls_pvt->galileo_utc_model);
                                            b_rinex_header_updated = true;
                                        }
                                }
                        }
                }

            // DEBUG MESSAGE: Display position in console output
            if (((d_sample_counter % d_display_rate_ms) == 0) and d_ls_pvt->b_valid_position == true)
                {
                    std::cout << "Position at " << boost::posix_time::to_simple_string(d_ls_pvt->d_position_UTC_time)
                              << " UTC is Lat = " << d_ls_pvt->d_latitude_d << " [deg], Long = " << d_ls_pvt->d_longitude_d
                              << " [deg], Height= " << d_ls_pvt->d_height_m << " [m]" << std::endl;

                    LOG(INFO) << "Position at " << boost::posix_time::to_simple_string(d_ls_pvt->d_position_UTC_time)
                              << " UTC is Lat = " << d_ls_pvt->d_latitude_d << " [deg], Long = " << d_ls_pvt->d_longitude_d
                              << " [deg], Height= " << d_ls_pvt->d_height_m << " [m]";

                    LOG(INFO) << "Dilution of Precision at " << boost::posix_time::to_simple_string(d_ls_pvt->d_position_UTC_time)
                              << " is HDOP = " << d_ls_pvt->d_HDOP << " VDOP = "
                              << d_ls_pvt->d_VDOP <<" TDOP = " << d_ls_pvt->d_TDOP
                              << " GDOP = " << d_ls_pvt->d_GDOP;
                }

            // MULTIPLEXED FILE RECORDING - Record results to file
            if(d_dump == true)
                {
                    try
                    {
                            double tmp_double;
                            for (unsigned int i = 0; i < d_nchannels; i++)
                                {
                                    tmp_double = in[i][0].Pseudorange_m;
                                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                                    tmp_double = 0;
                                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                                    d_dump_file.write((char*)&d_rx_time, sizeof(double));
                                }
                    }
                    catch (const std::ifstream::failure& e)
                    {
                            LOG(WARNING) << "Exception writing observables dump file " << e.what();
                    }
                }
        }

    consume_each(1); //one by one
    return 0;
}


