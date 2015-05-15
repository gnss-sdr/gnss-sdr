/*!
 * \file gps_l1_ca_pvt_cc.cc
 * \brief Implementation of a Position Velocity and Time computation block for GPS L1 C/A
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
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

#include "gps_l1_ca_pvt_cc.h"
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
#include "sbas_telemetry_data.h"
#include "sbas_ionospheric_correction.h"

using google::LogMessage;

extern concurrent_map<Gps_Ephemeris> global_gps_ephemeris_map;
extern concurrent_map<Gps_Iono> global_gps_iono_map;
extern concurrent_map<Gps_Utc_Model> global_gps_utc_model_map;

extern concurrent_queue<Sbas_Raw_Msg> global_sbas_raw_msg_queue;
extern concurrent_map<Sbas_Ionosphere_Correction> global_sbas_iono_map;
extern concurrent_map<Sbas_Satellite_Correction> global_sbas_sat_corr_map;
extern concurrent_map<Sbas_Ephemeris> global_sbas_ephemeris_map;

gps_l1_ca_pvt_cc_sptr
gps_l1_ca_make_pvt_cc(unsigned int nchannels, boost::shared_ptr<gr::msg_queue> queue, bool dump, std::string dump_filename, int averaging_depth, bool flag_averaging, int output_rate_ms, int display_rate_ms, bool flag_nmea_tty_port, std::string nmea_dump_filename, std::string nmea_dump_devname)
{
    return gps_l1_ca_pvt_cc_sptr(new gps_l1_ca_pvt_cc(nchannels, queue, dump, dump_filename, averaging_depth, flag_averaging, output_rate_ms, display_rate_ms, flag_nmea_tty_port, nmea_dump_filename, nmea_dump_devname));
}


gps_l1_ca_pvt_cc::gps_l1_ca_pvt_cc(unsigned int nchannels,
        boost::shared_ptr<gr::msg_queue> queue,
        bool dump, std::string dump_filename,
        int averaging_depth,
        bool flag_averaging,
        int output_rate_ms,
        int display_rate_ms,
        bool flag_nmea_tty_port,
        std::string nmea_dump_filename,
        std::string nmea_dump_devname) :
             gr::block("gps_l1_ca_pvt_cc", gr::io_signature::make(nchannels, nchannels,  sizeof(Gnss_Synchro)),
             gr::io_signature::make(1, 1, sizeof(gr_complex)) )
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

    d_ls_pvt = std::make_shared<gps_l1_ca_ls_pvt>((int)nchannels, dump_ls_pvt_filename, d_dump);
    d_ls_pvt->set_averaging_depth(d_averaging_depth);

    d_sample_counter = 0;
    d_last_sample_nav_output = 0;
    d_rx_time = 0.0;

    b_rinex_header_writen = false;
    b_rinex_header_updated = false;
    b_rinex_sbs_header_writen = false;
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
                    catch (std::ifstream::failure e)
                    {
                            LOG(INFO) << "Exception opening PVT dump file " << e.what();
                    }
                }
        }
}



gps_l1_ca_pvt_cc::~gps_l1_ca_pvt_cc()
{}



bool pseudoranges_pairCompare_min(const std::pair<int,Gnss_Synchro>& a, const std::pair<int,Gnss_Synchro>& b)
{
    return (a.second.Pseudorange_m) < (b.second.Pseudorange_m);
}



int gps_l1_ca_pvt_cc::general_work (int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,	gr_vector_void_star &output_items)
{
    d_sample_counter++;

    std::map<int,Gnss_Synchro> gnss_pseudoranges_map;

    Gnss_Synchro **in = (Gnss_Synchro **)  &input_items[0]; //Get the input pointer
    //Gnss_Synchro **out = (Gnss_Synchro **)  &output_items[0]; //Get the output pointer

    for (unsigned int i = 0; i < d_nchannels; i++)
        {
            if (in[i][0].Flag_valid_pseudorange == true)
                {
                    gnss_pseudoranges_map.insert(std::pair<int,Gnss_Synchro>(in[i][0].PRN, in[i][0])); // store valid pseudoranges in a map
                    d_rx_time = in[i][0].d_TOW_at_current_symbol; // all the channels have the same RX timestamp (common RX time pseudoranges)
                }
        }

    // ############ 1. READ EPHEMERIS/UTC_MODE/IONO FROM GLOBAL MAPS ####

    d_ls_pvt->gps_ephemeris_map = global_gps_ephemeris_map.get_map_copy();

    if (global_gps_utc_model_map.size() > 0)
        {
            // UTC MODEL data is shared for all the GPS satellites. Read always at a locked channel
            signed int i = 0;
            while(true)
                {
                    if (in[i][0].Flag_valid_pseudorange == true)
                        {
                            global_gps_utc_model_map.read(i, d_ls_pvt->gps_utc_model);
                            break;
                        }
                    i++;
                    if (i == (signed int)d_nchannels - 1)
                        {
                            break;
                        }
                }
        }

    if (global_gps_iono_map.size() > 0)
        {
            // IONO data is shared for all the GPS satellites. Read always at a locked channel
            signed int i = 0;
            while(true)
                {
                    if (in[i][0].Flag_valid_pseudorange == true)
                        {
                            global_gps_iono_map.read(i, d_ls_pvt->gps_iono);
                            break;
                        }
                    i++;
                    if (i == (signed int)d_nchannels - 1)
                        {
                            break;
                        }
                }
        }

    // update SBAS data collections
    if (global_sbas_iono_map.size() > 0)
        {
            // SBAS ionospheric correction is shared for all the GPS satellites. Read always at ID=0
            global_sbas_iono_map.read(0, d_ls_pvt->sbas_iono);
        }
    d_ls_pvt->sbas_sat_corr_map = global_sbas_sat_corr_map.get_map_copy();
    d_ls_pvt->sbas_ephemeris_map = global_sbas_ephemeris_map.get_map_copy();

    // read SBAS raw messages directly from queue and write them into rinex file
    Sbas_Raw_Msg sbas_raw_msg;
    while (global_sbas_raw_msg_queue.try_pop(sbas_raw_msg))
        {
            // create the header of not yet done
            if(!b_rinex_sbs_header_writen)
                {
                    rp->rinex_sbs_header(rp->sbsFile);
                    b_rinex_sbs_header_writen = true;
                }

            // Define the RX time of the SBAS message by using the GPS time.
            // It has only an effect if there has not been yet a SBAS MT12 available
            // when the message was received.
            if(sbas_raw_msg.get_rx_time_obj().is_related() == false
                    && gnss_pseudoranges_map.size() > 0
                    && d_ls_pvt->gps_ephemeris_map.size() > 0)
                {
                    // doesn't matter which channel/satellite we choose
                    Gnss_Synchro gs = gnss_pseudoranges_map.begin()->second;
                    Gps_Ephemeris eph = d_ls_pvt->gps_ephemeris_map.begin()->second;

                    double relative_rx_time = gs.Tracking_timestamp_secs;
                    int gps_week = eph.i_GPS_week;
                    double gps_sec = gs.d_TOW_at_current_symbol;

                    Sbas_Time_Relation time_rel(relative_rx_time, gps_week, gps_sec);
                    sbas_raw_msg.relate(time_rel);
                }

            // send the message to the rinex logger if it has a valid GPS time stamp
            if(sbas_raw_msg.get_rx_time_obj().is_related())
                {
                    rp->log_rinex_sbs(rp->sbsFile, sbas_raw_msg);
                }
        }

    // ############ 2 COMPUTE THE PVT ################################
    if (gnss_pseudoranges_map.size() > 0 and d_ls_pvt->gps_ephemeris_map.size() > 0)
        {
            // compute on the fly PVT solution
            //mod 8/4/2012 Set the PVT computation rate in this block
            if ((d_sample_counter % d_output_rate_ms) == 0)
                {
                    bool pvt_result;
                    pvt_result = d_ls_pvt->get_PVT(gnss_pseudoranges_map, d_rx_time, d_flag_averaging);
                    if (pvt_result == true)
                        {
                            d_kml_dump->print_position(d_ls_pvt, d_flag_averaging);
                            d_nmea_printer->Print_Nmea_Line(d_ls_pvt, d_flag_averaging);

                            if (!b_rinex_header_writen)
                                {
                                    std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter;
                                    gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.begin();
                                    if (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end())
                                        {
                                            rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, d_rx_time);
                                            rp->rinex_nav_header(rp->navFile, d_ls_pvt->gps_iono, d_ls_pvt->gps_utc_model);
                                            b_rinex_header_writen = true; // do not write header anymore
                                        }
                                }
                            if(b_rinex_header_writen) // Put here another condition to separate annotations (e.g 30 s)
                                {
                                    // Limit the RINEX navigation output rate to 1/6 seg
                                    // Notice that d_sample_counter period is 1ms (for GPS correlators)
                                    if ((d_sample_counter - d_last_sample_nav_output) >= 6000)
                                        {
                                            rp->log_rinex_nav(rp->navFile, d_ls_pvt->gps_ephemeris_map);
                                            d_last_sample_nav_output = d_sample_counter;
                                        }
                                    std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter;
                                    gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.begin();
                                    if (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end())
                                        {
                                            rp->log_rinex_obs(rp->obsFile, gps_ephemeris_iter->second, d_rx_time, gnss_pseudoranges_map);
                                        }
                                    if (!b_rinex_header_updated && (d_ls_pvt->gps_utc_model.d_A0 != 0))
                                        {
                                            rp->update_obs_header(rp->obsFile, d_ls_pvt->gps_utc_model);
                                            rp->update_nav_header(rp->navFile, d_ls_pvt->gps_utc_model, d_ls_pvt->gps_iono);
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
                              << d_ls_pvt->d_VDOP <<" TDOP = " << d_ls_pvt->d_TDOP << " GDOP = " << d_ls_pvt->d_GDOP;
                }
            // MULTIPLEXED FILE RECORDING - Record results to file
            if(d_dump == true)
                {
                    try
                    {
                            double tmp_double;
                            for (unsigned int i = 0; i < d_nchannels ; i++)
                                {
                                    tmp_double = in[i][0].Pseudorange_m;
                                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                                    tmp_double = 0;
                                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                                    d_dump_file.write((char*)&d_rx_time, sizeof(double));
                                }
                    }
                    catch (std::ifstream::failure e)
                    {
                            LOG(WARNING) << "Exception writing observables dump file " << e.what();
                    }
                }
        }

    consume_each(1); //one by one
    return noutput_items;
}


