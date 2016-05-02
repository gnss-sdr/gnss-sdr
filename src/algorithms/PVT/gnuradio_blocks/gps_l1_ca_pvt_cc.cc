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
#include <iostream>
#include <map>
#include <utility>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <gnuradio/gr_complex.h>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include "concurrent_map.h"
#include "sbas_telemetry_data.h"
#include "sbas_ionospheric_correction.h"

using google::LogMessage;

gps_l1_ca_pvt_cc_sptr
gps_l1_ca_make_pvt_cc(unsigned int nchannels, bool dump, std::string dump_filename, int averaging_depth, bool flag_averaging, int output_rate_ms, int display_rate_ms, bool flag_nmea_tty_port, std::string nmea_dump_filename, std::string nmea_dump_devname, bool flag_rtcm_server, bool flag_rtcm_tty_port, std::string rtcm_dump_devname)
{
    return gps_l1_ca_pvt_cc_sptr(new gps_l1_ca_pvt_cc(nchannels, dump, dump_filename, averaging_depth, flag_averaging, output_rate_ms, display_rate_ms, flag_nmea_tty_port, nmea_dump_filename, nmea_dump_devname, flag_rtcm_server, flag_rtcm_tty_port, rtcm_dump_devname));
}


void gps_l1_ca_pvt_cc::msg_handler_telemetry(pmt::pmt_t msg)
{
    try {
            if( pmt::any_ref(msg).type() == typeid(std::shared_ptr<Gps_Ephemeris>) )
                {
                    // ### GPS EPHEMERIS ###
                    std::shared_ptr<Gps_Ephemeris> gps_eph;
                    gps_eph = boost::any_cast<std::shared_ptr<Gps_Ephemeris>>(pmt::any_ref(msg));
                    DLOG(INFO) << "Ephemeris record has arrived from SAT ID "
                            << gps_eph->i_satellite_PRN << " (Block "
                            <<  gps_eph->satelliteBlock[gps_eph->i_satellite_PRN] << ")"
                            << "inserted with Toe="<< gps_eph->d_Toe<<" and GPS Week="
                            << gps_eph->i_GPS_week;
                    // update/insert new ephemeris record to the global ephemeris map
                    d_ls_pvt->gps_ephemeris_map[gps_eph->i_satellite_PRN] = *gps_eph;
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Gps_Iono>) )
                {
                    // ### GPS IONO ###
                    std::shared_ptr<Gps_Iono> gps_iono;
                    gps_iono = boost::any_cast<std::shared_ptr<Gps_Iono>>(pmt::any_ref(msg));
                    d_ls_pvt->gps_iono = *gps_iono;
                    DLOG(INFO) << "New IONO record has arrived ";
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Gps_Utc_Model>) )
                {
                    // ### GPS UTC MODEL ###
                    std::shared_ptr<Gps_Utc_Model> gps_utc_model;
                    gps_utc_model = boost::any_cast<std::shared_ptr<Gps_Utc_Model>>(pmt::any_ref(msg));
                    d_ls_pvt->gps_utc_model = *gps_utc_model;
                    DLOG(INFO) << "New UTC record has arrived ";
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Sbas_Ionosphere_Correction>) )
                {
                    // ### SBAS IONO ###
                    std::shared_ptr<Sbas_Ionosphere_Correction> sbas_iono;
                    sbas_iono = boost::any_cast<std::shared_ptr<Sbas_Ionosphere_Correction>>(pmt::any_ref(msg));
                    d_ls_pvt->sbas_iono = *sbas_iono;
                    DLOG(INFO) << "New SBAS IONO record has arrived ";
                }

            //TODO: add SBAS correction maps here
            //d_ls_pvt->sbas_sat_corr_map = global_sbas_sat_corr_map.get_map_copy();
            //d_ls_pvt->sbas_ephemeris_map = global_sbas_ephemeris_map.get_map_copy();

            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Sbas_Raw_Msg>) )
                {
                    std::shared_ptr<Sbas_Raw_Msg> sbas_raw_msg_ptr;
                    sbas_raw_msg_ptr = boost::any_cast<std::shared_ptr<Sbas_Raw_Msg>>(pmt::any_ref(msg));
                    Sbas_Raw_Msg sbas_raw_msg = *sbas_raw_msg_ptr;
                    // read SBAS raw messages directly from queue and write them into rinex file
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
            else
                {
                    LOG(WARNING) << "msg_handler_telemetry unknown object type!";
                }
    }
    catch(boost::bad_any_cast& e)
    {
            LOG(WARNING) << "msg_handler_telemetry Bad any cast!\n";
    }
}


gps_l1_ca_pvt_cc::gps_l1_ca_pvt_cc(unsigned int nchannels,
        bool dump, std::string dump_filename,
        int averaging_depth,
        bool flag_averaging,
        int output_rate_ms,
        int display_rate_ms,
        bool flag_nmea_tty_port,
        std::string nmea_dump_filename,
        std::string nmea_dump_devname,
        bool flag_rtcm_server,
        bool flag_rtcm_tty_port,
        std::string rtcm_dump_devname) :
             gr::block("gps_l1_ca_pvt_cc", gr::io_signature::make(nchannels, nchannels,  sizeof(Gnss_Synchro)),
             gr::io_signature::make(0, 0, sizeof(gr_complex)) )
{
    d_output_rate_ms = output_rate_ms;
    d_display_rate_ms = display_rate_ms;
    d_dump = dump;
    d_nchannels = nchannels;
    d_dump_filename = dump_filename;
    std::string dump_ls_pvt_filename = dump_filename;

    // GPS Ephemeris data message port in
    this->message_port_register_in(pmt::mp("telemetry"));
    this->set_msg_handler(pmt::mp("telemetry"),
            boost::bind(&gps_l1_ca_pvt_cc::msg_handler_telemetry, this, _1));

    //initialize kml_printer
    std::string kml_dump_filename;
    kml_dump_filename = d_dump_filename;
    d_kml_printer = std::make_shared<Kml_Printer>();
    d_kml_printer->set_headers(kml_dump_filename);

    //initialize geojson_printer
    std::string geojson_dump_filename;
    geojson_dump_filename = d_dump_filename;
    d_geojson_printer = std::make_shared<GeoJSON_Printer>();
    d_geojson_printer->set_headers(geojson_dump_filename);

    //initialize nmea_printer
    d_nmea_printer = std::make_shared<Nmea_Printer>(nmea_dump_filename, flag_nmea_tty_port, nmea_dump_devname);

    //initialize rtcm_printer
    std::string rtcm_dump_filename;
    rtcm_dump_filename = d_dump_filename;
    d_rtcm_printer = std::make_shared<Rtcm_Printer>(rtcm_dump_filename, flag_rtcm_server, flag_rtcm_tty_port, rtcm_dump_devname);
    b_rtcm_writing_started = false;

    d_dump_filename.append("_raw.dat");
    dump_ls_pvt_filename.append("_ls_pvt.dat");
    d_averaging_depth = averaging_depth;
    d_flag_averaging = flag_averaging;

    d_ls_pvt = std::make_shared<gps_l1_ca_ls_pvt>((int)nchannels, dump_ls_pvt_filename, d_dump);
    d_ls_pvt->set_averaging_depth(d_averaging_depth);

    d_sample_counter = 0;
    d_last_sample_nav_output = 0;
    d_rx_time = 0.0;

    d_last_status_print_seg = 0;

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
                    catch (const std::ifstream::failure & e)
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


void gps_l1_ca_pvt_cc::print_receiver_status(Gnss_Synchro** channels_synchronization_data)
{
    // Print the current receiver status using std::cout every second
    int current_rx_seg = floor(channels_synchronization_data[0][0].Tracking_timestamp_secs);
    if ( current_rx_seg!= d_last_status_print_seg)
        {
            d_last_status_print_seg = current_rx_seg;
            std::cout << "Current input signal time = " << current_rx_seg << " [s]" << std::endl << std::flush;
            //DLOG(INFO) << "GPS L1 C/A Tracking CH " << d_channel <<  ": Satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN)
            //          << ", CN0 = " << d_CN0_SNV_dB_Hz << " [dB-Hz]" << std::endl;
        }
}


int gps_l1_ca_pvt_cc::general_work (int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items __attribute__((unused)))
{
    gnss_pseudoranges_map.clear();
    d_sample_counter++;
    Gnss_Synchro **in = (Gnss_Synchro **)  &input_items[0]; //Get the input pointer

    print_receiver_status(in);

    // ############ 1. READ PSEUDORANGES ####
    for (unsigned int i = 0; i < d_nchannels; i++)
        {
            std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter;
            gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.begin();
            if (in[i][0].Flag_valid_pseudorange == true)
                {
                    gnss_pseudoranges_map.insert(std::pair<int,Gnss_Synchro>(in[i][0].PRN, in[i][0])); // store valid pseudoranges in a map
                    d_rx_time = in[i][0].d_TOW_at_current_symbol; // all the channels have the same RX timestamp (common RX time pseudoranges)
                    d_rtcm_printer->lock_time(gps_ephemeris_iter->second, d_rx_time, in[i][0]); // keep track of locking time
                }
            else
                {
                    d_rtcm_printer->lock_time(gps_ephemeris_iter->second, 0.0, in[i][0]);
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
                            d_kml_printer->print_position(d_ls_pvt, d_flag_averaging);
                            d_geojson_printer->print_position(d_ls_pvt, d_flag_averaging);
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
                            if(b_rtcm_writing_started)
                                {
                                    if((d_sample_counter % 1000) == 0)
                                        {
                                            std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter;
                                            gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.begin();
                                            if (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end())
                                                {
                                                    d_rtcm_printer->Print_Rtcm_MSM(4, gps_ephemeris_iter->second, {}, {}, d_rx_time, gnss_pseudoranges_map, 1234, 0, 0, 0, 0, 0);
                                                }
                                        }
                                    if((d_sample_counter % 5000) == 0)
                                        {
                                            std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter;
                                            gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.begin();
                                            if (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end())
                                                {
                                                    d_rtcm_printer->Print_Rtcm_MT1019(gps_ephemeris_iter->second);
                                                }
                                        }
                                }

                            if(!b_rtcm_writing_started) // the first time
                                {
                                    std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter;
                                    gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.begin();
                                    if (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end())
                                        {
                                            d_rtcm_printer->Print_Rtcm_MT1019(gps_ephemeris_iter->second);
                                            d_rtcm_printer->Print_Rtcm_MSM(4, gps_ephemeris_iter->second, {}, {}, d_rx_time, gnss_pseudoranges_map, 1234, 0, 0, 0, 0, 0);
                                        }
                                    b_rtcm_writing_started = true;
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
                    catch (const std::ifstream::failure & e)
                    {
                            LOG(WARNING) << "Exception writing observables dump file " << e.what();
                    }
                }
        }

    consume_each(1); //one by one
    return 1;
}


