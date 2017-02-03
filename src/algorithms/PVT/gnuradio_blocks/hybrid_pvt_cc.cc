/*!
 * \file hybrid_pvt_cc.cc
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

#include "hybrid_pvt_cc.h"
#include <algorithm>
#include <iostream>
#include <map>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/common_factor_rt.hpp>
#include <gnuradio/gr_complex.h>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include "concurrent_map.h"

using google::LogMessage;

hybrid_pvt_cc_sptr
hybrid_make_pvt_cc(unsigned int nchannels,
        bool dump,
        std::string dump_filename,
        int averaging_depth,
        bool flag_averaging,
        int output_rate_ms,
        int display_rate_ms,
        bool flag_nmea_tty_port,
        std::string nmea_dump_filename,
        std::string nmea_dump_devname,
        bool flag_rtcm_server,
        bool flag_rtcm_tty_port,
        unsigned short rtcm_tcp_port,
        unsigned short rtcm_station_id,
        std::map<int,int> rtcm_msg_rate_ms,
        std::string rtcm_dump_devname,
        const unsigned int type_of_receiver)
{
    return hybrid_pvt_cc_sptr(new hybrid_pvt_cc(nchannels,
            dump,
            dump_filename,
            averaging_depth,
            flag_averaging,
            output_rate_ms,
            display_rate_ms,
            flag_nmea_tty_port,
            nmea_dump_filename,
            nmea_dump_devname,
            flag_rtcm_server,
            flag_rtcm_tty_port,
            rtcm_tcp_port,
            rtcm_station_id,
            rtcm_msg_rate_ms,
            rtcm_dump_devname,
            type_of_receiver));
}


void hybrid_pvt_cc::msg_handler_telemetry(pmt::pmt_t msg)
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

            if( pmt::any_ref(msg).type() == typeid(std::shared_ptr<Galileo_Ephemeris>) )
                {
                    // ### Galileo EPHEMERIS ###
                    std::shared_ptr<Galileo_Ephemeris> galileo_eph;
                    galileo_eph = boost::any_cast<std::shared_ptr<Galileo_Ephemeris>>(pmt::any_ref(msg));
                    // insert new ephemeris record
                    DLOG(INFO) << "Galileo New Ephemeris record inserted in global map with TOW =" << galileo_eph->TOW_5
                            << ", GALILEO Week Number =" << galileo_eph->WN_5
                            << " and Ephemeris IOD = " << galileo_eph->IOD_ephemeris;
                    // update/insert new ephemeris record to the global ephemeris map
                    d_ls_pvt->galileo_ephemeris_map[galileo_eph->i_satellite_PRN] = *galileo_eph;
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Galileo_Iono>) )
                {
                    // ### Galileo IONO ###
                    std::shared_ptr<Galileo_Iono> galileo_iono;
                    galileo_iono = boost::any_cast<std::shared_ptr<Galileo_Iono>>(pmt::any_ref(msg));
                    d_ls_pvt->galileo_iono = *galileo_iono;
                    DLOG(INFO) << "New IONO record has arrived ";
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Galileo_Utc_Model>) )
                {
                    // ### Galileo UTC MODEL ###
                    std::shared_ptr<Galileo_Utc_Model> galileo_utc_model;
                    galileo_utc_model = boost::any_cast<std::shared_ptr<Galileo_Utc_Model>>(pmt::any_ref(msg));
                    d_ls_pvt->galileo_utc_model = *galileo_utc_model;
                    DLOG(INFO) << "New UTC record has arrived ";
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Galileo_Almanac>) )
                {
                    // ### Galileo Almanac ###
                    std::shared_ptr<Galileo_Almanac> galileo_almanac;
                    galileo_almanac = boost::any_cast<std::shared_ptr<Galileo_Almanac>>(pmt::any_ref(msg));
                    // update/insert new ephemeris record to the global ephemeris map
                    d_ls_pvt->galileo_almanac = *galileo_almanac;
                    DLOG(INFO) << "New Galileo Almanac has arrived ";
                }

            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Gps_CNAV_Ephemeris>) )
                {
                    // ### GPS CNAV message ###
                    std::shared_ptr<Gps_CNAV_Ephemeris> gps_cnav_ephemeris;
                    gps_cnav_ephemeris = boost::any_cast<std::shared_ptr<Gps_CNAV_Ephemeris>>(pmt::any_ref(msg));
                    // update/insert new ephemeris record to the global ephemeris map
                    d_ls_pvt->gps_cnav_ephemeris_map[gps_cnav_ephemeris->i_satellite_PRN] = *gps_cnav_ephemeris;
                    DLOG(INFO) << "New GPS CNAV ephemeris record has arrived ";
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Gps_CNAV_Iono>) )
                {
                    // ### GPS CNAV IONO ###
                    std::shared_ptr<Gps_CNAV_Iono> gps_cnav_iono;
                    gps_cnav_iono = boost::any_cast<std::shared_ptr<Gps_CNAV_Iono>>(pmt::any_ref(msg));
                    d_ls_pvt->gps_cnav_iono = *gps_cnav_iono;
                    DLOG(INFO) << "New CNAV IONO record has arrived ";
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Gps_CNAV_Utc_Model>) )
                {
                    // ### GPS CNAV UTC MODEL ###
                    std::shared_ptr<Gps_CNAV_Utc_Model> gps_cnav_utc_model;
                    gps_cnav_utc_model = boost::any_cast<std::shared_ptr<Gps_CNAV_Utc_Model>>(pmt::any_ref(msg));
                    d_ls_pvt->gps_cnav_utc_model = *gps_cnav_utc_model;
                    DLOG(INFO) << "New CNAV UTC record has arrived ";
                }
            else
                {
                    LOG(WARNING) << "msg_handler_telemetry unknown object type!";
                }

    }
    catch(boost::bad_any_cast& e)
    {
            LOG(WARNING) << "msg_handler_telemetry Bad any cast!";
    }
}


std::map<int,Gps_Ephemeris> hybrid_pvt_cc::get_GPS_L1_ephemeris_map()
{
    return d_ls_pvt->gps_ephemeris_map;
}


hybrid_pvt_cc::hybrid_pvt_cc(unsigned int nchannels, bool dump, std::string dump_filename,
        int averaging_depth, bool flag_averaging, int output_rate_ms, int display_rate_ms, bool flag_nmea_tty_port,
        std::string nmea_dump_filename, std::string nmea_dump_devname,
        bool flag_rtcm_server, bool flag_rtcm_tty_port, unsigned short rtcm_tcp_port,
        unsigned short rtcm_station_id, std::map<int,int> rtcm_msg_rate_ms, std::string rtcm_dump_devname, const unsigned int type_of_receiver) :
                gr::block("hybrid_pvt_cc", gr::io_signature::make(nchannels, nchannels,  sizeof(Gnss_Synchro)),
                gr::io_signature::make(0, 0, sizeof(gr_complex)))

{
    d_output_rate_ms = output_rate_ms;
    d_display_rate_ms = display_rate_ms;
    d_dump = dump;
    d_nchannels = nchannels;
    d_dump_filename = dump_filename;
    std::string dump_ls_pvt_filename = dump_filename;
    type_of_rx = type_of_receiver;

    // GPS Ephemeris data message port in
    this->message_port_register_in(pmt::mp("telemetry"));
    this->set_msg_handler(pmt::mp("telemetry"), boost::bind(&hybrid_pvt_cc::msg_handler_telemetry, this, _1));

    //initialize kml_printer
    std::string kml_dump_filename;
    kml_dump_filename = d_dump_filename;
    d_kml_dump = std::make_shared<Kml_Printer>();
    d_kml_dump->set_headers(kml_dump_filename);

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
    d_rtcm_printer = std::make_shared<Rtcm_Printer>(rtcm_dump_filename, flag_rtcm_server, flag_rtcm_tty_port, rtcm_tcp_port, rtcm_station_id, rtcm_dump_devname);
    if(rtcm_msg_rate_ms.find(1019) != rtcm_msg_rate_ms.end())
        {
            d_rtcm_MT1019_rate_ms = rtcm_msg_rate_ms[1019];
        }
    else
        {
            d_rtcm_MT1019_rate_ms = boost::math::lcm(5000, d_output_rate_ms);  // default value if not set
        }
    if(rtcm_msg_rate_ms.find(1045) != rtcm_msg_rate_ms.end())
        {
            d_rtcm_MT1045_rate_ms = rtcm_msg_rate_ms[1045];
        }
    else
        {
            d_rtcm_MT1045_rate_ms = boost::math::lcm(5000, d_output_rate_ms);  // default value if not set
        }
    if(rtcm_msg_rate_ms.find(1077) != rtcm_msg_rate_ms.end()) // whatever between 1071 and 1077
        {
            d_rtcm_MT1077_rate_ms = rtcm_msg_rate_ms[1077];
        }
    else
        {
            d_rtcm_MT1077_rate_ms = boost::math::lcm(1000, d_output_rate_ms);  // default value if not set
        }
    if(rtcm_msg_rate_ms.find(1097) != rtcm_msg_rate_ms.end()) // whatever between 1091 and 1097
        {
            d_rtcm_MT1097_rate_ms = rtcm_msg_rate_ms[1097];
            d_rtcm_MSM_rate_ms = rtcm_msg_rate_ms[1097];
        }
    else
        {
            d_rtcm_MT1097_rate_ms = boost::math::lcm(1000, d_output_rate_ms);  // default value if not set
            d_rtcm_MSM_rate_ms = boost::math::lcm(1000, d_output_rate_ms);  // default value if not set
        }
    b_rtcm_writing_started = false;

    d_dump_filename.append("_raw.dat");
    dump_ls_pvt_filename.append("_ls_pvt.dat");
    d_averaging_depth = averaging_depth;
    d_flag_averaging = flag_averaging;

    d_ls_pvt = std::make_shared<hybrid_ls_pvt>((int)nchannels, dump_ls_pvt_filename, d_dump);
    d_ls_pvt->set_averaging_depth(d_averaging_depth);

    d_sample_counter = 0;
    d_last_sample_nav_output = 0;
    d_rx_time = 0.0;
    d_TOW_at_curr_symbol_constellation = 0.0;
    b_rinex_header_written = false;
    b_rinex_header_updated = false;
    rp = std::make_shared<Rinex_Printer>();

    d_last_status_print_seg = 0;

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

    // Create Sys V message queue
    first_fix = true;
    sysv_msg_key = 1101;
    int msgflg = IPC_CREAT | 0666;
    if ((sysv_msqid = msgget(sysv_msg_key, msgflg )) == -1)
        {
            std::cout << "GNSS-SDR can not create message queues!" << std::endl;
            throw new std::exception();
        }
}



hybrid_pvt_cc::~hybrid_pvt_cc()
{
    msgctl(sysv_msqid, IPC_RMID, NULL);
}



bool hybrid_pvt_cc::observables_pairCompare_min(const std::pair<int,Gnss_Synchro>& a, const std::pair<int,Gnss_Synchro>& b)
{
    return (a.second.Pseudorange_m) < (b.second.Pseudorange_m);
}


void hybrid_pvt_cc::print_receiver_status(Gnss_Synchro** channels_synchronization_data)
{
    // Print the current receiver status using std::cout every second
    int current_rx_seg = floor(channels_synchronization_data[0][0].Tracking_timestamp_secs);
    if ( current_rx_seg != d_last_status_print_seg)
        {
            d_last_status_print_seg = current_rx_seg;
            std::cout << "Current input signal time = " << current_rx_seg << " [s]" << std::endl << std::flush;
            //DLOG(INFO) << "GPS L1 C/A Tracking CH " << d_channel <<  ": Satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN)
            //          << ", CN0 = " << d_CN0_SNV_dB_Hz << " [dB-Hz]" << std::endl;
        }
}


bool hybrid_pvt_cc::send_sys_v_ttff_msg(ttff_msgbuf ttff)
{
    /* Fill Sys V message structures */
    int msgsend_size;
    ttff_msgbuf msg;
    msg.ttff = ttff.ttff;
    msgsend_size = sizeof(msg.ttff);
    msg.mtype = 1; /* default message ID */

    /* SEND SOLUTION OVER A MESSAGE QUEUE */
    /* non-blocking Sys V message send */
    msgsnd(sysv_msqid, &msg, msgsend_size, IPC_NOWAIT);
    return true;
}


int hybrid_pvt_cc::general_work (int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items __attribute__((unused)))
{
    d_sample_counter++;
    unsigned int gps_channel = 0;
    unsigned int gal_channel = 0;

    gnss_observables_map.clear();

    Gnss_Synchro **in = (Gnss_Synchro **)  &input_items[0]; //Get the input pointer

    print_receiver_status(in);

    // ############ 1. READ PSEUDORANGES ####
    for (unsigned int i = 0; i < d_nchannels; i++)
        {
            if (in[i][0].Flag_valid_pseudorange == true)
                {
                    gnss_observables_map.insert(std::pair<int,Gnss_Synchro>(i, in[i][0])); // store valid observables in a map.
                    //d_rx_time = in[i][0].d_TOW_at_current_symbol; // all the channels have the same RX timestamp (common RX time pseudoranges)
                    d_TOW_at_curr_symbol_constellation = in[i][0].d_TOW_at_current_symbol; // d_TOW_at_current_symbol not corrected by delta t (just for debug)
                    d_rx_time = in[i][0].d_TOW_hybrid_at_current_symbol; // hybrid rx time, all the channels have the same RX timestamp (common RX time pseudoranges)
                    if(d_ls_pvt->gps_ephemeris_map.size() > 0)
                        {
                            std::map<int,Gps_Ephemeris>::iterator tmp_eph_iter = d_ls_pvt->gps_ephemeris_map.find(in[i][0].PRN);
                            if(tmp_eph_iter != d_ls_pvt->gps_ephemeris_map.end())
                                {
                                    d_rtcm_printer->lock_time(d_ls_pvt->gps_ephemeris_map.find(in[i][0].PRN)->second, d_rx_time, in[i][0]); // keep track of locking time
                                }
                        }
                    if(d_ls_pvt->galileo_ephemeris_map.size() > 0)
                        {
                            std::map<int,Galileo_Ephemeris>::iterator tmp_eph_iter = d_ls_pvt->galileo_ephemeris_map.find(in[i][0].PRN);
                            if(tmp_eph_iter != d_ls_pvt->galileo_ephemeris_map.end())
                                {
                                    d_rtcm_printer->lock_time(d_ls_pvt->galileo_ephemeris_map.find(in[i][0].PRN)->second, d_rx_time, in[i][0]); // keep track of locking time
                                }
                        }
                    if(d_ls_pvt->gps_cnav_ephemeris_map.size() > 0)
                        {
                            std::map<int,Gps_CNAV_Ephemeris>::iterator tmp_eph_iter = d_ls_pvt->gps_cnav_ephemeris_map.find(in[i][0].PRN);
                            if(tmp_eph_iter != d_ls_pvt->gps_cnav_ephemeris_map.end())
                                {
                                    d_rtcm_printer->lock_time(d_ls_pvt->gps_cnav_ephemeris_map.find(in[i][0].PRN)->second, d_rx_time, in[i][0]); // keep track of locking time
                                }
                        }
                }
        }

    std::map<int, Galileo_Ephemeris>::iterator galileo_ephemeris_iter;
    std::map<int, Gps_Ephemeris>::iterator gps_ephemeris_iter;
    std::map<int, Gps_CNAV_Ephemeris>::iterator gps_cnav_ephemeris_iter;
    std::map<int, Gnss_Synchro>::iterator gnss_observables_iter;

    /*
     *   TYPE  |  RECEIVER
     *     0   |  Unknown
     *     1   |  GPS L1 C/A
     *     2   |  GPS L2C
     *     3   |  GPS L5
     *     4   |  Galileo E1B
     *     5   |  Galileo E5a
     *     6   |  Galileo E5b
     *     7   |  GPS L1 C/A + GPS L2C
     *     8   |  GPS L1 C/A + GPS L5
     *     9   |  GPS L1 C/A + Galileo E1B
     *    10   |  GPS L1 C/A + Galileo E5a
     *    11   |  GPS L1 C/A + Galileo E5b
     *    12   |  Galileo E1B + GPS L2C
     *    13   |  Galileo E1B + GPS L5
     *    14   |  Galileo E1B + Galileo E5a
     *    15   |  Galileo E1B + Galileo E5b
     *    16   |  GPS L2C + GPS L5
     *    17   |  GPS L2C + Galileo E5a
     *    18   |  GPS L2C + Galileo E5b
     *    19   |  GPS L5 + Galileo E5a
     *    20   |  GPS L5 + Galileo E5b
     *    21   |  GPS L1 C/A + Galileo E1B + GPS L2C
     *    22   |  GPS L1 C/A + Galileo E1B + GPS L5
     */

    // ############ 2 COMPUTE THE PVT ################################
    if (gnss_observables_map.size() > 0)
        {
            // compute on the fly PVT solution
            if ((d_sample_counter % d_output_rate_ms) == 0)
                {
                    bool pvt_result;
                    pvt_result = d_ls_pvt->get_PVT(gnss_observables_map, d_rx_time, d_flag_averaging);

                    if (pvt_result == true)
                        {
                            // correct the observable to account for the receiver clock offset
                             for (std::map<int,Gnss_Synchro>::iterator it = gnss_observables_map.begin(); it != gnss_observables_map.end(); ++it)
                                 {
                                     it->second.Pseudorange_m = it->second.Pseudorange_m - d_ls_pvt->d_rx_dt_s * GPS_C_m_s;
                                 }
                             if(first_fix == true)
                                 {
                                     std::cout << "First position fix at " << boost::posix_time::to_simple_string(d_ls_pvt->d_position_UTC_time)
                                               << " UTC is Lat = " << d_ls_pvt->d_latitude_d << " [deg], Long = " << d_ls_pvt->d_longitude_d
                                               << " [deg], Height= " << d_ls_pvt->d_height_m << " [m]" << std::endl;
                                     ttff_msgbuf ttff;
                                     ttff.mtype = 1;
                                     ttff.ttff = d_sample_counter;
                                     send_sys_v_ttff_msg(ttff);
                                     first_fix = false;
                                 }
                            d_kml_dump->print_position(d_ls_pvt, d_flag_averaging);
                            d_geojson_printer->print_position(d_ls_pvt, d_flag_averaging);
                            d_nmea_printer->Print_Nmea_Line(d_ls_pvt, d_flag_averaging);

                            // ####################### RINEX FILES #################
                            if (!b_rinex_header_written) //  & we have utc data in nav message!
                                {
                                    galileo_ephemeris_iter = d_ls_pvt->galileo_ephemeris_map.begin();
                                    gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.begin();
                                    gps_cnav_ephemeris_iter = d_ls_pvt->gps_cnav_ephemeris_map.begin();

                                    if(type_of_rx == 1) // GPS L1 C/A only
                                        {
                                            if (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end())
                                                {
                                                    rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, d_rx_time);
                                                    rp->rinex_nav_header(rp->navFile, d_ls_pvt->gps_iono, d_ls_pvt->gps_utc_model);
                                                    b_rinex_header_written = true; // do not write header anymore
                                                }
                                        }
                                    if(type_of_rx == 2) // GPS L2C only
                                        {
                                            if (gps_cnav_ephemeris_iter != d_ls_pvt->gps_cnav_ephemeris_map.end())
                                                {
                                                    rp->rinex_obs_header(rp->obsFile, gps_cnav_ephemeris_iter->second, d_rx_time);
                                                    rp->rinex_nav_header(rp->navFile, d_ls_pvt->gps_cnav_iono, d_ls_pvt->gps_cnav_utc_model);
                                                    b_rinex_header_written = true; // do not write header anymore
                                                }
                                        }
                                    if(type_of_rx == 4) // Galileo E1B only
                                        {
                                            if (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.end())
                                                {
                                                    rp->rinex_obs_header(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time);
                                                    rp->rinex_nav_header(rp->navGalFile, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model, d_ls_pvt->galileo_almanac);
                                                    b_rinex_header_written = true; // do not write header anymore
                                                }
                                        }
                                    if(type_of_rx == 5) // Galileo E5a only
                                        {
                                            std::string signal("5X");
                                            if (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.end())
                                                {
                                                    rp->rinex_obs_header(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, signal);
                                                    rp->rinex_nav_header(rp->navGalFile, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model, d_ls_pvt->galileo_almanac);
                                                    b_rinex_header_written = true; // do not write header anymore
                                                }
                                        }
                                    if(type_of_rx == 6) // Galileo E5b only
                                        {
                                            std::string signal("7X");
                                            if (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.end())
                                                {
                                                    rp->rinex_obs_header(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, signal);
                                                    rp->rinex_nav_header(rp->navGalFile, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model, d_ls_pvt->galileo_almanac);
                                                    b_rinex_header_written = true; // do not write header anymore
                                                }
                                        }
                                    if(type_of_rx == 7) // GPS L1 C/A + GPS L2C
                                        {
                                            if ((gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end()) && (gps_cnav_ephemeris_iter != d_ls_pvt->gps_cnav_ephemeris_map.end()))
                                                {
                                                    rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, d_rx_time);
                                                    rp->rinex_nav_header(rp->navFile, d_ls_pvt->gps_iono, d_ls_pvt->gps_utc_model);
                                                    b_rinex_header_written = true; // do not write header anymore
                                                }
                                        }

                                    if(type_of_rx == 9) // GPS L1 C/A + Galileo E1B
                                        {
                                            if ((galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.end()) && (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end()) )
                                                {
                                                    std::string gal_signal("1B");
                                                    rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, d_rx_time, gal_signal);
                                                    rp->rinex_nav_header(rp->navMixFile,  d_ls_pvt->gps_iono,  d_ls_pvt->gps_utc_model, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model, d_ls_pvt->galileo_almanac);
                                                    b_rinex_header_written = true; // do not write header anymore
                                                }
                                        }
                                    if(type_of_rx == 10) //  GPS L1 C/A + Galileo E5a
                                        {
                                            if ((galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.end()) && (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end()) )
                                                {
                                                    std::string gal_signal("5X");
                                                    rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, d_rx_time, gal_signal);
                                                    rp->rinex_nav_header(rp->navMixFile,  d_ls_pvt->gps_iono,  d_ls_pvt->gps_utc_model, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model, d_ls_pvt->galileo_almanac);
                                                    b_rinex_header_written = true; // do not write header anymore
                                                }
                                        }
                                    if(type_of_rx == 11) //  GPS L1 C/A + Galileo E5b
                                        {
                                            if ((galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.end()) && (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end()) )
                                                {
                                                    std::string gal_signal("7X");
                                                    rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, d_rx_time, gal_signal);
                                                    rp->rinex_nav_header(rp->navMixFile,  d_ls_pvt->gps_iono,  d_ls_pvt->gps_utc_model, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model, d_ls_pvt->galileo_almanac);
                                                    b_rinex_header_written = true; // do not write header anymore
                                                }
                                        }
                                    if(type_of_rx == 14) //  Galileo E1B + Galileo E5a
                                        {
                                            if ((galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.end()) )
                                                {
                                                    std::string gal_signal("1B 5X");
                                                    rp->rinex_obs_header(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, gal_signal);
                                                    rp->rinex_nav_header(rp->navGalFile,  d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model, d_ls_pvt->galileo_almanac);
                                                    b_rinex_header_written = true; // do not write header anymore
                                                }
                                        }
                                    if(type_of_rx == 15) //  Galileo E1B + Galileo E5b
                                        {
                                            if ((galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.end()) )
                                                {
                                                    std::string gal_signal("1B 7X");
                                                    rp->rinex_obs_header(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, gal_signal);
                                                    rp->rinex_nav_header(rp->navGalFile,  d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model, d_ls_pvt->galileo_almanac);
                                                    b_rinex_header_written = true; // do not write header anymore
                                                }
                                        }
                                }
                            if(b_rinex_header_written) // The header is already written, we can now log the navigation message data
                                {
                                    // Limit the RINEX navigation output rate
                                    // Notice that d_sample_counter period is 4ms (for Galileo correlators)
                                    if ((d_sample_counter - d_last_sample_nav_output) >= 6000)
                                        {
                                            if(type_of_rx == 1) // GPS L1 C/A only
                                                {
                                                    rp->log_rinex_nav(rp->navFile, d_ls_pvt->gps_ephemeris_map);
                                                }
                                            if(type_of_rx == 2) // GPS L2C only
                                                {
                                                    rp->log_rinex_nav(rp->navFile, d_ls_pvt->gps_cnav_ephemeris_map);
                                                }
                                            if( (type_of_rx == 4)  || (type_of_rx == 5)  || (type_of_rx == 6) ) // Galileo
                                                {
                                                    rp->log_rinex_nav(rp->navGalFile, d_ls_pvt->galileo_ephemeris_map);
                                                }
                                            if(type_of_rx == 7) // GPS L1 C/A + GPS L2C
                                                {
                                                    rp->log_rinex_nav(rp->navFile, d_ls_pvt->gps_cnav_ephemeris_map);
                                                }
                                            if((type_of_rx == 9) || (type_of_rx == 10) || (type_of_rx == 11)) // GPS L1 C/A + Galileo
                                                {
                                                    rp->log_rinex_nav(rp->navMixFile, d_ls_pvt->gps_ephemeris_map, d_ls_pvt->galileo_ephemeris_map);
                                                }
                                            if((type_of_rx == 14) || (type_of_rx == 15)) //  Galileo E1B + Galileo E5a
                                                {
                                                    rp->log_rinex_nav(rp->navGalFile, d_ls_pvt->galileo_ephemeris_map);
                                                }

                                            d_last_sample_nav_output = d_sample_counter;
                                        }
                                    galileo_ephemeris_iter = d_ls_pvt->galileo_ephemeris_map.begin();
                                    gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.begin();
                                    gps_cnav_ephemeris_iter = d_ls_pvt->gps_cnav_ephemeris_map.begin();

                                    // Log observables into the RINEX file
                                    if(type_of_rx == 1) // GPS L1 C/A only
                                        {
                                            if (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end())
                                                {
                                                    rp->log_rinex_obs(rp->obsFile, gps_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                }
                                            if (!b_rinex_header_updated && (d_ls_pvt->gps_utc_model.d_A0 != 0))
                                                {
                                                    rp->update_obs_header(rp->obsFile, d_ls_pvt->gps_utc_model);
                                                    rp->update_nav_header(rp->navFile, d_ls_pvt->gps_utc_model, d_ls_pvt->gps_iono);
                                                    b_rinex_header_updated = true;
                                                }
                                        }
                                    if(type_of_rx == 2) // GPS L2C only
                                        {
                                            if (gps_cnav_ephemeris_iter != d_ls_pvt->gps_cnav_ephemeris_map.end())
                                                {
                                                    rp->log_rinex_obs(rp->obsFile, gps_cnav_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                }
                                            if (!b_rinex_header_updated && (d_ls_pvt->gps_cnav_utc_model.d_A0 != 0))
                                                {
                                                    rp->update_obs_header(rp->obsFile, d_ls_pvt->gps_cnav_utc_model);
                                                    rp->update_nav_header(rp->navFile, d_ls_pvt->gps_cnav_utc_model, d_ls_pvt->gps_cnav_iono);
                                                    b_rinex_header_updated = true;
                                                }
                                        }
                                    if(type_of_rx == 4) // Galileo E1B only
                                        {
                                            if (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.end())
                                                {
                                                    rp->log_rinex_obs(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, gnss_observables_map, "1B");
                                                }
                                            if (!b_rinex_header_updated && (d_ls_pvt->galileo_utc_model.A0_6 != 0))
                                                {
                                                    rp->update_nav_header(rp->navGalFile, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model, d_ls_pvt->galileo_almanac);
                                                    rp->update_obs_header(rp->obsFile, d_ls_pvt->galileo_utc_model);
                                                    b_rinex_header_updated = true;
                                                }
                                        }
                                    if(type_of_rx == 5) // Galileo E5a only
                                        {
                                            if (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.end())
                                                {
                                                    rp->log_rinex_obs(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, gnss_observables_map, "5X");
                                                }
                                            if (!b_rinex_header_updated && (d_ls_pvt->galileo_utc_model.A0_6 != 0))
                                                {
                                                    rp->update_nav_header(rp->navGalFile, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model, d_ls_pvt->galileo_almanac);
                                                    rp->update_obs_header(rp->obsFile, d_ls_pvt->galileo_utc_model);
                                                    b_rinex_header_updated = true;
                                                }
                                        }
                                    if(type_of_rx == 6) // Galileo E5b only
                                        {
                                            if (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.end())
                                                {
                                                    rp->log_rinex_obs(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, gnss_observables_map, "7X");
                                                }
                                            if (!b_rinex_header_updated && (d_ls_pvt->galileo_utc_model.A0_6 != 0))
                                                {
                                                    rp->update_nav_header(rp->navGalFile, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model, d_ls_pvt->galileo_almanac);
                                                    rp->update_obs_header(rp->obsFile, d_ls_pvt->galileo_utc_model);
                                                    b_rinex_header_updated = true;
                                                }
                                        }
                                    if(type_of_rx == 7) // GPS L1 C/A + GPS L2C
                                        {
                                            if( (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end()) && (gps_cnav_ephemeris_iter != d_ls_pvt->gps_cnav_ephemeris_map.end()) )
                                                {
                                                    rp->log_rinex_obs(rp->obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                }
                                            if (!b_rinex_header_updated && (d_ls_pvt->gps_utc_model.d_A0 != 0))
                                                {
                                                    rp->update_obs_header(rp->obsFile, d_ls_pvt->gps_utc_model);
                                                    rp->update_nav_header(rp->navFile, d_ls_pvt->gps_utc_model, d_ls_pvt->gps_iono);
                                                    b_rinex_header_updated = true;
                                                }
                                        }
                                    if(type_of_rx == 9) // GPS L1 C/A + Galileo E1B
                                        {
                                            if ((galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.end()) && (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end())  )
                                                {
                                                    rp->log_rinex_obs(rp->obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                }
                                            if (!b_rinex_header_updated && (d_ls_pvt->gps_utc_model.d_A0 != 0))
                                                {
                                                    rp->update_obs_header(rp->obsFile, d_ls_pvt->gps_utc_model);
                                                    rp->update_nav_header(rp->navMixFile, d_ls_pvt->gps_iono,  d_ls_pvt->gps_utc_model, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model, d_ls_pvt->galileo_almanac);
                                                    b_rinex_header_updated = true;
                                                }
                                        }
                                    if(type_of_rx == 14) // Galileo E1B + Galileo E5a
                                        {
                                            if (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.end())
                                                {
                                                    rp->log_rinex_obs(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, gnss_observables_map, "1B 5X");
                                                }
                                            if (!b_rinex_header_updated && (d_ls_pvt->galileo_utc_model.A0_6 != 0))
                                                {
                                                    rp->update_nav_header(rp->navGalFile, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model, d_ls_pvt->galileo_almanac);
                                                    rp->update_obs_header(rp->obsFile, d_ls_pvt->galileo_utc_model);
                                                    b_rinex_header_updated = true;
                                                }
                                        }
                                    if(type_of_rx == 15) // Galileo E1B + Galileo E5b
                                        {
                                            if (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.end())
                                                {
                                                    rp->log_rinex_obs(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, gnss_observables_map, "1B 7X");
                                                }
                                            if (!b_rinex_header_updated && (d_ls_pvt->galileo_utc_model.A0_6 != 0))
                                                {
                                                    rp->update_nav_header(rp->navGalFile, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model, d_ls_pvt->galileo_almanac);
                                                    rp->update_obs_header(rp->obsFile, d_ls_pvt->galileo_utc_model);
                                                    b_rinex_header_updated = true;
                                                }
                                        }
                                }

                            // ####################### RTCM MESSAGES #################
                            if(b_rtcm_writing_started)
                                {
                                    if(type_of_rx == 1) // GPS L1 C/A
                                        {
                                            if((d_sample_counter % d_rtcm_MT1019_rate_ms) == 0)
                                                {
                                                    for(std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.begin(); gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end(); gps_ephemeris_iter++ )
                                                        {
                                                            d_rtcm_printer->Print_Rtcm_MT1019(gps_ephemeris_iter->second);
                                                        }
                                                }
                                            if((d_sample_counter % d_rtcm_MSM_rate_ms) == 0)
                                                {
                                                    std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter;
                                                    gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.begin();
                                                    if (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end())
                                                        {
                                                            d_rtcm_printer->Print_Rtcm_MSM(7, gps_ephemeris_iter->second, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                        }
                                                }
                                        }
                                    if((type_of_rx == 4) || (type_of_rx == 5) || (type_of_rx == 6) || (type_of_rx == 14) || (type_of_rx == 15)) // Galileo
                                        {
                                            if((d_sample_counter % (d_rtcm_MT1045_rate_ms / 4) ) == 0)
                                                {
                                                    for(std::map<int,Galileo_Ephemeris>::iterator gal_ephemeris_iter = d_ls_pvt->galileo_ephemeris_map.begin(); gal_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.end(); gal_ephemeris_iter++ )
                                                        {
                                                            d_rtcm_printer->Print_Rtcm_MT1045(gal_ephemeris_iter->second);
                                                        }
                                                }
                                            if((d_sample_counter % (d_rtcm_MSM_rate_ms / 4) ) == 0)
                                                {
                                                    std::map<int,Galileo_Ephemeris>::iterator gal_ephemeris_iter;
                                                    gal_ephemeris_iter = d_ls_pvt->galileo_ephemeris_map.begin();
                                                    if (gal_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.end())
                                                        {
                                                            d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_ephemeris_iter->second, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                        }
                                                }
                                        }
                                    if(type_of_rx == 7) // GPS L1 C/A + GPS L2C
                                        {
                                            if((d_sample_counter % d_rtcm_MT1019_rate_ms) == 0)
                                                {
                                                    for(std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.begin(); gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end(); gps_ephemeris_iter++ )
                                                        {
                                                            d_rtcm_printer->Print_Rtcm_MT1019(gps_ephemeris_iter->second);
                                                        }
                                                }
                                            if((d_sample_counter % d_rtcm_MSM_rate_ms) == 0)
                                                 {
                                                     std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter;
                                                     gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.begin();
                                                     std::map<int,Gps_CNAV_Ephemeris>::iterator gps_cnav_ephemeris_iter;
                                                     gps_cnav_ephemeris_iter = d_ls_pvt->gps_cnav_ephemeris_map.begin();
                                                     if ((gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end()) && (gps_cnav_ephemeris_iter != d_ls_pvt->gps_cnav_ephemeris_map.end()) )
                                                         {
                                                             d_rtcm_printer->Print_Rtcm_MSM(7, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                         }
                                                 }
                                        }
                                    if(type_of_rx == 9) // GPS L1 C/A + Galileo E1B
                                        {
                                            if(((d_sample_counter % (d_rtcm_MT1019_rate_ms / 4)) == 0) && (d_rtcm_MT1019_rate_ms != 0))
                                                {
                                                    for(gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.begin(); gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end(); gps_ephemeris_iter++ )
                                                        {
                                                            d_rtcm_printer->Print_Rtcm_MT1019(gps_ephemeris_iter->second);
                                                        }
                                                }
                                            if(((d_sample_counter % (d_rtcm_MT1045_rate_ms / 4)) == 0) && (d_rtcm_MT1045_rate_ms != 0))
                                                {
                                                    for(galileo_ephemeris_iter = d_ls_pvt->galileo_ephemeris_map.begin(); galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.end(); galileo_ephemeris_iter++ )
                                                        {
                                                            d_rtcm_printer->Print_Rtcm_MT1045(galileo_ephemeris_iter->second);
                                                        }
                                                }
                                            if(((d_sample_counter % (d_rtcm_MT1097_rate_ms / 4) ) == 0) || ((d_sample_counter % (d_rtcm_MT1077_rate_ms / 4) ) == 0))
                                                {
                                                    //gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.end();
                                                    //galileo_ephemeris_iter = d_ls_pvt->galileo_ephemeris_map.end();
                                                    unsigned int i = 0;
                                                    for (gnss_observables_iter = gnss_observables_map.begin(); gnss_observables_iter != gnss_observables_map.end(); gnss_observables_iter++)
                                                        {
                                                            std::string system(&gnss_observables_iter->second.System, 1);
                                                            if(gps_channel == 0)
                                                                {
                                                                    if(system.compare("G") == 0)
                                                                        {
                                                                            // This is a channel with valid GPS signal
                                                                            gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                            if (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end())
                                                                                {
                                                                                    gps_channel = i;
                                                                                }
                                                                        }
                                                                }
                                                            if(gal_channel == 0)
                                                                {
                                                                    if(system.compare("E") == 0)
                                                                        {
                                                                            galileo_ephemeris_iter = d_ls_pvt->galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                            if (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.end())
                                                                                {
                                                                                    gal_channel = i;
                                                                                }
                                                                        }
                                                                }
                                                            i++;
                                                        }
                                                    if(((d_sample_counter % (d_rtcm_MT1097_rate_ms / 4) ) == 0) && (d_rtcm_MT1097_rate_ms != 0) )
                                                        {

                                                            if (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.end())
                                                                {
                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, galileo_ephemeris_iter->second, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                }
                                                        }
                                                    if(((d_sample_counter % (d_rtcm_MT1077_rate_ms / 4) ) == 0) && (d_rtcm_MT1077_rate_ms != 0) )
                                                        {
                                                            if (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end())
                                                                {
                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, gps_ephemeris_iter->second, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                }
                                                        }
                                                }
                                        }
                                }

                            if(!b_rtcm_writing_started) // the first time
                                {
                                    if(type_of_rx == 1) // GPS L1 C/A
                                        {
                                            for(std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.begin(); gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end(); gps_ephemeris_iter++ )
                                                {
                                                    d_rtcm_printer->Print_Rtcm_MT1019(gps_ephemeris_iter->second);
                                                }

                                            std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.begin();

                                            if (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end())
                                                {
                                                    d_rtcm_printer->Print_Rtcm_MSM(7, gps_ephemeris_iter->second, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                }
                                            b_rtcm_writing_started = true;
                                        }

                                    if((type_of_rx == 4) || (type_of_rx == 5) || (type_of_rx == 6) || (type_of_rx == 14) || (type_of_rx == 15)) // Galileo
                                        {
                                            for(std::map<int,Galileo_Ephemeris>::iterator gal_ephemeris_iter = d_ls_pvt->galileo_ephemeris_map.begin(); gal_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.end(); gal_ephemeris_iter++ )
                                                {
                                                    d_rtcm_printer->Print_Rtcm_MT1045(gal_ephemeris_iter->second);
                                                }

                                            std::map<int,Galileo_Ephemeris>::iterator gal_ephemeris_iter = d_ls_pvt->galileo_ephemeris_map.begin();

                                            if (gal_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.end())
                                                {
                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_ephemeris_iter->second, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                }
                                            b_rtcm_writing_started = true;
                                        }
                                    if(type_of_rx == 7) // GPS L1 C/A + GPS L2C
                                        {
                                            for(std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.begin(); gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end(); gps_ephemeris_iter++ )
                                                {
                                                    d_rtcm_printer->Print_Rtcm_MT1019(gps_ephemeris_iter->second);
                                                }

                                            std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.begin();
                                            std::map<int,Gps_CNAV_Ephemeris>::iterator gps_cnav_ephemeris_iter = d_ls_pvt->gps_cnav_ephemeris_map.begin();

                                            if ((gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end()) && (gps_cnav_ephemeris_iter != d_ls_pvt->gps_cnav_ephemeris_map.end()))
                                                {
                                                    d_rtcm_printer->Print_Rtcm_MSM(7, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                }
                                            b_rtcm_writing_started = true;
                                        }
                                    if(type_of_rx == 9) // GPS L1 C/A + Galileo E1B
                                        {
                                            if(d_rtcm_MT1019_rate_ms != 0) // allows deactivating messages by setting rate = 0
                                                {
                                                    for(std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.begin(); gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end(); gps_ephemeris_iter++ )
                                                        {
                                                            d_rtcm_printer->Print_Rtcm_MT1019(gps_ephemeris_iter->second);
                                                        }
                                                }
                                            if(d_rtcm_MT1045_rate_ms != 0)
                                                {
                                                    for(galileo_ephemeris_iter = d_ls_pvt->galileo_ephemeris_map.begin(); galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.end(); galileo_ephemeris_iter++ )
                                                        {
                                                            d_rtcm_printer->Print_Rtcm_MT1045(galileo_ephemeris_iter->second);
                                                        }
                                                }

                                            unsigned int i = 0;
                                            for (gnss_observables_iter = gnss_observables_map.begin(); gnss_observables_iter != gnss_observables_map.end(); gnss_observables_iter++)
                                                {
                                                    std::string system(&gnss_observables_iter->second.System, 1);
                                                    if(gps_channel == 0)
                                                        {
                                                            if(system.compare("G") == 0)
                                                                {
                                                                    // This is a channel with valid GPS signal
                                                                    gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                    if (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end())
                                                                        {
                                                                            gps_channel = i;
                                                                        }
                                                                }
                                                        }
                                                    if(gal_channel == 0)
                                                        {
                                                            if(system.compare("E") == 0)
                                                                {
                                                                    galileo_ephemeris_iter = d_ls_pvt->galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                    if (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.end())
                                                                        {
                                                                            gal_channel = i;
                                                                        }
                                                                }
                                                        }
                                                    i++;
                                                }

                                            if (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end() && (d_rtcm_MT1077_rate_ms != 0))
                                                {
                                                    d_rtcm_printer->Print_Rtcm_MSM(7, gps_ephemeris_iter->second, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                }

                                            if (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.end() && (d_rtcm_MT1097_rate_ms != 0) )
                                                {
                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, galileo_ephemeris_iter->second, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                }
                                            b_rtcm_writing_started = true;
                                        }
                                }
                        }
                }

            // DEBUG MESSAGE: Display position in console output
            if (((d_sample_counter % d_display_rate_ms) == 0) and d_ls_pvt->b_valid_position == true)
                {
                    std::cout << "Position at " << boost::posix_time::to_simple_string(d_ls_pvt->d_position_UTC_time)
                    << " UTC using "<< d_ls_pvt->d_valid_observations<<" observations is Lat = " << d_ls_pvt->d_latitude_d << " [deg], Long = " << d_ls_pvt->d_longitude_d
                    << " [deg], Height= " << d_ls_pvt->d_height_m << " [m]" << std::endl;

                    LOG(INFO) << "Position at " << boost::posix_time::to_simple_string(d_ls_pvt->d_position_UTC_time)
                    << " UTC using "<< d_ls_pvt->d_valid_observations<<" observations is Lat = " << d_ls_pvt->d_latitude_d << " [deg], Long = " << d_ls_pvt->d_longitude_d
                    << " [deg], Height= " << d_ls_pvt->d_height_m << " [m]";

                    /* std::cout << "Dilution of Precision at " << boost::posix_time::to_simple_string(d_ls_pvt->d_position_UTC_time)
                    << " UTC using "<< d_ls_pvt->d_valid_observations<<" observations is HDOP = " << d_ls_pvt->d_HDOP << " VDOP = "
                    << d_ls_pvt->d_VDOP <<" TDOP = " << d_ls_pvt->d_TDOP
                    << " GDOP = " << d_ls_pvt->d_GDOP << std::endl; */
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
    return 1;
}
