/*!
 * \file rtklib_pvt_cc.cc
 * \brief Interface of a Position Velocity and Time computation block
 * \author Javier Arribas, 2017. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "rtklib_pvt_cc.h"
#include "galileo_almanac.h"
#include "galileo_almanac_helper.h"
#include "pvt_conf.h"
#include "display.h"
#include "gnss_sdr_create_directory.h"
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/exception/all.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/serialization/map.hpp>
#include <glog/logging.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/io_signature.h>
#include <algorithm>
#include <iostream>
#include <map>
#include <exception>
#if OLD_BOOST
#include <boost/math/common_factor_rt.hpp>
namespace bc = boost::math;
#else
#include <boost/integer/common_factor_rt.hpp>
namespace bc = boost::integer;
#endif

using google::LogMessage;


rtklib_pvt_cc_sptr rtklib_make_pvt_cc(uint32_t nchannels,
    const Pvt_Conf& conf_,
    rtk_t& rtk)
{
    return rtklib_pvt_cc_sptr(new rtklib_pvt_cc(nchannels,
        conf_,
        rtk));
}


void rtklib_pvt_cc::msg_handler_telemetry(pmt::pmt_t msg)
{
    try
        {
            // ************* GPS telemetry *****************
            if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Gps_Ephemeris>))
                {
                    // ### GPS EPHEMERIS ###
                    std::shared_ptr<Gps_Ephemeris> gps_eph;
                    gps_eph = boost::any_cast<std::shared_ptr<Gps_Ephemeris>>(pmt::any_ref(msg));
                    DLOG(INFO) << "Ephemeris record has arrived from SAT ID "
                               << gps_eph->i_satellite_PRN << " (Block "
                               << gps_eph->satelliteBlock[gps_eph->i_satellite_PRN] << ")"
                               << "inserted with Toe=" << gps_eph->d_Toe << " and GPS Week="
                               << gps_eph->i_GPS_week;
                    // update/insert new ephemeris record to the global ephemeris map
                    d_ls_pvt->gps_ephemeris_map[gps_eph->i_satellite_PRN] = *gps_eph;
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Gps_Iono>))
                {
                    // ### GPS IONO ###
                    std::shared_ptr<Gps_Iono> gps_iono;
                    gps_iono = boost::any_cast<std::shared_ptr<Gps_Iono>>(pmt::any_ref(msg));
                    d_ls_pvt->gps_iono = *gps_iono;
                    DLOG(INFO) << "New IONO record has arrived ";
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Gps_Utc_Model>))
                {
                    // ### GPS UTC MODEL ###
                    std::shared_ptr<Gps_Utc_Model> gps_utc_model;
                    gps_utc_model = boost::any_cast<std::shared_ptr<Gps_Utc_Model>>(pmt::any_ref(msg));
                    d_ls_pvt->gps_utc_model = *gps_utc_model;
                    DLOG(INFO) << "New UTC record has arrived ";
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Gps_CNAV_Ephemeris>))
                {
                    // ### GPS CNAV message ###
                    std::shared_ptr<Gps_CNAV_Ephemeris> gps_cnav_ephemeris;
                    gps_cnav_ephemeris = boost::any_cast<std::shared_ptr<Gps_CNAV_Ephemeris>>(pmt::any_ref(msg));
                    // update/insert new ephemeris record to the global ephemeris map
                    d_ls_pvt->gps_cnav_ephemeris_map[gps_cnav_ephemeris->i_satellite_PRN] = *gps_cnav_ephemeris;
                    DLOG(INFO) << "New GPS CNAV ephemeris record has arrived ";
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Gps_CNAV_Iono>))
                {
                    // ### GPS CNAV IONO ###
                    std::shared_ptr<Gps_CNAV_Iono> gps_cnav_iono;
                    gps_cnav_iono = boost::any_cast<std::shared_ptr<Gps_CNAV_Iono>>(pmt::any_ref(msg));
                    d_ls_pvt->gps_cnav_iono = *gps_cnav_iono;
                    DLOG(INFO) << "New CNAV IONO record has arrived ";
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Gps_CNAV_Utc_Model>))
                {
                    // ### GPS CNAV UTC MODEL ###
                    std::shared_ptr<Gps_CNAV_Utc_Model> gps_cnav_utc_model;
                    gps_cnav_utc_model = boost::any_cast<std::shared_ptr<Gps_CNAV_Utc_Model>>(pmt::any_ref(msg));
                    d_ls_pvt->gps_cnav_utc_model = *gps_cnav_utc_model;
                    DLOG(INFO) << "New CNAV UTC record has arrived ";
                }

            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Gps_Almanac>))
                {
                    // ### GPS ALMANAC ###
                    std::shared_ptr<Gps_Almanac> gps_almanac;
                    gps_almanac = boost::any_cast<std::shared_ptr<Gps_Almanac>>(pmt::any_ref(msg));
                    d_ls_pvt->gps_almanac_map[gps_almanac->i_satellite_PRN] = *gps_almanac;
                    DLOG(INFO) << "New GPS almanac record has arrived ";
                }

            // **************** Galileo telemetry ********************
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Galileo_Ephemeris>))
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
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Galileo_Iono>))
                {
                    // ### Galileo IONO ###
                    std::shared_ptr<Galileo_Iono> galileo_iono;
                    galileo_iono = boost::any_cast<std::shared_ptr<Galileo_Iono>>(pmt::any_ref(msg));
                    d_ls_pvt->galileo_iono = *galileo_iono;
                    DLOG(INFO) << "New IONO record has arrived ";
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Galileo_Utc_Model>))
                {
                    // ### Galileo UTC MODEL ###
                    std::shared_ptr<Galileo_Utc_Model> galileo_utc_model;
                    galileo_utc_model = boost::any_cast<std::shared_ptr<Galileo_Utc_Model>>(pmt::any_ref(msg));
                    d_ls_pvt->galileo_utc_model = *galileo_utc_model;
                    DLOG(INFO) << "New UTC record has arrived ";
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Galileo_Almanac_Helper>))
                {
                    // ### Galileo Almanac ###
                    std::shared_ptr<Galileo_Almanac_Helper> galileo_almanac_helper;
                    galileo_almanac_helper = boost::any_cast<std::shared_ptr<Galileo_Almanac_Helper>>(pmt::any_ref(msg));

                    Galileo_Almanac sv1 = galileo_almanac_helper->get_almanac(1);
                    Galileo_Almanac sv2 = galileo_almanac_helper->get_almanac(2);
                    Galileo_Almanac sv3 = galileo_almanac_helper->get_almanac(3);

                    if (sv1.i_satellite_PRN != 0) d_ls_pvt->galileo_almanac_map[sv1.i_satellite_PRN] = sv1;
                    if (sv2.i_satellite_PRN != 0) d_ls_pvt->galileo_almanac_map[sv2.i_satellite_PRN] = sv2;
                    if (sv3.i_satellite_PRN != 0) d_ls_pvt->galileo_almanac_map[sv3.i_satellite_PRN] = sv3;
                    DLOG(INFO) << "New Galileo Almanac data have arrived ";
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Galileo_Almanac>))
                {
                    // ### Galileo Almanac ###
                    std::shared_ptr<Galileo_Almanac> galileo_alm;
                    galileo_alm = boost::any_cast<std::shared_ptr<Galileo_Almanac>>(pmt::any_ref(msg));
                    // update/insert new almanac record to the global almanac map
                    d_ls_pvt->galileo_almanac_map[galileo_alm->i_satellite_PRN] = *galileo_alm;
                }

            // **************** GLONASS GNAV Telemetry **************************
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Glonass_Gnav_Ephemeris>))
                {
                    // ### GLONASS GNAV EPHEMERIS ###
                    std::shared_ptr<Glonass_Gnav_Ephemeris> glonass_gnav_eph;
                    glonass_gnav_eph = boost::any_cast<std::shared_ptr<Glonass_Gnav_Ephemeris>>(pmt::any_ref(msg));
                    // TODO Add GLONASS with gps week number and tow,
                    // insert new ephemeris record
                    DLOG(INFO) << "GLONASS GNAV New Ephemeris record inserted in global map with TOW =" << glonass_gnav_eph->d_TOW
                               << ", Week Number =" << glonass_gnav_eph->d_WN
                               << " and Ephemeris IOD in UTC = " << glonass_gnav_eph->compute_GLONASS_time(glonass_gnav_eph->d_t_b)
                               << " from SV = " << glonass_gnav_eph->i_satellite_slot_number;
                    // update/insert new ephemeris record to the global ephemeris map
                    d_ls_pvt->glonass_gnav_ephemeris_map[glonass_gnav_eph->i_satellite_PRN] = *glonass_gnav_eph;
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Glonass_Gnav_Utc_Model>))
                {
                    // ### GLONASS GNAV UTC MODEL ###
                    std::shared_ptr<Glonass_Gnav_Utc_Model> glonass_gnav_utc_model;
                    glonass_gnav_utc_model = boost::any_cast<std::shared_ptr<Glonass_Gnav_Utc_Model>>(pmt::any_ref(msg));
                    d_ls_pvt->glonass_gnav_utc_model = *glonass_gnav_utc_model;
                    DLOG(INFO) << "New GLONASS GNAV UTC record has arrived ";
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Glonass_Gnav_Almanac>))
                {
                    // ### GLONASS GNAV Almanac ###
                    std::shared_ptr<Glonass_Gnav_Almanac> glonass_gnav_almanac;
                    glonass_gnav_almanac = boost::any_cast<std::shared_ptr<Glonass_Gnav_Almanac>>(pmt::any_ref(msg));
                    d_ls_pvt->glonass_gnav_almanac = *glonass_gnav_almanac;
                    DLOG(INFO) << "New GLONASS GNAV Almanac has arrived "
                               << ", GLONASS GNAV Slot Number =" << glonass_gnav_almanac->d_n_A;
                }
            else
                {
                    LOG(WARNING) << "msg_handler_telemetry unknown object type!";
                }
        }
    catch (boost::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_telemetry Bad any cast!";
        }
}


std::map<int, Gps_Ephemeris> rtklib_pvt_cc::get_GPS_L1_ephemeris_map()
{
    return d_ls_pvt->gps_ephemeris_map;
}


rtklib_pvt_cc::rtklib_pvt_cc(uint32_t nchannels,
    const Pvt_Conf& conf_,
    rtk_t& rtk) : gr::sync_block("rtklib_pvt_cc",
                      gr::io_signature::make(nchannels, nchannels, sizeof(Gnss_Synchro)),
                      gr::io_signature::make(0, 0, 0))
{
    d_output_rate_ms = conf_.output_rate_ms;
    d_display_rate_ms = conf_.display_rate_ms;
    d_dump = conf_.dump;
    d_dump_mat = conf_.dump_mat and d_dump;
    d_dump_filename = conf_.dump_filename;
    std::string dump_ls_pvt_filename = conf_.dump_filename;
    if (d_dump)
        {
            std::string dump_path;
            // Get path
            if (d_dump_filename.find_last_of("/") != std::string::npos)
                {
                    std::string dump_filename_ = d_dump_filename.substr(d_dump_filename.find_last_of("/") + 1);
                    dump_path = d_dump_filename.substr(0, d_dump_filename.find_last_of("/"));
                    d_dump_filename = dump_filename_;
                }
            else
                {
                    dump_path = std::string(".");
                }
            if (d_dump_filename.empty())
                {
                    d_dump_filename = "pvt";
                }
            // remove extension if any
            if (d_dump_filename.substr(1).find_last_of(".") != std::string::npos)
                {
                    d_dump_filename = d_dump_filename.substr(0, d_dump_filename.find_last_of("."));
                }
            dump_ls_pvt_filename = dump_path + boost::filesystem::path::preferred_separator + d_dump_filename;
            dump_ls_pvt_filename.append(".dat");
            // create directory
            if (!gnss_sdr_create_directory(dump_path))
                {
                    std::cerr << "GNSS-SDR cannot create dump file for the PVT block. Wrong permissions?" << std::endl;
                    d_dump = false;
                }
        }

    d_nchannels = nchannels;

    type_of_rx = conf_.type_of_receiver;

    // GPS Ephemeris data message port in
    this->message_port_register_in(pmt::mp("telemetry"));
    this->set_msg_handler(pmt::mp("telemetry"), boost::bind(&rtklib_pvt_cc::msg_handler_telemetry, this, _1));

    // initialize kml_printer
    std::string kml_dump_filename;
    kml_dump_filename = d_dump_filename;
    d_kml_output_enabled = conf_.kml_output_enabled;
    if (d_kml_output_enabled)
        {
            d_kml_dump = std::make_shared<Kml_Printer>(conf_.kml_output_path);
            d_kml_dump->set_headers(kml_dump_filename);
        }
    else
        {
            d_kml_dump = nullptr;
        }

    // initialize gpx_printer
    std::string gpx_dump_filename;
    gpx_dump_filename = d_dump_filename;
    d_gpx_output_enabled = conf_.gpx_output_enabled;
    if (d_gpx_output_enabled)
        {
            d_gpx_dump = std::make_shared<Gpx_Printer>(conf_.gpx_output_path);
            d_gpx_dump->set_headers(gpx_dump_filename);
        }
    else
        {
            d_gpx_dump = nullptr;
        }

    // initialize geojson_printer
    std::string geojson_dump_filename;
    geojson_dump_filename = d_dump_filename;

    d_geojson_output_enabled = conf_.geojson_output_enabled;
    if (d_geojson_output_enabled)
        {
            d_geojson_printer = std::make_shared<GeoJSON_Printer>(conf_.geojson_output_path);
            d_geojson_printer->set_headers(geojson_dump_filename);
        }
    else
        {
            d_geojson_printer = nullptr;
        }

    // initialize nmea_printer
    d_nmea_printer = std::make_shared<Nmea_Printer>(conf_.nmea_dump_filename, conf_.nmea_output_file_enabled, conf_.flag_nmea_tty_port, conf_.nmea_dump_devname, conf_.nmea_output_file_path);

    // initialize rtcm_printer
    std::string rtcm_dump_filename;
    rtcm_dump_filename = d_dump_filename;
    if (conf_.flag_rtcm_server or conf_.flag_rtcm_tty_port or conf_.rtcm_output_file_enabled)
        {
            d_rtcm_printer = std::make_shared<Rtcm_Printer>(rtcm_dump_filename, conf_.rtcm_output_file_enabled, conf_.flag_rtcm_server, conf_.flag_rtcm_tty_port, conf_.rtcm_tcp_port, conf_.rtcm_station_id, conf_.rtcm_dump_devname, true, conf_.rtcm_output_file_path);
            std::map<int, int> rtcm_msg_rate_ms = conf_.rtcm_msg_rate_ms;
            if (rtcm_msg_rate_ms.find(1019) != rtcm_msg_rate_ms.end())
                {
                    d_rtcm_MT1019_rate_ms = rtcm_msg_rate_ms[1019];
                }
            else
                {
                    d_rtcm_MT1019_rate_ms = bc::lcm(5000, d_output_rate_ms);  // default value if not set
                }
            if (rtcm_msg_rate_ms.find(1020) != rtcm_msg_rate_ms.end())
                {
                    d_rtcm_MT1020_rate_ms = rtcm_msg_rate_ms[1020];
                }
            else
                {
                    d_rtcm_MT1020_rate_ms = bc::lcm(5000, d_output_rate_ms);  // default value if not set
                }
            if (rtcm_msg_rate_ms.find(1045) != rtcm_msg_rate_ms.end())
                {
                    d_rtcm_MT1045_rate_ms = rtcm_msg_rate_ms[1045];
                }
            else
                {
                    d_rtcm_MT1045_rate_ms = bc::lcm(5000, d_output_rate_ms);  // default value if not set
                }
            if (rtcm_msg_rate_ms.find(1077) != rtcm_msg_rate_ms.end())  // whatever between 1071 and 1077
                {
                    d_rtcm_MT1077_rate_ms = rtcm_msg_rate_ms[1077];
                }
            else
                {
                    d_rtcm_MT1077_rate_ms = bc::lcm(1000, d_output_rate_ms);  // default value if not set
                }
            if (rtcm_msg_rate_ms.find(1087) != rtcm_msg_rate_ms.end())  // whatever between 1081 and 1087
                {
                    d_rtcm_MT1087_rate_ms = rtcm_msg_rate_ms[1087];
                }
            else
                {
                    d_rtcm_MT1087_rate_ms = bc::lcm(1000, d_output_rate_ms);  // default value if not set
                }
            if (rtcm_msg_rate_ms.find(1097) != rtcm_msg_rate_ms.end())  // whatever between 1091 and 1097
                {
                    d_rtcm_MT1097_rate_ms = rtcm_msg_rate_ms[1097];
                    d_rtcm_MSM_rate_ms = rtcm_msg_rate_ms[1097];
                }
            else
                {
                    d_rtcm_MT1097_rate_ms = bc::lcm(1000, d_output_rate_ms);  // default value if not set
                    d_rtcm_MSM_rate_ms = bc::lcm(1000, d_output_rate_ms);     // default value if not set
                }
            b_rtcm_writing_started = false;
            b_rtcm_enabled = true;
        }
    else
        {
            d_rtcm_MT1019_rate_ms = 0;
            d_rtcm_MT1045_rate_ms = 0;
            d_rtcm_MT1020_rate_ms = 0;
            d_rtcm_MT1077_rate_ms = 0;
            d_rtcm_MT1087_rate_ms = 0;
            d_rtcm_MT1097_rate_ms = 0;
            d_rtcm_MSM_rate_ms = 0;
            b_rtcm_enabled = false;
            b_rtcm_writing_started = false;
            d_rtcm_printer = nullptr;
        }

    // initialize RINEX printer
    b_rinex_header_written = false;
    b_rinex_header_updated = false;
    b_rinex_output_enabled = conf_.rinex_output_enabled;
    d_rinex_version = conf_.rinex_version;
    if (b_rinex_output_enabled)
        {
            rp = std::make_shared<Rinex_Printer>(d_rinex_version, conf_.rinex_output_path);
        }
    else
        {
            rp = nullptr;
        }
    d_rinexobs_rate_ms = conf_.rinexobs_rate_ms;
    d_rinexnav_rate_ms = conf_.rinexnav_rate_ms;

    // XML printer
    d_xml_storage = conf_.xml_output_enabled;
    if (d_xml_storage)
        {
            xml_base_path = conf_.xml_output_path;
            boost::filesystem::path full_path(boost::filesystem::current_path());
            const boost::filesystem::path p(xml_base_path);
            if (!boost::filesystem::exists(p))
                {
                    std::string new_folder;
                    for (auto& folder : boost::filesystem::path(xml_base_path))
                        {
                            new_folder += folder.string();
                            boost::system::error_code ec;
                            if (!boost::filesystem::exists(new_folder))
                                {
                                    if (!boost::filesystem::create_directory(new_folder, ec))
                                        {
                                            std::cout << "Could not create the " << new_folder << " folder." << std::endl;
                                            xml_base_path = full_path.string();
                                        }
                                }
                            new_folder += boost::filesystem::path::preferred_separator;
                        }
                }
            else
                {
                    xml_base_path = p.string();
                }
            if (xml_base_path.compare(".") != 0)
                {
                    std::cout << "XML files will be stored at " << xml_base_path << std::endl;
                }

            xml_base_path = xml_base_path + boost::filesystem::path::preferred_separator;
        }

    d_ls_pvt = std::make_shared<rtklib_solver>(static_cast<int32_t>(nchannels), dump_ls_pvt_filename, d_dump, d_dump_mat, rtk);
    d_ls_pvt->set_averaging_depth(1);

    d_rx_time = 0.0;

    d_last_status_print_seg = 0;

    // Create Sys V message queue
    first_fix = true;
    sysv_msg_key = 1101;
    int msgflg = IPC_CREAT | 0666;
    if ((sysv_msqid = msgget(sysv_msg_key, msgflg)) == -1)
        {
            std::cout << "GNSS-SDR can not create message queues!" << std::endl;
            throw new std::exception();
        }
    start = std::chrono::system_clock::now();
}


rtklib_pvt_cc::~rtklib_pvt_cc()
{
    msgctl(sysv_msqid, IPC_RMID, NULL);
    if (d_xml_storage)
        {
            // save GPS L2CM ephemeris to XML file
            std::string file_name = xml_base_path + "gps_cnav_ephemeris.xml";
            if (d_ls_pvt->gps_cnav_ephemeris_map.empty() == false)
                {
                    std::ofstream ofs;
                    try
                        {
                            ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                            boost::archive::xml_oarchive xml(ofs);
                            xml << boost::serialization::make_nvp("GNSS-SDR_cnav_ephemeris_map", d_ls_pvt->gps_cnav_ephemeris_map);
                            LOG(INFO) << "Saved GPS L2CM or L5 Ephemeris map data";
                        }
                    catch (std::exception& e)
                        {
                            LOG(WARNING) << e.what();
                        }
                }
            else
                {
                    LOG(INFO) << "Failed to save GPS L2CM or L5 Ephemeris, map is empty";
                }

            // save GPS L1 CA ephemeris to XML file
            file_name = xml_base_path + "gps_ephemeris.xml";
            if (d_ls_pvt->gps_ephemeris_map.empty() == false)
                {
                    std::ofstream ofs;
                    try
                        {
                            ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                            boost::archive::xml_oarchive xml(ofs);
                            xml << boost::serialization::make_nvp("GNSS-SDR_ephemeris_map", d_ls_pvt->gps_ephemeris_map);
                            LOG(INFO) << "Saved GPS L1 CA Ephemeris map data";
                        }
                    catch (const std::exception& e)
                        {
                            LOG(WARNING) << e.what();
                        }
                }
            else
                {
                    LOG(INFO) << "Failed to save GPS L1 CA Ephemeris, map is empty";
                }

            // save Galileo E1 ephemeris to XML file
            file_name = xml_base_path + "gal_ephemeris.xml";
            if (d_ls_pvt->galileo_ephemeris_map.empty() == false)
                {
                    std::ofstream ofs;
                    try
                        {
                            ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                            boost::archive::xml_oarchive xml(ofs);
                            xml << boost::serialization::make_nvp("GNSS-SDR_gal_ephemeris_map", d_ls_pvt->galileo_ephemeris_map);
                            LOG(INFO) << "Saved Galileo E1 Ephemeris map data";
                        }
                    catch (const std::exception& e)
                        {
                            LOG(WARNING) << e.what();
                        }
                }
            else
                {
                    LOG(INFO) << "Failed to save Galileo E1 Ephemeris, map is empty";
                }

            // save GLONASS GNAV ephemeris to XML file
            file_name = xml_base_path + "eph_GLONASS_GNAV.xml";
            if (d_ls_pvt->glonass_gnav_ephemeris_map.empty() == false)
                {
                    std::ofstream ofs;
                    try
                        {
                            ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                            boost::archive::xml_oarchive xml(ofs);
                            xml << boost::serialization::make_nvp("GNSS-SDR_gnav_ephemeris_map", d_ls_pvt->glonass_gnav_ephemeris_map);
                            LOG(INFO) << "Saved GLONASS GNAV Ephemeris map data";
                        }
                    catch (std::exception& e)
                        {
                            LOG(WARNING) << e.what();
                        }
                }
            else
                {
                    LOG(INFO) << "Failed to save GLONASS GNAV Ephemeris, map is empty";
                }

            // Save GPS UTC model parameters
            file_name = xml_base_path + "gps_utc_model.xml";
            if (d_ls_pvt->gps_utc_model.valid)
                {
                    std::ofstream ofs;
                    try
                        {
                            ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                            boost::archive::xml_oarchive xml(ofs);
                            xml << boost::serialization::make_nvp("GNSS-SDR_utc_model", d_ls_pvt->gps_utc_model);
                            LOG(INFO) << "Saved GPS UTC model parameters";
                        }
                    catch (std::exception& e)
                        {
                            LOG(WARNING) << e.what();
                        }
                }
            else
                {
                    LOG(INFO) << "Failed to save GPS UTC model parameters, not valid data";
                }

            // Save Galileo UTC model parameters
            file_name = xml_base_path + "gal_utc_model.xml";
            if (d_ls_pvt->galileo_utc_model.Delta_tLS_6 != 0.0)
                {
                    std::ofstream ofs;
                    try
                        {
                            ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                            boost::archive::xml_oarchive xml(ofs);
                            xml << boost::serialization::make_nvp("GNSS-SDR_gal_utc_model", d_ls_pvt->galileo_utc_model);
                            LOG(INFO) << "Saved Galileo UTC model parameters";
                        }
                    catch (std::exception& e)
                        {
                            LOG(WARNING) << e.what();
                        }
                }
            else
                {
                    LOG(INFO) << "Failed to save Galileo UTC model parameters, not valid data";
                }

            // Save GPS iono parameters
            file_name = xml_base_path + "gps_iono.xml";
            if (d_ls_pvt->gps_iono.valid == true)
                {
                    std::ofstream ofs;
                    try
                        {
                            ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                            boost::archive::xml_oarchive xml(ofs);
                            xml << boost::serialization::make_nvp("GNSS-SDR_iono_model", d_ls_pvt->gps_iono);
                            LOG(INFO) << "Saved GPS ionospheric model parameters";
                        }
                    catch (std::exception& e)
                        {
                            LOG(WARNING) << e.what();
                        }
                }
            else
                {
                    LOG(INFO) << "Failed to save GPS ionospheric model parameters, not valid data";
                }

            // Save GPS CNAV iono parameters
            file_name = xml_base_path + "gps_cnav_iono.xml";
            if (d_ls_pvt->gps_cnav_iono.valid == true)
                {
                    std::ofstream ofs;
                    try
                        {
                            ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                            boost::archive::xml_oarchive xml(ofs);
                            xml << boost::serialization::make_nvp("GNSS-SDR_cnav_iono_model", d_ls_pvt->gps_cnav_iono);
                            LOG(INFO) << "Saved GPS CNAV ionospheric model parameters";
                        }
                    catch (std::exception& e)
                        {
                            LOG(WARNING) << e.what();
                        }
                }
            else
                {
                    LOG(INFO) << "Failed to save GPS CNAV ionospheric model parameters, not valid data";
                }

            // Save Galileo iono parameters
            file_name = xml_base_path + "gal_iono.xml";
            if (d_ls_pvt->galileo_iono.ai0_5 != 0.0)
                {
                    std::ofstream ofs;
                    try
                        {
                            ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                            boost::archive::xml_oarchive xml(ofs);
                            xml << boost::serialization::make_nvp("GNSS-SDR_gal_iono_model", d_ls_pvt->galileo_iono);
                            LOG(INFO) << "Saved Galileo ionospheric model parameters";
                        }
                    catch (std::exception& e)
                        {
                            LOG(WARNING) << e.what();
                        }
                }
            else
                {
                    LOG(INFO) << "Failed to save Galileo ionospheric model parameters, not valid data";
                }

            // save GPS almanac to XML file
            file_name = xml_base_path + "gps_almanac.xml";
            if (d_ls_pvt->gps_almanac_map.empty() == false)
                {
                    std::ofstream ofs;
                    try
                        {
                            ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                            boost::archive::xml_oarchive xml(ofs);
                            xml << boost::serialization::make_nvp("GNSS-SDR_gps_almanac_map", d_ls_pvt->gps_almanac_map);
                            LOG(INFO) << "Saved GPS almanac map data";
                        }
                    catch (const std::exception& e)
                        {
                            LOG(WARNING) << e.what();
                        }
                }
            else
                {
                    LOG(INFO) << "Failed to save GPS almanac, map is empty";
                }

            // Save Galileo almanac
            file_name = xml_base_path + "gal_almanac.xml";
            if (d_ls_pvt->galileo_almanac_map.empty() == false)
                {
                    std::ofstream ofs;
                    try
                        {
                            ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                            boost::archive::xml_oarchive xml(ofs);
                            xml << boost::serialization::make_nvp("GNSS-SDR_gal_almanac_map", d_ls_pvt->galileo_almanac_map);
                            LOG(INFO) << "Saved Galileo almanac data";
                        }
                    catch (std::exception& e)
                        {
                            LOG(WARNING) << e.what();
                        }
                }
            else
                {
                    LOG(INFO) << "Failed to save Galileo almanac, not valid data";
                }

            // Save GPS CNAV UTC model parameters
            file_name = xml_base_path + "gps_cnav_utc_model.xml";
            if (d_ls_pvt->gps_cnav_utc_model.valid)
                {
                    std::ofstream ofs;
                    try
                        {
                            ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                            boost::archive::xml_oarchive xml(ofs);
                            xml << boost::serialization::make_nvp("GNSS-SDR_cnav_utc_model", d_ls_pvt->gps_cnav_utc_model);
                            LOG(INFO) << "Saved GPS CNAV UTC model parameters";
                        }
                    catch (std::exception& e)
                        {
                            LOG(WARNING) << e.what();
                        }
                }
            else
                {
                    LOG(INFO) << "Failed to save GPS CNAV UTC model parameters, not valid data";
                }

            // save GLONASS GNAV ephemeris to XML file
            file_name = xml_base_path + "glo_gnav_ephemeris.xml";
            if (d_ls_pvt->glonass_gnav_ephemeris_map.empty() == false)
                {
                    std::ofstream ofs;
                    try
                        {
                            ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                            boost::archive::xml_oarchive xml(ofs);
                            xml << boost::serialization::make_nvp("GNSS-SDR_gnav_ephemeris_map", d_ls_pvt->glonass_gnav_ephemeris_map);
                            LOG(INFO) << "Saved GLONASS GNAV ephemeris map data";
                        }
                    catch (std::exception& e)
                        {
                            LOG(WARNING) << e.what();
                        }
                }
            else
                {
                    LOG(INFO) << "Failed to save GLONASS GNAV ephemeris, map is empty";
                }

            // save GLONASS UTC model parameters to XML file
            file_name = xml_base_path + "glo_utc_model.xml";
            if (d_ls_pvt->glonass_gnav_utc_model.valid)
                {
                    std::ofstream ofs;
                    try
                        {
                            ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                            boost::archive::xml_oarchive xml(ofs);
                            xml << boost::serialization::make_nvp("GNSS-SDR_gnav_utc_model", d_ls_pvt->glonass_gnav_utc_model);
                            LOG(INFO) << "Saved GLONASS UTC model parameters";
                        }
                    catch (std::exception& e)
                        {
                            LOG(WARNING) << e.what();
                        }
                }
            else
                {
                    LOG(INFO) << "Failed to save GLONASS GNAV ephemeris, not valid data";
                }
        }
}


bool rtklib_pvt_cc::observables_pairCompare_min(const std::pair<int, Gnss_Synchro>& a, const std::pair<int, Gnss_Synchro>& b)
{
    return (a.second.Pseudorange_m) < (b.second.Pseudorange_m);
}


bool rtklib_pvt_cc::send_sys_v_ttff_msg(ttff_msgbuf ttff)
{
    // Fill Sys V message structures
    int msgsend_size;
    ttff_msgbuf msg;
    msg.ttff = ttff.ttff;
    msgsend_size = sizeof(msg.ttff);
    msg.mtype = 1;  //  default message ID

    // SEND SOLUTION OVER A MESSAGE QUEUE
    // non-blocking Sys V message send
    msgsnd(sysv_msqid, &msg, msgsend_size, IPC_NOWAIT);
    return true;
}


bool rtklib_pvt_cc::save_gnss_synchro_map_xml(const std::string file_name)
{
    if (gnss_observables_map.empty() == false)
        {
            std::ofstream ofs;
            try
                {
                    ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                    boost::archive::xml_oarchive xml(ofs);
                    xml << boost::serialization::make_nvp("GNSS-SDR_gnss_synchro_map", gnss_observables_map);
                    LOG(INFO) << "Saved gnss_sychro map data";
                }
            catch (std::exception& e)
                {
                    LOG(WARNING) << e.what();
                    return false;
                }
            return true;
        }
    else
        {
            LOG(WARNING) << "Failed to save gnss_synchro, map is empty";
            return false;
        }
}


bool rtklib_pvt_cc::load_gnss_synchro_map_xml(const std::string file_name)
{
    // load from xml (boost serialize)
    std::ifstream ifs;
    try
        {
            ifs.open(file_name.c_str(), std::ifstream::binary | std::ifstream::in);
            boost::archive::xml_iarchive xml(ifs);
            gnss_observables_map.clear();
            xml >> boost::serialization::make_nvp("GNSS-SDR_gnss_synchro_map", gnss_observables_map);
            //std::cout << "Loaded gnss_synchro map data with " << gnss_synchro_map.size() << " pseudoranges" << std::endl;
        }
    catch (std::exception& e)
        {
            std::cout << e.what() << "File: " << file_name;
            return false;
        }
    return true;
}


int rtklib_pvt_cc::work(int noutput_items, gr_vector_const_void_star& input_items,
    gr_vector_void_star& output_items __attribute__((unused)))
{
    for (int32_t epoch = 0; epoch < noutput_items; epoch++)
        {
            bool flag_display_pvt = false;
            bool flag_compute_pvt_output = false;
            bool flag_write_RTCM_1019_output = false;
            bool flag_write_RTCM_1020_output = false;
            bool flag_write_RTCM_1045_output = false;
            bool flag_write_RTCM_MSM_output = false;
            bool flag_write_RINEX_obs_output = false;
            bool flag_write_RINEX_nav_output = false;

            gnss_observables_map.clear();
            const Gnss_Synchro** in = reinterpret_cast<const Gnss_Synchro**>(&input_items[0]);  // Get the input buffer pointer
            // ############ 1. READ PSEUDORANGES ####
            for (uint32_t i = 0; i < d_nchannels; i++)
                {
                    if (in[i][epoch].Flag_valid_pseudorange)
                        {
                            std::map<int, Gps_Ephemeris>::const_iterator tmp_eph_iter_gps = d_ls_pvt->gps_ephemeris_map.find(in[i][epoch].PRN);
                            std::map<int, Galileo_Ephemeris>::const_iterator tmp_eph_iter_gal = d_ls_pvt->galileo_ephemeris_map.find(in[i][epoch].PRN);
                            std::map<int, Gps_CNAV_Ephemeris>::const_iterator tmp_eph_iter_cnav = d_ls_pvt->gps_cnav_ephemeris_map.find(in[i][epoch].PRN);
                            std::map<int, Glonass_Gnav_Ephemeris>::const_iterator tmp_eph_iter_glo_gnav = d_ls_pvt->glonass_gnav_ephemeris_map.find(in[i][epoch].PRN);
                            if (((tmp_eph_iter_gps->second.i_satellite_PRN == in[i][epoch].PRN) and (std::string(in[i][epoch].Signal).compare("1C") == 0)) or ((tmp_eph_iter_cnav->second.i_satellite_PRN == in[i][epoch].PRN) and (std::string(in[i][epoch].Signal).compare("2S") == 0)) or ((tmp_eph_iter_gal->second.i_satellite_PRN == in[i][epoch].PRN) and (std::string(in[i][epoch].Signal).compare("1B") == 0)) or ((tmp_eph_iter_gal->second.i_satellite_PRN == in[i][epoch].PRN) and (std::string(in[i][epoch].Signal).compare("5X") == 0)) or ((tmp_eph_iter_glo_gnav->second.i_satellite_PRN == in[i][epoch].PRN) and (std::string(in[i][epoch].Signal).compare("1G") == 0)) or ((tmp_eph_iter_glo_gnav->second.i_satellite_PRN == in[i][epoch].PRN) and (std::string(in[i][epoch].Signal).compare("2G") == 0)) or ((tmp_eph_iter_cnav->second.i_satellite_PRN == in[i][epoch].PRN) and (std::string(in[i][epoch].Signal).compare("L5") == 0)))
                                {
                                    // store valid observables in a map.
                                    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(i, in[i][epoch]));
                                }
                            if (b_rtcm_enabled)
                                {
                                    try
                                        {
                                            if (d_ls_pvt->gps_ephemeris_map.empty() == false)
                                                {
                                                    if (tmp_eph_iter_gps != d_ls_pvt->gps_ephemeris_map.cend())
                                                        {
                                                            d_rtcm_printer->lock_time(d_ls_pvt->gps_ephemeris_map.find(in[i][epoch].PRN)->second, in[i][epoch].RX_time, in[i][epoch]);  // keep track of locking time
                                                        }
                                                }
                                            if (d_ls_pvt->galileo_ephemeris_map.empty() == false)
                                                {
                                                    if (tmp_eph_iter_gal != d_ls_pvt->galileo_ephemeris_map.cend())
                                                        {
                                                            d_rtcm_printer->lock_time(d_ls_pvt->galileo_ephemeris_map.find(in[i][epoch].PRN)->second, in[i][epoch].RX_time, in[i][epoch]);  // keep track of locking time
                                                        }
                                                }
                                            if (d_ls_pvt->gps_cnav_ephemeris_map.empty() == false)
                                                {
                                                    if (tmp_eph_iter_cnav != d_ls_pvt->gps_cnav_ephemeris_map.cend())
                                                        {
                                                            d_rtcm_printer->lock_time(d_ls_pvt->gps_cnav_ephemeris_map.find(in[i][epoch].PRN)->second, in[i][epoch].RX_time, in[i][epoch]);  // keep track of locking time
                                                        }
                                                }
                                            if (d_ls_pvt->glonass_gnav_ephemeris_map.empty() == false)
                                                {
                                                    if (tmp_eph_iter_glo_gnav != d_ls_pvt->glonass_gnav_ephemeris_map.cend())
                                                        {
                                                            d_rtcm_printer->lock_time(d_ls_pvt->glonass_gnav_ephemeris_map.find(in[i][epoch].PRN)->second, in[i][epoch].RX_time, in[i][epoch]);  // keep track of locking time
                                                        }
                                                }
                                        }
                                    catch (const boost::exception& ex)
                                        {
                                            std::cout << "RTCM boost exception: " << boost::diagnostic_information(ex) << std::endl;
                                            LOG(ERROR) << "RTCM boost exception: " << boost::diagnostic_information(ex);
                                        }
                                    catch (const std::exception& ex)
                                        {
                                            std::cout << "RTCM std exception: " << ex.what() << std::endl;
                                            LOG(ERROR) << "RTCM std exception: " << ex.what();
                                        }
                                }
                        }
                }

            // ############ 2 COMPUTE THE PVT ################################
            if (gnss_observables_map.empty() == false)
                {
                    double current_RX_time = gnss_observables_map.begin()->second.RX_time;
                    uint32_t current_RX_time_ms = static_cast<uint32_t>(current_RX_time * 1000.0);
                    if (current_RX_time_ms % d_output_rate_ms == 0)
                        {
                            flag_compute_pvt_output = true;
                            d_rx_time = current_RX_time;
                            // std::cout.precision(17);
                            // std::cout << "current_RX_time: " << current_RX_time << " map time: " << gnss_observables_map.begin()->second.RX_time << std::endl;
                        }

                    // compute on the fly PVT solution
                    if (flag_compute_pvt_output == true)
                        {
                            // receiver clock correction is disabled to be coherent with the RINEX and RTCM standard
                            // std::cout << TEXT_RED << "(internal) accumulated RX clock offset: " << d_ls_pvt->get_time_offset_s() << "[s]" << TEXT_RESET << std::endl;
                            // for (std::map<int, Gnss_Synchro>::iterator it = gnss_observables_map.begin(); it != gnss_observables_map.cend(); ++it)
                            //    {
                            // todo: check if it has effect to correct the receiver time for the internal pvt solution
                            // take into account that the RINEX obs with the RX time (integer ms) CAN NOT be corrected to keep the coherence in obs time
                            //        it->second.Pseudorange_m = it->second.Pseudorange_m - d_ls_pvt->get_time_offset_s() * GPS_C_m_s;
                            //    }

                            if (d_ls_pvt->get_PVT(gnss_observables_map, false))
                                {
                                    //Optional debug code: export observables snapshot for rtklib unit testing
                                    //std::cout << "step 1: save gnss_synchro map" << std::endl;
                                    //save_gnss_synchro_map_xml("./gnss_synchro_map.xml");
                                    //getchar(); //stop the execution
                                    //end debug

                                    if (current_RX_time_ms % d_display_rate_ms == 0)
                                        {
                                            flag_display_pvt = true;
                                        }
                                    if (d_rtcm_MT1019_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                        {
                                            if (current_RX_time_ms % d_rtcm_MT1019_rate_ms == 0)
                                                {
                                                    flag_write_RTCM_1019_output = true;
                                                }
                                        }
                                    if (d_rtcm_MT1020_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                        {
                                            if (current_RX_time_ms % d_rtcm_MT1020_rate_ms == 0)
                                                {
                                                    flag_write_RTCM_1020_output = true;
                                                }
                                        }
                                    if (d_rtcm_MT1045_rate_ms != 0)
                                        {
                                            if (current_RX_time_ms % d_rtcm_MT1045_rate_ms == 0)
                                                {
                                                    flag_write_RTCM_1045_output = true;
                                                }
                                        }
                                    // TODO: RTCM 1077, 1087 and 1097 are not used, so, disable the output rates
                                    // if (current_RX_time_ms % d_rtcm_MT1077_rate_ms==0 and d_rtcm_MT1077_rate_ms != 0)
                                    //     {
                                    //         last_RTCM_1077_output_time = current_RX_time;
                                    //     }
                                    // if (current_RX_time_ms % d_rtcm_MT1087_rate_ms==0 and d_rtcm_MT1087_rate_ms != 0)
                                    //     {
                                    //         last_RTCM_1087_output_time = current_RX_time;
                                    //     }
                                    // if (current_RX_time_ms % d_rtcm_MT1097_rate_ms==0 and d_rtcm_MT1097_rate_ms != 0)
                                    //     {
                                    //         last_RTCM_1097_output_time = current_RX_time;
                                    //     }
                                    if (d_rtcm_MSM_rate_ms != 0)
                                        {
                                            if (current_RX_time_ms % d_rtcm_MSM_rate_ms == 0)
                                                {
                                                    flag_write_RTCM_MSM_output = true;
                                                }
                                        }
                                    if (d_rinexobs_rate_ms != 0)
                                        {
                                            if (current_RX_time_ms % static_cast<uint32_t>(d_rinexobs_rate_ms) == 0)
                                                {
                                                    flag_write_RINEX_obs_output = true;
                                                }
                                        }
                                    if (d_rinexnav_rate_ms != 0)
                                        {
                                            if (current_RX_time_ms % static_cast<uint32_t>(d_rinexnav_rate_ms) == 0)
                                                {
                                                    flag_write_RINEX_nav_output = true;
                                                }
                                        }

                                    if (first_fix == true)
                                        {
                                            std::cout << "First position fix at " << boost::posix_time::to_simple_string(d_ls_pvt->get_position_UTC_time())
                                                      << " UTC is Lat = " << d_ls_pvt->get_latitude() << " [deg], Long = " << d_ls_pvt->get_longitude()
                                                      << " [deg], Height= " << d_ls_pvt->get_height() << " [m]" << std::endl;
                                            ttff_msgbuf ttff;
                                            ttff.mtype = 1;
                                            end = std::chrono::system_clock::now();
                                            std::chrono::duration<double> elapsed_seconds = end - start;
                                            ttff.ttff = elapsed_seconds.count();
                                            send_sys_v_ttff_msg(ttff);
                                            first_fix = false;
                                        }
                                    if (d_kml_output_enabled) d_kml_dump->print_position(d_ls_pvt, false);
                                    if (d_gpx_output_enabled) d_gpx_dump->print_position(d_ls_pvt, false);
                                    if (d_geojson_output_enabled) d_geojson_printer->print_position(d_ls_pvt, false);
                                    d_nmea_printer->Print_Nmea_Line(d_ls_pvt, false);

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
                                     *    23   |  GLONASS L1 C/A
                                     *    24   |  GLONASS L2 C/A
                                     *    25   |  GLONASS L1 C/A + GLONASS L2 C/A
                                     *    26   |  GPS L1 C/A + GLONASS L1 C/A
                                     *    27   |  Galileo E1B + GLONASS L1 C/A
                                     *    28   |  GPS L2C + GLONASS L1 C/A
                                     *    29   |  GPS L1 C/A + GLONASS L2 C/A
                                     *    30   |  Galileo E1B + GLONASS L2 C/A
                                     *    31   |  GPS L2C + GLONASS L2 C/A
                                     *    32   |  GPS L1 C/A + Galileo E1B + GPS L5 + Galileo E5a
                                     */

                                    // ####################### RINEX FILES #################
                                    if (b_rinex_output_enabled)
                                        {
                                            std::map<int, Galileo_Ephemeris>::const_iterator galileo_ephemeris_iter;
                                            std::map<int, Gps_Ephemeris>::const_iterator gps_ephemeris_iter;
                                            std::map<int, Gps_CNAV_Ephemeris>::const_iterator gps_cnav_ephemeris_iter;
                                            std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_ephemeris_iter;
                                            if (!b_rinex_header_written)  //  & we have utc data in nav message!
                                                {
                                                    galileo_ephemeris_iter = d_ls_pvt->galileo_ephemeris_map.cbegin();
                                                    gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.cbegin();
                                                    gps_cnav_ephemeris_iter = d_ls_pvt->gps_cnav_ephemeris_map.cbegin();
                                                    glonass_gnav_ephemeris_iter = d_ls_pvt->glonass_gnav_ephemeris_map.cbegin();

                                                    if (type_of_rx == 1)  // GPS L1 C/A only
                                                        {
                                                            if (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.cend())
                                                                {
                                                                    rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, d_rx_time);
                                                                    rp->rinex_nav_header(rp->navFile, d_ls_pvt->gps_iono, d_ls_pvt->gps_utc_model);
                                                                    b_rinex_header_written = true;  // do not write header anymore
                                                                }
                                                        }
                                                    if (type_of_rx == 2)  // GPS L2C only
                                                        {
                                                            if (gps_cnav_ephemeris_iter != d_ls_pvt->gps_cnav_ephemeris_map.cend())
                                                                {
                                                                    std::string signal("2S");
                                                                    rp->rinex_obs_header(rp->obsFile, gps_cnav_ephemeris_iter->second, d_rx_time, signal);
                                                                    rp->rinex_nav_header(rp->navFile, d_ls_pvt->gps_cnav_iono, d_ls_pvt->gps_cnav_utc_model);
                                                                    b_rinex_header_written = true;  // do not write header anymore
                                                                }
                                                        }
                                                    if (type_of_rx == 3)  // GPS L5 only
                                                        {
                                                            if (gps_cnav_ephemeris_iter != d_ls_pvt->gps_cnav_ephemeris_map.cend())
                                                                {
                                                                    std::string signal("L5");
                                                                    rp->rinex_obs_header(rp->obsFile, gps_cnav_ephemeris_iter->second, d_rx_time, signal);
                                                                    rp->rinex_nav_header(rp->navFile, d_ls_pvt->gps_cnav_iono, d_ls_pvt->gps_cnav_utc_model);
                                                                    b_rinex_header_written = true;  // do not write header anymore
                                                                }
                                                        }
                                                    if (type_of_rx == 4)  // Galileo E1B only
                                                        {
                                                            if (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                {
                                                                    rp->rinex_obs_header(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time);
                                                                    rp->rinex_nav_header(rp->navGalFile, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model);
                                                                    b_rinex_header_written = true;  // do not write header anymore
                                                                }
                                                        }
                                                    if (type_of_rx == 5)  // Galileo E5a only
                                                        {
                                                            std::string signal("5X");
                                                            if (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                {
                                                                    rp->rinex_obs_header(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, signal);
                                                                    rp->rinex_nav_header(rp->navGalFile, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model);
                                                                    b_rinex_header_written = true;  // do not write header anymore
                                                                }
                                                        }
                                                    if (type_of_rx == 6)  // Galileo E5b only
                                                        {
                                                            std::string signal("7X");
                                                            if (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                {
                                                                    rp->rinex_obs_header(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, signal);
                                                                    rp->rinex_nav_header(rp->navGalFile, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model);
                                                                    b_rinex_header_written = true;  // do not write header anymore
                                                                }
                                                        }
                                                    if (type_of_rx == 7)  // GPS L1 C/A + GPS L2C
                                                        {
                                                            if ((gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != d_ls_pvt->gps_cnav_ephemeris_map.cend()))
                                                                {
                                                                    std::string signal("1C 2S");
                                                                    rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, d_rx_time, signal);
                                                                    rp->rinex_nav_header(rp->navFile, d_ls_pvt->gps_iono, d_ls_pvt->gps_utc_model);
                                                                    b_rinex_header_written = true;  // do not write header anymore
                                                                }
                                                        }
                                                    if (type_of_rx == 8)  // GPS L1 + GPS L5
                                                        {
                                                            if ((gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != d_ls_pvt->gps_cnav_ephemeris_map.cend()))
                                                                {
                                                                    std::string signal("1C L5");
                                                                    rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, d_rx_time, signal);
                                                                    rp->rinex_nav_header(rp->navFile, d_ls_pvt->gps_iono, d_ls_pvt->gps_utc_model);
                                                                    b_rinex_header_written = true;  // do not write header anymore
                                                                }
                                                        }
                                                    if (type_of_rx == 9)  // GPS L1 C/A + Galileo E1B
                                                        {
                                                            if ((galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.cend()) and (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.cend()))
                                                                {
                                                                    std::string gal_signal("1B");
                                                                    rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, d_rx_time, gal_signal);
                                                                    rp->rinex_nav_header(rp->navMixFile, d_ls_pvt->gps_iono, d_ls_pvt->gps_utc_model, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model);
                                                                    b_rinex_header_written = true;  // do not write header anymore
                                                                }
                                                        }
                                                    if (type_of_rx == 10)  //  GPS L1 C/A + Galileo E5a
                                                        {
                                                            if ((galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.cend()) and (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.cend()))
                                                                {
                                                                    std::string gal_signal("5X");
                                                                    rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, d_rx_time, gal_signal);
                                                                    rp->rinex_nav_header(rp->navMixFile, d_ls_pvt->gps_iono, d_ls_pvt->gps_utc_model, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model);
                                                                    b_rinex_header_written = true;  // do not write header anymore
                                                                }
                                                        }
                                                    if (type_of_rx == 11)  //  GPS L1 C/A + Galileo E5b
                                                        {
                                                            if ((galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.cend()) and (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.cend()))
                                                                {
                                                                    std::string gal_signal("7X");
                                                                    rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, d_rx_time, gal_signal);
                                                                    rp->rinex_nav_header(rp->navMixFile, d_ls_pvt->gps_iono, d_ls_pvt->gps_utc_model, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model);
                                                                    b_rinex_header_written = true;  // do not write header anymore
                                                                }
                                                        }
                                                    if (type_of_rx == 13)  // L5+E5a
                                                        {
                                                            if ((galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != d_ls_pvt->gps_cnav_ephemeris_map.cend()))
                                                                {
                                                                    std::string gal_signal("5X");
                                                                    std::string gps_signal("L5");
                                                                    rp->rinex_obs_header(rp->obsFile, gps_cnav_ephemeris_iter->second, galileo_ephemeris_iter->second, d_rx_time, gps_signal, gal_signal);
                                                                    rp->rinex_nav_header(rp->navMixFile, d_ls_pvt->gps_cnav_iono, d_ls_pvt->gps_cnav_utc_model, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model);
                                                                    b_rinex_header_written = true;  // do not write header anymore
                                                                }
                                                        }
                                                    if (type_of_rx == 14)  //  Galileo E1B + Galileo E5a
                                                        {
                                                            if ((galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.cend()))
                                                                {
                                                                    std::string gal_signal("1B 5X");
                                                                    rp->rinex_obs_header(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, gal_signal);
                                                                    rp->rinex_nav_header(rp->navGalFile, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model);
                                                                    b_rinex_header_written = true;  // do not write header anymore
                                                                }
                                                        }
                                                    if (type_of_rx == 15)  //  Galileo E1B + Galileo E5b
                                                        {
                                                            if ((galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.cend()))
                                                                {
                                                                    std::string gal_signal("1B 7X");
                                                                    rp->rinex_obs_header(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, gal_signal);
                                                                    rp->rinex_nav_header(rp->navGalFile, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model);
                                                                    b_rinex_header_written = true;  // do not write header anymore
                                                                }
                                                        }
                                                    if (type_of_rx == 23)  // GLONASS L1 C/A only
                                                        {
                                                            std::string signal("1G");
                                                            if (glonass_gnav_ephemeris_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend())
                                                                {
                                                                    rp->rinex_obs_header(rp->obsFile, glonass_gnav_ephemeris_iter->second, d_rx_time, signal);
                                                                    rp->rinex_nav_header(rp->navGloFile, d_ls_pvt->glonass_gnav_utc_model, glonass_gnav_ephemeris_iter->second);
                                                                    b_rinex_header_written = true;  // do not write header anymore
                                                                }
                                                        }
                                                    if (type_of_rx == 24)  // GLONASS L2 C/A only
                                                        {
                                                            std::string signal("2G");
                                                            if (glonass_gnav_ephemeris_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend())
                                                                {
                                                                    rp->rinex_obs_header(rp->obsFile, glonass_gnav_ephemeris_iter->second, d_rx_time, signal);
                                                                    rp->rinex_nav_header(rp->navGloFile, d_ls_pvt->glonass_gnav_utc_model, glonass_gnav_ephemeris_iter->second);
                                                                    b_rinex_header_written = true;  // do not write header anymore
                                                                }
                                                        }
                                                    if (type_of_rx == 25)  // GLONASS L1 C/A + GLONASS L2 C/A
                                                        {
                                                            std::string signal("1G 2G");
                                                            if (glonass_gnav_ephemeris_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend())
                                                                {
                                                                    rp->rinex_obs_header(rp->obsFile, glonass_gnav_ephemeris_iter->second, d_rx_time, signal);
                                                                    rp->rinex_nav_header(rp->navGloFile, d_ls_pvt->glonass_gnav_utc_model, glonass_gnav_ephemeris_iter->second);
                                                                    b_rinex_header_written = true;  // do not write header anymore
                                                                }
                                                        }

                                                    if (type_of_rx == 26)  // GPS L1 C/A + GLONASS L1 C/A
                                                        {
                                                            if ((glonass_gnav_ephemeris_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend()) and (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.cend()))
                                                                {
                                                                    std::string glo_signal("1G");
                                                                    rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, d_rx_time, glo_signal);
                                                                    if (d_rinex_version == 3)
                                                                        rp->rinex_nav_header(rp->navMixFile, d_ls_pvt->gps_iono, d_ls_pvt->gps_utc_model, d_ls_pvt->glonass_gnav_utc_model, d_ls_pvt->glonass_gnav_almanac);
                                                                    if (d_rinex_version == 2)
                                                                        {
                                                                            rp->rinex_nav_header(rp->navFile, d_ls_pvt->gps_iono, d_ls_pvt->gps_utc_model);
                                                                            rp->rinex_nav_header(rp->navGloFile, d_ls_pvt->glonass_gnav_utc_model, glonass_gnav_ephemeris_iter->second);
                                                                        }
                                                                    b_rinex_header_written = true;  // do not write header anymore
                                                                }
                                                        }
                                                    if (type_of_rx == 27)  //  Galileo E1B + GLONASS L1 C/A
                                                        {
                                                            if ((glonass_gnav_ephemeris_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend()) and (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.cend()))
                                                                {
                                                                    std::string glo_signal("1G");
                                                                    std::string gal_signal("1B");
                                                                    rp->rinex_obs_header(rp->obsFile, galileo_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, d_rx_time, glo_signal, gal_signal);
                                                                    rp->rinex_nav_header(rp->navMixFile, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model, d_ls_pvt->glonass_gnav_utc_model, d_ls_pvt->glonass_gnav_almanac);
                                                                    b_rinex_header_written = true;  // do not write header anymore
                                                                }
                                                        }
                                                    if (type_of_rx == 28)  // GPS L2C + GLONASS L1 C/A
                                                        {
                                                            if ((glonass_gnav_ephemeris_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != d_ls_pvt->gps_cnav_ephemeris_map.cend()))
                                                                {
                                                                    std::string glo_signal("1G");
                                                                    rp->rinex_obs_header(rp->obsFile, gps_cnav_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, d_rx_time, glo_signal);
                                                                    rp->rinex_nav_header(rp->navMixFile, d_ls_pvt->gps_cnav_iono, d_ls_pvt->gps_cnav_utc_model, d_ls_pvt->glonass_gnav_utc_model, d_ls_pvt->glonass_gnav_almanac);
                                                                    b_rinex_header_written = true;  // do not write header anymore
                                                                }
                                                        }
                                                    if (type_of_rx == 29)  // GPS L1 C/A + GLONASS L2 C/A
                                                        {
                                                            if ((glonass_gnav_ephemeris_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend()) and (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.cend()))
                                                                {
                                                                    std::string glo_signal("2G");
                                                                    rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, d_rx_time, glo_signal);
                                                                    if (d_rinex_version == 3)
                                                                        rp->rinex_nav_header(rp->navMixFile, d_ls_pvt->gps_iono, d_ls_pvt->gps_utc_model, d_ls_pvt->glonass_gnav_utc_model, d_ls_pvt->glonass_gnav_almanac);
                                                                    if (d_rinex_version == 2)
                                                                        {
                                                                            rp->rinex_nav_header(rp->navFile, d_ls_pvt->gps_iono, d_ls_pvt->gps_utc_model);
                                                                            rp->rinex_nav_header(rp->navGloFile, d_ls_pvt->glonass_gnav_utc_model, glonass_gnav_ephemeris_iter->second);
                                                                        }
                                                                    b_rinex_header_written = true;  // do not write header anymore
                                                                }
                                                        }
                                                    if (type_of_rx == 30)  //  Galileo E1B + GLONASS L2 C/A
                                                        {
                                                            if ((glonass_gnav_ephemeris_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend()) and (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.cend()))
                                                                {
                                                                    std::string glo_signal("2G");
                                                                    std::string gal_signal("1B");
                                                                    rp->rinex_obs_header(rp->obsFile, galileo_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, d_rx_time, glo_signal, gal_signal);
                                                                    rp->rinex_nav_header(rp->navMixFile, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model, d_ls_pvt->glonass_gnav_utc_model, d_ls_pvt->glonass_gnav_almanac);
                                                                    b_rinex_header_written = true;  // do not write header anymore
                                                                }
                                                        }
                                                    if (type_of_rx == 31)  // GPS L2C + GLONASS L2 C/A
                                                        {
                                                            if ((glonass_gnav_ephemeris_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != d_ls_pvt->gps_cnav_ephemeris_map.cend()))
                                                                {
                                                                    std::string glo_signal("2G");
                                                                    rp->rinex_obs_header(rp->obsFile, gps_cnav_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, d_rx_time, glo_signal);
                                                                    rp->rinex_nav_header(rp->navMixFile, d_ls_pvt->gps_cnav_iono, d_ls_pvt->gps_cnav_utc_model, d_ls_pvt->glonass_gnav_utc_model, d_ls_pvt->glonass_gnav_almanac);
                                                                    b_rinex_header_written = true;  // do not write header anymore
                                                                }
                                                        }

                                                    if (type_of_rx == 32)  // L1+E1+L5+E5a
                                                        {
                                                            if ((gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.cend()) and
                                                                (gps_cnav_ephemeris_iter != d_ls_pvt->gps_cnav_ephemeris_map.cend()) and
                                                                (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.cend()))
                                                                {
                                                                    std::string gal_signal("1B 5X");
                                                                    std::string gps_signal("1C L5");
                                                                    rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, galileo_ephemeris_iter->second, d_rx_time, gps_signal, gal_signal);
                                                                    rp->rinex_nav_header(rp->navMixFile, d_ls_pvt->gps_iono, d_ls_pvt->gps_utc_model, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model);
                                                                    b_rinex_header_written = true;  // do not write header anymore
                                                                }
                                                        }
                                                }

                                            if (b_rinex_header_written)  // The header is already written, we can now log the navigation message data
                                                {
                                                    if (flag_write_RINEX_nav_output)
                                                        {
                                                            if (type_of_rx == 1)  // GPS L1 C/A only
                                                                {
                                                                    rp->log_rinex_nav(rp->navFile, d_ls_pvt->gps_ephemeris_map);
                                                                }
                                                            if (type_of_rx == 2)  // GPS L2C only
                                                                {
                                                                    rp->log_rinex_nav(rp->navFile, d_ls_pvt->gps_cnav_ephemeris_map);
                                                                }
                                                            if (type_of_rx == 3)  // GPS L5 only
                                                                {
                                                                    rp->log_rinex_nav(rp->navFile, d_ls_pvt->gps_cnav_ephemeris_map);
                                                                }
                                                            if ((type_of_rx == 4) or (type_of_rx == 5) or (type_of_rx == 6))  // Galileo
                                                                {
                                                                    rp->log_rinex_nav(rp->navGalFile, d_ls_pvt->galileo_ephemeris_map);
                                                                }
                                                            if (type_of_rx == 7)  // GPS L1 C/A + GPS L2C
                                                                {
                                                                    rp->log_rinex_nav(rp->navFile, d_ls_pvt->gps_cnav_ephemeris_map);
                                                                }
                                                            if (type_of_rx == 8)  // L1+L5
                                                                {
                                                                    rp->log_rinex_nav(rp->navFile, d_ls_pvt->gps_ephemeris_map);
                                                                }
                                                            if ((type_of_rx == 9) or (type_of_rx == 10) or (type_of_rx == 11))  // GPS L1 C/A + Galileo
                                                                {
                                                                    rp->log_rinex_nav(rp->navMixFile, d_ls_pvt->gps_ephemeris_map, d_ls_pvt->galileo_ephemeris_map);
                                                                }
                                                            if (type_of_rx == 13)  //  L5+E5a
                                                                {
                                                                    rp->log_rinex_nav(rp->navFile, d_ls_pvt->gps_cnav_ephemeris_map, d_ls_pvt->galileo_ephemeris_map);
                                                                }
                                                            if ((type_of_rx == 14) or (type_of_rx == 15))  //  Galileo E1B + Galileo E5a
                                                                {
                                                                    rp->log_rinex_nav(rp->navGalFile, d_ls_pvt->galileo_ephemeris_map);
                                                                }
                                                            if ((type_of_rx == 23) or (type_of_rx == 24) or (type_of_rx == 25))  //  GLONASS L1 C/A, GLONASS L2 C/A
                                                                {
                                                                    rp->log_rinex_nav(rp->navGloFile, d_ls_pvt->glonass_gnav_ephemeris_map);
                                                                }
                                                            if (type_of_rx == 26)  //  GPS L1 C/A + GLONASS L1 C/A
                                                                {
                                                                    if (d_rinex_version == 3)
                                                                        rp->log_rinex_nav(rp->navMixFile, d_ls_pvt->gps_ephemeris_map, d_ls_pvt->glonass_gnav_ephemeris_map);
                                                                    if (d_rinex_version == 2)
                                                                        {
                                                                            rp->log_rinex_nav(rp->navFile, d_ls_pvt->gps_ephemeris_map);
                                                                            rp->log_rinex_nav(rp->navGloFile, d_ls_pvt->glonass_gnav_ephemeris_map);
                                                                        }
                                                                }
                                                            if (type_of_rx == 27)  //  Galileo E1B + GLONASS L1 C/A
                                                                {
                                                                    rp->log_rinex_nav(rp->navMixFile, d_ls_pvt->galileo_ephemeris_map, d_ls_pvt->glonass_gnav_ephemeris_map);
                                                                }
                                                            if (type_of_rx == 28)  //  GPS L2C + GLONASS L1 C/A
                                                                {
                                                                    rp->log_rinex_nav(rp->navMixFile, d_ls_pvt->gps_cnav_ephemeris_map, d_ls_pvt->glonass_gnav_ephemeris_map);
                                                                }
                                                            if (type_of_rx == 29)  //  GPS L1 C/A + GLONASS L2 C/A
                                                                {
                                                                    if (d_rinex_version == 3)
                                                                        rp->log_rinex_nav(rp->navMixFile, d_ls_pvt->gps_ephemeris_map, d_ls_pvt->glonass_gnav_ephemeris_map);
                                                                    if (d_rinex_version == 2)
                                                                        {
                                                                            rp->log_rinex_nav(rp->navFile, d_ls_pvt->gps_ephemeris_map);
                                                                            rp->log_rinex_nav(rp->navGloFile, d_ls_pvt->glonass_gnav_ephemeris_map);
                                                                        }
                                                                }
                                                            if (type_of_rx == 30)  //  Galileo E1B + GLONASS L2 C/A
                                                                {
                                                                    rp->log_rinex_nav(rp->navMixFile, d_ls_pvt->galileo_ephemeris_map, d_ls_pvt->glonass_gnav_ephemeris_map);
                                                                }
                                                            if (type_of_rx == 31)  //  GPS L2C + GLONASS L2 C/A
                                                                {
                                                                    rp->log_rinex_nav(rp->navMixFile, d_ls_pvt->gps_cnav_ephemeris_map, d_ls_pvt->glonass_gnav_ephemeris_map);
                                                                }
                                                            if (type_of_rx == 32)  // L1+E1+L5+E5a
                                                                {
                                                                    rp->log_rinex_nav(rp->navMixFile, d_ls_pvt->gps_ephemeris_map, d_ls_pvt->galileo_ephemeris_map);
                                                                }
                                                        }
                                                    galileo_ephemeris_iter = d_ls_pvt->galileo_ephemeris_map.cbegin();
                                                    gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.cbegin();
                                                    gps_cnav_ephemeris_iter = d_ls_pvt->gps_cnav_ephemeris_map.cbegin();
                                                    glonass_gnav_ephemeris_iter = d_ls_pvt->glonass_gnav_ephemeris_map.cbegin();

                                                    // Log observables into the RINEX file
                                                    if (flag_write_RINEX_obs_output)
                                                        {
                                                            if (type_of_rx == 1)  // GPS L1 C/A only
                                                                {
                                                                    if (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.cend())
                                                                        {
                                                                            rp->log_rinex_obs(rp->obsFile, gps_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                        }
                                                                    if (!b_rinex_header_updated and (d_ls_pvt->gps_utc_model.d_A0 != 0))
                                                                        {
                                                                            rp->update_obs_header(rp->obsFile, d_ls_pvt->gps_utc_model);
                                                                            rp->update_nav_header(rp->navFile, d_ls_pvt->gps_utc_model, d_ls_pvt->gps_iono);
                                                                            b_rinex_header_updated = true;
                                                                        }
                                                                }
                                                            if (type_of_rx == 2)  // GPS L2C only
                                                                {
                                                                    if (gps_cnav_ephemeris_iter != d_ls_pvt->gps_cnav_ephemeris_map.cend())
                                                                        {
                                                                            rp->log_rinex_obs(rp->obsFile, gps_cnav_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                        }
                                                                    if (!b_rinex_header_updated and (d_ls_pvt->gps_cnav_utc_model.d_A0 != 0))
                                                                        {
                                                                            rp->update_obs_header(rp->obsFile, d_ls_pvt->gps_cnav_utc_model);
                                                                            rp->update_nav_header(rp->navFile, d_ls_pvt->gps_cnav_utc_model, d_ls_pvt->gps_cnav_iono);
                                                                            b_rinex_header_updated = true;
                                                                        }
                                                                }
                                                            if (type_of_rx == 3)  // GPS L5
                                                                {
                                                                    if (gps_cnav_ephemeris_iter != d_ls_pvt->gps_cnav_ephemeris_map.cend())
                                                                        {
                                                                            rp->log_rinex_obs(rp->obsFile, gps_cnav_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                        }
                                                                    if (!b_rinex_header_updated and (d_ls_pvt->gps_cnav_utc_model.d_A0 != 0))
                                                                        {
                                                                            rp->update_obs_header(rp->obsFile, d_ls_pvt->gps_cnav_utc_model);
                                                                            rp->update_nav_header(rp->navFile, d_ls_pvt->gps_cnav_utc_model, d_ls_pvt->gps_cnav_iono);
                                                                            b_rinex_header_updated = true;
                                                                        }
                                                                }
                                                            if (type_of_rx == 4)  // Galileo E1B only
                                                                {
                                                                    if (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                        {
                                                                            rp->log_rinex_obs(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, gnss_observables_map, "1B");
                                                                        }
                                                                    if (!b_rinex_header_updated and (d_ls_pvt->galileo_utc_model.A0_6 != 0))
                                                                        {
                                                                            rp->update_nav_header(rp->navGalFile, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model);
                                                                            rp->update_obs_header(rp->obsFile, d_ls_pvt->galileo_utc_model);
                                                                            b_rinex_header_updated = true;
                                                                        }
                                                                }
                                                            if (type_of_rx == 5)  // Galileo E5a only
                                                                {
                                                                    if (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                        {
                                                                            rp->log_rinex_obs(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, gnss_observables_map, "5X");
                                                                        }
                                                                    if (!b_rinex_header_updated and (d_ls_pvt->galileo_utc_model.A0_6 != 0))
                                                                        {
                                                                            rp->update_nav_header(rp->navGalFile, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model);
                                                                            rp->update_obs_header(rp->obsFile, d_ls_pvt->galileo_utc_model);
                                                                            b_rinex_header_updated = true;
                                                                        }
                                                                }
                                                            if (type_of_rx == 6)  // Galileo E5b only
                                                                {
                                                                    if (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                        {
                                                                            rp->log_rinex_obs(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, gnss_observables_map, "7X");
                                                                        }
                                                                    if (!b_rinex_header_updated and (d_ls_pvt->galileo_utc_model.A0_6 != 0))
                                                                        {
                                                                            rp->update_nav_header(rp->navGalFile, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model);
                                                                            rp->update_obs_header(rp->obsFile, d_ls_pvt->galileo_utc_model);
                                                                            b_rinex_header_updated = true;
                                                                        }
                                                                }
                                                            if (type_of_rx == 7)  // GPS L1 C/A + GPS L2C
                                                                {
                                                                    if ((gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != d_ls_pvt->gps_cnav_ephemeris_map.cend()))
                                                                        {
                                                                            rp->log_rinex_obs(rp->obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                        }
                                                                    if (!b_rinex_header_updated and (d_ls_pvt->gps_utc_model.d_A0 != 0))
                                                                        {
                                                                            rp->update_obs_header(rp->obsFile, d_ls_pvt->gps_utc_model);
                                                                            rp->update_nav_header(rp->navFile, d_ls_pvt->gps_utc_model, d_ls_pvt->gps_iono);
                                                                            b_rinex_header_updated = true;
                                                                        }
                                                                }
                                                            if (type_of_rx == 8)  // L1+L5
                                                                {
                                                                    if ((gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != d_ls_pvt->gps_cnav_ephemeris_map.cend()))
                                                                        {
                                                                            rp->log_rinex_obs(rp->obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                        }
                                                                    if (!b_rinex_header_updated and ((d_ls_pvt->gps_cnav_utc_model.d_A0 != 0) or (d_ls_pvt->gps_utc_model.d_A0 != 0)))
                                                                        {
                                                                            if (d_ls_pvt->gps_cnav_utc_model.d_A0 != 0)
                                                                                {
                                                                                    rp->update_obs_header(rp->obsFile, d_ls_pvt->gps_cnav_utc_model);
                                                                                    rp->update_nav_header(rp->navFile, d_ls_pvt->gps_cnav_utc_model, d_ls_pvt->gps_cnav_iono);
                                                                                }
                                                                            else
                                                                                {
                                                                                    rp->update_obs_header(rp->obsFile, d_ls_pvt->gps_utc_model);
                                                                                    rp->update_nav_header(rp->navFile, d_ls_pvt->gps_utc_model, d_ls_pvt->gps_iono);
                                                                                }
                                                                            b_rinex_header_updated = true;
                                                                        }
                                                                }
                                                            if (type_of_rx == 9)  // GPS L1 C/A + Galileo E1B
                                                                {
                                                                    if ((galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.cend()) and (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.cend()))
                                                                        {
                                                                            rp->log_rinex_obs(rp->obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                        }
                                                                    if (!b_rinex_header_updated and (d_ls_pvt->gps_utc_model.d_A0 != 0))
                                                                        {
                                                                            rp->update_obs_header(rp->obsFile, d_ls_pvt->gps_utc_model);
                                                                            rp->update_nav_header(rp->navMixFile, d_ls_pvt->gps_iono, d_ls_pvt->gps_utc_model, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model);
                                                                            b_rinex_header_updated = true;
                                                                        }
                                                                }
                                                            if (type_of_rx == 13)  // L5+E5a
                                                                {
                                                                    if ((gps_cnav_ephemeris_iter != d_ls_pvt->gps_cnav_ephemeris_map.cend()) and (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.cend()))
                                                                        {
                                                                            rp->log_rinex_obs(rp->obsFile, gps_cnav_ephemeris_iter->second, galileo_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                        }
                                                                    if (!b_rinex_header_updated and (d_ls_pvt->gps_cnav_utc_model.d_A0 != 0) and (d_ls_pvt->galileo_utc_model.A0_6 != 0))
                                                                        {
                                                                            rp->update_obs_header(rp->obsFile, d_ls_pvt->gps_cnav_utc_model);
                                                                            rp->update_nav_header(rp->navMixFile, d_ls_pvt->gps_cnav_utc_model, d_ls_pvt->gps_cnav_iono, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model);
                                                                            b_rinex_header_updated = true;  // do not write header anymore
                                                                        }
                                                                }
                                                            if (type_of_rx == 14)  // Galileo E1B + Galileo E5a
                                                                {
                                                                    if (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                        {
                                                                            rp->log_rinex_obs(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, gnss_observables_map, "1B 5X");
                                                                        }
                                                                    if (!b_rinex_header_updated and (d_ls_pvt->galileo_utc_model.A0_6 != 0))
                                                                        {
                                                                            rp->update_nav_header(rp->navGalFile, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model);
                                                                            rp->update_obs_header(rp->obsFile, d_ls_pvt->galileo_utc_model);
                                                                            b_rinex_header_updated = true;
                                                                        }
                                                                }
                                                            if (type_of_rx == 15)  // Galileo E1B + Galileo E5b
                                                                {
                                                                    if (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                        {
                                                                            rp->log_rinex_obs(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, gnss_observables_map, "1B 7X");
                                                                        }
                                                                    if (!b_rinex_header_updated and (d_ls_pvt->galileo_utc_model.A0_6 != 0))
                                                                        {
                                                                            rp->update_nav_header(rp->navGalFile, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model);
                                                                            rp->update_obs_header(rp->obsFile, d_ls_pvt->galileo_utc_model);
                                                                            b_rinex_header_updated = true;
                                                                        }
                                                                }
                                                            if (type_of_rx == 23)  // GLONASS L1 C/A only
                                                                {
                                                                    if (glonass_gnav_ephemeris_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend())
                                                                        {
                                                                            rp->log_rinex_obs(rp->obsFile, glonass_gnav_ephemeris_iter->second, d_rx_time, gnss_observables_map, "1C");
                                                                        }
                                                                    if (!b_rinex_header_updated and (d_ls_pvt->glonass_gnav_utc_model.d_tau_c != 0))
                                                                        {
                                                                            rp->update_nav_header(rp->navGloFile, d_ls_pvt->glonass_gnav_utc_model, d_ls_pvt->glonass_gnav_almanac);
                                                                            rp->update_obs_header(rp->obsFile, d_ls_pvt->glonass_gnav_utc_model);
                                                                            b_rinex_header_updated = true;
                                                                        }
                                                                }
                                                            if (type_of_rx == 24)  // GLONASS L2 C/A only
                                                                {
                                                                    if (glonass_gnav_ephemeris_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend())
                                                                        {
                                                                            rp->log_rinex_obs(rp->obsFile, glonass_gnav_ephemeris_iter->second, d_rx_time, gnss_observables_map, "2C");
                                                                        }
                                                                    if (!b_rinex_header_updated and (d_ls_pvt->glonass_gnav_utc_model.d_tau_c != 0))
                                                                        {
                                                                            rp->update_nav_header(rp->navGloFile, d_ls_pvt->glonass_gnav_utc_model, d_ls_pvt->glonass_gnav_almanac);
                                                                            rp->update_obs_header(rp->obsFile, d_ls_pvt->glonass_gnav_utc_model);
                                                                            b_rinex_header_updated = true;
                                                                        }
                                                                }
                                                            if (type_of_rx == 25)  // GLONASS L1 C/A + GLONASS L2 C/A
                                                                {
                                                                    if (glonass_gnav_ephemeris_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend())
                                                                        {
                                                                            rp->log_rinex_obs(rp->obsFile, glonass_gnav_ephemeris_iter->second, d_rx_time, gnss_observables_map, "1C 2C");
                                                                        }
                                                                    if (!b_rinex_header_updated and (d_ls_pvt->glonass_gnav_utc_model.d_tau_c != 0))
                                                                        {
                                                                            rp->update_nav_header(rp->navMixFile, d_ls_pvt->glonass_gnav_utc_model, d_ls_pvt->glonass_gnav_almanac);
                                                                            rp->update_obs_header(rp->obsFile, d_ls_pvt->glonass_gnav_utc_model);
                                                                            b_rinex_header_updated = true;
                                                                        }
                                                                }
                                                            if (type_of_rx == 26)  // GPS L1 C/A + GLONASS L1 C/A
                                                                {
                                                                    if ((glonass_gnav_ephemeris_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend()) and (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.cend()))
                                                                        {
                                                                            rp->log_rinex_obs(rp->obsFile, gps_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                        }
                                                                    if (!b_rinex_header_updated and (d_ls_pvt->gps_utc_model.d_A0 != 0))
                                                                        {
                                                                            rp->update_obs_header(rp->obsFile, d_ls_pvt->gps_utc_model);
                                                                            rp->update_nav_header(rp->navMixFile, d_ls_pvt->gps_iono, d_ls_pvt->gps_utc_model, d_ls_pvt->glonass_gnav_utc_model, d_ls_pvt->glonass_gnav_almanac);
                                                                            b_rinex_header_updated = true;  // do not write header anymore
                                                                        }
                                                                }
                                                            if (type_of_rx == 27)  // Galileo E1B + GLONASS L1 C/A
                                                                {
                                                                    if ((glonass_gnav_ephemeris_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend()) and (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.cend()))
                                                                        {
                                                                            rp->log_rinex_obs(rp->obsFile, galileo_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                        }
                                                                    if (!b_rinex_header_updated and (d_ls_pvt->galileo_utc_model.A0_6 != 0))
                                                                        {
                                                                            rp->update_obs_header(rp->obsFile, d_ls_pvt->galileo_utc_model);
                                                                            rp->update_nav_header(rp->navMixFile, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model, d_ls_pvt->glonass_gnav_utc_model, d_ls_pvt->glonass_gnav_almanac);
                                                                            b_rinex_header_updated = true;  // do not write header anymore
                                                                        }
                                                                }
                                                            if (type_of_rx == 28)  // GPS L2C + GLONASS L1 C/A
                                                                {
                                                                    if ((glonass_gnav_ephemeris_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != d_ls_pvt->gps_cnav_ephemeris_map.cend()))
                                                                        {
                                                                            rp->log_rinex_obs(rp->obsFile, gps_cnav_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                        }
                                                                    if (!b_rinex_header_updated and (d_ls_pvt->gps_cnav_utc_model.d_A0 != 0))
                                                                        {
                                                                            rp->update_obs_header(rp->obsFile, d_ls_pvt->gps_cnav_utc_model);
                                                                            rp->update_nav_header(rp->navMixFile, d_ls_pvt->gps_cnav_iono, d_ls_pvt->gps_cnav_utc_model, d_ls_pvt->glonass_gnav_utc_model, d_ls_pvt->glonass_gnav_almanac);
                                                                            b_rinex_header_updated = true;  // do not write header anymore
                                                                        }
                                                                }
                                                            if (type_of_rx == 29)  // GPS L1 C/A + GLONASS L2 C/A
                                                                {
                                                                    if ((glonass_gnav_ephemeris_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend()) and (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.cend()))
                                                                        {
                                                                            rp->log_rinex_obs(rp->obsFile, gps_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                        }
                                                                    if (!b_rinex_header_updated and (d_ls_pvt->gps_utc_model.d_A0 != 0))
                                                                        {
                                                                            rp->update_obs_header(rp->obsFile, d_ls_pvt->gps_utc_model);
                                                                            rp->update_nav_header(rp->navMixFile, d_ls_pvt->gps_iono, d_ls_pvt->gps_utc_model, d_ls_pvt->glonass_gnav_utc_model, d_ls_pvt->glonass_gnav_almanac);
                                                                            b_rinex_header_updated = true;  // do not write header anymore
                                                                        }
                                                                }
                                                            if (type_of_rx == 30)  // Galileo E1B + GLONASS L2 C/A
                                                                {
                                                                    if ((glonass_gnav_ephemeris_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend()) and (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.cend()))
                                                                        {
                                                                            rp->log_rinex_obs(rp->obsFile, galileo_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                        }
                                                                    if (!b_rinex_header_updated and (d_ls_pvt->galileo_utc_model.A0_6 != 0))
                                                                        {
                                                                            rp->update_obs_header(rp->obsFile, d_ls_pvt->galileo_utc_model);
                                                                            rp->update_nav_header(rp->navMixFile, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model, d_ls_pvt->glonass_gnav_utc_model, d_ls_pvt->glonass_gnav_almanac);
                                                                            b_rinex_header_updated = true;  // do not write header anymore
                                                                        }
                                                                }
                                                            if (type_of_rx == 31)  // GPS L2C + GLONASS L2 C/A
                                                                {
                                                                    if ((glonass_gnav_ephemeris_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != d_ls_pvt->gps_cnav_ephemeris_map.cend()))
                                                                        {
                                                                            rp->log_rinex_obs(rp->obsFile, gps_cnav_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                        }
                                                                    if (!b_rinex_header_updated and (d_ls_pvt->gps_cnav_utc_model.d_A0 != 0))
                                                                        {
                                                                            rp->update_obs_header(rp->obsFile, d_ls_pvt->gps_cnav_utc_model);
                                                                            rp->update_nav_header(rp->navMixFile, d_ls_pvt->gps_cnav_iono, d_ls_pvt->gps_cnav_utc_model, d_ls_pvt->glonass_gnav_utc_model, d_ls_pvt->glonass_gnav_almanac);
                                                                            b_rinex_header_updated = true;  // do not write header anymore
                                                                        }
                                                                }
                                                            if (type_of_rx == 32)  // L1+E1+L5+E5a
                                                                {
                                                                    if ((gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != d_ls_pvt->gps_cnav_ephemeris_map.cend()) and (galileo_ephemeris_iter != d_ls_pvt->galileo_ephemeris_map.cend()))
                                                                        {
                                                                            rp->log_rinex_obs(rp->obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, galileo_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                        }
                                                                    if (!b_rinex_header_updated and ((d_ls_pvt->gps_cnav_utc_model.d_A0 != 0) or (d_ls_pvt->gps_utc_model.d_A0 != 0)) and (d_ls_pvt->galileo_utc_model.A0_6 != 0))
                                                                        {
                                                                            if (d_ls_pvt->gps_cnav_utc_model.d_A0 != 0)
                                                                                {
                                                                                    rp->update_obs_header(rp->obsFile, d_ls_pvt->gps_cnav_utc_model);
                                                                                    rp->update_nav_header(rp->navMixFile, d_ls_pvt->gps_cnav_utc_model, d_ls_pvt->gps_cnav_iono, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model);
                                                                                }
                                                                            else
                                                                                {
                                                                                    rp->update_obs_header(rp->obsFile, d_ls_pvt->gps_utc_model);
                                                                                    rp->update_nav_header(rp->navMixFile, d_ls_pvt->gps_iono, d_ls_pvt->gps_utc_model, d_ls_pvt->galileo_iono, d_ls_pvt->galileo_utc_model);
                                                                                }
                                                                            b_rinex_header_updated = true;  // do not write header anymore
                                                                        }
                                                                }
                                                        }
                                                }
                                        }

                                    // ####################### RTCM MESSAGES #################
                                    try
                                        {
                                            if (b_rtcm_writing_started and b_rtcm_enabled)
                                                {
                                                    if (type_of_rx == 1)  // GPS L1 C/A
                                                        {
                                                            if (flag_write_RTCM_1019_output == true)
                                                                {
                                                                    for (std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin(); gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend(); gps_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter->second);
                                                                        }
                                                                }
                                                            if (flag_write_RTCM_MSM_output == true)
                                                                {
                                                                    std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin();
                                                                    if (gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                }
                                                        }
                                                    if ((type_of_rx == 4) || (type_of_rx == 5) || (type_of_rx == 6) || (type_of_rx == 14) || (type_of_rx == 15))  // Galileo
                                                        {
                                                            if (flag_write_RTCM_1045_output == true)
                                                                {
                                                                    for (std::map<int, Galileo_Ephemeris>::const_iterator gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.cbegin(); gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend(); gal_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1045(gal_eph_iter->second);
                                                                        }
                                                                }
                                                            if (flag_write_RTCM_MSM_output == true)
                                                                {
                                                                    std::map<int, Galileo_Ephemeris>::const_iterator gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.cbegin();
                                                                    if (gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                }
                                                        }
                                                    if (type_of_rx == 7)  // GPS L1 C/A + GPS L2C
                                                        {
                                                            if (flag_write_RTCM_1019_output == true)
                                                                {
                                                                    for (std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin(); gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend(); gps_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter->second);
                                                                        }
                                                                }
                                                            if (flag_write_RTCM_MSM_output == true)
                                                                {
                                                                    std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin();
                                                                    std::map<int, Gps_CNAV_Ephemeris>::const_iterator gps_cnav_eph_iter = d_ls_pvt->gps_cnav_ephemeris_map.cbegin();
                                                                    if ((gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend()) and (gps_cnav_eph_iter != d_ls_pvt->gps_cnav_ephemeris_map.cend()))
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, gps_cnav_eph_iter->second, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                }
                                                        }
                                                    if (type_of_rx == 8)  // L1+L5
                                                        {
                                                            if (flag_write_RTCM_1019_output == true)
                                                                {
                                                                    for (std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin(); gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend(); gps_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter->second);
                                                                        }
                                                                }
                                                            if (flag_write_RTCM_MSM_output == true)
                                                                {
                                                                    std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin();
                                                                    std::map<int, Gps_CNAV_Ephemeris>::const_iterator gps_cnav_eph_iter = d_ls_pvt->gps_cnav_ephemeris_map.cbegin();
                                                                    if ((gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend()) and (gps_cnav_eph_iter != d_ls_pvt->gps_cnav_ephemeris_map.cend()))
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, gps_cnav_eph_iter->second, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                }
                                                        }
                                                    if (type_of_rx == 9)  // GPS L1 C/A + Galileo E1B
                                                        {
                                                            if (flag_write_RTCM_1019_output == true)
                                                                {
                                                                    for (std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin(); gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend(); gps_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter->second);
                                                                        }
                                                                }
                                                            if (flag_write_RTCM_1045_output == true)
                                                                {
                                                                    for (std::map<int, Galileo_Ephemeris>::const_iterator gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.cbegin(); gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend(); gal_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1045(gal_eph_iter->second);
                                                                        }
                                                                }
                                                            if (flag_write_RTCM_MSM_output == true)
                                                                {
                                                                    std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
                                                                    std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin();
                                                                    std::map<int, Galileo_Ephemeris>::const_iterator gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.cbegin();
                                                                    int gps_channel = 0;
                                                                    int gal_channel = 0;
                                                                    for (gnss_observables_iter = gnss_observables_map.cbegin(); gnss_observables_iter != gnss_observables_map.cend(); gnss_observables_iter++)
                                                                        {
                                                                            std::string system(&gnss_observables_iter->second.System, 1);
                                                                            if (gps_channel == 0)
                                                                                {
                                                                                    if (system.compare("G") == 0)
                                                                                        {
                                                                                            // This is a channel with valid GPS signal
                                                                                            gps_eph_iter = d_ls_pvt->gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                            if (gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend())
                                                                                                {
                                                                                                    gps_channel = 1;
                                                                                                }
                                                                                        }
                                                                                }
                                                                            if (gal_channel == 0)
                                                                                {
                                                                                    if (system.compare("E") == 0)
                                                                                        {
                                                                                            gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                            if (gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                                                {
                                                                                                    gal_channel = 1;
                                                                                                }
                                                                                        }
                                                                                }
                                                                        }
                                                                    if (gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                    if (gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                }
                                                        }
                                                    if (type_of_rx == 13)  // L5+E5a
                                                        {
                                                            std::map<int, Galileo_Ephemeris>::const_iterator gal_eph_iter;
                                                            std::map<int, Gps_CNAV_Ephemeris>::const_iterator gps_cnav_eph_iter;
                                                            if (flag_write_RTCM_1045_output == true)
                                                                {
                                                                    for (gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.cbegin(); gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend(); gal_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1045(gal_eph_iter->second);
                                                                        }
                                                                }
                                                            gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.cbegin();
                                                            gps_cnav_eph_iter = d_ls_pvt->gps_cnav_ephemeris_map.cbegin();
                                                            if (flag_write_RTCM_MSM_output == true)
                                                                {
                                                                    if (gps_cnav_eph_iter != d_ls_pvt->gps_cnav_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, {}, gps_cnav_eph_iter->second, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                    if (gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                }
                                                        }
                                                    if ((type_of_rx == 23) || (type_of_rx == 24) || (type_of_rx == 25))  // GLONASS
                                                        {
                                                            if (flag_write_RTCM_1020_output == true)
                                                                {
                                                                    for (std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_ephemeris_iter = d_ls_pvt->glonass_gnav_ephemeris_map.cbegin(); glonass_gnav_ephemeris_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend(); glonass_gnav_ephemeris_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1020(glonass_gnav_ephemeris_iter->second, d_ls_pvt->glonass_gnav_utc_model);
                                                                        }
                                                                }
                                                            if (flag_write_RTCM_MSM_output == true)
                                                                {
                                                                    std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glo_gnav_ephemeris_iter = d_ls_pvt->glonass_gnav_ephemeris_map.cbegin();
                                                                    if (glo_gnav_ephemeris_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, {}, glo_gnav_ephemeris_iter->second, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                }
                                                            b_rtcm_writing_started = true;
                                                        }
                                                    if (type_of_rx == 26)  // GPS L1 C/A + GLONASS L1 C/A
                                                        {
                                                            std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter;
                                                            if (flag_write_RTCM_1019_output == true)
                                                                {
                                                                    for (gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin(); gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend(); gps_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter->second);
                                                                        }
                                                                }
                                                            if (flag_write_RTCM_1020_output == true)
                                                                {
                                                                    for (std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_ephemeris_iter = d_ls_pvt->glonass_gnav_ephemeris_map.cbegin(); glonass_gnav_ephemeris_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend(); glonass_gnav_ephemeris_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1020(glonass_gnav_ephemeris_iter->second, d_ls_pvt->glonass_gnav_utc_model);
                                                                        }
                                                                }
                                                            if (flag_write_RTCM_MSM_output == true)
                                                                {
                                                                    std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
                                                                    std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_eph_iter = d_ls_pvt->glonass_gnav_ephemeris_map.cbegin();
                                                                    std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin();
                                                                    int gps_channel = 0;
                                                                    int glo_channel = 0;
                                                                    for (gnss_observables_iter = gnss_observables_map.cbegin(); gnss_observables_iter != gnss_observables_map.cend(); gnss_observables_iter++)
                                                                        {
                                                                            std::string system(&gnss_observables_iter->second.System, 1);
                                                                            if (gps_channel == 0)
                                                                                {
                                                                                    if (system.compare("G") == 0)
                                                                                        {
                                                                                            // This is a channel with valid GPS signal
                                                                                            gps_eph_iter = d_ls_pvt->gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                            if (gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend())
                                                                                                {
                                                                                                    gps_channel = 1;
                                                                                                }
                                                                                        }
                                                                                }
                                                                            if (glo_channel == 0)
                                                                                {
                                                                                    if (system.compare("R") == 0)
                                                                                        {
                                                                                            glonass_gnav_eph_iter = d_ls_pvt->glonass_gnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                            if (glonass_gnav_eph_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend())
                                                                                                {
                                                                                                    glo_channel = 1;
                                                                                                }
                                                                                        }
                                                                                }
                                                                        }

                                                                    if (glonass_gnav_eph_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, {}, glonass_gnav_eph_iter->second, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                    if (gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                }
                                                        }
                                                    if (type_of_rx == 27)  // GLONASS L1 C/A + Galileo E1B
                                                        {
                                                            if (flag_write_RTCM_1020_output == true)
                                                                {
                                                                    for (std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_eph_iter = d_ls_pvt->glonass_gnav_ephemeris_map.cbegin(); glonass_gnav_eph_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend(); glonass_gnav_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1020(glonass_gnav_eph_iter->second, d_ls_pvt->glonass_gnav_utc_model);
                                                                        }
                                                                }
                                                            if (flag_write_RTCM_1045_output == true)
                                                                {
                                                                    for (std::map<int, Galileo_Ephemeris>::const_iterator gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.cbegin(); gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend(); gal_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1045(gal_eph_iter->second);
                                                                        }
                                                                }
                                                            if (flag_write_RTCM_MSM_output == true)
                                                                {
                                                                    std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
                                                                    std::map<int, Galileo_Ephemeris>::const_iterator gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.cbegin();
                                                                    std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_eph_iter = d_ls_pvt->glonass_gnav_ephemeris_map.cbegin();
                                                                    int gal_channel = 0;
                                                                    int glo_channel = 0;
                                                                    for (gnss_observables_iter = gnss_observables_map.cbegin(); gnss_observables_iter != gnss_observables_map.cend(); gnss_observables_iter++)
                                                                        {
                                                                            std::string system(&gnss_observables_iter->second.System, 1);
                                                                            if (gal_channel == 0)
                                                                                {
                                                                                    if (system.compare("E") == 0)
                                                                                        {
                                                                                            // This is a channel with valid GPS signal
                                                                                            gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                            if (gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                                                {
                                                                                                    gal_channel = 1;
                                                                                                }
                                                                                        }
                                                                                }
                                                                            if (glo_channel == 0)
                                                                                {
                                                                                    if (system.compare("R") == 0)
                                                                                        {
                                                                                            glonass_gnav_eph_iter = d_ls_pvt->glonass_gnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                            if (glonass_gnav_eph_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend())
                                                                                                {
                                                                                                    glo_channel = 1;
                                                                                                }
                                                                                        }
                                                                                }
                                                                        }
                                                                    if (gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                    if (glonass_gnav_eph_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, {}, glonass_gnav_eph_iter->second, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                }
                                                        }
                                                    if (type_of_rx == 29)  // GPS L1 C/A + GLONASS L2 C/A
                                                        {
                                                            if (flag_write_RTCM_1019_output == true)
                                                                {
                                                                    for (std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin(); gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend(); gps_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter->second);
                                                                        }
                                                                }
                                                            if (flag_write_RTCM_1020_output == true)
                                                                {
                                                                    for (std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_eph_iter = d_ls_pvt->glonass_gnav_ephemeris_map.cbegin(); glonass_gnav_eph_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend(); glonass_gnav_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1020(glonass_gnav_eph_iter->second, d_ls_pvt->glonass_gnav_utc_model);
                                                                        }
                                                                }
                                                            if (flag_write_RTCM_MSM_output == true)
                                                                {
                                                                    std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
                                                                    std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin();
                                                                    std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_eph_iter = d_ls_pvt->glonass_gnav_ephemeris_map.cbegin();
                                                                    int gps_channel = 0;
                                                                    int glo_channel = 0;
                                                                    for (gnss_observables_iter = gnss_observables_map.cbegin(); gnss_observables_iter != gnss_observables_map.cend(); gnss_observables_iter++)
                                                                        {
                                                                            std::string system(&gnss_observables_iter->second.System, 1);
                                                                            if (gps_channel == 0)
                                                                                {
                                                                                    if (system.compare("G") == 0)
                                                                                        {
                                                                                            // This is a channel with valid GPS signal
                                                                                            gps_eph_iter = d_ls_pvt->gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                            if (gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend())
                                                                                                {
                                                                                                    gps_channel = 1;
                                                                                                }
                                                                                        }
                                                                                }
                                                                            if (glo_channel == 0)
                                                                                {
                                                                                    if (system.compare("R") == 0)
                                                                                        {
                                                                                            glonass_gnav_eph_iter = d_ls_pvt->glonass_gnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                            if (glonass_gnav_eph_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend())
                                                                                                {
                                                                                                    glo_channel = 1;
                                                                                                }
                                                                                        }
                                                                                }
                                                                        }
                                                                    if (glonass_gnav_eph_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, {}, glonass_gnav_eph_iter->second, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                    if (gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                }
                                                        }
                                                    if (type_of_rx == 30)  // GLONASS L2 C/A + Galileo E1B
                                                        {
                                                            if (flag_write_RTCM_1020_output == true)
                                                                {
                                                                    for (std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_eph_iter = d_ls_pvt->glonass_gnav_ephemeris_map.cbegin(); glonass_gnav_eph_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend(); glonass_gnav_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1020(glonass_gnav_eph_iter->second, d_ls_pvt->glonass_gnav_utc_model);
                                                                        }
                                                                }
                                                            if (flag_write_RTCM_1045_output == true)
                                                                {
                                                                    for (std::map<int, Galileo_Ephemeris>::const_iterator gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.cbegin(); gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend(); gal_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1045(gal_eph_iter->second);
                                                                        }
                                                                }
                                                            if (flag_write_RTCM_MSM_output == true)
                                                                {
                                                                    std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
                                                                    std::map<int, Galileo_Ephemeris>::const_iterator gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.cbegin();
                                                                    std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_eph_iter = d_ls_pvt->glonass_gnav_ephemeris_map.cbegin();
                                                                    int gal_channel = 0;
                                                                    int glo_channel = 0;
                                                                    for (gnss_observables_iter = gnss_observables_map.cbegin(); gnss_observables_iter != gnss_observables_map.cend(); gnss_observables_iter++)
                                                                        {
                                                                            std::string system(&gnss_observables_iter->second.System, 1);
                                                                            if (gal_channel == 0)
                                                                                {
                                                                                    if (system.compare("E") == 0)
                                                                                        {
                                                                                            // This is a channel with valid GPS signal
                                                                                            gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                            if (gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                                                {
                                                                                                    gal_channel = 1;
                                                                                                }
                                                                                        }
                                                                                }
                                                                            if (glo_channel == 0)
                                                                                {
                                                                                    if (system.compare("R") == 0)
                                                                                        {
                                                                                            glonass_gnav_eph_iter = d_ls_pvt->glonass_gnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                            if (glonass_gnav_eph_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend())
                                                                                                {
                                                                                                    glo_channel = 1;
                                                                                                }
                                                                                        }
                                                                                }
                                                                        }
                                                                    if (gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                    if (glonass_gnav_eph_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, {}, glonass_gnav_eph_iter->second, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                }
                                                        }
                                                    if (type_of_rx == 32)  // L1+E1+L5+E5a
                                                        {
                                                            if (flag_write_RTCM_1019_output == true)
                                                                {
                                                                    for (std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin(); gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend(); gps_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter->second);
                                                                        }
                                                                }
                                                            if (flag_write_RTCM_1045_output == true)
                                                                {
                                                                    for (std::map<int, Galileo_Ephemeris>::const_iterator gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.cbegin(); gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend(); gal_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1045(gal_eph_iter->second);
                                                                        }
                                                                }
                                                            if (flag_write_RTCM_MSM_output == true)
                                                                {
                                                                    std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
                                                                    std::map<int, Galileo_Ephemeris>::const_iterator gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.cbegin();
                                                                    std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin();
                                                                    int gal_channel = 0;
                                                                    int gps_channel = 0;
                                                                    for (gnss_observables_iter = gnss_observables_map.cbegin(); gnss_observables_iter != gnss_observables_map.cend(); gnss_observables_iter++)
                                                                        {
                                                                            std::string system(&gnss_observables_iter->second.System, 1);
                                                                            if (gal_channel == 0)
                                                                                {
                                                                                    if (system.compare("E") == 0)
                                                                                        {
                                                                                            // This is a channel with valid GPS signal
                                                                                            gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                            if (gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                                                {
                                                                                                    gal_channel = 1;
                                                                                                }
                                                                                        }
                                                                                }
                                                                            if (gps_channel == 0)
                                                                                {
                                                                                    if (system.compare("G") == 0)
                                                                                        {
                                                                                            gps_eph_iter = d_ls_pvt->gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                            if (gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend())
                                                                                                {
                                                                                                    gps_channel = 1;
                                                                                                }
                                                                                        }
                                                                                }
                                                                        }
                                                                    if (gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                    if (gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                }
                                                        }
                                                }

                                            if (!b_rtcm_writing_started and b_rtcm_enabled)  // the first time
                                                {
                                                    if (type_of_rx == 1)  // GPS L1 C/A
                                                        {
                                                            if (d_rtcm_MT1019_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                {
                                                                    for (std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin(); gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend(); gps_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter->second);
                                                                        }
                                                                }
                                                            if (d_rtcm_MSM_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                {
                                                                    std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin();

                                                                    if (gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                }
                                                            b_rtcm_writing_started = true;
                                                        }
                                                    if ((type_of_rx == 4) || (type_of_rx == 5) || (type_of_rx == 6) || (type_of_rx == 14) || (type_of_rx == 15))  // Galileo
                                                        {
                                                            if (d_rtcm_MT1045_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                {
                                                                    for (std::map<int, Galileo_Ephemeris>::const_iterator gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.cbegin(); gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend(); gal_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1045(gal_eph_iter->second);
                                                                        }
                                                                }
                                                            if (d_rtcm_MSM_rate_ms != 0)
                                                                {
                                                                    std::map<int, Galileo_Ephemeris>::const_iterator gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.cbegin();
                                                                    if (gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                }
                                                            b_rtcm_writing_started = true;
                                                        }
                                                    if (type_of_rx == 7)  // GPS L1 C/A + GPS L2C
                                                        {
                                                            if (d_rtcm_MT1019_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                {
                                                                    for (std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin(); gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend(); gps_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter->second);
                                                                        }
                                                                }
                                                            if (d_rtcm_MSM_rate_ms != 0)
                                                                {
                                                                    std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin();
                                                                    std::map<int, Gps_CNAV_Ephemeris>::const_iterator gps_cnav_eph_iter = d_ls_pvt->gps_cnav_ephemeris_map.cbegin();
                                                                    if ((gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend()) and (gps_cnav_eph_iter != d_ls_pvt->gps_cnav_ephemeris_map.cend()))
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, gps_cnav_eph_iter->second, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                }
                                                            b_rtcm_writing_started = true;
                                                        }
                                                    if (type_of_rx == 8)  // L1+L5
                                                        {
                                                            if (d_rtcm_MT1019_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                {
                                                                    for (std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin(); gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend(); gps_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter->second);
                                                                        }
                                                                }
                                                            if (d_rtcm_MSM_rate_ms != 0)
                                                                {
                                                                    std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin();
                                                                    std::map<int, Gps_CNAV_Ephemeris>::const_iterator gps_cnav_eph_iter = d_ls_pvt->gps_cnav_ephemeris_map.cbegin();
                                                                    if ((gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend()) and (gps_cnav_eph_iter != d_ls_pvt->gps_cnav_ephemeris_map.cend()))
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, gps_cnav_eph_iter->second, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                }
                                                            b_rtcm_writing_started = true;
                                                        }
                                                    if (type_of_rx == 9)  // GPS L1 C/A + Galileo E1B
                                                        {
                                                            if (d_rtcm_MT1019_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                {
                                                                    for (std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin(); gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend(); gps_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter->second);
                                                                        }
                                                                }
                                                            if (d_rtcm_MT1045_rate_ms != 0)
                                                                {
                                                                    for (std::map<int, Galileo_Ephemeris>::const_iterator gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.cbegin(); gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend(); gal_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1045(gal_eph_iter->second);
                                                                        }
                                                                }
                                                            if (d_rtcm_MSM_rate_ms != 0)
                                                                {
                                                                    std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
                                                                    std::map<int, Galileo_Ephemeris>::const_iterator gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.cbegin();
                                                                    std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin();
                                                                    int gps_channel = 0;
                                                                    int gal_channel = 0;
                                                                    for (gnss_observables_iter = gnss_observables_map.cbegin(); gnss_observables_iter != gnss_observables_map.cend(); gnss_observables_iter++)
                                                                        {
                                                                            std::string system(&gnss_observables_iter->second.System, 1);
                                                                            if (gps_channel == 0)
                                                                                {
                                                                                    if (system.compare("G") == 0)
                                                                                        {
                                                                                            // This is a channel with valid GPS signal
                                                                                            gps_eph_iter = d_ls_pvt->gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                            if (gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend())
                                                                                                {
                                                                                                    gps_channel = 1;
                                                                                                }
                                                                                        }
                                                                                }
                                                                            if (gal_channel == 0)
                                                                                {
                                                                                    if (system.compare("E") == 0)
                                                                                        {
                                                                                            gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                            if (gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                                                {
                                                                                                    gal_channel = 1;
                                                                                                }
                                                                                        }
                                                                                }
                                                                        }
                                                                    if (gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                    if (gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                }
                                                            b_rtcm_writing_started = true;
                                                        }
                                                    if ((type_of_rx == 23) || (type_of_rx == 24) || (type_of_rx == 25))  // GLONASS
                                                        {
                                                            if (d_rtcm_MT1020_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                {
                                                                    for (std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_eph_iter = d_ls_pvt->glonass_gnav_ephemeris_map.cbegin(); glonass_gnav_eph_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend(); glonass_gnav_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1020(glonass_gnav_eph_iter->second, d_ls_pvt->glonass_gnav_utc_model);
                                                                        }
                                                                }
                                                            if (d_rtcm_MSM_rate_ms != 0)
                                                                {
                                                                    std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glo_gnav_ephemeris_iter = d_ls_pvt->glonass_gnav_ephemeris_map.cbegin();
                                                                    if (glo_gnav_ephemeris_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, {}, glo_gnav_ephemeris_iter->second, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                }
                                                            b_rtcm_writing_started = true;
                                                        }
                                                    if (type_of_rx == 13)  // L5+E5a
                                                        {
                                                            std::map<int, Galileo_Ephemeris>::const_iterator gal_eph_iter;
                                                            if (d_rtcm_MT1045_rate_ms != 0)
                                                                {
                                                                    for (gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.cbegin(); gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend(); gal_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1045(gal_eph_iter->second);
                                                                        }
                                                                }
                                                            if (d_rtcm_MSM_rate_ms != 0)
                                                                {
                                                                    gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.cbegin();
                                                                    if (gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend() and (d_rtcm_MT1097_rate_ms != 0))
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                }
                                                            b_rtcm_writing_started = true;
                                                        }
                                                    if (type_of_rx == 26)  // GPS L1 C/A + GLONASS L1 C/A
                                                        {
                                                            if (d_rtcm_MT1019_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                {
                                                                    for (std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin(); gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend(); gps_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter->second);
                                                                        }
                                                                }
                                                            if (d_rtcm_MT1020_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                {
                                                                    for (std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_eph_iter = d_ls_pvt->glonass_gnav_ephemeris_map.cbegin(); glonass_gnav_eph_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend(); glonass_gnav_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1020(glonass_gnav_eph_iter->second, d_ls_pvt->glonass_gnav_utc_model);
                                                                        }
                                                                }
                                                            if (d_rtcm_MSM_rate_ms != 0)
                                                                {
                                                                    std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
                                                                    std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin();
                                                                    std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_eph_iter = d_ls_pvt->glonass_gnav_ephemeris_map.cbegin();
                                                                    int gps_channel = 0;
                                                                    int glo_channel = 0;
                                                                    for (gnss_observables_iter = gnss_observables_map.cbegin(); gnss_observables_iter != gnss_observables_map.cend(); gnss_observables_iter++)
                                                                        {
                                                                            std::string system(&gnss_observables_iter->second.System, 1);
                                                                            if (gps_channel == 0)
                                                                                {
                                                                                    if (system.compare("G") == 0)
                                                                                        {
                                                                                            // This is a channel with valid GPS signal
                                                                                            gps_eph_iter = d_ls_pvt->gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                            if (gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend())
                                                                                                {
                                                                                                    gps_channel = 1;
                                                                                                }
                                                                                        }
                                                                                }
                                                                            if (glo_channel == 0)
                                                                                {
                                                                                    if (system.compare("R") == 0)
                                                                                        {
                                                                                            glonass_gnav_eph_iter = d_ls_pvt->glonass_gnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                            if (glonass_gnav_eph_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend())
                                                                                                {
                                                                                                    glo_channel = 1;
                                                                                                }
                                                                                        }
                                                                                }
                                                                        }
                                                                    if (glonass_gnav_eph_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, {}, glonass_gnav_eph_iter->second, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                    if (gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                }
                                                            b_rtcm_writing_started = true;
                                                        }
                                                    if (type_of_rx == 27)  // GLONASS L1 C/A + Galileo E1B
                                                        {
                                                            if (d_rtcm_MT1020_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                {
                                                                    for (std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_eph_iter = d_ls_pvt->glonass_gnav_ephemeris_map.cbegin(); glonass_gnav_eph_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend(); glonass_gnav_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1020(glonass_gnav_eph_iter->second, d_ls_pvt->glonass_gnav_utc_model);
                                                                        }
                                                                }
                                                            if (d_rtcm_MT1045_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                {
                                                                    for (std::map<int, Galileo_Ephemeris>::const_iterator gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.cbegin(); gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend(); gal_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1045(gal_eph_iter->second);
                                                                        }
                                                                }
                                                            if (d_rtcm_MSM_rate_ms != 0)
                                                                {
                                                                    int gal_channel = 0;
                                                                    int glo_channel = 0;
                                                                    std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
                                                                    std::map<int, Galileo_Ephemeris>::const_iterator gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.cbegin();
                                                                    std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_eph_iter = = d_ls_pvt->glonass_gnav_ephemeris_map.cbegin();
                                                                    for (gnss_observables_iter = gnss_observables_map.cbegin(); gnss_observables_iter != gnss_observables_map.cend(); gnss_observables_iter++)
                                                                        {
                                                                            std::string system(&gnss_observables_iter->second.System, 1);
                                                                            if (gal_channel == 0)
                                                                                {
                                                                                    if (system.compare("E") == 0)
                                                                                        {
                                                                                            // This is a channel with valid GPS signal
                                                                                            gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                            if (gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                                                {
                                                                                                    gal_channel = 1;
                                                                                                }
                                                                                        }
                                                                                }
                                                                            if (glo_channel == 0)
                                                                                {
                                                                                    if (system.compare("R") == 0)
                                                                                        {
                                                                                            glonass_gnav_eph_iter = d_ls_pvt->glonass_gnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                            if (glonass_gnav_eph_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend())
                                                                                                {
                                                                                                    glo_channel = 1;
                                                                                                }
                                                                                        }
                                                                                }
                                                                        }
                                                                    if (gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                    if (glonass_gnav_eph_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, {}, glonass_gnav_eph_iter->second, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                }
                                                            b_rtcm_writing_started = true;
                                                        }
                                                    if (type_of_rx == 29)  // GPS L1 C/A + GLONASS L2 C/A
                                                        {
                                                            if (d_rtcm_MT1019_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                {
                                                                    for (std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin(); gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend(); gps_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter->second);
                                                                        }
                                                                }
                                                            if (d_rtcm_MT1020_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                {
                                                                    for (std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_eph_iter = d_ls_pvt->glonass_gnav_ephemeris_map.cbegin(); glonass_gnav_eph_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend(); glonass_gnav_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1020(glonass_gnav_eph_iter->second, d_ls_pvt->glonass_gnav_utc_model);
                                                                        }
                                                                }
                                                            if (d_rtcm_MSM_rate_ms != 0)
                                                                {
                                                                    std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin();
                                                                    std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_eph_iter = d_ls_pvt->glonass_gnav_ephemeris_map.cbegin();
                                                                    std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
                                                                    int gps_channel = 0;
                                                                    int glo_channel = 0;
                                                                    for (gnss_observables_iter = gnss_observables_map.cbegin(); gnss_observables_iter != gnss_observables_map.cend(); gnss_observables_iter++)
                                                                        {
                                                                            std::string system(&gnss_observables_iter->second.System, 1);
                                                                            if (gps_channel == 0)
                                                                                {
                                                                                    if (system.compare("G") == 0)
                                                                                        {
                                                                                            // This is a channel with valid GPS signal
                                                                                            gps_eph_iter = d_ls_pvt->gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                            if (gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend())
                                                                                                {
                                                                                                    gps_channel = 1;
                                                                                                }
                                                                                        }
                                                                                }
                                                                            if (glo_channel == 0)
                                                                                {
                                                                                    if (system.compare("R") == 0)
                                                                                        {
                                                                                            glonass_gnav_eph_iter = d_ls_pvt->glonass_gnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                            if (glonass_gnav_eph_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend())
                                                                                                {
                                                                                                    glo_channel = 1;
                                                                                                }
                                                                                        }
                                                                                }
                                                                        }
                                                                    if (glonass_gnav_eph_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, {}, glonass_gnav_eph_iter->second, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }

                                                                    if (gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                }
                                                            b_rtcm_writing_started = true;
                                                        }
                                                    if (type_of_rx == 30)  // GLONASS L2 C/A + Galileo E1B
                                                        {
                                                            if (d_rtcm_MT1020_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                {
                                                                    for (std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_eph_iter = d_ls_pvt->glonass_gnav_ephemeris_map.cbegin(); glonass_gnav_eph_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend(); glonass_gnav_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1020(glonass_gnav_eph_iter->second, d_ls_pvt->glonass_gnav_utc_model);
                                                                        }
                                                                }
                                                            if (d_rtcm_MT1045_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                {
                                                                    for (std::map<int, Galileo_Ephemeris>::const_iterator gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.cbegin(); gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend(); gal_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1045(gal_eph_iter->second);
                                                                        }
                                                                }
                                                            if (d_rtcm_MSM_rate_ms != 0)
                                                                {
                                                                    int gal_channel = 0;
                                                                    int glo_channel = 0;
                                                                    std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
                                                                    std::map<int, Galileo_Ephemeris>::const_iterator gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.cbegin();
                                                                    std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_eph_iter = d_ls_pvt->glonass_gnav_ephemeris_map.cbegin();
                                                                    for (gnss_observables_iter = gnss_observables_map.cbegin(); gnss_observables_iter != gnss_observables_map.cend(); gnss_observables_iter++)
                                                                        {
                                                                            std::string system(&gnss_observables_iter->second.System, 1);
                                                                            if (gal_channel == 0)
                                                                                {
                                                                                    if (system.compare("E") == 0)
                                                                                        {
                                                                                            // This is a channel with valid GPS signal
                                                                                            gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                            if (gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                                                {
                                                                                                    gal_channel = 1;
                                                                                                }
                                                                                        }
                                                                                }
                                                                            if (glo_channel == 0)
                                                                                {
                                                                                    if (system.compare("R") == 0)
                                                                                        {
                                                                                            glonass_gnav_eph_iter = d_ls_pvt->glonass_gnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                            if (glonass_gnav_eph_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend())
                                                                                                {
                                                                                                    glo_channel = 1;
                                                                                                }
                                                                                        }
                                                                                }
                                                                        }
                                                                    if (gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                    if (glonass_gnav_eph_iter != d_ls_pvt->glonass_gnav_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, {}, glonass_gnav_eph_iter->second, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                }
                                                            b_rtcm_writing_started = true;
                                                        }
                                                    if (type_of_rx == 32)  // L1+E1+L5+E5a
                                                        {
                                                            if (d_rtcm_MT1019_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                {
                                                                    for (std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin(); gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend(); gps_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter->second);
                                                                        }
                                                                }
                                                            if (d_rtcm_MT1045_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                {
                                                                    for (std::map<int, Galileo_Ephemeris>::const_iterator gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.cbegin(); gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend(); gal_eph_iter++)
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MT1045(gal_eph_iter->second);
                                                                        }
                                                                }
                                                            if (d_rtcm_MSM_rate_ms != 0)
                                                                {
                                                                    std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
                                                                    std::map<int, Galileo_Ephemeris>::const_iterator gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.cbegin();
                                                                    std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter = d_ls_pvt->gps_ephemeris_map.cbegin();
                                                                    int gps_channel = 0;
                                                                    int gal_channel = 0;
                                                                    for (gnss_observables_iter = gnss_observables_map.cbegin(); gnss_observables_iter != gnss_observables_map.cend(); gnss_observables_iter++)
                                                                        {
                                                                            std::string system(&gnss_observables_iter->second.System, 1);
                                                                            if (gps_channel == 0)
                                                                                {
                                                                                    if (system.compare("G") == 0)
                                                                                        {
                                                                                            // This is a channel with valid GPS signal
                                                                                            gps_eph_iter = d_ls_pvt->gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                            if (gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend())
                                                                                                {
                                                                                                    gps_channel = 1;
                                                                                                }
                                                                                        }
                                                                                }
                                                                            if (gal_channel == 0)
                                                                                {
                                                                                    if (system.compare("E") == 0)
                                                                                        {
                                                                                            gal_eph_iter = d_ls_pvt->galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                            if (gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                                                {
                                                                                                    gal_channel = 1;
                                                                                                }
                                                                                        }
                                                                                }
                                                                        }
                                                                    if (gps_eph_iter != d_ls_pvt->gps_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                    if (gal_eph_iter != d_ls_pvt->galileo_ephemeris_map.cend())
                                                                        {
                                                                            d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, 0, 0);
                                                                        }
                                                                }
                                                            b_rtcm_writing_started = true;
                                                        }
                                                }
                                        }
                                    catch (const boost::exception& ex)
                                        {
                                            std::cout << "RTCM boost exception: " << boost::diagnostic_information(ex) << std::endl;
                                            LOG(ERROR) << "RTCM boost exception: " << boost::diagnostic_information(ex);
                                        }
                                    catch (const std::exception& ex)
                                        {
                                            std::cout << "RTCM std exception: " << ex.what() << std::endl;
                                            LOG(ERROR) << "RTCM std exception: " << ex.what();
                                        }
                                }
                        }

                    // DEBUG MESSAGE: Display position in console output
                    if (d_ls_pvt->is_valid_position() and flag_display_pvt)
                        {
                            std::streamsize ss = std::cout.precision();  // save current precision
                            std::cout.setf(std::ios::fixed, std::ios::floatfield);
                            auto facet = new boost::posix_time::time_facet("%Y-%b-%d %H:%M:%S.%f %z");
                            std::cout.imbue(std::locale(std::cout.getloc(), facet));

                            std::cout << TEXT_BOLD_GREEN
                                      << "Position at " << d_ls_pvt->get_position_UTC_time()
                                      << " UTC using " << d_ls_pvt->get_num_valid_observations()
                                      << std::fixed << std::setprecision(9)
                                      << " observations is Lat = " << d_ls_pvt->get_latitude() << " [deg], Long = " << d_ls_pvt->get_longitude()
                                      << std::fixed << std::setprecision(3)
                                      << " [deg], Height = " << d_ls_pvt->get_height() << " [m]" << TEXT_RESET << std::endl;
                            std::cout << std::setprecision(ss);
                            DLOG(INFO) << "RX clock offset: " << d_ls_pvt->get_time_offset_s() << "[s]";

                            // boost::posix_time::ptime p_time;
                            // gtime_t rtklib_utc_time = gpst2time(adjgpsweek(d_ls_pvt->gps_ephemeris_map.cbegin()->second.i_GPS_week), d_rx_time);
                            // p_time = boost::posix_time::from_time_t(rtklib_utc_time.time);
                            // p_time += boost::posix_time::microseconds(round(rtklib_utc_time.sec * 1e6));
                            // std::cout << TEXT_MAGENTA << "Observable RX time (GPST) " << boost::posix_time::to_simple_string(p_time) << TEXT_RESET << std::endl;

                            DLOG(INFO) << "Position at " << boost::posix_time::to_simple_string(d_ls_pvt->get_position_UTC_time())
                                       << " UTC using " << d_ls_pvt->get_num_valid_observations() << " observations is Lat = " << d_ls_pvt->get_latitude() << " [deg], Long = " << d_ls_pvt->get_longitude()
                                       << " [deg], Height = " << d_ls_pvt->get_height() << " [m]";

                            /* std::cout << "Dilution of Precision at " << boost::posix_time::to_simple_string(d_ls_pvt->get_position_UTC_time())
                                         << " UTC using "<< d_ls_pvt->get_num_valid_observations() <<" observations is HDOP = " << d_ls_pvt->get_hdop() << " VDOP = "
                                         << d_ls_pvt->get_vdop()
                                         << " GDOP = " << d_ls_pvt->get_gdop() << std::endl; */
                        }
                }
        }

    return noutput_items;
}
