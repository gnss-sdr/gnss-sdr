/*!
 * \file rtklib_pvt_gs.cc
 * \brief Interface of a Position Velocity and Time computation block
 * \author Javier Arribas, 2017. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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

#include "rtklib_pvt_gs.h"
#include "MATH_CONSTANTS.h"
#include "beidou_dnav_almanac.h"
#include "beidou_dnav_ephemeris.h"
#include "beidou_dnav_iono.h"
#include "beidou_dnav_utc_model.h"
#include "display.h"
#include "galileo_almanac.h"
#include "galileo_almanac_helper.h"
#include "galileo_ephemeris.h"
#include "galileo_iono.h"
#include "galileo_utc_model.h"
#include "geojson_printer.h"
#include "glonass_gnav_almanac.h"
#include "glonass_gnav_ephemeris.h"
#include "glonass_gnav_utc_model.h"
#include "gnss_frequencies.h"
#include "gnss_sdr_create_directory.h"
#include "gps_almanac.h"
#include "gps_cnav_ephemeris.h"
#include "gps_cnav_iono.h"
#include "gps_cnav_utc_model.h"
#include "gps_ephemeris.h"
#include "gps_iono.h"
#include "gps_utc_model.h"
#include "gpx_printer.h"
#include "kml_printer.h"
#include "monitor_pvt.h"
#include "monitor_pvt_udp_sink.h"
#include "nmea_printer.h"
#include "pvt_conf.h"
#include "rinex_printer.h"
#include "rtcm_printer.h"
#include "rtklib_solver.h"
#include <boost/any.hpp>                   // for any_cast, any
#include <boost/archive/xml_iarchive.hpp>  // for xml_iarchive
#include <boost/archive/xml_oarchive.hpp>  // for xml_oarchive
#include <boost/bind/bind.hpp>             // for bind_t, bind
#include <boost/exception/diagnostic_information.hpp>
#include <boost/exception/exception.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/nvp.hpp>  // for nvp, make_nvp
#include <glog/logging.h>               // for LOG
#include <gnuradio/io_signature.h>      // for io_signature
#include <pmt/pmt_sugar.h>              // for mp
#include <algorithm>                    // for sort, unique
#include <exception>                    // for exception
#include <fstream>                      // for ofstream
#include <iomanip>                      // for put_time, setprecision
#include <iostream>                     // for operator<<
#include <locale>                       // for locale
#include <sstream>                      // for ostringstream
#include <stdexcept>                    // for length_error
#include <sys/ipc.h>                    // for IPC_CREAT
#include <sys/msg.h>                    // for msgctl

#if HAS_STD_FILESYSTEM
#include <system_error>
namespace errorlib = std;
#if HAS_STD_FILESYSTEM_EXPERIMENTAL
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif
#else
#include <boost/filesystem/path.hpp>
#include <boost/system/error_code.hpp>  // for error_code
namespace fs = boost::filesystem;
namespace errorlib = boost::system;
#endif

#if OLD_BOOST
#include <boost/math/common_factor_rt.hpp>
namespace bc = boost::math;
#else
#include <boost/integer/common_factor_rt.hpp>
namespace bc = boost::integer;
#endif


rtklib_pvt_gs_sptr rtklib_make_pvt_gs(uint32_t nchannels,
    const Pvt_Conf& conf_,
    const rtk_t& rtk)
{
    return rtklib_pvt_gs_sptr(new rtklib_pvt_gs(nchannels,
        conf_,
        rtk));
}


rtklib_pvt_gs::rtklib_pvt_gs(uint32_t nchannels,
    const Pvt_Conf& conf_,
    const rtk_t& rtk) : gr::sync_block("rtklib_pvt_gs",
                            gr::io_signature::make(nchannels, nchannels, sizeof(Gnss_Synchro)),
                            gr::io_signature::make(0, 0, 0))
{
    // Send feedback message to observables block with the receiver clock offset
    this->message_port_register_out(pmt::mp("pvt_to_observables"));
    // Send PVT status to gnss_flowgraph
    this->message_port_register_out(pmt::mp("status"));

    mapStringValues_["1C"] = evGPS_1C;
    mapStringValues_["2S"] = evGPS_2S;
    mapStringValues_["L5"] = evGPS_L5;
    mapStringValues_["1B"] = evGAL_1B;
    mapStringValues_["5X"] = evGAL_5X;
    mapStringValues_["1G"] = evGLO_1G;
    mapStringValues_["2G"] = evGLO_2G;
    mapStringValues_["B1"] = evBDS_B1;
    mapStringValues_["B2"] = evBDS_B2;
    mapStringValues_["B3"] = evBDS_B3;

    max_obs_block_rx_clock_offset_ms = conf_.max_obs_block_rx_clock_offset_ms;
    d_output_rate_ms = conf_.output_rate_ms;
    d_display_rate_ms = conf_.display_rate_ms;
    d_report_rate_ms = 1000;  // report every second PVT to gnss_synchro
    d_dump = conf_.dump;
    d_dump_mat = conf_.dump_mat and d_dump;
    d_dump_filename = conf_.dump_filename;
    std::string dump_ls_pvt_filename = conf_.dump_filename;
    if (d_dump)
        {
            std::string dump_path;
            // Get path
            if (d_dump_filename.find_last_of('/') != std::string::npos)
                {
                    std::string dump_filename_ = d_dump_filename.substr(d_dump_filename.find_last_of('/') + 1);
                    dump_path = d_dump_filename.substr(0, d_dump_filename.find_last_of('/'));
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
            if (d_dump_filename.substr(1).find_last_of('.') != std::string::npos)
                {
                    d_dump_filename = d_dump_filename.substr(0, d_dump_filename.find_last_of('.'));
                }
            dump_ls_pvt_filename = dump_path + fs::path::preferred_separator + d_dump_filename;
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
    this->set_msg_handler(pmt::mp("telemetry"), boost::bind(&rtklib_pvt_gs::msg_handler_telemetry, this, _1));

    // initialize kml_printer
    std::string kml_dump_filename;
    kml_dump_filename = d_dump_filename;
    d_kml_output_enabled = conf_.kml_output_enabled;
    d_kml_rate_ms = conf_.kml_rate_ms;
    if (d_kml_rate_ms == 0)
        {
            d_kml_output_enabled = false;
        }
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
    d_gpx_rate_ms = conf_.gpx_rate_ms;
    if (d_gpx_rate_ms == 0)
        {
            d_gpx_output_enabled = false;
        }
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
    d_geojson_rate_ms = conf_.geojson_rate_ms;
    if (d_geojson_rate_ms == 0)
        {
            d_geojson_output_enabled = false;
        }
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
    d_nmea_output_file_enabled = (conf_.nmea_output_file_enabled or conf_.flag_nmea_tty_port);
    d_nmea_rate_ms = conf_.nmea_rate_ms;
    if (d_nmea_rate_ms == 0)
        {
            d_nmea_output_file_enabled = false;
        }

    if (d_nmea_output_file_enabled)
        {
            d_nmea_printer = std::make_shared<Nmea_Printer>(conf_.nmea_dump_filename, conf_.nmea_output_file_enabled, conf_.flag_nmea_tty_port, conf_.nmea_dump_devname, conf_.nmea_output_file_path);
        }
    else
        {
            d_nmea_printer = nullptr;
        }

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

    // XML printer
    d_xml_storage = conf_.xml_output_enabled;
    if (d_xml_storage)
        {
            xml_base_path = conf_.xml_output_path;
            fs::path full_path(fs::current_path());
            const fs::path p(xml_base_path);
            if (!fs::exists(p))
                {
                    std::string new_folder;
                    for (auto& folder : fs::path(xml_base_path))
                        {
                            new_folder += folder.string();
                            errorlib::error_code ec;
                            if (!fs::exists(new_folder))
                                {
                                    if (!fs::create_directory(new_folder, ec))
                                        {
                                            std::cout << "Could not create the " << new_folder << " folder." << std::endl;
                                            xml_base_path = full_path.string();
                                        }
                                }
                            new_folder += fs::path::preferred_separator;
                        }
                }
            else
                {
                    xml_base_path = p.string();
                }
            if (xml_base_path != ".")
                {
                    std::cout << "XML files will be stored at " << xml_base_path << std::endl;
                }

            xml_base_path = xml_base_path + fs::path::preferred_separator;
        }

    d_rx_time = 0.0;
    d_last_status_print_seg = 0;

    // PVT MONITOR
    flag_monitor_pvt_enabled = conf_.monitor_enabled;
    if (flag_monitor_pvt_enabled)
        {
            std::string address_string = conf_.udp_addresses;
            std::vector<std::string> udp_addr_vec = split_string(address_string, '_');
            std::sort(udp_addr_vec.begin(), udp_addr_vec.end());
            udp_addr_vec.erase(std::unique(udp_addr_vec.begin(), udp_addr_vec.end()), udp_addr_vec.end());

            udp_sink_ptr = std::unique_ptr<Monitor_Pvt_Udp_Sink>(new Monitor_Pvt_Udp_Sink(udp_addr_vec, conf_.udp_port, conf_.protobuf_enabled));
        }
    else
        {
            udp_sink_ptr = nullptr;
        }

    // Create Sys V message queue
    first_fix = true;
    sysv_msg_key = 1101;
    int msgflg = IPC_CREAT | 0666;
    if ((sysv_msqid = msgget(sysv_msg_key, msgflg)) == -1)
        {
            std::cout << "GNSS-SDR can not create message queues!" << std::endl;
            throw std::exception();
        }

    // Display time in local time zone
    d_show_local_time_zone = conf_.show_local_time_zone;
    std::ostringstream os;
#ifdef HAS_PUT_TIME
    time_t when = std::time(nullptr);
    auto const tm = *std::localtime(&when);
    os << std::put_time(&tm, "%z");
#endif
    std::string utc_diff_str = os.str();  // in ISO 8601 format: "+HHMM" or "-HHMM"
    if (utc_diff_str.empty())
        {
            utc_diff_str = "+0000";
        }
    int h = std::stoi(utc_diff_str.substr(0, 3), nullptr, 10);
    int m = std::stoi(utc_diff_str[0] + utc_diff_str.substr(3), nullptr, 10);
    d_utc_diff_time = boost::posix_time::hours(h) + boost::posix_time::minutes(m);
    std::ostringstream os2;
#ifdef HAS_PUT_TIME
    os2 << std::put_time(&tm, "%Z");
#endif
    std::string time_zone_abrv = os2.str();
    if (time_zone_abrv.empty())
        {
            if (utc_diff_str == "+0000")
                {
                    d_local_time_str = " UTC";
                }
            else
                {
                    d_local_time_str = " (UTC " + utc_diff_str.substr(0, 3) + ":" + utc_diff_str.substr(3, 2) + ")";
                }
        }
    else
        {
            d_local_time_str = std::string(" ") + time_zone_abrv + " (UTC " + utc_diff_str.substr(0, 3) + ":" + utc_diff_str.substr(3, 2) + ")";
        }

    // user PVT solver
    d_user_pvt_solver = std::make_shared<Rtklib_Solver>(static_cast<int32_t>(nchannels), dump_ls_pvt_filename, d_dump, d_dump_mat, rtk);
    d_user_pvt_solver->set_averaging_depth(1);

    // internal PVT solver, mainly used to estimate the receiver clock
    rtk_t internal_rtk = rtk;
    internal_rtk.opt.mode = PMODE_SINGLE;  // use single positioning mode in internal PVT solver
    d_internal_pvt_solver = std::make_shared<Rtklib_Solver>(static_cast<int32_t>(nchannels), dump_ls_pvt_filename, false, false, internal_rtk);
    d_internal_pvt_solver->set_averaging_depth(1);

    d_waiting_obs_block_rx_clock_offset_correction_msg = false;

    start = std::chrono::system_clock::now();
}


rtklib_pvt_gs::~rtklib_pvt_gs()
{
    msgctl(sysv_msqid, IPC_RMID, nullptr);
    try
        {
            if (d_xml_storage)
                {
                    // save GPS L2CM ephemeris to XML file
                    std::string file_name = xml_base_path + "gps_cnav_ephemeris.xml";
                    if (d_internal_pvt_solver->gps_cnav_ephemeris_map.empty() == false)
                        {
                            std::ofstream ofs;
                            try
                                {
                                    ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                                    boost::archive::xml_oarchive xml(ofs);
                                    xml << boost::serialization::make_nvp("GNSS-SDR_cnav_ephemeris_map", d_internal_pvt_solver->gps_cnav_ephemeris_map);
                                    LOG(INFO) << "Saved GPS L2CM or L5 Ephemeris map data";
                                }
                            catch (const boost::archive::archive_exception& e)
                                {
                                    LOG(WARNING) << e.what();
                                }
                            catch (const std::exception& e)
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
                    if (d_internal_pvt_solver->gps_ephemeris_map.empty() == false)
                        {
                            std::ofstream ofs;
                            try
                                {
                                    ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                                    boost::archive::xml_oarchive xml(ofs);
                                    xml << boost::serialization::make_nvp("GNSS-SDR_ephemeris_map", d_internal_pvt_solver->gps_ephemeris_map);
                                    LOG(INFO) << "Saved GPS L1 CA Ephemeris map data";
                                }
                            catch (const boost::archive::archive_exception& e)
                                {
                                    LOG(WARNING) << e.what();
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
                    if (d_internal_pvt_solver->galileo_ephemeris_map.empty() == false)
                        {
                            std::ofstream ofs;
                            try
                                {
                                    ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                                    boost::archive::xml_oarchive xml(ofs);
                                    xml << boost::serialization::make_nvp("GNSS-SDR_gal_ephemeris_map", d_internal_pvt_solver->galileo_ephemeris_map);
                                    LOG(INFO) << "Saved Galileo E1 Ephemeris map data";
                                }
                            catch (const boost::archive::archive_exception& e)
                                {
                                    LOG(WARNING) << e.what();
                                }
                            catch (const std::ofstream::failure& e)
                                {
                                    LOG(WARNING) << "Problem opening output XML file";
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
                    if (d_internal_pvt_solver->glonass_gnav_ephemeris_map.empty() == false)
                        {
                            std::ofstream ofs;
                            try
                                {
                                    ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                                    boost::archive::xml_oarchive xml(ofs);
                                    xml << boost::serialization::make_nvp("GNSS-SDR_gnav_ephemeris_map", d_internal_pvt_solver->glonass_gnav_ephemeris_map);
                                    LOG(INFO) << "Saved GLONASS GNAV Ephemeris map data";
                                }
                            catch (const boost::archive::archive_exception& e)
                                {
                                    LOG(WARNING) << e.what();
                                }
                            catch (const std::ofstream::failure& e)
                                {
                                    LOG(WARNING) << "Problem opening output XML file";
                                }
                            catch (const std::exception& e)
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
                    if (d_internal_pvt_solver->gps_utc_model.valid)
                        {
                            std::ofstream ofs;
                            try
                                {
                                    ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                                    boost::archive::xml_oarchive xml(ofs);
                                    xml << boost::serialization::make_nvp("GNSS-SDR_utc_model", d_internal_pvt_solver->gps_utc_model);
                                    LOG(INFO) << "Saved GPS UTC model parameters";
                                }
                            catch (const std::ofstream::failure& e)
                                {
                                    LOG(WARNING) << "Problem opening output XML file";
                                }
                            catch (const boost::archive::archive_exception& e)
                                {
                                    LOG(WARNING) << e.what();
                                }
                            catch (const std::exception& e)
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
                    if (d_internal_pvt_solver->galileo_utc_model.Delta_tLS_6 != 0.0)
                        {
                            std::ofstream ofs;
                            try
                                {
                                    ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                                    boost::archive::xml_oarchive xml(ofs);
                                    xml << boost::serialization::make_nvp("GNSS-SDR_gal_utc_model", d_internal_pvt_solver->galileo_utc_model);
                                    LOG(INFO) << "Saved Galileo UTC model parameters";
                                }
                            catch (const boost::archive::archive_exception& e)
                                {
                                    LOG(WARNING) << e.what();
                                }
                            catch (const std::ofstream::failure& e)
                                {
                                    LOG(WARNING) << "Problem opening output XML file";
                                }
                            catch (const std::exception& e)
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
                    if (d_internal_pvt_solver->gps_iono.valid == true)
                        {
                            std::ofstream ofs;
                            try
                                {
                                    ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                                    boost::archive::xml_oarchive xml(ofs);
                                    xml << boost::serialization::make_nvp("GNSS-SDR_iono_model", d_internal_pvt_solver->gps_iono);
                                    LOG(INFO) << "Saved GPS ionospheric model parameters";
                                }
                            catch (const boost::archive::archive_exception& e)
                                {
                                    LOG(WARNING) << e.what();
                                }
                            catch (const std::ofstream::failure& e)
                                {
                                    LOG(WARNING) << "Problem opening output XML file";
                                }
                            catch (const std::exception& e)
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
                    if (d_internal_pvt_solver->gps_cnav_iono.valid == true)
                        {
                            std::ofstream ofs;
                            try
                                {
                                    ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                                    boost::archive::xml_oarchive xml(ofs);
                                    xml << boost::serialization::make_nvp("GNSS-SDR_cnav_iono_model", d_internal_pvt_solver->gps_cnav_iono);
                                    LOG(INFO) << "Saved GPS CNAV ionospheric model parameters";
                                }
                            catch (const boost::archive::archive_exception& e)
                                {
                                    LOG(WARNING) << e.what();
                                }
                            catch (const std::ofstream::failure& e)
                                {
                                    LOG(WARNING) << "Problem opening output XML file";
                                }
                            catch (const std::exception& e)
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
                    if (d_internal_pvt_solver->galileo_iono.ai0_5 != 0.0)
                        {
                            std::ofstream ofs;
                            try
                                {
                                    ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                                    boost::archive::xml_oarchive xml(ofs);
                                    xml << boost::serialization::make_nvp("GNSS-SDR_gal_iono_model", d_internal_pvt_solver->galileo_iono);
                                    LOG(INFO) << "Saved Galileo ionospheric model parameters";
                                }
                            catch (const boost::archive::archive_exception& e)
                                {
                                    LOG(WARNING) << e.what();
                                }
                            catch (const std::ofstream::failure& e)
                                {
                                    LOG(WARNING) << "Problem opening output XML file";
                                }
                            catch (const std::exception& e)
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
                    if (d_internal_pvt_solver->gps_almanac_map.empty() == false)
                        {
                            std::ofstream ofs;
                            try
                                {
                                    ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                                    boost::archive::xml_oarchive xml(ofs);
                                    xml << boost::serialization::make_nvp("GNSS-SDR_gps_almanac_map", d_internal_pvt_solver->gps_almanac_map);
                                    LOG(INFO) << "Saved GPS almanac map data";
                                }
                            catch (const boost::archive::archive_exception& e)
                                {
                                    LOG(WARNING) << e.what();
                                }
                            catch (const std::ofstream::failure& e)
                                {
                                    LOG(WARNING) << "Problem opening output XML file";
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
                    if (d_internal_pvt_solver->galileo_almanac_map.empty() == false)
                        {
                            std::ofstream ofs;
                            try
                                {
                                    ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                                    boost::archive::xml_oarchive xml(ofs);
                                    xml << boost::serialization::make_nvp("GNSS-SDR_gal_almanac_map", d_internal_pvt_solver->galileo_almanac_map);
                                    LOG(INFO) << "Saved Galileo almanac data";
                                }
                            catch (const boost::archive::archive_exception& e)
                                {
                                    LOG(WARNING) << e.what();
                                }
                            catch (const std::ofstream::failure& e)
                                {
                                    LOG(WARNING) << "Problem opening output XML file";
                                }
                            catch (const std::exception& e)
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
                    if (d_internal_pvt_solver->gps_cnav_utc_model.valid)
                        {
                            std::ofstream ofs;
                            try
                                {
                                    ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                                    boost::archive::xml_oarchive xml(ofs);
                                    xml << boost::serialization::make_nvp("GNSS-SDR_cnav_utc_model", d_internal_pvt_solver->gps_cnav_utc_model);
                                    LOG(INFO) << "Saved GPS CNAV UTC model parameters";
                                }
                            catch (const boost::archive::archive_exception& e)
                                {
                                    LOG(WARNING) << e.what();
                                }
                            catch (const std::ofstream::failure& e)
                                {
                                    LOG(WARNING) << "Problem opening output XML file";
                                }
                            catch (const std::exception& e)
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
                    if (d_internal_pvt_solver->glonass_gnav_ephemeris_map.empty() == false)
                        {
                            std::ofstream ofs;
                            try
                                {
                                    ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                                    boost::archive::xml_oarchive xml(ofs);
                                    xml << boost::serialization::make_nvp("GNSS-SDR_gnav_ephemeris_map", d_internal_pvt_solver->glonass_gnav_ephemeris_map);
                                    LOG(INFO) << "Saved GLONASS GNAV ephemeris map data";
                                }
                            catch (const boost::archive::archive_exception& e)
                                {
                                    LOG(WARNING) << e.what();
                                }
                            catch (const std::ofstream::failure& e)
                                {
                                    LOG(WARNING) << "Problem opening output XML file";
                                }
                            catch (const std::exception& e)
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
                    if (d_internal_pvt_solver->glonass_gnav_utc_model.valid)
                        {
                            std::ofstream ofs;
                            try
                                {
                                    ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                                    boost::archive::xml_oarchive xml(ofs);
                                    xml << boost::serialization::make_nvp("GNSS-SDR_gnav_utc_model", d_internal_pvt_solver->glonass_gnav_utc_model);
                                    LOG(INFO) << "Saved GLONASS UTC model parameters";
                                }
                            catch (const boost::archive::archive_exception& e)
                                {
                                    LOG(WARNING) << e.what();
                                }
                            catch (const std::ofstream::failure& e)
                                {
                                    LOG(WARNING) << "Problem opening output XML file";
                                }
                            catch (const std::exception& e)
                                {
                                    LOG(WARNING) << e.what();
                                }
                        }
                    else
                        {
                            LOG(INFO) << "Failed to save GLONASS GNAV ephemeris, not valid data";
                        }

                    // save BeiDou DNAV ephemeris to XML file
                    file_name = xml_base_path + "bds_dnav_ephemeris.xml";
                    if (d_internal_pvt_solver->beidou_dnav_ephemeris_map.empty() == false)
                        {
                            std::ofstream ofs;
                            try
                                {
                                    ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                                    boost::archive::xml_oarchive xml(ofs);
                                    xml << boost::serialization::make_nvp("GNSS-SDR_bds_dnav_ephemeris_map", d_internal_pvt_solver->beidou_dnav_ephemeris_map);
                                    LOG(INFO) << "Saved BeiDou DNAV Ephemeris map data";
                                }
                            catch (const boost::archive::archive_exception& e)
                                {
                                    LOG(WARNING) << e.what();
                                }
                            catch (const std::ofstream::failure& e)
                                {
                                    LOG(WARNING) << "Problem opening output XML file";
                                }
                            catch (const std::exception& e)
                                {
                                    LOG(WARNING) << e.what();
                                }
                        }
                    else
                        {
                            LOG(INFO) << "Failed to save BeiDou DNAV Ephemeris, map is empty";
                        }

                    // Save BeiDou DNAV iono parameters
                    file_name = xml_base_path + "bds_dnav_iono.xml";
                    if (d_internal_pvt_solver->beidou_dnav_iono.valid)
                        {
                            std::ofstream ofs;
                            try
                                {
                                    ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                                    boost::archive::xml_oarchive xml(ofs);
                                    xml << boost::serialization::make_nvp("GNSS-SDR_bds_dnav_iono_model", d_internal_pvt_solver->beidou_dnav_iono);
                                    LOG(INFO) << "Saved BeiDou DNAV ionospheric model parameters";
                                }
                            catch (const boost::archive::archive_exception& e)
                                {
                                    LOG(WARNING) << e.what();
                                }
                            catch (const std::ofstream::failure& e)
                                {
                                    LOG(WARNING) << "Problem opening output XML file";
                                }
                            catch (const std::exception& e)
                                {
                                    LOG(WARNING) << e.what();
                                }
                        }
                    else
                        {
                            LOG(INFO) << "Failed to save BeiDou DNAV ionospheric model parameters, not valid data";
                        }

                    // save BeiDou DNAV almanac to XML file
                    file_name = xml_base_path + "bds_dnav_almanac.xml";
                    if (d_internal_pvt_solver->beidou_dnav_almanac_map.empty() == false)
                        {
                            std::ofstream ofs;
                            try
                                {
                                    ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                                    boost::archive::xml_oarchive xml(ofs);
                                    xml << boost::serialization::make_nvp("GNSS-SDR_bds_dnav_almanac_map", d_internal_pvt_solver->beidou_dnav_almanac_map);
                                    LOG(INFO) << "Saved BeiDou DNAV almanac map data";
                                }
                            catch (const boost::archive::archive_exception& e)
                                {
                                    LOG(WARNING) << e.what();
                                }
                            catch (const std::ofstream::failure& e)
                                {
                                    LOG(WARNING) << "Problem opening output XML file";
                                }
                            catch (const std::exception& e)
                                {
                                    LOG(WARNING) << e.what();
                                }
                        }
                    else
                        {
                            LOG(INFO) << "Failed to save BeiDou DNAV almanac, map is empty";
                        }

                    // Save BeiDou UTC model parameters
                    file_name = xml_base_path + "bds_dnav_utc_model.xml";
                    if (d_internal_pvt_solver->beidou_dnav_utc_model.valid)
                        {
                            std::ofstream ofs;
                            try
                                {
                                    ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                                    boost::archive::xml_oarchive xml(ofs);
                                    xml << boost::serialization::make_nvp("GNSS-SDR_bds_dnav_utc_model", d_internal_pvt_solver->beidou_dnav_utc_model);
                                    LOG(INFO) << "Saved BeiDou DNAV UTC model parameters";
                                }
                            catch (const boost::archive::archive_exception& e)
                                {
                                    LOG(WARNING) << e.what();
                                }
                            catch (const std::ofstream::failure& e)
                                {
                                    LOG(WARNING) << "Problem opening output XML file";
                                }
                            catch (const std::exception& e)
                                {
                                    LOG(WARNING) << e.what();
                                }
                        }
                    else
                        {
                            LOG(INFO) << "Failed to save BeiDou DNAV UTC model parameters, not valid data";
                        }
                }
        }
    catch (std::length_error& e)
        {
            LOG(WARNING) << e.what();
        }
}


void rtklib_pvt_gs::msg_handler_telemetry(const pmt::pmt_t& msg)
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
                    if (b_rinex_header_written)  // The header is already written, we can now log the navigation message data
                        {
                            bool new_annotation = false;
                            if (d_internal_pvt_solver->gps_ephemeris_map.find(gps_eph->i_satellite_PRN) == d_internal_pvt_solver->gps_ephemeris_map.cend())
                                {
                                    new_annotation = true;
                                }
                            else
                                {
                                    if (d_internal_pvt_solver->gps_ephemeris_map[gps_eph->i_satellite_PRN].d_Toe != gps_eph->d_Toe)
                                        {
                                            new_annotation = true;
                                        }
                                }
                            if (new_annotation == true)
                                {
                                    // New record!
                                    std::map<int32_t, Gps_Ephemeris> new_eph;
                                    std::map<int32_t, Galileo_Ephemeris> new_gal_eph;
                                    std::map<int32_t, Glonass_Gnav_Ephemeris> new_glo_eph;
                                    new_eph[gps_eph->i_satellite_PRN] = *gps_eph;
                                    switch (type_of_rx)
                                        {
                                        case 1:  // GPS L1 C/A only
                                            rp->log_rinex_nav(rp->navFile, new_eph);
                                            break;
                                        case 8:  // L1+L5
                                            rp->log_rinex_nav(rp->navFile, new_eph);
                                            break;
                                        case 11:  // GPS L1 C/A + Galileo E5b
                                            rp->log_rinex_nav(rp->navMixFile, new_eph, new_gal_eph);
                                            break;
                                        case 26:  // GPS L1 C/A + GLONASS L1 C/A
                                            if (d_rinex_version == 3)
                                                {
                                                    rp->log_rinex_nav(rp->navMixFile, new_eph, new_glo_eph);
                                                }
                                            if (d_rinex_version == 2)
                                                {
                                                    rp->log_rinex_nav(rp->navFile, new_glo_eph);
                                                }
                                            break;
                                        case 29:  //  GPS L1 C/A + GLONASS L2 C/A
                                            if (d_rinex_version == 3)
                                                {
                                                    rp->log_rinex_nav(rp->navMixFile, new_eph, new_glo_eph);
                                                }
                                            if (d_rinex_version == 2)
                                                {
                                                    rp->log_rinex_nav(rp->navFile, new_eph);
                                                }
                                            break;
                                        case 32:  // L1+E1+L5+E5a
                                            rp->log_rinex_nav(rp->navMixFile, new_eph, new_gal_eph);
                                            break;
                                        case 33:  // L1+E1+E5a
                                            rp->log_rinex_nav(rp->navMixFile, new_eph, new_gal_eph);
                                            break;
                                        default:
                                            break;
                                        }
                                }
                        }
                    d_internal_pvt_solver->gps_ephemeris_map[gps_eph->i_satellite_PRN] = *gps_eph;
                    d_user_pvt_solver->gps_ephemeris_map[gps_eph->i_satellite_PRN] = *gps_eph;
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Gps_Iono>))
                {
                    // ### GPS IONO ###
                    std::shared_ptr<Gps_Iono> gps_iono;
                    gps_iono = boost::any_cast<std::shared_ptr<Gps_Iono>>(pmt::any_ref(msg));
                    d_internal_pvt_solver->gps_iono = *gps_iono;
                    d_user_pvt_solver->gps_iono = *gps_iono;
                    DLOG(INFO) << "New IONO record has arrived ";
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Gps_Utc_Model>))
                {
                    // ### GPS UTC MODEL ###
                    std::shared_ptr<Gps_Utc_Model> gps_utc_model;
                    gps_utc_model = boost::any_cast<std::shared_ptr<Gps_Utc_Model>>(pmt::any_ref(msg));
                    d_internal_pvt_solver->gps_utc_model = *gps_utc_model;
                    d_user_pvt_solver->gps_utc_model = *gps_utc_model;
                    DLOG(INFO) << "New UTC record has arrived ";
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Gps_CNAV_Ephemeris>))
                {
                    // ### GPS CNAV message ###
                    std::shared_ptr<Gps_CNAV_Ephemeris> gps_cnav_ephemeris;
                    gps_cnav_ephemeris = boost::any_cast<std::shared_ptr<Gps_CNAV_Ephemeris>>(pmt::any_ref(msg));
                    // update/insert new ephemeris record to the global ephemeris map
                    if (b_rinex_header_written)  // The header is already written, we can now log the navigation message data
                        {
                            bool new_annotation = false;
                            if (d_internal_pvt_solver->gps_cnav_ephemeris_map.find(gps_cnav_ephemeris->i_satellite_PRN) == d_internal_pvt_solver->gps_cnav_ephemeris_map.cend())
                                {
                                    new_annotation = true;
                                }
                            else
                                {
                                    if (d_internal_pvt_solver->gps_cnav_ephemeris_map[gps_cnav_ephemeris->i_satellite_PRN].d_Toe1 != gps_cnav_ephemeris->d_Toe1)
                                        {
                                            new_annotation = true;
                                        }
                                }
                            if (new_annotation == true)
                                {
                                    // New record!
                                    std::map<int32_t, Galileo_Ephemeris> new_gal_eph;
                                    std::map<int32_t, Gps_CNAV_Ephemeris> new_cnav_eph;
                                    std::map<int32_t, Glonass_Gnav_Ephemeris> new_glo_eph;
                                    new_cnav_eph[gps_cnav_ephemeris->i_satellite_PRN] = *gps_cnav_ephemeris;
                                    switch (type_of_rx)
                                        {
                                        case 2:  // GPS L2C only
                                            rp->log_rinex_nav(rp->navFile, new_cnav_eph);
                                            break;
                                        case 3:  // GPS L5 only
                                            rp->log_rinex_nav(rp->navFile, new_cnav_eph);
                                            break;
                                        case 7:  // GPS L1 C/A + GPS L2C
                                            rp->log_rinex_nav(rp->navFile, new_cnav_eph);
                                            break;
                                        case 13:  //  L5+E5a
                                            rp->log_rinex_nav(rp->navFile, new_cnav_eph, new_gal_eph);
                                            break;
                                        case 28:  //  GPS L2C + GLONASS L1 C/A
                                            rp->log_rinex_nav(rp->navMixFile, new_cnav_eph, new_glo_eph);
                                            break;
                                        case 31:  //  GPS L2C + GLONASS L2 C/A
                                            rp->log_rinex_nav(rp->navMixFile, new_cnav_eph, new_glo_eph);
                                            break;
                                        default:
                                            break;
                                        }
                                }
                        }
                    d_internal_pvt_solver->gps_cnav_ephemeris_map[gps_cnav_ephemeris->i_satellite_PRN] = *gps_cnav_ephemeris;
                    d_user_pvt_solver->gps_cnav_ephemeris_map[gps_cnav_ephemeris->i_satellite_PRN] = *gps_cnav_ephemeris;
                    DLOG(INFO) << "New GPS CNAV ephemeris record has arrived ";
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Gps_CNAV_Iono>))
                {
                    // ### GPS CNAV IONO ###
                    std::shared_ptr<Gps_CNAV_Iono> gps_cnav_iono;
                    gps_cnav_iono = boost::any_cast<std::shared_ptr<Gps_CNAV_Iono>>(pmt::any_ref(msg));
                    d_internal_pvt_solver->gps_cnav_iono = *gps_cnav_iono;
                    d_user_pvt_solver->gps_cnav_iono = *gps_cnav_iono;
                    DLOG(INFO) << "New CNAV IONO record has arrived ";
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Gps_CNAV_Utc_Model>))
                {
                    // ### GPS CNAV UTC MODEL ###
                    std::shared_ptr<Gps_CNAV_Utc_Model> gps_cnav_utc_model;
                    gps_cnav_utc_model = boost::any_cast<std::shared_ptr<Gps_CNAV_Utc_Model>>(pmt::any_ref(msg));
                    d_internal_pvt_solver->gps_cnav_utc_model = *gps_cnav_utc_model;
                    d_user_pvt_solver->gps_cnav_utc_model = *gps_cnav_utc_model;
                    DLOG(INFO) << "New CNAV UTC record has arrived ";
                }

            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Gps_Almanac>))
                {
                    // ### GPS ALMANAC ###
                    std::shared_ptr<Gps_Almanac> gps_almanac;
                    gps_almanac = boost::any_cast<std::shared_ptr<Gps_Almanac>>(pmt::any_ref(msg));
                    d_internal_pvt_solver->gps_almanac_map[gps_almanac->i_satellite_PRN] = *gps_almanac;
                    d_user_pvt_solver->gps_almanac_map[gps_almanac->i_satellite_PRN] = *gps_almanac;
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
                    if (b_rinex_header_written)  // The header is already written, we can now log the navigation message data
                        {
                            bool new_annotation = false;
                            if (d_internal_pvt_solver->galileo_ephemeris_map.find(galileo_eph->i_satellite_PRN) == d_internal_pvt_solver->galileo_ephemeris_map.cend())
                                {
                                    new_annotation = true;
                                }
                            else
                                {
                                    if (d_internal_pvt_solver->galileo_ephemeris_map[galileo_eph->i_satellite_PRN].t0e_1 != galileo_eph->t0e_1)
                                        {
                                            new_annotation = true;
                                        }
                                }
                            if (new_annotation == true)
                                {
                                    // New record!
                                    std::map<int32_t, Galileo_Ephemeris> new_gal_eph;
                                    std::map<int32_t, Gps_CNAV_Ephemeris> new_cnav_eph;
                                    std::map<int32_t, Gps_Ephemeris> new_eph;
                                    std::map<int32_t, Glonass_Gnav_Ephemeris> new_glo_eph;
                                    new_gal_eph[galileo_eph->i_satellite_PRN] = *galileo_eph;
                                    switch (type_of_rx)
                                        {
                                        case 6:  // Galileo E5b only
                                            rp->log_rinex_nav(rp->navGalFile, new_gal_eph);
                                            break;
                                        case 11:  //  GPS L1 C/A + Galileo E5b
                                            rp->log_rinex_nav(rp->navMixFile, new_eph, new_gal_eph);
                                            break;
                                        case 13:  //  L5+E5a
                                            rp->log_rinex_nav(rp->navFile, new_cnav_eph, new_gal_eph);
                                            break;
                                        case 15:  //  Galileo E1B + Galileo E5b
                                            rp->log_rinex_nav(rp->navGalFile, new_gal_eph);
                                            break;
                                        case 27:  //  Galileo E1B + GLONASS L1 C/A
                                            rp->log_rinex_nav(rp->navMixFile, new_gal_eph, new_glo_eph);
                                            break;
                                        case 30:  //  Galileo E1B + GLONASS L2 C/A
                                            rp->log_rinex_nav(rp->navMixFile, new_gal_eph, new_glo_eph);
                                            break;
                                        case 32:  // L1+E1+L5+E5a
                                            rp->log_rinex_nav(rp->navMixFile, new_eph, new_gal_eph);
                                            break;
                                        case 33:  // L1+E1+E5a
                                            rp->log_rinex_nav(rp->navMixFile, new_eph, new_gal_eph);
                                            break;
                                        default:
                                            break;
                                        }
                                }
                        }
                    d_internal_pvt_solver->galileo_ephemeris_map[galileo_eph->i_satellite_PRN] = *galileo_eph;
                    d_user_pvt_solver->galileo_ephemeris_map[galileo_eph->i_satellite_PRN] = *galileo_eph;
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Galileo_Iono>))
                {
                    // ### Galileo IONO ###
                    std::shared_ptr<Galileo_Iono> galileo_iono;
                    galileo_iono = boost::any_cast<std::shared_ptr<Galileo_Iono>>(pmt::any_ref(msg));
                    d_internal_pvt_solver->galileo_iono = *galileo_iono;
                    d_user_pvt_solver->galileo_iono = *galileo_iono;
                    DLOG(INFO) << "New IONO record has arrived ";
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Galileo_Utc_Model>))
                {
                    // ### Galileo UTC MODEL ###
                    std::shared_ptr<Galileo_Utc_Model> galileo_utc_model;
                    galileo_utc_model = boost::any_cast<std::shared_ptr<Galileo_Utc_Model>>(pmt::any_ref(msg));
                    d_internal_pvt_solver->galileo_utc_model = *galileo_utc_model;
                    d_user_pvt_solver->galileo_utc_model = *galileo_utc_model;
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

                    if (sv1.i_satellite_PRN != 0)
                        {
                            d_internal_pvt_solver->galileo_almanac_map[sv1.i_satellite_PRN] = sv1;
                            d_user_pvt_solver->galileo_almanac_map[sv1.i_satellite_PRN] = sv1;
                        }
                    if (sv2.i_satellite_PRN != 0)
                        {
                            d_internal_pvt_solver->galileo_almanac_map[sv2.i_satellite_PRN] = sv2;
                            d_user_pvt_solver->galileo_almanac_map[sv2.i_satellite_PRN] = sv2;
                        }
                    if (sv3.i_satellite_PRN != 0)
                        {
                            d_internal_pvt_solver->galileo_almanac_map[sv3.i_satellite_PRN] = sv3;
                            d_user_pvt_solver->galileo_almanac_map[sv3.i_satellite_PRN] = sv3;
                        }
                    DLOG(INFO) << "New Galileo Almanac data have arrived ";
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Galileo_Almanac>))
                {
                    // ### Galileo Almanac ###
                    std::shared_ptr<Galileo_Almanac> galileo_alm;
                    galileo_alm = boost::any_cast<std::shared_ptr<Galileo_Almanac>>(pmt::any_ref(msg));
                    // update/insert new almanac record to the global almanac map
                    d_internal_pvt_solver->galileo_almanac_map[galileo_alm->i_satellite_PRN] = *galileo_alm;
                    d_user_pvt_solver->galileo_almanac_map[galileo_alm->i_satellite_PRN] = *galileo_alm;
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
                    if (b_rinex_header_written)  // The header is already written, we can now log the navigation message data
                        {
                            bool new_annotation = false;
                            if (d_internal_pvt_solver->glonass_gnav_ephemeris_map.find(glonass_gnav_eph->i_satellite_PRN) == d_internal_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                {
                                    new_annotation = true;
                                }
                            else
                                {
                                    if (d_internal_pvt_solver->glonass_gnav_ephemeris_map[glonass_gnav_eph->i_satellite_PRN].d_t_b != glonass_gnav_eph->d_t_b)
                                        {
                                            new_annotation = true;
                                        }
                                }
                            if (new_annotation == true)
                                {
                                    // New record!
                                    std::map<int32_t, Galileo_Ephemeris> new_gal_eph;
                                    std::map<int32_t, Gps_CNAV_Ephemeris> new_cnav_eph;
                                    std::map<int32_t, Gps_Ephemeris> new_eph;
                                    std::map<int32_t, Glonass_Gnav_Ephemeris> new_glo_eph;
                                    new_glo_eph[glonass_gnav_eph->i_satellite_PRN] = *glonass_gnav_eph;
                                    switch (type_of_rx)
                                        {
                                        case 25:  // GLONASS L1 C/A + GLONASS L2 C/A
                                            rp->log_rinex_nav(rp->navGloFile, new_glo_eph);
                                            break;
                                        case 26:  //  GPS L1 C/A + GLONASS L1 C/A
                                            if (d_rinex_version == 3)
                                                {
                                                    rp->log_rinex_nav(rp->navMixFile, new_eph, new_glo_eph);
                                                }
                                            if (d_rinex_version == 2)
                                                {
                                                    rp->log_rinex_nav(rp->navGloFile, new_glo_eph);
                                                }
                                            break;
                                        case 27:  //  Galileo E1B + GLONASS L1 C/A
                                            rp->log_rinex_nav(rp->navMixFile, new_gal_eph, new_glo_eph);
                                            break;
                                        case 28:  //  GPS L2C + GLONASS L1 C/A
                                            rp->log_rinex_nav(rp->navMixFile, new_cnav_eph, new_glo_eph);
                                            break;
                                        case 29:  //  GPS L1 C/A + GLONASS L2 C/A
                                            if (d_rinex_version == 3)
                                                {
                                                    rp->log_rinex_nav(rp->navMixFile, new_eph, new_glo_eph);
                                                }
                                            if (d_rinex_version == 2)
                                                {
                                                    rp->log_rinex_nav(rp->navGloFile, new_glo_eph);
                                                }
                                            break;
                                        case 30:  //  Galileo E1B + GLONASS L2 C/A
                                            rp->log_rinex_nav(rp->navMixFile, new_gal_eph, new_glo_eph);
                                            break;
                                        case 31:  //  GPS L2C + GLONASS L2 C/A
                                            rp->log_rinex_nav(rp->navMixFile, new_cnav_eph, new_glo_eph);
                                            break;
                                        default:
                                            break;
                                        }
                                }
                        }
                    d_internal_pvt_solver->glonass_gnav_ephemeris_map[glonass_gnav_eph->i_satellite_PRN] = *glonass_gnav_eph;
                    d_user_pvt_solver->glonass_gnav_ephemeris_map[glonass_gnav_eph->i_satellite_PRN] = *glonass_gnav_eph;
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Glonass_Gnav_Utc_Model>))
                {
                    // ### GLONASS GNAV UTC MODEL ###
                    std::shared_ptr<Glonass_Gnav_Utc_Model> glonass_gnav_utc_model;
                    glonass_gnav_utc_model = boost::any_cast<std::shared_ptr<Glonass_Gnav_Utc_Model>>(pmt::any_ref(msg));
                    d_internal_pvt_solver->glonass_gnav_utc_model = *glonass_gnav_utc_model;
                    d_user_pvt_solver->glonass_gnav_utc_model = *glonass_gnav_utc_model;
                    DLOG(INFO) << "New GLONASS GNAV UTC record has arrived ";
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Glonass_Gnav_Almanac>))
                {
                    // ### GLONASS GNAV Almanac ###
                    std::shared_ptr<Glonass_Gnav_Almanac> glonass_gnav_almanac;
                    glonass_gnav_almanac = boost::any_cast<std::shared_ptr<Glonass_Gnav_Almanac>>(pmt::any_ref(msg));
                    d_internal_pvt_solver->glonass_gnav_almanac = *glonass_gnav_almanac;
                    d_user_pvt_solver->glonass_gnav_almanac = *glonass_gnav_almanac;
                    DLOG(INFO) << "New GLONASS GNAV Almanac has arrived "
                               << ", GLONASS GNAV Slot Number =" << glonass_gnav_almanac->d_n_A;
                }

            // ************* BeiDou telemetry *****************
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Beidou_Dnav_Ephemeris>))
                {
                    // ### Beidou EPHEMERIS ###
                    std::shared_ptr<Beidou_Dnav_Ephemeris> bds_dnav_eph;
                    bds_dnav_eph = boost::any_cast<std::shared_ptr<Beidou_Dnav_Ephemeris>>(pmt::any_ref(msg));
                    DLOG(INFO) << "Ephemeris record has arrived from SAT ID "
                               << bds_dnav_eph->i_satellite_PRN << " (Block "
                               << bds_dnav_eph->satelliteBlock[bds_dnav_eph->i_satellite_PRN] << ")"
                               << "inserted with Toe=" << bds_dnav_eph->d_Toe << " and BDS Week="
                               << bds_dnav_eph->i_BEIDOU_week;
                    // update/insert new ephemeris record to the global ephemeris map
                    if (b_rinex_header_written)  // The header is already written, we can now log the navigation message data
                        {
                            bool new_annotation = false;
                            if (d_internal_pvt_solver->beidou_dnav_ephemeris_map.find(bds_dnav_eph->i_satellite_PRN) == d_internal_pvt_solver->beidou_dnav_ephemeris_map.cend())
                                {
                                    new_annotation = true;
                                }
                            else
                                {
                                    if (d_internal_pvt_solver->beidou_dnav_ephemeris_map[bds_dnav_eph->i_satellite_PRN].d_Toc != bds_dnav_eph->d_Toc)
                                        {
                                            new_annotation = true;
                                        }
                                }
                            if (new_annotation == true)
                                {
                                    // New record!
                                    std::map<int32_t, Beidou_Dnav_Ephemeris> new_bds_eph;
                                    new_bds_eph[bds_dnav_eph->i_satellite_PRN] = *bds_dnav_eph;
                                    switch (type_of_rx)
                                        {
                                        case 500:  // BDS B1I only
                                            rp->log_rinex_nav(rp->navFile, new_bds_eph);
                                            break;
                                        default:
                                            break;
                                        }
                                }
                        }
                    d_internal_pvt_solver->beidou_dnav_ephemeris_map[bds_dnav_eph->i_satellite_PRN] = *bds_dnav_eph;
                    d_user_pvt_solver->beidou_dnav_ephemeris_map[bds_dnav_eph->i_satellite_PRN] = *bds_dnav_eph;
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Beidou_Dnav_Iono>))
                {
                    // ### BeiDou IONO ###
                    std::shared_ptr<Beidou_Dnav_Iono> bds_dnav_iono;
                    bds_dnav_iono = boost::any_cast<std::shared_ptr<Beidou_Dnav_Iono>>(pmt::any_ref(msg));
                    d_internal_pvt_solver->beidou_dnav_iono = *bds_dnav_iono;
                    d_user_pvt_solver->beidou_dnav_iono = *bds_dnav_iono;
                    DLOG(INFO) << "New BeiDou DNAV IONO record has arrived ";
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Beidou_Dnav_Utc_Model>))
                {
                    // ### BeiDou UTC MODEL ###
                    std::shared_ptr<Beidou_Dnav_Utc_Model> bds_dnav_utc_model;
                    bds_dnav_utc_model = boost::any_cast<std::shared_ptr<Beidou_Dnav_Utc_Model>>(pmt::any_ref(msg));
                    d_internal_pvt_solver->beidou_dnav_utc_model = *bds_dnav_utc_model;
                    d_user_pvt_solver->beidou_dnav_utc_model = *bds_dnav_utc_model;
                    DLOG(INFO) << "New BeiDou DNAV UTC record has arrived ";
                }
            else if (pmt::any_ref(msg).type() == typeid(std::shared_ptr<Beidou_Dnav_Almanac>))
                {
                    // ### BeiDou ALMANAC ###
                    std::shared_ptr<Beidou_Dnav_Almanac> bds_dnav_almanac;
                    bds_dnav_almanac = boost::any_cast<std::shared_ptr<Beidou_Dnav_Almanac>>(pmt::any_ref(msg));
                    d_internal_pvt_solver->beidou_dnav_almanac_map[bds_dnav_almanac->i_satellite_PRN] = *bds_dnav_almanac;
                    d_user_pvt_solver->beidou_dnav_almanac_map[bds_dnav_almanac->i_satellite_PRN] = *bds_dnav_almanac;
                    DLOG(INFO) << "New BeiDou DNAV almanac record has arrived ";
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


std::map<int, Gps_Ephemeris> rtklib_pvt_gs::get_gps_ephemeris_map() const
{
    return d_internal_pvt_solver->gps_ephemeris_map;
}


std::map<int, Gps_Almanac> rtklib_pvt_gs::get_gps_almanac_map() const
{
    return d_internal_pvt_solver->gps_almanac_map;
}


std::map<int, Galileo_Ephemeris> rtklib_pvt_gs::get_galileo_ephemeris_map() const
{
    return d_internal_pvt_solver->galileo_ephemeris_map;
}


std::map<int, Galileo_Almanac> rtklib_pvt_gs::get_galileo_almanac_map() const
{
    return d_internal_pvt_solver->galileo_almanac_map;
}


std::map<int, Beidou_Dnav_Ephemeris> rtklib_pvt_gs::get_beidou_dnav_ephemeris_map() const
{
    return d_internal_pvt_solver->beidou_dnav_ephemeris_map;
}


std::map<int, Beidou_Dnav_Almanac> rtklib_pvt_gs::get_beidou_dnav_almanac_map() const
{
    return d_internal_pvt_solver->beidou_dnav_almanac_map;
}


void rtklib_pvt_gs::clear_ephemeris()
{
    d_internal_pvt_solver->gps_ephemeris_map.clear();
    d_internal_pvt_solver->gps_almanac_map.clear();
    d_internal_pvt_solver->galileo_ephemeris_map.clear();
    d_internal_pvt_solver->galileo_almanac_map.clear();
    d_internal_pvt_solver->beidou_dnav_ephemeris_map.clear();
    d_internal_pvt_solver->beidou_dnav_almanac_map.clear();

    d_user_pvt_solver->gps_ephemeris_map.clear();
    d_user_pvt_solver->gps_almanac_map.clear();
    d_user_pvt_solver->galileo_ephemeris_map.clear();
    d_user_pvt_solver->galileo_almanac_map.clear();
    d_user_pvt_solver->beidou_dnav_ephemeris_map.clear();
    d_user_pvt_solver->beidou_dnav_almanac_map.clear();
}


bool rtklib_pvt_gs::send_sys_v_ttff_msg(ttff_msgbuf ttff)
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


bool rtklib_pvt_gs::save_gnss_synchro_map_xml(const std::string& file_name)
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
            catch (const std::exception& e)
                {
                    LOG(WARNING) << e.what();
                    return false;
                }
            return true;
        }

    LOG(WARNING) << "Failed to save gnss_synchro, map is empty";
    return false;
}


bool rtklib_pvt_gs::load_gnss_synchro_map_xml(const std::string& file_name)
{
    // load from xml (boost serialize)
    std::ifstream ifs;
    try
        {
            ifs.open(file_name.c_str(), std::ifstream::binary | std::ifstream::in);
            boost::archive::xml_iarchive xml(ifs);
            gnss_observables_map.clear();
            xml >> boost::serialization::make_nvp("GNSS-SDR_gnss_synchro_map", gnss_observables_map);
            // std::cout << "Loaded gnss_synchro map data with " << gnss_synchro_map.size() << " pseudoranges" << std::endl;
        }
    catch (const std::exception& e)
        {
            std::cout << e.what() << "File: " << file_name;
            return false;
        }
    return true;
}


std::vector<std::string> rtklib_pvt_gs::split_string(const std::string& s, char delim) const
{
    std::vector<std::string> v;
    std::stringstream ss(s);
    std::string item;

    while (std::getline(ss, item, delim))
        {
            *(std::back_inserter(v)++) = item;
        }

    return v;
}


bool rtklib_pvt_gs::get_latest_PVT(double* longitude_deg,
    double* latitude_deg,
    double* height_m,
    double* ground_speed_kmh,
    double* course_over_ground_deg,
    time_t* UTC_time) const
{
    if (d_user_pvt_solver->is_valid_position())
        {
            *latitude_deg = d_user_pvt_solver->get_latitude();
            *longitude_deg = d_user_pvt_solver->get_longitude();
            *height_m = d_user_pvt_solver->get_height();
            *ground_speed_kmh = d_user_pvt_solver->get_speed_over_ground() * 3600.0 / 1000.0;
            *course_over_ground_deg = d_user_pvt_solver->get_course_over_ground();
            *UTC_time = convert_to_time_t(d_user_pvt_solver->get_position_UTC_time());

            return true;
        }

    return false;
}


void rtklib_pvt_gs::apply_rx_clock_offset(std::map<int, Gnss_Synchro>& observables_map,
    double rx_clock_offset_s)
{
    // apply corrections according to Rinex 3.04, Table 1: Observation Corrections for Receiver Clock Offset
    std::map<int, Gnss_Synchro>::iterator observables_iter;

    for (observables_iter = observables_map.begin(); observables_iter != observables_map.end(); observables_iter++)
        {
            // all observables in the map are valid
            observables_iter->second.RX_time -= rx_clock_offset_s;
            observables_iter->second.Pseudorange_m -= rx_clock_offset_s * SPEED_OF_LIGHT;

            switch (mapStringValues_[observables_iter->second.Signal])
                {
                case evGPS_1C:
                    observables_iter->second.Carrier_phase_rads -= rx_clock_offset_s * FREQ1;
                    break;
                case evGPS_L5:
                    observables_iter->second.Carrier_phase_rads -= rx_clock_offset_s * FREQ5;
                    break;
                case evSBAS_1C:
                    observables_iter->second.Carrier_phase_rads -= rx_clock_offset_s * FREQ1;
                    break;
                case evGAL_1B:
                    observables_iter->second.Carrier_phase_rads -= rx_clock_offset_s * FREQ1;
                    break;
                case evGAL_5X:
                    observables_iter->second.Carrier_phase_rads -= rx_clock_offset_s * FREQ5;
                    break;
                case evGPS_2S:
                    observables_iter->second.Carrier_phase_rads -= rx_clock_offset_s * FREQ2;
                    break;
                case evBDS_B3:
                    observables_iter->second.Carrier_phase_rads -= rx_clock_offset_s * FREQ3_BDS;
                    break;
                case evGLO_1G:
                    observables_iter->second.Carrier_phase_rads -= rx_clock_offset_s * FREQ1_GLO;
                    break;
                case evGLO_2G:
                    observables_iter->second.Carrier_phase_rads -= rx_clock_offset_s * FREQ2_GLO;
                    break;
                case evBDS_B1:
                    observables_iter->second.Carrier_phase_rads -= rx_clock_offset_s * FREQ1_BDS;
                    break;
                case evBDS_B2:
                    observables_iter->second.Carrier_phase_rads -= rx_clock_offset_s * FREQ2_BDS;
                    break;
                default:
                    break;
                }
        }
}


std::map<int, Gnss_Synchro> rtklib_pvt_gs::interpolate_observables(std::map<int, Gnss_Synchro>& observables_map_t0,
    std::map<int, Gnss_Synchro>& observables_map_t1,
    double rx_time_s)
{
    std::map<int, Gnss_Synchro> interp_observables_map;
    // Linear interpolation: y(t) = y(t0) + (y(t1) - y(t0)) * (t - t0) / (t1 - t0)

    // check TOW rollover
    double time_factor;
    if ((observables_map_t1.cbegin()->second.RX_time -
            observables_map_t0.cbegin()->second.RX_time) > 0)
        {
            time_factor = (rx_time_s - observables_map_t0.cbegin()->second.RX_time) /
                          (observables_map_t1.cbegin()->second.RX_time -
                              observables_map_t0.cbegin()->second.RX_time);
        }
    else
        {
            // TOW rollover situation
            time_factor = (604800000.0 + rx_time_s - observables_map_t0.cbegin()->second.RX_time) /
                          (604800000.0 + observables_map_t1.cbegin()->second.RX_time -
                              observables_map_t0.cbegin()->second.RX_time);
        }

    std::map<int, Gnss_Synchro>::const_iterator observables_iter;
    for (observables_iter = observables_map_t0.cbegin(); observables_iter != observables_map_t0.cend(); observables_iter++)
        {
            // 1. Check if the observable exist in t0 and t1
            // the map key is the channel ID (see work())
            try
                {
                    if (observables_map_t1.at(observables_iter->first).PRN == observables_iter->second.PRN)
                        {
                            interp_observables_map.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                            interp_observables_map.at(observables_iter->first).RX_time = rx_time_s;  // interpolation point
                            interp_observables_map.at(observables_iter->first).Pseudorange_m += (observables_map_t1.at(observables_iter->first).Pseudorange_m - observables_iter->second.Pseudorange_m) * time_factor;
                            interp_observables_map.at(observables_iter->first).Carrier_phase_rads += (observables_map_t1.at(observables_iter->first).Carrier_phase_rads - observables_iter->second.Carrier_phase_rads) * time_factor;
                            interp_observables_map.at(observables_iter->first).Carrier_Doppler_hz += (observables_map_t1.at(observables_iter->first).Carrier_Doppler_hz - observables_iter->second.Carrier_Doppler_hz) * time_factor;
                        }
                }
            catch (const std::out_of_range& oor)
                {
                    // observable does not exist in t1
                }
        }
    return interp_observables_map;
}


int rtklib_pvt_gs::work(int noutput_items, gr_vector_const_void_star& input_items,
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

            gnss_observables_map.clear();
            const auto** in = reinterpret_cast<const Gnss_Synchro**>(&input_items[0]);  // Get the input buffer pointer
            // ############ 1. READ PSEUDORANGES ####
            for (uint32_t i = 0; i < d_nchannels; i++)
                {
                    if (in[i][epoch].Flag_valid_pseudorange)
                        {
                            std::map<int, Gps_Ephemeris>::const_iterator tmp_eph_iter_gps = d_internal_pvt_solver->gps_ephemeris_map.find(in[i][epoch].PRN);
                            std::map<int, Galileo_Ephemeris>::const_iterator tmp_eph_iter_gal = d_internal_pvt_solver->galileo_ephemeris_map.find(in[i][epoch].PRN);
                            std::map<int, Gps_CNAV_Ephemeris>::const_iterator tmp_eph_iter_cnav = d_internal_pvt_solver->gps_cnav_ephemeris_map.find(in[i][epoch].PRN);
                            std::map<int, Glonass_Gnav_Ephemeris>::const_iterator tmp_eph_iter_glo_gnav = d_internal_pvt_solver->glonass_gnav_ephemeris_map.find(in[i][epoch].PRN);
                            std::map<int, Beidou_Dnav_Ephemeris>::const_iterator tmp_eph_iter_bds_dnav = d_internal_pvt_solver->beidou_dnav_ephemeris_map.find(in[i][epoch].PRN);

                            bool store_valid_observable = false;

                            if (tmp_eph_iter_gps != d_internal_pvt_solver->gps_ephemeris_map.cend())
                                {
                                    uint32_t prn_aux = tmp_eph_iter_gps->second.i_satellite_PRN;
                                    if ((prn_aux == in[i][epoch].PRN) and (std::string(in[i][epoch].Signal) == "1C"))
                                        {
                                            store_valid_observable = true;
                                        }
                                }
                            if (tmp_eph_iter_gal != d_internal_pvt_solver->galileo_ephemeris_map.cend())
                                {
                                    uint32_t prn_aux = tmp_eph_iter_gal->second.i_satellite_PRN;
                                    if ((prn_aux == in[i][epoch].PRN) and ((std::string(in[i][epoch].Signal) == "1B") or (std::string(in[i][epoch].Signal) == "5X")))
                                        {
                                            store_valid_observable = true;
                                        }
                                }
                            if (tmp_eph_iter_cnav != d_internal_pvt_solver->gps_cnav_ephemeris_map.cend())
                                {
                                    uint32_t prn_aux = tmp_eph_iter_cnav->second.i_satellite_PRN;
                                    if ((prn_aux == in[i][epoch].PRN) and ((std::string(in[i][epoch].Signal) == "2S") or (std::string(in[i][epoch].Signal) == "L5")))
                                        {
                                            store_valid_observable = true;
                                        }
                                }
                            if (tmp_eph_iter_glo_gnav != d_internal_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                {
                                    uint32_t prn_aux = tmp_eph_iter_glo_gnav->second.i_satellite_PRN;
                                    if ((prn_aux == in[i][epoch].PRN) and ((std::string(in[i][epoch].Signal) == "1G") or (std::string(in[i][epoch].Signal) == "2G")))
                                        {
                                            store_valid_observable = true;
                                        }
                                }
                            if (tmp_eph_iter_bds_dnav != d_internal_pvt_solver->beidou_dnav_ephemeris_map.cend())
                                {
                                    uint32_t prn_aux = tmp_eph_iter_bds_dnav->second.i_satellite_PRN;
                                    if ((prn_aux == in[i][epoch].PRN) and ((std::string(in[i][epoch].Signal) == "B1") or (std::string(in[i][epoch].Signal) == "B3")))
                                        {
                                            store_valid_observable = true;
                                        }
                                }

                            if (store_valid_observable)
                                {
                                    // store valid observables in a map.
                                    gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(i, in[i][epoch]));
                                }

                            if (b_rtcm_enabled)
                                {
                                    try
                                        {
                                            if (d_internal_pvt_solver->gps_ephemeris_map.empty() == false)
                                                {
                                                    if (tmp_eph_iter_gps != d_internal_pvt_solver->gps_ephemeris_map.cend())
                                                        {
                                                            d_rtcm_printer->lock_time(d_internal_pvt_solver->gps_ephemeris_map.find(in[i][epoch].PRN)->second, in[i][epoch].RX_time, in[i][epoch]);  // keep track of locking time
                                                        }
                                                }
                                            if (d_internal_pvt_solver->galileo_ephemeris_map.empty() == false)
                                                {
                                                    if (tmp_eph_iter_gal != d_internal_pvt_solver->galileo_ephemeris_map.cend())
                                                        {
                                                            d_rtcm_printer->lock_time(d_internal_pvt_solver->galileo_ephemeris_map.find(in[i][epoch].PRN)->second, in[i][epoch].RX_time, in[i][epoch]);  // keep track of locking time
                                                        }
                                                }
                                            if (d_internal_pvt_solver->gps_cnav_ephemeris_map.empty() == false)
                                                {
                                                    if (tmp_eph_iter_cnav != d_internal_pvt_solver->gps_cnav_ephemeris_map.cend())
                                                        {
                                                            d_rtcm_printer->lock_time(d_internal_pvt_solver->gps_cnav_ephemeris_map.find(in[i][epoch].PRN)->second, in[i][epoch].RX_time, in[i][epoch]);  // keep track of locking time
                                                        }
                                                }
                                            if (d_internal_pvt_solver->glonass_gnav_ephemeris_map.empty() == false)
                                                {
                                                    if (tmp_eph_iter_glo_gnav != d_internal_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                        {
                                                            d_rtcm_printer->lock_time(d_internal_pvt_solver->glonass_gnav_ephemeris_map.find(in[i][epoch].PRN)->second, in[i][epoch].RX_time, in[i][epoch]);  // keep track of locking time
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
                    // LOG(INFO) << "diff raw obs time: " << gnss_observables_map.cbegin()->second.RX_time * 1000.0 - old_time_debug;
                    // old_time_debug = gnss_observables_map.cbegin()->second.RX_time * 1000.0;
                    uint32_t current_RX_time_ms = 0;
                    // #### solve PVT and store the corrected observable set
                    if (d_internal_pvt_solver->get_PVT(gnss_observables_map, false))
                        {
                            double Rx_clock_offset_s = d_internal_pvt_solver->get_time_offset_s();
                            if (fabs(Rx_clock_offset_s) * 1000.0 > max_obs_block_rx_clock_offset_ms)
                                {
                                    if (!d_waiting_obs_block_rx_clock_offset_correction_msg)
                                        {
                                            this->message_port_pub(pmt::mp("pvt_to_observables"), pmt::make_any(Rx_clock_offset_s));
                                            d_waiting_obs_block_rx_clock_offset_correction_msg = true;
                                            LOG(INFO) << "Sent clock offset correction to observables: " << Rx_clock_offset_s << "[s]";
                                        }
                                }
                            else
                                {
                                    d_waiting_obs_block_rx_clock_offset_correction_msg = false;
                                    gnss_observables_map_t0 = gnss_observables_map_t1;
                                    apply_rx_clock_offset(gnss_observables_map, Rx_clock_offset_s);
                                    gnss_observables_map_t1 = gnss_observables_map;

                                    // ### select the rx_time and interpolate observables at that time
                                    if (!gnss_observables_map_t0.empty())
                                        {
                                            uint32_t t0_int_ms = static_cast<uint32_t>(gnss_observables_map_t0.cbegin()->second.RX_time * 1000.0);
                                            uint32_t adjust_next_20ms = 20 - t0_int_ms % 20;
                                            current_RX_time_ms = t0_int_ms + adjust_next_20ms;

                                            if (current_RX_time_ms % d_output_rate_ms == 0)
                                                {
                                                    d_rx_time = static_cast<double>(current_RX_time_ms) / 1000.0;
                                                    // std::cout << " obs time t0: " << gnss_observables_map_t0.cbegin()->second.RX_time
                                                    //           << " t1: " << gnss_observables_map_t1.cbegin()->second.RX_time
                                                    //           << " interp time: " << d_rx_time << std::endl;
                                                    gnss_observables_map = interpolate_observables(gnss_observables_map_t0,
                                                        gnss_observables_map_t1,
                                                        d_rx_time);
                                                    flag_compute_pvt_output = true;
                                                    // d_rx_time = current_RX_time;
                                                    // std::cout.precision(17);
                                                    // std::cout << "current_RX_time: " << current_RX_time << " map time: " << gnss_observables_map.begin()->second.RX_time << std::endl;
                                                }
                                        }
                                }
                        }
                    // debug code
                    // else
                    //     {
                    //         LOG(INFO) << "Internal PVT solver error";
                    //     }

                    // compute on the fly PVT solution
                    if (flag_compute_pvt_output == true)
                        {
                            if (d_user_pvt_solver->get_PVT(gnss_observables_map, false))
                                {
                                    double Rx_clock_offset_s = d_user_pvt_solver->get_time_offset_s();
                                    if (fabs(Rx_clock_offset_s) > 0.000001)  // 1us !!
                                        {
                                            LOG(INFO) << "Warning: Rx clock offset at interpolated RX time: " << Rx_clock_offset_s * 1000.0 << "[ms]"
                                                      << " at RX time: " << static_cast<uint32_t>(d_rx_time * 1000.0) << " [ms]";
                                        }
                                    else
                                        {
                                            DLOG(INFO) << "Rx clock offset at interpolated RX time: " << Rx_clock_offset_s * 1000.0 << "[s]"
                                                       << " at RX time: " << static_cast<uint32_t>(d_rx_time * 1000.0) << " [ms]";
                                            // Optional debug code: export observables snapshot for rtklib unit testing
                                            // std::cout << "step 1: save gnss_synchro map" << std::endl;
                                            // save_gnss_synchro_map_xml("./gnss_synchro_map.xml");
                                            // getchar(); // stop the execution
                                            // end debug
                                            if (d_display_rate_ms != 0)
                                                {
                                                    if (current_RX_time_ms % d_display_rate_ms == 0)
                                                        {
                                                            flag_display_pvt = true;
                                                        }
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

                                            if (first_fix == true)
                                                {
                                                    if (d_show_local_time_zone)
                                                        {
                                                            boost::posix_time::ptime time_first_solution = d_user_pvt_solver->get_position_UTC_time() + d_utc_diff_time;
                                                            std::cout << "First position fix at " << time_first_solution << d_local_time_str;
                                                        }
                                                    else
                                                        {
                                                            std::cout << "First position fix at " << d_user_pvt_solver->get_position_UTC_time() << " UTC";
                                                        }
                                                    std::cout << " is Lat = " << d_user_pvt_solver->get_latitude() << " [deg], Long = " << d_user_pvt_solver->get_longitude()
                                                              << " [deg], Height= " << d_user_pvt_solver->get_height() << " [m]" << std::endl;
                                                    ttff_msgbuf ttff;
                                                    ttff.mtype = 1;
                                                    end = std::chrono::system_clock::now();
                                                    std::chrono::duration<double> elapsed_seconds = end - start;
                                                    ttff.ttff = elapsed_seconds.count();
                                                    send_sys_v_ttff_msg(ttff);
                                                    first_fix = false;
                                                }
                                            if (d_kml_output_enabled)
                                                {
                                                    if (current_RX_time_ms % d_kml_rate_ms == 0)
                                                        {
                                                            d_kml_dump->print_position(d_user_pvt_solver, false);
                                                        }
                                                }
                                            if (d_gpx_output_enabled)
                                                {
                                                    if (current_RX_time_ms % d_gpx_rate_ms == 0)
                                                        {
                                                            d_gpx_dump->print_position(d_user_pvt_solver, false);
                                                        }
                                                }
                                            if (d_geojson_output_enabled)
                                                {
                                                    if (current_RX_time_ms % d_geojson_rate_ms == 0)
                                                        {
                                                            d_geojson_printer->print_position(d_user_pvt_solver, false);
                                                        }
                                                }
                                            if (d_nmea_output_file_enabled)
                                                {
                                                    if (current_RX_time_ms % d_nmea_rate_ms == 0)
                                                        {
                                                            d_nmea_printer->Print_Nmea_Line(d_user_pvt_solver, false);
                                                        }
                                                }

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
                                             *    13   |  Galileo E5a + GPS L5
                                             *    14   |  Galileo E1B + Galileo E5a
                                             *    15   |  Galileo E1B + Galileo E5b
                                             *    16   |  GPS L2C + GPS L5
                                             *    17   |  GPS L2C + Galileo E5a
                                             *    18   |  GPS L2C + Galileo E5b
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
                                             *    50   |  Beidou B1I
                                             *    51   |  Beidou B1I + GPS L1 C/A
                                             *    52   |  Beidou B1I + Galileo E1B
                                             *    53   |  Beidou B1I + GLONASS L1 C/A
                                             *    54   |  Beidou B1I + GPS L1 C/A + Galileo E1B
                                             *    55   |  Beidou B1I + GPS L1 C/A + GLONASS L1 C/A + Galileo E1B
                                             *    56   |  Beidou B1I + Beidou B3I
                                             *    60   |  Beidou B3I
                                             *    61   |  Beidou B3I + GPS L2C
                                             *    62   |  Beidou B3I + GLONASS L2 C/A
                                             *    63   |  Beidou B3I + GPS L2C + GLONASS L2 C/A
                                             */

                                            // ####################### RINEX FILES #################
                                            if (b_rinex_output_enabled)
                                                {
                                                    std::map<int, Galileo_Ephemeris>::const_iterator galileo_ephemeris_iter;
                                                    std::map<int, Gps_Ephemeris>::const_iterator gps_ephemeris_iter;
                                                    std::map<int, Gps_CNAV_Ephemeris>::const_iterator gps_cnav_ephemeris_iter;
                                                    std::map<int, Glonass_Gnav_Ephemeris>::const_iterator glonass_gnav_ephemeris_iter;
                                                    std::map<int, Beidou_Dnav_Ephemeris>::const_iterator beidou_dnav_ephemeris_iter;
                                                    if (!b_rinex_header_written)  //  & we have utc data in nav message!
                                                        {
                                                            galileo_ephemeris_iter = d_user_pvt_solver->galileo_ephemeris_map.cbegin();
                                                            gps_ephemeris_iter = d_user_pvt_solver->gps_ephemeris_map.cbegin();
                                                            gps_cnav_ephemeris_iter = d_user_pvt_solver->gps_cnav_ephemeris_map.cbegin();
                                                            glonass_gnav_ephemeris_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.cbegin();
                                                            beidou_dnav_ephemeris_iter = d_user_pvt_solver->beidou_dnav_ephemeris_map.cbegin();
                                                            switch (type_of_rx)
                                                                {
                                                                case 1:  // GPS L1 C/A only
                                                                    if (gps_ephemeris_iter != d_user_pvt_solver->gps_ephemeris_map.cend())
                                                                        {
                                                                            rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, d_rx_time);
                                                                            rp->rinex_nav_header(rp->navFile, d_user_pvt_solver->gps_iono, d_user_pvt_solver->gps_utc_model, gps_ephemeris_iter->second);
                                                                            rp->log_rinex_nav(rp->navFile, d_user_pvt_solver->gps_ephemeris_map);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }
                                                                    break;
                                                                case 2:  // GPS L2C only
                                                                    if (gps_cnav_ephemeris_iter != d_user_pvt_solver->gps_cnav_ephemeris_map.cend())
                                                                        {
                                                                            std::string signal("2S");
                                                                            rp->rinex_obs_header(rp->obsFile, gps_cnav_ephemeris_iter->second, d_rx_time, signal);
                                                                            rp->rinex_nav_header(rp->navFile, d_user_pvt_solver->gps_cnav_iono, d_user_pvt_solver->gps_cnav_utc_model);
                                                                            rp->log_rinex_nav(rp->navFile, d_user_pvt_solver->gps_cnav_ephemeris_map);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }
                                                                    break;
                                                                case 3:  // GPS L5 only
                                                                    if (gps_cnav_ephemeris_iter != d_user_pvt_solver->gps_cnav_ephemeris_map.cend())
                                                                        {
                                                                            std::string signal("L5");
                                                                            rp->rinex_obs_header(rp->obsFile, gps_cnav_ephemeris_iter->second, d_rx_time, signal);
                                                                            rp->rinex_nav_header(rp->navFile, d_user_pvt_solver->gps_cnav_iono, d_user_pvt_solver->gps_cnav_utc_model);
                                                                            rp->log_rinex_nav(rp->navFile, d_user_pvt_solver->gps_cnav_ephemeris_map);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }
                                                                    break;
                                                                case 4:  // Galileo E1B only
                                                                    if (galileo_ephemeris_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                        {
                                                                            rp->rinex_obs_header(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time);
                                                                            rp->rinex_nav_header(rp->navGalFile, d_user_pvt_solver->galileo_iono, d_user_pvt_solver->galileo_utc_model);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }
                                                                    break;
                                                                case 5:  // Galileo E5a only
                                                                    if (galileo_ephemeris_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                        {
                                                                            std::string signal("5X");
                                                                            rp->rinex_obs_header(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, signal);
                                                                            rp->rinex_nav_header(rp->navGalFile, d_user_pvt_solver->galileo_iono, d_user_pvt_solver->galileo_utc_model);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }
                                                                    break;
                                                                case 6:  // Galileo E5b only
                                                                    if (galileo_ephemeris_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                        {
                                                                            std::string signal("7X");
                                                                            rp->rinex_obs_header(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, signal);
                                                                            rp->rinex_nav_header(rp->navGalFile, d_user_pvt_solver->galileo_iono, d_user_pvt_solver->galileo_utc_model);
                                                                            rp->log_rinex_nav(rp->navGalFile, d_user_pvt_solver->galileo_ephemeris_map);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }
                                                                    break;
                                                                case 7:  // GPS L1 C/A + GPS L2C
                                                                    if ((gps_ephemeris_iter != d_user_pvt_solver->gps_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != d_user_pvt_solver->gps_cnav_ephemeris_map.cend()))
                                                                        {
                                                                            std::string signal("1C 2S");
                                                                            rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, d_rx_time, signal);
                                                                            rp->rinex_nav_header(rp->navFile, d_user_pvt_solver->gps_iono, d_user_pvt_solver->gps_utc_model, gps_ephemeris_iter->second);
                                                                            rp->log_rinex_nav(rp->navFile, d_user_pvt_solver->gps_cnav_ephemeris_map);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }
                                                                    break;
                                                                case 8:  // GPS L1 + GPS L5
                                                                    if ((gps_ephemeris_iter != d_user_pvt_solver->gps_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != d_user_pvt_solver->gps_cnav_ephemeris_map.cend()))
                                                                        {
                                                                            std::string signal("1C L5");
                                                                            rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, d_rx_time, signal);
                                                                            rp->rinex_nav_header(rp->navFile, d_user_pvt_solver->gps_iono, d_user_pvt_solver->gps_utc_model, gps_ephemeris_iter->second);
                                                                            rp->log_rinex_nav(rp->navFile, d_user_pvt_solver->gps_ephemeris_map);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }
                                                                    break;
                                                                case 9:  // GPS L1 C/A + Galileo E1B
                                                                    if ((galileo_ephemeris_iter != d_user_pvt_solver->galileo_ephemeris_map.cend()) and (gps_ephemeris_iter != d_user_pvt_solver->gps_ephemeris_map.cend()))
                                                                        {
                                                                            std::string gal_signal("1B");
                                                                            rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, d_rx_time, gal_signal);
                                                                            rp->rinex_nav_header(rp->navMixFile, d_user_pvt_solver->gps_iono, d_user_pvt_solver->gps_utc_model, gps_ephemeris_iter->second, d_user_pvt_solver->galileo_iono, d_user_pvt_solver->galileo_utc_model);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }
                                                                    break;
                                                                case 10:  //  GPS L1 C/A + Galileo E5a
                                                                    if ((galileo_ephemeris_iter != d_user_pvt_solver->galileo_ephemeris_map.cend()) and (gps_ephemeris_iter != d_user_pvt_solver->gps_ephemeris_map.cend()))
                                                                        {
                                                                            std::string gal_signal("5X");
                                                                            rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, d_rx_time, gal_signal);
                                                                            rp->rinex_nav_header(rp->navMixFile, d_user_pvt_solver->gps_iono, d_user_pvt_solver->gps_utc_model, gps_ephemeris_iter->second, d_user_pvt_solver->galileo_iono, d_user_pvt_solver->galileo_utc_model);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }
                                                                    break;
                                                                case 11:  //  GPS L1 C/A + Galileo E5b
                                                                    if ((galileo_ephemeris_iter != d_user_pvt_solver->galileo_ephemeris_map.cend()) and (gps_ephemeris_iter != d_user_pvt_solver->gps_ephemeris_map.cend()))
                                                                        {
                                                                            std::string gal_signal("7X");
                                                                            rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, d_rx_time, gal_signal);
                                                                            rp->rinex_nav_header(rp->navMixFile, d_user_pvt_solver->gps_iono, d_user_pvt_solver->gps_utc_model, gps_ephemeris_iter->second, d_user_pvt_solver->galileo_iono, d_user_pvt_solver->galileo_utc_model);
                                                                            rp->log_rinex_nav(rp->navMixFile, d_user_pvt_solver->gps_ephemeris_map, d_user_pvt_solver->galileo_ephemeris_map);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }
                                                                    break;
                                                                case 13:  // L5+E5a
                                                                    if ((galileo_ephemeris_iter != d_user_pvt_solver->galileo_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != d_user_pvt_solver->gps_cnav_ephemeris_map.cend()))
                                                                        {
                                                                            std::string gal_signal("5X");
                                                                            std::string gps_signal("L5");
                                                                            rp->rinex_obs_header(rp->obsFile, gps_cnav_ephemeris_iter->second, galileo_ephemeris_iter->second, d_rx_time, gps_signal, gal_signal);
                                                                            rp->rinex_nav_header(rp->navMixFile, d_user_pvt_solver->gps_cnav_iono, d_user_pvt_solver->gps_cnav_utc_model, d_user_pvt_solver->galileo_iono, d_user_pvt_solver->galileo_utc_model);
                                                                            rp->log_rinex_nav(rp->navFile, d_user_pvt_solver->gps_cnav_ephemeris_map, d_user_pvt_solver->galileo_ephemeris_map);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }
                                                                    break;
                                                                case 14:  //  Galileo E1B + Galileo E5a
                                                                    if ((galileo_ephemeris_iter != d_user_pvt_solver->galileo_ephemeris_map.cend()))
                                                                        {
                                                                            std::string gal_signal("1B 5X");
                                                                            rp->rinex_obs_header(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, gal_signal);
                                                                            rp->rinex_nav_header(rp->navGalFile, d_user_pvt_solver->galileo_iono, d_user_pvt_solver->galileo_utc_model);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }
                                                                    break;
                                                                case 15:  //  Galileo E1B + Galileo E5b
                                                                    if ((galileo_ephemeris_iter != d_user_pvt_solver->galileo_ephemeris_map.cend()))
                                                                        {
                                                                            std::string gal_signal("1B 7X");
                                                                            rp->rinex_obs_header(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, gal_signal);
                                                                            rp->rinex_nav_header(rp->navGalFile, d_user_pvt_solver->galileo_iono, d_user_pvt_solver->galileo_utc_model);
                                                                            rp->log_rinex_nav(rp->navGalFile, d_user_pvt_solver->galileo_ephemeris_map);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }
                                                                    break;
                                                                case 23:  // GLONASS L1 C/A only
                                                                    if (glonass_gnav_ephemeris_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                        {
                                                                            std::string signal("1G");
                                                                            rp->rinex_obs_header(rp->obsFile, glonass_gnav_ephemeris_iter->second, d_rx_time, signal);
                                                                            rp->rinex_nav_header(rp->navGloFile, d_user_pvt_solver->glonass_gnav_utc_model, glonass_gnav_ephemeris_iter->second);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }
                                                                    break;
                                                                case 24:  // GLONASS L2 C/A only
                                                                    if (glonass_gnav_ephemeris_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                        {
                                                                            std::string signal("2G");
                                                                            rp->rinex_obs_header(rp->obsFile, glonass_gnav_ephemeris_iter->second, d_rx_time, signal);
                                                                            rp->rinex_nav_header(rp->navGloFile, d_user_pvt_solver->glonass_gnav_utc_model, glonass_gnav_ephemeris_iter->second);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }
                                                                    break;
                                                                case 25:  // GLONASS L1 C/A + GLONASS L2 C/A
                                                                    if (glonass_gnav_ephemeris_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                        {
                                                                            std::string signal("1G 2G");
                                                                            rp->rinex_obs_header(rp->obsFile, glonass_gnav_ephemeris_iter->second, d_rx_time, signal);
                                                                            rp->rinex_nav_header(rp->navGloFile, d_user_pvt_solver->glonass_gnav_utc_model, glonass_gnav_ephemeris_iter->second);
                                                                            rp->log_rinex_nav(rp->navGloFile, d_user_pvt_solver->glonass_gnav_ephemeris_map);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }
                                                                    break;
                                                                case 26:  // GPS L1 C/A + GLONASS L1 C/A
                                                                    if ((glonass_gnav_ephemeris_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend()) and (gps_ephemeris_iter != d_user_pvt_solver->gps_ephemeris_map.cend()))
                                                                        {
                                                                            std::string glo_signal("1G");
                                                                            rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, d_rx_time, glo_signal);
                                                                            if (d_rinex_version == 3)
                                                                                {
                                                                                    rp->rinex_nav_header(rp->navMixFile, d_user_pvt_solver->gps_iono, d_user_pvt_solver->gps_utc_model, gps_ephemeris_iter->second, d_user_pvt_solver->glonass_gnav_utc_model, d_user_pvt_solver->glonass_gnav_almanac);
                                                                                    rp->log_rinex_nav(rp->navMixFile, d_user_pvt_solver->gps_ephemeris_map, d_user_pvt_solver->glonass_gnav_ephemeris_map);
                                                                                }
                                                                            if (d_rinex_version == 2)
                                                                                {
                                                                                    rp->rinex_nav_header(rp->navFile, d_user_pvt_solver->gps_iono, d_user_pvt_solver->gps_utc_model, gps_ephemeris_iter->second);
                                                                                    rp->rinex_nav_header(rp->navGloFile, d_user_pvt_solver->glonass_gnav_utc_model, glonass_gnav_ephemeris_iter->second);
                                                                                    rp->log_rinex_nav(rp->navFile, d_user_pvt_solver->gps_ephemeris_map);
                                                                                    rp->log_rinex_nav(rp->navGloFile, d_user_pvt_solver->glonass_gnav_ephemeris_map);
                                                                                }
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }
                                                                    break;
                                                                case 27:  //  Galileo E1B + GLONASS L1 C/A
                                                                    if ((glonass_gnav_ephemeris_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend()) and (galileo_ephemeris_iter != d_user_pvt_solver->galileo_ephemeris_map.cend()))
                                                                        {
                                                                            std::string glo_signal("1G");
                                                                            std::string gal_signal("1B");
                                                                            rp->rinex_obs_header(rp->obsFile, galileo_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, d_rx_time, glo_signal, gal_signal);
                                                                            rp->rinex_nav_header(rp->navMixFile, d_user_pvt_solver->galileo_iono, d_user_pvt_solver->galileo_utc_model, d_user_pvt_solver->glonass_gnav_utc_model, d_user_pvt_solver->glonass_gnav_almanac);
                                                                            rp->log_rinex_nav(rp->navMixFile, d_user_pvt_solver->galileo_ephemeris_map, d_user_pvt_solver->glonass_gnav_ephemeris_map);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }
                                                                    break;
                                                                case 28:  // GPS L2C + GLONASS L1 C/A
                                                                    if ((glonass_gnav_ephemeris_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != d_user_pvt_solver->gps_cnav_ephemeris_map.cend()))
                                                                        {
                                                                            std::string glo_signal("1G");
                                                                            rp->rinex_obs_header(rp->obsFile, gps_cnav_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, d_rx_time, glo_signal);
                                                                            rp->rinex_nav_header(rp->navMixFile, d_user_pvt_solver->gps_cnav_iono, d_user_pvt_solver->gps_cnav_utc_model, d_user_pvt_solver->glonass_gnav_utc_model, d_user_pvt_solver->glonass_gnav_almanac);
                                                                            rp->log_rinex_nav(rp->navMixFile, d_user_pvt_solver->gps_cnav_ephemeris_map, d_user_pvt_solver->glonass_gnav_ephemeris_map);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }
                                                                    break;
                                                                case 29:  // GPS L1 C/A + GLONASS L2 C/A
                                                                    if ((glonass_gnav_ephemeris_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend()) and (gps_ephemeris_iter != d_user_pvt_solver->gps_ephemeris_map.cend()))
                                                                        {
                                                                            std::string glo_signal("2G");
                                                                            rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, d_rx_time, glo_signal);
                                                                            if (d_rinex_version == 3)
                                                                                {
                                                                                    rp->rinex_nav_header(rp->navMixFile, d_user_pvt_solver->gps_iono, d_user_pvt_solver->gps_utc_model, gps_ephemeris_iter->second, d_user_pvt_solver->glonass_gnav_utc_model, d_user_pvt_solver->glonass_gnav_almanac);
                                                                                    rp->log_rinex_nav(rp->navMixFile, d_user_pvt_solver->gps_ephemeris_map, d_user_pvt_solver->glonass_gnav_ephemeris_map);
                                                                                }
                                                                            if (d_rinex_version == 2)
                                                                                {
                                                                                    rp->rinex_nav_header(rp->navFile, d_user_pvt_solver->gps_iono, d_user_pvt_solver->gps_utc_model, gps_ephemeris_iter->second);
                                                                                    rp->rinex_nav_header(rp->navGloFile, d_user_pvt_solver->glonass_gnav_utc_model, glonass_gnav_ephemeris_iter->second);
                                                                                    rp->log_rinex_nav(rp->navFile, d_user_pvt_solver->gps_ephemeris_map);
                                                                                    rp->log_rinex_nav(rp->navGloFile, d_user_pvt_solver->glonass_gnav_ephemeris_map);
                                                                                }
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }
                                                                    break;
                                                                case 30:  //  Galileo E1B + GLONASS L2 C/A
                                                                    if ((glonass_gnav_ephemeris_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend()) and (galileo_ephemeris_iter != d_user_pvt_solver->galileo_ephemeris_map.cend()))
                                                                        {
                                                                            std::string glo_signal("2G");
                                                                            std::string gal_signal("1B");
                                                                            rp->rinex_obs_header(rp->obsFile, galileo_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, d_rx_time, glo_signal, gal_signal);
                                                                            rp->rinex_nav_header(rp->navMixFile, d_user_pvt_solver->galileo_iono, d_user_pvt_solver->galileo_utc_model, d_user_pvt_solver->glonass_gnav_utc_model, d_user_pvt_solver->glonass_gnav_almanac);
                                                                            rp->log_rinex_nav(rp->navMixFile, d_user_pvt_solver->galileo_ephemeris_map, d_user_pvt_solver->glonass_gnav_ephemeris_map);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }
                                                                    break;
                                                                case 31:  // GPS L2C + GLONASS L2 C/A
                                                                    if ((glonass_gnav_ephemeris_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != d_user_pvt_solver->gps_cnav_ephemeris_map.cend()))
                                                                        {
                                                                            std::string glo_signal("2G");
                                                                            rp->rinex_obs_header(rp->obsFile, gps_cnav_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, d_rx_time, glo_signal);
                                                                            rp->rinex_nav_header(rp->navMixFile, d_user_pvt_solver->gps_cnav_iono, d_user_pvt_solver->gps_cnav_utc_model, d_user_pvt_solver->glonass_gnav_utc_model, d_user_pvt_solver->glonass_gnav_almanac);
                                                                            rp->log_rinex_nav(rp->navMixFile, d_user_pvt_solver->gps_cnav_ephemeris_map, d_user_pvt_solver->glonass_gnav_ephemeris_map);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }
                                                                    break;
                                                                case 32:  // L1+E1+L5+E5a
                                                                    if ((gps_ephemeris_iter != d_user_pvt_solver->gps_ephemeris_map.cend()) and
                                                                        (gps_cnav_ephemeris_iter != d_user_pvt_solver->gps_cnav_ephemeris_map.cend()) and
                                                                        (galileo_ephemeris_iter != d_user_pvt_solver->galileo_ephemeris_map.cend()))
                                                                        {
                                                                            std::string gal_signal("1B 5X");
                                                                            std::string gps_signal("1C L5");
                                                                            rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, galileo_ephemeris_iter->second, d_rx_time, gps_signal, gal_signal);
                                                                            rp->rinex_nav_header(rp->navMixFile, d_user_pvt_solver->gps_iono, d_user_pvt_solver->gps_utc_model, gps_ephemeris_iter->second, d_user_pvt_solver->galileo_iono, d_user_pvt_solver->galileo_utc_model);
                                                                            rp->log_rinex_nav(rp->navMixFile, d_user_pvt_solver->gps_ephemeris_map, d_user_pvt_solver->galileo_ephemeris_map);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }
                                                                    break;
                                                                case 33:  // L1+E1+E5a
                                                                    if ((gps_ephemeris_iter != d_user_pvt_solver->gps_ephemeris_map.cend()) and
                                                                        (galileo_ephemeris_iter != d_user_pvt_solver->galileo_ephemeris_map.cend()))
                                                                        {
                                                                            std::string gal_signal("1B 5X");
                                                                            rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, d_rx_time, gal_signal);
                                                                            rp->rinex_nav_header(rp->navMixFile, d_user_pvt_solver->gps_iono, d_user_pvt_solver->gps_utc_model, gps_ephemeris_iter->second, d_user_pvt_solver->galileo_iono, d_user_pvt_solver->galileo_utc_model);
                                                                            rp->log_rinex_nav(rp->navMixFile, d_user_pvt_solver->gps_ephemeris_map, d_user_pvt_solver->galileo_ephemeris_map);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }
                                                                    break;
                                                                case 500:  // BDS B1I only
                                                                    if (beidou_dnav_ephemeris_iter != d_user_pvt_solver->beidou_dnav_ephemeris_map.cend())
                                                                        {
                                                                            rp->rinex_obs_header(rp->obsFile, beidou_dnav_ephemeris_iter->second, d_rx_time, "B1");
                                                                            rp->rinex_nav_header(rp->navFile, d_user_pvt_solver->beidou_dnav_iono, d_user_pvt_solver->beidou_dnav_utc_model);
                                                                            rp->log_rinex_nav(rp->navFile, d_user_pvt_solver->beidou_dnav_ephemeris_map);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }

                                                                    break;
                                                                case 501:  // BeiDou B1I + GPS L1 C/A
                                                                    if ((gps_ephemeris_iter != d_user_pvt_solver->gps_ephemeris_map.cend()) and (beidou_dnav_ephemeris_iter != d_user_pvt_solver->beidou_dnav_ephemeris_map.cend()))
                                                                        {
                                                                            std::string bds_signal("B1");
                                                                            // rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second, beidou_dnav_ephemeris_iter->second, d_rx_time, bds_signal);
                                                                            // rp->rinex_nav_header(rp->navMixFile, d_user_pvt_solver->gps_iono, d_user_pvt_solver->gps_utc_model, gps_ephemeris_iter->second, d_user_pvt_solver->beidou_dnav_iono, d_user_pvt_solver->beidou_dnav_utc_model);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }

                                                                    break;
                                                                case 502:  // BeiDou B1I + Galileo E1B
                                                                    if ((galileo_ephemeris_iter != d_user_pvt_solver->galileo_ephemeris_map.cend()) and (beidou_dnav_ephemeris_iter != d_user_pvt_solver->beidou_dnav_ephemeris_map.cend()))
                                                                        {
                                                                            std::string bds_signal("B1");
                                                                            std::string gal_signal("1B");
                                                                            // rp->rinex_obs_header(rp->obsFile, galileo_ephemeris_iter->second, beidou_dnav_ephemeris_iter->second, d_rx_time, gal_signal, bds_signal);
                                                                            // rp->rinex_nav_header(rp->navMixFile, d_user_pvt_solver->galileo_iono, d_user_pvt_solver->galileo_utc_model, d_user_pvt_solver->beidou_dnav_iono, d_user_pvt_solver->beidou_dnav_utc_model);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }

                                                                    break;
                                                                case 503:  // BeiDou B1I + GLONASS L1 C/A
                                                                    if (beidou_dnav_ephemeris_iter != d_user_pvt_solver->beidou_dnav_ephemeris_map.cend())
                                                                        {
                                                                            // rp->rinex_obs_header(rp->obsFile, beidou_dnav_ephemeris_iter->second, d_rx_time, "B1");
                                                                            // rp->rinex_nav_header(rp->navFile, d_user_pvt_solver->beidou_dnav_iono, d_user_pvt_solver->beidou_dnav_utc_model);
                                                                            // rp->log_rinex_nav(rp->navFile, d_user_pvt_solver->beidou_dnav_ephemeris_map);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }

                                                                    break;
                                                                case 504:  // BeiDou B1I + GPS L1 C/A + Galileo E1B
                                                                    if (beidou_dnav_ephemeris_iter != d_user_pvt_solver->beidou_dnav_ephemeris_map.cend())
                                                                        {
                                                                            // rp->rinex_obs_header(rp->obsFile, beidou_dnav_ephemeris_iter->second, d_rx_time, "B1");
                                                                            // rp->rinex_nav_header(rp->navFile, d_user_pvt_solver->beidou_dnav_iono, d_user_pvt_solver->beidou_dnav_utc_model);
                                                                            // rp->log_rinex_nav(rp->navFile, d_user_pvt_solver->beidou_dnav_ephemeris_map);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }

                                                                    break;
                                                                case 505:  // BeiDou B1I + GPS L1 C/A + GLONASS L1 C/A + Galileo E1B
                                                                    if (beidou_dnav_ephemeris_iter != d_user_pvt_solver->beidou_dnav_ephemeris_map.cend())
                                                                        {
                                                                            // rp->rinex_obs_header(rp->obsFile, beidou_dnav_ephemeris_iter->second, d_rx_time, "B1");
                                                                            // rp->rinex_nav_header(rp->navFile, d_user_pvt_solver->beidou_dnav_iono, d_user_pvt_solver->beidou_dnav_utc_model);
                                                                            // rp->log_rinex_nav(rp->navFile, d_user_pvt_solver->beidou_dnav_ephemeris_map);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }

                                                                    break;
                                                                case 506:  // BeiDou B1I + Beidou B3I
                                                                    if (beidou_dnav_ephemeris_iter != d_user_pvt_solver->beidou_dnav_ephemeris_map.cend())
                                                                        {
                                                                            // rp->rinex_obs_header(rp->obsFile, beidou_dnav_ephemeris_iter->second, d_rx_time, "B1");
                                                                            // rp->rinex_nav_header(rp->navFile, d_user_pvt_solver->beidou_dnav_iono, d_user_pvt_solver->beidou_dnav_utc_model);
                                                                            // rp->log_rinex_nav(rp->navFile, d_user_pvt_solver->beidou_dnav_ephemeris_map);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }

                                                                    break;
                                                                case 600:  // BDS B3I only
                                                                    if (beidou_dnav_ephemeris_iter != d_user_pvt_solver->beidou_dnav_ephemeris_map.cend())
                                                                        {
                                                                            rp->rinex_obs_header(rp->obsFile, beidou_dnav_ephemeris_iter->second, d_rx_time, "B3");
                                                                            rp->rinex_nav_header(rp->navFile, d_user_pvt_solver->beidou_dnav_iono, d_user_pvt_solver->beidou_dnav_utc_model);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }

                                                                    break;
                                                                case 601:  // BeiDou B3I + GPS L2C
                                                                    if (beidou_dnav_ephemeris_iter != d_user_pvt_solver->beidou_dnav_ephemeris_map.cend())
                                                                        {
                                                                            rp->rinex_obs_header(rp->obsFile, beidou_dnav_ephemeris_iter->second, d_rx_time, "B3");
                                                                            // rp->rinex_nav_header(rp->navFile, d_user_pvt_solver->beidou_dnav_iono, d_user_pvt_solver->beidou_dnav_utc_model);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }

                                                                    break;
                                                                case 602:  // BeiDou B3I + GLONASS L2 C/A
                                                                    if (beidou_dnav_ephemeris_iter != d_user_pvt_solver->beidou_dnav_ephemeris_map.cend())
                                                                        {
                                                                            rp->rinex_obs_header(rp->obsFile, beidou_dnav_ephemeris_iter->second, d_rx_time, "B3");
                                                                            // rp->rinex_nav_header(rp->navFile, d_user_pvt_solver->beidou_dnav_iono, d_user_pvt_solver->beidou_dnav_utc_model);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }

                                                                    break;
                                                                case 603:  // BeiDou B3I + GPS L2C + GLONASS L2 C/A
                                                                    if (beidou_dnav_ephemeris_iter != d_user_pvt_solver->beidou_dnav_ephemeris_map.cend())
                                                                        {
                                                                            rp->rinex_obs_header(rp->obsFile, beidou_dnav_ephemeris_iter->second, d_rx_time, "B3");
                                                                            // rp->rinex_nav_header(rp->navFile, d_user_pvt_solver->beidou_dnav_iono, d_user_pvt_solver->beidou_dnav_utc_model);
                                                                            b_rinex_header_written = true;  // do not write header anymore
                                                                        }

                                                                    break;
                                                                default:
                                                                    break;
                                                                }
                                                        }
                                                    if (b_rinex_header_written)  // The header is already written, we can now log the navigation message data
                                                        {
                                                            galileo_ephemeris_iter = d_user_pvt_solver->galileo_ephemeris_map.cbegin();
                                                            gps_ephemeris_iter = d_user_pvt_solver->gps_ephemeris_map.cbegin();
                                                            gps_cnav_ephemeris_iter = d_user_pvt_solver->gps_cnav_ephemeris_map.cbegin();
                                                            glonass_gnav_ephemeris_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.cbegin();
                                                            beidou_dnav_ephemeris_iter = d_user_pvt_solver->beidou_dnav_ephemeris_map.cbegin();

                                                            // Log observables into the RINEX file
                                                            if (flag_write_RINEX_obs_output)
                                                                {
                                                                    switch (type_of_rx)
                                                                        {
                                                                        case 1:  // GPS L1 C/A only
                                                                            if (gps_ephemeris_iter != d_user_pvt_solver->gps_ephemeris_map.cend())
                                                                                {
                                                                                    rp->log_rinex_obs(rp->obsFile, gps_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                                    if (!b_rinex_header_updated and (d_user_pvt_solver->gps_utc_model.d_A0 != 0))
                                                                                        {
                                                                                            rp->update_obs_header(rp->obsFile, d_user_pvt_solver->gps_utc_model);
                                                                                            rp->update_nav_header(rp->navFile, d_user_pvt_solver->gps_utc_model, d_user_pvt_solver->gps_iono, gps_ephemeris_iter->second);
                                                                                            b_rinex_header_updated = true;
                                                                                        }
                                                                                }
                                                                            break;
                                                                        case 2:  // GPS L2C only
                                                                            if (gps_cnav_ephemeris_iter != d_user_pvt_solver->gps_cnav_ephemeris_map.cend())
                                                                                {
                                                                                    rp->log_rinex_obs(rp->obsFile, gps_cnav_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                                }
                                                                            if (!b_rinex_header_updated and (d_user_pvt_solver->gps_cnav_utc_model.d_A0 != 0))
                                                                                {
                                                                                    rp->update_obs_header(rp->obsFile, d_user_pvt_solver->gps_cnav_utc_model);
                                                                                    rp->update_nav_header(rp->navFile, d_user_pvt_solver->gps_cnav_utc_model, d_user_pvt_solver->gps_cnav_iono);
                                                                                    b_rinex_header_updated = true;
                                                                                }
                                                                            break;
                                                                        case 3:  // GPS L5
                                                                            if (gps_cnav_ephemeris_iter != d_user_pvt_solver->gps_cnav_ephemeris_map.cend())
                                                                                {
                                                                                    rp->log_rinex_obs(rp->obsFile, gps_cnav_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                                }
                                                                            if (!b_rinex_header_updated and (d_user_pvt_solver->gps_cnav_utc_model.d_A0 != 0))
                                                                                {
                                                                                    rp->update_obs_header(rp->obsFile, d_user_pvt_solver->gps_cnav_utc_model);
                                                                                    rp->update_nav_header(rp->navFile, d_user_pvt_solver->gps_cnav_utc_model, d_user_pvt_solver->gps_cnav_iono);
                                                                                    b_rinex_header_updated = true;
                                                                                }
                                                                            break;
                                                                        case 4:  // Galileo E1B only
                                                                            if (galileo_ephemeris_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                {
                                                                                    rp->log_rinex_obs(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, gnss_observables_map, "1B");
                                                                                }
                                                                            if (!b_rinex_header_updated and (d_user_pvt_solver->galileo_utc_model.A0_6 != 0))
                                                                                {
                                                                                    rp->update_nav_header(rp->navGalFile, d_user_pvt_solver->galileo_iono, d_user_pvt_solver->galileo_utc_model);
                                                                                    rp->update_obs_header(rp->obsFile, d_user_pvt_solver->galileo_utc_model);
                                                                                    b_rinex_header_updated = true;
                                                                                }
                                                                            break;
                                                                        case 5:  // Galileo E5a only
                                                                            if (galileo_ephemeris_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                {
                                                                                    rp->log_rinex_obs(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, gnss_observables_map, "5X");
                                                                                }
                                                                            if (!b_rinex_header_updated and (d_user_pvt_solver->galileo_utc_model.A0_6 != 0))
                                                                                {
                                                                                    rp->update_nav_header(rp->navGalFile, d_user_pvt_solver->galileo_iono, d_user_pvt_solver->galileo_utc_model);
                                                                                    rp->update_obs_header(rp->obsFile, d_user_pvt_solver->galileo_utc_model);
                                                                                    b_rinex_header_updated = true;
                                                                                }
                                                                            break;
                                                                        case 6:  // Galileo E5b only
                                                                            if (galileo_ephemeris_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                {
                                                                                    rp->log_rinex_obs(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, gnss_observables_map, "7X");
                                                                                }
                                                                            if (!b_rinex_header_updated and (d_user_pvt_solver->galileo_utc_model.A0_6 != 0))
                                                                                {
                                                                                    rp->update_nav_header(rp->navGalFile, d_user_pvt_solver->galileo_iono, d_user_pvt_solver->galileo_utc_model);
                                                                                    rp->update_obs_header(rp->obsFile, d_user_pvt_solver->galileo_utc_model);
                                                                                    b_rinex_header_updated = true;
                                                                                }
                                                                            break;
                                                                        case 7:  // GPS L1 C/A + GPS L2C
                                                                            if ((gps_ephemeris_iter != d_user_pvt_solver->gps_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != d_user_pvt_solver->gps_cnav_ephemeris_map.cend()))
                                                                                {
                                                                                    rp->log_rinex_obs(rp->obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                                    if (!b_rinex_header_updated and (d_user_pvt_solver->gps_utc_model.d_A0 != 0))
                                                                                        {
                                                                                            rp->update_obs_header(rp->obsFile, d_user_pvt_solver->gps_utc_model);
                                                                                            rp->update_nav_header(rp->navFile, d_user_pvt_solver->gps_utc_model, d_user_pvt_solver->gps_iono, gps_ephemeris_iter->second);
                                                                                            b_rinex_header_updated = true;
                                                                                        }
                                                                                }
                                                                            break;
                                                                        case 8:  // L1+L5
                                                                            if ((gps_ephemeris_iter != d_user_pvt_solver->gps_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != d_user_pvt_solver->gps_cnav_ephemeris_map.cend()))
                                                                                {
                                                                                    rp->log_rinex_obs(rp->obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                                    if (!b_rinex_header_updated and ((d_user_pvt_solver->gps_cnav_utc_model.d_A0 != 0) or (d_user_pvt_solver->gps_utc_model.d_A0 != 0)))
                                                                                        {
                                                                                            if (d_user_pvt_solver->gps_cnav_utc_model.d_A0 != 0)
                                                                                                {
                                                                                                    rp->update_obs_header(rp->obsFile, d_user_pvt_solver->gps_cnav_utc_model);
                                                                                                    rp->update_nav_header(rp->navFile, d_user_pvt_solver->gps_cnav_utc_model, d_user_pvt_solver->gps_cnav_iono);
                                                                                                }
                                                                                            else
                                                                                                {
                                                                                                    rp->update_obs_header(rp->obsFile, d_user_pvt_solver->gps_utc_model);
                                                                                                    rp->update_nav_header(rp->navFile, d_user_pvt_solver->gps_utc_model, d_user_pvt_solver->gps_iono, gps_ephemeris_iter->second);
                                                                                                }
                                                                                            b_rinex_header_updated = true;
                                                                                        }
                                                                                }
                                                                            break;
                                                                        case 9:  // GPS L1 C/A + Galileo E1B
                                                                            if ((galileo_ephemeris_iter != d_user_pvt_solver->galileo_ephemeris_map.cend()) and (gps_ephemeris_iter != d_user_pvt_solver->gps_ephemeris_map.cend()))
                                                                                {
                                                                                    rp->log_rinex_obs(rp->obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                                    if (!b_rinex_header_updated and (d_user_pvt_solver->gps_utc_model.d_A0 != 0))
                                                                                        {
                                                                                            rp->update_obs_header(rp->obsFile, d_user_pvt_solver->gps_utc_model);
                                                                                            rp->update_nav_header(rp->navMixFile, d_user_pvt_solver->gps_iono, d_user_pvt_solver->gps_utc_model, gps_ephemeris_iter->second, d_user_pvt_solver->galileo_iono, d_user_pvt_solver->galileo_utc_model);
                                                                                            b_rinex_header_updated = true;
                                                                                        }
                                                                                }
                                                                            break;
                                                                        case 13:  // L5+E5a
                                                                            if ((gps_cnav_ephemeris_iter != d_user_pvt_solver->gps_cnav_ephemeris_map.cend()) and (galileo_ephemeris_iter != d_user_pvt_solver->galileo_ephemeris_map.cend()))
                                                                                {
                                                                                    rp->log_rinex_obs(rp->obsFile, gps_cnav_ephemeris_iter->second, galileo_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                                }
                                                                            if (!b_rinex_header_updated and (d_user_pvt_solver->gps_cnav_utc_model.d_A0 != 0) and (d_user_pvt_solver->galileo_utc_model.A0_6 != 0))
                                                                                {
                                                                                    rp->update_obs_header(rp->obsFile, d_user_pvt_solver->gps_cnav_utc_model);
                                                                                    rp->update_nav_header(rp->navMixFile, d_user_pvt_solver->gps_cnav_utc_model, d_user_pvt_solver->gps_cnav_iono, d_user_pvt_solver->galileo_iono, d_user_pvt_solver->galileo_utc_model);
                                                                                    b_rinex_header_updated = true;  // do not write header anymore
                                                                                }
                                                                            break;
                                                                        case 14:  // Galileo E1B + Galileo E5a
                                                                            if (galileo_ephemeris_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                {
                                                                                    rp->log_rinex_obs(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, gnss_observables_map, "1B 5X");
                                                                                }
                                                                            if (!b_rinex_header_updated and (d_user_pvt_solver->galileo_utc_model.A0_6 != 0))
                                                                                {
                                                                                    rp->update_nav_header(rp->navGalFile, d_user_pvt_solver->galileo_iono, d_user_pvt_solver->galileo_utc_model);
                                                                                    rp->update_obs_header(rp->obsFile, d_user_pvt_solver->galileo_utc_model);
                                                                                    b_rinex_header_updated = true;
                                                                                }
                                                                            break;
                                                                        case 15:  // Galileo E1B + Galileo E5b
                                                                            if (galileo_ephemeris_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                {
                                                                                    rp->log_rinex_obs(rp->obsFile, galileo_ephemeris_iter->second, d_rx_time, gnss_observables_map, "1B 7X");
                                                                                }
                                                                            if (!b_rinex_header_updated and (d_user_pvt_solver->galileo_utc_model.A0_6 != 0))
                                                                                {
                                                                                    rp->update_nav_header(rp->navGalFile, d_user_pvt_solver->galileo_iono, d_user_pvt_solver->galileo_utc_model);
                                                                                    rp->update_obs_header(rp->obsFile, d_user_pvt_solver->galileo_utc_model);
                                                                                    b_rinex_header_updated = true;
                                                                                }
                                                                            break;
                                                                        case 23:  // GLONASS L1 C/A only
                                                                            if (glonass_gnav_ephemeris_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                                {
                                                                                    rp->log_rinex_obs(rp->obsFile, glonass_gnav_ephemeris_iter->second, d_rx_time, gnss_observables_map, "1C");
                                                                                }
                                                                            if (!b_rinex_header_updated and (d_user_pvt_solver->glonass_gnav_utc_model.d_tau_c != 0))
                                                                                {
                                                                                    rp->update_nav_header(rp->navGloFile, d_user_pvt_solver->glonass_gnav_utc_model, d_user_pvt_solver->glonass_gnav_almanac);
                                                                                    rp->update_obs_header(rp->obsFile, d_user_pvt_solver->glonass_gnav_utc_model);
                                                                                    b_rinex_header_updated = true;
                                                                                }
                                                                            break;
                                                                        case 24:  // GLONASS L2 C/A only
                                                                            if (glonass_gnav_ephemeris_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                                {
                                                                                    rp->log_rinex_obs(rp->obsFile, glonass_gnav_ephemeris_iter->second, d_rx_time, gnss_observables_map, "2C");
                                                                                }
                                                                            if (!b_rinex_header_updated and (d_user_pvt_solver->glonass_gnav_utc_model.d_tau_c != 0))
                                                                                {
                                                                                    rp->update_nav_header(rp->navGloFile, d_user_pvt_solver->glonass_gnav_utc_model, d_user_pvt_solver->glonass_gnav_almanac);
                                                                                    rp->update_obs_header(rp->obsFile, d_user_pvt_solver->glonass_gnav_utc_model);
                                                                                    b_rinex_header_updated = true;
                                                                                }
                                                                            break;
                                                                        case 25:  // GLONASS L1 C/A + GLONASS L2 C/A
                                                                            if (glonass_gnav_ephemeris_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                                {
                                                                                    rp->log_rinex_obs(rp->obsFile, glonass_gnav_ephemeris_iter->second, d_rx_time, gnss_observables_map, "1C 2C");
                                                                                }
                                                                            if (!b_rinex_header_updated and (d_user_pvt_solver->glonass_gnav_utc_model.d_tau_c != 0))
                                                                                {
                                                                                    rp->update_nav_header(rp->navMixFile, d_user_pvt_solver->glonass_gnav_utc_model, d_user_pvt_solver->glonass_gnav_almanac);
                                                                                    rp->update_obs_header(rp->obsFile, d_user_pvt_solver->glonass_gnav_utc_model);
                                                                                    b_rinex_header_updated = true;
                                                                                }
                                                                            break;
                                                                        case 26:  // GPS L1 C/A + GLONASS L1 C/A
                                                                            if ((glonass_gnav_ephemeris_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend()) and (gps_ephemeris_iter != d_user_pvt_solver->gps_ephemeris_map.cend()))
                                                                                {
                                                                                    rp->log_rinex_obs(rp->obsFile, gps_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                                    if (!b_rinex_header_updated and (d_user_pvt_solver->gps_utc_model.d_A0 != 0))
                                                                                        {
                                                                                            rp->update_obs_header(rp->obsFile, d_user_pvt_solver->gps_utc_model);
                                                                                            rp->update_nav_header(rp->navMixFile, d_user_pvt_solver->gps_iono, d_user_pvt_solver->gps_utc_model, gps_ephemeris_iter->second, d_user_pvt_solver->glonass_gnav_utc_model, d_user_pvt_solver->glonass_gnav_almanac);
                                                                                            b_rinex_header_updated = true;  // do not write header anymore
                                                                                        }
                                                                                }
                                                                            break;
                                                                        case 27:  // Galileo E1B + GLONASS L1 C/A
                                                                            if ((glonass_gnav_ephemeris_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend()) and (galileo_ephemeris_iter != d_user_pvt_solver->galileo_ephemeris_map.cend()))
                                                                                {
                                                                                    rp->log_rinex_obs(rp->obsFile, galileo_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                                }
                                                                            if (!b_rinex_header_updated and (d_user_pvt_solver->galileo_utc_model.A0_6 != 0))
                                                                                {
                                                                                    rp->update_obs_header(rp->obsFile, d_user_pvt_solver->galileo_utc_model);
                                                                                    rp->update_nav_header(rp->navMixFile, d_user_pvt_solver->galileo_iono, d_user_pvt_solver->galileo_utc_model, d_user_pvt_solver->glonass_gnav_utc_model, d_user_pvt_solver->glonass_gnav_almanac);
                                                                                    b_rinex_header_updated = true;  // do not write header anymore
                                                                                }
                                                                            break;
                                                                        case 28:  // GPS L2C + GLONASS L1 C/A
                                                                            if ((glonass_gnav_ephemeris_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != d_user_pvt_solver->gps_cnav_ephemeris_map.cend()))
                                                                                {
                                                                                    rp->log_rinex_obs(rp->obsFile, gps_cnav_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                                }
                                                                            if (!b_rinex_header_updated and (d_user_pvt_solver->gps_cnav_utc_model.d_A0 != 0))
                                                                                {
                                                                                    rp->update_obs_header(rp->obsFile, d_user_pvt_solver->gps_cnav_utc_model);
                                                                                    rp->update_nav_header(rp->navMixFile, d_user_pvt_solver->gps_cnav_iono, d_user_pvt_solver->gps_cnav_utc_model, d_user_pvt_solver->glonass_gnav_utc_model, d_user_pvt_solver->glonass_gnav_almanac);
                                                                                    b_rinex_header_updated = true;  // do not write header anymore
                                                                                }
                                                                            break;
                                                                        case 29:  // GPS L1 C/A + GLONASS L2 C/A
                                                                            if ((glonass_gnav_ephemeris_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend()) and (gps_ephemeris_iter != d_user_pvt_solver->gps_ephemeris_map.cend()))
                                                                                {
                                                                                    rp->log_rinex_obs(rp->obsFile, gps_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                                    if (!b_rinex_header_updated and (d_user_pvt_solver->gps_utc_model.d_A0 != 0))
                                                                                        {
                                                                                            rp->update_obs_header(rp->obsFile, d_user_pvt_solver->gps_utc_model);
                                                                                            rp->update_nav_header(rp->navMixFile, d_user_pvt_solver->gps_iono, d_user_pvt_solver->gps_utc_model, gps_ephemeris_iter->second, d_user_pvt_solver->glonass_gnav_utc_model, d_user_pvt_solver->glonass_gnav_almanac);
                                                                                            b_rinex_header_updated = true;  // do not write header anymore
                                                                                        }
                                                                                }
                                                                            break;
                                                                        case 30:  // Galileo E1B + GLONASS L2 C/A
                                                                            if ((glonass_gnav_ephemeris_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend()) and (galileo_ephemeris_iter != d_user_pvt_solver->galileo_ephemeris_map.cend()))
                                                                                {
                                                                                    rp->log_rinex_obs(rp->obsFile, galileo_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                                }
                                                                            if (!b_rinex_header_updated and (d_user_pvt_solver->galileo_utc_model.A0_6 != 0))
                                                                                {
                                                                                    rp->update_obs_header(rp->obsFile, d_user_pvt_solver->galileo_utc_model);
                                                                                    rp->update_nav_header(rp->navMixFile, d_user_pvt_solver->galileo_iono, d_user_pvt_solver->galileo_utc_model, d_user_pvt_solver->glonass_gnav_utc_model, d_user_pvt_solver->glonass_gnav_almanac);
                                                                                    b_rinex_header_updated = true;  // do not write header anymore
                                                                                }
                                                                            break;
                                                                        case 31:  // GPS L2C + GLONASS L2 C/A
                                                                            if ((glonass_gnav_ephemeris_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != d_user_pvt_solver->gps_cnav_ephemeris_map.cend()))
                                                                                {
                                                                                    rp->log_rinex_obs(rp->obsFile, gps_cnav_ephemeris_iter->second, glonass_gnav_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                                }
                                                                            if (!b_rinex_header_updated and (d_user_pvt_solver->gps_cnav_utc_model.d_A0 != 0))
                                                                                {
                                                                                    rp->update_obs_header(rp->obsFile, d_user_pvt_solver->gps_cnav_utc_model);
                                                                                    rp->update_nav_header(rp->navMixFile, d_user_pvt_solver->gps_cnav_iono, d_user_pvt_solver->gps_cnav_utc_model, d_user_pvt_solver->glonass_gnav_utc_model, d_user_pvt_solver->glonass_gnav_almanac);
                                                                                    b_rinex_header_updated = true;  // do not write header anymore
                                                                                }
                                                                            break;
                                                                        case 32:  // L1+E1+L5+E5a
                                                                            if ((gps_ephemeris_iter != d_user_pvt_solver->gps_ephemeris_map.cend()) and (gps_cnav_ephemeris_iter != d_user_pvt_solver->gps_cnav_ephemeris_map.cend()) and (galileo_ephemeris_iter != d_user_pvt_solver->galileo_ephemeris_map.cend()))
                                                                                {
                                                                                    rp->log_rinex_obs(rp->obsFile, gps_ephemeris_iter->second, gps_cnav_ephemeris_iter->second, galileo_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                                    if (!b_rinex_header_updated and ((d_user_pvt_solver->gps_cnav_utc_model.d_A0 != 0) or (d_user_pvt_solver->gps_utc_model.d_A0 != 0)) and (d_user_pvt_solver->galileo_utc_model.A0_6 != 0))
                                                                                        {
                                                                                            if (d_user_pvt_solver->gps_cnav_utc_model.d_A0 != 0)
                                                                                                {
                                                                                                    rp->update_obs_header(rp->obsFile, d_user_pvt_solver->gps_cnav_utc_model);
                                                                                                    rp->update_nav_header(rp->navMixFile, d_user_pvt_solver->gps_cnav_utc_model, d_user_pvt_solver->gps_cnav_iono, d_user_pvt_solver->galileo_iono, d_user_pvt_solver->galileo_utc_model);
                                                                                                }
                                                                                            else
                                                                                                {
                                                                                                    rp->update_obs_header(rp->obsFile, d_user_pvt_solver->gps_utc_model);
                                                                                                    rp->update_nav_header(rp->navMixFile, d_user_pvt_solver->gps_iono, d_user_pvt_solver->gps_utc_model, gps_ephemeris_iter->second, d_user_pvt_solver->galileo_iono, d_user_pvt_solver->galileo_utc_model);
                                                                                                }
                                                                                            b_rinex_header_updated = true;  // do not write header anymore
                                                                                        }
                                                                                }
                                                                            break;
                                                                        case 33:  // L1+E1+E5a
                                                                            if ((gps_ephemeris_iter != d_user_pvt_solver->gps_ephemeris_map.cend()) and (galileo_ephemeris_iter != d_user_pvt_solver->galileo_ephemeris_map.cend()))
                                                                                {
                                                                                    rp->log_rinex_obs(rp->obsFile, gps_ephemeris_iter->second, galileo_ephemeris_iter->second, d_rx_time, gnss_observables_map);
                                                                                    if (!b_rinex_header_updated and (d_user_pvt_solver->gps_utc_model.d_A0 != 0) and (d_user_pvt_solver->galileo_utc_model.A0_6 != 0))
                                                                                        {
                                                                                            rp->update_obs_header(rp->obsFile, d_user_pvt_solver->gps_utc_model);
                                                                                            rp->update_nav_header(rp->navMixFile, d_user_pvt_solver->gps_iono, d_user_pvt_solver->gps_utc_model, gps_ephemeris_iter->second, d_user_pvt_solver->galileo_iono, d_user_pvt_solver->galileo_utc_model);
                                                                                            b_rinex_header_updated = true;  // do not write header anymore
                                                                                        }
                                                                                }
                                                                            break;
                                                                        case 500:  // BDS B1I only
                                                                            if (beidou_dnav_ephemeris_iter != d_user_pvt_solver->beidou_dnav_ephemeris_map.cend())
                                                                                {
                                                                                    rp->log_rinex_obs(rp->obsFile, beidou_dnav_ephemeris_iter->second, d_rx_time, gnss_observables_map, "B1");
                                                                                }
                                                                            if (!b_rinex_header_updated and (d_user_pvt_solver->beidou_dnav_utc_model.d_A0_UTC != 0))
                                                                                {
                                                                                    rp->update_obs_header(rp->obsFile, d_user_pvt_solver->beidou_dnav_utc_model);
                                                                                    rp->update_nav_header(rp->navFile, d_user_pvt_solver->beidou_dnav_utc_model, d_user_pvt_solver->beidou_dnav_iono);
                                                                                    b_rinex_header_updated = true;
                                                                                }
                                                                            break;
                                                                        default:
                                                                            break;
                                                                        }
                                                                }
                                                        }
                                                }

                                            // ####################### RTCM MESSAGES #################
                                            try
                                                {
                                                    if (b_rtcm_writing_started and b_rtcm_enabled)
                                                        {
                                                            switch (type_of_rx)
                                                                {
                                                                case 1:  // GPS L1 C/A
                                                                    if (flag_write_RTCM_1019_output == true)
                                                                        {
                                                                            for (const auto& gps_eph_iter : d_user_pvt_solver->gps_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (flag_write_RTCM_MSM_output == true)
                                                                        {
                                                                            auto gps_eph_iter = d_user_pvt_solver->gps_ephemeris_map.cbegin();
                                                                            if (gps_eph_iter != d_user_pvt_solver->gps_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                        }
                                                                    break;
                                                                case 4:
                                                                case 5:
                                                                case 6:
                                                                    if (flag_write_RTCM_1045_output == true)
                                                                        {
                                                                            for (const auto& gal_eph_iter : d_user_pvt_solver->galileo_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1045(gal_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (flag_write_RTCM_MSM_output == true)
                                                                        {
                                                                            auto gal_eph_iter = d_user_pvt_solver->galileo_ephemeris_map.cbegin();
                                                                            if (gal_eph_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                        }
                                                                    break;
                                                                case 7:  // GPS L1 C/A + GPS L2C
                                                                    if (flag_write_RTCM_1019_output == true)
                                                                        {
                                                                            for (const auto& gps_eph_iter : d_user_pvt_solver->gps_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (flag_write_RTCM_MSM_output == true)
                                                                        {
                                                                            auto gps_eph_iter = d_user_pvt_solver->gps_ephemeris_map.cbegin();
                                                                            auto gps_cnav_eph_iter = d_user_pvt_solver->gps_cnav_ephemeris_map.cbegin();
                                                                            if ((gps_eph_iter != d_user_pvt_solver->gps_ephemeris_map.cend()) and (gps_cnav_eph_iter != d_user_pvt_solver->gps_cnav_ephemeris_map.cend()))
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, gps_cnav_eph_iter->second, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                        }
                                                                    break;
                                                                case 8:  // L1+L5
                                                                    if (flag_write_RTCM_1019_output == true)
                                                                        {
                                                                            for (const auto& gps_eph_iter : d_user_pvt_solver->gps_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (flag_write_RTCM_MSM_output == true)
                                                                        {
                                                                            auto gps_eph_iter = d_user_pvt_solver->gps_ephemeris_map.cbegin();
                                                                            auto gps_cnav_eph_iter = d_user_pvt_solver->gps_cnav_ephemeris_map.cbegin();
                                                                            if ((gps_eph_iter != d_user_pvt_solver->gps_ephemeris_map.cend()) and (gps_cnav_eph_iter != d_user_pvt_solver->gps_cnav_ephemeris_map.cend()))
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, gps_cnav_eph_iter->second, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                        }
                                                                    break;
                                                                case 9:  // GPS L1 C/A + Galileo E1B
                                                                    if (flag_write_RTCM_1019_output == true)
                                                                        {
                                                                            for (const auto& gps_eph_iter : d_user_pvt_solver->gps_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (flag_write_RTCM_1045_output == true)
                                                                        {
                                                                            for (const auto& gal_eph_iter : d_user_pvt_solver->galileo_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1045(gal_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (flag_write_RTCM_MSM_output == true)
                                                                        {
                                                                            std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
                                                                            auto gps_eph_iter = d_user_pvt_solver->gps_ephemeris_map.cbegin();
                                                                            auto gal_eph_iter = d_user_pvt_solver->galileo_ephemeris_map.cbegin();
                                                                            int gps_channel = 0;
                                                                            int gal_channel = 0;
                                                                            for (gnss_observables_iter = gnss_observables_map.cbegin(); gnss_observables_iter != gnss_observables_map.cend(); gnss_observables_iter++)
                                                                                {
                                                                                    std::string system(&gnss_observables_iter->second.System, 1);
                                                                                    if (gps_channel == 0)
                                                                                        {
                                                                                            if (system == "G")
                                                                                                {
                                                                                                    // This is a channel with valid GPS signal
                                                                                                    gps_eph_iter = d_user_pvt_solver->gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (gps_eph_iter != d_user_pvt_solver->gps_ephemeris_map.cend())
                                                                                                        {
                                                                                                            gps_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                    if (gal_channel == 0)
                                                                                        {
                                                                                            if (system == "E")
                                                                                                {
                                                                                                    gal_eph_iter = d_user_pvt_solver->galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (gal_eph_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                                        {
                                                                                                            gal_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                }
                                                                            if (gps_eph_iter != d_user_pvt_solver->gps_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                            if (gal_eph_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                        }
                                                                    break;
                                                                case 13:  // L5+E5a
                                                                    if (flag_write_RTCM_1045_output == true)
                                                                        {
                                                                            for (const auto& gal_eph_iter : d_user_pvt_solver->galileo_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1045(gal_eph_iter.second);
                                                                                }
                                                                        }

                                                                    if (flag_write_RTCM_MSM_output and d_rtcm_MSM_rate_ms != 0)
                                                                        {
                                                                            std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
                                                                            auto gal_eph_iter = d_user_pvt_solver->galileo_ephemeris_map.cbegin();
                                                                            auto gps_cnav_eph_iter = d_user_pvt_solver->gps_cnav_ephemeris_map.cbegin();
                                                                            int gal_channel = 0;
                                                                            int gps_channel = 0;
                                                                            for (gnss_observables_iter = gnss_observables_map.cbegin(); gnss_observables_iter != gnss_observables_map.cend(); gnss_observables_iter++)
                                                                                {
                                                                                    std::string system(&gnss_observables_iter->second.System, 1);
                                                                                    if (gps_channel == 0)
                                                                                        {
                                                                                            if (system == "G")
                                                                                                {
                                                                                                    // This is a channel with valid GPS signal
                                                                                                    gps_cnav_eph_iter = d_user_pvt_solver->gps_cnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (gps_cnav_eph_iter != d_user_pvt_solver->gps_cnav_ephemeris_map.cend())
                                                                                                        {
                                                                                                            gps_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                    if (gal_channel == 0)
                                                                                        {
                                                                                            if (system == "E")
                                                                                                {
                                                                                                    gal_eph_iter = d_user_pvt_solver->galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (gal_eph_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                                        {
                                                                                                            gal_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                }

                                                                            if (gal_eph_iter != d_user_pvt_solver->galileo_ephemeris_map.cend() and (d_rtcm_MT1097_rate_ms != 0))
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                            if (gps_cnav_eph_iter != d_user_pvt_solver->gps_cnav_ephemeris_map.cend() and (d_rtcm_MT1077_rate_ms != 0))
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, gps_cnav_eph_iter->second, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                        }
                                                                    break;
                                                                case 14:
                                                                case 15:
                                                                    if (flag_write_RTCM_1045_output == true)
                                                                        {
                                                                            for (const auto& gal_eph_iter : d_user_pvt_solver->galileo_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1045(gal_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (flag_write_RTCM_MSM_output == true)
                                                                        {
                                                                            auto gal_eph_iter = d_user_pvt_solver->galileo_ephemeris_map.cbegin();
                                                                            if (gal_eph_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                        }
                                                                    break;
                                                                case 23:
                                                                case 24:
                                                                case 25:
                                                                    if (flag_write_RTCM_1020_output == true)
                                                                        {
                                                                            for (auto glonass_gnav_ephemeris_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.cbegin(); glonass_gnav_ephemeris_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend(); glonass_gnav_ephemeris_iter++)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1020(glonass_gnav_ephemeris_iter->second, d_user_pvt_solver->glonass_gnav_utc_model);
                                                                                }
                                                                        }
                                                                    if (flag_write_RTCM_MSM_output == true)
                                                                        {
                                                                            auto glo_gnav_ephemeris_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.cbegin();
                                                                            if (glo_gnav_ephemeris_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, {}, glo_gnav_ephemeris_iter->second, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                        }
                                                                    b_rtcm_writing_started = true;
                                                                    break;
                                                                case 26:  // GPS L1 C/A + GLONASS L1 C/A
                                                                    if (flag_write_RTCM_1019_output == true)
                                                                        {
                                                                            for (const auto& gps_eph_iter : d_user_pvt_solver->gps_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (flag_write_RTCM_1020_output == true)
                                                                        {
                                                                            for (auto glonass_gnav_ephemeris_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.cbegin(); glonass_gnav_ephemeris_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend(); glonass_gnav_ephemeris_iter++)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1020(glonass_gnav_ephemeris_iter->second, d_user_pvt_solver->glonass_gnav_utc_model);
                                                                                }
                                                                        }
                                                                    if (flag_write_RTCM_MSM_output == true)
                                                                        {
                                                                            std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
                                                                            auto glonass_gnav_eph_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.cbegin();
                                                                            auto gps_eph_iter = d_user_pvt_solver->gps_ephemeris_map.cbegin();
                                                                            int gps_channel = 0;
                                                                            int glo_channel = 0;
                                                                            for (gnss_observables_iter = gnss_observables_map.cbegin(); gnss_observables_iter != gnss_observables_map.cend(); gnss_observables_iter++)
                                                                                {
                                                                                    std::string system(&gnss_observables_iter->second.System, 1);
                                                                                    if (gps_channel == 0)
                                                                                        {
                                                                                            if (system == "G")
                                                                                                {
                                                                                                    // This is a channel with valid GPS signal
                                                                                                    gps_eph_iter = d_user_pvt_solver->gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (gps_eph_iter != d_user_pvt_solver->gps_ephemeris_map.cend())
                                                                                                        {
                                                                                                            gps_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                    if (glo_channel == 0)
                                                                                        {
                                                                                            if (system == "R")
                                                                                                {
                                                                                                    glonass_gnav_eph_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (glonass_gnav_eph_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                                                        {
                                                                                                            glo_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                }

                                                                            if (glonass_gnav_eph_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, {}, glonass_gnav_eph_iter->second, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                            if (gps_eph_iter != d_user_pvt_solver->gps_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                        }
                                                                    break;
                                                                case 27:  // GLONASS L1 C/A + Galileo E1B
                                                                    if (flag_write_RTCM_1020_output == true)
                                                                        {
                                                                            for (auto glonass_gnav_eph_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.cbegin(); glonass_gnav_eph_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend(); glonass_gnav_eph_iter++)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1020(glonass_gnav_eph_iter->second, d_user_pvt_solver->glonass_gnav_utc_model);
                                                                                }
                                                                        }
                                                                    if (flag_write_RTCM_1045_output == true)
                                                                        {
                                                                            for (const auto& gal_eph_iter : d_user_pvt_solver->galileo_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1045(gal_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (flag_write_RTCM_MSM_output == true)
                                                                        {
                                                                            std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
                                                                            auto gal_eph_iter = d_user_pvt_solver->galileo_ephemeris_map.cbegin();
                                                                            auto glonass_gnav_eph_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.cbegin();
                                                                            int gal_channel = 0;
                                                                            int glo_channel = 0;
                                                                            for (gnss_observables_iter = gnss_observables_map.cbegin(); gnss_observables_iter != gnss_observables_map.cend(); gnss_observables_iter++)
                                                                                {
                                                                                    std::string system(&gnss_observables_iter->second.System, 1);
                                                                                    if (gal_channel == 0)
                                                                                        {
                                                                                            if (system == "E")
                                                                                                {
                                                                                                    // This is a channel with valid GPS signal
                                                                                                    gal_eph_iter = d_user_pvt_solver->galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (gal_eph_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                                        {
                                                                                                            gal_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                    if (glo_channel == 0)
                                                                                        {
                                                                                            if (system == "R")
                                                                                                {
                                                                                                    glonass_gnav_eph_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (glonass_gnav_eph_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                                                        {
                                                                                                            glo_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                }
                                                                            if (gal_eph_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                            if (glonass_gnav_eph_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, {}, glonass_gnav_eph_iter->second, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                        }
                                                                    break;
                                                                case 29:  // GPS L1 C/A + GLONASS L2 C/A
                                                                    if (flag_write_RTCM_1019_output == true)
                                                                        {
                                                                            for (const auto& gps_eph_iter : d_user_pvt_solver->gps_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (flag_write_RTCM_1020_output == true)
                                                                        {
                                                                            for (auto glonass_gnav_eph_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.cbegin(); glonass_gnav_eph_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend(); glonass_gnav_eph_iter++)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1020(glonass_gnav_eph_iter->second, d_user_pvt_solver->glonass_gnav_utc_model);
                                                                                }
                                                                        }
                                                                    if (flag_write_RTCM_MSM_output == true)
                                                                        {
                                                                            std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
                                                                            auto gps_eph_iter = d_user_pvt_solver->gps_ephemeris_map.cbegin();
                                                                            auto glonass_gnav_eph_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.cbegin();
                                                                            int gps_channel = 0;
                                                                            int glo_channel = 0;
                                                                            for (gnss_observables_iter = gnss_observables_map.cbegin(); gnss_observables_iter != gnss_observables_map.cend(); gnss_observables_iter++)
                                                                                {
                                                                                    std::string system(&gnss_observables_iter->second.System, 1);
                                                                                    if (gps_channel == 0)
                                                                                        {
                                                                                            if (system == "G")
                                                                                                {
                                                                                                    // This is a channel with valid GPS signal
                                                                                                    gps_eph_iter = d_user_pvt_solver->gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (gps_eph_iter != d_user_pvt_solver->gps_ephemeris_map.cend())
                                                                                                        {
                                                                                                            gps_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                    if (glo_channel == 0)
                                                                                        {
                                                                                            if (system == "R")
                                                                                                {
                                                                                                    glonass_gnav_eph_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (glonass_gnav_eph_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                                                        {
                                                                                                            glo_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                }
                                                                            if (glonass_gnav_eph_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, {}, glonass_gnav_eph_iter->second, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                            if (gps_eph_iter != d_user_pvt_solver->gps_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                        }
                                                                    break;
                                                                case 30:  // GLONASS L2 C/A + Galileo E1B
                                                                    if (flag_write_RTCM_1020_output == true)
                                                                        {
                                                                            for (auto glonass_gnav_eph_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.cbegin(); glonass_gnav_eph_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend(); glonass_gnav_eph_iter++)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1020(glonass_gnav_eph_iter->second, d_user_pvt_solver->glonass_gnav_utc_model);
                                                                                }
                                                                        }
                                                                    if (flag_write_RTCM_1045_output == true)
                                                                        {
                                                                            for (const auto& gal_eph_iter : d_user_pvt_solver->galileo_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1045(gal_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (flag_write_RTCM_MSM_output == true)
                                                                        {
                                                                            std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
                                                                            auto gal_eph_iter = d_user_pvt_solver->galileo_ephemeris_map.cbegin();
                                                                            auto glonass_gnav_eph_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.cbegin();
                                                                            int gal_channel = 0;
                                                                            int glo_channel = 0;
                                                                            for (gnss_observables_iter = gnss_observables_map.cbegin(); gnss_observables_iter != gnss_observables_map.cend(); gnss_observables_iter++)
                                                                                {
                                                                                    std::string system(&gnss_observables_iter->second.System, 1);
                                                                                    if (gal_channel == 0)
                                                                                        {
                                                                                            if (system == "E")
                                                                                                {
                                                                                                    // This is a channel with valid GPS signal
                                                                                                    gal_eph_iter = d_user_pvt_solver->galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (gal_eph_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                                        {
                                                                                                            gal_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                    if (glo_channel == 0)
                                                                                        {
                                                                                            if (system == "R")
                                                                                                {
                                                                                                    glonass_gnav_eph_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (glonass_gnav_eph_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                                                        {
                                                                                                            glo_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                }
                                                                            if (gal_eph_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                            if (glonass_gnav_eph_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, {}, glonass_gnav_eph_iter->second, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                        }
                                                                    break;
                                                                case 32:  // L1+E1+L5+E5a
                                                                    if (flag_write_RTCM_1019_output == true)
                                                                        {
                                                                            for (const auto& gps_eph_iter : d_user_pvt_solver->gps_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (flag_write_RTCM_1045_output == true)
                                                                        {
                                                                            for (const auto& gal_eph_iter : d_user_pvt_solver->galileo_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1045(gal_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (flag_write_RTCM_MSM_output == true)
                                                                        {
                                                                            std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
                                                                            auto gal_eph_iter = d_user_pvt_solver->galileo_ephemeris_map.cbegin();
                                                                            auto gps_eph_iter = d_user_pvt_solver->gps_ephemeris_map.cbegin();
                                                                            int gal_channel = 0;
                                                                            int gps_channel = 0;
                                                                            for (gnss_observables_iter = gnss_observables_map.cbegin(); gnss_observables_iter != gnss_observables_map.cend(); gnss_observables_iter++)
                                                                                {
                                                                                    std::string system(&gnss_observables_iter->second.System, 1);
                                                                                    if (gal_channel == 0)
                                                                                        {
                                                                                            if (system == "E")
                                                                                                {
                                                                                                    // This is a channel with valid GPS signal
                                                                                                    gal_eph_iter = d_user_pvt_solver->galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (gal_eph_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                                        {
                                                                                                            gal_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                    if (gps_channel == 0)
                                                                                        {
                                                                                            if (system == "G")
                                                                                                {
                                                                                                    gps_eph_iter = d_user_pvt_solver->gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (gps_eph_iter != d_user_pvt_solver->gps_ephemeris_map.cend())
                                                                                                        {
                                                                                                            gps_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                }
                                                                            if (gps_eph_iter != d_user_pvt_solver->gps_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                            if (gal_eph_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                        }
                                                                    break;
                                                                default:
                                                                    break;
                                                                }
                                                        }

                                                    if (!b_rtcm_writing_started and b_rtcm_enabled)  // the first time
                                                        {
                                                            switch (type_of_rx)
                                                                {
                                                                case 1:                              // GPS L1 C/A
                                                                    if (d_rtcm_MT1019_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                        {
                                                                            for (const auto& gps_eph_iter : d_user_pvt_solver->gps_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (d_rtcm_MSM_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                        {
                                                                            auto gps_eph_iter = d_user_pvt_solver->gps_ephemeris_map.cbegin();

                                                                            if (gps_eph_iter != d_user_pvt_solver->gps_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                        }
                                                                    b_rtcm_writing_started = true;
                                                                    break;
                                                                case 4:
                                                                case 5:
                                                                case 6:
                                                                    if (d_rtcm_MT1045_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                        {
                                                                            for (const auto& gal_eph_iter : d_user_pvt_solver->galileo_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1045(gal_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (d_rtcm_MSM_rate_ms != 0)
                                                                        {
                                                                            auto gal_eph_iter = d_user_pvt_solver->galileo_ephemeris_map.cbegin();
                                                                            if (gal_eph_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                        }
                                                                    b_rtcm_writing_started = true;
                                                                    break;
                                                                case 7:                              // GPS L1 C/A + GPS L2C
                                                                    if (d_rtcm_MT1019_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                        {
                                                                            for (const auto& gps_eph_iter : d_user_pvt_solver->gps_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (d_rtcm_MSM_rate_ms != 0)
                                                                        {
                                                                            auto gps_eph_iter = d_user_pvt_solver->gps_ephemeris_map.cbegin();
                                                                            auto gps_cnav_eph_iter = d_user_pvt_solver->gps_cnav_ephemeris_map.cbegin();
                                                                            if ((gps_eph_iter != d_user_pvt_solver->gps_ephemeris_map.cend()) and (gps_cnav_eph_iter != d_user_pvt_solver->gps_cnav_ephemeris_map.cend()))
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, gps_cnav_eph_iter->second, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                        }
                                                                    b_rtcm_writing_started = true;
                                                                    break;
                                                                case 8:                              // L1+L5
                                                                    if (d_rtcm_MT1019_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                        {
                                                                            for (const auto& gps_eph_iter : d_user_pvt_solver->gps_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (d_rtcm_MSM_rate_ms != 0)
                                                                        {
                                                                            auto gps_eph_iter = d_user_pvt_solver->gps_ephemeris_map.cbegin();
                                                                            auto gps_cnav_eph_iter = d_user_pvt_solver->gps_cnav_ephemeris_map.cbegin();
                                                                            if ((gps_eph_iter != d_user_pvt_solver->gps_ephemeris_map.cend()) and (gps_cnav_eph_iter != d_user_pvt_solver->gps_cnav_ephemeris_map.cend()))
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, gps_cnav_eph_iter->second, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                        }
                                                                    b_rtcm_writing_started = true;
                                                                    break;
                                                                case 9:                              // GPS L1 C/A + Galileo E1B
                                                                    if (d_rtcm_MT1019_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                        {
                                                                            for (const auto& gps_eph_iter : d_user_pvt_solver->gps_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (d_rtcm_MT1045_rate_ms != 0)
                                                                        {
                                                                            for (const auto& gal_eph_iter : d_user_pvt_solver->galileo_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1045(gal_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (d_rtcm_MSM_rate_ms != 0)
                                                                        {
                                                                            std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
                                                                            auto gal_eph_iter = d_user_pvt_solver->galileo_ephemeris_map.cbegin();
                                                                            auto gps_eph_iter = d_user_pvt_solver->gps_ephemeris_map.cbegin();
                                                                            int gps_channel = 0;
                                                                            int gal_channel = 0;
                                                                            for (gnss_observables_iter = gnss_observables_map.cbegin(); gnss_observables_iter != gnss_observables_map.cend(); gnss_observables_iter++)
                                                                                {
                                                                                    std::string system(&gnss_observables_iter->second.System, 1);
                                                                                    if (gps_channel == 0)
                                                                                        {
                                                                                            if (system == "G")
                                                                                                {
                                                                                                    // This is a channel with valid GPS signal
                                                                                                    gps_eph_iter = d_user_pvt_solver->gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (gps_eph_iter != d_user_pvt_solver->gps_ephemeris_map.cend())
                                                                                                        {
                                                                                                            gps_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                    if (gal_channel == 0)
                                                                                        {
                                                                                            if (system == "E")
                                                                                                {
                                                                                                    gal_eph_iter = d_user_pvt_solver->galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (gal_eph_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                                        {
                                                                                                            gal_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                }
                                                                            if (gps_eph_iter != d_user_pvt_solver->gps_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                            if (gal_eph_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                        }
                                                                    b_rtcm_writing_started = true;
                                                                    break;

                                                                case 13:  // L5+E5a
                                                                    if (d_rtcm_MT1045_rate_ms != 0)
                                                                        {
                                                                            for (const auto& gal_eph_iter : d_user_pvt_solver->galileo_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1045(gal_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (d_rtcm_MSM_rate_ms != 0)
                                                                        {
                                                                            std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
                                                                            auto gal_eph_iter = d_user_pvt_solver->galileo_ephemeris_map.cbegin();
                                                                            int gal_channel = 0;
                                                                            for (gnss_observables_iter = gnss_observables_map.cbegin(); gnss_observables_iter != gnss_observables_map.cend(); gnss_observables_iter++)
                                                                                {
                                                                                    std::string system(&gnss_observables_iter->second.System, 1);
                                                                                    if (gal_channel == 0)
                                                                                        {
                                                                                            if (system == "E")
                                                                                                {
                                                                                                    gal_eph_iter = d_user_pvt_solver->galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (gal_eph_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                                        {
                                                                                                            gal_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                }

                                                                            if (gal_eph_iter != d_user_pvt_solver->galileo_ephemeris_map.cend() and (d_rtcm_MT1097_rate_ms != 0))
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                        }
                                                                    b_rtcm_writing_started = true;
                                                                    break;
                                                                case 14:
                                                                case 15:
                                                                    if (d_rtcm_MT1045_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                        {
                                                                            for (const auto& gal_eph_iter : d_user_pvt_solver->galileo_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1045(gal_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (d_rtcm_MSM_rate_ms != 0)
                                                                        {
                                                                            auto gal_eph_iter = d_user_pvt_solver->galileo_ephemeris_map.cbegin();
                                                                            if (gal_eph_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                        }
                                                                    b_rtcm_writing_started = true;
                                                                    break;
                                                                case 23:
                                                                case 24:
                                                                case 25:
                                                                    if (d_rtcm_MT1020_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                        {
                                                                            for (auto glonass_gnav_eph_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.cbegin(); glonass_gnav_eph_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend(); glonass_gnav_eph_iter++)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1020(glonass_gnav_eph_iter->second, d_user_pvt_solver->glonass_gnav_utc_model);
                                                                                }
                                                                        }
                                                                    if (d_rtcm_MSM_rate_ms != 0)
                                                                        {
                                                                            auto glo_gnav_ephemeris_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.cbegin();
                                                                            if (glo_gnav_ephemeris_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, {}, glo_gnav_ephemeris_iter->second, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                        }
                                                                    b_rtcm_writing_started = true;
                                                                    break;
                                                                case 26:                             // GPS L1 C/A + GLONASS L1 C/A
                                                                    if (d_rtcm_MT1019_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                        {
                                                                            for (const auto& gps_eph_iter : d_user_pvt_solver->gps_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (d_rtcm_MT1020_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                        {
                                                                            for (auto glonass_gnav_eph_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.cbegin(); glonass_gnav_eph_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend(); glonass_gnav_eph_iter++)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1020(glonass_gnav_eph_iter->second, d_user_pvt_solver->glonass_gnav_utc_model);
                                                                                }
                                                                        }
                                                                    if (d_rtcm_MSM_rate_ms != 0)
                                                                        {
                                                                            std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
                                                                            auto gps_eph_iter = d_user_pvt_solver->gps_ephemeris_map.cbegin();
                                                                            auto glonass_gnav_eph_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.cbegin();
                                                                            int gps_channel = 0;
                                                                            int glo_channel = 0;
                                                                            for (gnss_observables_iter = gnss_observables_map.cbegin(); gnss_observables_iter != gnss_observables_map.cend(); gnss_observables_iter++)
                                                                                {
                                                                                    std::string system(&gnss_observables_iter->second.System, 1);
                                                                                    if (gps_channel == 0)
                                                                                        {
                                                                                            if (system == "G")
                                                                                                {
                                                                                                    // This is a channel with valid GPS signal
                                                                                                    gps_eph_iter = d_user_pvt_solver->gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (gps_eph_iter != d_user_pvt_solver->gps_ephemeris_map.cend())
                                                                                                        {
                                                                                                            gps_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                    if (glo_channel == 0)
                                                                                        {
                                                                                            if (system == "R")
                                                                                                {
                                                                                                    glonass_gnav_eph_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (glonass_gnav_eph_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                                                        {
                                                                                                            glo_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                }
                                                                            if (glonass_gnav_eph_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, {}, glonass_gnav_eph_iter->second, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                            if (gps_eph_iter != d_user_pvt_solver->gps_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                        }
                                                                    b_rtcm_writing_started = true;
                                                                    break;
                                                                case 27:                             // GLONASS L1 C/A + Galileo E1B
                                                                    if (d_rtcm_MT1020_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                        {
                                                                            for (auto glonass_gnav_eph_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.cbegin(); glonass_gnav_eph_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend(); glonass_gnav_eph_iter++)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1020(glonass_gnav_eph_iter->second, d_user_pvt_solver->glonass_gnav_utc_model);
                                                                                }
                                                                        }
                                                                    if (d_rtcm_MT1045_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                        {
                                                                            for (const auto& gal_eph_iter : d_user_pvt_solver->galileo_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1045(gal_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (d_rtcm_MSM_rate_ms != 0)
                                                                        {
                                                                            int gal_channel = 0;
                                                                            int glo_channel = 0;
                                                                            std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
                                                                            auto gal_eph_iter = d_user_pvt_solver->galileo_ephemeris_map.cbegin();
                                                                            auto glonass_gnav_eph_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.cbegin();
                                                                            for (gnss_observables_iter = gnss_observables_map.cbegin(); gnss_observables_iter != gnss_observables_map.cend(); gnss_observables_iter++)
                                                                                {
                                                                                    std::string system(&gnss_observables_iter->second.System, 1);
                                                                                    if (gal_channel == 0)
                                                                                        {
                                                                                            if (system == "E")
                                                                                                {
                                                                                                    // This is a channel with valid GPS signal
                                                                                                    gal_eph_iter = d_user_pvt_solver->galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (gal_eph_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                                        {
                                                                                                            gal_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                    if (glo_channel == 0)
                                                                                        {
                                                                                            if (system == "R")
                                                                                                {
                                                                                                    glonass_gnav_eph_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (glonass_gnav_eph_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                                                        {
                                                                                                            glo_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                }
                                                                            if (gal_eph_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                            if (glonass_gnav_eph_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, {}, glonass_gnav_eph_iter->second, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                        }
                                                                    b_rtcm_writing_started = true;
                                                                    break;
                                                                case 29:                             // GPS L1 C/A + GLONASS L2 C/A
                                                                    if (d_rtcm_MT1019_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                        {
                                                                            for (const auto& gps_eph_iter : d_user_pvt_solver->gps_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (d_rtcm_MT1020_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                        {
                                                                            for (auto glonass_gnav_eph_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.cbegin(); glonass_gnav_eph_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend(); glonass_gnav_eph_iter++)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1020(glonass_gnav_eph_iter->second, d_user_pvt_solver->glonass_gnav_utc_model);
                                                                                }
                                                                        }
                                                                    if (d_rtcm_MSM_rate_ms != 0)
                                                                        {
                                                                            auto gps_eph_iter = d_user_pvt_solver->gps_ephemeris_map.cbegin();
                                                                            auto glonass_gnav_eph_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.cbegin();
                                                                            std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
                                                                            int gps_channel = 0;
                                                                            int glo_channel = 0;
                                                                            for (gnss_observables_iter = gnss_observables_map.cbegin(); gnss_observables_iter != gnss_observables_map.cend(); gnss_observables_iter++)
                                                                                {
                                                                                    std::string system(&gnss_observables_iter->second.System, 1);
                                                                                    if (gps_channel == 0)
                                                                                        {
                                                                                            if (system == "G")
                                                                                                {
                                                                                                    // This is a channel with valid GPS signal
                                                                                                    gps_eph_iter = d_user_pvt_solver->gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (gps_eph_iter != d_user_pvt_solver->gps_ephemeris_map.cend())
                                                                                                        {
                                                                                                            gps_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                    if (glo_channel == 0)
                                                                                        {
                                                                                            if (system == "R")
                                                                                                {
                                                                                                    glonass_gnav_eph_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (glonass_gnav_eph_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                                                        {
                                                                                                            glo_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                }
                                                                            if (glonass_gnav_eph_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, {}, glonass_gnav_eph_iter->second, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }

                                                                            if (gps_eph_iter != d_user_pvt_solver->gps_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                        }
                                                                    b_rtcm_writing_started = true;
                                                                    break;
                                                                case 30:                             // GLONASS L2 C/A + Galileo E1B
                                                                    if (d_rtcm_MT1020_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                        {
                                                                            for (auto glonass_gnav_eph_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.cbegin(); glonass_gnav_eph_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend(); glonass_gnav_eph_iter++)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1020(glonass_gnav_eph_iter->second, d_user_pvt_solver->glonass_gnav_utc_model);
                                                                                }
                                                                        }
                                                                    if (d_rtcm_MT1045_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                        {
                                                                            for (const auto& gal_eph_iter : d_user_pvt_solver->galileo_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1045(gal_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (d_rtcm_MSM_rate_ms != 0)
                                                                        {
                                                                            int gal_channel = 0;
                                                                            int glo_channel = 0;
                                                                            std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
                                                                            auto gal_eph_iter = d_user_pvt_solver->galileo_ephemeris_map.cbegin();
                                                                            auto glonass_gnav_eph_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.cbegin();
                                                                            for (gnss_observables_iter = gnss_observables_map.cbegin(); gnss_observables_iter != gnss_observables_map.cend(); gnss_observables_iter++)
                                                                                {
                                                                                    std::string system(&gnss_observables_iter->second.System, 1);
                                                                                    if (gal_channel == 0)
                                                                                        {
                                                                                            if (system == "E")
                                                                                                {
                                                                                                    // This is a channel with valid GPS signal
                                                                                                    gal_eph_iter = d_user_pvt_solver->galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (gal_eph_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                                        {
                                                                                                            gal_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                    if (glo_channel == 0)
                                                                                        {
                                                                                            if (system == "R")
                                                                                                {
                                                                                                    glonass_gnav_eph_iter = d_user_pvt_solver->glonass_gnav_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (glonass_gnav_eph_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                                                        {
                                                                                                            glo_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                }
                                                                            if (gal_eph_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                            if (glonass_gnav_eph_iter != d_user_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, {}, glonass_gnav_eph_iter->second, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                        }
                                                                    b_rtcm_writing_started = true;
                                                                    break;
                                                                case 32:                             // L1+E1+L5+E5a
                                                                    if (d_rtcm_MT1019_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                        {
                                                                            for (const auto& gps_eph_iter : d_user_pvt_solver->gps_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1019(gps_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (d_rtcm_MT1045_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                                                        {
                                                                            for (const auto& gal_eph_iter : d_user_pvt_solver->galileo_ephemeris_map)
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MT1045(gal_eph_iter.second);
                                                                                }
                                                                        }
                                                                    if (d_rtcm_MSM_rate_ms != 0)
                                                                        {
                                                                            std::map<int, Gnss_Synchro>::const_iterator gnss_observables_iter;
                                                                            auto gal_eph_iter = d_user_pvt_solver->galileo_ephemeris_map.cbegin();
                                                                            auto gps_eph_iter = d_user_pvt_solver->gps_ephemeris_map.cbegin();
                                                                            int gps_channel = 0;
                                                                            int gal_channel = 0;
                                                                            for (gnss_observables_iter = gnss_observables_map.cbegin(); gnss_observables_iter != gnss_observables_map.cend(); gnss_observables_iter++)
                                                                                {
                                                                                    std::string system(&gnss_observables_iter->second.System, 1);
                                                                                    if (gps_channel == 0)
                                                                                        {
                                                                                            if (system == "G")
                                                                                                {
                                                                                                    // This is a channel with valid GPS signal
                                                                                                    gps_eph_iter = d_user_pvt_solver->gps_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (gps_eph_iter != d_user_pvt_solver->gps_ephemeris_map.cend())
                                                                                                        {
                                                                                                            gps_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                    if (gal_channel == 0)
                                                                                        {
                                                                                            if (system == "E")
                                                                                                {
                                                                                                    gal_eph_iter = d_user_pvt_solver->galileo_ephemeris_map.find(gnss_observables_iter->second.PRN);
                                                                                                    if (gal_eph_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                                        {
                                                                                                            gal_channel = 1;
                                                                                                        }
                                                                                                }
                                                                                        }
                                                                                }
                                                                            if (gps_eph_iter != d_user_pvt_solver->gps_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                            if (gal_eph_iter != d_user_pvt_solver->galileo_ephemeris_map.cend())
                                                                                {
                                                                                    d_rtcm_printer->Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, d_rx_time, gnss_observables_map, 0, 0, 0, false, false);
                                                                                }
                                                                        }
                                                                    b_rtcm_writing_started = true;
                                                                    break;
                                                                default:
                                                                    break;
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

                    // DEBUG MESSAGE: Display position in console output
                    if (d_user_pvt_solver->is_valid_position() and flag_display_pvt)
                        {
                            boost::posix_time::ptime time_solution;
                            std::string UTC_solution_str;
                            if (d_show_local_time_zone)
                                {
                                    time_solution = d_user_pvt_solver->get_position_UTC_time() + d_utc_diff_time;
                                    UTC_solution_str = d_local_time_str;
                                }
                            else
                                {
                                    time_solution = d_user_pvt_solver->get_position_UTC_time();
                                    UTC_solution_str = " UTC";
                                }
                            std::streamsize ss = std::cout.precision();  // save current precision
                            std::cout.setf(std::ios::fixed, std::ios::floatfield);
                            auto facet = new boost::posix_time::time_facet("%Y-%b-%d %H:%M:%S.%f %z");
                            std::cout.imbue(std::locale(std::cout.getloc(), facet));
                            std::cout
                                << TEXT_BOLD_GREEN
                                << "Position at " << time_solution << UTC_solution_str
                                << " using " << d_user_pvt_solver->get_num_valid_observations()
                                << std::fixed << std::setprecision(9)
                                << " observations is Lat = " << d_user_pvt_solver->get_latitude() << " [deg], Long = " << d_user_pvt_solver->get_longitude()
                                << std::fixed << std::setprecision(3)
                                << " [deg], Height = " << d_user_pvt_solver->get_height() << " [m]" << TEXT_RESET << std::endl;

                            std::cout << std::setprecision(ss);
                            DLOG(INFO) << "RX clock offset: " << d_user_pvt_solver->get_time_offset_s() << "[s]";

                            // boost::posix_time::ptime p_time;
                            // gtime_t rtklib_utc_time = gpst2time(adjgpsweek(d_user_pvt_solver->gps_ephemeris_map.cbegin()->second.i_GPS_week), d_rx_time);
                            // p_time = boost::posix_time::from_time_t(rtklib_utc_time.time);
                            // p_time += boost::posix_time::microseconds(round(rtklib_utc_time.sec * 1e6));
                            // std::cout << TEXT_MAGENTA << "Observable RX time (GPST) " << boost::posix_time::to_simple_string(p_time) << TEXT_RESET << std::endl;

                            DLOG(INFO) << "Position at " << boost::posix_time::to_simple_string(d_user_pvt_solver->get_position_UTC_time())
                                       << " UTC using " << d_user_pvt_solver->get_num_valid_observations() << " observations is Lat = " << d_user_pvt_solver->get_latitude() << " [deg], Long = " << d_user_pvt_solver->get_longitude()
                                       << " [deg], Height = " << d_user_pvt_solver->get_height() << " [m]";

                            /* std::cout << "Dilution of Precision at " << boost::posix_time::to_simple_string(d_user_pvt_solver->get_position_UTC_time())
                                         << " UTC using "<< d_user_pvt_solver->get_num_valid_observations() <<" observations is HDOP = " << d_user_pvt_solver->get_hdop() << " VDOP = "
                                         << d_user_pvt_solver->get_vdop()
                                         << " GDOP = " << d_user_pvt_solver->get_gdop() << std::endl; */
                        }

                    // PVT MONITOR
                    if (d_user_pvt_solver->is_valid_position())
                        {
                            std::shared_ptr<Monitor_Pvt> monitor_pvt = std::make_shared<Monitor_Pvt>(d_user_pvt_solver->get_monitor_pvt());

                            // publish new position to the gnss_flowgraph channel status monitor
                            if (current_RX_time_ms % d_report_rate_ms == 0)
                                {
                                    this->message_port_pub(pmt::mp("status"), pmt::make_any(monitor_pvt));
                                }
                            if (flag_monitor_pvt_enabled)
                                {
                                    udp_sink_ptr->write_monitor_pvt(monitor_pvt);
                                }
                        }
                }
        }

    return noutput_items;
}
