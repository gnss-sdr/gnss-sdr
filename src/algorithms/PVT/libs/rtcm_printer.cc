/*!
 * \file rtcm_printer.cc
 * \brief Implementation of a RTCM 3.2 printer for GNSS-SDR
 * This class provides a implementation of a subset of the RTCM Standard 10403.2
 * for Differential GNSS Services
 *
 * \author Carles Fernandez-Prades, 2014. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "rtcm_printer.h"
#include "galileo_ephemeris.h"
#include "galileo_has_data.h"
#include "glonass_gnav_ephemeris.h"
#include "glonass_gnav_utc_model.h"
#include "gnss_sdr_filesystem.h"
#include "gnss_sdr_make_unique.h"
#include "gnss_synchro.h"
#include "gps_cnav_ephemeris.h"
#include "gps_ephemeris.h"
#include "rtcm.h"
#include "rtklib_solver.h"
#include <boost/exception/diagnostic_information.hpp>
#include <glog/logging.h>
#include <ctime>      // for tm
#include <exception>  // for exception
#include <fcntl.h>    // for O_RDWR
#include <iostream>   // for cout, cerr
#include <termios.h>  // for tcgetattr
#include <unistd.h>   // for close, write
#include <vector>     // for std::vector


Rtcm_Printer::Rtcm_Printer(const std::string& filename,
    bool flag_rtcm_file_dump,
    bool flag_rtcm_server,
    bool flag_rtcm_tty_port,
    uint16_t rtcm_tcp_port,
    uint16_t rtcm_station_id,
    const std::string& rtcm_dump_devname,
    bool time_tag_name,
    const std::string& base_path) : rtcm_base_path(base_path),
                                    rtcm_devname(rtcm_dump_devname),
                                    port(rtcm_tcp_port),
                                    station_id(rtcm_station_id),
                                    d_rtcm_writing_started(false),
                                    d_rtcm_file_dump(flag_rtcm_file_dump)
{
    const boost::posix_time::ptime pt = boost::posix_time::second_clock::local_time();
    const tm timeinfo = boost::posix_time::to_tm(pt);
    if (d_rtcm_file_dump)
        {
            fs::path full_path(fs::current_path());
            const fs::path p(rtcm_base_path);
            if (!fs::exists(p))
                {
                    std::string new_folder;
                    for (const auto& folder : fs::path(rtcm_base_path))
                        {
                            new_folder += folder.string();
                            errorlib::error_code ec;
                            if (!fs::exists(new_folder))
                                {
                                    if (!fs::create_directory(new_folder, ec))
                                        {
                                            std::cout << "Could not create the " << new_folder << " folder.\n";
                                            rtcm_base_path = full_path.string();
                                        }
                                }
                            new_folder += fs::path::preferred_separator;
                        }
                }
            else
                {
                    rtcm_base_path = p.string();
                }
            if (rtcm_base_path != ".")
                {
                    std::cout << "RTCM binary file will be stored at " << rtcm_base_path << '\n';
                }

            rtcm_base_path = rtcm_base_path + fs::path::preferred_separator;
        }

    if (time_tag_name)
        {
            std::stringstream strm0;
            const int32_t year = timeinfo.tm_year - 100;
            strm0 << year;
            const int32_t month = timeinfo.tm_mon + 1;
            if (month < 10)
                {
                    strm0 << "0";
                }
            strm0 << month;
            const int32_t day = timeinfo.tm_mday;
            if (day < 10)
                {
                    strm0 << "0";
                }
            strm0 << day << "_";
            const int32_t hour = timeinfo.tm_hour;
            if (hour < 10)
                {
                    strm0 << "0";
                }
            strm0 << hour;
            const int32_t min = timeinfo.tm_min;
            if (min < 10)
                {
                    strm0 << "0";
                }
            strm0 << min;
            const int32_t sec = timeinfo.tm_sec;
            if (sec < 10)
                {
                    strm0 << "0";
                }
            strm0 << sec;

            rtcm_filename = filename + "_" + strm0.str() + ".rtcm";
        }
    else
        {
            rtcm_filename = filename + ".rtcm";
        }
    rtcm_filename = rtcm_base_path + rtcm_filename;
    if (d_rtcm_file_dump)
        {
            rtcm_file_descriptor.open(rtcm_filename.c_str(), std::ios::out);
            if (rtcm_file_descriptor.is_open())
                {
                    DLOG(INFO) << "RTCM printer writing on " << rtcm_filename.c_str();
                }
            else
                {
                    std::cout << "File " << rtcm_filename << "cannot be saved. Wrong permissions?\n";
                }
        }

    if (flag_rtcm_tty_port == true)
        {
            rtcm_dev_descriptor = init_serial(rtcm_devname.c_str());
            if (rtcm_dev_descriptor != -1)
                {
                    DLOG(INFO) << "RTCM printer writing on " << rtcm_devname.c_str();
                }
        }
    else
        {
            rtcm_dev_descriptor = -1;
        }

    rtcm = std::make_unique<Rtcm>(port);

    if (flag_rtcm_server)
        {
            rtcm->run_server();
        }
}


Rtcm_Printer::~Rtcm_Printer()
{
    DLOG(INFO) << "RTCM printer destructor called.";
    if (rtcm->is_server_running())
        {
            try
                {
                    rtcm->stop_server();
                }
            catch (const boost::exception& e)
                {
                    LOG(WARNING) << "Boost exception: " << boost::diagnostic_information(e);
                }
            catch (const std::exception& ex)
                {
                    LOG(WARNING) << "STD exception: " << ex.what();
                }
        }
    if (rtcm_file_descriptor.is_open())
        {
            const auto pos = rtcm_file_descriptor.tellp();
            try
                {
                    rtcm_file_descriptor.close();
                }
            catch (const std::exception& e)
                {
                    std::cerr << e.what() << '\n';
                }
            if (pos == 0)
                {
                    errorlib::error_code ec;
                    if (!fs::remove(fs::path(rtcm_filename), ec))
                        {
                            LOG(INFO) << "Error deleting temporary RTCM file";
                        }
                }
        }
    try
        {
            close_serial();
        }
    catch (const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
}


void Rtcm_Printer::Print_Rtcm_Messages(const Rtklib_Solver* pvt_solver,
    const std::map<int, Gnss_Synchro>& gnss_observables_map,
    double rx_time,
    int32_t type_of_rx,
    int32_t rtcm_MSM_rate_ms,
    int32_t rtcm_MT1019_rate_ms,
    int32_t rtcm_MT1020_rate_ms,
    int32_t rtcm_MT1045_rate_ms,
    int32_t rtcm_MT1077_rate_ms,
    int32_t rtcm_MT1097_rate_ms,
    bool flag_write_RTCM_MSM_output,
    bool flag_write_RTCM_1019_output,
    bool flag_write_RTCM_1020_output,
    bool flag_write_RTCM_1045_output,
    bool enable_rx_clock_correction)
{
    try
        {
            if (d_rtcm_writing_started)
                {
                    switch (type_of_rx)
                        {
                        case 1:  // GPS L1 C/A
                            if (flag_write_RTCM_1019_output == true)
                                {
                                    for (const auto& gps_eph_iter : pvt_solver->gps_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1019(gps_eph_iter.second);
                                        }
                                }
                            if (flag_write_RTCM_MSM_output == true)
                                {
                                    const auto gps_eph_iter = pvt_solver->gps_ephemeris_map.cbegin();
                                    if (gps_eph_iter != pvt_solver->gps_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            break;
                        case 4:  // Galileo E1B
                        case 5:  // Galileo E5a
                        case 6:  // Galileo E5b
                            if (flag_write_RTCM_1045_output == true)
                                {
                                    for (const auto& gal_eph_iter : pvt_solver->galileo_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1045(gal_eph_iter.second);
                                        }
                                }
                            if (flag_write_RTCM_MSM_output == true)
                                {
                                    const auto gal_eph_iter = pvt_solver->galileo_ephemeris_map.cbegin();
                                    if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            break;
                        case 7:  // GPS L1 C/A + GPS L2C
                            if (flag_write_RTCM_1019_output == true)
                                {
                                    for (const auto& gps_eph_iter : pvt_solver->gps_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1019(gps_eph_iter.second);
                                        }
                                }
                            if (flag_write_RTCM_MSM_output == true)
                                {
                                    const auto gps_eph_iter = pvt_solver->gps_ephemeris_map.cbegin();
                                    const auto gps_cnav_eph_iter = pvt_solver->gps_cnav_ephemeris_map.cbegin();
                                    if ((gps_eph_iter != pvt_solver->gps_ephemeris_map.cend()) and (gps_cnav_eph_iter != pvt_solver->gps_cnav_ephemeris_map.cend()))
                                        {
                                            Print_Rtcm_MSM(7, gps_eph_iter->second, gps_cnav_eph_iter->second, {}, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            break;
                        case 8:  // L1+L5
                            if (flag_write_RTCM_1019_output == true)
                                {
                                    for (const auto& gps_eph_iter : pvt_solver->gps_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1019(gps_eph_iter.second);
                                        }
                                }
                            if (flag_write_RTCM_MSM_output == true)
                                {
                                    const auto gps_eph_iter = pvt_solver->gps_ephemeris_map.cbegin();
                                    const auto gps_cnav_eph_iter = pvt_solver->gps_cnav_ephemeris_map.cbegin();
                                    if ((gps_eph_iter != pvt_solver->gps_ephemeris_map.cend()) and (gps_cnav_eph_iter != pvt_solver->gps_cnav_ephemeris_map.cend()))
                                        {
                                            Print_Rtcm_MSM(7, gps_eph_iter->second, gps_cnav_eph_iter->second, {}, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            break;
                        case 9:  // GPS L1 C/A + Galileo E1B
                            if (flag_write_RTCM_1019_output == true)
                                {
                                    for (const auto& gps_eph_iter : pvt_solver->gps_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1019(gps_eph_iter.second);
                                        }
                                }
                            if (flag_write_RTCM_1045_output == true)
                                {
                                    for (const auto& gal_eph_iter : pvt_solver->galileo_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1045(gal_eph_iter.second);
                                        }
                                }
                            if (flag_write_RTCM_MSM_output == true)
                                {
                                    auto gps_eph_iter = pvt_solver->gps_ephemeris_map.cbegin();
                                    auto gal_eph_iter = pvt_solver->galileo_ephemeris_map.cbegin();
                                    int gps_channel = 0;
                                    int gal_channel = 0;
                                    for (const auto& gnss_observables_iter : gnss_observables_map)
                                        {
                                            const std::string system(gnss_observables_iter.second.System, 1);
                                            if (gps_channel == 0)
                                                {
                                                    if (system == "G")
                                                        {
                                                            // This is a channel with valid GPS signal
                                                            gps_eph_iter = pvt_solver->gps_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (gps_eph_iter != pvt_solver->gps_ephemeris_map.cend())
                                                                {
                                                                    gps_channel = 1;
                                                                }
                                                        }
                                                }
                                            if (gal_channel == 0)
                                                {
                                                    if (system == "E")
                                                        {
                                                            gal_eph_iter = pvt_solver->galileo_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                                                {
                                                                    gal_channel = 1;
                                                                }
                                                        }
                                                }
                                        }
                                    if (gps_eph_iter != pvt_solver->gps_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                    if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            break;
                        case 13:  // L5+E5a
                            if (flag_write_RTCM_1045_output == true)
                                {
                                    for (const auto& gal_eph_iter : pvt_solver->galileo_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1045(gal_eph_iter.second);
                                        }
                                }

                            if (flag_write_RTCM_MSM_output and rtcm_MSM_rate_ms != 0)
                                {
                                    auto gal_eph_iter = pvt_solver->galileo_ephemeris_map.cbegin();
                                    auto gps_cnav_eph_iter = pvt_solver->gps_cnav_ephemeris_map.cbegin();
                                    int gal_channel = 0;
                                    int gps_channel = 0;
                                    for (const auto& gnss_observables_iter : gnss_observables_map)
                                        {
                                            const std::string system(gnss_observables_iter.second.System, 1);
                                            if (gps_channel == 0)
                                                {
                                                    if (system == "G")
                                                        {
                                                            // This is a channel with valid GPS signal
                                                            gps_cnav_eph_iter = pvt_solver->gps_cnav_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (gps_cnav_eph_iter != pvt_solver->gps_cnav_ephemeris_map.cend())
                                                                {
                                                                    gps_channel = 1;
                                                                }
                                                        }
                                                }
                                            if (gal_channel == 0)
                                                {
                                                    if (system == "E")
                                                        {
                                                            gal_eph_iter = pvt_solver->galileo_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                                                {
                                                                    gal_channel = 1;
                                                                }
                                                        }
                                                }
                                        }

                                    if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend() and (rtcm_MT1097_rate_ms != 0))
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                    if (gps_cnav_eph_iter != pvt_solver->gps_cnav_ephemeris_map.cend() and (rtcm_MT1077_rate_ms != 0))
                                        {
                                            Print_Rtcm_MSM(7, {}, gps_cnav_eph_iter->second, {}, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            break;
                        case 14:  // Galileo E1B + Galileo E5a
                        case 15:  // Galileo E1B + Galileo E5b
                            if (flag_write_RTCM_1045_output == true)
                                {
                                    for (const auto& gal_eph_iter : pvt_solver->galileo_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1045(gal_eph_iter.second);
                                        }
                                }
                            if (flag_write_RTCM_MSM_output == true)
                                {
                                    const auto gal_eph_iter = pvt_solver->galileo_ephemeris_map.cbegin();
                                    if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            break;
                        case 23:  // GLONASS L1 C/A
                        case 24:  // GLONASS L2 C/A
                        case 25:  // GLONASS L1 C/A + GLONASS L2 C/A
                            if (flag_write_RTCM_1020_output == true)
                                {
                                    for (const auto& glonass_gnav_ephemeris_iter : pvt_solver->glonass_gnav_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1020(glonass_gnav_ephemeris_iter.second, pvt_solver->glonass_gnav_utc_model);
                                        }
                                }
                            if (flag_write_RTCM_MSM_output == true)
                                {
                                    const auto glo_gnav_ephemeris_iter = pvt_solver->glonass_gnav_ephemeris_map.cbegin();
                                    if (glo_gnav_ephemeris_iter != pvt_solver->glonass_gnav_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, {}, glo_gnav_ephemeris_iter->second, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            d_rtcm_writing_started = true;
                            break;
                        case 26:  // GPS L1 C/A + GLONASS L1 C/A
                            if (flag_write_RTCM_1019_output == true)
                                {
                                    for (const auto& gps_eph_iter : pvt_solver->gps_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1019(gps_eph_iter.second);
                                        }
                                }
                            if (flag_write_RTCM_1020_output == true)
                                {
                                    for (const auto& glonass_gnav_ephemeris_iter : pvt_solver->glonass_gnav_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1020(glonass_gnav_ephemeris_iter.second, pvt_solver->glonass_gnav_utc_model);
                                        }
                                }
                            if (flag_write_RTCM_MSM_output == true)
                                {
                                    auto glonass_gnav_eph_iter = pvt_solver->glonass_gnav_ephemeris_map.cbegin();
                                    auto gps_eph_iter = pvt_solver->gps_ephemeris_map.cbegin();
                                    int gps_channel = 0;
                                    int glo_channel = 0;
                                    for (const auto& gnss_observables_iter : gnss_observables_map)
                                        {
                                            const std::string system(gnss_observables_iter.second.System, 1);
                                            if (gps_channel == 0)
                                                {
                                                    if (system == "G")
                                                        {
                                                            // This is a channel with valid GPS signal
                                                            gps_eph_iter = pvt_solver->gps_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (gps_eph_iter != pvt_solver->gps_ephemeris_map.cend())
                                                                {
                                                                    gps_channel = 1;
                                                                }
                                                        }
                                                }
                                            if (glo_channel == 0)
                                                {
                                                    if (system == "R")
                                                        {
                                                            glonass_gnav_eph_iter = pvt_solver->glonass_gnav_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (glonass_gnav_eph_iter != pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                {
                                                                    glo_channel = 1;
                                                                }
                                                        }
                                                }
                                        }

                                    if (glonass_gnav_eph_iter != pvt_solver->glonass_gnav_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, {}, glonass_gnav_eph_iter->second, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                    if (gps_eph_iter != pvt_solver->gps_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            break;
                        case 27:  // GLONASS L1 C/A + Galileo E1B
                            if (flag_write_RTCM_1020_output == true)
                                {
                                    for (const auto& glonass_gnav_eph_iter : pvt_solver->glonass_gnav_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1020(glonass_gnav_eph_iter.second, pvt_solver->glonass_gnav_utc_model);
                                        }
                                }
                            if (flag_write_RTCM_1045_output == true)
                                {
                                    for (const auto& gal_eph_iter : pvt_solver->galileo_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1045(gal_eph_iter.second);
                                        }
                                }
                            if (flag_write_RTCM_MSM_output == true)
                                {
                                    auto gal_eph_iter = pvt_solver->galileo_ephemeris_map.cbegin();
                                    auto glonass_gnav_eph_iter = pvt_solver->glonass_gnav_ephemeris_map.cbegin();
                                    int gal_channel = 0;
                                    int glo_channel = 0;
                                    for (const auto& gnss_observables_iter : gnss_observables_map)
                                        {
                                            const std::string system(gnss_observables_iter.second.System, 1);
                                            if (gal_channel == 0)
                                                {
                                                    if (system == "E")
                                                        {
                                                            // This is a channel with valid GPS signal
                                                            gal_eph_iter = pvt_solver->galileo_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                                                {
                                                                    gal_channel = 1;
                                                                }
                                                        }
                                                }
                                            if (glo_channel == 0)
                                                {
                                                    if (system == "R")
                                                        {
                                                            glonass_gnav_eph_iter = pvt_solver->glonass_gnav_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (glonass_gnav_eph_iter != pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                {
                                                                    glo_channel = 1;
                                                                }
                                                        }
                                                }
                                        }
                                    if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                    if (glonass_gnav_eph_iter != pvt_solver->glonass_gnav_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, {}, glonass_gnav_eph_iter->second, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            break;
                        case 29:  // GPS L1 C/A + GLONASS L2 C/A
                            if (flag_write_RTCM_1019_output == true)
                                {
                                    for (const auto& gps_eph_iter : pvt_solver->gps_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1019(gps_eph_iter.second);
                                        }
                                }
                            if (flag_write_RTCM_1020_output == true)
                                {
                                    for (const auto& glonass_gnav_eph_iter : pvt_solver->glonass_gnav_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1020(glonass_gnav_eph_iter.second, pvt_solver->glonass_gnav_utc_model);
                                        }
                                }
                            if (flag_write_RTCM_MSM_output == true)
                                {
                                    auto gps_eph_iter = pvt_solver->gps_ephemeris_map.cbegin();
                                    auto glonass_gnav_eph_iter = pvt_solver->glonass_gnav_ephemeris_map.cbegin();
                                    int gps_channel = 0;
                                    int glo_channel = 0;
                                    for (const auto& gnss_observables_iter : gnss_observables_map)
                                        {
                                            const std::string system(gnss_observables_iter.second.System, 1);
                                            if (gps_channel == 0)
                                                {
                                                    if (system == "G")
                                                        {
                                                            // This is a channel with valid GPS signal
                                                            gps_eph_iter = pvt_solver->gps_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (gps_eph_iter != pvt_solver->gps_ephemeris_map.cend())
                                                                {
                                                                    gps_channel = 1;
                                                                }
                                                        }
                                                }
                                            if (glo_channel == 0)
                                                {
                                                    if (system == "R")
                                                        {
                                                            glonass_gnav_eph_iter = pvt_solver->glonass_gnav_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (glonass_gnav_eph_iter != pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                {
                                                                    glo_channel = 1;
                                                                }
                                                        }
                                                }
                                        }
                                    if (glonass_gnav_eph_iter != pvt_solver->glonass_gnav_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, {}, glonass_gnav_eph_iter->second, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                    if (gps_eph_iter != pvt_solver->gps_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            break;
                        case 30:  // GLONASS L2 C/A + Galileo E1B
                            if (flag_write_RTCM_1020_output == true)
                                {
                                    for (const auto& glonass_gnav_eph_iter : pvt_solver->glonass_gnav_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1020(glonass_gnav_eph_iter.second, pvt_solver->glonass_gnav_utc_model);
                                        }
                                }
                            if (flag_write_RTCM_1045_output == true)
                                {
                                    for (const auto& gal_eph_iter : pvt_solver->galileo_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1045(gal_eph_iter.second);
                                        }
                                }
                            if (flag_write_RTCM_MSM_output == true)
                                {
                                    auto gal_eph_iter = pvt_solver->galileo_ephemeris_map.cbegin();
                                    auto glonass_gnav_eph_iter = pvt_solver->glonass_gnav_ephemeris_map.cbegin();
                                    int gal_channel = 0;
                                    int glo_channel = 0;
                                    for (const auto& gnss_observables_iter : gnss_observables_map)
                                        {
                                            const std::string system(gnss_observables_iter.second.System, 1);
                                            if (gal_channel == 0)
                                                {
                                                    if (system == "E")
                                                        {
                                                            // This is a channel with valid GPS signal
                                                            gal_eph_iter = pvt_solver->galileo_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                                                {
                                                                    gal_channel = 1;
                                                                }
                                                        }
                                                }
                                            if (glo_channel == 0)
                                                {
                                                    if (system == "R")
                                                        {
                                                            glonass_gnav_eph_iter = pvt_solver->glonass_gnav_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (glonass_gnav_eph_iter != pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                {
                                                                    glo_channel = 1;
                                                                }
                                                        }
                                                }
                                        }
                                    if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                    if (glonass_gnav_eph_iter != pvt_solver->glonass_gnav_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, {}, glonass_gnav_eph_iter->second, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            break;
                        case 32:  // L1+E1+L5+E5a
                            if (flag_write_RTCM_1019_output == true)
                                {
                                    for (const auto& gps_eph_iter : pvt_solver->gps_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1019(gps_eph_iter.second);
                                        }
                                }
                            if (flag_write_RTCM_1045_output == true)
                                {
                                    for (const auto& gal_eph_iter : pvt_solver->galileo_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1045(gal_eph_iter.second);
                                        }
                                }
                            if (flag_write_RTCM_MSM_output == true)
                                {
                                    auto gal_eph_iter = pvt_solver->galileo_ephemeris_map.cbegin();
                                    auto gps_eph_iter = pvt_solver->gps_ephemeris_map.cbegin();
                                    int gal_channel = 0;
                                    int gps_channel = 0;
                                    for (const auto& gnss_observables_iter : gnss_observables_map)
                                        {
                                            const std::string system(gnss_observables_iter.second.System, 1);
                                            if (gal_channel == 0)
                                                {
                                                    if (system == "E")
                                                        {
                                                            // This is a channel with valid GPS signal
                                                            gal_eph_iter = pvt_solver->galileo_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                                                {
                                                                    gal_channel = 1;
                                                                }
                                                        }
                                                }
                                            if (gps_channel == 0)
                                                {
                                                    if (system == "G")
                                                        {
                                                            gps_eph_iter = pvt_solver->gps_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (gps_eph_iter != pvt_solver->gps_ephemeris_map.cend())
                                                                {
                                                                    gps_channel = 1;
                                                                }
                                                        }
                                                }
                                        }
                                    if (gps_eph_iter != pvt_solver->gps_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                    if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            break;
                        case 101:  // Galileo E1B + Galileo E6B
                        case 102:  // Galileo E5a + Galileo E6B
                        case 103:  // Galileo E5b + Galileo E6B
                        case 104:  // Galileo E1B + Galileo E5a + Galileo E6B
                        case 105:  // Galileo E1B + Galileo E5b + Galileo E6B
                            if (flag_write_RTCM_1045_output == true)
                                {
                                    for (const auto& gal_eph_iter : pvt_solver->galileo_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1045(gal_eph_iter.second);
                                        }
                                }
                            if (flag_write_RTCM_MSM_output == true)
                                {
                                    const auto gal_eph_iter = pvt_solver->galileo_ephemeris_map.cbegin();
                                    if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            break;
                        case 106:  // GPS L1 C/A + Galileo E1B + Galileo E6B
                            if (flag_write_RTCM_1019_output == true)
                                {
                                    for (const auto& gps_eph_iter : pvt_solver->gps_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1019(gps_eph_iter.second);
                                        }
                                }
                            if (rtcm_MT1045_rate_ms != 0)
                                {
                                    for (const auto& gal_eph_iter : pvt_solver->galileo_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1045(gal_eph_iter.second);
                                        }
                                }
                            if (flag_write_RTCM_MSM_output == true)
                                {
                                    auto gal_eph_iter = pvt_solver->galileo_ephemeris_map.cbegin();
                                    auto gps_eph_iter = pvt_solver->gps_ephemeris_map.cbegin();
                                    int gal_channel = 0;
                                    int gps_channel = 0;
                                    for (const auto& gnss_observables_iter : gnss_observables_map)
                                        {
                                            const std::string system(gnss_observables_iter.second.System, 1);
                                            if (gal_channel == 0)
                                                {
                                                    if (system == "E")
                                                        {
                                                            // This is a channel with valid GPS signal
                                                            gal_eph_iter = pvt_solver->galileo_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                                                {
                                                                    gal_channel = 1;
                                                                }
                                                        }
                                                }
                                            if (gps_channel == 0)
                                                {
                                                    if (system == "G")
                                                        {
                                                            gps_eph_iter = pvt_solver->gps_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (gps_eph_iter != pvt_solver->gps_ephemeris_map.cend())
                                                                {
                                                                    gps_channel = 1;
                                                                }
                                                        }
                                                }
                                        }
                                    if (gps_eph_iter != pvt_solver->gps_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                    if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            break;
                        case 107:  // GPS L1 C/A + Galileo E6B (print only GPS data)
                            if (flag_write_RTCM_1019_output == true)
                                {
                                    for (const auto& gps_eph_iter : pvt_solver->gps_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1019(gps_eph_iter.second);
                                        }
                                }
                            if (flag_write_RTCM_MSM_output == true)
                                {
                                    const auto gps_eph_iter = pvt_solver->gps_ephemeris_map.cbegin();
                                    if (gps_eph_iter != pvt_solver->gps_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            break;
                        default:
                            break;
                        }
                }

            if (!d_rtcm_writing_started)  // the first time
                {
                    switch (type_of_rx)
                        {
                        case 1:                            // GPS L1 C/A
                            if (rtcm_MT1019_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                {
                                    for (const auto& gps_eph_iter : pvt_solver->gps_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1019(gps_eph_iter.second);
                                        }
                                }
                            if (rtcm_MSM_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                {
                                    const auto gps_eph_iter = pvt_solver->gps_ephemeris_map.cbegin();
                                    if (gps_eph_iter != pvt_solver->gps_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            d_rtcm_writing_started = true;
                            break;
                        case 4:                            // Galileo E1B
                        case 5:                            // Galileo E5a
                        case 6:                            // Galileo E5b
                            if (rtcm_MT1045_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                {
                                    for (const auto& gal_eph_iter : pvt_solver->galileo_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1045(gal_eph_iter.second);
                                        }
                                }
                            if (rtcm_MSM_rate_ms != 0)
                                {
                                    const auto gal_eph_iter = pvt_solver->galileo_ephemeris_map.cbegin();
                                    if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            d_rtcm_writing_started = true;
                            break;
                        case 7:                            // GPS L1 C/A + GPS L2C
                            if (rtcm_MT1019_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                {
                                    for (const auto& gps_eph_iter : pvt_solver->gps_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1019(gps_eph_iter.second);
                                        }
                                }
                            if (rtcm_MSM_rate_ms != 0)
                                {
                                    const auto gps_eph_iter = pvt_solver->gps_ephemeris_map.cbegin();
                                    const auto gps_cnav_eph_iter = pvt_solver->gps_cnav_ephemeris_map.cbegin();
                                    if ((gps_eph_iter != pvt_solver->gps_ephemeris_map.cend()) and (gps_cnav_eph_iter != pvt_solver->gps_cnav_ephemeris_map.cend()))
                                        {
                                            Print_Rtcm_MSM(7, gps_eph_iter->second, gps_cnav_eph_iter->second, {}, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            d_rtcm_writing_started = true;
                            break;
                        case 8:                            // L1+L5
                            if (rtcm_MT1019_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                {
                                    for (const auto& gps_eph_iter : pvt_solver->gps_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1019(gps_eph_iter.second);
                                        }
                                }
                            if (rtcm_MSM_rate_ms != 0)
                                {
                                    const auto gps_eph_iter = pvt_solver->gps_ephemeris_map.cbegin();
                                    const auto gps_cnav_eph_iter = pvt_solver->gps_cnav_ephemeris_map.cbegin();
                                    if ((gps_eph_iter != pvt_solver->gps_ephemeris_map.cend()) and (gps_cnav_eph_iter != pvt_solver->gps_cnav_ephemeris_map.cend()))
                                        {
                                            Print_Rtcm_MSM(7, gps_eph_iter->second, gps_cnav_eph_iter->second, {}, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            d_rtcm_writing_started = true;
                            break;
                        case 9:                            // GPS L1 C/A + Galileo E1B
                            if (rtcm_MT1019_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                {
                                    for (const auto& gps_eph_iter : pvt_solver->gps_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1019(gps_eph_iter.second);
                                        }
                                }
                            if (rtcm_MT1045_rate_ms != 0)
                                {
                                    for (const auto& gal_eph_iter : pvt_solver->galileo_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1045(gal_eph_iter.second);
                                        }
                                }
                            if (rtcm_MSM_rate_ms != 0)
                                {
                                    auto gal_eph_iter = pvt_solver->galileo_ephemeris_map.cbegin();
                                    auto gps_eph_iter = pvt_solver->gps_ephemeris_map.cbegin();
                                    int gps_channel = 0;
                                    int gal_channel = 0;
                                    for (const auto& gnss_observables_iter : gnss_observables_map)
                                        {
                                            const std::string system(gnss_observables_iter.second.System, 1);
                                            if (gps_channel == 0)
                                                {
                                                    if (system == "G")
                                                        {
                                                            // This is a channel with valid GPS signal
                                                            gps_eph_iter = pvt_solver->gps_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (gps_eph_iter != pvt_solver->gps_ephemeris_map.cend())
                                                                {
                                                                    gps_channel = 1;
                                                                }
                                                        }
                                                }
                                            if (gal_channel == 0)
                                                {
                                                    if (system == "E")
                                                        {
                                                            gal_eph_iter = pvt_solver->galileo_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                                                {
                                                                    gal_channel = 1;
                                                                }
                                                        }
                                                }
                                        }
                                    if (gps_eph_iter != pvt_solver->gps_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                    if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            d_rtcm_writing_started = true;
                            break;

                        case 13:  // L5+E5a
                            if (rtcm_MT1045_rate_ms != 0)
                                {
                                    for (const auto& gal_eph_iter : pvt_solver->galileo_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1045(gal_eph_iter.second);
                                        }
                                }
                            if (rtcm_MSM_rate_ms != 0)
                                {
                                    auto gal_eph_iter = pvt_solver->galileo_ephemeris_map.cbegin();
                                    int gal_channel = 0;
                                    for (const auto& gnss_observables_iter : gnss_observables_map)
                                        {
                                            const std::string system(gnss_observables_iter.second.System, 1);
                                            if (gal_channel == 0)
                                                {
                                                    if (system == "E")
                                                        {
                                                            gal_eph_iter = pvt_solver->galileo_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                                                {
                                                                    gal_channel = 1;
                                                                }
                                                        }
                                                }
                                        }

                                    if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend() and (rtcm_MT1097_rate_ms != 0))
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            d_rtcm_writing_started = true;
                            break;
                        case 14:                           // Galileo E1B + Galileo E5a
                        case 15:                           // Galileo E1B + Galileo E5b
                            if (rtcm_MT1045_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                {
                                    for (const auto& gal_eph_iter : pvt_solver->galileo_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1045(gal_eph_iter.second);
                                        }
                                }
                            if (rtcm_MSM_rate_ms != 0)
                                {
                                    const auto gal_eph_iter = pvt_solver->galileo_ephemeris_map.cbegin();
                                    if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            d_rtcm_writing_started = true;
                            break;
                        case 23:                           // GLONASS L1 C/A
                        case 24:                           // GLONASS L2 C/A
                        case 25:                           // GLONASS L1 C/A + GLONASS L2 C/A
                            if (rtcm_MT1020_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                {
                                    for (const auto& glonass_gnav_eph_iter : pvt_solver->glonass_gnav_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1020(glonass_gnav_eph_iter.second, pvt_solver->glonass_gnav_utc_model);
                                        }
                                }
                            if (rtcm_MSM_rate_ms != 0)
                                {
                                    const auto glo_gnav_ephemeris_iter = pvt_solver->glonass_gnav_ephemeris_map.cbegin();
                                    if (glo_gnav_ephemeris_iter != pvt_solver->glonass_gnav_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, {}, glo_gnav_ephemeris_iter->second, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            d_rtcm_writing_started = true;
                            break;
                        case 26:                           // GPS L1 C/A + GLONASS L1 C/A
                            if (rtcm_MT1019_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                {
                                    for (const auto& gps_eph_iter : pvt_solver->gps_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1019(gps_eph_iter.second);
                                        }
                                }
                            if (rtcm_MT1020_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                {
                                    for (const auto& glonass_gnav_eph_iter : pvt_solver->glonass_gnav_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1020(glonass_gnav_eph_iter.second, pvt_solver->glonass_gnav_utc_model);
                                        }
                                }
                            if (rtcm_MSM_rate_ms != 0)
                                {
                                    auto gps_eph_iter = pvt_solver->gps_ephemeris_map.cbegin();
                                    auto glonass_gnav_eph_iter = pvt_solver->glonass_gnav_ephemeris_map.cbegin();
                                    int gps_channel = 0;
                                    int glo_channel = 0;
                                    for (const auto& gnss_observables_iter : gnss_observables_map)
                                        {
                                            const std::string system(gnss_observables_iter.second.System, 1);
                                            if (gps_channel == 0)
                                                {
                                                    if (system == "G")
                                                        {
                                                            // This is a channel with valid GPS signal
                                                            gps_eph_iter = pvt_solver->gps_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (gps_eph_iter != pvt_solver->gps_ephemeris_map.cend())
                                                                {
                                                                    gps_channel = 1;
                                                                }
                                                        }
                                                }
                                            if (glo_channel == 0)
                                                {
                                                    if (system == "R")
                                                        {
                                                            glonass_gnav_eph_iter = pvt_solver->glonass_gnav_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (glonass_gnav_eph_iter != pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                {
                                                                    glo_channel = 1;
                                                                }
                                                        }
                                                }
                                        }
                                    if (glonass_gnav_eph_iter != pvt_solver->glonass_gnav_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, {}, glonass_gnav_eph_iter->second, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                    if (gps_eph_iter != pvt_solver->gps_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            d_rtcm_writing_started = true;
                            break;
                        case 27:                           // GLONASS L1 C/A + Galileo E1B
                            if (rtcm_MT1020_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                {
                                    for (const auto& glonass_gnav_eph_iter : pvt_solver->glonass_gnav_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1020(glonass_gnav_eph_iter.second, pvt_solver->glonass_gnav_utc_model);
                                        }
                                }
                            if (rtcm_MT1045_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                {
                                    for (const auto& gal_eph_iter : pvt_solver->galileo_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1045(gal_eph_iter.second);
                                        }
                                }
                            if (rtcm_MSM_rate_ms != 0)
                                {
                                    int gal_channel = 0;
                                    int glo_channel = 0;
                                    auto gal_eph_iter = pvt_solver->galileo_ephemeris_map.cbegin();
                                    auto glonass_gnav_eph_iter = pvt_solver->glonass_gnav_ephemeris_map.cbegin();
                                    for (const auto& gnss_observables_iter : gnss_observables_map)
                                        {
                                            const std::string system(gnss_observables_iter.second.System, 1);
                                            if (gal_channel == 0)
                                                {
                                                    if (system == "E")
                                                        {
                                                            // This is a channel with valid GPS signal
                                                            gal_eph_iter = pvt_solver->galileo_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                                                {
                                                                    gal_channel = 1;
                                                                }
                                                        }
                                                }
                                            if (glo_channel == 0)
                                                {
                                                    if (system == "R")
                                                        {
                                                            glonass_gnav_eph_iter = pvt_solver->glonass_gnav_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (glonass_gnav_eph_iter != pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                {
                                                                    glo_channel = 1;
                                                                }
                                                        }
                                                }
                                        }
                                    if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                    if (glonass_gnav_eph_iter != pvt_solver->glonass_gnav_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, {}, glonass_gnav_eph_iter->second, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            d_rtcm_writing_started = true;
                            break;
                        case 29:                           // GPS L1 C/A + GLONASS L2 C/A
                            if (rtcm_MT1019_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                {
                                    for (const auto& gps_eph_iter : pvt_solver->gps_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1019(gps_eph_iter.second);
                                        }
                                }
                            if (rtcm_MT1020_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                {
                                    for (const auto& glonass_gnav_eph_iter : pvt_solver->glonass_gnav_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1020(glonass_gnav_eph_iter.second, pvt_solver->glonass_gnav_utc_model);
                                        }
                                }
                            if (rtcm_MSM_rate_ms != 0)
                                {
                                    auto gps_eph_iter = pvt_solver->gps_ephemeris_map.cbegin();
                                    auto glonass_gnav_eph_iter = pvt_solver->glonass_gnav_ephemeris_map.cbegin();
                                    int gps_channel = 0;
                                    int glo_channel = 0;
                                    for (const auto& gnss_observables_iter : gnss_observables_map)
                                        {
                                            const std::string system(gnss_observables_iter.second.System, 1);
                                            if (gps_channel == 0)
                                                {
                                                    if (system == "G")
                                                        {
                                                            // This is a channel with valid GPS signal
                                                            gps_eph_iter = pvt_solver->gps_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (gps_eph_iter != pvt_solver->gps_ephemeris_map.cend())
                                                                {
                                                                    gps_channel = 1;
                                                                }
                                                        }
                                                }
                                            if (glo_channel == 0)
                                                {
                                                    if (system == "R")
                                                        {
                                                            glonass_gnav_eph_iter = pvt_solver->glonass_gnav_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (glonass_gnav_eph_iter != pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                {
                                                                    glo_channel = 1;
                                                                }
                                                        }
                                                }
                                        }
                                    if (glonass_gnav_eph_iter != pvt_solver->glonass_gnav_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, {}, glonass_gnav_eph_iter->second, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }

                                    if (gps_eph_iter != pvt_solver->gps_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            d_rtcm_writing_started = true;
                            break;
                        case 30:                           // GLONASS L2 C/A + Galileo E1B
                            if (rtcm_MT1020_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                {
                                    for (const auto& glonass_gnav_eph_iter : pvt_solver->glonass_gnav_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1020(glonass_gnav_eph_iter.second, pvt_solver->glonass_gnav_utc_model);
                                        }
                                }
                            if (rtcm_MT1045_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                {
                                    for (const auto& gal_eph_iter : pvt_solver->galileo_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1045(gal_eph_iter.second);
                                        }
                                }
                            if (rtcm_MSM_rate_ms != 0)
                                {
                                    int gal_channel = 0;
                                    int glo_channel = 0;
                                    auto gal_eph_iter = pvt_solver->galileo_ephemeris_map.cbegin();
                                    auto glonass_gnav_eph_iter = pvt_solver->glonass_gnav_ephemeris_map.cbegin();
                                    for (const auto& gnss_observables_iter : gnss_observables_map)
                                        {
                                            const std::string system(gnss_observables_iter.second.System, 1);
                                            if (gal_channel == 0)
                                                {
                                                    if (system == "E")
                                                        {
                                                            // This is a channel with valid GPS signal
                                                            gal_eph_iter = pvt_solver->galileo_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                                                {
                                                                    gal_channel = 1;
                                                                }
                                                        }
                                                }
                                            if (glo_channel == 0)
                                                {
                                                    if (system == "R")
                                                        {
                                                            glonass_gnav_eph_iter = pvt_solver->glonass_gnav_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (glonass_gnav_eph_iter != pvt_solver->glonass_gnav_ephemeris_map.cend())
                                                                {
                                                                    glo_channel = 1;
                                                                }
                                                        }
                                                }
                                        }
                                    if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                    if (glonass_gnav_eph_iter != pvt_solver->glonass_gnav_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, {}, glonass_gnav_eph_iter->second, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            d_rtcm_writing_started = true;
                            break;
                        case 32:                           // L1+E1+L5+E5a
                            if (rtcm_MT1019_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                {
                                    for (const auto& gps_eph_iter : pvt_solver->gps_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1019(gps_eph_iter.second);
                                        }
                                }
                            if (rtcm_MT1045_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                {
                                    for (const auto& gal_eph_iter : pvt_solver->galileo_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1045(gal_eph_iter.second);
                                        }
                                }
                            if (rtcm_MSM_rate_ms != 0)
                                {
                                    auto gal_eph_iter = pvt_solver->galileo_ephemeris_map.cbegin();
                                    auto gps_eph_iter = pvt_solver->gps_ephemeris_map.cbegin();
                                    int gps_channel = 0;
                                    int gal_channel = 0;
                                    for (const auto& gnss_observables_iter : gnss_observables_map)
                                        {
                                            const std::string system(gnss_observables_iter.second.System, 1);
                                            if (gps_channel == 0)
                                                {
                                                    if (system == "G")
                                                        {
                                                            // This is a channel with valid GPS signal
                                                            gps_eph_iter = pvt_solver->gps_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (gps_eph_iter != pvt_solver->gps_ephemeris_map.cend())
                                                                {
                                                                    gps_channel = 1;
                                                                }
                                                        }
                                                }
                                            if (gal_channel == 0)
                                                {
                                                    if (system == "E")
                                                        {
                                                            gal_eph_iter = pvt_solver->galileo_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                                                {
                                                                    gal_channel = 1;
                                                                }
                                                        }
                                                }
                                        }
                                    if (gps_eph_iter != pvt_solver->gps_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                    if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            d_rtcm_writing_started = true;
                            break;
                        case 101:                          // Galileo E1B + Galileo E6B
                        case 102:                          // Galileo E5a + Galileo E6B
                        case 103:                          // Galileo E5b + Galileo E6B
                            if (rtcm_MT1045_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                {
                                    for (const auto& gal_eph_iter : pvt_solver->galileo_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1045(gal_eph_iter.second);
                                        }
                                }
                            if (rtcm_MSM_rate_ms != 0)
                                {
                                    const auto gal_eph_iter = pvt_solver->galileo_ephemeris_map.cbegin();
                                    if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            d_rtcm_writing_started = true;
                            break;
                        case 104:                          // Galileo E1B + Galileo E5a + Galileo E6B
                        case 105:                          // Galileo E1B + Galileo E5b + Galileo E6B
                            if (rtcm_MT1045_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                {
                                    for (const auto& gal_eph_iter : pvt_solver->galileo_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1045(gal_eph_iter.second);
                                        }
                                }
                            if (rtcm_MSM_rate_ms != 0)
                                {
                                    const auto gal_eph_iter = pvt_solver->galileo_ephemeris_map.cbegin();
                                    if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            d_rtcm_writing_started = true;
                            break;
                        case 106:                          // GPS L1 C/A + Galileo E1B + Galileo E6B
                            if (rtcm_MT1019_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                {
                                    for (const auto& gps_eph_iter : pvt_solver->gps_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1019(gps_eph_iter.second);
                                        }
                                }
                            if (rtcm_MT1045_rate_ms != 0)
                                {
                                    for (const auto& gal_eph_iter : pvt_solver->galileo_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1045(gal_eph_iter.second);
                                        }
                                }
                            if (rtcm_MSM_rate_ms != 0)
                                {
                                    auto gal_eph_iter = pvt_solver->galileo_ephemeris_map.cbegin();
                                    auto gps_eph_iter = pvt_solver->gps_ephemeris_map.cbegin();
                                    int gal_channel = 0;
                                    int gps_channel = 0;
                                    for (const auto& gnss_observables_iter : gnss_observables_map)
                                        {
                                            const std::string system(gnss_observables_iter.second.System, 1);
                                            if (gal_channel == 0)
                                                {
                                                    if (system == "E")
                                                        {
                                                            // This is a channel with valid GPS signal
                                                            gal_eph_iter = pvt_solver->galileo_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                                                {
                                                                    gal_channel = 1;
                                                                }
                                                        }
                                                }
                                            if (gps_channel == 0)
                                                {
                                                    if (system == "G")
                                                        {
                                                            gps_eph_iter = pvt_solver->gps_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (gps_eph_iter != pvt_solver->gps_ephemeris_map.cend())
                                                                {
                                                                    gps_channel = 1;
                                                                }
                                                        }
                                                }
                                        }
                                    if (gps_eph_iter != pvt_solver->gps_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                    if (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            d_rtcm_writing_started = true;
                            break;
                        case 107:
                            if (rtcm_MT1019_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                {
                                    for (const auto& gps_eph_iter : pvt_solver->gps_ephemeris_map)
                                        {
                                            Print_Rtcm_MT1019(gps_eph_iter.second);
                                        }
                                }
                            if (rtcm_MSM_rate_ms != 0)
                                {
                                    auto gps_eph_iter = pvt_solver->gps_ephemeris_map.cbegin();
                                    int gps_channel = 0;
                                    for (const auto& gnss_observables_iter : gnss_observables_map)
                                        {
                                            const std::string system(gnss_observables_iter.second.System, 1);
                                            if (gps_channel == 0)
                                                {
                                                    if (system == "G")
                                                        {
                                                            // This is a channel with valid GPS signal
                                                            gps_eph_iter = pvt_solver->gps_ephemeris_map.find(gnss_observables_iter.second.PRN);
                                                            if (gps_eph_iter != pvt_solver->gps_ephemeris_map.cend())
                                                                {
                                                                    gps_channel = 1;
                                                                }
                                                        }
                                                }
                                        }
                                    if (gps_eph_iter != pvt_solver->gps_ephemeris_map.cend())
                                        {
                                            Print_Rtcm_MSM(7, gps_eph_iter->second, {}, {}, {}, rx_time, gnss_observables_map, enable_rx_clock_correction, 0, 0, false, false);
                                        }
                                }
                            d_rtcm_writing_started = true;
                            break;
                        default:
                            break;
                        }
                }
        }
    catch (const boost::exception& ex)
        {
            std::cout << "RTCM boost exception: " << boost::diagnostic_information(ex) << '\n';
            LOG(ERROR) << "RTCM boost exception: " << boost::diagnostic_information(ex);
        }
    catch (const std::exception& ex)
        {
            std::cout << "RTCM std exception: " << ex.what() << '\n';
            LOG(ERROR) << "RTCM std exception: " << ex.what();
        }
}


void Rtcm_Printer::Print_IGM_Messages(const Galileo_HAS_data& has_data)
{
    try
        {
            if (has_data.header.orbit_correction_flag && has_data.header.clock_fullset_flag)
                {
                    Print_IGM03(has_data);
                }
            if (has_data.header.orbit_correction_flag && !has_data.header.clock_fullset_flag)
                {
                    Print_IGM01(has_data);
                }
            if (!has_data.header.orbit_correction_flag && has_data.header.clock_fullset_flag)
                {
                    Print_IGM02(has_data);
                }
            if (has_data.header.code_bias_flag)
                {
                    Print_IGM05(has_data);
                }
        }
    catch (const boost::exception& ex)
        {
            std::cout << "RTCM boost exception: " << boost::diagnostic_information(ex) << '\n';
            LOG(ERROR) << "RTCM boost exception: " << boost::diagnostic_information(ex);
        }
    catch (const std::exception& ex)
        {
            std::cout << "RTCM std exception: " << ex.what() << '\n';
            LOG(ERROR) << "RTCM std exception: " << ex.what();
        }
}


bool Rtcm_Printer::Print_Rtcm_MT1001(const Gps_Ephemeris& gps_eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables)
{
    const std::string m1001 = rtcm->print_MT1001(gps_eph, obs_time, observables, station_id);
    Rtcm_Printer::Print_Message(m1001);
    return true;
}


bool Rtcm_Printer::Print_Rtcm_MT1002(const Gps_Ephemeris& gps_eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables)
{
    const std::string m1002 = rtcm->print_MT1002(gps_eph, obs_time, observables, station_id);
    Rtcm_Printer::Print_Message(m1002);
    return true;
}


bool Rtcm_Printer::Print_Rtcm_MT1003(const Gps_Ephemeris& gps_eph, const Gps_CNAV_Ephemeris& cnav_eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables)
{
    const std::string m1003 = rtcm->print_MT1003(gps_eph, cnav_eph, obs_time, observables, station_id);
    Rtcm_Printer::Print_Message(m1003);
    return true;
}


bool Rtcm_Printer::Print_Rtcm_MT1004(const Gps_Ephemeris& gps_eph, const Gps_CNAV_Ephemeris& cnav_eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables)
{
    const std::string m1003 = rtcm->print_MT1004(gps_eph, cnav_eph, obs_time, observables, station_id);
    Rtcm_Printer::Print_Message(m1003);
    return true;
}


bool Rtcm_Printer::Print_Rtcm_MT1009(const Glonass_Gnav_Ephemeris& glonass_gnav_eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables)
{
    const std::string m1009 = rtcm->print_MT1009(glonass_gnav_eph, obs_time, observables, station_id);
    Rtcm_Printer::Print_Message(m1009);
    return true;
}


bool Rtcm_Printer::Print_Rtcm_MT1010(const Glonass_Gnav_Ephemeris& glonass_gnav_eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables)
{
    const std::string m1010 = rtcm->print_MT1010(glonass_gnav_eph, obs_time, observables, station_id);
    Rtcm_Printer::Print_Message(m1010);
    return true;
}


bool Rtcm_Printer::Print_Rtcm_MT1011(const Glonass_Gnav_Ephemeris& glonass_gnav_ephL1, const Glonass_Gnav_Ephemeris& glonass_gnav_ephL2, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables)
{
    const std::string m1011 = rtcm->print_MT1011(glonass_gnav_ephL1, glonass_gnav_ephL2, obs_time, observables, station_id);
    Rtcm_Printer::Print_Message(m1011);
    return true;
}


bool Rtcm_Printer::Print_Rtcm_MT1012(const Glonass_Gnav_Ephemeris& glonass_gnav_ephL1, const Glonass_Gnav_Ephemeris& glonass_gnav_ephL2, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables)
{
    const std::string m1012 = rtcm->print_MT1012(glonass_gnav_ephL1, glonass_gnav_ephL2, obs_time, observables, station_id);
    Rtcm_Printer::Print_Message(m1012);
    return true;
}


bool Rtcm_Printer::Print_Rtcm_MT1019(const Gps_Ephemeris& gps_eph)
{
    const std::string m1019 = rtcm->print_MT1019(gps_eph);
    Rtcm_Printer::Print_Message(m1019);
    return true;
}


bool Rtcm_Printer::Print_Rtcm_MT1020(const Glonass_Gnav_Ephemeris& glonass_gnav_eph, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model)
{
    const std::string m1020 = rtcm->print_MT1020(glonass_gnav_eph, glonass_gnav_utc_model);
    Rtcm_Printer::Print_Message(m1020);
    return true;
}


bool Rtcm_Printer::Print_Rtcm_MT1045(const Galileo_Ephemeris& gal_eph)
{
    const std::string m1045 = rtcm->print_MT1045(gal_eph);
    Rtcm_Printer::Print_Message(m1045);
    return true;
}


bool Rtcm_Printer::Print_Rtcm_MSM(uint32_t msm_number, const Gps_Ephemeris& gps_eph,
    const Gps_CNAV_Ephemeris& gps_cnav_eph,
    const Galileo_Ephemeris& gal_eph,
    const Glonass_Gnav_Ephemeris& glo_gnav_eph,
    double obs_time,
    const std::map<int32_t, Gnss_Synchro>& observables,
    uint32_t clock_steering_indicator,
    uint32_t external_clock_indicator,
    int32_t smooth_int,
    bool divergence_free,
    bool more_messages)
{
    std::string msm;
    if (msm_number == 1)
        {
            msm = rtcm->print_MSM_1(gps_eph, gps_cnav_eph, gal_eph, glo_gnav_eph, obs_time, observables, station_id, clock_steering_indicator, external_clock_indicator, smooth_int, divergence_free, more_messages);
        }
    else if (msm_number == 2)
        {
            msm = rtcm->print_MSM_2(gps_eph, gps_cnav_eph, gal_eph, glo_gnav_eph, obs_time, observables, station_id, clock_steering_indicator, external_clock_indicator, smooth_int, divergence_free, more_messages);
        }
    else if (msm_number == 3)
        {
            msm = rtcm->print_MSM_3(gps_eph, gps_cnav_eph, gal_eph, glo_gnav_eph, obs_time, observables, station_id, clock_steering_indicator, external_clock_indicator, smooth_int, divergence_free, more_messages);
        }
    else if (msm_number == 4)
        {
            msm = rtcm->print_MSM_4(gps_eph, gps_cnav_eph, gal_eph, glo_gnav_eph, obs_time, observables, station_id, clock_steering_indicator, external_clock_indicator, smooth_int, divergence_free, more_messages);
        }
    else if (msm_number == 5)
        {
            msm = rtcm->print_MSM_5(gps_eph, gps_cnav_eph, gal_eph, glo_gnav_eph, obs_time, observables, station_id, clock_steering_indicator, external_clock_indicator, smooth_int, divergence_free, more_messages);
        }
    else if (msm_number == 6)
        {
            msm = rtcm->print_MSM_6(gps_eph, gps_cnav_eph, gal_eph, glo_gnav_eph, obs_time, observables, station_id, clock_steering_indicator, external_clock_indicator, smooth_int, divergence_free, more_messages);
        }
    else if (msm_number == 7)
        {
            msm = rtcm->print_MSM_7(gps_eph, gps_cnav_eph, gal_eph, glo_gnav_eph, obs_time, observables, station_id, clock_steering_indicator, external_clock_indicator, smooth_int, divergence_free, more_messages);
        }
    else
        {
            return false;
        }

    Rtcm_Printer::Print_Message(msm);
    return true;
}


bool Rtcm_Printer::Print_IGM01(const Galileo_HAS_data& has_data)
{
    const std::vector<std::string> msgs = rtcm->print_IGM01(has_data);
    if (msgs.empty())
        {
            return false;
        }
    for (const auto& s : msgs)
        {
            Rtcm_Printer::Print_Message(s);
        }
    return true;
}


bool Rtcm_Printer::Print_IGM02(const Galileo_HAS_data& has_data)
{
    const std::vector<std::string> msgs = rtcm->print_IGM02(has_data);
    if (msgs.empty())
        {
            return false;
        }
    for (const auto& s : msgs)
        {
            Rtcm_Printer::Print_Message(s);
        }
    return true;
}


bool Rtcm_Printer::Print_IGM03(const Galileo_HAS_data& has_data)
{
    const std::vector<std::string> msgs = rtcm->print_IGM03(has_data);
    if (msgs.empty())
        {
            return false;
        }
    for (const auto& s : msgs)
        {
            Rtcm_Printer::Print_Message(s);
        }
    return true;
}


bool Rtcm_Printer::Print_IGM05(const Galileo_HAS_data& has_data)
{
    const std::vector<std::string> msgs = rtcm->print_IGM05(has_data);
    if (msgs.empty())
        {
            return false;
        }
    for (const auto& s : msgs)
        {
            Rtcm_Printer::Print_Message(s);
        }
    return true;
}


int Rtcm_Printer::init_serial(const std::string& serial_device)
{
    /*
     * Opens the serial device and sets the default baud rate for a RTCM transmission (9600,8,N,1)
     */
    int32_t fd = 0;
    // clang-format off
    struct termios options{};
    // clang-format on
    const int64_t BAUD = B9600;  // BAUD  =  B38400;
    const int64_t DATABITS = CS8;
    const int64_t STOPBITS = 0;
    const int64_t PARITYON = 0;
    const int64_t PARITY = 0;

    fd = open(serial_device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY | O_CLOEXEC);
    if (fd == -1)
        {
            return fd;  // failed to open TTY port
        }

    if (fcntl(fd, F_SETFL, 0) == -1)
        {
            LOG(INFO) << "Error enabling direct I/O";  // clear all flags on descriptor, enable direct I/O
        }
    tcgetattr(fd, &options);  // read serial port options

    options.c_cflag = BAUD | DATABITS | STOPBITS | PARITYON | PARITY | CLOCAL | CREAD;
    // enable receiver, set 8 bit data, ignore control lines
    // options.c_cflag |= (CLOCAL | CREAD | CS8);
    options.c_iflag = IGNPAR;

    // set the new port options
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}


void Rtcm_Printer::close_serial() const
{
    if (rtcm_dev_descriptor != -1)
        {
            close(rtcm_dev_descriptor);
        }
}


bool Rtcm_Printer::Print_Message(const std::string& message)
{
    // write to file
    if (d_rtcm_file_dump)
        {
            try
                {
                    rtcm_file_descriptor << message << '\n';
                }
            catch (const std::exception& ex)
                {
                    DLOG(INFO) << "RTCM printer cannot write on the output file " << rtcm_filename.c_str();
                    return false;
                }
        }

    // write to serial device
    if (rtcm_dev_descriptor != -1)
        {
            if (write(rtcm_dev_descriptor, message.c_str(), message.length()) == -1)
                {
                    DLOG(INFO) << "RTCM printer cannot write on serial device " << rtcm_devname.c_str();
                    std::cout << "RTCM printer cannot write on serial device " << rtcm_devname.c_str() << '\n';
                    return false;
                }
        }
    return true;
}


std::string Rtcm_Printer::print_MT1005_test()
{
    std::string test = rtcm->print_MT1005_test();
    return test;
}


uint32_t Rtcm_Printer::lock_time(const Gps_Ephemeris& eph, double obs_time, const Gnss_Synchro& gnss_synchro)
{
    return rtcm->lock_time(eph, obs_time, gnss_synchro);
}


uint32_t Rtcm_Printer::lock_time(const Gps_CNAV_Ephemeris& eph, double obs_time, const Gnss_Synchro& gnss_synchro)
{
    return rtcm->lock_time(eph, obs_time, gnss_synchro);
}


uint32_t Rtcm_Printer::lock_time(const Galileo_Ephemeris& eph, double obs_time, const Gnss_Synchro& gnss_synchro)
{
    return rtcm->lock_time(eph, obs_time, gnss_synchro);
}


uint32_t Rtcm_Printer::lock_time(const Glonass_Gnav_Ephemeris& eph, double obs_time, const Gnss_Synchro& gnss_synchro)
{
    return rtcm->lock_time(eph, obs_time, gnss_synchro);
}
