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
#include <ctime>      // for tm
#include <exception>  // for exception
#include <fcntl.h>    // for O_RDWR
#include <iostream>   // for cout, cerr
#include <iterator>   // for next
#include <set>        // for set
#include <termios.h>  // for tcgetattr
#include <unistd.h>   // for close, write
#include <utility>    // for make_pair
#include <vector>     // for std::vector

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

namespace
{
constexpr uint32_t rtcm_msm_max_cell_mask_bits = 64;


uint32_t msm_cell_mask_bits(const std::map<int32_t, Gnss_Synchro>& observables)
{
    std::set<uint32_t> satellites;
    std::set<std::string> signals;
    for (const auto& observable : observables)
        {
            satellites.insert(observable.second.PRN);
            const std::string signal_(observable.second.Signal);
            signals.insert(std::string(1, observable.second.System) + signal_.substr(0, 2));
        }
    return static_cast<uint32_t>(satellites.size() * signals.size());
}


std::vector<std::map<int32_t, Gnss_Synchro>> split_MSM_observables(const std::map<int32_t, Gnss_Synchro>& observables)
{
    std::map<uint32_t, std::map<int32_t, Gnss_Synchro>> observables_by_satellite;
    for (const auto& observable : observables)
        {
            observables_by_satellite[observable.second.PRN].insert(observable);
        }

    std::vector<std::map<int32_t, Gnss_Synchro>> blocks;
    std::map<int32_t, Gnss_Synchro> current_block;
    for (const auto& satellite_observables : observables_by_satellite)
        {
            std::map<int32_t, Gnss_Synchro> candidate_block(current_block);
            candidate_block.insert(satellite_observables.second.cbegin(), satellite_observables.second.cend());
            if (!current_block.empty() && (msm_cell_mask_bits(candidate_block) > rtcm_msm_max_cell_mask_bits))
                {
                    blocks.push_back(current_block);
                    current_block.clear();
                    candidate_block = satellite_observables.second;
                }

            if (msm_cell_mask_bits(candidate_block) <= rtcm_msm_max_cell_mask_bits)
                {
                    current_block.insert(satellite_observables.second.cbegin(), satellite_observables.second.cend());
                    continue;
                }

            for (const auto& observable : satellite_observables.second)
                {
                    std::map<int32_t, Gnss_Synchro> signal_candidate(current_block);
                    signal_candidate.insert(observable);
                    if (!current_block.empty() && (msm_cell_mask_bits(signal_candidate) > rtcm_msm_max_cell_mask_bits))
                        {
                            blocks.push_back(current_block);
                            current_block.clear();
                        }
                    current_block.insert(observable);
                }
        }

    if (!current_block.empty())
        {
            blocks.push_back(current_block);
        }

    return blocks;
}
}  // namespace

Rtcm_Printer::Rtcm_Printer(const std::string& filename,
    bool flag_rtcm_file_dump,
    bool flag_rtcm_server,
    bool flag_rtcm_tty_port,
    uint16_t rtcm_tcp_port,
    uint16_t rtcm_station_id,
    const std::string& rtcm_dump_devname,
    uint32_t signal_enabled_flags,
    bool time_tag_name,
    const std::string& base_path) : rtcm_base_path(base_path),
                                    rtcm_devname(rtcm_dump_devname),
                                    port(rtcm_tcp_port),
                                    station_id(rtcm_station_id),
                                    d_rtcm_has_written_once(false),
                                    d_rtcm_file_dump(flag_rtcm_file_dump),
                                    d_flags(signal_enabled_flags)
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
    bool rtcm_MSM_enabled,
    bool rtcm_MT1019_enabled,
    bool rtcm_MT1020_enabled,
    bool rtcm_MT1045_enabled,
    bool rtcm_MT1077_enabled,
    bool rtcm_MT1087_enabled,
    bool rtcm_MT1097_enabled,
    bool flag_write_RTCM_MSM_output,
    bool flag_write_RTCM_1019_output,
    bool flag_write_RTCM_1020_output,
    bool flag_write_RTCM_1045_output,
    bool enable_rx_clock_correction)
{
    try
        {
            const auto print_MT1019 = (!d_rtcm_has_written_once && rtcm_MT1019_enabled) || flag_write_RTCM_1019_output;
            const auto print_MT1020 = (!d_rtcm_has_written_once && rtcm_MT1020_enabled) || flag_write_RTCM_1020_output;
            const auto print_MT1045 = (!d_rtcm_has_written_once && rtcm_MT1045_enabled) || flag_write_RTCM_1045_output;
            const auto print_MSM = (!d_rtcm_has_written_once && rtcm_MSM_enabled) || flag_write_RTCM_MSM_output;

            if (print_MT1019 && d_flags.check_any_enabled(GPS_1C))
                {
                    for (const auto& gps_eph_iter : pvt_solver->gps_ephemeris_map)
                        {
                            Print_Rtcm_MT1019(gps_eph_iter.second);
                        }
                }
            if (print_MT1020 && d_flags.has_glonass)
                {
                    for (const auto& glonass_gnav_eph_iter : pvt_solver->glonass_gnav_ephemeris_map)
                        {
                            Print_Rtcm_MT1020(glonass_gnav_eph_iter.second, pvt_solver->glonass_gnav_utc_model);
                        }
                }
            if (print_MT1045 && d_flags.has_galileo)
                {
                    for (const auto& gal_eph_iter : pvt_solver->galileo_ephemeris_map)
                        {
                            Print_Rtcm_MT1045(gal_eph_iter.second);
                        }
                }
            if (print_MSM)
                {
                    std::map<int32_t, Gnss_Synchro> gps_observables;
                    std::map<int32_t, Gnss_Synchro> galileo_observables;
                    std::map<int32_t, Gnss_Synchro> glonass_observables;

                    for (const auto& gnss_observables_iter : gnss_observables_map)
                        {
                            const std::string signal_(gnss_observables_iter.second.Signal);
                            const std::string signal = signal_.substr(0, 2);
                            switch (gnss_observables_iter.second.System)
                                {
                                case 'G':
                                    if ((signal == "1C") || (signal == "2S") || (signal == "5X"))
                                        {
                                            gps_observables.insert(std::make_pair(static_cast<int32_t>(gnss_observables_iter.first), gnss_observables_iter.second));
                                        }
                                    break;
                                case 'E':
                                    if ((signal == "1B") || (signal == "5X") || (signal == "7X"))
                                        {
                                            galileo_observables.insert(std::make_pair(static_cast<int32_t>(gnss_observables_iter.first), gnss_observables_iter.second));
                                        }
                                    break;
                                case 'R':
                                    if ((signal == "1C") || (signal == "2C"))
                                        {
                                            glonass_observables.insert(std::make_pair(static_cast<int32_t>(gnss_observables_iter.first), gnss_observables_iter.second));
                                        }
                                    break;
                                default:
                                    break;
                                }
                        }

                    const auto get_observation_time_s = [rx_time](const std::map<int32_t, Gnss_Synchro>& observables) -> double {
                        if (observables.empty())
                            {
                                return rx_time;
                            }
                        return observables.cbegin()->second.RX_time;
                    };

                    auto gps_eph_iter = pvt_solver->gps_ephemeris_map.cend();
                    auto gps_cnav_eph_iter = pvt_solver->gps_cnav_ephemeris_map.cend();
                    for (const auto& gps_observables_iter : gps_observables)
                        {
                            if (gps_eph_iter == pvt_solver->gps_ephemeris_map.cend())
                                {
                                    gps_eph_iter = pvt_solver->gps_ephemeris_map.find(gps_observables_iter.second.PRN);
                                }
                            if (gps_cnav_eph_iter == pvt_solver->gps_cnav_ephemeris_map.cend())
                                {
                                    gps_cnav_eph_iter = pvt_solver->gps_cnav_ephemeris_map.find(gps_observables_iter.second.PRN);
                                }
                        }

                    auto gal_eph_iter = pvt_solver->galileo_ephemeris_map.cend();
                    for (const auto& galileo_observables_iter : galileo_observables)
                        {
                            if (gal_eph_iter == pvt_solver->galileo_ephemeris_map.cend())
                                {
                                    gal_eph_iter = pvt_solver->galileo_ephemeris_map.find(galileo_observables_iter.second.PRN);
                                }
                        }

                    auto glonass_gnav_eph_iter = pvt_solver->glonass_gnav_ephemeris_map.cend();
                    for (const auto& glonass_observables_iter : glonass_observables)
                        {
                            if (glonass_gnav_eph_iter == pvt_solver->glonass_gnav_ephemeris_map.cend())
                                {
                                    glonass_gnav_eph_iter = pvt_solver->glonass_gnav_ephemeris_map.find(glonass_observables_iter.second.PRN);
                                }
                        }

                    const bool print_gps_msm = rtcm_MT1077_enabled && !gps_observables.empty() &&
                                               ((gps_eph_iter != pvt_solver->gps_ephemeris_map.cend()) ||
                                                   (gps_cnav_eph_iter != pvt_solver->gps_cnav_ephemeris_map.cend()));
                    const bool print_galileo_msm = rtcm_MT1097_enabled && !galileo_observables.empty() &&
                                                   (gal_eph_iter != pvt_solver->galileo_ephemeris_map.cend());
                    const bool print_glonass_msm = rtcm_MT1087_enabled && !glonass_observables.empty() &&
                                                   (glonass_gnav_eph_iter != pvt_solver->glonass_gnav_ephemeris_map.cend());

                    uint32_t pending_msm_messages = 0;
                    if (print_gps_msm)
                        {
                            pending_msm_messages++;
                        }
                    if (print_galileo_msm)
                        {
                            pending_msm_messages++;
                        }
                    if (print_glonass_msm)
                        {
                            pending_msm_messages++;
                        }

                    if (print_gps_msm)
                        {
                            Gps_Ephemeris gps_eph;
                            Gps_CNAV_Ephemeris gps_cnav_eph;
                            if (gps_eph_iter != pvt_solver->gps_ephemeris_map.cend())
                                {
                                    gps_eph = gps_eph_iter->second;
                                }
                            if (gps_cnav_eph_iter != pvt_solver->gps_cnav_ephemeris_map.cend())
                                {
                                    gps_cnav_eph = gps_cnav_eph_iter->second;
                                }
                            pending_msm_messages--;
                            Print_Rtcm_MSM(7, gps_eph, gps_cnav_eph, {}, {}, get_observation_time_s(gps_observables), gps_observables, enable_rx_clock_correction, 0, 0, false, pending_msm_messages > 0);
                        }
                    if (print_galileo_msm)
                        {
                            pending_msm_messages--;
                            Print_Rtcm_MSM(7, {}, {}, gal_eph_iter->second, {}, get_observation_time_s(galileo_observables), galileo_observables, enable_rx_clock_correction, 0, 0, false, pending_msm_messages > 0);
                        }
                    if (print_glonass_msm)
                        {
                            pending_msm_messages--;
                            Print_Rtcm_MSM(7, {}, {}, {}, glonass_gnav_eph_iter->second, get_observation_time_s(glonass_observables), glonass_observables, enable_rx_clock_correction, 0, 0, false, pending_msm_messages > 0);
                        }
                }
            d_rtcm_has_written_once = true;
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
            const bool has_orbit_corrections = has_data.header.orbit_correction_flag;
            const bool has_clock_fullset_corrections = has_data.header.clock_fullset_flag;
            const bool has_clock_subset_corrections = has_data.header.clock_subset_flag;

            if (has_orbit_corrections && has_clock_fullset_corrections)
                {
                    Print_IGM03(has_data);
                }
            if (has_orbit_corrections && !has_clock_fullset_corrections)
                {
                    Print_IGM01(has_data);
                }
            if (!has_orbit_corrections && has_clock_fullset_corrections)
                {
                    Print_IGM02(has_data);
                }
            if (has_clock_subset_corrections)
                {
                    Print_IGM02(has_data, true);
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
    const std::vector<std::map<int32_t, Gnss_Synchro>> observable_blocks = split_MSM_observables(observables);
    bool printed_any_message = false;
    bool failed_to_print = false;

    for (auto block_iter = observable_blocks.cbegin(); block_iter != observable_blocks.cend(); ++block_iter)
        {
            const bool block_more_messages = (std::next(block_iter) != observable_blocks.cend()) || more_messages;
            std::string msm;
            if (msm_number == 1)
                {
                    msm = rtcm->print_MSM_1(gps_eph, gps_cnav_eph, gal_eph, glo_gnav_eph, obs_time, *block_iter, station_id, clock_steering_indicator, external_clock_indicator, smooth_int, divergence_free, block_more_messages);
                }
            else if (msm_number == 2)
                {
                    msm = rtcm->print_MSM_2(gps_eph, gps_cnav_eph, gal_eph, glo_gnav_eph, obs_time, *block_iter, station_id, clock_steering_indicator, external_clock_indicator, smooth_int, divergence_free, block_more_messages);
                }
            else if (msm_number == 3)
                {
                    msm = rtcm->print_MSM_3(gps_eph, gps_cnav_eph, gal_eph, glo_gnav_eph, obs_time, *block_iter, station_id, clock_steering_indicator, external_clock_indicator, smooth_int, divergence_free, block_more_messages);
                }
            else if (msm_number == 4)
                {
                    msm = rtcm->print_MSM_4(gps_eph, gps_cnav_eph, gal_eph, glo_gnav_eph, obs_time, *block_iter, station_id, clock_steering_indicator, external_clock_indicator, smooth_int, divergence_free, block_more_messages);
                }
            else if (msm_number == 5)
                {
                    msm = rtcm->print_MSM_5(gps_eph, gps_cnav_eph, gal_eph, glo_gnav_eph, obs_time, *block_iter, station_id, clock_steering_indicator, external_clock_indicator, smooth_int, divergence_free, block_more_messages);
                }
            else if (msm_number == 6)
                {
                    msm = rtcm->print_MSM_6(gps_eph, gps_cnav_eph, gal_eph, glo_gnav_eph, obs_time, *block_iter, station_id, clock_steering_indicator, external_clock_indicator, smooth_int, divergence_free, block_more_messages);
                }
            else if (msm_number == 7)
                {
                    msm = rtcm->print_MSM_7(gps_eph, gps_cnav_eph, gal_eph, glo_gnav_eph, obs_time, *block_iter, station_id, clock_steering_indicator, external_clock_indicator, smooth_int, divergence_free, block_more_messages);
                }
            else
                {
                    return false;
                }

            if (msm.empty())
                {
                    failed_to_print = true;
                    continue;
                }
            Rtcm_Printer::Print_Message(msm);
            printed_any_message = true;
        }

    return printed_any_message && !failed_to_print;
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


bool Rtcm_Printer::Print_IGM02(const Galileo_HAS_data& has_data, bool use_clock_subset)
{
    const std::vector<std::string> msgs = rtcm->print_IGM02(has_data, use_clock_subset);
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
