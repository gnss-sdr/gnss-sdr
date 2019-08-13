/*!
 * \file rtcm_printer.cc
 * \brief Implementation of a RTCM 3.2 printer for GNSS-SDR
 * This class provides a implementation of a subset of the RTCM Standard 10403.2
 * for Differential GNSS Services
 *
 * \author Carles Fernandez-Prades, 2014. cfernandez(at)cttc.es
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

#include "rtcm_printer.h"
#include "galileo_ephemeris.h"
#include "glonass_gnav_ephemeris.h"
#include "glonass_gnav_utc_model.h"
#include "gnss_synchro.h"
#include "gps_cnav_ephemeris.h"
#include "gps_ephemeris.h"
#include "rtcm.h"
#include <boost/exception/diagnostic_information.hpp>
#include <glog/logging.h>
#include <ctime>      // for tm
#include <exception>  // for exception
#include <fcntl.h>    // for O_RDWR
#include <iostream>   // for cout, cerr
#include <termios.h>  // for tcgetattr
#include <unistd.h>   // for close, write

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
#include <boost/filesystem/operations.hpp>   // for create_directories, exists
#include <boost/filesystem/path.hpp>         // for path, operator<<
#include <boost/filesystem/path_traits.hpp>  // for filesystem
#include <boost/system/error_code.hpp>       // for error_code
namespace fs = boost::filesystem;
namespace errorlib = boost::system;
#endif


Rtcm_Printer::Rtcm_Printer(const std::string& filename, bool flag_rtcm_file_dump, bool flag_rtcm_server, bool flag_rtcm_tty_port, uint16_t rtcm_tcp_port, uint16_t rtcm_station_id, const std::string& rtcm_dump_devname, bool time_tag_name, const std::string& base_path)
{
    boost::posix_time::ptime pt = boost::posix_time::second_clock::local_time();
    tm timeinfo = boost::posix_time::to_tm(pt);
    d_rtcm_file_dump = flag_rtcm_file_dump;
    rtcm_base_path = base_path;
    if (d_rtcm_file_dump)
        {
            fs::path full_path(fs::current_path());
            const fs::path p(rtcm_base_path);
            if (!fs::exists(p))
                {
                    std::string new_folder;
                    for (auto& folder : fs::path(rtcm_base_path))
                        {
                            new_folder += folder.string();
                            errorlib::error_code ec;
                            if (!fs::exists(new_folder))
                                {
                                    if (!fs::create_directory(new_folder, ec))
                                        {
                                            std::cout << "Could not create the " << new_folder << " folder." << std::endl;
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
                    std::cout << "RTCM binary file will be stored at " << rtcm_base_path << std::endl;
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
                    std::cout << "File " << rtcm_filename << "cannot be saved. Wrong permissions?" << std::endl;
                }
        }

    rtcm_devname = rtcm_dump_devname;
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

    port = rtcm_tcp_port;
    station_id = rtcm_station_id;

    rtcm = std::make_shared<Rtcm>(port);

    if (flag_rtcm_server)
        {
            rtcm->run_server();
        }
}


Rtcm_Printer::~Rtcm_Printer()
{
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
            int64_t pos;
            pos = rtcm_file_descriptor.tellp();
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


bool Rtcm_Printer::Print_Rtcm_MT1001(const Gps_Ephemeris& gps_eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables)
{
    std::string m1001 = rtcm->print_MT1001(gps_eph, obs_time, observables, station_id);
    Rtcm_Printer::Print_Message(m1001);
    return true;
}


bool Rtcm_Printer::Print_Rtcm_MT1002(const Gps_Ephemeris& gps_eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables)
{
    std::string m1002 = rtcm->print_MT1002(gps_eph, obs_time, observables, station_id);
    Rtcm_Printer::Print_Message(m1002);
    return true;
}


bool Rtcm_Printer::Print_Rtcm_MT1003(const Gps_Ephemeris& gps_eph, const Gps_CNAV_Ephemeris& cnav_eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables)
{
    std::string m1003 = rtcm->print_MT1003(gps_eph, cnav_eph, obs_time, observables, station_id);
    Rtcm_Printer::Print_Message(m1003);
    return true;
}


bool Rtcm_Printer::Print_Rtcm_MT1004(const Gps_Ephemeris& gps_eph, const Gps_CNAV_Ephemeris& cnav_eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables)
{
    std::string m1003 = rtcm->print_MT1004(gps_eph, cnav_eph, obs_time, observables, station_id);
    Rtcm_Printer::Print_Message(m1003);
    return true;
}


bool Rtcm_Printer::Print_Rtcm_MT1009(const Glonass_Gnav_Ephemeris& glonass_gnav_eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables)
{
    std::string m1009 = rtcm->print_MT1009(glonass_gnav_eph, obs_time, observables, station_id);
    Rtcm_Printer::Print_Message(m1009);
    return true;
}


bool Rtcm_Printer::Print_Rtcm_MT1010(const Glonass_Gnav_Ephemeris& glonass_gnav_eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables)
{
    std::string m1010 = rtcm->print_MT1010(glonass_gnav_eph, obs_time, observables, station_id);
    Rtcm_Printer::Print_Message(m1010);
    return true;
}


bool Rtcm_Printer::Print_Rtcm_MT1011(const Glonass_Gnav_Ephemeris& glonass_gnav_ephL1, const Glonass_Gnav_Ephemeris& glonass_gnav_ephL2, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables)
{
    std::string m1011 = rtcm->print_MT1011(glonass_gnav_ephL1, glonass_gnav_ephL2, obs_time, observables, station_id);
    Rtcm_Printer::Print_Message(m1011);
    return true;
}


bool Rtcm_Printer::Print_Rtcm_MT1012(const Glonass_Gnav_Ephemeris& glonass_gnav_ephL1, const Glonass_Gnav_Ephemeris& glonass_gnav_ephL2, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables)
{
    std::string m1012 = rtcm->print_MT1012(glonass_gnav_ephL1, glonass_gnav_ephL2, obs_time, observables, station_id);
    Rtcm_Printer::Print_Message(m1012);
    return true;
}


bool Rtcm_Printer::Print_Rtcm_MT1019(const Gps_Ephemeris& gps_eph)
{
    std::string m1019 = rtcm->print_MT1019(gps_eph);
    Rtcm_Printer::Print_Message(m1019);
    return true;
}


bool Rtcm_Printer::Print_Rtcm_MT1020(const Glonass_Gnav_Ephemeris& glonass_gnav_eph, const Glonass_Gnav_Utc_Model& glonass_gnav_utc_model)
{
    std::string m1020 = rtcm->print_MT1020(glonass_gnav_eph, glonass_gnav_utc_model);
    Rtcm_Printer::Print_Message(m1020);
    return true;
}


bool Rtcm_Printer::Print_Rtcm_MT1045(const Galileo_Ephemeris& gal_eph)
{
    std::string m1045 = rtcm->print_MT1045(gal_eph);
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


int Rtcm_Printer::init_serial(const std::string& serial_device)
{
    /*
     * Opens the serial device and sets the default baud rate for a RTCM transmission (9600,8,N,1)
     */
    int32_t fd = 0;
    // clang-format off
    struct termios options{};
    // clang-format on
    int64_t BAUD;
    int64_t DATABITS;
    int64_t STOPBITS;
    int64_t PARITYON;
    int64_t PARITY;

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

    BAUD = B9600;
    // BAUD  =  B38400;
    DATABITS = CS8;
    STOPBITS = 0;
    PARITYON = 0;
    PARITY = 0;

    options.c_cflag = BAUD | DATABITS | STOPBITS | PARITYON | PARITY | CLOCAL | CREAD;
    // enable receiver, set 8 bit data, ignore control lines
    // options.c_cflag |= (CLOCAL | CREAD | CS8);
    options.c_iflag = IGNPAR;

    // set the new port options
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}


void Rtcm_Printer::close_serial()
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
                    rtcm_file_descriptor << message << std::endl;
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
                    std::cout << "RTCM printer cannot write on serial device " << rtcm_devname.c_str() << std::endl;
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
