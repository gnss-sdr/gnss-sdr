/*!
 * \file nmea_printer.cc
 * \brief Implementation of a NMEA 2.1 printer for GNSS-SDR
 * This class provides a implementation of a subset of the NMEA-0183 standard for interfacing
 * marine electronic devices as defined by the National Marine Electronics Association (NMEA).
 * See http://www.nmea.org/ for the NMEA 183 standard
 *
 * \author Javier Arribas, 2012. jarribas(at)cttc.es
 *
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

#include "nmea_printer.h"
#include "rtklib_solution.h"
#include "rtklib_solver.h"
#include <glog/logging.h>
#include <array>
#include <cstdint>
#include <exception>
#include <fcntl.h>
#include <iostream>  // for cout, cerr
#include <termios.h>

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


Nmea_Printer::Nmea_Printer(const std::string& filename, bool flag_nmea_output_file, bool flag_nmea_tty_port, std::string nmea_dump_devname, const std::string& base_path)
{
    nmea_base_path = base_path;
    d_flag_nmea_output_file = flag_nmea_output_file;
    if (d_flag_nmea_output_file == true)
        {
            fs::path full_path(fs::current_path());
            const fs::path p(nmea_base_path);
            if (!fs::exists(p))
                {
                    std::string new_folder;
                    for (auto& folder : fs::path(nmea_base_path))
                        {
                            new_folder += folder.string();
                            errorlib::error_code ec;
                            if (!fs::exists(new_folder))
                                {
                                    if (!fs::create_directory(new_folder, ec))
                                        {
                                            std::cout << "Could not create the " << new_folder << " folder." << std::endl;
                                            nmea_base_path = full_path.string();
                                        }
                                }
                            new_folder += fs::path::preferred_separator;
                        }
                }
            else
                {
                    nmea_base_path = p.string();
                }

            if ((nmea_base_path != ".") and (d_flag_nmea_output_file == true))
                {
                    std::cout << "NMEA files will be stored at " << nmea_base_path << std::endl;
                }

            nmea_base_path = nmea_base_path + fs::path::preferred_separator;

            nmea_filename = nmea_base_path + filename;

            nmea_file_descriptor.open(nmea_filename.c_str(), std::ios::out);
            if (nmea_file_descriptor.is_open())
                {
                    DLOG(INFO) << "NMEA printer writing on " << nmea_filename.c_str();
                }
            else
                {
                    std::cout << "File " << nmea_filename << " cannot be saved. Wrong permissions?" << std::endl;
                }
        }

    nmea_devname = std::move(nmea_dump_devname);
    if (flag_nmea_tty_port == true)
        {
            nmea_dev_descriptor = init_serial(nmea_devname);
            if (nmea_dev_descriptor != -1)
                {
                    DLOG(INFO) << "NMEA printer writing on " << nmea_devname.c_str();
                }
        }
    else
        {
            nmea_dev_descriptor = -1;
        }
    print_avg_pos = false;
}


Nmea_Printer::~Nmea_Printer()
{
    auto pos = nmea_file_descriptor.tellp();
    try
        {
            if (nmea_file_descriptor.is_open())
                {
                    nmea_file_descriptor.close();
                }
        }
    catch (const std::ofstream::failure& e)
        {
            std::cerr << "Problem closing NMEA dump file: " << nmea_filename << '\n';
        }
    catch (const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    if (pos == 0)
        {
            errorlib::error_code ec;
            if (!fs::remove(fs::path(nmea_filename), ec))
                {
                    std::cerr << "Problem removing NMEA temporary file: " << nmea_filename << '\n';
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


int Nmea_Printer::init_serial(const std::string& serial_device)
{
    /*!
     * Opens the serial device and sets the default baud rate for a NMEA transmission (9600,8,N,1)
     */
    int fd = 0;
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


void Nmea_Printer::close_serial()
{
    if (nmea_dev_descriptor != -1)
        {
            close(nmea_dev_descriptor);
        }
}


bool Nmea_Printer::Print_Nmea_Line(const std::shared_ptr<Rtklib_Solver>& pvt_data, bool print_average_values)
{
    std::string GPRMC;
    std::string GPGGA;
    std::string GPGSA;
    std::string GPGSV;

    // set the new PVT data
    d_PVT_data = pvt_data;
    print_avg_pos = print_average_values;

    // generate the NMEA sentences

    // GPRMC
    GPRMC = get_GPRMC();
    // GPGGA (Global Positioning System Fixed Data)
    GPGGA = get_GPGGA();
    // GPGSA
    GPGSA = get_GPGSA();
    // GPGSV
    GPGSV = get_GPGSV();

    // write to log file
    if (d_flag_nmea_output_file)
        {
            try
                {
                    // GPRMC
                    nmea_file_descriptor << GPRMC;
                    // GPGGA (Global Positioning System Fixed Data)
                    nmea_file_descriptor << GPGGA;
                    // GPGSA
                    nmea_file_descriptor << GPGSA;
                    // GPGSV
                    nmea_file_descriptor << GPGSV;
                }
            catch (const std::exception& ex)
                {
                    DLOG(INFO) << "NMEA printer can not write on output file" << nmea_filename.c_str();
                }
        }

    // write to serial device
    if (nmea_dev_descriptor != -1)
        {
            if (write(nmea_dev_descriptor, GPRMC.c_str(), GPRMC.length()) == -1)
                {
                    DLOG(INFO) << "NMEA printer cannot write on serial device" << nmea_devname.c_str();
                    return false;
                }
            if (write(nmea_dev_descriptor, GPGGA.c_str(), GPGGA.length()) == -1)
                {
                    DLOG(INFO) << "NMEA printer cannot write on serial device" << nmea_devname.c_str();
                    return false;
                }
            if (write(nmea_dev_descriptor, GPGSA.c_str(), GPGSA.length()) == -1)
                {
                    DLOG(INFO) << "NMEA printer cannot write on serial device" << nmea_devname.c_str();
                    return false;
                }
            if (write(nmea_dev_descriptor, GPGSV.c_str(), GPGSV.length()) == -1)
                {
                    DLOG(INFO) << "NMEA printer cannot write on serial device" << nmea_devname.c_str();
                    return false;
                }
        }
    return true;
}


char Nmea_Printer::checkSum(const std::string& sentence)
{
    char check = 0;
    // iterate over the string, XOR each byte with the total sum:
    for (char c : sentence)
        {
            check = char(check ^ c);
        }
    // return the result
    return check;
}


std::string Nmea_Printer::latitude_to_hm(double lat)
{
    bool north;
    if (lat < 0.0)
        {
            north = false;
            lat = -lat;
        }
    else
        {
            north = true;
        }

    int deg = static_cast<int>(lat);
    double mins = lat - static_cast<double>(deg);
    mins *= 60.0;
    std::ostringstream out_string;
    out_string.setf(std::ios::fixed, std::ios::floatfield);
    out_string.fill('0');
    out_string.width(2);
    out_string << deg;
    out_string.width(2);
    out_string << static_cast<int>(mins) << ".";
    out_string.width(4);
    out_string << static_cast<int>((mins - static_cast<double>(static_cast<int>(mins))) * 1e4);

    if (north == true)
        {
            out_string << ",N";
        }
    else
        {
            out_string << ",S";
        }
    return out_string.str();
}


std::string Nmea_Printer::longitude_to_hm(double longitude)
{
    bool east;
    if (longitude < 0.0)
        {
            east = false;
            longitude = -longitude;
        }
    else
        {
            east = true;
        }
    int deg = static_cast<int>(longitude);
    double mins = longitude - static_cast<double>(deg);
    mins *= 60.0;
    std::ostringstream out_string;
    out_string.setf(std::ios::fixed, std::ios::floatfield);
    out_string.width(3);
    out_string.fill('0');
    out_string << deg;
    out_string.width(2);
    out_string << static_cast<int>(mins) << ".";
    out_string.width(4);
    out_string << static_cast<int>((mins - static_cast<double>(static_cast<int>(mins))) * 1e4);

    if (east == true)
        {
            out_string << ",E";
        }
    else
        {
            out_string << ",W";
        }
    return out_string.str();
}


std::string Nmea_Printer::get_UTC_NMEA_time(boost::posix_time::ptime d_position_UTC_time)
{
    // UTC Time: hhmmss.sss
    std::stringstream sentence_str;

    boost::posix_time::time_duration td = d_position_UTC_time.time_of_day();
    int utc_hours;
    int utc_mins;
    int utc_seconds;
    int utc_milliseconds;

    utc_hours = td.hours();
    utc_mins = td.minutes();
    utc_seconds = td.seconds();
    utc_milliseconds = td.total_milliseconds() - td.total_seconds() * 1000;

    if (utc_hours < 10)
        {
            sentence_str << "0";  //  two digits for hours
        }
    sentence_str << utc_hours;

    if (utc_mins < 10)
        {
            sentence_str << "0";  //  two digits for minutes
        }
    sentence_str << utc_mins;

    if (utc_seconds < 10)
        {
            sentence_str << "0";  //  two digits for seconds
        }
    sentence_str << utc_seconds;

    if (utc_milliseconds < 10)
        {
            sentence_str << ".00";  //  three digits for ms
            sentence_str << utc_milliseconds;
        }
    else if (utc_milliseconds < 100)
        {
            sentence_str << ".0";  //   three digits for ms
            sentence_str << utc_milliseconds;
        }
    else
        {
            sentence_str << ".";  //   three digits for ms
            sentence_str << utc_milliseconds;
        }
    return sentence_str.str();
}


std::string Nmea_Printer::get_GPRMC()
{
    // Sample -> $GPRMC,161229.487,A,3723.2475,N,12158.3416,W,0.13,309.62,120598,*10
    std::stringstream sentence_str;
    std::array<unsigned char, 1024> buff{};
    outnmea_rmc(buff.data(), &d_PVT_data->pvt_sol);
    sentence_str << buff.data();
    return sentence_str.str();
}


std::string Nmea_Printer::get_GPGSA()
{
    // $GPGSA,A,3,07,02,26,27,09,04,15, , , , , ,1.8,1.0,1.5*33
    // GSA-GNSS DOP and Active Satellites
    std::stringstream sentence_str;
    std::array<unsigned char, 1024> buff{};
    outnmea_gsa(buff.data(), &d_PVT_data->pvt_sol, d_PVT_data->pvt_ssat.data());
    sentence_str << buff.data();
    return sentence_str.str();
}


std::string Nmea_Printer::get_GPGSV()
{
    // GSV-GNSS Satellites in View
    // $GPGSV,2,1,07,07,79,048,42,02,51,062,43,26,36,256,42,27,27,138,42*71
    // Notice that NMEA 2.1 only supports 12 channels
    std::stringstream sentence_str;
    std::array<unsigned char, 1024> buff{};
    outnmea_gsv(buff.data(), &d_PVT_data->pvt_sol, d_PVT_data->pvt_ssat.data());
    sentence_str << buff.data();
    return sentence_str.str();
}


std::string Nmea_Printer::get_GPGGA()
{
    std::stringstream sentence_str;
    std::array<unsigned char, 1024> buff{};
    outnmea_gga(buff.data(), &d_PVT_data->pvt_sol);
    sentence_str << buff.data();
    return sentence_str.str();
    // $GPGGA,104427.591,5920.7009,N,01803.2938,E,1,05,3.3,78.2,M,23.2,M,0.0,0000*4A
}
