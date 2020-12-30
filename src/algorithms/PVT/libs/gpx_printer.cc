/*!
 * \file gpx_printer.cc
 * \brief Implementation of a class that prints PVT information to a gpx file
 * \author Álvaro Cebrián Juan, 2018. acebrianjuan(at)gmail.com
 *
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


#include "gpx_printer.h"
#include "pvt_solution.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <glog/logging.h>
#include <ctime>      // for tm
#include <exception>  // for exception
#include <iomanip>    // for operator<<
#include <iostream>   // for cout, cerr
#include <sstream>    // for stringstream

// clang-format off
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
// clang-format on


Gpx_Printer::Gpx_Printer(const std::string& base_path)
{
    positions_printed = false;
    indent = "  ";
    gpx_base_path = base_path;
    fs::path full_path(fs::current_path());
    const fs::path p(gpx_base_path);
    if (!fs::exists(p))
        {
            std::string new_folder;
            for (const auto& folder : fs::path(gpx_base_path))
                {
                    new_folder += folder.string();
                    errorlib::error_code ec;
                    if (!fs::exists(new_folder))
                        {
                            if (!fs::create_directory(new_folder, ec))
                                {
                                    std::cout << "Could not create the " << new_folder << " folder.\n";
                                    gpx_base_path = full_path.string();
                                }
                        }
                    new_folder += fs::path::preferred_separator;
                }
        }
    else
        {
            gpx_base_path = p.string();
        }
    if (gpx_base_path != ".")
        {
            std::cout << "GPX files will be stored at " << gpx_base_path << '\n';
        }

    gpx_base_path = gpx_base_path + fs::path::preferred_separator;
}


bool Gpx_Printer::set_headers(const std::string& filename, bool time_tag_name)
{
    const boost::posix_time::ptime pt = boost::posix_time::second_clock::local_time();
    const tm timeinfo = boost::posix_time::to_tm(pt);

    if (time_tag_name)
        {
            std::stringstream strm0;
            const int year = timeinfo.tm_year - 100;
            strm0 << year;
            const int month = timeinfo.tm_mon + 1;
            if (month < 10)
                {
                    strm0 << "0";
                }
            strm0 << month;
            const int day = timeinfo.tm_mday;
            if (day < 10)
                {
                    strm0 << "0";
                }
            strm0 << day << "_";
            const int hour = timeinfo.tm_hour;
            if (hour < 10)
                {
                    strm0 << "0";
                }
            strm0 << hour;
            const int min = timeinfo.tm_min;
            if (min < 10)
                {
                    strm0 << "0";
                }
            strm0 << min;
            const int sec = timeinfo.tm_sec;
            if (sec < 10)
                {
                    strm0 << "0";
                }
            strm0 << sec;

            gpx_filename = filename + "_" + strm0.str() + ".gpx";
        }
    else
        {
            gpx_filename = filename + ".gpx";
        }

    gpx_filename = gpx_base_path + gpx_filename;
    gpx_file.open(gpx_filename.c_str());

    if (gpx_file.is_open())
        {
            DLOG(INFO) << "GPX printer writing on " << filename.c_str();
            // Set iostream numeric format and precision
            gpx_file.setf(gpx_file.std::ofstream::fixed, gpx_file.std::ofstream::floatfield);
            gpx_file << std::setprecision(14);
            gpx_file << R"(<?xml version="1.0" encoding="UTF-8"?>)" << '\n'
                     << R"(<gpx version="1.1" creator="GNSS-SDR")" << '\n'
                     << indent << "xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd http://www.garmin.com/xmlschemas/GpxExtensions/v3 http://www.garmin.com/xmlschemas/GpxExtensionsv3.xsd http://www.garmin.com/xmlschemas/TrackPointExtension/v2 http://www.garmin.com/xmlschemas/TrackPointExtensionv2.xsd\"\n"
                     << indent << "xmlns=\"http://www.topografix.com/GPX/1/1\"\n"
                     << indent << "xmlns:gpxx=\"http://www.garmin.com/xmlschemas/GpxExtensions/v3\"\n"
                     << indent << "xmlns:gpxtpx=\"http://www.garmin.com/xmlschemas/TrackPointExtension/v2\"\n"
                     << indent << "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\">\n"
                     << indent << "<trk>\n"
                     << indent << indent << "<name>Position fixes computed by GNSS-SDR v" << GNSS_SDR_VERSION << "</name>\n"
                     << indent << indent << "<desc>GNSS-SDR position log generated at " << pt << " (local time)</desc>\n"
                     << indent << indent << "<trkseg>\n";
            return true;
        }
    std::cout << "File " << gpx_filename << " cannot be saved. Wrong permissions?\n";
    return false;
}


bool Gpx_Printer::print_position(const Pvt_Solution* const position, bool print_average_values)
{
    double latitude;
    double longitude;
    double height;

    positions_printed = true;

    const double speed_over_ground = position->get_speed_over_ground();    // expressed in m/s
    const double course_over_ground = position->get_course_over_ground();  // expressed in deg

    const double hdop = position->get_hdop();
    const double vdop = position->get_vdop();
    const double pdop = position->get_pdop();
    std::string utc_time = to_iso_extended_string(position->get_position_UTC_time());
    if (utc_time.length() < 23)
        {
            utc_time += ".";
        }
    utc_time.resize(23, '0');  // time up to ms
    utc_time.append("Z");      // UTC time zone

    if (print_average_values == false)
        {
            latitude = position->get_latitude();
            longitude = position->get_longitude();
            height = position->get_height();
        }
    else
        {
            latitude = position->get_avg_latitude();
            longitude = position->get_avg_longitude();
            height = position->get_avg_height();
        }

    if (gpx_file.is_open())
        {
            gpx_file << indent << indent << indent << "<trkpt lon=\"" << longitude << "\" lat=\"" << latitude << "\"><ele>" << height << "</ele>"
                     << "<time>" << utc_time << "</time>"
                     << "<hdop>" << hdop << "</hdop><vdop>" << vdop << "</vdop><pdop>" << pdop << "</pdop>"
                     << "<extensions><gpxtpx:TrackPointExtension>"
                     << "<gpxtpx:speed>" << speed_over_ground << "</gpxtpx:speed>"
                     << "<gpxtpx:course>" << course_over_ground << "</gpxtpx:course>"
                     << "</gpxtpx:TrackPointExtension></extensions></trkpt>\n";
            return true;
        }
    return false;
}


bool Gpx_Printer::close_file()
{
    if (gpx_file.is_open())
        {
            gpx_file << indent << indent << "</trkseg>\n"
                     << indent << "</trk>\n"
                     << "</gpx>";
            gpx_file.close();
            return true;
        }
    return false;
}


Gpx_Printer::~Gpx_Printer()
{
    DLOG(INFO) << "GPX printer destructor called.";
    try
        {
            close_file();
        }
    catch (const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    if (!positions_printed)
        {
            errorlib::error_code ec;
            if (!fs::remove(fs::path(gpx_filename), ec))
                {
                    LOG(INFO) << "Error deleting temporary GPX file";
                }
        }
}
