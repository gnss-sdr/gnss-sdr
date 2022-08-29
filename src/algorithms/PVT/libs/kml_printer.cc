/*!
 * \file kml_printer.cc
 * \brief Implementation of a class that prints PVT information to a kml file
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *         Álvaro Cebrián Juan, 2018. acebrianjuan(at)gmail.com
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

#include "kml_printer.h"
#include "gnss_sdr_filesystem.h"
#include "pvt_solution.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <glog/logging.h>
#include <cstdlib>    // for mkstemp
#include <ctime>      // for tm
#include <exception>  // for exception
#include <iomanip>    // for std::setprecision
#include <iostream>   // for cout, cerr
#include <sstream>
#include <sys/stat.h>   // for S_IXUSR | S_IRWXG | S_IRWXO
#include <sys/types.h>  // for mode_t


Kml_Printer::Kml_Printer(const std::string& base_path) : kml_base_path(base_path),
                                                         indent("  "),
                                                         positions_printed(false)
{
    fs::path full_path(fs::current_path());
    const fs::path p(kml_base_path);
    if (!fs::exists(p))
        {
            std::string new_folder;
            for (const auto& folder : fs::path(kml_base_path))
                {
                    new_folder += folder.string();
                    errorlib::error_code ec;
                    if (!fs::exists(new_folder))
                        {
                            if (!fs::create_directory(new_folder, ec))
                                {
                                    std::cout << "Could not create the " << new_folder << " folder.\n";
                                    kml_base_path = full_path.string();
                                }
                        }
                    new_folder += fs::path::preferred_separator;
                }
        }
    else
        {
            kml_base_path = p.string();
        }
    if (kml_base_path != ".")
        {
            std::cout << "KML files will be stored at " << kml_base_path << '\n';
        }

    kml_base_path = kml_base_path + fs::path::preferred_separator;

    char tmp_filename_[] = "/tmp/file.XXXXXX";
    mode_t mask = umask(S_IXUSR | S_IRWXG | S_IRWXO);
    int fd = mkstemp(tmp_filename_);
    if (fd == -1)
        {
            std::cerr << "Error in KML printer: failed to create temporary file\n";
        }
    else
        {
            close(fd);
        }
    umask(mask);
    fs::path tmp_filename = fs::path(tmp_filename_);

    tmp_file_str = tmp_filename.string();

    point_id = 0;
}


bool Kml_Printer::set_headers(const std::string& filename, bool time_tag_name)
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

            kml_filename = filename + "_" + strm0.str() + ".kml";
        }
    else
        {
            kml_filename = filename + ".kml";
        }
    kml_filename = kml_base_path + kml_filename;
    kml_file.open(kml_filename.c_str());

    tmp_file.open(tmp_file_str.c_str());

    if (kml_file.is_open() && tmp_file.is_open())
        {
            DLOG(INFO) << "KML printer writing on " << filename.c_str();
            // Set iostream numeric format and precision
            kml_file.setf(kml_file.std::ofstream::fixed, kml_file.std::ofstream::floatfield);
            kml_file << std::setprecision(14);

            tmp_file.setf(tmp_file.std::ofstream::fixed, tmp_file.std::ofstream::floatfield);
            tmp_file << std::setprecision(14);

            kml_file << R"(<?xml version="1.0" encoding="UTF-8"?>)" << '\n'
                     << R"(<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:gx="http://www.google.com/kml/ext/2.2">)" << '\n'
                     << indent << "<Document>\n"
                     << indent << indent << "<name>GNSS Track</name>\n"
                     << indent << indent << "<description><![CDATA[\n"
                     << indent << indent << indent << "<table>\n"
                     << indent << indent << indent << indent << "<tr><td>GNSS-SDR Receiver position log file created at " << pt << "</td></tr>\n"
                     << indent << indent << indent << indent << "<tr><td>https://gnss-sdr.org/</td></tr>\n"
                     << indent << indent << indent << "</table>\n"
                     << indent << indent << "]]></description>\n"
                     << indent << indent << "<!-- Normal track style -->\n"
                     << indent << indent << "<Style id=\"track_n\">\n"
                     << indent << indent << indent << "<IconStyle>\n"
                     << indent << indent << indent << indent << "<color>ff00ffff</color>\n"
                     << indent << indent << indent << indent << "<scale>0.3</scale>\n"
                     << indent << indent << indent << indent << "<Icon>\n"
                     << indent << indent << indent << indent << indent << "<href>http://maps.google.com/mapfiles/kml/shapes/shaded_dot.png</href>\n"
                     << indent << indent << indent << indent << "</Icon>\n"
                     << indent << indent << indent << "</IconStyle>\n"
                     << indent << indent << indent << "<LabelStyle>\n"
                     << indent << indent << indent << indent << "<scale>0</scale>\n"
                     << indent << indent << indent << "</LabelStyle>\n"
                     << indent << indent << "</Style>\n"
                     << indent << indent << "<!-- Highlighted track style -->\n"
                     << indent << indent << "<Style id=\"track_h\">\n"
                     << indent << indent << indent << "<IconStyle>\n"
                     << indent << indent << indent << indent << "<color>ff00ffff</color>\n"
                     << indent << indent << indent << indent << "<scale>1</scale>\n"
                     << indent << indent << indent << indent << "<Icon>\n"
                     << indent << indent << indent << indent << indent << "<href>http://maps.google.com/mapfiles/kml/shapes/shaded_dot.png</href>\n"
                     << indent << indent << indent << indent << "</Icon>\n"
                     << indent << indent << indent << "</IconStyle>\n"
                     << indent << indent << "</Style>\n"
                     << indent << indent << "<StyleMap id=\"track\">\n"
                     << indent << indent << indent << "<Pair>\n"
                     << indent << indent << indent << indent << "<key>normal</key>\n"
                     << indent << indent << indent << indent << "<styleUrl>#track_n</styleUrl>\n"
                     << indent << indent << indent << "</Pair>\n"
                     << indent << indent << indent << "<Pair>\n"
                     << indent << indent << indent << indent << "<key>highlight</key>\n"
                     << indent << indent << indent << indent << "<styleUrl>#track_h</styleUrl>\n"
                     << indent << indent << indent << "</Pair>\n"
                     << indent << indent << "</StyleMap>\n"
                     << indent << indent << "<Style id=\"yellowLineGreenPoly\">\n"
                     << indent << indent << indent << "<LineStyle>\n"
                     << indent << indent << indent << indent << "<color>7f00ffff</color>\n"
                     << indent << indent << indent << indent << "<width>1</width>\n"
                     << indent << indent << indent << "</LineStyle>\n"
                     << indent << indent << indent << "<PolyStyle>\n"
                     << indent << indent << indent << indent << "<color>7f00ff00</color>\n"
                     << indent << indent << indent << "</PolyStyle>\n"
                     << indent << indent << "</Style>\n"
                     << indent << indent << "<Folder>\n"
                     << indent << indent << indent << "<name>Points</name>\n";

            return true;
        }
    std::cout << "File " << kml_filename << " cannot be saved. Wrong permissions?\n";
    return false;
}


bool Kml_Printer::print_position(const Pvt_Solution* const position, bool print_average_values)
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

    if (kml_file.is_open() && tmp_file.is_open())
        {
            point_id++;
            kml_file << indent << indent << indent << "<Placemark>\n"
                     << indent << indent << indent << indent << "<name>" << point_id << "</name>\n"
                     << indent << indent << indent << indent << "<snippet/>\n"
                     << indent << indent << indent << indent << "<description><![CDATA[\n"
                     << indent << indent << indent << indent << indent << "<table>\n"
                     << indent << indent << indent << indent << indent << indent << "<tr><td>Time:</td><td>" << utc_time << "</td></tr>\n"
                     << indent << indent << indent << indent << indent << indent << "<tr><td>Longitude:</td><td>" << longitude << "</td><td>deg</td></tr>\n"
                     << indent << indent << indent << indent << indent << indent << "<tr><td>Latitude:</td><td>" << latitude << "</td><td>deg</td></tr>\n"
                     << indent << indent << indent << indent << indent << indent << "<tr><td>Altitude:</td><td>" << height << "</td><td>m</td></tr>\n"
                     << indent << indent << indent << indent << indent << indent << "<tr><td>Speed:</td><td>" << speed_over_ground << "</td><td>m/s</td></tr>\n"
                     << indent << indent << indent << indent << indent << indent << "<tr><td>Course:</td><td>" << course_over_ground << "</td><td>deg</td></tr>\n"
                     << indent << indent << indent << indent << indent << indent << "<tr><td>HDOP:</td><td>" << hdop << "</td></tr>\n"
                     << indent << indent << indent << indent << indent << indent << "<tr><td>VDOP:</td><td>" << vdop << "</td></tr>\n"
                     << indent << indent << indent << indent << indent << indent << "<tr><td>PDOP:</td><td>" << pdop << "</td></tr>\n"
                     << indent << indent << indent << indent << indent << "</table>\n"
                     << indent << indent << indent << indent << "]]></description>\n"
                     << indent << indent << indent << indent << "<TimeStamp>\n"
                     << indent << indent << indent << indent << indent << "<when>" << utc_time << "</when>\n"
                     << indent << indent << indent << indent << "</TimeStamp>\n"
                     << indent << indent << indent << indent << "<styleUrl>#track</styleUrl>\n"
                     << indent << indent << indent << indent << "<Point>\n"
                     << indent << indent << indent << indent << indent << "<altitudeMode>absolute</altitudeMode>\n"
                     << indent << indent << indent << indent << indent << "<coordinates>" << longitude << "," << latitude << "," << height << "</coordinates>\n"
                     << indent << indent << indent << indent << "</Point>\n"
                     << indent << indent << indent << "</Placemark>\n";

            tmp_file << indent << indent << indent << indent << indent
                     << longitude << "," << latitude << "," << height << '\n';

            return true;
        }
    return false;
}


bool Kml_Printer::close_file()
{
    if (kml_file.is_open() && tmp_file.is_open())
        {
            tmp_file.close();

            kml_file << indent << indent << "</Folder>"
                     << indent << indent << "<Placemark>\n"
                     << indent << indent << indent << "<name>Path</name>\n"
                     << indent << indent << indent << "<styleUrl>#yellowLineGreenPoly</styleUrl>\n"
                     << indent << indent << indent << "<LineString>\n"
                     << indent << indent << indent << indent << "<extrude>0</extrude>\n"
                     << indent << indent << indent << indent << "<tessellate>1</tessellate>\n"
                     << indent << indent << indent << indent << "<altitudeMode>absolute</altitudeMode>\n"
                     << indent << indent << indent << indent << "<coordinates>\n";

            // Copy the contents of tmp_file into kml_file
            std::ifstream src(tmp_file_str, std::ios::binary);
            kml_file << src.rdbuf();

            kml_file << indent << indent << indent << indent << "</coordinates>\n"
                     << indent << indent << indent << "</LineString>\n"
                     << indent << indent << "</Placemark>\n"
                     << indent << "</Document>\n"
                     << "</kml>";

            kml_file.close();

            return true;
        }
    return false;
}


Kml_Printer::~Kml_Printer()
{
    DLOG(INFO) << "KML printer destructor called.";
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
            if (!fs::remove(fs::path(kml_filename), ec))
                {
                    LOG(INFO) << "Error deleting temporary KML file";
                }
        }
}
