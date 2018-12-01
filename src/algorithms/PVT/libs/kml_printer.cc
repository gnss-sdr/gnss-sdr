/*!
 * \file kml_printer.cc
 * \brief Implementation of a class that prints PVT information to a kml file
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *         Álvaro Cebrián Juan, 2018. acebrianjuan(at)gmail.com
 *
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

#include "kml_printer.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem/operations.hpp>   // for create_directories, exists
#include <boost/filesystem/path.hpp>         // for path, operator<<
#include <boost/filesystem/path_traits.hpp>  // for filesystem
#include <glog/logging.h>
#include <sstream>

using google::LogMessage;


Kml_Printer::Kml_Printer(const std::string& base_path)
{
    positions_printed = false;
    indent = "  ";
    kml_base_path = base_path;
    boost::filesystem::path full_path(boost::filesystem::current_path());
    const boost::filesystem::path p(kml_base_path);
    if (!boost::filesystem::exists(p))
        {
            std::string new_folder;
            for (auto& folder : boost::filesystem::path(kml_base_path))
                {
                    new_folder += folder.string();
                    boost::system::error_code ec;
                    if (!boost::filesystem::exists(new_folder))
                        {
                            if (!boost::filesystem::create_directory(new_folder, ec))
                                {
                                    std::cout << "Could not create the " << new_folder << " folder." << std::endl;
                                    kml_base_path = full_path.string();
                                }
                        }
                    new_folder += boost::filesystem::path::preferred_separator;
                }
        }
    else
        {
            kml_base_path = p.string();
        }
    if (kml_base_path.compare(".") != 0)
        {
            std::cout << "KML files will be stored at " << kml_base_path << std::endl;
        }

    kml_base_path = kml_base_path + boost::filesystem::path::preferred_separator;

    boost::filesystem::path tmp_base_path = boost::filesystem::temp_directory_path();
    boost::filesystem::path tmp_filename = boost::filesystem::unique_path();
    boost::filesystem::path tmp_file = tmp_base_path / tmp_filename;

    tmp_file_str = tmp_file.string();

    point_id = 0;
}


bool Kml_Printer::set_headers(std::string filename, bool time_tag_name)
{
    boost::posix_time::ptime pt = boost::posix_time::second_clock::local_time();
    tm timeinfo = boost::posix_time::to_tm(pt);

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
            kml_file.setf(kml_file.fixed, kml_file.floatfield);
            kml_file << std::setprecision(14);

            tmp_file.setf(tmp_file.fixed, tmp_file.floatfield);
            tmp_file << std::setprecision(14);

            kml_file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << std::endl
                     << "<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\">" << std::endl
                     << indent << "<Document>" << std::endl
                     << indent << indent << "<name>GNSS Track</name>" << std::endl
                     << indent << indent << "<description><![CDATA[" << std::endl
                     << indent << indent << indent << "<table>" << std::endl
                     << indent << indent << indent << indent << "<tr><td>GNSS-SDR Receiver position log file created at " << pt << "</td></tr>" << std::endl
                     << indent << indent << indent << indent << "<tr><td>https://gnss-sdr.org/</td></tr>" << std::endl
                     << indent << indent << indent << "</table>" << std::endl
                     << indent << indent << "]]></description>" << std::endl
                     << indent << indent << "<!-- Normal track style -->" << std::endl
                     << indent << indent << "<Style id=\"track_n\">" << std::endl
                     << indent << indent << indent << "<IconStyle>" << std::endl
                     << indent << indent << indent << indent << "<color>ff00ffff</color>" << std::endl
                     << indent << indent << indent << indent << "<scale>0.3</scale>" << std::endl
                     << indent << indent << indent << indent << "<Icon>" << std::endl
                     << indent << indent << indent << indent << indent << "<href>http://maps.google.com/mapfiles/kml/shapes/shaded_dot.png</href>" << std::endl
                     << indent << indent << indent << indent << "</Icon>" << std::endl
                     << indent << indent << indent << "</IconStyle>" << std::endl
                     << indent << indent << indent << "<LabelStyle>" << std::endl
                     << indent << indent << indent << indent << "<scale>0</scale>" << std::endl
                     << indent << indent << indent << "</LabelStyle>" << std::endl
                     << indent << indent << "</Style>" << std::endl
                     << indent << indent << "<!-- Highlighted track style -->" << std::endl
                     << indent << indent << "<Style id=\"track_h\">" << std::endl
                     << indent << indent << indent << "<IconStyle>" << std::endl
                     << indent << indent << indent << indent << "<color>ff00ffff</color>" << std::endl
                     << indent << indent << indent << indent << "<scale>1</scale>" << std::endl
                     << indent << indent << indent << indent << "<Icon>" << std::endl
                     << indent << indent << indent << indent << indent << "<href>http://maps.google.com/mapfiles/kml/shapes/shaded_dot.png</href>" << std::endl
                     << indent << indent << indent << indent << "</Icon>" << std::endl
                     << indent << indent << indent << "</IconStyle>" << std::endl
                     << indent << indent << "</Style>" << std::endl
                     << indent << indent << "<StyleMap id=\"track\">" << std::endl
                     << indent << indent << indent << "<Pair>" << std::endl
                     << indent << indent << indent << indent << "<key>normal</key>" << std::endl
                     << indent << indent << indent << indent << "<styleUrl>#track_n</styleUrl>" << std::endl
                     << indent << indent << indent << "</Pair>" << std::endl
                     << indent << indent << indent << "<Pair>" << std::endl
                     << indent << indent << indent << indent << "<key>highlight</key>" << std::endl
                     << indent << indent << indent << indent << "<styleUrl>#track_h</styleUrl>" << std::endl
                     << indent << indent << indent << "</Pair>" << std::endl
                     << indent << indent << "</StyleMap>" << std::endl
                     << indent << indent << "<Style id=\"yellowLineGreenPoly\">" << std::endl
                     << indent << indent << indent << "<LineStyle>" << std::endl
                     << indent << indent << indent << indent << "<color>7f00ffff</color>" << std::endl
                     << indent << indent << indent << indent << "<width>1</width>" << std::endl
                     << indent << indent << indent << "</LineStyle>" << std::endl
                     << indent << indent << indent << "<PolyStyle>" << std::endl
                     << indent << indent << indent << indent << "<color>7f00ff00</color>" << std::endl
                     << indent << indent << indent << "</PolyStyle>" << std::endl
                     << indent << indent << "</Style>" << std::endl
                     << indent << indent << "<Folder>" << std::endl
                     << indent << indent << indent << "<name>Points</name>" << std::endl;

            return true;
        }
    else
        {
            std::cout << "File " << kml_filename << " cannot be saved. Wrong permissions?" << std::endl;
            return false;
        }
}


bool Kml_Printer::print_position(const std::shared_ptr<rtklib_solver>& position, bool print_average_values)
{
    double latitude;
    double longitude;
    double height;

    positions_printed = true;

    std::shared_ptr<rtklib_solver> position_ = position;

    double speed_over_ground = position_->get_speed_over_ground();    // expressed in m/s
    double course_over_ground = position_->get_course_over_ground();  // expressed in deg

    double hdop = position_->get_hdop();
    double vdop = position_->get_vdop();
    double pdop = position_->get_pdop();
    std::string utc_time = to_iso_extended_string(position_->get_position_UTC_time());
    if (utc_time.length() < 23) utc_time += ".";
    utc_time.resize(23, '0');  // time up to ms
    utc_time.append("Z");      // UTC time zone

    if (print_average_values == false)
        {
            latitude = position_->get_latitude();
            longitude = position_->get_longitude();
            height = position_->get_height();
        }
    else
        {
            latitude = position_->get_avg_latitude();
            longitude = position_->get_avg_longitude();
            height = position_->get_avg_height();
        }

    if (kml_file.is_open() && tmp_file.is_open())
        {
            point_id++;
            kml_file << indent << indent << indent << "<Placemark>" << std::endl
                     << indent << indent << indent << indent << "<name>" << point_id << "</name>" << std::endl
                     << indent << indent << indent << indent << "<snippet/>" << std::endl
                     << indent << indent << indent << indent << "<description><![CDATA[" << std::endl
                     << indent << indent << indent << indent << indent << "<table>" << std::endl
                     << indent << indent << indent << indent << indent << indent << "<tr><td>Time:</td><td>" << utc_time << "</td></tr>" << std::endl
                     << indent << indent << indent << indent << indent << indent << "<tr><td>Longitude:</td><td>" << longitude << "</td><td>deg</td></tr>" << std::endl
                     << indent << indent << indent << indent << indent << indent << "<tr><td>Latitude:</td><td>" << latitude << "</td><td>deg</td></tr>" << std::endl
                     << indent << indent << indent << indent << indent << indent << "<tr><td>Altitude:</td><td>" << height << "</td><td>m</td></tr>" << std::endl
                     << indent << indent << indent << indent << indent << indent << "<tr><td>Speed:</td><td>" << speed_over_ground << "</td><td>m/s</td></tr>" << std::endl
                     << indent << indent << indent << indent << indent << indent << "<tr><td>Course:</td><td>" << course_over_ground << "</td><td>deg</td></tr>" << std::endl
                     << indent << indent << indent << indent << indent << indent << "<tr><td>HDOP:</td><td>" << hdop << "</td></tr>" << std::endl
                     << indent << indent << indent << indent << indent << indent << "<tr><td>VDOP:</td><td>" << vdop << "</td></tr>" << std::endl
                     << indent << indent << indent << indent << indent << indent << "<tr><td>PDOP:</td><td>" << pdop << "</td></tr>" << std::endl
                     << indent << indent << indent << indent << indent << "</table>" << std::endl
                     << indent << indent << indent << indent << "]]></description>" << std::endl
                     << indent << indent << indent << indent << "<TimeStamp>" << std::endl
                     << indent << indent << indent << indent << indent << "<when>" << utc_time << "</when>" << std::endl
                     << indent << indent << indent << indent << "</TimeStamp>" << std::endl
                     << indent << indent << indent << indent << "<styleUrl>#track</styleUrl>" << std::endl
                     << indent << indent << indent << indent << "<Point>" << std::endl
                     << indent << indent << indent << indent << indent << "<altitudeMode>absolute</altitudeMode>" << std::endl
                     << indent << indent << indent << indent << indent << "<coordinates>" << longitude << "," << latitude << "," << height << "</coordinates>" << std::endl
                     << indent << indent << indent << indent << "</Point>" << std::endl
                     << indent << indent << indent << "</Placemark>" << std::endl;

            tmp_file << indent << indent << indent << indent << indent
                     << longitude << "," << latitude << "," << height << std::endl;

            return true;
        }
    else
        {
            return false;
        }
}


bool Kml_Printer::close_file()
{
    if (kml_file.is_open() && tmp_file.is_open())
        {
            tmp_file.close();

            kml_file << indent << indent << "</Folder>"
                     << indent << indent << "<Placemark>" << std::endl
                     << indent << indent << indent << "<name>Path</name>" << std::endl
                     << indent << indent << indent << "<styleUrl>#yellowLineGreenPoly</styleUrl>" << std::endl
                     << indent << indent << indent << "<LineString>" << std::endl
                     << indent << indent << indent << indent << "<extrude>0</extrude>" << std::endl
                     << indent << indent << indent << indent << "<tessellate>1</tessellate>" << std::endl
                     << indent << indent << indent << indent << "<altitudeMode>absolute</altitudeMode>" << std::endl
                     << indent << indent << indent << indent << "<coordinates>" << std::endl;

            // Copy the contents of tmp_file into kml_file
            std::ifstream src(tmp_file_str, std::ios::binary);
            kml_file << src.rdbuf();

            kml_file << indent << indent << indent << indent << "</coordinates>" << std::endl
                     << indent << indent << indent << "</LineString>" << std::endl
                     << indent << indent << "</Placemark>" << std::endl
                     << indent << "</Document>" << std::endl
                     << "</kml>";

            kml_file.close();

            return true;
        }
    else
        {
            return false;
        }
}


Kml_Printer::~Kml_Printer()
{
    close_file();
    if (!positions_printed)
        {
            if (remove(kml_filename.c_str()) != 0) LOG(INFO) << "Error deleting temporary KML file";
        }
}
