/*!
 * \file kml_printer.cc
 * \brief Implementation of a class that prints PVT information to a kml file
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
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

    if (kml_file.is_open())
        {
            DLOG(INFO) << "KML printer writing on " << filename.c_str();
            // Set iostream numeric format and precision
            kml_file.setf(kml_file.fixed, kml_file.floatfield);
            kml_file << std::setprecision(14);
            kml_file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << std::endl
                     << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">" << std::endl
                     << indent << "<Document>" << std::endl
                     << indent << indent << "<name>GNSS Track</name>" << std::endl
                     << indent << indent << "<description>GNSS-SDR Receiver position log file created at " << pt << "</description>" << std::endl
                     << indent << indent << "<Style id=\"yellowLineGreenPoly\">" << std::endl
                     << indent << indent << indent << "<LineStyle>" << std::endl
                     << indent << indent << indent << indent << "<color>7f00ffff</color>" << std::endl
                     << indent << indent << indent << indent << "<width>1</width>" << std::endl
                     << indent << indent << indent << "</LineStyle>" << std::endl
                     << indent << indent << indent << "<PolyStyle>" << std::endl
                     << indent << indent << indent << indent << "<color>7f00ff00</color>" << std::endl
                     << indent << indent << indent << "</PolyStyle>" << std::endl
                     << indent << indent << "</Style>" << std::endl
                     << indent << indent << "<Placemark>" << std::endl
                     << indent << indent << indent << "<name>GNSS-SDR PVT</name>" << std::endl
                     << indent << indent << indent << "<description>GNSS-SDR position log</description>" << std::endl
                     << indent << indent << indent << "<styleUrl>#yellowLineGreenPoly</styleUrl>" << std::endl
                     << indent << indent << indent << "<LineString>" << std::endl
                     << indent << indent << indent << indent << "<extrude>0</extrude>" << std::endl
                     << indent << indent << indent << indent << "<tessellate>1</tessellate>" << std::endl
                     << indent << indent << indent << indent << "<altitudeMode>absolute</altitudeMode>" << std::endl
                     << indent << indent << indent << indent << "<coordinates>" << std::endl;
            return true;
        }
    else
        {
            std::cout << "File " << kml_filename << " cannot be saved. Wrong permissions?" << std::endl;
            return false;
        }
}


bool Kml_Printer::print_position(const std::shared_ptr<Pvt_Solution>& position, bool print_average_values)
{
    double latitude;
    double longitude;
    double height;

    positions_printed = true;

    std::shared_ptr<Pvt_Solution> position_ = position;

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

    if (kml_file.is_open())
        {
            kml_file << indent << indent << indent << indent << indent
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
    if (kml_file.is_open())
        {
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
