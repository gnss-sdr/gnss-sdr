/*!
 * \file kml_printer.cc
 * \brief Implementation of a class that prints PVT information to a kml file
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "kml_printer.h"
#include <sstream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <glog/logging.h>

using google::LogMessage;

bool Kml_Printer::set_headers(std::string filename,  bool time_tag_name)
{
    boost::posix_time::ptime pt = boost::posix_time::second_clock::local_time();
    tm timeinfo = boost::posix_time::to_tm(pt);

    if (time_tag_name)
        {
            std::stringstream strm0;
            const int year = timeinfo.tm_year - 100;
            strm0 << year;
            const int month = timeinfo.tm_mon + 1;
            if(month < 10)
                {
                    strm0 << "0";
                }
            strm0 << month;
            const int day = timeinfo.tm_mday;
            if(day < 10)
                {
                    strm0 << "0";
                }
            strm0 << day << "_";
            const int hour = timeinfo.tm_hour;
            if(hour < 10)
                {
                    strm0 << "0";
                }
            strm0 << hour;
            const int min = timeinfo.tm_min;
            if(min < 10)
                {
                    strm0 << "0";
                }
            strm0 << min;
            const int sec = timeinfo.tm_sec;
            if(sec < 10)
                {
                    strm0 << "0";
                }
            strm0 << sec;

            kml_filename = filename + "_" +  strm0.str() + ".kml";
        }
    else
        {
            kml_filename = filename + ".kml";
        }
    kml_file.open(kml_filename.c_str());

    if (kml_file.is_open())
        {
            DLOG(INFO) << "KML printer writing on " << filename.c_str();
            // Set iostream numeric format and precision
            kml_file.setf(kml_file.fixed, kml_file.floatfield);
            kml_file << std::setprecision(14);
            kml_file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << std::endl
                    << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">" << std::endl
                    << "    <Document>" << std::endl
                    << "    <name>GNSS Track</name>" << std::endl
                    << "    <description>GNSS-SDR Receiver position log file created at " << pt
                    << "    </description>" << std::endl
                    << "<Style id=\"yellowLineGreenPoly\">" << std::endl
                    << " <LineStyle>" << std::endl
                    << "     <color>7f00ffff</color>" << std::endl
                    << "        <width>1</width>" << std::endl
                    << "    </LineStyle>" << std::endl
                    << "<PolyStyle>" << std::endl
                    << "    <color>7f00ff00</color>" << std::endl
                    << "</PolyStyle>" << std::endl
                    << "</Style>" << std::endl
                    << "<Placemark>" << std::endl
                    << "<name>GNSS-SDR PVT</name>" << std::endl
                    << "<description>GNSS-SDR position log</description>" << std::endl
                    << "<styleUrl>#yellowLineGreenPoly</styleUrl>" << std::endl
                    << "<LineString>" << std::endl
                    << "<extrude>0</extrude>" << std::endl
                    << "<tessellate>1</tessellate>" << std::endl
                    << "<altitudeMode>absolute</altitudeMode>" << std::endl
                    << "<coordinates>" << std::endl;
            return true;
        }
    else
        {
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
            kml_file << longitude << "," << latitude << "," << height << std::endl;
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

            kml_file << "</coordinates>" << std::endl
                     << "</LineString>" << std::endl
                     << "</Placemark>" << std::endl
                     << "</Document>" << std::endl
                     << "</kml>";
            kml_file.close();
            return true;
        }
    else
        {
            return false;
        }
}



Kml_Printer::Kml_Printer ()
{
    positions_printed = false;
}



Kml_Printer::~Kml_Printer ()
{
    close_file();
    if(!positions_printed)
        {
            if(remove(kml_filename.c_str()) != 0) LOG(INFO) << "Error deleting temporary KML file";
        }
}

