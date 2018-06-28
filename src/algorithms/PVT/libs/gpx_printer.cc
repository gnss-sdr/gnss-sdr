/*!
 * \file gpx_printer.cc
 * \brief Implementation of a class that prints PVT information to a gpx file
 * \author Álvaro Cebrián Juan, 2018. acebrianjuan(at)gmail.com
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


#include "gpx_printer.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <glog/logging.h>
#include <sstream>

using google::LogMessage;

bool Gpx_Printer::set_headers(std::string filename, bool time_tag_name)
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

            gpx_filename = filename + "_" + strm0.str() + ".gpx";
        }
    else
        {
            gpx_filename = filename + ".gpx";
        }
    gpx_file.open(gpx_filename.c_str());

    if (gpx_file.is_open())
        {
            DLOG(INFO) << "GPX printer writing on " << filename.c_str();
            // Set iostream numeric format and precision
            gpx_file.setf(gpx_file.fixed, gpx_file.floatfield);
            gpx_file << std::setprecision(14);
            gpx_file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << std::endl
                     << "<gpx version=\"1.1\" creator=\"GNSS-SDR\"" << std::endl
                     << "xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\"" << std::endl
                     << "xmlns=\"http://www.topografix.com/GPX/1/1\"" << std::endl
                     << "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\">" << std::endl
                     << "<trk>" << std::endl
                     << indent << "<name>Position fixes computed by GNSS-SDR v" << GNSS_SDR_VERSION << "</name>" << std::endl
                     << indent << "<desc>GNSS-SDR position log generated at " << pt << " (local time)</desc>" << std::endl
                     << indent << "<trkseg>" << std::endl;
            return true;
        }
    else
        {
            return false;
        }
}


bool Gpx_Printer::print_position(const std::shared_ptr<rtklib_solver>& position, bool print_average_values)
{
    double latitude;
    double longitude;
    double height;

    positions_printed = true;
    std::shared_ptr<rtklib_solver> position_ = position;

    double hdop = position_->get_hdop();
    double vdop = position_->get_vdop();
    double pdop = position_->get_pdop();
    std::string utc_time = to_iso_extended_string(position_->get_position_UTC_time());
    utc_time.resize(23);   // time up to ms
    utc_time.append("Z");  // UTC time zone

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

    if (gpx_file.is_open())
        {
            gpx_file << indent << indent << "<trkpt lon=\"" << longitude << "\" lat=\"" << latitude << "\"><ele>" << height << "</ele>"
                     << "<time>" << utc_time << "</time>"
                     << "<hdop>" << hdop << "</hdop><vdop>" << vdop << "</vdop><pdop>" << pdop << "</pdop></trkpt>" << std::endl;
            return true;
        }
    else
        {
            return false;
        }
}


bool Gpx_Printer::close_file()
{
    if (gpx_file.is_open())
        {
            gpx_file << indent << "</trkseg>" << std::endl
                     << "</trk>" << std::endl
                     << "</gpx>";
            gpx_file.close();
            return true;
        }
    else
        {
            return false;
        }
}


Gpx_Printer::Gpx_Printer()
{
    positions_printed = false;
    indent = "  ";
}


Gpx_Printer::~Gpx_Printer()
{
    close_file();
    if (!positions_printed)
        {
            if (remove(gpx_filename.c_str()) != 0) LOG(INFO) << "Error deleting temporary GPX file";
        }
}
