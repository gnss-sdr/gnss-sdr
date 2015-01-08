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
#include <ctime>
#include <glog/logging.h>

using google::LogMessage;

bool Kml_Printer::set_headers(std::string filename)
{
    time_t rawtime;
    struct tm * timeinfo;
    time ( &rawtime );
    timeinfo = localtime ( &rawtime );
    kml_file.open(filename.c_str());
    if (kml_file.is_open())
        {
            DLOG(INFO) << "KML printer writing on " << filename.c_str();
            // Set iostream numeric format and precision
            kml_file.setf(kml_file.fixed, kml_file.floatfield);
            kml_file << std::setprecision(14);
            kml_file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << std::endl
                    << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">" << std::endl
                    << "	<Document>" << std::endl
                    << "	<name>GNSS Track</name>" << std::endl
                    << "	<description>GNSS-SDR Receiver position log file created at " << asctime (timeinfo)
                    << "	</description>" << std::endl
                    << "<Style id=\"yellowLineGreenPoly\">" << std::endl
                    << " <LineStyle>" << std::endl
                    << " 	<color>7f00ffff</color>" << std::endl
                    << "		<width>1</width>" << std::endl
                    << "	</LineStyle>" << std::endl
                    << "<PolyStyle>" << std::endl
                    << "	<color>7f00ff00</color>" << std::endl
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



bool Kml_Printer::print_position(const std::shared_ptr<gps_l1_ca_ls_pvt>& position, bool print_average_values)
{
    double latitude;
    double longitude;
    double height;

    std::shared_ptr<gps_l1_ca_ls_pvt> position_ = position;

    if (print_average_values == false)
        {
            latitude = position_->d_latitude_d;
            longitude = position_->d_longitude_d;
            height = position_->d_height_m;
        }
    else
        {
            latitude = position_->d_avg_latitude_d;
            longitude = position_->d_avg_longitude_d;
            height = position_->d_avg_height_m;
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

//ToDo: make the class ls_pvt generic and heritate the particular gps/gal/glo ls_pvt in order to
// reuse kml_printer functions
bool Kml_Printer::print_position_galileo(const std::shared_ptr<galileo_e1_ls_pvt>& position, bool print_average_values)
{
    double latitude;
    double longitude;
    double height;
    std::shared_ptr<galileo_e1_ls_pvt> position_ = position;
    if (print_average_values == false)
        {
            latitude = position_->d_latitude_d;
            longitude = position_->d_longitude_d;
            height = position_->d_height_m;
        }
    else
        {
            latitude = position_->d_avg_latitude_d;
            longitude = position_->d_avg_longitude_d;
            height = position_->d_avg_height_m;
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

bool Kml_Printer::print_position_hybrid(const std::shared_ptr<hybrid_ls_pvt>& position, bool print_average_values)
{
    double latitude;
    double longitude;
    double height;
    if (print_average_values == false)
        {
            latitude = position->d_latitude_d;
            longitude = position->d_longitude_d;
            height = position->d_height_m;
        }
    else
        {
            latitude = position->d_avg_latitude_d;
            longitude = position->d_avg_longitude_d;
            height = position->d_avg_height_m;
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



Kml_Printer::Kml_Printer () {}



Kml_Printer::~Kml_Printer ()
{
    close_file();
}

