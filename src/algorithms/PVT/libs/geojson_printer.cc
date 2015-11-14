/*!
 * \file geojson_printer.cc
 * \brief Implementation of a class that prints PVT solutions in GeoJSON format
 * \author Carles Fernandez-Prades, 2015. cfernandez(at)cttc.es
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


#include "geojson_printer.h"
#include <iostream>
#include <iomanip>

GeoJSON_Printer::GeoJSON_Printer () {}



GeoJSON_Printer::~GeoJSON_Printer ()
{
    close_file();
}


bool GeoJSON_Printer::set_headers(std::string filename)
{
    //time_t rawtime;
    //struct tm * timeinfo;
    //time ( &rawtime );
    //timeinfo = localtime ( &rawtime );
    geojson_file.open(filename.c_str());
    if (geojson_file.is_open())
        {
            DLOG(INFO) << "GeoJSON printer writing on " << filename.c_str();
            // Set iostream numeric format and precision
            geojson_file.setf(geojson_file.fixed, geojson_file.floatfield);
            geojson_file << std::setprecision(14);
            geojson_file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << std::endl

                    << "<altitudeMode>absolute</altitudeMode>" << std::endl
                    << "<coordinates>" << std::endl;
            return true;
        }
    else
        {
            return false;
        }
}



bool GeoJSON_Printer::print_position(const std::shared_ptr<Ls_Pvt>& position, bool print_average_values)
{
    double latitude;
    double longitude;
    double height;

    std::shared_ptr<Ls_Pvt> position_ = position;

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

    if (geojson_file.is_open())
        {
            geojson_file << longitude << "," << latitude << "," << height << std::endl;
            return true;
        }
    else
        {
            return false;
        }
}



bool GeoJSON_Printer::close_file()
{
    if (geojson_file.is_open())
        {
            geojson_file << "</coordinates>" << std::endl
                     << "</LineString>" << std::endl
                     << "</Placemark>" << std::endl
                     << "</Document>" << std::endl
                     << "</kml>";
            geojson_file.close();
            return true;
        }
    else
        {
            return false;
        }
}


