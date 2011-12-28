/*!
 * \file kml_printer.cc
 * \brief Implementation of a class that prints PVT information to a kml file
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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
#include <glog/log_severity.h>
#include <glog/logging.h>
#include <time.h>

bool kml_printer::set_headers(std::string filename)
{
    time_t rawtime;
    struct tm * timeinfo;

    time ( &rawtime );
    timeinfo = localtime ( &rawtime );

    kml_file.open(filename.c_str());
    if (kml_file.is_open())
        {
            DLOG(INFO)<<"KML printer writting on "<<filename.c_str();
            kml_file<<"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n"
                    <<"<kml xmlns=\"http://www.opengis.net/kml/2.2\">\r\n"
                    <<"	<Document>\r\n"
                    <<"	<name>GNSS Track</name>\r\n"
                    <<"	<description>GNSS-SDR Receiver position log file created at "<<asctime (timeinfo)
                    <<"	</description>\r\n"
                    <<"<Style id=\"yellowLineGreenPoly\">\r\n"
                    <<" <LineStyle>\r\n"
                    <<" 	<color>7f00ffff</color>\r\n"
                    <<"		<width>1</width>\r\n"
                    <<"	</LineStyle>\r\n"
                    <<"<PolyStyle>\r\n"
                    <<"	<color>7f00ff00</color>\r\n"
                    <<"</PolyStyle>\r\n"
                    <<"</Style>\r\n"
                    <<"<Placemark>\r\n"
                    <<"<name>GNSS-SDR PVT</name>\r\n"
                    <<"<description>GNSS-SDR position log</description>\r\n"
                    <<"<styleUrl>#yellowLineGreenPoly</styleUrl>\r\n"
                    <<"<LineString>\r\n"
                    <<"<extrude>0</extrude>\r\n"
                    <<"<tessellate>1</tessellate>\r\n"
                    <<"<altitudeMode>absolute</altitudeMode>\r\n"
                    <<"<coordinates>\r\n";
            return true;
        }else{
                return false;
        }
}


bool kml_printer::print_position(gps_l1_ca_ls_pvt* position,bool print_average_values)
{
    double latitude;
    double longitude;
    double height;
    if (print_average_values==false)
        {
            latitude=position->d_latitude_d;
            longitude=position->d_longitude_d;
            height=position->d_height_m;
        }else{
                latitude=position->d_avg_latitude_d;
                longitude=position->d_avg_longitude_d;
                height=position->d_avg_height_m;
        }
    if (kml_file.is_open())
        {
            kml_file<<longitude<<","<<latitude<<","<<height<<"\r\n";
            return true;
        }else
            {
                return false;
            }
}

bool kml_printer::close_file()
{
    if (kml_file.is_open())
        {
            kml_file<<"</coordinates>\r\n"
                    <<"</LineString>\r\n"
                    <<"</Placemark>\r\n"
                    <<"</Document>\r\n"
                    <<"</kml>";
            kml_file.close();
            return true;
        }else{
                return false;
        }
}

kml_printer::kml_printer () {}

kml_printer::~kml_printer () {}
