/*!
 * \file kml_printer.cc
 * \brief Implementation of a class that prints PVT information to a kml file
 * for GPSTK data structures
 * \author Javier Arribas, 2012. jarribas(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
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

#include "kml_printer_gpstk.h"
#include <glog/log_severity.h>
#include <glog/logging.h>
#include <time.h>

bool Kml_Printer_gpstk::set_headers(std::string filename)
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
            kml_file.setf(kml_file.fixed,kml_file.floatfield);
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



bool Kml_Printer_gpstk::print_position(gpstk::Position position)
{
	double latitude;
	double longitude;
	double height;
	latitude = position.geodeticLatitude();
	longitude = position.getLongitude();
	if (longitude>190)
	{
		longitude=longitude-360;
	}
	height = position.getHeight();

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



bool Kml_Printer_gpstk::close_file()
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



Kml_Printer_gpstk::Kml_Printer_gpstk () {}



Kml_Printer_gpstk::~Kml_Printer_gpstk () {}

