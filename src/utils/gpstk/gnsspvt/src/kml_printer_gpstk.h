/*!
 * \file kml_printer.h
 * \brief Interface of a class that prints PVT information to a kml file
 * for GPSTK data structures
 * \author Javier Arribas, 2012. jarribas(at)cttc.es
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


#ifndef GNSS_SDR_KML_PRINTER_H_
#define GNSS_SDR_KML_PRINTER_H_

#include <iostream>
#include <fstream>
#include "gpstk/Position.hpp"


/*!
 * \brief Prints PVT information to OGC KML format file (can be viewed with Google Earth)
 *
 * See http://www.opengeospatial.org/standards/kml
 */
class Kml_Printer_gpstk
{
private:
    std::ofstream kml_file;

public:
    bool set_headers(std::string filename);
    bool print_position(gpstk::Position position);
    bool close_file();
    Kml_Printer_gpstk();
    ~Kml_Printer_gpstk();
};

#endif
