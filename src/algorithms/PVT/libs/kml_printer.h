/*!
 * \file kml_printer.h
 * \brief Interface of a class that prints PVT information to a kml file
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


#ifndef GNSS_SDR_KML_PRINTER_H_
#define GNSS_SDR_KML_PRINTER_H_

#include "pvt_solution.h"
#include <fstream>
#include <memory>
#include <string>


/*!
 * \brief Prints PVT information to OGC KML format file (can be viewed with Google Earth)
 *
 * See http://www.opengeospatial.org/standards/kml
 */
class Kml_Printer
{
private:
    std::ofstream kml_file;
    bool positions_printed;
    std::string kml_filename;

public:
    Kml_Printer();
    ~Kml_Printer();
    bool set_headers(std::string filename, bool time_tag_name = true);
    bool print_position(const std::shared_ptr<Pvt_Solution>& position, bool print_average_values);
    bool close_file();
};

#endif
