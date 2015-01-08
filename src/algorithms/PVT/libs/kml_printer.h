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

#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include "gps_l1_ca_ls_pvt.h"
#include "galileo_e1_ls_pvt.h"
#include "hybrid_ls_pvt.h"

/*!
 * \brief Prints PVT information to OGC KML format file (can be viewed with Google Earth)
 *
 * See http://www.opengeospatial.org/standards/kml
 */
class Kml_Printer
{
private:
    std::ofstream kml_file;
public:
    bool set_headers(std::string filename);
    bool print_position(const std::shared_ptr<gps_l1_ca_ls_pvt>& position, bool print_average_values);
    bool print_position_galileo(const std::shared_ptr<galileo_e1_ls_pvt>& position, bool print_average_values);
    bool print_position_hybrid(const std::shared_ptr<hybrid_ls_pvt>& position, bool print_average_values);
    bool close_file();
    Kml_Printer();
    ~Kml_Printer();
};

#endif
