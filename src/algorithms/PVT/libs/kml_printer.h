/*!
 * \file kml_printer.h
 * \brief Interface of a class that prints PVT information to a kml file
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *         Álvaro Cebrián Juan, 2018. acebrianjuan(at)gmail.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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


#ifndef GNSS_SDR_KML_PRINTER_H_
#define GNSS_SDR_KML_PRINTER_H_

#include <fstream>  // for ofstream
#include <memory>   // for shared_ptr


class Rtklib_Solver;

/*!
 * \brief Prints PVT information to OGC KML format file (can be viewed with Google Earth)
 *
 * See http://www.opengeospatial.org/standards/kml
 */
class Kml_Printer
{
public:
    Kml_Printer(const std::string& base_path = std::string("."));
    ~Kml_Printer();
    bool set_headers(const std::string& filename, bool time_tag_name = true);
    bool print_position(const std::shared_ptr<Rtklib_Solver>& position, bool print_average_values);
    bool close_file();

private:
    std::ofstream kml_file;
    std::ofstream tmp_file;
    bool positions_printed;
    std::string kml_filename;
    std::string kml_base_path;
    std::string tmp_file_str;
    unsigned int point_id;
    std::string indent;
};

#endif
