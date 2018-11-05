/*!
 * \file gpx_printer.h
 * \brief Interface of a class that prints PVT information to a gpx file
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


#ifndef GNSS_SDR_GPX_PRINTER_H_
#define GNSS_SDR_GPX_PRINTER_H_

#include "pvt_solution.h"
#include "rtklib_solver.h"
#include <fstream>
#include <memory>
#include <string>


/*!
 * \brief Prints PVT information to GPX format file
 *
 * See http://www.topografix.com/gpx.asp
 */
class Gpx_Printer
{
private:
    std::ofstream gpx_file;
    bool positions_printed;
    std::string gpx_filename;
    std::string indent;
    std::string gpx_base_path;

public:
    Gpx_Printer(const std::string& base_path = ".");
    ~Gpx_Printer();
    bool set_headers(std::string filename, bool time_tag_name = true);
    bool print_position(const std::shared_ptr<rtklib_solver>& position, bool print_average_values);
    bool close_file();
};

#endif
