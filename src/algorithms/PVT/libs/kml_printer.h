/*!
 * \file kml_printer.h
 * \brief Prints PVT information to a GoogleEarth kml file
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


#ifndef KML_PRINTER_H_
#define	KML_PRINTER_H_

#include <iostream>
#include <fstream>

#include "gps_l1_ca_ls_pvt.h"

class kml_printer
{
private:

	std::ofstream kml_file;

public:

	bool set_headers(std::string filename);

	bool print_position(gps_l1_ca_ls_pvt* position,bool print_average_values);

	bool close_file();

	kml_printer();
	~kml_printer();
};

#endif
