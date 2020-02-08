/*!
 * \file geojson_printer.h
 * \brief Interface of a class that prints PVT solutions in GeoJSON format
 * \author Carles Fernandez-Prades, 2015. cfernandez(at)cttc.es
 *
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
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GEOJSON_PRINTER_H
#define GNSS_SDR_GEOJSON_PRINTER_H


#include <fstream>
#include <memory>
#include <string>

class Pvt_Solution;

/*!
 * \brief Prints PVT solutions in GeoJSON format file
 *
 * See https://tools.ietf.org/html/rfc7946
 */
class GeoJSON_Printer
{
public:
    explicit GeoJSON_Printer(const std::string& base_path = ".");
    ~GeoJSON_Printer();
    bool set_headers(const std::string& filename, bool time_tag_name = true);
    bool print_position(const std::shared_ptr<Pvt_Solution>& position, bool print_average_values);
    bool close_file();

private:
    std::ofstream geojson_file;
    bool first_pos;
    std::string filename_;
    std::string geojson_base_path;
};

#endif
