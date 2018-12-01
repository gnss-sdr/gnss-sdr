/*!
 * \file pvt_conf.cc
 * \brief Class that contains all the configuration parameters for a PVT block
 * \author Carles Fernandez, 2018. cfernandez(at)cttc.es
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

#include "pvt_conf.h"

Pvt_Conf::Pvt_Conf()
{
    type_of_receiver = 0U;
    output_rate_ms = 0;
    display_rate_ms = 0;

    rinex_version = 0;
    rinexobs_rate_ms = 0;
    rinexnav_rate_ms = 0;

    dump = false;
    dump_mat = true;

    flag_nmea_tty_port = false;

    flag_rtcm_server = false;
    flag_rtcm_tty_port = false;
    rtcm_tcp_port = 0U;
    rtcm_station_id = 0U;

    output_enabled = true;
    rinex_output_enabled = true;
    gpx_output_enabled = true;
    geojson_output_enabled = true;
    nmea_output_file_enabled = true;
    kml_output_enabled = true;
    xml_output_enabled = true;
    rtcm_output_file_enabled = true;

    output_path = std::string(".");
    rinex_output_path = std::string(".");
    gpx_output_path = std::string(".");
    geojson_output_path = std::string(".");
    nmea_output_file_path = std::string(".");
    kml_output_path = std::string(".");
    xml_output_path = std::string(".");
    rtcm_output_file_path = std::string(".");
}
