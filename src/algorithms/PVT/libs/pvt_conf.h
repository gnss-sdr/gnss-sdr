/*!
 * \file pvt_conf.h
 * \brief Class that contains all the configuration parameters for the PVT block
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

#ifndef GNSS_SDR_PVT_CONF_H_
#define GNSS_SDR_PVT_CONF_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <map>

class Pvt_Conf
{
public:
    uint32_t type_of_receiver;
    int32_t output_rate_ms;
    int32_t display_rate_ms;

    int32_t rinex_version;
    int32_t rinexobs_rate_ms;
    int32_t rinexnav_rate_ms;
    std::map<int, int> rtcm_msg_rate_ms;

    bool dump;
    std::string dump_filename;

    bool flag_nmea_tty_port;
    std::string nmea_dump_filename;
    std::string nmea_dump_devname;

    bool flag_rtcm_server;
    bool flag_rtcm_tty_port;
    uint16_t rtcm_tcp_port;
    uint16_t rtcm_station_id;
    std::string rtcm_dump_devname;

    bool output_enabled;
    bool rinex_output_enabled;
    bool gpx_output_enabled;
    bool geojson_output_enabled;
    bool nmea_output_file_enabled;
    bool kml_output_enabled;
    bool xml_output_enabled;
    bool rtcm_output_file_enabled;

    std::string output_path;
    std::string rinex_output_path;
    std::string gpx_output_path;
    std::string geojson_output_path;
    std::string nmea_output_file_path;
    std::string kml_output_path;
    std::string xml_output_path;
    std::string rtcm_output_file_path;

    Pvt_Conf();
};

#endif
