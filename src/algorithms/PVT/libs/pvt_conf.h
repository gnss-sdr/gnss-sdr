/*!
 * \file pvt_conf.h
 * \brief Class that contains all the configuration parameters for the PVT block
 * \author Carles Fernandez, 2018. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_PVT_CONF_H
#define GNSS_SDR_PVT_CONF_H

#include <cstdint>
#include <map>
#include <string>

/** \addtogroup PVT
 * \{ */
/** \addtogroup PVT_libs
 * \{ */


class Pvt_Conf
{
public:
    Pvt_Conf();

    std::map<int, int> rtcm_msg_rate_ms;

    std::string rinex_name;
    std::string dump_filename;
    std::string nmea_dump_filename;
    std::string nmea_dump_devname;
    std::string rtcm_dump_devname;
    std::string output_path;
    std::string rinex_output_path;
    std::string gpx_output_path;
    std::string geojson_output_path;
    std::string nmea_output_file_path;
    std::string kml_output_path;
    std::string xml_output_path;
    std::string rtcm_output_file_path;
    std::string udp_addresses;

    uint32_t type_of_receiver;
    int32_t output_rate_ms;
    int32_t display_rate_ms;
    int32_t kml_rate_ms;
    int32_t gpx_rate_ms;
    int32_t geojson_rate_ms;
    int32_t nmea_rate_ms;
    int32_t rinex_version;
    int32_t rinexobs_rate_ms;
    int32_t max_obs_block_rx_clock_offset_ms;
    int udp_port;

    uint16_t rtcm_tcp_port;
    uint16_t rtcm_station_id;

    bool flag_nmea_tty_port;
    bool flag_rtcm_server;
    bool flag_rtcm_tty_port;
    bool output_enabled;
    bool rinex_output_enabled;
    bool gpx_output_enabled;
    bool geojson_output_enabled;
    bool nmea_output_file_enabled;
    bool kml_output_enabled;
    bool xml_output_enabled;
    bool rtcm_output_file_enabled;
    bool monitor_enabled;
    bool protobuf_enabled;
    bool enable_rx_clock_correction;
    bool show_local_time_zone;
    bool pre_2009_file;
    bool dump;
    bool dump_mat;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_PVT_CONF_H
