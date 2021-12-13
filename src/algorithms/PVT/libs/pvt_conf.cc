/*!
 * \file pvt_conf.cc
 * \brief Class that contains all the configuration parameters for a PVT block
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

#include "pvt_conf.h"

Pvt_Conf::Pvt_Conf()
{
    type_of_receiver = 0U;
    observable_interval_ms = 20U;
    output_rate_ms = 0;
    display_rate_ms = 0;
    kml_rate_ms = 1000;
    gpx_rate_ms = 1000;
    geojson_rate_ms = 1000;
    nmea_rate_ms = 1000;

    max_obs_block_rx_clock_offset_ms = 40;
    rinex_version = 0;
    rinexobs_rate_ms = 0;
    rinex_name = std::string("-");

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

    enable_rx_clock_correction = true;
    monitor_enabled = false;
    monitor_ephemeris_enabled = false;
    protobuf_enabled = true;
    udp_port = 0;
    udp_eph_port = 0;
    pre_2009_file = false;
    show_local_time_zone = false;

    log_source_timetag = false;
    log_source_timetag_file = "PVT_timetag.dat";
}
