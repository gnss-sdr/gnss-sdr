/*!
 * \file tlm_conf.cc
 * \brief Class that contains all the configuration parameters for generic
 * telemetry decoder block.
 * \author Carles Fernandez, 2020. cfernandez(at)cttc.es
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

#include "tlm_conf.h"


void Tlm_Conf::SetFromConfiguration(const ConfigurationInterface *configuration,
    const std::string &role)
{
    const std::string default_dumpname("telemetry");
    dump_filename = configuration->property(role + ".dump_filename", default_dumpname);
    dump = configuration->property(role + ".dump", false);
    dump_mat = configuration->property(role + ".dump_mat", dump);
    remove_dat = configuration->property(role + ".remove_dat", false);
    dump_crc_stats = configuration->property(role + ".dump_crc_stats", false);
    const std::string default_crc_stats_dumpname("telemetry_crc_stats");
    dump_crc_stats_filename = configuration->property(role + ".dump_crc_stats_filename", default_crc_stats_dumpname);
    enable_navdata_monitor = configuration->property("NavDataMonitor.enable_monitor", false);
    // navigation data assistance
    enable_navdata_assist = configuration->property(role + ".enable_navdata_assist", false);
    navdata_assist_real_time = configuration->property(role + ".navdata_assist_real_time", false);
    const uint32_t default_navdata_assist_TOW(0);
    navdata_assist_Tow_ms = configuration->property(role + ".navdata_assist_TOW", default_navdata_assist_TOW);
    const uint32_t default_navdata_assist_samplestamp(0);
    navdata_assist_samplestamp = configuration->property(role + ".navdata_assist_samplestamp", default_navdata_assist_samplestamp);
    const uint32_t default_navdata_assist_GNSS_UTC_leap_s = 18;
    navdata_assist_GNSS_UTC_leap_s = configuration->property(role + ".navdata_assist_GNSS_UTC_leap_s", default_navdata_assist_GNSS_UTC_leap_s);
}
