/*!
 * \file pvt_sd_conf.h
 * \brief Class that contains all the configuration parameters for spoofing detection techniques that are a part of PVT block
 * \author Harshad Sathaye sathaye.h(at)northeastern.edu
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_PVT_SD_CONF_H
#define GNSS_SDR_PVT_SD_CONF_H

#include <cstdint>
#include <map>
#include <string>

class Pvt_SD_Conf
{
public:
    Pvt_SD_Conf();

    bool position_check;
    bool dump_pos_checks_results;

    int max_jump_distance;
    int geo_fence_radius;
    int velocity_difference;

    double static_lat;
    double static_lon;
    double static_alt;
};

#endif  // GNSS_SDR_PVT_CONF_H
